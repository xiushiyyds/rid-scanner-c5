/**
 * crid_scan_main.c — Remote ID Scanner 主入口
 *
 * ESP32 Remote ID Scanner
 * Standards: ASTM F3411-22a / ASD-STAN prEN 4709-002 / GB 42590-2023 / GB 46750-2025
 *
 * 架构：
 *   - crid_sniffer:   Wi-Fi 混杂模式抓包，ISR 安全回调
 *   - crid_parser:    opendroneid 库解码
 *   - crid_tracker:   无人机追踪表（线程安全）
 *   - crid_display:   信息展示（摘要/详情/状态行）
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "driver/uart.h"
#include "opendroneid.h"
#include "esp_wifi.h"
#include "esp_bt.h"

#include "crid_rx_types.h"
#include "gps_module.h"
#include "crid_ble.h"
#include "crid_ble_scan.h"
#include "crid_sniffer.h"
#include "crid_parser.h"
#include "crid_tracker.h"
#include "crid_display.h"
#include "crid_json.h"
#include "geofence.h"
#include "lcd_display.h"
#include "driver/gpio.h"
#include "sim_core.h"
#include "sim_wifi.h"
#include "dji_droneid.h"

#ifndef CRID_VERSION_STRING
#define CRID_VERSION_STRING "1.9.9-lcdfix30"
#endif
#ifndef CRID_BUILD_DATE
#define CRID_BUILD_DATE     __DATE__
#endif
#ifndef CRID_BUILD_TIME
#define CRID_BUILD_TIME     __TIME__
#endif

/* ================================================================
 * UART 数据端口配置
 *
 * 硬件连接：ESP32-S3 UART1
 *   - TX: GPIO17
 *   - RX: GPIO18 (不使用)
 *   - 波特率: 115200
 *
 * 此端口仅输出 UAV 解析数据（uav_discovery / uav_update / uav_status
 * / uav_timeout / uav_detail / status），方便上位机接收纯净数据流。
 *
 * 调试/告警/错误/启动信息仍通过 USB CDC (stdout) 输出。
 * ================================================================ */

#define UART_DATA_PORT_NUM      UART_NUM_1
#define UART_DATA_TX_PIN        17
#define UART_DATA_RX_PIN        18
#define UART_DATA_BAUD_RATE     115200
#define UART_DATA_BUF_SIZE      1024

static bool s_uart_data_initialized = false;

/* ================================================================
 * v1.3 新增：设备工作模式
 *
 * ESP32-C5 只有一个 Wi-Fi radio，不能同时扫描和发射。
 * 需要在扫描模式和模拟模式之间切换。
 * ================================================================ */
typedef enum {
    MODE_SCAN = 0,       // 正常扫描模式
    MODE_SIMULATE,       // RID 模拟发射模式
} device_mode_t;

static volatile device_mode_t s_device_mode = MODE_SCAN;
/* lcdfix19: 模拟"武装"状态。
 * 手机网页点"启用模拟"只配置参数并置 armed=true，不直接发射；
 * 用户在板子上长按Boot进入模拟设置页，确认状态栏显示"已启用"后，
 * 再短按Boot才真正切到AP发射。停止/断连时清除。 */
static volatile bool s_sim_armed = false;

/* ================================================================
 * lcdfix28: 页面驱动的射频模式互斥
 *
 * ESP32-C5 只有一个 Wi-Fi radio，BLE 扫描、WiFi sniffer、WiFi AP 发射
 * 三者不能高效共存。按用户要求，以 LCD 当前页面作为模式锚点：
 *
 *   HOME / LIST / DETAIL  → 侦测模式：BLE 扫描 + WiFi sniffer 全开，
 *                           模拟器停止。射频专心抓无人机 RID。
 *   SIM_CONFIG            → 模拟待命：停 BLE 扫描 + WiFi sniffer，
 *                           但不启动 AP（用户还在配参数）。
 *   SIM_STATUS            → 模拟发射：停侦测，sim 正在 AP TX。
 *
 * 进入 SIM 页时停侦测；离开 SIM 页（回 HOME 等侦测页）时停模拟器、
 * 恢复侦测。BLE NUS 连接本身（手机连板子控制）始终保持，不受影响。
 * ================================================================ */

/* 侦测射频是否处于活动状态（BLE scan + WiFi sniffer） */
static volatile bool s_detect_active = true;

/* 模拟器配置（提前定义，供 detect/push_status 等函数引用） */
static sim_config_t s_sim_config;

static esp_err_t detect_stop(void) {
    if (!s_detect_active) return ESP_OK;
    ESP_LOGI("MODE", "Detect STOP: halting BLE scan + WiFi sniffer");
    /* lcdfix30: 先关扫描允许开关，挡住 BLE 连接/断开事件里的延迟重启 */
    crid_ble_scan_set_allowed(false);
    crid_sniffer_stop_channel_hold();
    vTaskDelay(pdMS_TO_TICKS(100));
    crid_ble_scan_stop();
    vTaskDelay(pdMS_TO_TICKS(100));
    crid_sniffer_deinit();
    vTaskDelay(pdMS_TO_TICKS(300));
    s_detect_active = false;
    return ESP_OK;
}

/* lcdfix30: TDD BLE 回调——WiFi 阶段禁止 BLE 扫描，BLE 阶段允许 */
static void tdd_ble_pause(void) {
    crid_ble_scan_set_tdd_allowed(false);
}
static void tdd_ble_resume(void) {
    crid_ble_scan_set_tdd_allowed(true);
}

static esp_err_t detect_start(void) {
    if (s_detect_active) return ESP_OK;
    ESP_LOGI("MODE", "Detect START: WiFi sniffer + BLE TDD scan");
    /* lcdfix30: 开页面级扫描允许。TDD 级开关由 channel_hold_task 控制 */
    crid_ble_scan_set_allowed(true);
    crid_ble_scan_set_tdd_allowed(false);  /* 初始 WiFi 阶段先关 BLE */
    esp_err_t ret = crid_sniffer_init();
    if (ret != ESP_OK) {
        ESP_LOGE("MODE", "Sniffer re-init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    /* lcdfix30: 注册 TDD 回调，由 channel_hold_task 分时调度 BLE 扫描 */
    crid_sniffer_set_ble_tdd_callbacks(tdd_ble_pause, tdd_ble_resume);
    crid_sniffer_start_channel_hold();
    s_detect_active = true;
    return ESP_OK;
}

/* 主动把当前 sim 状态推给已连接的手机 */
static void push_sim_status_to_phone(void) {
    char buf[256];
    sim_state_t state = sim_get_state();
    const char *state_str;
    switch (state) {
        case SIM_STATE_IDLE:    state_str = "idle"; break;
        case SIM_STATE_RUNNING: state_str = "running"; break;
        case SIM_STATE_STOPPED: state_str = "stopped"; break;
        default:                state_str = "unknown"; break;
    }
    const char *mode_str;
    if (state == SIM_STATE_RUNNING) {
        mode_str = "simulate";
    } else if (s_sim_armed) {
        mode_str = "armed";
    } else {
        mode_str = "scan";
    }
    double lat = s_sim_config.base_lat, lon = s_sim_config.base_lon;
    float heading = 0;
    if (state == SIM_STATE_RUNNING) {
        sim_get_current_position(&lat, &lon, &heading);
    }
    snprintf(buf, sizeof(buf),
             "{\"cmd\":\"sim_status\",\"state\":\"%s\",\"armed\":%s,"
             "\"targets\":%d,\"lat\":%.6f,\"lon\":%.6f,\"heading\":%.1f,"
             "\"channel\":%d,\"mode\":\"%s\",\"tx_power\":%d,\"tx_count\":%u,"
             "\"device_mode\":\"%s\"}\n",
             state_str, s_sim_armed ? "true" : "false",
             s_sim_config.target_count, lat, lon, heading,
             s_sim_config.channel,
             sim_flight_mode_name(s_sim_config.flight_mode),
             s_sim_config.tx_power, (unsigned)sim_get_tx_count(),
             mode_str);
    crid_ble_write_cb(buf, strlen(buf), NULL);
}

/**
 * 切换到模拟发射模式（lcdfix28：侦测已由页面回调停掉，这里只负责 AP）
 * 停止侦测 → sim_init → sim_start
 * @param target_count 目标数量 (0=使用上次配置, 1~64)
 */
static esp_err_t switch_to_simulate_mode(int target_count) {
    if (s_device_mode == MODE_SIMULATE) {
        ESP_LOGW("MODE", "Already in simulate mode");
        return ESP_OK;
    }

    if (target_count > 0) {
        if (target_count > SIM_MAX_TARGETS) target_count = SIM_MAX_TARGETS;
        s_sim_config.target_count = target_count;
    }

    ESP_LOGI("MODE", "Switching to SIMULATE mode (targets=%d)...",
             s_sim_config.target_count);

    /* 确保侦测射频已停（页面进入 SIM 页时应已停，这里幂等兜底） */
    detect_stop();
    /* 给 WiFi 控制器足够时间完全释放后再 init AP */
    vTaskDelay(pdMS_TO_TICKS(400));

    /* lcdfix30: 暂停 BLE 广播，减少射频抢占，让注入帧完整发出。
     * 已有 NUS 连接不断开，GATT 通知仍可工作。 */
    crid_ble_pause_air();

    esp_err_t ret = sim_init();
    if (ret != ESP_OK) {
        ESP_LOGE("MODE", "Sim init failed: %s", esp_err_to_name(ret));
        detect_start();
        return ret;
    }

    ret = sim_start(&s_sim_config);
    if (ret != ESP_OK) {
        ESP_LOGE("MODE", "Sim start failed: %s", esp_err_to_name(ret));
        sim_wifi_deinit();
        detect_start();
        return ret;
    }

    s_device_mode = MODE_SIMULATE;
    s_sim_armed = true;
    ESP_LOGI("MODE", "Now in SIMULATE mode (%d targets)", s_sim_config.target_count);
    push_sim_status_to_phone();
    return ESP_OK;
}

/**
 * 切换到侦测模式（lcdfix28：停模拟器 + 恢复 BLE 扫描/WiFi sniffer）
 */
static esp_err_t switch_to_scan_mode(void) {
    if (s_device_mode == MODE_SCAN) {
        ESP_LOGW("MODE", "Already in scan mode");
        return ESP_OK;
    }

    ESP_LOGI("MODE", "Switching to SCAN mode...");

    sim_stop();
    vTaskDelay(pdMS_TO_TICKS(400));

    /* lcdfix30: 恢复 BLE 广播 */
    crid_ble_resume_air();

    esp_err_t ret = detect_start();
    if (ret != ESP_OK) {
        return ret;
    }

    s_device_mode = MODE_SCAN;
    s_sim_armed = false;
    ESP_LOGI("MODE", "Now in SCAN mode");
    push_sim_status_to_phone();
    return ESP_OK;
}

/**
 * 获取当前设备模式
 */
device_mode_t get_device_mode(void) {
    return s_device_mode;
}

/**
 * 更新 LCD 模拟器显示信息（多目标版）
 */
static void update_lcd_sim_info(void) {
    sim_display_info_t info = { 0 };

    if (s_device_mode == MODE_SIMULATE) {
        info.is_sim_running = (sim_get_state() == SIM_STATE_RUNNING);
        info.is_sim_armed = s_sim_armed;
        sim_get_current_position(&info.sim_lat, &info.sim_lon, &info.sim_heading);
        info.sim_tx_count = sim_get_tx_count();
        info.sim_channel = s_sim_config.channel;
        info.sim_flight_mode = (uint8_t)s_sim_config.flight_mode;
        info.sim_alt = s_sim_config.altitude_msl;
        info.sim_target_count = s_sim_config.target_count;
        info.sim_tx_power = s_sim_config.tx_power;
        /* 多目标时 UAS_ID 显示为目标数量信息 */
        if (s_sim_config.target_count > 1) {
            snprintf(info.sim_uas_id, sizeof(info.sim_uas_id), "x%d targets",
                     s_sim_config.target_count);
        } else {
            strncpy(info.sim_uas_id, s_sim_config.uas_id, sizeof(info.sim_uas_id) - 1);
        }
        strncpy(info.sim_ssid, s_sim_config.ssid, sizeof(info.sim_ssid) - 1);
    } else {
        info.is_sim_running = false;
        info.is_sim_armed = s_sim_armed;
        info.sim_lat = s_sim_config.base_lat;
        info.sim_lon = s_sim_config.base_lon;
        info.sim_channel = s_sim_config.channel;
        info.sim_flight_mode = (uint8_t)s_sim_config.flight_mode;
        info.sim_target_count = s_sim_config.target_count;
        info.sim_tx_power = s_sim_config.tx_power;
        strncpy(info.sim_uas_id, s_sim_config.uas_id, sizeof(info.sim_uas_id) - 1);
        strncpy(info.sim_ssid, s_sim_config.ssid, sizeof(info.sim_ssid) - 1);
    }

    lcd_display_set_sim_info(&info);
}

static void uart_data_port_init(void) {
#if CONFIG_IDF_TARGET_ESP32C5
    return;
#endif
    uart_config_t uart_config = {
        .baud_rate = UART_DATA_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_DATA_PORT_NUM, &uart_config);
    uart_set_pin(UART_DATA_PORT_NUM, UART_DATA_TX_PIN, UART_DATA_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_DATA_PORT_NUM, UART_DATA_BUF_SIZE, 0, 0, NULL, 0);
    s_uart_data_initialized = true;
}

/**
 * UART 数据写入回调：将 JSON 数据同时写到 UART1
 */
static void uart_data_write_cb(const char *data, size_t len, void *ctx) {
    (void)ctx;
    if (!s_uart_data_initialized) return;
    uart_write_bytes(UART_DATA_PORT_NUM, data, len);
}

/**
 * 数据流扇出回调：写入 UART1 + BLE
 */
static void data_write_fanout(const char *data, size_t len, void *ctx) {
    uart_data_write_cb(data, len, ctx);
    crid_ble_write_cb(data, len, ctx);
}

/* ================================================================
 * LCD 按键配置 — T-Display-C5 适配
 *
 * T-Display-C5 自带两个按键：
 *   - User Button (GPIO0): lcd_display 模块内部轮询，切换页面
 *   - Boot Button (GPIO28): lcd_display 模块内部轮询，选择/确认
 *
 * 按键轮询任务已内置在 lcd_display.c 的 button_poll_task 中，
 * 无需在此处初始化 GPIO 中断。
 * ================================================================ */

/* ================================================================
 * 解析任务 (从队列取数据，使用 opendroneid 库解析)
 * ================================================================ */

static void parser_task(void *pvParameter) {
    json_debug("RID_MAIN", "Parser task started");

    QueueHandle_t queue = crid_sniffer_get_queue();
    SemaphoreHandle_t mutex = crid_tracker_get_mutex();
    sniffer_msg_t msg;
    uint32_t last_cleanup_ms = 0;
    uint32_t last_idle_output_ms = 0;

    while (1) {
        if (xQueueReceive(queue, &msg, pdMS_TO_TICKS(1000)) != pdTRUE) {
            uint32_t now = esp_log_timestamp();
            if (crid_tracker_get_active_count() == 0 &&
                (now - last_idle_output_ms >= 1000)) {
                json_no_aircraft();
                last_idle_output_ms = now;
            }
            continue;
        }

        // 普通 Beacon（无 Vendor IE）：直接跳过
        if (msg.msg_type == MSG_TYPE_BEACON_NO_VENDOR) {
            continue;
        }

        // 非 RID Vendor IE：直接跳过
        if (msg.msg_type == MSG_TYPE_NON_RID_VENDOR) {
            continue;
        }

        // DJI DroneID 私有协议处理
        if (msg.msg_type == MSG_TYPE_DJI_DRONEID) {
            if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) != pdTRUE) continue;

            uav_track_t *uav = crid_tracker_find_or_create(msg.src_mac);
            if (uav == NULL) {
                uint32_t now = esp_log_timestamp();
                if (now - last_cleanup_ms >= 10000) {
                    crid_tracker_cleanup(UAV_TIMEOUT_MS);
                    last_cleanup_ms = now;
                    uav = crid_tracker_find_or_create(msg.src_mac);
                }
                if (uav == NULL) {
                    xSemaphoreGive(mutex);
                    continue;
                }
            }

            bool was_new = (uav->msg_count == 0);
            uav->msg_count++;
            uav->last_seen_ms = esp_log_timestamp();
            uav->last_rssi = msg.rssi;
            uav->last_channel = msg.channel;
            uav->is_dji = true;
            uav->protocol = RID_PROTOCOL_UNKNOWN;  // 私有协议，非标准

            // 解析 DJI DroneID
            dji_droneid_data_t dji_data;
            if (dji_droneid_parse(msg.data, msg.data_len, &dji_data) == 0) {
                uav->dji_type = dji_data.type;
                strncpy(uav->dji_serial, dji_data.serial, sizeof(uav->dji_serial) - 1);
                uav->dji_serial[sizeof(uav->dji_serial) - 1] = '\0';
                strncpy(uav->dji_model, dji_data.model_name, sizeof(uav->dji_model) - 1);
                uav->dji_model[sizeof(uav->dji_model) - 1] = '\0';
                uav->dji_model_code = (uint16_t)dji_data.product_type;
                uav->dji_latitude = dji_data.latitude;
                uav->dji_longitude = dji_data.longitude;
                uav->dji_altitude = dji_data.altitude;
                uav->dji_height = dji_data.height;
                uav->dji_speed_h = dji_data.speed_h;
                uav->dji_speed_v = dji_data.speed_up;
                uav->dji_heading = dji_data.heading;
                uav->dji_battery = 0;  // DroneID v2 telemetry 不直接含电量
                /* pilot GPS 仅在完整格式帧中存在；短格式回退到 home 坐标 */
                if (dji_data.has_pilot_gps) {
                    uav->dji_pilot_lat = dji_data.pilot_latitude;
                    uav->dji_pilot_lon = dji_data.pilot_longitude;
                } else {
                    uav->dji_pilot_lat = dji_data.home_latitude;
                    uav->dji_pilot_lon = dji_data.home_longitude;
                }
                strncpy(uav->dji_identification, dji_data.drone_id,
                        sizeof(uav->dji_identification) - 1);
                uav->dji_identification[sizeof(uav->dji_identification) - 1] = '\0';

                // 同步到标准 location 结构（用于地理围栏和显示）
                if (dji_data.type == 0x10) {  // Telemetry
                    uav->location.valid = true;
                    uav->location.latitude = dji_data.latitude;
                    uav->location.longitude = dji_data.longitude;
                    uav->location.altitude_baro = dji_data.altitude;
                    uav->location.height = dji_data.height;
                    uav->location.speed_horizontal = dji_data.speed_h;
                    uav->location.speed_vertical = dji_data.speed_up;
                    uav->location.direction = dji_data.heading;

                    // 设置 basic_id
                    uav->basic_id.valid = true;
                    uav->basic_id.id_type = 1;  /* Serial Number */
                    uav->basic_id.ua_type = 2;  /* Multirotor */
                    strncpy(uav->basic_id.uas_id, dji_data.serial,
                            sizeof(uav->basic_id.uas_id) - 1);
                    uav->basic_id.uas_id[sizeof(uav->basic_id.uas_id) - 1] = '\0';
                }

                // 地理围栏检测
                geofence_alert_t alert;
                if (geofence_check(uav->location.latitude, uav->location.longitude,
                                   uav->location.altitude_baro, &alert)) {
                    uav->alert_level = (uint8_t)alert.level;
                    strncpy(uav->alert_zone, alert.zone_name, sizeof(uav->alert_zone) - 1);
                } else {
                    uav->alert_level = 0;
                    uav->alert_zone[0] = '\0';
                }
            }

            xSemaphoreGive(mutex);

            // 输出 DJI 无人机数据
            if (was_new) {
                json_uav_discovery(uav);
            }
            json_uav_update(uav);
            continue;
        }

        // 以下仅处理 MSG_TYPE_RID

        if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
            continue;
        }

        uav_track_t *uav = crid_tracker_find_or_create(msg.src_mac);
        if (uav == NULL) {
            // 追踪表已满，尝试清理超时条目腾出空间
            uint32_t now = esp_log_timestamp();
            if (now - last_cleanup_ms >= 10000) {  // 每 10 秒最多清理一次
                crid_tracker_cleanup(UAV_TIMEOUT_MS);
                last_cleanup_ms = now;
                // 重试一次
                uav = crid_tracker_find_or_create(msg.src_mac);
            }
            if (uav == NULL) {
                json_warning("RID_MAIN", "Tracker full! Cannot track new UAV");
                xSemaphoreGive(mutex);
                continue;
            }
        }

        bool was_new = (uav->msg_count == 0);
        uav->msg_count++;
        uav->last_seen_ms = esp_log_timestamp();
        uav->last_rssi = msg.rssi;
        uav->last_channel = msg.channel;

        // 记录 OUI 和传输/协议类型
        uav->oui[0] = msg.oui[0];
        uav->oui[1] = msg.oui[1];
        uav->oui[2] = msg.oui[2];
        uav->oui_type = msg.oui_type;
        uav->transport = (uint8_t)GET_RID_TRANSPORT(msg.oui[0], msg.oui[1], msg.oui[2]);

        // 解码（解析器内部自动识别协议并返回）
        rid_protocol_t detected_proto = crid_parser_decode(uav, msg.data, msg.data_len);

        // 仅解码成功时更新协议类型，失败则保持上次已知的协议
        if (detected_proto != RID_PROTOCOL_UNKNOWN) {
            uav->protocol = (uint8_t)detected_proto;
        }

        // 提取分层数据（供显示层使用）
        crid_parser_extract_layered(uav);

        // v1.8: 跨传输方式去重（WiFi/BLE）
        // 如果这是一个新 MAC 但 UAS ID 已在另一个条目中存在，
        // 说明同一架无人机从不同射频接口被收到，合并到已有条目。
        if (was_new && uav->basic_id.valid && uav->basic_id.uas_id[0] != '\0') {
            uav_track_t *existing = crid_tracker_find_by_uas_id(uav->basic_id.uas_id);
            if (existing != NULL && existing != uav) {
                // 合并：把新数据拷贝到已有条目，保留旧条目的 first_seen 和统计
                existing->msg_count += uav->msg_count;
                existing->last_seen_ms = uav->last_seen_ms;
                existing->last_rssi = uav->last_rssi;
                existing->last_channel = uav->last_channel;
                // 合并 OUI/transport 信息（标记多通道接收）
                existing->oui[0] = uav->oui[0];
                existing->oui[1] = uav->oui[1];
                existing->oui[2] = uav->oui[2];
                existing->transport = uav->transport;
                existing->protocol = uav->protocol;
                // 合并完整数据
                existing->uas_data = uav->uas_data;
                existing->basic_id = uav->basic_id;
                existing->location = uav->location;
                existing->system = uav->system;
                existing->self_id = uav->self_id;
                existing->operator_id = uav->operator_id;
                existing->gb46750 = uav->gb46750;
                existing->last_pack = uav->last_pack;
                /* [Bug DD 修复] 跨信道合并时保留 DJI 标记：
                 * 如果已有条目是 DJI 无人机（先从 WiFi DroneID 收到），
                 * 不要被后续 BLE 标准 RID 条目的 is_dji=false 覆盖；
                 * 同时 DJI 私有字段只在新条目确实携带 DJI 数据时才更新。 */
                if (uav->is_dji) {
                    existing->is_dji   = true;
                    existing->dji_type = uav->dji_type;
                }
                if (uav->dji_model_code) {
                    existing->dji_model_code = uav->dji_model_code;
                }
                if (uav->dji_serial[0]) {
                    strncpy(existing->dji_serial, uav->dji_serial,
                            sizeof(existing->dji_serial) - 1);
                    existing->dji_serial[sizeof(existing->dji_serial) - 1] = '\0';
                }
                if (uav->dji_model[0]) {
                    strncpy(existing->dji_model, uav->dji_model,
                            sizeof(existing->dji_model) - 1);
                    existing->dji_model[sizeof(existing->dji_model) - 1] = '\0';
                }
                // 将当前 MAC 条目标记为非活跃（释放槽位）
                uav->active = false;
                uav->msg_count = 0;
                uav = existing;
                was_new = false;  // 不是新目标，只是更新
            }
        }

        // 地理围栏告警检测
        if (uav->location.valid) {
            geofence_alert_t alert;
            // 选择高度：优先使用相对高度(height)，否则用气压高度(altitude_baro)
            float check_alt = uav->location.height > 0 ? uav->location.height : uav->location.altitude_baro;
            if (geofence_check(uav->location.latitude, uav->location.longitude, check_alt, &alert)) {
                uav->alert_level = (uint8_t)alert.level;
                strncpy(uav->alert_zone, alert.zone_name, sizeof(uav->alert_zone) - 1);
                uav->alert_zone[sizeof(uav->alert_zone) - 1] = '\0';
            } else {
                uav->alert_level = 0;
                uav->alert_zone[0] = '\0';
            }
        }

        xSemaphoreGive(mutex);

        // 新发现 UAV 时输出发现事件 + 完整更新
        if (was_new && uav->basic_id.valid) {
            json_uav_discovery(uav);
        }
        // 每次解码后输出完整解析数据
        if (uav->basic_id.valid) {
            json_uav_update(uav);
        }
    }
}


static void monitor_task(void *pvParameter) {
    json_debug("RID_MAIN", "Monitor task started");

    uint32_t loop_count = 0;
    uint32_t last_packets = 0;
    uint32_t last_mgmt = 0;
    uint32_t last_rid = 0;
    uint32_t last_beacons = 0;
    uint32_t last_non_rid = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(60000));
        loop_count++;

        sniffer_stats_t *stats = crid_sniffer_get_stats();
        SemaphoreHandle_t mutex = crid_tracker_get_mutex();

        uint32_t total_pkts = stats->total_packets;
        uint32_t mgmt_pkts = stats->mgmt_frames;
        uint32_t rid_pkts = stats->rid_detections;
        uint32_t overflows = stats->queue_overflows;
        uint32_t beacons = stats->beacon_count;
        uint32_t non_rid = stats->non_rid_vendor_ie;

        float pkt_rate = (total_pkts - last_packets) / 60.0f;
        float mgmt_rate = (mgmt_pkts - last_mgmt) / 60.0f;
        float beacon_rate = (beacons - last_beacons) / 60.0f;
        float rid_rate = (rid_pkts - last_rid) / 60.0f;
        float non_rid_rate = (non_rid - last_non_rid) / 60.0f;

        last_packets = total_pkts;
        last_mgmt = mgmt_pkts;
        last_rid = rid_pkts;
        last_beacons = beacons;
        last_non_rid = non_rid;

        // 打印活跃无人机列表并清理超时
        int active = 0;
        if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            active = crid_tracker_get_active_count();

            uav_track_t *table = crid_tracker_get_table();
            for (int i = 0; i < MAX_TRACKED_UAVS; i++) {
                if (!table[i].active) continue;
                json_uav_status(&table[i]);
            }

            // 清理超时条目
            crid_tracker_cleanup(UAV_TIMEOUT_MS);

            xSemaphoreGive(mutex);
        }

        // 输出汇总状态 JSON
        json_status_report(loop_count,
                          (uint32_t)esp_get_free_heap_size(),
                          total_pkts, pkt_rate,
                          mgmt_pkts, mgmt_rate,
                          beacons, beacon_rate,
                          rid_pkts, rid_rate,
                          non_rid, non_rid_rate,
                          overflows, active);
    }
}

/* ================================================================
 * v1.4: 模拟器 BLE 回调实现（多目标版）
 *
 * lcdfix15 关键修复：
 * 这些回调在 NimBLE host task 上下文中被调用。
 * WiFi init/start/deinit 可能阻塞数百毫秒，直接在 host task 里
 * 同步执行会导致 NimBLE 协议栈看门狗超时或 GATT 通知失败，
 * 表现为网页显示未连接。
 * 改为投递到独立的 mode_switch_task，host task 立即返回。
 * ================================================================ */

typedef enum {
    MODE_OP_START = 1,   /* 启动模拟发射（sim_init + sim_start） */
    MODE_OP_STOP  = 2,   /* 停模拟器，恢复侦测 */
    MODE_OP_ENTER_SIM = 3,  /* 进入 SIM 页：只停侦测，不启动 AP（配参数阶段） */
    MODE_OP_EXIT_SIM  = 4,  /* 离开 SIM 页回到侦测页：若在发射则停掉，恢复侦测 */
} mode_op_t;

typedef struct {
    mode_op_t op;
    int target_count;
} mode_msg_t;

static QueueHandle_t s_mode_queue = NULL;

/* 串行化所有射频重配操作，避免 WiFi init/deinit 与 sim_start 并发 */
static SemaphoreHandle_t s_rf_mutex = NULL;

static void mode_switch_task(void *arg) {
    mode_msg_t msg;
    while (1) {
        if (xQueueReceive(s_mode_queue, &msg, portMAX_DELAY) == pdTRUE) {
            if (xSemaphoreTake(s_rf_mutex, portMAX_DELAY) == pdTRUE) {
                switch (msg.op) {
                case MODE_OP_START:
                    switch_to_simulate_mode(msg.target_count);
                    break;
                case MODE_OP_STOP:
                case MODE_OP_EXIT_SIM:
                    /* 若正在发射，先停模拟器；侦测由 detect_start 恢复 */
                    if (s_device_mode == MODE_SIMULATE) {
                        switch_to_scan_mode();
                    } else {
                        /* 只是在配置页待过、没发射：恢复侦测 + BLE 广播即可 */
                        /* lcdfix30: 恢复 BLE 广播 */
                        crid_ble_resume_air();
                        detect_start();
                        s_sim_armed = false;
                        push_sim_status_to_phone();
                    }
                    break;
                case MODE_OP_ENTER_SIM:
                    /* 进入模拟配置/状态页：停掉侦测，把射频让给 AP 发射。
                     * 不在此处启动 AP，等用户在配置页按 Boot 或手机发 SIM_START。 */
                    detect_stop();
                    break;
                }
                xSemaphoreGive(s_rf_mutex);
            }
        }
    }
}

static void post_mode_op(mode_op_t op, int count) {
    if (!s_mode_queue) return;
    mode_msg_t msg = { .op = op, .target_count = count };
    xQueueSend(s_mode_queue, &msg, 0);
}

/* lcdfix28: 手机"启用模拟"直接启动发射（不再需要板子二次按 Boot）。
 * 启动后把 LCD 切到 SIM_STATUS 页，让用户看到 TX 计数。 */
static void sim_ble_start_handler(int target_count) {
    if (target_count > 0) {
        if (target_count > SIM_MAX_TARGETS) target_count = SIM_MAX_TARGETS;
        s_sim_config.target_count = target_count;
    }
    if (s_device_mode == MODE_SIMULATE && sim_get_state() == SIM_STATE_RUNNING) {
        ESP_LOGI("MODE", "SIM already running, apply config only");
        return;
    }
    /* 确保射频已切到模拟侧。LCD 切页会触发 ENTER_SIM（detect_stop）；
     * 若已在 SIM 页但侦测还在活动（异常路径），手动补投 ENTER_SIM。
     * 队列 FIFO 保证 ENTER_SIM 先于 START 执行。 */
    lcd_page_t cur = lcd_display_get_page();
    if (cur != LCD_PAGE_SIM_CONFIG && cur != LCD_PAGE_SIM_STATUS) {
        lcd_display_set_page(LCD_PAGE_SIM_STATUS);
    } else if (s_detect_active) {
        post_mode_op(MODE_OP_ENTER_SIM, 0);
    }
    post_mode_op(MODE_OP_START, target_count);
}
static void sim_ble_stop_handler(void) {
    s_sim_armed = false;
    if (s_device_mode == MODE_SIMULATE) {
        post_mode_op(MODE_OP_STOP, 0);
    } else if (!s_detect_active) {
        /* 在配置页没发射就点了停止：恢复侦测 */
        post_mode_op(MODE_OP_EXIT_SIM, 0);
    }
    /* 回到主页 */
    lcd_display_set_page(LCD_PAGE_HOME);
}

/**
 * BLE SIM_CONFIG 回调：更新模拟器配置
 */
static void sim_ble_config_handler(double lat, double lon, int mode, int channel,
                                    int count, int tx_power) {
    if (xSemaphoreTake(s_rf_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        s_sim_config.base_lat = lat;
        s_sim_config.base_lon = lon;
        s_sim_config.flight_mode = (sim_flight_mode_t)mode;
        s_sim_config.channel = (uint8_t)channel;

        if (count > 0) {
            if (count > SIM_MAX_TARGETS) count = SIM_MAX_TARGETS;
            s_sim_config.target_count = count;
        }
        if (tx_power > 0) {
            s_sim_config.tx_power = (int8_t)tx_power;
        }

        if (s_device_mode == MODE_SIMULATE && sim_get_state() == SIM_STATE_RUNNING) {
            sim_update_config(&s_sim_config);
        }
        xSemaphoreGive(s_rf_mutex);
    }

    ESP_LOGI("MODE", "SIM config updated: lat=%.4f lon=%.4f mode=%d ch=%d "
             "targets=%d tx_power=%d",
             lat, lon, mode, channel, s_sim_config.target_count, s_sim_config.tx_power);
}

/**
 * BLE SIM_STATUS 回调：返回模拟器状态 JSON
 */
static void sim_ble_status_handler(char *buf, size_t buf_size) {
    sim_state_t state = sim_get_state();
    const char *state_str;
    switch (state) {
        case SIM_STATE_IDLE:    state_str = "idle"; break;
        case SIM_STATE_RUNNING: state_str = "running"; break;
        case SIM_STATE_STOPPED: state_str = "stopped"; break;
        default:                state_str = "unknown"; break;
    }

    double lat = 0, lon = 0;
    float heading = 0;
    if (state == SIM_STATE_RUNNING) {
        sim_get_current_position(&lat, &lon, &heading);
    } else {
        lat = s_sim_config.base_lat;
        lon = s_sim_config.base_lon;
    }

    /* lcdfix28: device_mode 反映当前射频状态。
     * running=正在发射；armed=在模拟配置页（侦测已停，待发射）；scan=侦测中。 */
    const char *mode_str;
    if (state == SIM_STATE_RUNNING) {
        mode_str = "simulate";
    } else if (!s_detect_active) {
        mode_str = "armed";
    } else {
        mode_str = "scan";
    }

    snprintf(buf, buf_size,
             "{\"cmd\":\"sim_status\","
             "\"state\":\"%s\","
             "\"armed\":%s,"
             "\"targets\":%d,"
             "\"lat\":%.6f,"
             "\"lon\":%.6f,"
             "\"heading\":%.1f,"
             "\"channel\":%d,"
             "\"mode\":\"%s\","
             "\"tx_power\":%d,"
             "\"tx_count\":%u,"
             "\"device_mode\":\"%s\"}\n",
             state_str,
             s_sim_armed ? "true" : "false",
             s_sim_config.target_count,
             lat, lon, heading,
             s_sim_config.channel,
             sim_flight_mode_name(s_sim_config.flight_mode),
             s_sim_config.tx_power,
             (unsigned)sim_get_tx_count(),
             mode_str);
}

/* ================================================================
 * 主函数
 * ================================================================ */

/* ================================================================
 * LCD 模拟器信息定时更新任务
 * ================================================================ */
static void lcd_sim_update_task(void *arg) {
    while (1) {
        update_lcd_sim_info();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* ================================================================
 * GPS 自身位置上报任务
 * lcdfix17: 无论有没有 GPS 硬件，每3秒都通过BLE发送 SELF_GPS 状态。
 * 有定位：SELF_GPS:lat,lon,alt,sats
 * 无定位/无硬件：SELF_GPS:0,0,0,0  （网页据此显示等待定位）
 * 之前只在 valid 时发送，没接 GPS 模块时网页完全收不到状态，
 * 用户无法区分是没连上还是没定位。
 * ================================================================ */
static void gps_report_task(void *arg) {
    char buf[96];
    while (1) {
        gps_data_t gd = gps_get_data();
        if (gd.valid && gd.fix_quality > 0) {
            snprintf(buf, sizeof(buf), "SELF_GPS:%.6f,%.6f,%.1f,%d\n",
                     gd.latitude, gd.longitude, gd.altitude, gd.satellites);
        } else {
            /* 无定位或无硬件：发零值，让网页知道板子在线但GPS未定位 */
            snprintf(buf, sizeof(buf), "SELF_GPS:0,0,0,0\n");
        }
        crid_ble_write_cb(buf, strlen(buf), NULL);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

/* LCD 按键回调：在 SIM_CONFIG 页按 Boot 启动/停止发射。
 * lcdfix28: 去掉"必须先手机武装"限制——在配置页直接 Boot 即可发射，
 * 手机和板子都能独立控制。WiFi 重配投递到 mode_switch_task 异步执行。 */
static void lcd_sim_start_handler(void) {
    if (s_device_mode == MODE_SIMULATE && sim_get_state() == SIM_STATE_RUNNING) {
        post_mode_op(MODE_OP_STOP, 0);
    } else {
        post_mode_op(MODE_OP_START, 0);
    }
}
static void lcd_sim_stop_handler(void) {
    sim_ble_stop_handler();
}
static void lcd_sim_mode_handler(int mode) {
    /* 用当前配置更新 mode */
    sim_ble_config_handler(s_sim_config.base_lat, s_sim_config.base_lon,
                           mode, (int)s_sim_config.channel, 0, 0);
}

/* lcdfix28: 页面切换回调（在 LCD 按键任务上下文，不能阻塞）。
 * 进 SIM 页 → 投递 ENTER_SIM 停侦测；离开 SIM 页 → 投递 EXIT_SIM 恢复。 */
static void lcd_page_change_handler(lcd_page_t new_page) {
    ESP_LOGI("MODE", "LCD page changed -> %d", (int)new_page);
    if (new_page == LCD_PAGE_SIM_CONFIG || new_page == LCD_PAGE_SIM_STATUS) {
        post_mode_op(MODE_OP_ENTER_SIM, 0);
    } else {
        /* HOME / LIST / DETAIL：离开模拟，恢复侦测 */
        post_mode_op(MODE_OP_EXIT_SIM, 0);
    }
}

/* lcdfix16: 提供给 LCD 状态栏的 GPS 回调。
 * 由 lcd_display 组件在渲染时调用，避免 lcd_display 反向依赖 gps_module。
 * 返回 true=有有效定位，同时把经纬度/海拔/卫星数写回出参。 */
static bool lcd_gps_provider(double *lat, double *lon, double *alt, int *sats_out) {
    gps_data_t gd = gps_get_data();
    bool fix = gd.valid && gd.fix_quality > 0;
    if (lat) *lat = gd.latitude;
    if (lon) *lon = gd.longitude;
    if (alt) *alt = gd.altitude;
    if (sats_out) *sats_out = gd.satellites;
    return fix;
}

void app_main(void) {
    // 0. 初始化 UART 数据端口（GPIO17 TX），用于输出 UAV 解析数据
    uart_data_port_init();

    // 设置数据流回调：UAV 数据同时输出到 stdout（USB CDC）和 UART1
    json_set_data_write_cb(uart_data_write_cb, NULL);

    // 初始化模拟器配置（默认值）和射频重配互斥锁
    sim_get_default_config(&s_sim_config);
    s_rf_mutex = xSemaphoreCreateMutex();
    s_mode_queue = xQueueCreate(6, sizeof(mode_msg_t));
    /* lcdfix16: 栈从 4096 扩到 8192。
     * switch_to_simulate_mode 会同步调用 esp_wifi_init/deinit + NimBLE stop/start，
     * 调用栈深，4096 会栈溢出撞 InterruptWatchdog，表现为按第二次才重启。 */
    xTaskCreatePinnedToCore(mode_switch_task, "mode_sw", 8192, NULL, 5, NULL, 0);
    ESP_LOGI("RID_MAIN", "SIM config: lat=%.4f lon=%.4f ch=%u mode=%s targets=%d tx_power=%d",
             s_sim_config.base_lat, s_sim_config.base_lon,
             s_sim_config.channel, sim_flight_mode_name(s_sim_config.flight_mode),
             s_sim_config.target_count, s_sim_config.tx_power);

    // JSON 启动信息（调试流 → 仅 USB CDC）
    json_startup_info(CRID_VERSION_STRING, CRID_BUILD_DATE, CRID_BUILD_TIME,
                      esp_get_idf_version(),
                      (uint32_t)esp_get_free_heap_size(),
                      ODID_PROTOCOL_VERSION);

    // 1. 初始化追踪器
    crid_tracker_init();

    // 1.5 初始化地理围栏模块
    geofence_init();
    dji_droneid_init();

    // 1.6 初始化 GPS 模块 (ATGM336H, UART1, 9600 baud)
    gps_init();
    ESP_LOGI("RID_MAIN", "GPS module initialized");

    // 2. 初始化 NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        json_warning("RID_MAIN", "Erasing NVS flash...");
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        char err[64];
        snprintf(err, sizeof(err), "NVS init failed: %s", esp_err_to_name(ret));
        json_error("RID_MAIN", err);
        return;
    }

    // 3. 初始化网络接口和事件循环
    esp_netif_init();
    esp_event_loop_create_default();

    // 4. lcdfix26: BLE 必须在 LCD 之前初始化。
    //    lcdfix25 把 BLE 移到 WiFi 之前，controller init 通过了，
    //    但 ext_adv_configure 后 controller 在 r_ble_ll_mem_msys_update
    //    分配链路层缓冲区时 assert 崩溃——因为 LCD 初始化后内部 SRAM 被
    //    切碎（总剩余 89KB，最大连续块仅 40KB）。
    //    BLE controller 需要大块连续内部 SRAM 用于 ext_adv 链路层缓冲，
    //    必须在 LCD 的 13.6KB DMA bounce buffer 分配之前、内存还是整块时拿到。
    //    启动时屏幕会黑几秒（等 BLE controller + PHY init ~3.5s），可接受。

    // 4a. 注册所有 BLE 回调（纯函数指针赋值，不依赖 BLE 或 LCD 已初始化）
    crid_ble_register_pair_display(lcd_display_show_pair_pin);
    crid_ble_register_sim_callbacks(sim_ble_start_handler,
                                    sim_ble_stop_handler,
                                    sim_ble_config_handler,
                                    sim_ble_status_handler);

    // 4b. 释放不用的经典蓝牙控制器内存（~70KB BSS/data），给 BLE 更多空间
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);

    ESP_LOGI("RID_MAIN", "Before BLE init - free heap: %u, internal free: %u, largest block: %u",
             (unsigned)esp_get_free_heap_size(),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));

    // 4c. 初始化 BLE（NimBLE controller + host）
    ret = crid_ble_init();
    if (ret != ESP_OK) {
        ESP_LOGE("RID_MAIN", "BLE init failed: %s (free heap=%u, internal=%u, largest=%u)",
                 esp_err_to_name(ret),
                 (unsigned)esp_get_free_heap_size(),
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
        json_warning("RID_MAIN", "BLE init failed (non-fatal)");
    } else {
        // BLE 就绪后切换为扇出回调：UART + BLE 双路输出
        json_set_data_write_cb(data_write_fanout, NULL);
        json_debug("RID_MAIN", "BLE data channel + SIM callbacks registered");

        // 初始化 BLE RID 扫描模块
        crid_ble_scan_init();
        json_debug("RID_MAIN", "BLE RID scanner initialized (will start on host sync)");
    }

    ESP_LOGI("RID_MAIN", "After BLE init - free heap: %u, internal free: %u, largest block: %u",
             (unsigned)esp_get_free_heap_size(),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));

    // 5. 初始化 LCD 显示模块（在 BLE 之后）
    //    BLE 已经拿到连续 SRAM，LCD 的 DMA bounce buffer 从剩余空间分配。
    //    BLE 回调指针已提前注册，LCD 按键事件可正常扇出。
    if (lcd_display_init() == 0) {
        lcd_display_set_source(crid_tracker_get_table(),
                               crid_tracker_get_mutex(),
                               MAX_TRACKED_UAVS);
        /* 注册 LCD 按键模拟器操作回调 */
        lcd_display_register_sim_callbacks(
            lcd_sim_start_handler,
            lcd_sim_stop_handler,
            lcd_sim_mode_handler
        );
        /* lcdfix28: 注册页面切换回调，实现射频模式互斥 */
        lcd_display_register_page_change_cb(lcd_page_change_handler);
        /* lcdfix16: 注册 GPS 状态提供回调（状态栏 GPS 图标用） */
        lcd_display_register_gps_provider(lcd_gps_provider);
        /* lcdfix19: 注册当前扫描信道回调（1/6/11 跳频） */
        lcd_display_register_channel_provider(crid_sniffer_get_current_channel);
        json_debug("RID_MAIN", "LCD display ready (ST7789 170x320)");
    } else {
        json_warning("RID_MAIN", "LCD init failed (non-fatal, serial only)");
    }

    // 6. 初始化 Wi-Fi sniffer（在 BLE 之后）
    ret = crid_sniffer_init();
    if (ret != ESP_OK) {
        json_error("RID_MAIN", "Sniffer init failed!");
        return;
    }

    // lcdfix30: 注册 TDD BLE 回调，由 channel_hold_task 分时调度
    crid_sniffer_set_ble_tdd_callbacks(tdd_ble_pause, tdd_ble_resume);

    // 7. 创建任务
    BaseType_t task_created;

    task_created = xTaskCreate(parser_task, "parser",
                               PARSER_TASK_STACK, NULL, PARSER_TASK_PRIO, NULL);
    if (task_created != pdPASS) {
        json_error("RID_MAIN", "Failed to create parser task!");
        return;
    }

    task_created = xTaskCreate(monitor_task, "monitor",
                               MONITOR_TASK_STACK, NULL, MONITOR_TASK_PRIO, NULL);
    if (task_created != pdPASS) {
        json_error("RID_MAIN", "Failed to create monitor task!");
        return;
    }

    crid_sniffer_start_channel_hold();

    // 8. 启动完成（调试流 → USB CDC）
    json_startup_banner(CRID_VERSION_STRING, CRID_BUILD_DATE, CRID_BUILD_TIME,
                        FIXED_CHANNEL, MAX_TRACKED_UAVS,
                        (uint32_t)esp_get_free_heap_size());

    // 创建 LCD 模拟器信息更新任务
    xTaskCreatePinnedToCore(lcd_sim_update_task, "lcd_sim_upd", 2048, NULL, 2, NULL, 0);
    ESP_LOGI("RID_MAIN", "SIM LCD update task started");

    // GPS 自身位置上报任务 (每3秒发送 SELF_GPS)
    xTaskCreatePinnedToCore(gps_report_task, "gps_rpt", 3072, NULL, 3, NULL, 0);
    ESP_LOGI("RID_MAIN", "GPS report task started (3s interval)");

    // 初始化 LCD 模拟器信息
    update_lcd_sim_info();
}
