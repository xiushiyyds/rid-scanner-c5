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
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "driver/uart.h"
#include "opendroneid.h"
#include "esp_wifi.h"

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
#include "dji_droneid.h"

#ifndef CRID_VERSION_STRING
#define CRID_VERSION_STRING "1.9.8"
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

/* 模式切换互斥锁 */
static SemaphoreHandle_t s_mode_mutex = NULL;

/* 模拟器配置 */
static sim_config_t s_sim_config;

/**
 * 切换到模拟模式
 * 停止 sniffer → 停止 BLE 数据回调 → 初始化并启动模拟器
 * @param target_count 目标数量 (0=使用上次配置, 1~64)
 */
static esp_err_t switch_to_simulate_mode(int target_count) {
    if (s_device_mode == MODE_SIMULATE) {
        ESP_LOGW("MODE", "Already in simulate mode");
        return ESP_OK;
    }

    /* 如果指定了目标数量，更新配置 */
    if (target_count > 0) {
        if (target_count > SIM_MAX_TARGETS) target_count = SIM_MAX_TARGETS;
        s_sim_config.target_count = target_count;
    }

    ESP_LOGI("MODE", "Switching to SIMULATE mode (targets=%d)...",
             s_sim_config.target_count);

    /* 停止 sniffer（释放 Wi-Fi radio） */
    crid_sniffer_deinit();
    vTaskDelay(pdMS_TO_TICKS(200));

    /* 初始化模拟器（如果还没初始化） */
    esp_err_t ret = sim_init();
    if (ret != ESP_OK) {
        ESP_LOGE("MODE", "Sim init failed: %s", esp_err_to_name(ret));
        /* 尝试恢复扫描模式 */
        crid_sniffer_init();
        return ret;
    }

    /* 启动模拟发射 */
    ret = sim_start(&s_sim_config);
    if (ret != ESP_OK) {
        ESP_LOGE("MODE", "Sim start failed: %s", esp_err_to_name(ret));
        crid_sniffer_init();
        return ret;
    }

    s_device_mode = MODE_SIMULATE;
    ESP_LOGI("MODE", "Now in SIMULATE mode (%d targets)", s_sim_config.target_count);
    return ESP_OK;
}

/**
 * 切换到扫描模式
 * 停止模拟器 → 重新初始化 sniffer → 恢复 BLE 数据回调
 */
static esp_err_t switch_to_scan_mode(void) {
    if (s_device_mode == MODE_SCAN) {
        ESP_LOGW("MODE", "Already in scan mode");
        return ESP_OK;
    }

    ESP_LOGI("MODE", "Switching to SCAN mode...");

    /* 停止模拟器 */
    sim_stop();
    vTaskDelay(pdMS_TO_TICKS(200));

    /* 重新初始化 sniffer */
    esp_err_t ret = crid_sniffer_init();
    if (ret != ESP_OK) {
        ESP_LOGE("MODE", "Sniffer re-init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    crid_sniffer_start_channel_hold();

    s_device_mode = MODE_SCAN;
    ESP_LOGI("MODE", "Now in SCAN mode");
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
 * ================================================================ */

/**
 * BLE SIM_START 回调：切换到模拟模式
 * @param target_count 目标数量 (0=使用上次配置)
 */
static void sim_ble_start_handler(int target_count) {
    if (s_mode_mutex) xSemaphoreTake(s_mode_mutex, portMAX_DELAY);
    switch_to_simulate_mode(target_count);
    if (s_mode_mutex) xSemaphoreGive(s_mode_mutex);
}

/**
 * BLE SIM_STOP 回调：切换到扫描模式
 */
static void sim_ble_stop_handler(void) {
    if (s_mode_mutex) xSemaphoreTake(s_mode_mutex, portMAX_DELAY);
    switch_to_scan_mode();
    if (s_mode_mutex) xSemaphoreGive(s_mode_mutex);
}

/**
 * BLE SIM_CONFIG 回调：更新模拟器配置
 */
static void sim_ble_config_handler(double lat, double lon, int mode, int channel,
                                    int count, int tx_power) {
    if (s_mode_mutex) xSemaphoreTake(s_mode_mutex, portMAX_DELAY);

    s_sim_config.base_lat = lat;
    s_sim_config.base_lon = lon;
    s_sim_config.flight_mode = (sim_flight_mode_t)mode;
    s_sim_config.channel = (uint8_t)channel;

    /* count > 0 时更新目标数量 */
    if (count > 0) {
        if (count > SIM_MAX_TARGETS) count = SIM_MAX_TARGETS;
        s_sim_config.target_count = count;
    }

    /* tx_power > 0 时更新发射功率 */
    if (tx_power > 0) {
        s_sim_config.tx_power = (int8_t)tx_power;
    }

    /* 如果模拟器正在运行，实时更新配置 */
    if (s_device_mode == MODE_SIMULATE && sim_get_state() == SIM_STATE_RUNNING) {
        sim_update_config(&s_sim_config);
    }

    if (s_mode_mutex) xSemaphoreGive(s_mode_mutex);

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

    snprintf(buf, buf_size,
             "{\"cmd\":\"sim_status\","
             "\"state\":\"%s\","
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
             s_sim_config.target_count,
             lat, lon, heading,
             s_sim_config.channel,
             sim_flight_mode_name(s_sim_config.flight_mode),
             s_sim_config.tx_power,
             (unsigned)sim_get_tx_count(),
             (s_device_mode == MODE_SIMULATE) ? "simulate" : "scan");
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
 * GPS 自身位置上报任务 - 每3秒通过BLE发送 SELF_GPS:lat,lon,alt,sats
 * ================================================================ */
static void gps_report_task(void *arg) {
    char buf[96];
    while (1) {
        gps_data_t gd = gps_get_data();
        if (gd.valid && gd.fix_quality > 0) {
            int len = snprintf(buf, sizeof(buf), "SELF_GPS:%.6f,%.6f,%.1f,%d\n",
                               gd.latitude, gd.longitude, gd.altitude, gd.satellites);
            if (len > 0) {
                crid_ble_write_cb(buf, (size_t)len, NULL);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}


void app_main(void) {
    // 0. 初始化 UART 数据端口（GPIO17 TX），用于输出 UAV 解析数据
    uart_data_port_init();

    // 设置数据流回调：UAV 数据同时输出到 stdout（USB CDC）和 UART1
    json_set_data_write_cb(uart_data_write_cb, NULL);

    // 初始化模拟器配置（默认值）和模式切换互斥锁
    sim_get_default_config(&s_sim_config);
    s_mode_mutex = xSemaphoreCreateMutex();
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

    // 4. 初始化 Wi-Fi sniffer
    ret = crid_sniffer_init();
    if (ret != ESP_OK) {
        json_error("RID_MAIN", "Sniffer init failed!");
        return;
    }

    // 5. 初始化 BLE NUS (NimBLE，内存分配在 SPIRAM)
    ret = crid_ble_init();
    if (ret != ESP_OK) {
        json_warning("RID_MAIN", "BLE init failed (non-fatal)");
    } else {
        // 注册模拟器控制回调（多目标版）
        crid_ble_register_sim_callbacks(sim_ble_start_handler,
                                        sim_ble_stop_handler,
                                        sim_ble_config_handler,
                                        sim_ble_status_handler);

        // BLE 就绪后切换为扇出回调：UART + BLE 双路输出
        json_set_data_write_cb(data_write_fanout, NULL);
        json_debug("RID_MAIN", "BLE data channel + SIM callbacks registered");

        // v1.8: 初始化 BLE RID 扫描模块
        // 实际扫描在 NimBLE host sync 后由 crid_ble.c 的 sync 回调触发
        crid_ble_scan_init();
        json_debug("RID_MAIN", "BLE RID scanner initialized (will start on host sync)");
    }

    // 6. 初始化 LCD 显示模块（ST7789 + SPI）
    if (lcd_display_init() == 0) {
        // 绑定追踪表数据源
        lcd_display_set_source(crid_tracker_get_table(),
                               crid_tracker_get_mutex(),
                               MAX_TRACKED_UAVS);

        // 注册BLE配对码显示回调
        crid_ble_register_pair_display(lcd_display_show_pair_pin);

        // 按键轮询已由 lcd_display 模块内部处理（T-Display-C5 自带按键）
        
        json_debug("RID_MAIN", "LCD display ready (ST7789 240x320)");
    } else {
        json_warning("RID_MAIN", "LCD init failed (non-fatal, serial only)");
    }

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
