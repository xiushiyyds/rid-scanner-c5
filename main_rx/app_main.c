/**
 * app_main.c — Remote ID Detector 主入口 (v2.1.0-detector)
 *
 * ESP32-C5 Remote ID 纯侦测板固件
 * Standards: ASTM F3411-22a / ASD-STAN prEN 4709-002 / GB 42590-2023 / GB 46750-2025
 *
 * 架构 (v2.1.0):
 *   - crid_sniffer:   Wi-Fi 混杂模式抓包（持续运行，PTA 硬件共存）
 *   - crid_ble_scan:  BLE RID 广播扫描（window<itvl 占空比，GATT 自动共存）
 *     · 未连接：HIGH 80% duty（window=80ms/itvl=100ms）
 *     · 已连接：LOW 40% duty（window=40ms/itvl=100ms，GATT 事件在间隙调度）
 *   - crid_parser:    多协议解码（GB46750 → GB42590 → ASTM → DJI）
 *   - crid_tracker:   无人机追踪表
 *   - lcd_display:    ST7789 LCD 信息展示
 *
 * v2.1.0 关键架构变更：
 *   - 删除应用层 TDD 时分复用。TDD 频繁 cancel/restart 扫描干扰 PTA
 *     硬件共存，是 WiFi total_pkts=0 的根因。
 *   - WiFi/BLE 共存由 CONFIG_ESP_COEX_SW_COEXIST_ENABLE=y 的 PTA 仲裁。
 *   - BLE GATT 连接事件由控制器在 scan window 间隙自动调度。
 *   - GB46750 BLE/WiFi 路径修复，支持变长包透传。
 *
 * 纯侦测板：只做 RID 信号侦测和显示，无模拟发射。
 * WiFi init 在所有任务创建之后，使用自定义 config 削减 buffer。
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
#include "dji_droneid.h"
#include "evlog.h"

#if CONFIG_RID_SCANNER_BUILD_SIMULATOR
#include "sim_core.h"
#include "sim_cities.h"
#include "sim_lcd_ui.h"
#endif

#ifndef CRID_VERSION_STRING
#define CRID_VERSION_STRING "2.7.0"
#endif
#ifndef CRID_BUILD_DATE
#define CRID_BUILD_DATE     __DATE__
#endif
#ifndef CRID_BUILD_TIME
#define CRID_BUILD_TIME     __TIME__
#endif

/* ================================================================
 * UART 数据端口配置
 * ================================================================ */
#define UART_DATA_PORT_NUM      UART_NUM_1
#define UART_DATA_TX_PIN        17
#define UART_DATA_RX_PIN        18
#define UART_DATA_BAUD_RATE     115200
#define UART_DATA_BUF_SIZE      1024

static bool s_uart_data_initialized = false;

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

static void uart_data_write_cb(const char *data, size_t len, void *ctx) {
    (void)ctx;
    if (!s_uart_data_initialized) return;
    uart_write_bytes(UART_DATA_PORT_NUM, data, len);
}

static void data_write_fanout(const char *data, size_t len, void *ctx) {
    uart_data_write_cb(data, len, ctx);
    crid_ble_write_cb(data, len, ctx);
}

/* ================================================================
 * 解析任务
 * ================================================================ */
static void parser_task(void *pvParameter) {
    json_debug("RID_MAIN", "Parser task started");

    QueueHandle_t queue = crid_sniffer_get_queue();
    SemaphoreHandle_t mutex = crid_tracker_get_mutex();
    sniffer_msg_t msg;
    uint32_t last_cleanup_ms = 0;
    uint32_t last_idle_output_ms = 0;

    while (1) {
        /* v2.0.6: 即使没收到包也定期清理超时 track，避免占满表 */
        {
            uint32_t now = esp_log_timestamp();
            if (now - last_cleanup_ms >= 5000) {
                if (xSemaphoreTake(mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    crid_tracker_cleanup(UAV_TIMEOUT_MS);
                    xSemaphoreGive(mutex);
                }
                last_cleanup_ms = now;
            }
        }
        if (xQueueReceive(queue, &msg, pdMS_TO_TICKS(1000)) != pdTRUE) {
            uint32_t now = esp_log_timestamp();
            if (crid_tracker_get_active_count() == 0 &&
                (now - last_idle_output_ms >= 1000)) {
                json_no_aircraft();
                last_idle_output_ms = now;
            }
            continue;
        }

        if (msg.msg_type == MSG_TYPE_BEACON_NO_VENDOR) continue;
        if (msg.msg_type == MSG_TYPE_NON_RID_VENDOR) continue;

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
            crid_tracker_update_rssi(uav, msg.rssi);

            uav->is_dji = true;
            uav->protocol = RID_PROTOCOL_UNKNOWN;

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
                uav->dji_battery = 0;
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

                if (dji_data.type == 0x10) {
                    uav->location.valid = true;
                    uav->location.latitude = dji_data.latitude;
                    uav->location.longitude = dji_data.longitude;
                    uav->location.altitude_baro = dji_data.altitude;
                    uav->location.height = dji_data.height;
                    uav->location.speed_horizontal = dji_data.speed_h;
                    uav->location.speed_vertical = dji_data.speed_up;
                    uav->location.direction = dji_data.heading;

                    uav->basic_id.valid = true;
                    uav->basic_id.id_type = 1;
                    uav->basic_id.ua_type = 2;
                    strncpy(uav->basic_id.uas_id, dji_data.serial,
                            sizeof(uav->basic_id.uas_id) - 1);
                    uav->basic_id.uas_id[sizeof(uav->basic_id.uas_id) - 1] = '\0';
                }

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

            /* v2.0.5: DJI 路径也做跨 MAC 合并（DJI DroneID 用固定 SN 但 WiFi MAC 随机） */
            if (uav->basic_id.valid && uav->basic_id.uas_id[0] != '\0') {
                uav_track_t *existing = crid_tracker_find_by_uas_id(uav->basic_id.uas_id);
                if (existing != NULL && existing != uav) {
                    existing->msg_count += uav->msg_count;
                    existing->last_seen_ms = uav->last_seen_ms;
                    existing->last_rssi = uav->last_rssi;
                    existing->last_channel = uav->last_channel;
                    existing->is_dji = true;
                    existing->dji_type = uav->dji_type;
                    existing->dji_model_code = uav->dji_model_code;
                    if (uav->location.latitude != 0 || uav->location.longitude != 0)
                        existing->location = uav->location;
                    if (uav->basic_id.valid)
                        existing->basic_id = uav->basic_id;
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
                    existing->dji_latitude = uav->dji_latitude;
                    existing->dji_longitude = uav->dji_longitude;
                    existing->dji_altitude = uav->dji_altitude;
                    existing->dji_height = uav->dji_height;
                    existing->dji_speed_h = uav->dji_speed_h;
                    existing->dji_speed_v = uav->dji_speed_v;
                    existing->dji_heading = uav->dji_heading;
                    existing->dji_pilot_lat = uav->dji_pilot_lat;
                    existing->dji_pilot_lon = uav->dji_pilot_lon;
                    strncpy(existing->dji_identification, uav->dji_identification,
                            sizeof(existing->dji_identification) - 1);
                    uav->active = false;
                    uav->msg_count = 0;
                    ESP_LOGI("RID_DEDUP", "DJI merged MAC %02X:%02X:%02X:%02X:%02X:%02X -> SN %s",
                             msg.src_mac[0], msg.src_mac[1], msg.src_mac[2],
                             msg.src_mac[3], msg.src_mac[4], msg.src_mac[5],
                             uav->basic_id.uas_id);
                    uav = existing;
                    was_new = false;
                }
            }

            /* v2.6.4: 证据留存条件在 mutex 内判断，但 evlog_write 移到 mutex 外。
             * evlog_write 内部可能执行 esp_partition_erase_range（扇区擦除 30-50ms），
             * 持锁期间会阻塞 LCD 刷新和 parser 的其他迭代。 */
            bool need_evlog = (was_new || uav->msg_count % 50 == 0) &&
                              (uav->dji_serial[0] || uav->location.valid);

            xSemaphoreGive(mutex);

            /* v2.6.4: 在 mutex 外写 Flash（uav 指针指向 tracker 静态表，
             * monitor_task 可能将 slot active=false 但不释放内存，
             * evlog_write 只读取字段，30-50ms 内数据不会被覆盖） */
            if (need_evlog) {
                evlog_write(uav);
            }

            /* v2.0.8: DJI 路径同样立即发首次，后续 1 秒节流 */
            if (was_new && (uav->basic_id.valid || uav->location.valid)) {
                json_uav_discovery(uav);
                json_uav_update(uav);
                uav->last_push_ms = esp_log_timestamp();
            } else if (uav->basic_id.valid || uav->location.valid) {
                uint32_t now = esp_log_timestamp();
                /* v2.6.2: 推送节流从 1000ms 降到 500ms，雷达 RSSI 刷新率翻倍。
                 * RID 广播约 1Hz，但锁定后 sniffer 包接收率高，500ms 节流
                 * 确保每次收到新包都能及时推送，雷达跟手度显著改善。 */
                if (now - uav->last_push_ms >= 500) {
                    uav->last_push_ms = now;
                    json_uav_update(uav);
                }
            }
            continue;
        }

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
        crid_tracker_update_rssi(uav, msg.rssi);

        uav->oui[0] = msg.oui[0];
        uav->oui[1] = msg.oui[1];
        uav->oui[2] = msg.oui[2];
        uav->oui_type = msg.oui_type;
        uav->transport = (uint8_t)GET_RID_TRANSPORT(msg.oui[0], msg.oui[1], msg.oui[2]);

        rid_protocol_t detected_proto = crid_parser_decode(uav, msg.data, msg.data_len);
        if (detected_proto != RID_PROTOCOL_UNKNOWN) {
            uav->protocol = (uint8_t)detected_proto;
        }

        crid_parser_extract_layered(uav);

        /* v2.0.5: 每次收到 BasicID 都检查跨 MAC 合并（不再限定 was_new）。
         * 无人机 RID 广播每 1-3 秒随机换 MAC，旧 MAC 的 track 上已有 SN，
         * 新 MAC 第一次收到 BasicID 时也应合并到同一 track，否则每个随机 MAC
         * 都会在网页上变成重复目标。 */
        if (uav->basic_id.valid && uav->basic_id.uas_id[0] != '\0') {
            uav_track_t *existing = crid_tracker_find_by_uas_id(uav->basic_id.uas_id);
            if (existing != NULL && existing != uav) {
                existing->msg_count += uav->msg_count;
                existing->last_seen_ms = uav->last_seen_ms;
                existing->last_rssi = uav->last_rssi;
                existing->last_channel = uav->last_channel;
                existing->oui[0] = uav->oui[0];
                existing->oui[1] = uav->oui[1];
                existing->oui[2] = uav->oui[2];
                existing->transport = uav->transport;
                existing->protocol = uav->protocol;
                /* 只合并有效字段，不盲目覆盖已有数据 */
                if (uav->location.latitude != 0 || uav->location.longitude != 0)
                    existing->location = uav->location;
                if (uav->basic_id.valid)
                    existing->basic_id = uav->basic_id;
                if (uav->system.valid)
                    existing->system = uav->system;
                if (uav->self_id.valid)
                    existing->self_id = uav->self_id;
                if (uav->operator_id.valid)
                    existing->operator_id = uav->operator_id;
                if (uav->gb46750.valid)
                    existing->gb46750 = uav->gb46750;
                existing->uas_data = uav->uas_data;
                existing->last_pack = uav->last_pack;
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
                uav->active = false;
                uav->msg_count = 0;
                ESP_LOGI("RID_DEDUP", "ASTM merged MAC %02X:%02X:%02X:%02X:%02X:%02X -> SN %s",
                         msg.src_mac[0], msg.src_mac[1], msg.src_mac[2],
                         msg.src_mac[3], msg.src_mac[4], msg.src_mac[5],
                         uav->basic_id.uas_id);
                uav = existing;
                was_new = false;
            }
        }

        if (uav->location.valid) {
            geofence_alert_t alert;
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

        /* v2.6.4: 证据留存条件在 mutex 内判断，evlog_write 移到 mutex 外 */
        bool need_evlog = (was_new || uav->msg_count % 50 == 0) &&
                          (uav->basic_id.valid || uav->location.valid);

        xSemaphoreGive(mutex);

        if (need_evlog) {
            evlog_write(uav);
        }

        /* v2.0.8: 数据推送策略。
         * 新目标立即发 discovery + update（无节流）。
         * 已有目标 1 秒节流推 update（RID 广播约 1Hz，1s 节流刚好）。
         * monitor_task 不再单独推 uav_status（由 update 覆盖）。 */
        if (was_new && (uav->basic_id.valid || uav->location.valid)) {
            json_uav_discovery(uav);
            json_uav_update(uav);
            uav->last_push_ms = esp_log_timestamp();
        } else if (uav->basic_id.valid || uav->location.valid) {
            uint32_t now = esp_log_timestamp();
            /* v2.6.2: 500ms 节流（原 1000ms），雷达 RSSI 更新翻倍 */
            if (now - uav->last_push_ms >= 500) {
                uav->last_push_ms = now;
                json_uav_update(uav);
            }
        }
    }
}

/* ================================================================
 * 监控任务
 * ================================================================ */
static void monitor_task(void *pvParameter) {
    json_debug("RID_MAIN", "Monitor task started");

    uint32_t loop_count = 0;
    uint32_t last_packets = 0, last_mgmt = 0, last_rid = 0;
    uint32_t last_beacons = 0, last_non_rid = 0;

    while (1) {
        /* v2.0.8: 每60秒输出一次完整统计报告。
         * 活跃目标的实时数据由 parser_task 在收到 RID 消息时立即推送，
         * 不再由 monitor 定期推 uav_status（避免重复推送和 BLE 队列拥塞）。 */
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

        int active = 0;
        if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            active = crid_tracker_get_active_count();
            crid_tracker_cleanup(UAV_TIMEOUT_MS);
            xSemaphoreGive(mutex);
        }

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
 * GPS 自身位置上报任务
 * ================================================================ */
static void gps_report_task(void *arg) {
    (void)arg;
    char buf[96];
    while (1) {
        gps_data_t gd = gps_get_data();
        if (gd.valid && gd.fix_quality > 0) {
            /* v2.0.5: 增加 HDOP 字段，手机端用于精度圆圈和质量评估 */
            snprintf(buf, sizeof(buf), "SELF_GPS:%.6f,%.6f,%.1f,%d,%.1f\n",
                     gd.latitude, gd.longitude, gd.altitude, gd.satellites, gd.hdop);
        } else {
            /* v2.6.3: 未定位时也发送卫星数和HDOP，让网页显示"搜索中(N星)"
             * 而不是"搜索中(0星)"。之前只发4个0导致网页解析sats=0。 */
            snprintf(buf, sizeof(buf), "SELF_GPS:0,0,0,%d,%.1f\n",
                     gd.satellites, gd.hdop);
        }
        /* v2.6.8: 通过 json_raw_line 走 data_write() 通道（stdout DATA:前缀 + BLE），
         * 跟 uav_update 等 JSON 数据走完全相同的路径，确保 USB 网页端能收到。
         * 之前只调 crid_ble_write_cb 导致 USB 网页端收不到 SELF_GPS。 */
        json_raw_line(buf);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

/* LCD 状态栏 GPS 回调 */
static bool lcd_gps_provider(double *lat, double *lon, double *alt, int *sats_out) {
    gps_data_t gd = gps_get_data();
    bool fix = gd.valid && gd.fix_quality > 0;
    if (lat) *lat = gd.latitude;
    if (lon) *lon = gd.longitude;
    if (alt) *alt = gd.altitude;
    if (sats_out) *sats_out = gd.satellites;
    return fix;
}

/* ================================================================
 * 主函数
 *
 * 初始化顺序（严格遵守踩坑记录）：
 *   1. UART + JSON callbacks
 *   2. tracker + geofence + dji
 *   3. GPS
 *   4. NVS + netif + event loop
 *   5. Release classic BT
 *   6. BLE init + BLE scan init
 *   7. LCD init + providers
 *   8. sniffer queue create
 *   9. parser task
 *  10. monitor task
 *  11. gps_report task
 *  12. WiFi sniffer init (LAST)
 *  13. channel rotation start
 *  14. startup banner
 * ================================================================ */
void app_main(void) {

#if CONFIG_RID_SCANNER_BUILD_SIMULATOR
    /* ================================================================
     * 模拟器构建：直接进入模拟发射模式（不显示菜单）。
     * 注意：此构建链接了 crid_simulator 组件，不能用于侦测——
     * 链接模拟器会改变内存布局导致 WiFi/BLE 共存异常。
     * 侦测请刷 firmware-detector 构建。
     * ================================================================ */
    if (lcd_display_early_init() != 0) {
        ESP_LOGE("RID_MAIN", "LCD early init failed");
    }
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    esp_netif_init();
    esp_event_loop_create_default();

    ESP_LOGI("RID_MAIN", "Boot mode: SIMULATOR");

    lcd_display_init_for_sim();
    sim_init();

    sim_config_t sim_cfg;
    sim_get_default_config(&sim_cfg);
    sim_cfg.target_count = 300;
    sim_cfg.channel = 6;
    sim_cfg.tx_power = 20;
    sim_cfg.brand = SIM_BRAND_MIXED;
    sim_cfg.chan_mode = SIM_CHAN_ROTATE_1_6_11;
    sim_cfg.speed = 5.0f;
    sim_cfg.flight_mode = SIM_MODE_CIRCLE;
    sim_cfg.frame_interval_ms = 3;
    sim_cfg.round_interval_ms = 1000;
    sim_cfg.base_lat = g_sim_cities[sim_cfg.city_index].lat;
    sim_cfg.base_lon = g_sim_cities[sim_cfg.city_index].lon;
    sim_update_config(&sim_cfg);

    ESP_LOGI("RID_MAIN", "Simulator armed (not transmitting). Press B on LCD to start.");
    sim_lcd_ui_start();
    printf("\nRID Simulator ready. Type 'help' for commands.\n\n");

    char cli_ch;
    while (1) {
        int n = fread(&cli_ch, 1, 1, stdin);
        if (n > 0) {
            putchar(cli_ch);
            sim_cli_feed(&cli_ch, 1);
        } else {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
    /* 不会到达这里 */
#else
    /* ================================================================
     * 纯侦测构建（默认）：直接启动，不开机菜单，不链接模拟器。
     * 这是日常使用的固件，WiFi/BLE 共存最稳定。
     * ================================================================ */
    ESP_LOGI("RID_MAIN", "Starting detector (pure build, no simulator)...");
#endif

    /* ================================================================
     * 侦测模式启动流程（与 v2.3.2 完全一致）
     * BLE 先初始化抢占连续内部 SRAM，LCD 在 BLE 之后初始化，WiFi 最后启动。
     * ================================================================ */
    // 4. NVS + netif + event loop
    // (模拟器构建中 NVS 已在上方初始化，这里跳过)
#if !CONFIG_RID_SCANNER_BUILD_SIMULATOR
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        json_error("RID_MAIN", "NVS init failed");
        return;
    }
    esp_netif_init();
    esp_event_loop_create_default();
#endif

    // 1. UART 数据端口 + JSON 回调（在 NVS 之后）
    uart_data_port_init();
    json_set_data_write_cb(uart_data_write_cb, NULL);

    // JSON 启动信息
    json_startup_info(CRID_VERSION_STRING, CRID_BUILD_DATE, CRID_BUILD_TIME,
                      esp_get_idf_version(),
                      (uint32_t)esp_get_free_heap_size(),
                      ODID_PROTOCOL_VERSION);

    // 2. 追踪器 + 地理围栏 + DJI
    crid_tracker_init();
    geofence_init();
    dji_droneid_init();

    // 3. GPS
    gps_init();
    ESP_LOGI("RID_MAIN", "GPS module initialized");

    // 3.5 证据日志
    evlog_init();

    // 5. 释放经典蓝牙内存
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);

    ESP_LOGI("RID_MAIN", "Before BLE init - free heap: %u, internal: %u, largest: %u",
             (unsigned)esp_get_free_heap_size(),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));

    // 6. BLE 初始化（必须在 LCD 之前，抢占连续内部 SRAM）
    crid_ble_register_pair_display(lcd_display_show_pair_pin);

    ret = crid_ble_init();
    if (ret != ESP_OK) {
        ESP_LOGE("RID_MAIN", "BLE init failed: %s", esp_err_to_name(ret));
        json_warning("RID_MAIN", "BLE init failed (non-fatal)");
    } else {
        json_set_data_write_cb(data_write_fanout, NULL);
        crid_ble_scan_init();
        ESP_LOGI("RID_MAIN", "BLE ready, data fanout enabled");
    }

    ESP_LOGI("RID_MAIN", "After BLE init - free heap: %u, internal: %u, largest: %u",
             (unsigned)esp_get_free_heap_size(),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));

    // 7. LCD 初始化（BLE 之后）
    if (lcd_display_init() == 0) {
        lcd_display_set_source(crid_tracker_get_table(),
                               crid_tracker_get_mutex(),
                               MAX_TRACKED_UAVS);
        lcd_display_register_gps_provider(lcd_gps_provider);
        lcd_display_register_channel_provider(crid_sniffer_get_current_channel);
        json_debug("RID_MAIN", "LCD display ready (ST7789 170x320)");
    } else {
        json_warning("RID_MAIN", "LCD init failed (non-fatal, serial only)");
    }

    // 8. 创建 sniffer 队列（不依赖 WiFi）
    if (crid_sniffer_queue_create() != ESP_OK) {
        json_error("RID_MAIN", "Failed to create sniffer queue!");
        return;
    }

    /* v2.6.7: WiFi init 必须在创建应用任务之前。
     * v2.6.4 把 parser/monitor/NimBLE host 栈加大后，应用任务先创建会吃掉
     * 连续内部 SRAM，导致 esp_wifi_init 分配 10×1700B DMA 接收缓冲时
     * 返回 ESP_ERR_NO_MEM，sniffer 初始化失败，信道轮转永远跑不起来。
     * WiFi 驱动需要大块连续 DMA 内存，必须最先抢占。 */
    char mem_dbg[96];
    snprintf(mem_dbg, sizeof(mem_dbg),
             "WiFi init before app tasks: internal_free=%u largest=%u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    json_debug("RID_MAIN", mem_dbg);

    ret = crid_sniffer_init();
    if (ret != ESP_OK) {
        json_error("RID_MAIN", "Sniffer init failed!");
        return;
    }

    snprintf(mem_dbg, sizeof(mem_dbg),
             "WiFi init OK, creating app tasks: internal_free=%u largest=%u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    json_debug("RID_MAIN", mem_dbg);

    // 9-11. 创建应用任务（WiFi 已占住 DMA 内存，剩余内存给任务栈）
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

    /* v2.6.4: 栈 3072→4096。crid_ble_write_cb 内部有 1024 字节栈快照，
     * 加上本任务的 96 字节 buf 和调用帧，3072 偏紧。 */
    xTaskCreatePinnedToCore(gps_report_task, "gps_rpt", 4096, NULL, 3, NULL, 0);

    // 13. 启动信道轮转（BLE 持续扫描，WiFi 共存硬件自动分时）
    crid_sniffer_start_channel_hold();

    // 14. 启动完成
    json_startup_banner(CRID_VERSION_STRING, CRID_BUILD_DATE, CRID_BUILD_TIME,
                        FIXED_CHANNEL, MAX_TRACKED_UAVS,
                        (uint32_t)esp_get_free_heap_size());

    ESP_LOGI("RID_MAIN", "Detector v%s started — BLE continuous scan + WiFi sniffer",
             CRID_VERSION_STRING);
}
