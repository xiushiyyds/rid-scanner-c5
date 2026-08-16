/**
 * crid_sniffer.c — Wi-Fi Sniffer 模块 (detector v2.1.0)
 *
 * 纯侦测板：WiFi sniffer 持续运行，与 BLE 扫描通过 PTA 硬件共存。
 * v2.1.0: 删除 TDD 时分复用，WiFi 不再被应用层打断。
 * WiFi buffer 大幅削减以节省内部 SRAM。
 * 信道轮转：ch6 长驻 800ms，ch1/ch11 各 300ms。
 */

#include <string.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi_types.h"

#include "freertos/task.h"
#include "crid_sniffer.h"
#include "crid_json.h"
#include "opendroneid.h"
#include "dji_droneid.h"

/* 跟踪 sniffer 侧 esp_wifi_init 状态 */
static bool s_sniffer_wifi_inited = false;

/* ---- 模块内部状态 ---- */
static QueueHandle_t g_sniffer_queue = NULL;
static sniffer_stats_t g_stats;
int8_t g_rid_min_rssi = RID_MIN_RSSI_DEFAULT;
static const char *TAG = "WIFI_SNIFFER";

/* ---- 固定信道 ---- */
#ifndef FIXED_CHANNEL
#define FIXED_CHANNEL 6
#endif

/* ================================================================
 ISR 回调
 ================================================================ */
static void wifi_sniffer_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
    g_stats.total_packets++;
    if (g_stats.total_packets == 1) {
        ESP_LOGI(TAG, "✓ First WiFi packet received! Sniffer is working.");
    }
    if (type != WIFI_PKT_MGMT) return;
    g_stats.mgmt_frames++;

    wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t *)buf;
    uint16_t actual_len = pkt->rx_ctrl.sig_len;   // 不含 FCS

    if (actual_len < 36) return;

    wifi_mac_hdr_t *hdr = (wifi_mac_hdr_t *)pkt->payload;

    uint8_t type_field = (hdr->frame_ctrl & 0x0C) >> 2;
    uint8_t subtype    = (hdr->frame_ctrl & 0xF0) >> 4;
    if (type_field != 0 || (subtype != 8 && subtype != 5)) return;

    if (subtype == 8) g_stats.beacon_count++;

    uint8_t *ie_ptr = pkt->payload + 36;
    uint16_t ie_total_len = actual_len - 36;

    uint8_t ssid[33] = {0};
    uint8_t ssid_len = 0;
    bool has_rid_ie = false;
    uint16_t i = 0;

    // ---------- 结构化 IE 遍历 ----------
    while (i + 2 <= ie_total_len) {
        uint8_t id  = ie_ptr[i];
        uint8_t len = ie_ptr[i + 1];
        if (i + 2 + len > ie_total_len) break;

        // 处理 Vendor IE (ID=0xDD)
        if (id == 0xDD) {
            if (len >= 5) {
                uint8_t oui0 = ie_ptr[i + 2];
                uint8_t oui1 = ie_ptr[i + 3];
                uint8_t oui2 = ie_ptr[i + 4];
                uint8_t oui_type = ie_ptr[i + 5];

                if (IS_RID_OUI(oui0, oui1, oui2) && oui_type == 0x0D) {
                    has_rid_ie = true;
                    g_stats.rid_detections++;

                    if (pkt->rx_ctrl.rssi < g_rid_min_rssi) {
                        break;
                    }

                    uint8_t *odid_payload = &ie_ptr[i + 7];
                    uint8_t odid_len = len - 5;

                    /* v2.1.0: 降低最小长度门槛。
                     * ASTM MessagePack 最小 28B (3B header + 25B msg)。
                     * GB46750 header 仅 6B (Magic+Ver+Len+Flags)，实际
                     * mandatory 字段使最小包远大于 6，但不能用 28 门槛
                     * 阻挡短包。让解析器判断有效性。 */
                    if (odid_len < 6) {
                        break;
                    }

                    {
                        sniffer_msg_t msg;
                        memset(&msg, 0, sizeof(msg));

                        memcpy(msg.src_mac, hdr->addr2, 6);
                        msg.rssi = pkt->rx_ctrl.rssi;
                        msg.channel = pkt->rx_ctrl.channel;
                        msg.timestamp_ms = (uint32_t)(xTaskGetTickCountFromISR() * portTICK_PERIOD_MS);
                        msg.is_rid = true;
                        msg.msg_type = MSG_TYPE_RID;
                        msg.oui[0] = oui0; msg.oui[1] = oui1; msg.oui[2] = oui2;
                        msg.oui_type = oui_type;
                        msg.has_vendor_ie = true;

                        memcpy(msg.ssid, ssid, ssid_len);
                        msg.ssid_len = ssid_len;

                        uint16_t copy_len = odid_len;
                        memcpy(msg.data, odid_payload, copy_len);
                        msg.data_len = copy_len;

                        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                        if (xQueueSendFromISR(g_sniffer_queue, &msg, &xHigherPriorityTaskWoken) != pdTRUE) {
                            g_stats.queue_overflows++;
                        }
                        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
                    }
                    break;
                } else if (IS_DJI_OUI(oui0, oui1, oui2)) {
                    g_stats.dji_detections++;

                    if (pkt->rx_ctrl.rssi < g_rid_min_rssi) {
                        break;
                    }

                    uint8_t *dji_payload = &ie_ptr[i + 6];
                    uint8_t dji_len = len - 4;

                    if (dji_len >= 4) {
                        sniffer_msg_t msg;
                        memset(&msg, 0, sizeof(msg));

                        memcpy(msg.src_mac, hdr->addr2, 6);
                        msg.rssi = pkt->rx_ctrl.rssi;
                        msg.channel = pkt->rx_ctrl.channel;
                        msg.timestamp_ms = (uint32_t)(xTaskGetTickCountFromISR() * portTICK_PERIOD_MS);
                        msg.is_rid = false;
                        msg.msg_type = MSG_TYPE_DJI_DRONEID;
                        msg.oui[0] = oui0; msg.oui[1] = oui1; msg.oui[2] = oui2;
                        msg.oui_type = 0xFF;
                        msg.has_vendor_ie = true;

                        uint16_t copy_len = dji_len;
                        memcpy(msg.data, dji_payload, copy_len);
                        msg.data_len = copy_len;

                        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                        if (xQueueSendFromISR(g_sniffer_queue, &msg, &xHigherPriorityTaskWoken) != pdTRUE) {
                            g_stats.queue_overflows++;
                        }
                        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
                    }
                    break;
                } else {
                    g_stats.non_rid_vendor_ie++;
                }
            }
        }
        // 处理 SSID IE (ID=0x00)
        else if (id == 0x00 && ssid_len == 0) {
            if (len <= 32) {
                ssid_len = len;
                if (ssid_len > 0) {
                    memcpy(ssid, &ie_ptr[i + 2], ssid_len);
                }
            }
        }

        i += 2 + len;
    }

    // ---------- 无 Vendor IE 的 Beacon 采样 ----------
    static uint32_t s_beacon_no_vendor_count = 0;
    if (subtype == 8 && !has_rid_ie) {
        s_beacon_no_vendor_count++;
        if ((s_beacon_no_vendor_count & 0x7F) == 0) {
            sniffer_msg_t msg;
            memset(&msg, 0, sizeof(msg));
            memcpy(msg.src_mac, hdr->addr2, 6);
            msg.rssi = pkt->rx_ctrl.rssi;
            msg.channel = pkt->rx_ctrl.channel;
            msg.timestamp_ms = (uint32_t)(xTaskGetTickCountFromISR() * portTICK_PERIOD_MS);
            msg.is_rid = false;
            msg.msg_type = MSG_TYPE_BEACON_NO_VENDOR;
            msg.has_vendor_ie = false;
            memcpy(msg.ssid, ssid, ssid_len);
            msg.ssid_len = ssid_len;

            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            if (xQueueSendFromISR(g_sniffer_queue, &msg, &xHigherPriorityTaskWoken) != pdTRUE) {
                g_stats.queue_overflows++;
            }
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

/* ================================================================
 信道轮转（三信道轮巡）

 WiFi sniffer 持续运行，与 BLE 扫描通过 PTA 硬件共存。
 不再依赖应用层 TDD（v2.1.0 删除）。

 ch6 驻留 800ms（多数无人机默认信道，偏重），
 ch1/ch11 各驻留 300ms（覆盖非默认信道）。
 周期 1.4 秒，ch6 占 57%，ch1/ch11 各占 21%。
 ================================================================ */
static const uint8_t SCAN_CHANNELS[] = {6, 1, 11};
static const uint16_t CHANNEL_DWELL_MS_ARR[] = {800, 300, 300};
#define SCAN_CHANNEL_COUNT (sizeof(SCAN_CHANNELS) / sizeof(SCAN_CHANNELS[0]))
static volatile uint8_t s_current_channel = 6;

uint8_t crid_sniffer_get_current_channel(void) {
    return s_current_channel;
}

/* ---- 信道轮转任务 ---- */
static volatile bool s_hold_should_stop = false;
static TaskHandle_t s_hold_task_handle = NULL;

static void channel_hold_task(void *pvParameter) {
    (void)pvParameter;
    char msg[80];
    snprintf(msg, sizeof(msg), "Channel rotation: ch6 2s / ch1,ch11 500ms (BLE continuous)");
    json_debug("RID_SNIFF", msg);

    vTaskDelay(pdMS_TO_TICKS(2000));

    uint8_t idx = 0;
    for (uint8_t i = 0; i < SCAN_CHANNEL_COUNT; i++) {
        if (SCAN_CHANNELS[i] == 6) { idx = i; break; }
    }

    /* 诊断统计 */
    uint32_t diag_counter = 0;
    uint32_t last_total = 0, last_rid = 0, last_beacon = 0;

    while (!s_hold_should_stop) {
        uint8_t ch = SCAN_CHANNELS[idx];
        uint16_t dwell = CHANNEL_DWELL_MS_ARR[idx];
        s_current_channel = ch;

        esp_err_t ret = esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
        if (ret != ESP_OK && !s_hold_should_stop) {
            snprintf(msg, sizeof(msg), "set channel %d: %s", ch, esp_err_to_name(ret));
            json_warning("RID_SNIFF", msg);
        }
        idx = (idx + 1) % SCAN_CHANNEL_COUNT;

        /* 分段 delay，stop 时能快速退出 */
        for (int i = 0; i < (dwell / 100) && !s_hold_should_stop; i++) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        if (s_hold_should_stop) break;

        /* 每 ~10 轮打印一次诊断 */
        diag_counter++;
        if (diag_counter % 10 == 0) {
            uint32_t cur_total = g_stats.total_packets;
            uint32_t cur_rid = g_stats.rid_detections;
            uint32_t cur_beacon = g_stats.beacon_count;
            uint32_t d_total = cur_total - last_total;
            uint32_t d_rid = cur_rid - last_rid;
            uint32_t d_beacon = cur_beacon - last_beacon;
            last_total = cur_total;
            last_rid = cur_rid;
            last_beacon = cur_beacon;
            snprintf(msg, sizeof(msg),
                     "Diag: total=%lu(+%lu) mgmt=+%lu RID=%lu(+%lu) ch=%u",
                     (unsigned long)cur_total, (unsigned long)d_total,
                     (unsigned long)d_beacon,
                     (unsigned long)cur_rid, (unsigned long)d_rid,
                     s_current_channel);
            json_debug("RID_SNIFF", msg);
        }
    }

    s_hold_should_stop = false;
    s_hold_task_handle = NULL;
    vTaskDelete(NULL);
}

/* ---- 公开接口 ---- */
QueueHandle_t crid_sniffer_get_queue(void) {
    return g_sniffer_queue;
}

esp_err_t crid_sniffer_queue_create(void) {
    if (g_sniffer_queue != NULL) {
        return ESP_OK;
    }
    g_sniffer_queue = xQueueCreate(SNIFFER_QUEUE_SIZE, sizeof(sniffer_msg_t));
    if (g_sniffer_queue == NULL) {
        json_error("RID_SNIFF", "Failed to create sniffer queue!");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

sniffer_stats_t *crid_sniffer_get_stats(void) {
    return &g_stats;
}

esp_err_t crid_sniffer_init(void) {
    memset(&g_stats, 0, sizeof(g_stats));
    char err[64];
    esp_err_t ret;

    ESP_LOGI(TAG, "Initializing Wi-Fi Sniffer (detector, reduced buffers)...");

    if (g_sniffer_queue == NULL) {
        g_sniffer_queue = xQueueCreate(SNIFFER_QUEUE_SIZE, sizeof(sniffer_msg_t));
        if (g_sniffer_queue == NULL) {
            json_error("RID_SNIFF", "Failed to create sniffer queue!");
            return ESP_ERR_NO_MEM;
        }
    } else {
        sniffer_msg_t discard;
        while (xQueueReceive(g_sniffer_queue, &discard, 0) == pdTRUE) {}
    }

    if (!s_sniffer_wifi_inited) {
        /* 自定义 WiFi init config：大幅削减 buffer 节省内部 SRAM。
         * sniffer 只收不发，但 ESP-IDF v5.5 要求 dynamic_tx_buf_num >= 1，
         * 否则 esp_wifi_init 返回 ESP_ERR_INVALID_ARG。
         * v2.1.1 修复：dynamic_tx_buf_num 0→1, cache_tx_buf_num 0→1。 */
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        cfg.static_rx_buf_num = 4;        // 10→4, 省 ~10KB
        cfg.dynamic_rx_buf_num = 8;       // 32→8
        cfg.static_tx_buf_num = 0;        // sniffer 不发数据帧
        cfg.dynamic_tx_buf_num = 1;       // v5.5 要求 >=1（原为0导致init失败）
        cfg.rx_ba_win = 2;                // 6→2
        cfg.csi_enable = 0;               // 不需要 CSI
        cfg.mgmt_sbuf_num = 4;            // 32→4
        cfg.cache_tx_buf_num = 1;         // v5.5 要求 >=1（原为0导致init失败）
        cfg.ampdu_tx_enable = 0;          // sniffer 不发 AMPDU
        cfg.amsdu_tx_enable = 0;          // sniffer 不发 AMSDU

        ret = esp_wifi_init(&cfg);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "esp_wifi_init returned %s", esp_err_to_name(ret));
        }
        s_sniffer_wifi_inited = true;
    } else {
        ESP_LOGI(TAG, "Wi-Fi driver already initialized, reusing");
    }

    ret = esp_wifi_set_mode(WIFI_MODE_NULL);
    if (ret != ESP_OK) {
        snprintf(err, sizeof(err), "Wi-Fi mode failed: %s", esp_err_to_name(ret));
        json_error("RID_SNIFF", err);
        return ret;
    }

    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        snprintf(err, sizeof(err), "Wi-Fi start failed: %s", esp_err_to_name(ret));
        json_error("RID_SNIFF", err);
        return ret;
    }

    esp_wifi_set_promiscuous_rx_cb(wifi_sniffer_cb);

    wifi_promiscuous_filter_t filter = {
        .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT
    };
    esp_wifi_set_promiscuous_filter(&filter);

    ret = esp_wifi_set_promiscuous(true);
    if (ret != ESP_OK) {
        snprintf(err, sizeof(err), "Promiscuous mode failed: %s", esp_err_to_name(ret));
        json_error("RID_SNIFF", err);
        return ret;
    }

    /* 不锁协议，11b/g/n 全收。关闭省电。 */
    esp_wifi_set_ps(WIFI_PS_NONE);

    ret = esp_wifi_set_channel(FIXED_CHANNEL, WIFI_SECOND_CHAN_NONE);
    if (ret != ESP_OK) {
        snprintf(err, sizeof(err), "Set channel failed: %s", esp_err_to_name(ret));
        json_error("RID_SNIFF", err);
        return ret;
    }

    snprintf(err, sizeof(err), "Wi-Fi monitor mode enabled (reduced bufs), channel %d", FIXED_CHANNEL);
    json_debug("RID_SNIFF", err);

    return ESP_OK;
}

void crid_sniffer_start_channel_hold(void) {
    if (s_hold_task_handle != NULL) {
        for (int i = 0; i < 20 && s_hold_task_handle != NULL; i++) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    s_hold_should_stop = false;
    xTaskCreate(channel_hold_task, "ch_hold",
                CH_HOLD_TASK_STACK, NULL, CH_HOLD_TASK_PRIO, &s_hold_task_handle);
}

void crid_sniffer_stop_channel_hold(void) {
    s_hold_should_stop = true;
}
