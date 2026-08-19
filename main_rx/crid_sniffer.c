/**
 * crid_sniffer.c — Wi-Fi Sniffer 模块 (detector v2.1.0)
 *
 * 纯侦测板：WiFi sniffer 持续运行，与 BLE 扫描通过 PTA 硬件共存。
 * v2.1.0: 删除 TDD 时分复用，WiFi 不再被应用层打断。
 * WiFi buffer 大幅削减以节省内部 SRAM。
 * 信道轮转：ch6 1500ms, ch1/ch11 各 250ms（v2.5.9 回退 v2.3.2）。
 */

#include <string.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi_types.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/idf_additions.h"

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

                    /* DJI Vendor IE: [ID][len][OUI 3B][payload...]
                     * payload 紧跟 OUI 之后（无 vendor_type 字节），
                     * 从 ie_ptr[i+5] 开始，长度 len-3。 */
                    uint8_t *dji_payload = &ie_ptr[i + 5];
                    uint8_t dji_len = len - 3;

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
 信道轮转（v2.6.0 自适应信道锁定）

 无目标时：ch6 1000ms / ch1 250ms / ch11 250ms，周期 1.5s。
 发现目标后：锁定目标所在信道持续监听，不再轮巡。
 目标全部超时消失后：恢复轮巡。

 绝大多数 WiFi RID 设备（DJI/Parrot 等）固定使用 ch6。
 自适应锁定避免发现目标后还在切信道导致丢包。
 ================================================================ */
static const uint8_t SCAN_CHANNELS[] = {6, 1, 11};
static const uint16_t CHANNEL_DWELL_MS_ARR[] = {1000, 250, 250};
#define SCAN_CHANNEL_COUNT (sizeof(SCAN_CHANNELS) / sizeof(SCAN_CHANNELS[0]))
static volatile uint8_t s_current_channel = 6;
static volatile uint8_t s_locked_channel = 0;  /* 0=未锁定，非0=锁定信道 */

uint8_t crid_sniffer_get_current_channel(void) {
    return s_current_channel;
}

/* ---- 信道轮转任务 ---- */
static volatile bool s_hold_should_stop = false;
static TaskHandle_t s_hold_task_handle = NULL;

/* 判断是否有活跃目标，并找出最近的信道（用于锁定） */
static uint8_t get_best_lock_channel(void) {
    extern uav_track_t *crid_tracker_get_table(void);
    uav_track_t *table = crid_tracker_get_table();
    if (!table) return 0;

    uint8_t best_ch = 0;
    uint32_t best_last_seen = 0;
    for (int i = 0; i < MAX_TRACKED_UAVS; i++) {
        if (table[i].mac[0] || table[i].mac[1]) {
            uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
            if (now - table[i].last_seen_ms < 30000) {
                /* 活跃目标，取最近更新的那个的信道 */
                if (table[i].last_seen_ms > best_last_seen) {
                    best_last_seen = table[i].last_seen_ms;
                    best_ch = table[i].last_channel & 0x7F;
                    if (best_ch == 0) best_ch = 6;  /* BLE 源(ch=0)默认 ch6 */
                }
            }
        }
    }
    return best_ch;
}

static void channel_hold_task(void *pvParameter) {
    (void)pvParameter;
    char msg[96];
    snprintf(msg, sizeof(msg),
             "Channel rotation v2.6.0: ch6 1.0s / ch1,ch11 250ms (adaptive lock)");
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
        /* 自适应：检查是否需要锁定/解锁 */
        uint8_t lock_ch = get_best_lock_channel();
        if (lock_ch != s_locked_channel) {
            if (lock_ch != 0) {
                s_locked_channel = lock_ch;
                snprintf(msg, sizeof(msg), "Adaptive lock: ch%u (target found)", lock_ch);
                json_debug("RID_SNIFF", msg);
            } else if (s_locked_channel != 0) {
                snprintf(msg, sizeof(msg), "Adaptive unlock: ch%u (no targets)", s_locked_channel);
                json_debug("RID_SNIFF", msg);
                s_locked_channel = 0;
            }
        }

        uint8_t ch;
        uint16_t dwell;

        if (s_locked_channel != 0) {
            /* 锁定状态：持续在目标信道上 */
            ch = s_locked_channel;
            dwell = 200;  /* 每 200ms 检查一次是否还需要锁定 */
        } else {
            /* 轮巡状态 */
            ch = SCAN_CHANNELS[idx];
            dwell = CHANNEL_DWELL_MS_ARR[idx];
            idx = (idx + 1) % SCAN_CHANNEL_COUNT;
        }

        s_current_channel = ch;

        esp_err_t ret = esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
        if (ret != ESP_OK && !s_hold_should_stop) {
            snprintf(msg, sizeof(msg), "set channel %d: %s", ch, esp_err_to_name(ret));
            json_warning("RID_SNIFF", msg);
        }

        /* 分段 delay，stop 时能快速退出 */
        for (int i = 0; i < (dwell / 50) && !s_hold_should_stop; i++) {
            vTaskDelay(pdMS_TO_TICKS(50));
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
                     "Diag: total=%lu(+%lu) mgmt=+%lu RID=%lu(+%lu) ch=%u%s",
                     (unsigned long)cur_total, (unsigned long)d_total,
                     (unsigned long)d_beacon,
                     (unsigned long)cur_rid, (unsigned long)d_rid,
                     s_current_channel,
                     s_locked_channel ? " LOCKED" : "");
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
        /* v2.1.3: 直接使用 WIFI_INIT_CONFIG_DEFAULT()，不手动削减 buffer。
         * IDF v5.5 对 tx_buf、mgmt_sbuf 等有最低值校验，
         * 之前多次因自定义值 out of range 导致 esp_wifi_init 失败。
         * 内部 SRAM 140KB，BLE init 后剩 ~110KB，默认 WiFi 配置可接受。 */
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

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

    char msg[96];
    snprintf(msg, sizeof(msg),
             "ch_hold create: free_heap=%u internal_free=%u internal_largest=%u",
             (unsigned)esp_get_free_heap_size(),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    json_debug("RID_SNIFF", msg);

    /* 优先内部 SRAM 栈；失败则尝试 PSRAM 栈（CONFIG_FREERTOS_TASK_CREATE_ALLOW_EXT_MEM=y）。
     * ch_hold 任务不做 DMA、不做 SPI 传输，PSRAM 栈完全安全，延迟可忽略。 */
    BaseType_t ok = xTaskCreate(channel_hold_task, "ch_hold",
                CH_HOLD_TASK_STACK, NULL, CH_HOLD_TASK_PRIO, &s_hold_task_handle);
    if (ok == pdPASS) {
        json_debug("RID_SNIFF", "ch_hold task created on internal SRAM (OK)");
    }
    if (ok != pdPASS) {
        snprintf(msg, sizeof(msg),
                 "internal SRAM task create failed (free=%u), trying PSRAM stack",
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
        json_warning("RID_SNIFF", msg);

        ok = xTaskCreateWithCaps(channel_hold_task, "ch_hold",
                CH_HOLD_TASK_STACK, NULL, CH_HOLD_TASK_PRIO,
                &s_hold_task_handle, MALLOC_CAP_SPIRAM);
        if (ok != pdPASS) {
            snprintf(msg, sizeof(msg),
                     "FATAL: ch_hold task create failed on both SRAM and PSRAM! free=%u internal=%u",
                     (unsigned)esp_get_free_heap_size(),
                     (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
            json_error("RID_SNIFF", msg);
            s_hold_task_handle = NULL;
        } else {
            json_debug("RID_SNIFF", "ch_hold task created on PSRAM stack (OK)");
        }
    }
}

void crid_sniffer_stop_channel_hold(void) {
    s_hold_should_stop = true;
}


