/**
 * crid_sniffer.c — Wi-Fi Sniffer 模块 (稳定版)
 * 
 * 修复：
 *   - ISR 中使用 xQueueSendFromISR
 *   - decodeMessagePack 调用修正（正确指针）
 *   - 修正 ODID payload 偏移（跳过 Message Counter）
 *   - IE 遍历结构化跳转
 *   - 移除 ISR 中的日志
 *   - 使用 xTaskGetTickCountFromISR
 *   - 消除 type-limits 警告（直接赋值）
 */

#include <string.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi_types.h"

/* sniffer 使用 WIFI_MODE_NULL + promiscuous，不需要 STA/AP netif，
 * 但 esp_wifi_init 要求 default event loop 和 netif 已初始化（app_main 完成）。 */
#include "freertos/task.h"
#include "crid_sniffer.h"
#include "crid_json.h"
#include "opendroneid.h"
#include "dji_droneid.h"

/* lcdfix16: 跟踪 sniffer 侧 esp_wifi_init 状态。
 * ESP-IDF v5.5 中 esp_wifi_init 重复调用不返回 ESP_ERR_WIFI_INITED
 * （该宏在部分版本中根本不存在），用静态标志更可靠。 */
static bool s_sniffer_wifi_inited = false;

/* ================================================================
 MAC 地址调试过滤器
 ================================================================ */
#define DEBUG_MAC_FILTER_ENABLED    0   
#ifndef DEBUG_MAC_FILTER_ENABLED
static const uint8_t debug_target_mac[6] = {
    0x24, 0x0A, 0xC4, 0x12, 0x34, 0x56  
};
#define DEBUG_MAC_PREFIX_LEN        6   
#endif

/* ---- 模块内部状态 ---- */
static QueueHandle_t g_sniffer_queue = NULL;
static sniffer_stats_t g_stats;
int8_t g_rid_min_rssi = RID_MIN_RSSI_DEFAULT;
static const char *TAG = "WIFI_SNIFFER";

/* ---- 固定信道（需在项目配置中定义） ---- */
#ifndef FIXED_CHANNEL
#define FIXED_CHANNEL 6
#endif

/* ================================================================
 Debug 辅助 (仅在任务/初始化中使用)
 ================================================================ */
#if PARSER_DEBUG_HEX_DUMP
static void hex_dump(const char *tag, const char *prefix, const uint8_t *data, uint8_t len) {
    char line[128];
    int pos = 0;
    pos += snprintf(line + pos, sizeof(line) - pos, "%s [%u]  ", prefix, len);
    for (int i = 0; i < len; i++) {
        pos += snprintf(line + pos, sizeof(line) - pos, "%02X ", data[i]);
        if ((i + 1) % 16 == 0 && i + 1 < len) {
            ESP_LOGI(tag, "%s", line);
            pos = snprintf(line, sizeof(line), "        ");
        }
    }
    if (pos > 0) ESP_LOGI(tag, "%s", line);
}
#endif

/* ================================================================
 ISR 回调
 ================================================================ */
static void wifi_sniffer_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
    g_stats.total_packets++;
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
            if (len >= 5) {  // OUI(3) + Type(1) + Counter(1) 至少
                uint8_t oui0 = ie_ptr[i + 2];
                uint8_t oui1 = ie_ptr[i + 3];
                uint8_t oui2 = ie_ptr[i + 4];
                uint8_t oui_type = ie_ptr[i + 5];

                if (IS_RID_OUI(oui0, oui1, oui2) && oui_type == 0x0D) {
                    has_rid_ie = true;
                    g_stats.rid_detections++;

                    // 跳过弱信号
                    if (pkt->rx_ctrl.rssi < g_rid_min_rssi) {
                        break;
                    }

                    // 【修正】跳过 ID+Len+OUI+Type+Counter = 7 字节
                    uint8_t *odid_payload = &ie_ptr[i + 7];
                    // 有效数据长度 = Len - 5 (OUI+Type+Counter)
                    uint8_t odid_len = len - 5;

                    // 最小长度校验：MessagePack header(3) + 至少1条消息(25) = 28 字节
                    if (odid_len < 28) {
                        break;
                    }

                    // 不在 ISR 中做解码（decodeMessagePack 计算量大，
                    //  memset + 解码耗时可能超过看门狗限制），
                    //  直接投递原始 payload 到队列，由 parser_task 解码。
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

                        // 复制 ODID 原始数据（odid_len 最大 250，msg.data 至少 256 字节，安全）
                        uint16_t copy_len = odid_len;
                        memcpy(msg.data, odid_payload, copy_len);
                        msg.data_len = copy_len;

                        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                        if (xQueueSendFromISR(g_sniffer_queue, &msg, &xHigherPriorityTaskWoken) != pdTRUE) {
                            g_stats.queue_overflows++;
                        }
                        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
                    }
                    break;  // 一个帧只应有一个 RID IE
                } else if (IS_DJI_OUI(oui0, oui1, oui2)) {
                    // DJI DroneID 私有协议检测
                    g_stats.dji_detections++;
                    
                    // 跳过弱信号
                    if (pkt->rx_ctrl.rssi < g_rid_min_rssi) {
                        break;
                    }
                    
                    // DJI payload: 跳过 IE ID(1) + Len(1) + OUI(3) + OUI_Type(1) = 6 字节
                    // payload 从 common header (0x58,0x62,0x13) 开始
                    uint8_t *dji_payload = &ie_ptr[i + 6];
                    // 有效数据长度 = IE Len - OUI(3) - OUI_Type(1)
                    uint8_t dji_len = len - 4;
                    
                    if (dji_len >= 4) {  // 至少 common header(3) + subcommand(1)
                        sniffer_msg_t msg;
                        memset(&msg, 0, sizeof(msg));
                        
                        memcpy(msg.src_mac, hdr->addr2, 6);
                        msg.rssi = pkt->rx_ctrl.rssi;
                        msg.channel = pkt->rx_ctrl.channel;
                        msg.timestamp_ms = (uint32_t)(xTaskGetTickCountFromISR() * portTICK_PERIOD_MS);
                        msg.is_rid = false;
                        msg.msg_type = MSG_TYPE_DJI_DRONEID;
                        msg.oui[0] = oui0; msg.oui[1] = oui1; msg.oui[2] = oui2;
                        msg.oui_type = 0xFF;  // DJI 特殊标记
                        msg.has_vendor_ie = true;
                        
                        /* dji_len 是 uint8_t，最大 255，msg.data[256] 可完整容纳 */
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

/* ---- 信道跳频 + WiFi/BLE 分时调度 (TDD) ----
 * lcdfix30: ESP32-C5 单射频，BLE 扫描和 WiFi sniffer 不能同时有效工作。
 * 改为分时：WiFi 抓包阶段停 BLE 扫描，BLE 扫描阶段暂停 WiFi promiscuous。
 * WiFi 占 ~87% 时间，BLE 占 ~13%，两者都能工作且互不抢射频。
 *
 * 信道偏置：ch6 长驻，ch1/ch11 快速扫。 */
static const uint8_t SCAN_CHANNELS[] = {6, 1, 6, 11};
static const uint16_t CHANNEL_DWELL_MS_ARR[] = {2000, 250, 2000, 250};
#define SCAN_CHANNEL_COUNT (sizeof(SCAN_CHANNELS) / sizeof(SCAN_CHANNELS[0]))
static volatile uint8_t s_current_channel = 6;

/* lcdfix30: TDD 阶段 */
typedef enum {
    TDD_PHASE_WIFI = 0,
    TDD_PHASE_BLE  = 1,
} tdd_phase_t;
static volatile tdd_phase_t s_tdd_phase = TDD_PHASE_WIFI;

/* TDD 回调：由 app_main 注册，用于暂停/恢复 BLE 扫描 */
typedef void (*tdd_ble_cb_t)(void);
static tdd_ble_cb_t s_ble_pause_cb  = NULL;
static tdd_ble_cb_t s_ble_resume_cb = NULL;

/* lcdfix30: TDD 通过回调控制 BLE 扫描，
 * 不直接调用 scan_set_allowed（那是 app_main 的页面互斥开关）。 */
void crid_sniffer_set_ble_tdd_callbacks(void (*pause_cb)(void), void (*resume_cb)(void)) {
    s_ble_pause_cb = pause_cb;
    s_ble_resume_cb = resume_cb;
}

uint8_t crid_sniffer_get_current_channel(void) {
    return s_current_channel;
}

/* lcdfix30: 获取当前 TDD 阶段（供 LCD 显示） */
int crid_sniffer_get_tdd_phase(void) {
    return (int)s_tdd_phase;
}

/* ---- 信道跳频任务 ----
 * 启动后先停 2 秒等 WiFi 控制器稳定，然后在 1/6/11 间轮询。
 * lcdfix15: 增加 stop flag，deinit 时必须停掉任务，
 * 否则切到模拟AP模式后它还在 esp_wifi_set_channel 干扰AP。 */
static volatile bool s_hold_should_stop = false;
static TaskHandle_t s_hold_task_handle = NULL;

static void channel_hold_task(void *pvParameter) {
    char msg[64];
    snprintf(msg, sizeof(msg), "TDD: WiFi 2s / BLE 300ms (WiFi 87%%)");
    json_debug("RID_SNIFF", msg);

    vTaskDelay(pdMS_TO_TICKS(2000));

    uint8_t idx = 0;
    for (uint8_t i = 0; i < SCAN_CHANNEL_COUNT; i++) {
        if (SCAN_CHANNELS[i] == 6) { idx = i; break; }
    }

    /* lcdfix30: 诊断统计 */
    uint32_t diag_counter = 0;
    uint32_t last_total = 0, last_rid = 0, last_beacon = 0;

    while (!s_hold_should_stop) {
        /* === WiFi 阶段：BLE 扫描停止，WiFi 独占射频 === */
        s_tdd_phase = TDD_PHASE_WIFI;
        if (s_ble_pause_cb) s_ble_pause_cb();
        /* 确保 promiscuous 开着（BLE 阶段可能关了） */
        esp_wifi_set_promiscuous(true);

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

        /* === BLE 阶段：暂停 WiFi promiscuous，启动 BLE 扫描 === */
        s_tdd_phase = TDD_PHASE_BLE;
        esp_wifi_set_promiscuous(false);
        if (s_ble_resume_cb) s_ble_resume_cb();

        /* BLE 扫描 300ms */
        for (int i = 0; i < 3 && !s_hold_should_stop; i++) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        if (s_hold_should_stop) break;

        /* lcdfix30: 每 ~10 轮打印一次诊断 */
        diag_counter++;
        if (diag_counter % 5 == 0) {
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

    /* 退出时恢复 WiFi promiscuous 并停 BLE 扫描 */
    s_tdd_phase = TDD_PHASE_WIFI;
    if (s_ble_pause_cb) s_ble_pause_cb();
    esp_wifi_set_promiscuous(true);

    s_hold_should_stop = false;
    s_hold_task_handle = NULL;
    vTaskDelete(NULL);
}

/* ---- 公开接口 ---- */
QueueHandle_t crid_sniffer_get_queue(void) {
    return g_sniffer_queue;
}

sniffer_stats_t *crid_sniffer_get_stats(void) {
    return &g_stats;
}

esp_err_t crid_sniffer_init(void) {
    memset(&g_stats, 0, sizeof(g_stats));
    char err[64];
    esp_err_t ret;

    ESP_LOGI(TAG, "Initializing Wi-Fi Sniffer...");

    /* 复用已有队列（模式切换时 parser_task 仍持有旧句柄） */
    if (g_sniffer_queue == NULL) {
        g_sniffer_queue = xQueueCreate(SNIFFER_QUEUE_SIZE, sizeof(sniffer_msg_t));
    } else {
        /* 清空旧队列中可能残留的数据 */
        sniffer_msg_t discard;
        while (xQueueReceive(g_sniffer_queue, &discard, 0) == pdTRUE) {}
    }
    if (g_sniffer_queue == NULL) {
        json_error("RID_SNIFF", "Failed to create sniffer queue!");
        return ESP_ERR_NO_MEM;
    }

    /* lcdfix16: 用静态标志跟踪 WiFi 初始化状态，
     * 避免引用不存在的 ESP_ERR_WIFI_INITED 宏导致编译失败。 */
    if (!s_sniffer_wifi_inited) {
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ret = esp_wifi_init(&cfg);
        if (ret != ESP_OK) {
            /* 模式切换路径下 controller 可能仍处于 init 状态，
             * 记录警告但继续 set_mode/start，通常能恢复。 */
            ESP_LOGW(TAG, "esp_wifi_init returned %s (reusing existing driver)",
                     esp_err_to_name(ret));
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
        .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT  // 只抓管理帧（Beacon/Probe），跳过 DATA/CTRL
    };
    esp_wifi_set_promiscuous_filter(&filter);

    ret = esp_wifi_set_promiscuous(true);
    if (ret != ESP_OK) {
        snprintf(err, sizeof(err), "Promiscuous mode failed: %s", esp_err_to_name(ret));
        json_error("RID_SNIFF", err);
        return ret;
    }

    /* lcdfix19: 不锁协议！
     * lcdfix17 曾加 esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B)，
     * 只接收 11b DSSS 帧。但厂家 RID 测试模块的 WiFi 广播可能走 11g/11n
     * OFDM 调制，锁 11b 会直接把这些帧在控制器层过滤掉，导致侦测 0 目标。
     * v1.9.7（上午能收到的版本）没有这行，恢复默认（11b/g/n 全收）。
     * 关闭省电，确保射频持续接收。 */
    esp_wifi_set_ps(WIFI_PS_NONE);

    ret = esp_wifi_set_channel(FIXED_CHANNEL, WIFI_SECOND_CHAN_NONE);
    if (ret != ESP_OK) {
        snprintf(err, sizeof(err), "Set channel failed: %s", esp_err_to_name(ret));
        json_error("RID_SNIFF", err);
        return ret;
    }

    snprintf(err, sizeof(err), "Wi-Fi monitor mode enabled, promiscuous ON, channel %d", FIXED_CHANNEL);
    json_debug("RID_SNIFF", err);

    return ESP_OK;
}

void crid_sniffer_start_channel_hold(void) {
    if (s_hold_task_handle != NULL) {
        /* 任务可能还在退出中，等它完成 */
        for (int i = 0; i < 20 && s_hold_task_handle != NULL; i++) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    s_hold_should_stop = false;
    xTaskCreate(channel_hold_task, "ch_hold",
                CH_HOLD_TASK_STACK, NULL, CH_HOLD_TASK_PRIO, &s_hold_task_handle);
}

/* lcdfix15: 停止信道保持任务（切到模拟模式前必须调用） */
void crid_sniffer_stop_channel_hold(void) {
    s_hold_should_stop = true;
}

/**
 * v1.3: 停止 sniffer 并释放 Wi-Fi 资源
 * 用于模式切换到模拟模式前调用。
 */
esp_err_t crid_sniffer_deinit(void) {
    ESP_LOGI(TAG, "Stopping sniffer and releasing Wi-Fi...");

    /* lcdfix15: 先停信道保持任务，否则它在 wifi stop/deinit 后
     * 还在 esp_wifi_set_channel 干扰模拟器 AP 模式 */
    crid_sniffer_stop_channel_hold();
    vTaskDelay(pdMS_TO_TICKS(200));

    esp_err_t ret = esp_wifi_set_promiscuous(false);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "set_promiscuous(false) failed: %s", esp_err_to_name(ret));
    }

    /* 停止 Wi-Fi */
    ret = esp_wifi_stop();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "esp_wifi_stop failed: %s", esp_err_to_name(ret));
    }

    /* 反初始化 Wi-Fi 驱动 */
    ret = esp_wifi_deinit();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "esp_wifi_deinit failed: %s", esp_err_to_name(ret));
    }
    /* lcdfix16: 复位 WiFi 初始化标志，下次 sniffer init 重新 esp_wifi_init。 */
    s_sniffer_wifi_inited = false;

    ESP_LOGI(TAG, "Sniffer stopped, Wi-Fi released");
    return ESP_OK;
}