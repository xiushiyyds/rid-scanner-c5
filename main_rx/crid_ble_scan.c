/**
 * crid_ble_scan.c — BLE Remote ID 扫描实现 (detector v2.1.0)
 *
 * 使用 NimBLE host stack 扫描 BLE Legacy Advertising，
 * 匹配 ASTM F3411 / ASD-STAN 分配的 Service UUID 0xFFFA，
 * 提取 OpenDroneID / GB46750 message pack 并投递到 sniffer 队列。
 *
 * v2.1.0 架构变更（关键）：
 *   - 删除应用层 TDD 时分复用任务。TDD 频繁 cancel/restart 扫描会
 *     干扰 ESP-IDF PTA 硬件共存状态机，是 WiFi total_pkts=0 的根因。
 *   - BLE 扫描使用 scan_window < scan_interval 占空比，控制器自动在
 *     间隙调度 GATT 连接事件（与肩灯/手机原理一致）。
 *   - WiFi/BLE 共存由 CONFIG_ESP_COEX_SW_COEXIST_ENABLE=y 的 PTA
 *     硬件仲裁自动处理，WiFi sniffer 持续运行不被打断。
 *   - 扫描占空比：
 *     HIGH（未连接）：window=128*0.625=80ms, itvl=160*0.625=100ms = 80%
 *     LOW （已连接）：window=64 *0.625=40ms, itvl=160*0.625=100ms = 40%
 *     40% duty 下每 100ms 有 60ms 空隙，GATT 连接事件(30-50ms间隔,
 *     3-5ms时长)轻松调度，同时保持 40% BLE RID 捕获率。
 *
 * v2.1.0 新增：GB46750-2025 变长包透传。
 *   find_odid_service_data() 跳过 UUID(2)+AppCode(1)+MsgCounter(1)=4 字节
 *   后返回的 payload：
 *     - ASTM:  payload[0] = 0xF0/0xF1/0xF2（单条 25B 或 MessagePack）
 *     - GB46750: payload[0] = 0xFF（变长，header 6B + content）
 *   GB46750 包直接透传原始 payload，不包装 ASTM MessagePack 前缀。
 */

#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* NimBLE */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"

#include "crid_ble_scan.h"
#include "crid_sniffer.h"
#include "crid_rx_types.h"

static const char *TAG = "RID_BLE_SCAN";

/* ---- ASTM F3411 BLE Service UUID ---- */
#define ODID_BLE_SVC_UUID16     0xFFFA

/* ---- 扫描状态 ---- */
static bool s_scan_running = false;

/* 扫描允许开关（侦测板始终 true，保留供未来使用） */
static volatile bool s_scan_allowed = true;

/* ---- 扫描占空比模式 ---- */
typedef enum {
    SCAN_DUTY_HIGH,          /* v2.7.0: 40%，BLE RID 与 WiFi sniffer 平衡 */
    SCAN_DUTY_LOW,           /* 已连接：40%，GATT 连接事件稳定 */
    SCAN_DUTY_WIFI_LOCKED,   /* WiFi 目标锁定：10%，把空中时间让给 WiFi sniffer */
} scan_duty_t;

static volatile scan_duty_t s_scan_duty = SCAN_DUTY_HIGH;

/* ---- 前向声明 ---- */
static int ble_scan_gap_event(struct ble_gap_event *event, void *arg);
static void process_adv_report(const uint8_t *addr, const uint8_t *data, uint8_t data_len,
                               int8_t rssi, bool is_extended);

/* ================================================================
 * 从 Advertising data 中提取 ODID 单条消息 (Service Data UUID 0xFFFA)
 *
 * BLE Service Data 格式：
 *   [Len][Type=0x16][UUID_LO][UUID_HI][AppCode=0x0D][MsgCounter][Payload...]
 *
 * 返回 Payload 指针（MsgCounter 之后的第一个字节）：
 *   - ASTM:    0xF0/0xF1/0xF2 开头，单条 25B 或 MessagePack
 *   - GB46750: 0xFF 开头，变长
 * ================================================================ */
static const uint8_t *find_odid_service_data(const uint8_t *adv_data, uint8_t adv_len,
                                              uint8_t *out_msg_len) {
    uint8_t pos = 0;
    while (pos + 2 <= adv_len) {
        uint8_t field_len = adv_data[pos];
        if (field_len == 0) break;
        if (pos + 1 + field_len > adv_len) break;

        uint8_t ad_type = adv_data[pos + 1];
        const uint8_t *ad_data = &adv_data[pos + 2];
        uint8_t ad_data_len = field_len - 1;

        if (ad_type == 0x16 && ad_data_len >= 7) {
            uint16_t uuid = ad_data[0] | (ad_data[1] << 8);
            if (uuid == ODID_BLE_SVC_UUID16) {
                if (ad_data[2] != 0x0D) {
                    pos += 1 + field_len;
                    continue;
                }
                /* ad_data: [UUID_LO][UUID_HI][AppCode][MsgCounter][Payload...]
                 *          ^0        ^1        ^2        ^3           ^4
                 * 跳过 4 字节，返回 MsgCounter 之后的 payload。 */
                *out_msg_len = ad_data_len - 4;
                return ad_data + 4;
            }
        }

        pos += 1 + field_len;
    }
    return NULL;
}

/* ================================================================
 * 处理一条广播报告
 *
 * 支持三种 payload 格式：
 *   1. ASTM 单条 25B：包装成 MessagePack [0xF0,25,1,payload]
 *   2. ASTM MessagePack：0xF? + 25 + count + count×25 直通
 *   3. GB46750 变长包：0xFF 开头，原样透传给解析器
 * ================================================================ */
static void process_adv_report(const uint8_t *addr, const uint8_t *data, uint8_t data_len,
                               int8_t rssi, bool is_extended) {
    uint8_t msg_len = 0;
    const uint8_t *payload = find_odid_service_data(data, data_len, &msg_len);

    if (payload == NULL || msg_len == 0) {
        return;
    }

    const uint8_t *pack_data;
    uint8_t pack_len;
    uint8_t pack_buf[3 + 10 * 25];

    if (payload[0] == 0xFF) {
        /* ---- GB46750-2025 变长包 ----
         * payload 格式: [0xFF][Ver][DataLength][Flags(3B)][Content...]
         * 直接透传原始 payload，不做 ASTM 包装。
         * WiFi sniffer 路径投递的 payload 起点与此一致（都是 MsgCounter 之后），
         * parser_task 无需区分来源。 */
        if (msg_len < 6) return;  /* GB46750 header = Magic+Ver+Len+Flags = 6B */
        pack_data = payload;
        pack_len = msg_len;
    } else if (msg_len == 25) {
        /* ---- ASTM 单条消息：包装成 MessagePack ---- */
        pack_buf[0] = 0xF0;
        pack_buf[1] = 25;
        pack_buf[2] = 1;
        memcpy(pack_buf + 3, payload, 25);
        pack_data = pack_buf;
        pack_len = 28;
    } else if (msg_len >= 28 && (payload[0] >> 4) == 0xF &&
               payload[1] == 25 && payload[2] >= 1 && payload[2] <= 10 &&
               msg_len >= 3 + payload[2] * 25) {
        /* ---- ASTM MessagePack（多条） ---- */
        pack_data = payload;
        pack_len = 3 + payload[2] * 25;
    } else {
        return;
    }

    sniffer_msg_t msg;
    memset(&msg, 0, sizeof(msg));

    memcpy(msg.src_mac, addr, 6);
    msg.rssi = rssi;
    msg.channel = 0;
    msg.msg_type = MSG_TYPE_RID;
    msg.is_rid = true;

    msg.oui[0] = 0xFF;
    msg.oui[1] = 0xFF;
    msg.oui[2] = 0xFA;
    msg.oui_type = 0;

    if (is_extended) {
        msg.channel |= 0x80;
    }

    memcpy(msg.data, pack_data, pack_len);
    msg.data_len = pack_len;

    QueueHandle_t queue = crid_sniffer_get_queue();
    if (queue != NULL) {
        if (xQueueSend(queue, &msg, 0) != pdTRUE) {
            ESP_LOGD(TAG, "BLE scan queue full");
        }
    }
}

/* ================================================================
 * NimBLE GAP 事件回调
 * ================================================================ */
static int ble_scan_gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
    case BLE_GAP_EVENT_DISC: {
        struct ble_gap_disc_desc *desc = &event->disc;
        process_adv_report(desc->addr.val,
                           desc->data,
                           desc->length_data,
                           desc->rssi,
                           false);
        break;
    }

#if MYNEWT_VAL(BLE_EXT_ADV)
    case BLE_GAP_EVENT_EXT_DISC: {
        struct ble_gap_ext_disc_desc *desc = &event->ext_disc;
        process_adv_report(desc->addr.val,
                           desc->data,
                           desc->length_data,
                           desc->rssi,
                           true);
        break;
    }
#endif

    case BLE_GAP_EVENT_DISC_COMPLETE:
        ESP_LOGD(TAG, "Discovery complete");
        break;

    default:
        break;
    }
    return 0;
}

/* ================================================================
 * 启动扫描（Legacy passive）
 *
 * v2.1.0: 扫描参数根据 s_scan_duty 动态设置。
 *   HIGH: itvl=160(100ms), window=128(80ms) = 80% 占空比
 * v2.7.0: BLE 空闲占空比从 80% 降到 40%。实测未连接手机时 BLE 扫描抢占
 *         过多 WiFi 空中时间，导致国标 RID Beacon 接收率极低；40% 在保留
 *         BLE RID 扫描能力的同时，把 WiFi 接收时间提升约 3 倍。
 * 占空比：
 *   HIGH（空闲/未连接）：40%，window=40ms / itvl=100ms
 *   LOW （已连接）：     40%，window=40ms / itvl=100ms
 *   WIFI_LOCKED（WiFi目标锁定）：10%，window=10ms / itvl=100ms
 *
 * window < itvl 确保控制器在扫描间隙有时间处理 GATT 连接事件。
 * 这是 BLE 协议标准设计，与肩灯/手机同时扫描+连接的原理相同。
 * ================================================================ */
static const char *duty_name(scan_duty_t d) {
    switch (d) {
    case SCAN_DUTY_HIGH:        return "BALANCED 40%";
    case SCAN_DUTY_LOW:         return "LOW 40%";
    case SCAN_DUTY_WIFI_LOCKED: return "WIFI_LOCKED 10%";
    }
    return "?";
}

static uint16_t duty_window(scan_duty_t d) {
    switch (d) {
    case SCAN_DUTY_HIGH:        return 64;   /* 40ms, v2.7.0: 80ms -> 40ms */
    case SCAN_DUTY_LOW:         return 64;   /* 40ms  */
    case SCAN_DUTY_WIFI_LOCKED: return 16;   /* 10ms  */
    }
    return 128;
}

static int start_scan(void) {
    struct ble_gap_disc_params disc_params = {0};
    int rc;

#if MYNEWT_VAL(BLE_EXT_ADV)
    /* EXT_ADV=y 路径（当前 sdkconfig 关闭了 EXT_ADV，不会走到这里） */
    struct ble_gap_ext_disc_params uncoded_params = {0};
    uncoded_params.itvl = 160;
    uncoded_params.window = duty_window(s_scan_duty);
    uncoded_params.passive = 1;

    rc = ble_gap_ext_disc(BLE_OWN_ADDR_PUBLIC,
                          0, 0, 0, 0, 0,
                          &uncoded_params, NULL,
                          ble_scan_gap_event, NULL);
    if (rc == 0) {
        ESP_LOGI(TAG, "BLE Extended scan started (duty=%s)", duty_name(s_scan_duty));
        return 0;
    }
    ESP_LOGW(TAG, "Extended scan failed (%d), falling back to legacy", rc);
#else
    ESP_LOGD(TAG, "BLE_EXT_ADV=n, using legacy scan");
#endif

    memset(&disc_params, 0, sizeof(disc_params));
    disc_params.itvl = 160;                              /* 100ms */
    disc_params.window = duty_window(s_scan_duty);
    ESP_LOGI(TAG, "BLE scan started (duty=%s, window=%ums/itvl=100ms)",
             duty_name(s_scan_duty),
             (unsigned)(disc_params.window * 10 / 16));
    disc_params.filter_policy = 0;
    disc_params.limited = 0;
    disc_params.passive = 1;
    disc_params.filter_duplicates = 0;

    rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER,
                      &disc_params, ble_scan_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to start BLE scan: %d", rc);
        return rc;
    }

    return 0;
}

/* ================================================================
 * 公开接口
 * ================================================================ */

esp_err_t crid_ble_scan_init(void) {
    ESP_LOGI(TAG, "BLE RID scanner init (v2.1.0, PTA coexist, no TDD)");
    s_scan_running = false;
    return ESP_OK;
}

esp_err_t crid_ble_scan_start(void) {
    if (!s_scan_allowed) {
        ESP_LOGI(TAG, "Scan start suppressed (allowed=%d)", s_scan_allowed);
        s_scan_running = false;
        return ESP_ERR_INVALID_STATE;
    }

    if (s_scan_running) {
        ESP_LOGW(TAG, "Scan already running");
        return ESP_OK;
    }

    if (!ble_hs_synced()) {
        ESP_LOGW(TAG, "NimBLE host not yet synced, retry in 1s");
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (!ble_hs_synced()) {
            ESP_LOGE(TAG, "NimBLE host not synced");
            return ESP_ERR_INVALID_STATE;
        }
    }

    int rc = start_scan();
    if (rc != 0) {
        return ESP_FAIL;
    }

    s_scan_running = true;
    return ESP_OK;
}

void crid_ble_scan_stop(void) {
    if (s_scan_running) {
        ble_gap_disc_cancel();
        s_scan_running = false;
    }
    ESP_LOGI(TAG, "BLE scan stopped");
}

bool crid_ble_scan_is_running(void) {
    return s_scan_running;
}

void crid_ble_scan_set_allowed(bool allowed) {
    s_scan_allowed = allowed;
    if (!allowed && s_scan_running) {
        crid_ble_scan_stop();
    }
}

/* ================================================================
 * 占空比切换
 *
 * 注意：这些函数只修改 s_scan_duty 标志。如果扫描正在运行，
 * 调用者（通常是独立 task，不是 NimBLE host task）需要 stop + start
 * 才能让新参数生效。set_duty_high/low 本身不做 cancel/restart，
 * 因为不能在 NimBLE host task 上下文做阻塞操作。
 *
 * 典型用法（在独立 task 中）：
 *   crid_ble_scan_set_duty_low();
 *   crid_ble_scan_stop();
 *   vTaskDelay(pdMS_TO_TICKS(50));
 *   crid_ble_scan_start();
 * ================================================================ */
void crid_ble_scan_set_duty_high(void) {
    if (s_scan_duty == SCAN_DUTY_HIGH) return;
    s_scan_duty = SCAN_DUTY_HIGH;
    ESP_LOGI(TAG, "Scan duty -> HIGH/BALANCED (40% window=40ms/itvl=100ms)");
}

void crid_ble_scan_set_duty_low(void) {
    if (s_scan_duty == SCAN_DUTY_LOW) return;
    s_scan_duty = SCAN_DUTY_LOW;
    ESP_LOGI(TAG, "Scan duty -> LOW (40% window=40ms/itvl=100ms, GATT coexist)");
}

void crid_ble_scan_set_duty_wifi_locked(void) {
    if (s_scan_duty == SCAN_DUTY_WIFI_LOCKED) return;
    s_scan_duty = SCAN_DUTY_WIFI_LOCKED;
    ESP_LOGI(TAG, "Scan duty -> WIFI_LOCKED (10% window=10ms/itvl=100ms, WiFi priority)");
}
