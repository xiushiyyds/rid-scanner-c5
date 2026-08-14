/**
 * crid_ble_scan.c — BLE Remote ID 扫描实现
 *
 * 使用 NimBLE host stack 扫描 BLE Legacy 和 Extended Advertising，
 * 匹配 ASTM F3411 / ASD-STAN 分配的 Service UUID 0xFFFA，
 * 提取 OpenDroneID message pack 并投递到 sniffer 队列。
 *
 * 支持：
 *   - BLE Legacy Advertising (BT4)：31 字节 payload
 *   - BLE Extended Advertising (BT5)：最大 254 字节，支持 Coded PHY (Long Range)
 *
 * v1.9.2: 按 ESP-IDF v5.4 / NimBLE 真实头文件修正全部 API
 *         (回调返回 int、BLE_OWN_ADDR_PUBLIC、ext_disc 字段、
 *          disc_desc 无 channel 字段、移除不存在的 ble_gap_disc_flags)
 * v1.8:   初版
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
/* 0xFFFA (16-bit) in little-endian adv data: 0xFA, 0xFF */
#define ODID_BLE_SVC_UUID16     0xFFFA

/* ---- 扫描状态 ---- */
static bool s_scan_running = false;
static uint8_t s_ext_adv_type = 0;  /* 0=Legacy, 1=Extended */

/* ---- 前向声明（NimBLE 回调必须返回 int） ---- */
static int ble_scan_gap_event(struct ble_gap_event *event, void *arg);
static void process_adv_report(const uint8_t *addr, const uint8_t *data, uint8_t data_len,
                               int8_t rssi, bool is_extended);

/* ================================================================
 * 从 Advertising data 中提取 ODID 单条消息 (Service Data UUID 0xFFFA)
 *
 * BLE Advertising data 格式:
 *   [length(1)] [AD Type(1)] [AD Data(length-1)]
 *   AD Type 0x16 = Service Data - 16-bit UUID
 *
 * ASTM F3411 BLE Service Data 布局 (AD Data 部分):
 *   [UUID_LE: FA FF] [App Code: 0x0D] [message_counter: 1B] [25-byte ODID message]
 *
 * 每条 BLE 广播只携带一条 25 字节 ODID 消息（不是 MessagePack），
 * message_counter 用于接收端区分已接收的消息类型。
 *
 * 返回值: 指向 25 字节单条消息的指针（跳过 UUID + App Code + counter）
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
        uint8_t ad_data_len = field_len - 1;  /* 减去 AD Type 自身 */

        /* AD Type 0x16 = Service Data (16-bit UUID) */
        if (ad_type == 0x16 && ad_data_len >= 7) {
            uint16_t uuid = ad_data[0] | (ad_data[1] << 8);  /* little-endian */
            if (uuid == ODID_BLE_SVC_UUID16) {
                /* 校验 App Code = 0x0D (Drone ID service data marker) */
                if (ad_data[2] != 0x0D) {
                    pos += 1 + field_len;
                    continue;
                }
                /* 跳过 UUID(2) + App Code(1) + message_counter(1) = 4 字节 */
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
 * BLE 广播携带的是单条 25 字节 ODID 消息（非 MessagePack）。
 * 为复用 WiFi 路径的 MessagePack 解析器，将单条消息包装成
 * 1-MsgPack：[0xF0][0x19][0x01][25-byte message]
 * ================================================================ */
static void process_adv_report(const uint8_t *addr, const uint8_t *data, uint8_t data_len,
                               int8_t rssi, bool is_extended) {
    uint8_t msg_len = 0;
    const uint8_t *odid_msg = find_odid_service_data(data, data_len, &msg_len);

    if (odid_msg == NULL || msg_len == 0) {
        return;  /* 不是 ODID 广播 */
    }

    /* ASTM F3411 BLE: 每条广播恰好 25 字节单消息 */
    if (msg_len != 25) {
        ESP_LOGD(TAG, "Unexpected BLE ODID msg_len=%d (expect 25)", msg_len);
        return;
    }

    /* 将单条消息包装成 MessagePack 供 parser 统一处理 */
    uint8_t pack_buf[28];  /* 3 字节头 + 25 字节消息 */
    pack_buf[0] = 0xF0;    /* MessageType=PACKED(0xF), ProtoVersion=ASTM(0x0) */
    pack_buf[1] = 25;      /* SingleMessageSize */
    pack_buf[2] = 1;       /* MsgPackSize = 1 */
    memcpy(pack_buf + 3, odid_msg, 25);

    /* 构造 sniffer 消息，复用现有解析队列 */
    sniffer_msg_t msg;
    memset(&msg, 0, sizeof(msg));

    /* BLE 没有传统 MAC 概念，用蓝牙地址作为标识 */
    memcpy(msg.src_mac, addr, 6);
    msg.rssi = rssi;
    msg.channel = 0;   /* NimBLE disc_desc 不提供信道，填 0 */
    msg.msg_type = MSG_TYPE_RID;
    msg.is_rid = true;

    /* 标记来源为 BLE（使用 ASTM 分配的 BLE OUI 0xFFFA，
     * 这样 GET_RID_TRANSPORT() 会正确返回 RID_TRANSPORT_BLUETOOTH_LEGACY） */
    msg.oui[0] = 0xFF;
    msg.oui[1] = 0xFF;
    msg.oui[2] = 0xFA;
    msg.oui_type = 0;

    /* BLE Extended (Coded PHY/Long Range) 标记 LR 子类型 */
    if (is_extended) {
        msg.channel |= 0x80;  /* 高位标记为 LR（channel 实际值 0-39 不受影响） */
    }

    /* 复制包装后的 MessagePack (28 字节) */
    memcpy(msg.data, pack_buf, 28);
    msg.data_len = 28;

    ESP_LOGD(TAG, "BLE ODID: rssi=%d ext=%d", rssi, is_extended);

    /* 投递到 sniffer 队列（非 ISR 上下文，用 xQueueSend） */
    QueueHandle_t queue = crid_sniffer_get_queue();
    if (queue != NULL) {
        if (xQueueSend(queue, &msg, 0) != pdTRUE) {
            ESP_LOGD(TAG, "BLE scan queue full");
        }
    }
}

/* ================================================================
 * NimBLE GAP 事件回调
 * 注意：NimBLE 回调签名为 int (*)(struct ble_gap_event*, void*)，必须返回 0
 * ================================================================ */
static int ble_scan_gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
    case BLE_GAP_EVENT_DISC: {
        /* Legacy advertising report
         * struct ble_gap_disc_desc { event_type, length_data, addr, rssi,
         *                            data, direct_addr };  无 channel 字段 */
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
        /* Extended advertising report (BT5 Coded PHY / Long Range)
         * struct ble_gap_ext_disc_desc { props, data_status, legacy_event_type,
         *   addr, rssi, tx_power, sid, prim_phy, sec_phy, length_data, data, ... }
         * 无 channel 字段；sec_phy=BLE_HCI_LE_PHY_CODED(3) 表示 Long Range */
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
 * 启动扫描
 * ================================================================ */
static int start_scan(void) {
    /*
     * lcdfix10：优先使用 Legacy 扫描，提高与手机 BLE 广播的兼容性。
     * Extended scanning 在某些控制器版本上会与 legacy advertising
     * 产生资源争用，导致本机广播不可见。先确保 Legacy 稳定，后续有
     * Long Range 需求再切换。
     */
    /* lcdfix17: BLE 扫描占空比调到 50%。
     * lcdfix16 的 30% (itvl=160/window=48) 在某些环境下窗口太短，
     * 37/38/39 三个广告信道轮询一周容易错过。改为 itvl=96(60ms)/window=48(30ms)，
     * 50% 占空比，既给 WiFi 留足空口收 Beacon，又保证 BLE 广告接收稳定。 */
    struct ble_gap_disc_params disc_params = {0};
    int rc;

    memset(&disc_params, 0, sizeof(disc_params));
    disc_params.itvl = 96;            /* 96 × 0.625ms = 60ms */
    disc_params.window = 48;          /* 48 × 0.625ms = 30ms */
    disc_params.filter_policy = 0;    /* 接收所有 */
    disc_params.limited = 0;
    disc_params.passive = 1;          /* passive scan，不发 scan request */
    disc_params.filter_duplicates = 0;

    rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER,
                      &disc_params, ble_scan_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to start BLE legacy scan: %d", rc);
        return rc;
    }

    s_ext_adv_type = 0;
    ESP_LOGI(TAG, "BLE Legacy scan started");
    return 0;
}

/* ================================================================
 * 公开接口
 * ================================================================ */

esp_err_t crid_ble_scan_init(void) {
    ESP_LOGI(TAG, "BLE RID scanner init");
    s_scan_running = false;
    return ESP_OK;
}

esp_err_t crid_ble_scan_start(void) {
    if (s_scan_running) {
        ESP_LOGW(TAG, "Scan already running");
        return ESP_OK;
    }

    /* 确保 NimBLE host 已同步 */
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
    if (!s_scan_running) return;

    ble_gap_disc_cancel();
    s_scan_running = false;
    ESP_LOGI(TAG, "BLE scan stopped");
}

bool crid_ble_scan_is_running(void) {
    return s_scan_running;
}
