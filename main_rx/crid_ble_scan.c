/**
 * crid_ble_scan.c — BLE Remote ID 扫描实现 (detector variant v2.0.0)
 *
 * 使用 NimBLE host stack 扫描 BLE Legacy 和 Extended Advertising，
 * 匹配 ASTM F3411 / ASD-STAN 分配的 Service UUID 0xFFFA，
 * 提取 OpenDroneID message pack 并投递到 sniffer 队列。
 *
 * 纯侦测板：BLE 连续扫描，无 TDD 分时。
 * 扫描占空比 10%（window=12.5ms / itvl=125ms），给 WiFi sniffer 留空口。
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
static uint8_t s_ext_adv_type = 0;  /* 0=Legacy, 1=Extended */

/* 扫描允许开关（侦测板始终 true，保留供未来使用） */
static volatile bool s_scan_allowed = true;

/* ---- 前向声明 ---- */
static int ble_scan_gap_event(struct ble_gap_event *event, void *arg);
static void process_adv_report(const uint8_t *addr, const uint8_t *data, uint8_t data_len,
                               int8_t rssi, bool is_extended);

/* ================================================================
 * 从 Advertising data 中提取 ODID 单条消息 (Service Data UUID 0xFFFA)
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

    if (msg_len == 25) {
        pack_buf[0] = 0xF0;
        pack_buf[1] = 25;
        pack_buf[2] = 1;
        memcpy(pack_buf + 3, payload, 25);
        pack_data = pack_buf;
        pack_len = 28;
    } else if (msg_len >= 28 && (payload[0] >> 4) == 0xF &&
               payload[1] == 25 && payload[2] >= 1 && payload[2] <= 10 &&
               msg_len >= 3 + payload[2] * 25) {
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
 * 启动扫描
 * ================================================================ */
static int start_scan(void) {
    struct ble_gap_disc_params disc_params = {0};
    int rc;

#if MYNEWT_VAL(BLE_EXT_ADV)
    struct ble_gap_ext_disc_params uncoded_params = {0};
    struct ble_gap_ext_disc_params coded_params = {0};

    uncoded_params.itvl = 200;    /* 160 × 0.625ms = 100ms */
    uncoded_params.window = 20;   /* 80 × 0.625ms = 50ms (50% duty) */
    uncoded_params.passive = 1;

    coded_params.itvl = 200;
    coded_params.window = 20;
    coded_params.passive = 1;

    rc = ble_gap_ext_disc(BLE_OWN_ADDR_PUBLIC,
                          0, 0, 0, 0, 0,
                          &uncoded_params,
                          &coded_params,
                          ble_scan_gap_event, NULL);

    if (rc == 0) {
        s_ext_adv_type = 1;
        ESP_LOGI(TAG, "BLE Extended scan started (1M + Coded PHY, 10%% duty)");
        return 0;
    }

    ESP_LOGW(TAG, "Extended scan failed (%d), falling back to legacy", rc);
#else
    ESP_LOGI(TAG, "BLE_EXT_ADV not enabled, using legacy scan");
#endif

    /* Legacy 扫描：10% 占空比，给 WiFi sniffer 留空口 */
    memset(&disc_params, 0, sizeof(disc_params));
    disc_params.itvl = 160;           /* 160 × 0.625ms = 100ms */
    disc_params.window = 80;          /* 80 × 0.625ms = 50ms (50% duty) */
    disc_params.filter_policy = 0;
    disc_params.limited = 0;
    disc_params.passive = 1;
    disc_params.filter_duplicates = 0;

    rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER,
                      &disc_params, ble_scan_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to start BLE legacy scan: %d", rc);
        return rc;
    }

    s_ext_adv_type = 0;
    ESP_LOGI(TAG, "BLE Legacy scan started (50%% duty, passive)");
    return 0;
}

/* ================================================================
 * 公开接口
 * ================================================================ */

esp_err_t crid_ble_scan_init(void) {
    ESP_LOGI(TAG, "BLE RID scanner init (detector, continuous scan)");
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
    if (!s_scan_running) return;

    ble_gap_disc_cancel();
    s_scan_running = false;
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
