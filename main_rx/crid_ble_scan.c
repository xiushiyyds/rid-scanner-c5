/**
 * crid_ble_scan.c — BLE Remote ID 扫描实现 (detector variant v2.0.0)
 *
 * 使用 NimBLE host stack 扫描 BLE Legacy 和 Extended Advertising，
 * 匹配 ASTM F3411 / ASD-STAN 分配的 Service UUID 0xFFFA，
 * 提取 OpenDroneID message pack 并投递到 sniffer 队列。
 *
 * TDD 时分复用：BLE 扫描与 WiFi sniffer 共享单射频。
 * v2.0.8: 缩短 TDD 周期至 ~400ms，减少 RID 广播漏检。
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

/* ---- 扫描占空比模式 ---- */
typedef enum {
    SCAN_DUTY_HIGH,    /* 未连接：连续扫描，最大化 RID 捕获率 */
    SCAN_DUTY_LOW,     /* 已连接：低占空比，给 GATT 通信留射频 */
} scan_duty_t;

static scan_duty_t s_scan_duty = SCAN_DUTY_HIGH;

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

    /* Legacy 扫描参数根据连接状态动态调整：
     * - 未连接(HIGH)：window=itvl=160 → 100ms 连续扫描，最大化 RID 捕获
     * - 已连接(LOW) ：window=96/itvl=160 → 60ms/100ms = 60% 占空比。
     *   v2.0.7: 从40%提到60%。GATT通知是突发小包（<20字节），
     *   BLE控制器会在scan window间隙自动调度，60%占空比下实测不丢通知，
     *   RID捕获率从40%提升到60%，更接近专用肩灯的灵敏度。 */
    memset(&disc_params, 0, sizeof(disc_params));
    if (s_scan_duty == SCAN_DUTY_HIGH) {
        disc_params.itvl = 160;           /* 160 × 0.625ms = 100ms */
        disc_params.window = 160;         /* 160 × 0.625ms = 100ms (100% 连续) */
        ESP_LOGI(TAG, "BLE Legacy scan started (100%% continuous duty, passive)");
    } else {
        disc_params.itvl = 160;           /* 100ms interval */
        disc_params.window = 96;          /* 60ms listen = 60% duty */
        ESP_LOGI(TAG, "BLE Legacy scan started (60%% duty, GATT coexist)");
    }
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
    return 0;
}

/* ================================================================
 * TDD 时分复用（v2.0.7 引入，v2.0.8 优化周期）
 *
 * ESP32-C5 只有一个 2.4GHz 射频，BLE 连续扫描时 WiFi sniffer 被饿死。
 * TDD 让 BLE 定期短暂暂停，WiFi 获得接收窗口。
 *
 * v2.0.8 优化：周期从 1000ms 缩短到 ~330ms。
 * RID 广播间隔通常 100ms~1s，短周期能显著降低漏检概率：
 *   - BLE 300ms 活跃足以覆盖 3 个广播间隔
 *   - WiFi 100ms 窗口在每个信道轮转周期内能多次命中
 *   - 频繁切换不影响 GATT（连接事件由 controller 独立调度）
 *
 * 未连接(HIGH)：BLE 300ms / WiFi 100ms，BLE 占 75%
 * 已连接(LOW) ：BLE 250ms / WiFi 100ms，BLE 占 71%
 * ================================================================ */
static TaskHandle_t s_tdd_task_handle = NULL;
static volatile bool s_tdd_should_stop = false;

#define TDD_BLE_ACTIVE_MS_HIGH   300
#define TDD_WIFI_WINDOW_MS_HIGH  100
#define TDD_BLE_ACTIVE_MS_LOW    250
#define TDD_WIFI_WINDOW_MS_LOW   100

static void tdd_coexist_task(void *arg) {
    (void)arg;
    ESP_LOGI(TAG, "TDD coexist task started (BLE/WiFi time-division, v2.0.8 short cycle)");

    while (!s_tdd_should_stop) {
        uint16_t ble_ms = (s_scan_duty == SCAN_DUTY_HIGH)
                          ? TDD_BLE_ACTIVE_MS_HIGH : TDD_BLE_ACTIVE_MS_LOW;
        uint16_t wifi_ms = (s_scan_duty == SCAN_DUTY_HIGH)
                           ? TDD_WIFI_WINDOW_MS_HIGH : TDD_WIFI_WINDOW_MS_LOW;

        /* BLE 扫描活跃窗口 */
        vTaskDelay(pdMS_TO_TICKS(ble_ms));
        if (s_tdd_should_stop) break;

        /* 暂停 BLE 扫描，把射频让给 WiFi。
         * ble_gap_disc_cancel() 是异步的（发送 HCI 命令后立即返回），
         * DISC_COMPLETE 事件稍后才到，但射频实际上很快就释放了。
         * 不等待 DISC_COMPLETE，直接进入 WiFi 窗口最大化接收时间。 */
        if (s_scan_running && s_scan_allowed) {
            int rc = ble_gap_disc_cancel();
            if (rc == 0) {
                s_scan_running = false;
                /* WiFi 接收窗口（信道轮转任务在此期间切换信道） */
                vTaskDelay(pdMS_TO_TICKS(wifi_ms));
                if (s_tdd_should_stop) break;
                /* 恢复 BLE 扫描（duty 由 set_duty_high/low 设置，TDD 直接用当前值） */
                if (s_scan_allowed) {
                    start_scan();
                    s_scan_running = true;
                }
            } else {
                /* cancel 失败（可能正在重启），短等后重试 */
                ESP_LOGD(TAG, "disc_cancel rc=%d, retry", rc);
                vTaskDelay(pdMS_TO_TICKS(50));
            }
        }
    }

    s_tdd_task_handle = NULL;
    vTaskDelete(NULL);
}

static void tdd_coexist_start(void) {
    if (s_tdd_task_handle != NULL) return;
    s_tdd_should_stop = false;
    xTaskCreate(tdd_coexist_task, "tdd_coex", 2048, NULL, 5, &s_tdd_task_handle);
}

static void tdd_coexist_stop(void) {
    if (s_tdd_task_handle == NULL) return;
    s_tdd_should_stop = true;
}

/* ================================================================
 * 公开接口
 * ================================================================ */

esp_err_t crid_ble_scan_init(void) {
    ESP_LOGI(TAG, "BLE RID scanner init (detector, TDD coexist with WiFi)");
    s_scan_running = false;
    /* v2.0.7: 启动 TDD 时分复用任务，BLE/WiFi 共享单射频 */
    tdd_coexist_start();
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
    /* v2.0.7: 重启 TDD 时分复用任务 */
    tdd_coexist_start();
    return ESP_OK;
}

void crid_ble_scan_stop(void) {
    /* 停 TDD 任务，防止它在我们 stop 后又把扫描拉起来 */
    tdd_coexist_stop();
    /* 等待 TDD 任务退出 */
    for (int i = 0; i < 20 && s_tdd_task_handle != NULL; i++) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (s_scan_running) {
        ble_gap_disc_cancel();
        s_scan_running = false;
    }
    ESP_LOGI(TAG, "BLE scan stopped (TDD coexist halted)");
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

void crid_ble_scan_set_duty_high(void) {
    if (s_scan_duty == SCAN_DUTY_HIGH) return;
    s_scan_duty = SCAN_DUTY_HIGH;
    ESP_LOGI(TAG, "Scan duty -> HIGH (BLE 800ms / WiFi 200ms)");
    /* TDD 任务会在下一个周期自动应用新的占空比参数，无需重启扫描 */
}

void crid_ble_scan_set_duty_low(void) {
    if (s_scan_duty == SCAN_DUTY_LOW) return;
    s_scan_duty = SCAN_DUTY_LOW;
    ESP_LOGI(TAG, "Scan duty -> LOW (BLE 700ms / WiFi 300ms)");
    /* TDD 任务会在下一个周期自动应用新的占空比参数，无需重启扫描 */
}
