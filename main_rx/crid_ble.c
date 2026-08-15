/**
 * crid_ble.c — BLE NUS 数据通道实现 (detector variant v2.0.0)
 *
 * 使用 NimBLE 实现 Nordic UART Service (NUS) 外设，
 * 所有 JSON 数据通过 BLE 通知发送到已连接的客户端。
 * 纯侦测板：无模拟发射控制，仅保留 SIM_PAIR 配对。
 */

#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_bt.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"

#include "crid_ble.h"
#include "crid_ble_scan.h"

static const char *TAG = "RID_BLE";

/* ================================================================
 * 配对状态
 * ================================================================ */
static bool g_paired = false;
static char g_pair_pin[5] = {0};
static ble_pair_display_cb_t g_pair_display_cb = NULL;
static void generate_pair_pin(void);

/* ================================================================
 * BLE RX 命令解析（仅 PAIR，侦测板不接受控制命令）
 * ================================================================ */
static void ble_rx_command_handler(const uint8_t *data, uint16_t len) {
    char cmd_buf[256];
    if (len >= sizeof(cmd_buf)) len = sizeof(cmd_buf) - 1;
    memcpy(cmd_buf, data, len);
    cmd_buf[len] = '\0';

    while (len > 0 && (cmd_buf[len-1] == '\n' || cmd_buf[len-1] == '\r')) {
        cmd_buf[--len] = '\0';
    }

    ESP_LOGI(TAG, "BLE RX cmd: %s", cmd_buf);

    /* SIM_PAIR 配对命令，不受配对状态限制 */
    if (strncmp(cmd_buf, "SIM_PAIR", 8) == 0) {
        if (cmd_buf[8] == ' ') {
            const char *input = cmd_buf + 9;
            if (strcmp(input, g_pair_pin) == 0) {
                g_paired = true;
                ESP_LOGI(TAG, "Pairing success");
                const char *ok = "PAIR_OK\n";
                crid_ble_write_cb(ok, strlen(ok), NULL);
            } else {
                ESP_LOGW(TAG, "Pairing failed: wrong PIN");
                const char *fail = "PAIR_FAIL\n";
                crid_ble_write_cb(fail, strlen(fail), NULL);
            }
        }
        return;
    }

    /* 非配对命令需要先完成配对 */
    if (!g_paired) {
        const char *reject = "PAIR_REQUIRED\n";
        crid_ble_write_cb(reject, strlen(reject), NULL);
        return;
    }

    /* 侦测板：已配对后收到非 PAIR 命令，返回 OK（忽略控制命令） */
    const char *ok = "{\"status\":\"ok\"}\n";
    crid_ble_write_cb(ok, strlen(ok), NULL);
}

/* ================================================================
 * NUS UUIDs
 * ================================================================ */
static const ble_uuid128_t gatt_svr_svc_nus_uuid =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                     0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E);

static const ble_uuid128_t gatt_svr_chr_nus_tx_uuid =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                     0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E);

static const ble_uuid128_t gatt_svr_chr_nus_rx_uuid =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                     0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E);

/* ================================================================
 * GATT 状态
 * ================================================================ */
static uint16_t g_nus_rx_handle;
static uint16_t g_nus_tx_handle;
static uint16_t g_nus_tx_cccd_handle;  /* CCCD descriptor handle (tx_handle + 1) */
static uint16_t g_nus_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static bool g_ble_initialized = false;
static bool g_subscribed = false;     /* 手机是否已使能通知 */
static esp_timer_handle_t g_adv_timer = NULL;
#define ADV_TIMEOUT_US (60 * 1000 * 1000)

/* ================================================================
 * 数据发送队列
 * ================================================================ */
#define BLE_TX_QUEUE_LEN    32
#define BLE_TX_BUF_SIZE     1024
#define BLE_TX_TASK_STACK   3072
#define BLE_TX_TASK_PRIO    3

static QueueHandle_t g_ble_tx_queue = NULL;
static uint32_t g_ble_queue_overflow_count = 0;

/* 行缓冲：将多次 crid_ble_write_cb 片段拼成完整一行（以 \n 结尾）后再入队，
 * 解决 JSON 被多个 DAT_PRINTF 拆分、被其他消息插队导致截断交错的问题。
 * 同时过滤 [ZH]...[/ZH] LCD 专用消息，不发给手机。 */
static char g_ble_linebuf[BLE_TX_BUF_SIZE];
static size_t g_ble_linebuf_len = 0;
static portMUX_TYPE g_ble_line_mux = portMUX_INITIALIZER_UNLOCKED;

/* ================================================================
 * 前置声明
 * ================================================================ */
static int gatt_svr_svc_nus_access(uint16_t conn_handle, uint16_t attr_handle,
                                    struct ble_gatt_access_ctxt *ctxt, void *arg);
static int ble_gap_event_cb(struct ble_gap_event *event, void *arg);
static void ble_host_task(void *param);
static void ble_tx_task(void *param);
static void ble_on_sync(void);
static void ble_advertise_start(void);
static void ble_delayed_scan_task(void *arg);
static void ble_connected_scan_task(void *arg);
static void ble_enqueue_line(const char *line, size_t len);  /* v2.0.5 forward decl */

/* ================================================================
 * GATT 服务定义
 * ================================================================ */
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_nus_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &gatt_svr_chr_nus_tx_uuid.u,
                .access_cb = gatt_svr_svc_nus_access,
                .flags = BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_READ,
            },
            {
                .uuid = &gatt_svr_chr_nus_rx_uuid.u,
                .access_cb = gatt_svr_svc_nus_access,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            { 0 },
        },
    },
    { 0 },
};

static int
gatt_svr_svc_nus_access(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)arg;
    (void)conn_handle;

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        if (attr_handle == g_nus_rx_handle) {
            uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
            if (len > 0) {
                uint8_t buf[128];
                if (len > sizeof(buf)) len = sizeof(buf);
                int rc = ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf), NULL);
                if (rc == 0) {
                    ble_rx_command_handler(buf, len);
                }
            }
        }
    }
    return 0;
}

/* 广播超时回调 */
static void adv_timeout_cb(void *arg) {
    (void)arg;
    ESP_LOGI(TAG, "No BLE connection for 60s, restarting advertising");
    ble_gap_adv_stop();
    g_nus_conn_handle = BLE_HS_CONN_HANDLE_NONE;
    if (g_adv_timer) {
        esp_timer_stop(g_adv_timer);
        esp_timer_delete(g_adv_timer);
        g_adv_timer = NULL;
    }
    ble_advertise_start();
}

/* ================================================================
 * GAP 事件回调
 * ================================================================ */
static int
ble_gap_event_cb(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (g_adv_timer) {
            esp_timer_stop(g_adv_timer);
            esp_timer_delete(g_adv_timer);
            g_adv_timer = NULL;
        }
        if (event->connect.status == 0) {
            g_nus_conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "Connected, conn_handle=%d", g_nus_conn_handle);

            /* v2.0.8: 不在 NimBLE host task 上下文做阻塞操作。
             * TDD 停止和扫描恢复交给独立 task 处理，避免死锁。
             * TDD 任务会在下次循环检测到 s_tdd_should_stop 自动退出，
             * 即使扫描继续跑一会儿也不影响 GATT（controller 会调度）。 */
            if (g_pair_display_cb) {
                g_pair_display_cb("RID-Scanner");
            }

            /* 宽松连接间隔 30~50ms */
            struct ble_gap_upd_params params = {
                .itvl_min = 24,
                .itvl_max = 40,
                .latency = 1,
                .supervision_timeout = 400,
                .min_ce_len = 0,
                .max_ce_len = 0,
            };
            ble_gap_update_params(event->connect.conn_handle, &params);
        } else {
            g_nus_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            ESP_LOGE(TAG, "Connect failed (status=%d)", event->connect.status);
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGW(TAG, "Disconnected (reason=0x%04x)", event->disconnect.reason);
        g_nus_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        g_subscribed = false;
        crid_ble_reset_pair();
        /* v2.0.5: 连接断开时把行缓冲中残留的数据整体入队，
         * 避免最后一行 JSON（不以 \n 结尾）丢失。 */
        portENTER_CRITICAL(&g_ble_line_mux);
        if (g_ble_linebuf_len > 0) {
            ble_enqueue_line(g_ble_linebuf, g_ble_linebuf_len);
            g_ble_linebuf_len = 0;
        }
        portEXIT_CRITICAL(&g_ble_line_mux);
        /* 断开后恢复高占空比连续扫描 */
        crid_ble_scan_set_duty_high();
        ble_advertise_start();
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        /* 手机写入 CCCD 使能 TX characteristic notification。
         * v2.0.8: attr_handle 是 CCCD descriptor handle。
         * 同时匹配 cccd_handle 和 tx_handle（容错不同 NimBLE 版本行为）。 */
        ESP_LOGI(TAG, "SUBSCRIBE: attr=%d tx_val=%d tx_cccd=%d notify=%d indicate=%d",
                 event->subscribe.attr_handle,
                 g_nus_tx_handle, g_nus_tx_cccd_handle,
                 event->subscribe.cur_notify, event->subscribe.cur_indicate);

        /* 兜底：如果 cccd_handle 没被 register_cb 捕获到，
         * 但 attr_handle 与 tx_handle 相差 1，也认为是 TX 的 CCCD */
        bool is_tx_cccd = (event->subscribe.attr_handle == g_nus_tx_cccd_handle) ||
                          (g_nus_tx_cccd_handle == 0 && g_nus_tx_handle != 0 &&
                           event->subscribe.attr_handle == g_nus_tx_handle + 1);

        if (is_tx_cccd && event->subscribe.cur_notify == 1) {
            ESP_LOGI(TAG, "Client subscribed to TX notifications");
            /* 如果 cccd_handle 之前没拿到，现在学到了 */
            if (g_nus_tx_cccd_handle == 0) {
                g_nus_tx_cccd_handle = event->subscribe.attr_handle;
                ESP_LOGI(TAG, "Learned TX CCCD handle=%d", g_nus_tx_cccd_handle);
            }
            g_subscribed = true;
            g_paired = true;
            const char *pair_ok = "PAIR_OK\n";
            crid_ble_write_cb(pair_ok, strlen(pair_ok), NULL);
            vTaskDelay(pdMS_TO_TICKS(50));
            const char *status = "STATUS:targets:0,gps:searching\n";
            crid_ble_write_cb(status, strlen(status), NULL);

            /* 延迟 2 秒后以低占空比恢复 BLE RID 扫描，
             * 给手机留时间做 initial data fetch，不抢射频 */
            xTaskCreate(ble_connected_scan_task, "ble_conn_scan",
                        2048, NULL, 4, NULL);
        }
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "ADV_COMPLETE reason=%d", event->adv_complete.reason);
        if (event->adv_complete.reason != 0) {
            ble_advertise_start();
        }
        break;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU exchanged: %d", event->mtu.value);
        break;

    case BLE_GAP_EVENT_CONN_UPDATE:
        if (event->conn_update.status == 0) {
            ESP_LOGI(TAG, "Connection parameters updated");
        } else {
            ESP_LOGW(TAG, "Connection parameter update rejected (status=%d)", event->conn_update.status);
        }
        break;

    case BLE_GAP_EVENT_ENC_CHANGE:
        ESP_LOGI(TAG, "Encryption change: status=%d", event->enc_change.status);
        break;

    default:
        break;
    }

    return 0;
}

/* ================================================================
 * 广播配置 (Legacy Advertising)
 * ================================================================ */
static uint8_t g_own_addr_type = BLE_OWN_ADDR_PUBLIC;

static int ble_build_adv_data(uint8_t *out, size_t out_sz)
{
    struct ble_hs_adv_fields fields;
    const char *name = "RID-Scanner";

    memset(&fields, 0, sizeof(fields));
    fields.name = (uint8_t *)name;
    fields.name_len = (uint8_t)strlen(name);
    fields.name_is_complete = 1;
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    uint8_t buf[BLE_HS_ADV_MAX_SZ];
    uint8_t buf_sz = 0;
    int rc = ble_hs_adv_set_fields(&fields, buf, &buf_sz, sizeof(buf));
    if (rc != 0) return rc;
    if (buf_sz > out_sz) return BLE_HS_EMSGSIZE;
    memcpy(out, buf, buf_sz);
    return (int)buf_sz;
}

static int ble_build_scan_rsp(uint8_t *out, size_t out_sz)
{
    struct ble_hs_adv_fields rsp_fields;
    memset(&rsp_fields, 0, sizeof(rsp_fields));
    rsp_fields.uuids128 = (ble_uuid128_t *)&gatt_svr_svc_nus_uuid;
    rsp_fields.num_uuids128 = 1;
    rsp_fields.uuids128_is_complete = 1;

    uint8_t buf[BLE_HS_ADV_MAX_SZ];
    uint8_t buf_sz = 0;
    int rc = ble_hs_adv_set_fields(&rsp_fields, buf, &buf_sz, sizeof(buf));
    if (rc != 0) return rc;
    if (buf_sz > out_sz) return BLE_HS_EMSGSIZE;
    memcpy(out, buf, buf_sz);
    return (int)buf_sz;
}

static void
ble_advertise_start(void)
{
    int rc;

    uint8_t adv_buf[BLE_HS_ADV_MAX_SZ];
    int adv_len = ble_build_adv_data(adv_buf, sizeof(adv_buf));
    if (adv_len < 0) {
        ESP_LOGE(TAG, "build adv data failed (%d)", adv_len);
        return;
    }
    rc = ble_gap_adv_set_data(adv_buf, adv_len);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_set_data failed (rc=%d)", rc);
        return;
    }

    uint8_t rsp_buf[BLE_HS_ADV_MAX_SZ];
    int rsp_len = ble_build_scan_rsp(rsp_buf, sizeof(rsp_buf));
    if (rsp_len < 0) {
        ESP_LOGE(TAG, "build rsp data failed (%d)", rsp_len);
        return;
    }
    rc = ble_gap_adv_rsp_set_data(rsp_buf, rsp_len);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_rsp_set_data failed (rc=%d)", rc);
        return;
    }

    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    adv_params.itvl_min  = BLE_GAP_ADV_ITVL_MS(100);
    adv_params.itvl_max  = BLE_GAP_ADV_ITVL_MS(150);
    adv_params.channel_map = 0;
    adv_params.filter_policy = 0;
    adv_params.high_duty_cycle = 0;

    rc = ble_gap_adv_start(g_own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, ble_gap_event_cb, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "ble_gap_adv_start failed (rc=%d)", rc);
        return;
    }

    ESP_LOGI(TAG, "Advertising ACTIVE as 'RID-Scanner' (legacy)");

    if (g_adv_timer) {
        esp_timer_stop(g_adv_timer);
        esp_timer_delete(g_adv_timer);
    }
    esp_timer_create_args_t tmr = { .callback = adv_timeout_cb, .arg = NULL, .name = "ble_adv_tmo" };
    if (esp_timer_create(&tmr, &g_adv_timer) == ESP_OK) {
        esp_timer_start_once(g_adv_timer, ADV_TIMEOUT_US);
    }
}

/* ================================================================
 * 连接后低占空比恢复 BLE 扫描
 * ================================================================ */
static void ble_connected_scan_task(void *arg) {
    (void)arg;
    /* 先停 TDD 和扫描（在独立 task 上下文，不阻塞 NimBLE host） */
    crid_ble_scan_stop();
    /* 等 2 秒让手机完成 service discovery + 初始数据交换 */
    vTaskDelay(pdMS_TO_TICKS(2000));
    if (g_nus_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ESP_LOGI(TAG, "Resuming BLE RID scan (low duty, GATT coexist)");
        crid_ble_scan_set_duty_low();
        if (!crid_ble_scan_is_running()) {
            crid_ble_scan_start();
        }
    }
    vTaskDelete(NULL);
}

/* ================================================================
 * 延迟启动 BLE 扫描
 * ================================================================ */
static void ble_delayed_scan_task(void *arg)
{
    uint32_t delay_ms = (uint32_t)(intptr_t)arg;
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
    ESP_LOGI(TAG, "Starting/restarting BLE RID scan (delay=%lu ms)", (unsigned long)delay_ms);
    crid_ble_scan_stop();
    vTaskDelay(pdMS_TO_TICKS(100));
    crid_ble_scan_set_duty_high();
    crid_ble_scan_start();
    vTaskDelete(NULL);
}

void crid_ble_delayed_scan_restart(uint32_t delay_ms) {
    xTaskCreate(ble_delayed_scan_task, "ble_scan_rst", 2048,
                (void *)(intptr_t)delay_ms, 4, NULL);
}

/* ================================================================
 * NimBLE 同步回调
 * ================================================================ */
static void
ble_on_sync(void)
{
    ESP_LOGI(TAG, "NimBLE host synced");

    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_util_ensure_addr failed (rc=%d)", rc);
    }

    rc = ble_hs_id_infer_auto(0, &g_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto failed (rc=%d)", rc);
    } else {
        uint8_t addr_val[6] = {0};
        ble_hs_id_copy_addr(g_own_addr_type, addr_val, NULL);
        ESP_LOGI(TAG, "BLE address type=%d addr=%02x:%02x:%02x:%02x:%02x:%02x",
                 g_own_addr_type, addr_val[5], addr_val[4], addr_val[3],
                 addr_val[2], addr_val[1], addr_val[0]);
    }

    /* v2.0.8: 在 sync 回调中查找 characteristic handle。
     * NimBLE 在调用 sync_cb 之前已经完成了 ble_gatts_start()，
     * GATT 表已完全注册，ble_gatts_find_chr 可以安全使用。 */
    int rc;
    rc = ble_gatts_find_chr(&gatt_svr_svc_nus_uuid.u,
                             &gatt_svr_chr_nus_tx_uuid.u,
                             NULL, &g_nus_tx_handle);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to find NUS TX chr handle (rc=%d)", rc);
        g_nus_tx_handle = 0;
    } else {
        /* CCCD descriptor 紧跟在 characteristic value handle 后面。
         * GATT 表布局：decl_handle, value_handle, cccd_handle */
        g_nus_tx_cccd_handle = g_nus_tx_handle + 1;
        ESP_LOGI(TAG, "NUS TX val_handle=%d cccd_handle=%d",
                 g_nus_tx_handle, g_nus_tx_cccd_handle);
    }

    rc = ble_gatts_find_chr(&gatt_svr_svc_nus_uuid.u,
                             &gatt_svr_chr_nus_rx_uuid.u,
                             NULL, &g_nus_rx_handle);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to find NUS RX chr handle (rc=%d)", rc);
        g_nus_rx_handle = 0;
    } else {
        ESP_LOGI(TAG, "NUS RX val_handle=%d", g_nus_rx_handle);
    }

    ble_advertise_start();

    BaseType_t ok = xTaskCreate(ble_delayed_scan_task, "ble_delayed_scan",
                                2048, (void *)(uintptr_t)800, 4, NULL);
    if (ok != pdPASS) {
        crid_ble_scan_start();
    }
}

/* ================================================================
 * NimBLE 主机任务
 * ================================================================ */
static void
ble_host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/* ================================================================
 * BLE 发送任务
 * ================================================================ */
static void
ble_tx_task(void *param)
{
    (void)param;
    char *buf;
    uint32_t dropped_no_conn = 0;
    uint32_t dropped_no_handle = 0;
    uint32_t sent_ok = 0;
    uint32_t notify_errors = 0;

    while (1) {
        if (xQueueReceive(g_ble_tx_queue, &buf, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (!buf) continue;

        size_t data_len = strnlen(buf, BLE_TX_BUF_SIZE);
        if (data_len == 0) { free(buf); continue; }

        if (g_nus_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
            dropped_no_conn++;
            free(buf);
            continue;
        }
        if (g_nus_tx_handle == 0) {
            dropped_no_handle++;
            if ((dropped_no_handle % 50) == 1) {
                ESP_LOGW(TAG, "TX: g_nus_tx_handle=0, dropping data (count=%lu)",
                         (unsigned long)dropped_no_handle);
            }
            free(buf);
            continue;
        }

        uint16_t mtu = ble_att_mtu(g_nus_conn_handle);
        uint16_t chunk_size = (mtu >= 6) ? (mtu - 3) : 20;
        size_t offset = 0;
        bool send_ok = true;

        while (offset < data_len) {
            size_t send_len = data_len - offset;
            if (send_len > chunk_size) send_len = chunk_size;

            struct os_mbuf *om = ble_hs_mbuf_from_flat(buf + offset, send_len);
            if (om) {
                int rc = ble_gattc_notify_custom(g_nus_conn_handle,
                                                  g_nus_tx_handle, om);
                if (rc != 0) {
                    notify_errors++;
                    if ((notify_errors % 20) == 1) {
                        ESP_LOGW(TAG, "notify_custom rc=%d (total=%lu)",
                                 rc, (unsigned long)notify_errors);
                    }
                    send_ok = false;
                    break;
                }
            }
            offset += send_len;
            if (offset < data_len) {
                vTaskDelay(pdMS_TO_TICKS(5));
            }
        }
        if (send_ok) sent_ok++;
        if ((sent_ok % 100) == 0) {
            ESP_LOGI(TAG, "TX stats: sent=%lu no_conn=%lu no_hdl=%lu errs=%lu",
                     (unsigned long)sent_ok, (unsigned long)dropped_no_conn,
                     (unsigned long)dropped_no_handle, (unsigned long)notify_errors);
        }

        free(buf);
    }
}

/* ================================================================
 * 公开接口
 * ================================================================ */

void crid_ble_register_pair_display(ble_pair_display_cb_t cb) {
    g_pair_display_cb = cb;
}

bool crid_ble_is_paired(void) {
    return g_paired;
}

void crid_ble_reset_pair(void) {
    g_paired = false;
    memset(g_pair_pin, 0, sizeof(g_pair_pin));
}

static void generate_pair_pin(void) {
    uint32_t r = esp_random() % 10000;
    snprintf(g_pair_pin, sizeof(g_pair_pin), "%04u", (unsigned int)r);
    g_paired = false;
    ESP_LOGI(TAG, "Generated pair PIN: %s", g_pair_pin);
    if (g_pair_display_cb) {
        g_pair_display_cb(g_pair_pin);
    }
}

esp_err_t
crid_ble_init(void)
{
    if (g_ble_initialized) return ESP_OK;

    ESP_LOGI(TAG, "Initializing BLE (NimBLE NUS, detector)...");

    g_ble_tx_queue = xQueueCreate(BLE_TX_QUEUE_LEN, sizeof(char *));
    if (!g_ble_tx_queue) {
        ESP_LOGE(TAG, "TX queue creation failed");
        return ESP_ERR_NO_MEM;
    }

    ble_hs_cfg.sync_cb = ble_on_sync;

    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %s", esp_err_to_name(err));
        return err;
    }

    int rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        nimble_port_deinit();
        return ESP_FAIL;
    }

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        nimble_port_deinit();
        return ESP_FAIL;
    }

    nimble_port_freertos_init(ble_host_task);

    BaseType_t task_created = xTaskCreatePinnedToCore(ble_tx_task, "ble_tx",
                                BLE_TX_TASK_STACK, NULL, BLE_TX_TASK_PRIO, NULL,
                                tskNO_AFFINITY);
    if (task_created != pdPASS) {
        nimble_port_deinit();
        return ESP_ERR_NO_MEM;
    }

    g_ble_initialized = true;
    ESP_LOGI(TAG, "BLE initialized");
    return ESP_OK;
}

/* 将一行完整数据入队发送。以 [ZH] 开头的 LCD 专用消息直接丢弃。 */
static void ble_enqueue_line(const char *line, size_t len) {
    if (len == 0) return;
    /* 过滤 LCD 中文摘要：[ZH]...[/ZH] */
    if (len >= 4 && line[0] == '[' && line[1] == 'Z' && line[2] == 'H' && line[3] == ']') {
        return;
    }
    if (len > BLE_TX_BUF_SIZE - 1) len = BLE_TX_BUF_SIZE - 1;

    char *buf = (char *)malloc(len + 1);
    if (!buf) return;
    memcpy(buf, line, len);
    buf[len] = '\0';

    if (xQueueSend(g_ble_tx_queue, &buf, pdMS_TO_TICKS(20)) != pdTRUE) {
        free(buf);
        g_ble_queue_overflow_count++;
    }
}

void
crid_ble_write_cb(const char *data, size_t len, void *ctx)
{
    (void)ctx;

    if (!g_ble_initialized || !g_ble_tx_queue) return;
    if (!data || len == 0) return;
    if (g_nus_conn_handle == BLE_HS_CONN_HANDLE_NONE) return;

    /* 行缓冲：多个 DAT_PRINTF 片段拼成完整一行（\n 结尾）后整体入队，
     * 保证 JSON 原子性，不会被 SELF_GPS 等消息插队截断。 */
    portENTER_CRITICAL(&g_ble_line_mux);
    for (size_t i = 0; i < len; i++) {
        char c = data[i];
        if (c == '\n') {
            /* 一行结束，入队 */
            ble_enqueue_line(g_ble_linebuf, g_ble_linebuf_len);
            g_ble_linebuf_len = 0;
        } else if (c != '\r') {
            if (g_ble_linebuf_len < sizeof(g_ble_linebuf) - 1) {
                g_ble_linebuf[g_ble_linebuf_len++] = c;
            } else {
                /* 行太长，强制刷新 */
                ble_enqueue_line(g_ble_linebuf, g_ble_linebuf_len);
                g_ble_linebuf_len = 0;
                g_ble_linebuf[g_ble_linebuf_len++] = c;
            }
        }
    }
    portEXIT_CRITICAL(&g_ble_line_mux);
}

bool
crid_ble_is_connected(void)
{
    return g_nus_conn_handle != BLE_HS_CONN_HANDLE_NONE;
}
