/**
 * crid_ble.c — BLE NUS 数据通道实现
 *
 * 使用 NimBLE 实现 Nordic UART Service (NUS) 外设，
 * 所有 JSON 数据通过 BLE 通知发送到已连接的客户端。
 *
 * 内存策略：
 *   发送缓冲区从 SPIRAM 分配，队列使用指针传递避免数据拷贝。
 */

#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

/* ESP controller API */
#include "esp_bt.h"

/* NimBLE host stack */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"

#include "crid_ble.h"
#include "crid_ble_scan.h"

static const char *TAG = "RID_BLE";

/* ================================================================
 * v1.5: 配对状态（前置声明，供命令处理器使用）
 * ================================================================ */
static bool g_paired = false;
static char g_pair_pin[5] = {0};  // 4位配对码 + null
static ble_pair_display_cb_t g_pair_display_cb = NULL;
static void generate_pair_pin(void);  // 前向声明

/* ================================================================
 * v1.3 新增：模拟器控制回调
 * ================================================================ */
static sim_ble_start_cb_t s_sim_start_cb = NULL;
static sim_ble_stop_cb_t s_sim_stop_cb = NULL;
static sim_ble_config_cb_t s_sim_config_cb = NULL;
static sim_ble_status_cb_t s_sim_status_cb = NULL;

void crid_ble_register_sim_callbacks(sim_ble_start_cb_t start_cb,
                                      sim_ble_stop_cb_t stop_cb,
                                      sim_ble_config_cb_t config_cb,
                                      sim_ble_status_cb_t status_cb) {
    s_sim_start_cb = start_cb;
    s_sim_stop_cb = stop_cb;
    s_sim_config_cb = config_cb;
    s_sim_status_cb = status_cb;
    ESP_LOGI(TAG, "SIM BLE callbacks registered");
}

/* ================================================================
 * v1.4: BLE RX 命令解析（SIM 控制命令 - 多目标版）
 * ================================================================ */
static void ble_rx_command_handler(const uint8_t *data, uint16_t len) {
    /* 将收到的数据转为字符串 */
    char cmd_buf[256];
    if (len >= sizeof(cmd_buf)) len = sizeof(cmd_buf) - 1;
    memcpy(cmd_buf, data, len);
    cmd_buf[len] = '\0';

    /* 去除尾部换行 */
    while (len > 0 && (cmd_buf[len-1] == '\n' || cmd_buf[len-1] == '\r')) {
        cmd_buf[--len] = '\0';
    }

    ESP_LOGI(TAG, "BLE RX cmd: %s", cmd_buf);

    /* v1.5: 配对验证命令，不受配对状态限制 */
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

    /* v1.5: 非配对命令需要先完成配对 */
    if (!g_paired) {
        const char *reject = "PAIR_REQUIRED\n";
        crid_ble_write_cb(reject, strlen(reject), NULL);
        ESP_LOGW(TAG, "Command rejected: not paired");
        return;
    }

    /* 解析命令 */
    if (strncmp(cmd_buf, "SIM_START", 9) == 0) {
        if (s_sim_start_cb) {
            /* SIM_START [count] — 可选参数指定目标数量 */
            int count = 0;  /* 0 = 使用上次配置 */
            if (cmd_buf[9] == ' ') {
                sscanf(cmd_buf + 10, "%d", &count);
                if (count < 1) count = 0;  /* 无效值回退 */
                if (count > 64) count = 64;
            }
            s_sim_start_cb(count);
            char reply[128];
            if (count > 0) {
                snprintf(reply, sizeof(reply),
                         "{\"cmd\":\"sim_start\",\"status\":\"ok\",\"targets\":%d}\n", count);
            } else {
                snprintf(reply, sizeof(reply),
                         "{\"cmd\":\"sim_start\",\"status\":\"ok\",\"targets\":\"last\"}\n");
            }
            crid_ble_write_cb(reply, strlen(reply), NULL);
        }
    } else if (strncmp(cmd_buf, "SIM_STOP", 8) == 0) {
        if (s_sim_stop_cb) {
            s_sim_stop_cb();
            char reply[64];
            snprintf(reply, sizeof(reply), "{\"cmd\":\"sim_stop\",\"status\":\"ok\"}\n");
            crid_ble_write_cb(reply, strlen(reply), NULL);
        }
    } else if (strncmp(cmd_buf, "SIM_CONFIG", 10) == 0) {
        /* SIM_CONFIG <lat> <lon> <mode> <channel> [count] [tx_power] */
        double lat = 38.9140, lon = 121.6147;
        int mode = 0, channel = 6, count = 0, tx_power = 0;
        int parsed = sscanf(cmd_buf + 10, "%lf %lf %d %d %d %d",
                            &lat, &lon, &mode, &channel, &count, &tx_power);
        (void)parsed;
        if (count < 0) count = 0;
        if (count > 64) count = 64;
        if (tx_power < 0) tx_power = 0;
        if (s_sim_config_cb) {
            s_sim_config_cb(lat, lon, mode, channel, count, tx_power);
            char reply[256];
            snprintf(reply, sizeof(reply),
                     "{\"cmd\":\"sim_config\",\"lat\":%.4f,\"lon\":%.4f,"
                     "\"mode\":%d,\"ch\":%d,\"targets\":%d,\"tx_power\":%d}\n",
                     lat, lon, mode, channel, count, tx_power);
            crid_ble_write_cb(reply, strlen(reply), NULL);
        }
    } else if (strncmp(cmd_buf, "SIM_STATUS", 10) == 0) {
        if (s_sim_status_cb) {
            char status_buf[512];
            s_sim_status_cb(status_buf, sizeof(status_buf));
            crid_ble_write_cb(status_buf, strlen(status_buf), NULL);
        }
    }
}

/* ================================================================
 * NUS UUIDs (Nordic UART Service)
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
static uint16_t g_nus_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static bool g_ble_initialized = false;
static esp_timer_handle_t g_adv_timer = NULL;
#define ADV_TIMEOUT_US (60 * 1000 * 1000)  /* 60秒 */

/* lcdfix21: NUS 外设使用的 Extended Adv 实例号（必须在 adv_timeout_cb 前定义） */
#define NUS_ADV_INSTANCE   0

/* ================================================================
 * 数据发送队列 (指针传递，缓冲区从 SPIRAM 分配)
 * ================================================================ */

#define BLE_TX_QUEUE_LEN    16
#define BLE_TX_BUF_SIZE     512
#define BLE_TX_TASK_STACK   3072
#define BLE_TX_TASK_PRIO    3

static QueueHandle_t g_ble_tx_queue = NULL;
static uint32_t g_ble_queue_overflow_count = 0;  /* 队列溢出计数（诊断用） */

/* ================================================================
 * v1.5 新增：配对状态
 * ================================================================ */

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
    (void)attr_handle;

    /* v1.3: 处理 RX characteristic 写入（来自 app 的 SIM 控制命令） */
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        /* 检查是否写入了 RX characteristic */
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

/* 广播超时回调：60秒无人连接则关闭 BLE */
static void adv_timeout_cb(void *arg) {
    ESP_LOGI(TAG, "No BLE connection for 60s, restarting advertising");
    ble_gap_ext_adv_stop(NUS_ADV_INSTANCE);
    g_nus_conn_handle = BLE_HS_CONN_HANDLE_NONE;
    if (g_adv_timer) {
        esp_timer_stop(g_adv_timer);
        esp_timer_delete(g_adv_timer);
        g_adv_timer = NULL;
    }
    /* 重新广播，保持设备可发现 */
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
        /* 连接成功，取消广播超时定时器 */
        if (g_adv_timer) {
            esp_timer_stop(g_adv_timer);
            esp_timer_delete(g_adv_timer);
            g_adv_timer = NULL;
        }
        if (event->connect.status == 0) {
            g_nus_conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "Connected, conn_handle=%d", g_nus_conn_handle);
            /* 连接即授权：侦测设备无敏感数据，无需 PIN 配对 */
            g_paired = true;
            if (g_pair_display_cb) {
                g_pair_display_cb("RID-Scanner");  /* 显示"蓝牙已连接"3秒 */
            }

            /* 主动通知网页端：连接即配对成功（兼容旧网页的 PAIR_OK 流程）*/
            const char *pair_ok = "PAIR_OK\n";
            crid_ble_write_cb(pair_ok, strlen(pair_ok), NULL);
            /* 顺便推送一次状态 */
            const char *status = "STATUS:targets:0,gps:searching\n";
            crid_ble_write_cb(status, strlen(status), NULL);

            /* lcdfix15: 不要请求短连接间隔！
             * ESP32-C5 单天线，BLE 中心连接 + Observer 扫描并发时，
             * 过短的连接间隔（7.5~15ms）会把扫描时间窗全部挤占，
             * 导致收不到肩灯/无人机的 BLE RID 广播。
             * 用宽松间隔 30~50ms，给扫描留足时间。 */
            struct ble_gap_upd_params params = {
                .itvl_min = 24,  /* 30ms */
                .itvl_max = 40,  /* 50ms */
                .latency = 1,    /* 允许跳过1个间隔，进一步释放扫描时间 */
                .supervision_timeout = 400,
                .min_ce_len = 0,
                .max_ce_len = 0,
            };
            ble_gap_update_params(event->connect.conn_handle, &params);

            /* lcdfix15: 连接建立后 controller 会自动停止 BLE 扫描，
             * 必须延迟重启扫描，否则收不到肩灯 RID 广播。
             * 不能在 host task 里 vTaskDelay，用单独任务。 */
            crid_ble_delayed_scan_restart(1500);
        } else {
            g_nus_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            ESP_LOGE(TAG, "Connect failed (status=%d)", event->connect.status);
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGW(TAG, "Disconnected (reason=0x%04x)", event->disconnect.reason);
        g_nus_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        crid_ble_reset_pair();
        ble_advertise_start();
        /* 断开后重启 RID 扫描（connection 期间可能被 controller 停掉） */
        crid_ble_delayed_scan_restart(500);
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        /* lcdfix21: 连入成功 reason=0 不要重启广播，否则会和现有连接冲突。
         * 60 秒超时（BLE_HS_ETIMEOUT）才是真正需要重启的情况。 */
        ESP_LOGI(TAG, "ADV_COMPLETE reason=%d instance=%d",
                 event->adv_complete.reason, event->adv_complete.instance);
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
 * 广播配置 (lcdfix21)
 *
 * lcdfix21 关键修复：
 *   sdkconfig 打开 CONFIG_BT_NIMBLE_EXT_ADV=y 后，旧版 ble_gap_adv_start()
 *   在 NimBLE 中被编译成直接返回 BLE_HS_ENOTSUP(=8)，导致板子根本没在广播，
 *   手机自然搜不到。必须改用 Extended Advertising 实例 API，并把 legacy_pdu=1
 *   强制走 Legacy PDU（31 字节），保证手机/浏览器兼容性。
 * ================================================================ */

static int g_adv_configured = 0;
/* lcdfix22: 由 ble_hs_id_infer_auto() 推断的实际地址类型。
 * ESP32-C5 可能没有公共地址，硬编码 BLE_OWN_ADDR_PUBLIC 会导致
 * ext_adv_configure 在控制器层静默失败（手机搜不到广播）。*/
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

    /* 首次启动时配置 Extended Adv 实例 0，legacy_pdu=1 强制 Legacy PDU。
     * 注意：ble_gap_ext_adv_configure 同一个 instance 只能调用一次，
     * 断开重连时直接 start 即可，不能重复 configure（会返回 ENOMEM）。 */
    if (!g_adv_configured) {
        struct ble_gap_ext_adv_params ext_params;
        memset(&ext_params, 0, sizeof(ext_params));
        ext_params.connectable      = 1;
        ext_params.scannable        = 1;
        ext_params.legacy_pdu       = 1;   /* 走 Legacy 31 字节 PDU，手机兼容 */
        ext_params.anonymous        = 0;
        ext_params.include_tx_power = 0;
        ext_params.directed         = 0;
        ext_params.high_duty_directed = 0;
        ext_params.itvl_min         = BLE_GAP_ADV_ITVL_MS(100);
        ext_params.itvl_max         = BLE_GAP_ADV_ITVL_MS(150);
        ext_params.channel_map      = 0;   /* 默认 37/38/39 全用 */
        ext_params.own_addr_type    = g_own_addr_type;
        ext_params.filter_policy    = 0;
        ext_params.primary_phy      = BLE_GAP_LE_PHY_1M;
        ext_params.secondary_phy    = BLE_GAP_LE_PHY_1M;
        ext_params.tx_power         = 127; /* 让控制器选默认功率 */
        ext_params.sid              = 0;

        int8_t selected_tx_power = 0;
        rc = ble_gap_ext_adv_configure(NUS_ADV_INSTANCE, &ext_params,
                                        &selected_tx_power,
                                        ble_gap_event_cb, NULL);
        if (rc != 0) {
            ESP_LOGE(TAG, "ext_adv_configure failed (rc=%d)", rc);
            return;
        }

        /* 设置广播数据 */
        uint8_t adv_buf[BLE_HS_ADV_MAX_SZ];
        int adv_len = ble_build_adv_data(adv_buf, sizeof(adv_buf));
        if (adv_len < 0) {
            ESP_LOGE(TAG, "build adv data failed (%d)", adv_len);
            return;
        }
        struct os_mbuf *adv_om = ble_hs_mbuf_from_flat(adv_buf, (uint16_t)adv_len);
        if (!adv_om) {
            ESP_LOGE(TAG, "adv mbuf alloc failed");
            return;
        }
        rc = ble_gap_ext_adv_set_data(NUS_ADV_INSTANCE, adv_om);
        os_mbuf_free_chain(adv_om);
        if (rc != 0) {
            ESP_LOGE(TAG, "ext_adv_set_data failed (rc=%d)", rc);
            return;
        }

        /* 设置扫描应答数据（NUS UUID） */
        uint8_t rsp_buf[BLE_HS_ADV_MAX_SZ];
        int rsp_len = ble_build_scan_rsp(rsp_buf, sizeof(rsp_buf));
        if (rsp_len < 0) {
            ESP_LOGE(TAG, "build rsp data failed (%d)", rsp_len);
            return;
        }
        struct os_mbuf *rsp_om = ble_hs_mbuf_from_flat(rsp_buf, (uint16_t)rsp_len);
        if (!rsp_om) {
            ESP_LOGE(TAG, "rsp mbuf alloc failed");
            return;
        }
        rc = ble_gap_ext_adv_rsp_set_data(NUS_ADV_INSTANCE, rsp_om);
        os_mbuf_free_chain(rsp_om);
        if (rc != 0) {
            ESP_LOGE(TAG, "ext_adv_rsp_set_data failed (rc=%d)", rc);
            return;
        }

        g_adv_configured = 1;
        ESP_LOGI(TAG, "Extended adv instance %d configured (legacy_pdu=1, 100-150ms)",
                 NUS_ADV_INSTANCE);
    }

    rc = ble_gap_ext_adv_start(NUS_ADV_INSTANCE, 0, 0);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "ext_adv_start failed (rc=%d)", rc);
        return;
    }

    ESP_LOGI(TAG, "Advertising as 'RID-Scanner' with NUS UUID");

    /* 启动 60 秒超时定时器：无人连接则自动关闭广播 */
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
 * 延迟启动 BLE 扫描的任务
 *
 * 不能在 ble_on_sync()（NimBLE 主机任务上下文）里 vTaskDelay，
 * 否则会阻塞整个 NimBLE host 任务，导致 GAP 事件无法处理、广播不可见。
 * 单独创建一个短生命周期任务来延迟启动扫描。
 * ================================================================ */
static void ble_delayed_scan_task(void *arg)
{
    uint32_t delay_ms = (uint32_t)(intptr_t)arg;
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
    ESP_LOGI(TAG, "Starting/restarting BLE RID scan (delay=%lu ms)", (unsigned long)delay_ms);
    /* 先停再启：BLE 连接建立后 controller 会停掉扫描，
     * crid_ble_scan_start() 内部有 s_scan_running 标志但 controller 状态
     * 可能已经不同步，强制 cancel 后再启动 */
    crid_ble_scan_stop();
    vTaskDelay(pdMS_TO_TICKS(100));
    crid_ble_scan_start();
    vTaskDelete(NULL);
}

/* lcdfix15: 可在任意上下文调用（GAP事件/定时器），安全重启扫描 */
void crid_ble_delayed_scan_restart(uint32_t delay_ms) {
    xTaskCreate(ble_delayed_scan_task, "ble_scan_rst", 2048,
                (void *)(intptr_t)delay_ms, 4, NULL);
}

/* ================================================================
 * NimBLE 同步回调 — 主机就绪后开始广播并查找句柄
 * ================================================================ */

static void
ble_on_sync(void)
{
    ESP_LOGI(TAG, "NimBLE host synced");

    /* lcdfix23: 必须在 on_sync 中按官方标准顺序执行：
     *   1) ble_hs_util_ensure_addr(0) — 确保 controller 已配置身份地址
     *      （优先公共地址；不存在则自动生成静态随机地址）
     *   2) ble_hs_id_infer_auto(0, &type) — 推断当前可用的地址类型
     *
     * lcdfix22 的错误：ensure_addr 被错误放在了 crid_ble_init() 中，
     * 那时 NimBLE host 尚未启动、controller 未就绪，调用等于空操作；
     * 而 on_sync 中直接 infer_auto 没有先 ensure_addr，导致板子无公共
     * 地址时身份地址未生成，ext_adv 即使返回 0 也不发射广播。
     * 参照 ESP-IDF v5.5.2 bleprph 示例。 */
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

    /* 查找 TX characteristic 句柄（通知从此发送） */
    rc = ble_gatts_find_chr(&gatt_svr_svc_nus_uuid.u,
                                 &gatt_svr_chr_nus_tx_uuid.u,
                                 NULL, &g_nus_tx_handle);
    if (rc != 0) {
        ESP_LOGE(TAG, "TX characteristic not found (rc=%d)", rc);
        g_nus_tx_handle = 0;
    } else {
        ESP_LOGI(TAG, "TX handle=0x%04x", g_nus_tx_handle);
    }

    /* 查找 RX characteristic 句柄（接收客户端写入） */
    rc = ble_gatts_find_chr(&gatt_svr_svc_nus_uuid.u,
                             &gatt_svr_chr_nus_rx_uuid.u,
                             NULL, &g_nus_rx_handle);
    if (rc != 0) {
        ESP_LOGE(TAG, "RX characteristic not found (rc=%d)", rc);
        g_nus_rx_handle = 0;
    } else {
        ESP_LOGI(TAG, "RX handle=0x%04x", g_nus_rx_handle);
    }

    /* 先启动广播——不阻塞 host 任务 */
    ble_advertise_start();

    /* 延迟启动 BLE RID 扫描（单独任务，不阻塞 NimBLE host）。
     * 给 advertising 800ms 稳定时间，避免控制器同时配置 adv+scan 的竞态。 */
    BaseType_t ok = xTaskCreate(ble_delayed_scan_task, "ble_delayed_scan",
                                2048, (void *)(uintptr_t)800, 4, NULL);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create delayed scan task, starting scan immediately");
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
 * BLE 发送任务 — 从队列取数据并通过通知发送
 * ================================================================ */

static void
ble_tx_task(void *param)
{
    (void)param;
    char *buf;

    while (1) {
        if (xQueueReceive(g_ble_tx_queue, &buf, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (!buf) continue;

        size_t data_len = strnlen(buf, BLE_TX_BUF_SIZE);
        if (data_len > 0 && g_nus_conn_handle != BLE_HS_CONN_HANDLE_NONE && g_nus_tx_handle != 0) {
            uint16_t mtu = ble_att_mtu(g_nus_conn_handle);
            uint16_t chunk_size = (mtu >= 6) ? (mtu - 3) : 20;
            size_t offset = 0;

            while (offset < data_len) {
                size_t send_len = data_len - offset;
                if (send_len > chunk_size) send_len = chunk_size;

                struct os_mbuf *om = ble_hs_mbuf_from_flat(buf + offset, send_len);
                if (om) {
                    int rc = ble_gattc_notify_custom(g_nus_conn_handle,
                                                      g_nus_tx_handle, om);
                    if (rc != 0) {
                        os_mbuf_free_chain(om);
                        break;
                    }
                }
                offset += send_len;
                if (offset < data_len) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
            }
        }

        free(buf);
    }
}

/* ================================================================
 * 公开接口
 * ================================================================ */

/* ================================================================
 * v1.5 配对相关函数
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

    ESP_LOGI(TAG, "Initializing BLE (NimBLE NUS)...");

    /* 创建发送队列 (元素为 char* 指针) */
    g_ble_tx_queue = xQueueCreate(BLE_TX_QUEUE_LEN, sizeof(char *));
    if (!g_ble_tx_queue) {
        ESP_LOGE(TAG, "TX queue creation failed");
        return ESP_ERR_NO_MEM;
    }

    /* 设置同步回调（必须在 nimble_port_init 之前） */
    ble_hs_cfg.sync_cb = ble_on_sync;

    /* 初始化 NimBLE 控制器 + 主机。
     * 注意：nimble_port_init() 内部会调用 esp_bt_controller_init/enable，
     * 但 controller enable 是异步的——在 nimble_port_freertos_init() 启动
     * host task 后才真正完成。不能在这里同步检查 ENABLED 状态，否则会
     * 误判为 controller 初始化失败而提前返回。sync_cb 触发即表示就绪。 */
    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %s", esp_err_to_name(err));
        return err;
    }

    /* lcdfix23: ble_hs_util_ensure_addr(0) 已移至 ble_on_sync() 中，
     * 必须在 NimBLE host 启动并 sync 后调用才有效。此处调用时 controller
     * 尚未就绪，是空操作。地址设置详见 on_sync 注释。 */

    /* 注册 GATT 服务 */
    int rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "GATT count failed (rc=%d)", rc);
        nimble_port_deinit();
        return ESP_FAIL;
    }

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "GATT add failed (rc=%d)", rc);
        nimble_port_deinit();
        return ESP_FAIL;
    }

    /* 启动 NimBLE 主机任务 */
    nimble_port_freertos_init(ble_host_task);

    /* 创建数据发送任务 */
    BaseType_t task_created = xTaskCreatePinnedToCore(ble_tx_task, "ble_tx",
                                BLE_TX_TASK_STACK, NULL, BLE_TX_TASK_PRIO, NULL,
                                tskNO_AFFINITY);
    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "BLE TX task creation failed");
        nimble_port_deinit();
        return ESP_ERR_NO_MEM;
    }

    g_ble_initialized = true;
    ESP_LOGI(TAG, "BLE initialized");
    return ESP_OK;
}

void
crid_ble_write_cb(const char *data, size_t len, void *ctx)
{
    (void)ctx;

    if (!g_ble_initialized || !g_ble_tx_queue) {
        return;
    }
    if (!data || len == 0) return;

    if (len > BLE_TX_BUF_SIZE - 1) {
        len = BLE_TX_BUF_SIZE - 1;
    }

    if (g_nus_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
        return;
    }

    /* 按实际长度分配，减少 SPIRAM 碎片 */
    char *buf = (char *)malloc(len + 1);
    if (!buf) return;

    memcpy(buf, data, len);
    buf[len] = '\0';

    if (xQueueSend(g_ble_tx_queue, &buf, pdMS_TO_TICKS(10)) != pdTRUE) {
        free(buf);
        g_ble_queue_overflow_count++;
        if (g_ble_queue_overflow_count % 100 == 1) {
            ESP_LOGW(TAG, "BLE TX queue overflow (total: %lu)", (unsigned long)g_ble_queue_overflow_count);
        }
    }
}

bool
crid_ble_is_connected(void)
{
    return g_nus_conn_handle != BLE_HS_CONN_HANDLE_NONE;
}