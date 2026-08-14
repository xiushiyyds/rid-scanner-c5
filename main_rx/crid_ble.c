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
    ble_gap_adv_stop();
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

            /* 请求更低的连接间隔以提高吞吐量 (7.5ms~15ms) */
            struct ble_gap_upd_params params = {
                .itvl_min = 6,  /* 7.5ms = 6x1.25ms */
                .itvl_max = 12, /* 15ms = 12×1.25ms */
                .latency = 0,
                .supervision_timeout = 400,  /* 4秒 */
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
        ESP_LOGE(TAG, "Disconnected (reason=0x%04x)", event->disconnect.reason);
        g_nus_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        crid_ble_reset_pair();  /* v1.5: 断开时重置配对 */
        ble_advertise_start();
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ble_advertise_start();
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
 * 广播配置
 * ================================================================ */

static void
ble_advertise_start(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    const char *name = "RID-Scanner";

    memset(&fields, 0, sizeof(fields));
    memset(&adv_params, 0, sizeof(adv_params));

    /* 广播：设备名 + 标志（UUID 放扫描应答） */
    fields.name = (uint8_t *)name;
    fields.name_len = (uint8_t)strlen(name);
    fields.name_is_complete = 1;
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_set_fields failed (rc=%d)", rc);
        return;
    }

    /* 扫描应答：NUS 服务 UUID */
    struct ble_hs_adv_fields rsp_fields;
    memset(&rsp_fields, 0, sizeof(rsp_fields));
    rsp_fields.uuids128 = (ble_uuid128_t *)&gatt_svr_svc_nus_uuid;
    rsp_fields.num_uuids128 = 1;
    rsp_fields.uuids128_is_complete = 1;

    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_rsp_set_fields failed (rc=%d)", rc);
        return;
    }

    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                           &adv_params, ble_gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_start failed (rc=%d)", rc);
        return;
    }

    ESP_LOGI(TAG, "Advertising as '%s' with NUS UUID", name);

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
    vTaskDelay(pdMS_TO_TICKS(800));
    ESP_LOGI(TAG, "Starting BLE RID scan after advertising settle");
    crid_ble_scan_start();
    vTaskDelete(NULL);
}

/* ================================================================
 * NimBLE 同步回调 — 主机就绪后开始广播并查找句柄
 * ================================================================ */

static void
ble_on_sync(void)
{
    ESP_LOGI(TAG, "NimBLE host synced");

    /* 查找 TX characteristic 句柄（通知从此发送） */
    int rc = ble_gatts_find_chr(&gatt_svr_svc_nus_uuid.u,
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
                                2048, NULL, 4, NULL);
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

    /* 确保使用公共地址（BLE_OWN_ADDR_PUBLIC 需要身份地址已配置） */
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGW(TAG, "ble_hs_util_ensure_addr returned %d (non-fatal)", rc);
    }

    /* 注册 GATT 服务 */
    rc = ble_gatts_count_cfg(gatt_svr_svcs);
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