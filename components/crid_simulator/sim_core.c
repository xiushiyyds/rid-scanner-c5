/**
 * sim_core.c — RID 伪造模拟器核心逻辑（多目标版）
 *
 * 支持最多 SIM_MAX_TARGETS(64) 架无人机同时模拟。
 * 每架无人机有独立 MAC、UAS_ID、轨迹和飞行参数。
 * TX task 轮询所有目标，每轮约 1 秒完成。
 *
 * MAC 地址前缀：FA:0B:BC (GB42590 中国 C-RID 标准)
 * VendorType：0x0D
 */

#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "esp_random.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "sim_core.h"
#include "sim_encode.h"
#include "sim_wifi.h"
#include "sim_patrol.h"

static const char *TAG = "SIM_CORE";

/* ================================================================
 * 内部状态
 * ================================================================ */
#define SIM_NVS_NAMESPACE "crid_sim"
#define SIM_TX_TASK_STACK 6144
#define SIM_TX_TASK_PRIO  5
#define SIM_FRAME_BUF_SIZE 512

/* 多目标帧间隔 (ms)：50 个目标 × 5ms = 250ms/轮，留余量给周期 */
#define SIM_PER_FRAME_DELAY_MS  5

static sim_state_t s_sim_state = SIM_STATE_IDLE;
static sim_config_t s_sim_config;
static SemaphoreHandle_t s_config_mutex = NULL;
static TaskHandle_t s_tx_task = NULL;
static volatile uint32_t s_tx_count = 0;
static uint8_t s_msg_counter = 0;

/* 第一架无人机的实时位置（向后兼容 sim_get_current_position） */
static volatile uint32_t s_current_lat_i32 = 0;
static volatile uint32_t s_current_lon_i32 = 0;
static volatile float s_current_heading = 0.0f;

/* ================================================================
 * 每个目标的独立状态
 * ================================================================ */
typedef struct {
    uint8_t mac[6];              // 唯一 MAC (FA:0B:BC + 3字节随机)
    char uas_id[21];             // 唯一 ID（真实序列号格式 + 随机后缀）
    sim_encode_config_t encode_cfg;
    double current_lat;
    double current_lon;
    float current_heading;
    float alt_offset;            // 高度随机偏移 ±10m
    float speed_offset;          // 速度随机偏移 ±20%
    double base_lat_offset;      // 无人机基准纬度偏移（在中心点周围）
    double base_lon_offset;      // 无人机基准经度偏移
    double op_lat_offset;        // 操作员纬度偏移（独立，50~500m）
    double op_lon_offset;        // 操作员经度偏移
    uint8_t channel;             // 该目标独立信道 (1~13)
} sim_target_t;

static sim_target_t s_targets[SIM_MAX_TARGETS];

/* ================================================================
 * 默认配置（大连市中心坐标）
 * ================================================================ */
void sim_get_default_config(sim_config_t *config) {
    if (!config) return;
    memset(config, 0, sizeof(sim_config_t));

    config->base_lat = 38.9140;     // 大连市中心
    config->base_lon = 121.6147;
    config->altitude_msl = 50.0f;
    config->altitude_agl = 50.0f;
    config->speed = 5.0f;
    config->channel = 6;
    config->flight_mode = SIM_MODE_CIRCLE;
    config->target_count = 1;       // 默认单目标（向后兼容）
    config->tx_power = 20;          // 20 × 0.25dBm = 5dBm
    strncpy(config->uas_id, "SIM-ESP32C5-001", sizeof(config->uas_id) - 1);
    strncpy(config->operator_id, "SIM-OP-001", sizeof(config->operator_id) - 1);
    strncpy(config->ssid, "NekolunaRID-SIM", sizeof(config->ssid) - 1);
}

const char *sim_flight_mode_name(sim_flight_mode_t mode) {
    switch (mode) {
        case SIM_MODE_CIRCLE:   return "CIRCLE";
        case SIM_MODE_PINGPONG: return "PINGPONG";
        case SIM_MODE_S_SEARCH: return "S_SEARCH";
        default:                return "UNKNOWN";
    }
}

/* ================================================================
 * 内部：生成唯一随机 MAC（FA:0B:BC 前缀）
 * 后 3 字节全部随机，并与已有目标去重
 * ================================================================ */
static void generate_unique_mac(uint8_t *mac, int target_idx) {
    (void)target_idx;
    mac[0] = 0xFA;
    mac[1] = 0x0B;
    mac[2] = 0xBC;

    for (int attempt = 0; attempt < 20; attempt++) {
        uint32_t r = esp_random();
        mac[3] = (r >> 16) & 0xFF;
        mac[4] = (r >> 8) & 0xFF;
        mac[5] = r & 0xFF;

        /* 与已初始化的目标去重 */
        bool dup = false;
        for (int j = 0; j < target_idx; j++) {
            if (memcmp(s_targets[j].mac, mac, 6) == 0) {
                dup = true;
                break;
            }
        }
        if (!dup) return;
    }
    /* 极端情况下 20 次都碰撞，用索引兜底 */
    mac[3] = (uint8_t)(target_idx & 0xFF);
    mac[4] = (uint8_t)(esp_random() & 0xFF);
    mac[5] = (uint8_t)(esp_random() & 0xFF);
}

/* ================================================================
 * 内部：生成随机 UAS ID
 * 从真实厂商序列号模板池中选模板，后缀用随机数填充，
 * 而不是用目标索引递增（避免被识别为模拟器）
 * ================================================================ */
static void generate_random_uas_id(char *out, size_t out_len, int target_idx) {
    /* 真实 DJI / Autel 序列号格式池（15字符左右） */
    static const char *id_templates[] = {
        "4TADL%c%c%c%03u%02u",   /* DJI Mini 4 Pro 类 */
        "4TADK%c%c%c%03u%02u",   /* DJI Air 3 类 */
        "8UUXN%c%c%c%03u%02u",   /* DJI M30T 类 */
        "4TADC%c%c%c%03u%02u",   /* DJI Mavic 3 类 */
        "89XHE%c%c%c%03u%02u",   /* Autel EVO II 类 */
        "4TADM%c%c%c%03u%02u",   /* DJI Mini 3 Pro 类 */
        "3IXDJ%c%c%c%03u%02u",   /* DJI Air 2S 类 */
        "4TADL%c%c%c%03u%02u",   /* DJI Mavic 3 Pro 类 */
        "6AXFL%c%c%c%03u%02u",   /* FIMI X8 SE 类 */
        "4TADN%c%c%c%03u%02u",   /* DJI Mini 2 类 */
        "8UUHG%c%c%c%03u%02u",   /* DJI Matrice 30 类 */
        "4TADV%c%c%c%03u%02u",   /* DJI Mavic Air 2 类 */
    };
    int tmpl_count = (int)(sizeof(id_templates) / sizeof(id_templates[0]));

    /* 用随机数选模板（不是 target_idx 取模） */
    int tmpl_idx = (int)(esp_random() % (uint32_t)tmpl_count);
    uint32_t r1 = esp_random();
    uint32_t r2 = esp_random();
    char c1 = 'A' + (r1 % 26);
    char c2 = 'A' + ((r1 >> 8) % 26);
    char c3 = 'A' + ((r1 >> 16) % 26);
    unsigned num1 = r2 % 1000;
    unsigned num2 = (r2 >> 16) % 100;

    snprintf(out, out_len, id_templates[tmpl_idx], c1, c2, c3, num1, num2);

    /* 与已有目标去重 */
    for (int attempt = 0; attempt < 20; attempt++) {
        bool dup = false;
        for (int j = 0; j < target_idx; j++) {
            if (strncmp(s_targets[j].uas_id, out, 20) == 0) {
                dup = true;
                break;
            }
        }
        if (!dup) return;
        /* 碰撞了，重新随机 */
        r1 = esp_random();
        r2 = esp_random();
        c1 = 'A' + (r1 % 26);
        c2 = 'A' + ((r1 >> 8) % 26);
        c3 = 'A' + ((r1 >> 16) % 26);
        num1 = r2 % 1000;
        num2 = (r2 >> 16) % 100;
        snprintf(out, out_len, id_templates[tmpl_idx], c1, c2, c3, num1, num2);
    }
}

/* ================================================================
 * 内部：初始化所有目标
 * ================================================================ */
static void init_targets(int target_count) {
    if (target_count < 1) target_count = 1;
    if (target_count > SIM_MAX_TARGETS) target_count = SIM_MAX_TARGETS;

    for (int i = 0; i < target_count; i++) {
        sim_target_t *t = &s_targets[i];

        /* 生成唯一随机 MAC（后 3 字节全随机+去重） */
        generate_unique_mac(t->mac, i);

        /* 生成随机真实序列号格式 UAS ID（非递增） */
        generate_random_uas_id(t->uas_id, sizeof(t->uas_id), i);

        /* 无人机基准坐标偏移：在中心点周围 0~200m 内随机散布 */
        float angle_rand = (float)((esp_random() % 36000) / 100.0f);
        float dist_rand = (float)((esp_random() % 2000) / 1000.0f);  /* 0~2.0, 最大 ~200m */
        t->base_lat_offset = 0.001 * dist_rand * cosf(angle_rand * 3.14159f / 180.0f);
        t->base_lon_offset = 0.001 * dist_rand * sinf(angle_rand * 3.14159f / 180.0f);

        /* 操作员位置偏移：与无人机独立，距离基准点 50~500m，随机方向 */
        /* 真实场景中飞手往往在无人机的一侧，距离几十米到几百米甚至更远 */
        float op_angle = (float)((esp_random() % 36000) / 100.0f) * 3.14159f / 180.0f;
        float op_dist_deg = 0.0005f + (float)((esp_random() % 4500) / 10000.0f); /* 0.0005~0.005 度 ≈ 50~500m */
        t->op_lat_offset = op_dist_deg * cosf(op_angle);
        t->op_lon_offset = op_dist_deg * sinf(op_angle);

        /* 高度随机偏移 ±10m */
        t->alt_offset = ((float)(esp_random() % 2000) / 100.0f) - 10.0f;

        /* 速度随机偏移 ±20% */
        t->speed_offset = 0.8f + ((float)(esp_random() % 400) / 1000.0f);

        /* 所有目标统一使用 s_sim_config.channel（固定信道）。
         * AP 模式下 esp_wifi_set_channel() 频繁切换会让 WiFi 控制器不稳定，
         * 且同一时间 sniffer 只能监听一个信道，随机信道的 Beacon 收不到。
         * 真实多无人机场景虽然会分散在不同信道，但 RID 压力测试优先保证
         * 稳定性和可检测性。*/
        t->channel = s_sim_config.channel ? s_sim_config.channel : 6;

        /* 初始位置和航向 */
        t->current_lat = s_sim_config.base_lat + t->base_lat_offset;
        t->current_lon = s_sim_config.base_lon + t->base_lon_offset;
        t->current_heading = 0.0f;

        ESP_LOGD(TAG, "Target %d: MAC=%02X:%02X:%02X:%02X:%02X:%02X ID=%s ch=%u "
                 "op_off=(%.6f,%.6f) alt_off=%.1f spd_off=%.2f",
                 i, t->mac[0], t->mac[1], t->mac[2], t->mac[3], t->mac[4], t->mac[5],
                 t->uas_id, t->channel,
                 t->op_lat_offset, t->op_lon_offset,
                 t->alt_offset, t->speed_offset);
    }
}

/* ================================================================
 * 内部：初始化路径引擎（所有目标）
 * ================================================================ */
static void init_patrol_engines(int target_count, sim_flight_mode_t mode, float speed) {
    for (int i = 0; i < target_count; i++) {
        sim_target_t *t = &s_targets[i];
        double inst_lat = s_sim_config.base_lat + t->base_lat_offset;
        double inst_lon = s_sim_config.base_lon + t->base_lon_offset;
        float inst_speed = speed * 0.1f * t->speed_offset;
        /* 每个目标有不同的相位偏移，让轨迹错开 */
        float phase = (float)i * (2.0f * 3.14159f / (float)target_count);

        sim_patrol_init_instance(i, inst_lat, inst_lon, inst_speed,
                                 (sim_patrol_mode_t)mode, phase);
    }
}

/* ================================================================
 * 内部：构建单个目标的编码器配置
 * ================================================================ */
static void build_target_encode_config(sim_target_t *t, const sim_config_t *cfg,
                                        double lat, double lon, float heading,
                                        float base_speed, int target_idx) {
    (void)target_idx;  /* 偏移已存在 t->op_* 中，不再需要索引 */
    sim_encode_config_t *enc = &t->encode_cfg;
    memset(enc, 0, sizeof(sim_encode_config_t));

    /* 复制该目标的唯一 MAC */
    memcpy(enc->mac_address, t->mac, 6);

    /* 该目标的唯一 UAS ID */
    strncpy(enc->uas_id, t->uas_id, SIM_UAS_ID_MAX_LEN);
    enc->uas_id[SIM_UAS_ID_MAX_LEN] = '\0';
    enc->id_type = 1;     /* ODID_IDTYPE_SERIAL_NUMBER */
    enc->ua_type = 2;     /* ODID_UATYPE_HELICOPTER_OR_MULTIROTOR (绝大多数消费级无人机) */

    /* 位置和速度（由路径引擎实时计算） */
    enc->latitude = (float)lat;
    enc->longitude = (float)lon;
    enc->altitude_msl = cfg->altitude_msl + t->alt_offset;
    enc->altitude_agl = cfg->altitude_agl + t->alt_offset;
    enc->speed_horizontal = base_speed * t->speed_offset;
    enc->speed_vertical = 0.0f;
    enc->heading = heading;
    enc->status = 1;  /* 空中 */

    /* 操作员位置：使用该目标独立的飞手偏移（50~500m 随机方向） */
    enc->operator_lat = (float)(cfg->base_lat + t->op_lat_offset);
    enc->operator_lon = (float)(cfg->base_lon + t->op_lon_offset);
    enc->operator_alt = 5.0f + (float)(esp_random() % 300) / 10.0f;  /* 5~35m 随机 */
    enc->operator_location_type = 1;  /* ODID_OPERATOR_LOCATION_TYPE_LIVE_GNSS */

    /* 分类信息 (模拟中国 C-RID 分类) */
    enc->classification_type = 2;   /* Category = 2 (轻型无人机, 最常见) */
    enc->category_eu = 2;
    enc->class_eu = 0;
    enc->height_type = 1;  /* ODID_HEIGHT_REF_OVER_GROUND (相对地面高度) */

    /* SSID 和信道（每个目标使用独立信道） */
    strncpy(enc->ssid, cfg->ssid, sizeof(enc->ssid) - 1);
    enc->ssid[sizeof(enc->ssid) - 1] = '\0';
    enc->channel = t->channel;
}

/* ================================================================
 * NVS 配置持久化
 * ================================================================ */
static esp_err_t nvs_load_config(sim_config_t *cfg) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(SIM_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        /* 无已保存配置，使用默认值 */
        sim_get_default_config(cfg);
        ESP_LOGI(TAG, "NVS empty, using defaults");
        return ESP_OK;
    }

    size_t size = sizeof(sim_config_t);
    err = nvs_get_blob(handle, "sim_cfg", cfg, &size);
    nvs_close(handle);

    if (err != ESP_OK) {
        sim_get_default_config(cfg);
        ESP_LOGW(TAG, "NVS load failed, using defaults");
    } else {
        /* 确保 target_count 在合法范围内 */
        if (cfg->target_count < 1 || cfg->target_count > SIM_MAX_TARGETS) {
            cfg->target_count = 1;
        }
        /* 确保 tx_power 合法 */
        if (cfg->tx_power < 0) {
            cfg->tx_power = 20;
        }
        ESP_LOGI(TAG, "Config loaded from NVS: targets=%d tx_power=%d",
                 cfg->target_count, cfg->tx_power);
    }
    return err;
}

static esp_err_t nvs_save_config(const sim_config_t *cfg) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(SIM_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;

    err = nvs_set_blob(handle, "sim_cfg", cfg, sizeof(sim_config_t));
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

/* ================================================================
 * 多目标 TX 任务
 * 轮询所有目标，每个目标间隔 5ms，一轮总周期约 1 秒
 * ================================================================ */
static void sim_tx_task(void *arg) {
    ESP_LOGI(TAG, "Multi-target TX task started");

    uint8_t frame_buf[SIM_FRAME_BUF_SIZE];

    while (1) {
        /* 每轮重新读取目标数量（响应 BLE SIM_CONFIG 实时更新） */
        if (s_config_mutex) xSemaphoreTake(s_config_mutex, portMAX_DELAY);
        int count = s_sim_config.target_count;
        if (s_config_mutex) xSemaphoreGive(s_config_mutex);

        TickType_t round_start_tick = xTaskGetTickCount();
        ESP_LOGD(TAG, "Round start: %d targets", count);

        for (int i = 0; i < count; i++) {
            sim_target_t *t = &s_targets[i];

            /* 获取当前路径位置 */
            double lat, lon;
            float heading;
            sim_patrol_next(i, &lat, &lon, &heading);

            t->current_lat = lat;
            t->current_lon = lon;
            t->current_heading = heading;

            /* 构建编码器配置 */
            if (s_config_mutex) {
                xSemaphoreTake(s_config_mutex, portMAX_DELAY);
            }

            build_target_encode_config(t, &s_sim_config, lat, lon, heading,
                                       s_sim_config.speed, i);

            /* 高度波动（每个目标独立） */
            float alt_wave = 3.0f * sinf((float)(s_tx_count + i * 7) * 0.1f);
            t->encode_cfg.altitude_msl += alt_wave;
            t->encode_cfg.altitude_agl += alt_wave;

            /* 速度波动 */
            float spd_wave = 0.9f + 0.1f * sinf((float)(s_tx_count + i * 13) * 0.05f);
            t->encode_cfg.speed_horizontal *= spd_wave;

            if (s_config_mutex) {
                xSemaphoreGive(s_config_mutex);
            }

            /* 更新第一架无人机的位置供外部查询 */
            if (i == 0) {
                s_current_lat_i32 = (uint32_t)(int32_t)(lat * 1e7);
                s_current_lon_i32 = (uint32_t)(int32_t)(lon * 1e7);
                s_current_heading = heading;
            }

            /* 所有目标统一信道（已在 init_targets 中设置为 config.channel），
             * 这里只做一次保护性切换，确认 AP 在正确信道上。 */
            static uint8_t s_last_tx_channel = 0;
            if (s_last_tx_channel != t->channel) {
                sim_wifi_set_channel(t->channel);
                s_last_tx_channel = t->channel;
            }

            /* 构建 Beacon 帧 */
            uint16_t frame_len = 0;
            uint8_t mc = s_msg_counter++;
            if (!sim_encode_beacon_frame(&t->encode_cfg, mc,
                                          frame_buf, SIM_FRAME_BUF_SIZE, &frame_len)) {
                ESP_LOGE(TAG, "Target %d frame build failed!", i);
                continue;
            }

            /* 发送 Beacon 帧 */
            esp_err_t ret = sim_wifi_send_raw_frame(frame_buf, frame_len);
            if (ret == ESP_OK) {
                s_tx_count++;
                if ((s_tx_count % (count * 60)) == 0) {
                    ESP_LOGI(TAG, "TX #%u (T%d): lat=%.6f lon=%.6f hdg=%.1f alt=%.1f",
                             (unsigned)s_tx_count, i, lat, lon, heading,
                             t->encode_cfg.altitude_msl);
                }
            } else {
                ESP_LOGW(TAG, "TX failed (T%d): %s", i, esp_err_to_name(ret));
            }

            /* 帧间隔：避免射频冲突 */
            vTaskDelay(pdMS_TO_TICKS(SIM_PER_FRAME_DELAY_MS));
        }

        /* 一轮结束后等待剩余时间，保持总周期约 1 秒 */
        TickType_t elapsed_ticks = xTaskGetTickCount() - round_start_tick;
        TickType_t target_ticks = pdMS_TO_TICKS(1000);
        if (elapsed_ticks < target_ticks) {
            vTaskDelay(target_ticks - elapsed_ticks);
        }
    }
}

/* ================================================================
 * 公开 API
 * ================================================================ */

esp_err_t sim_init(void) {
    if (s_sim_state != SIM_STATE_IDLE) {
        ESP_LOGW(TAG, "Already initialized (state=%d)", s_sim_state);
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing simulator core (multi-target)...");

    /* 创建配置互斥锁 */
    if (s_config_mutex == NULL) {
        s_config_mutex = xSemaphoreCreateMutex();
        if (s_config_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create config mutex");
            return ESP_ERR_NO_MEM;
        }
    }

    /* 从 NVS 加载配置（无则用默认值） */
    nvs_load_config(&s_sim_config);

    s_sim_state = SIM_STATE_STOPPED;
    s_tx_count = 0;
    s_msg_counter = 0;

    ESP_LOGI(TAG, "Simulator initialized. Default targets=%d, tx_power=%d (%.1f dBm)",
             s_sim_config.target_count, s_sim_config.tx_power,
             s_sim_config.tx_power * 0.25f);

    return ESP_OK;
}

esp_err_t sim_start(const sim_config_t *config) {
    if (s_sim_state == SIM_STATE_RUNNING) {
        ESP_LOGW(TAG, "Already running");
        return ESP_ERR_INVALID_STATE;
    }

    /* 更新配置 */
    if (config) {
        if (s_config_mutex) xSemaphoreTake(s_config_mutex, portMAX_DELAY);
        memcpy(&s_sim_config, config, sizeof(sim_config_t));
        if (s_config_mutex) xSemaphoreGive(s_config_mutex);
    }

    /* 确保 target_count 在合法范围 */
    if (s_sim_config.target_count < 1) s_sim_config.target_count = 1;
    if (s_sim_config.target_count > SIM_MAX_TARGETS) s_sim_config.target_count = SIM_MAX_TARGETS;

    int count = s_sim_config.target_count;

    ESP_LOGI(TAG, "Starting simulator: lat=%.6f lon=%.6f ch=%u mode=%s targets=%d tx_power=%d",
             s_sim_config.base_lat, s_sim_config.base_lon,
             s_sim_config.channel, sim_flight_mode_name(s_sim_config.flight_mode),
             count, s_sim_config.tx_power);

    /* 初始化 Wi-Fi AP 模式（传入 tx_power） */
    esp_err_t ret = sim_wifi_init(s_sim_config.channel, s_sim_config.ssid,
                                   s_sim_config.tx_power);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Wi-Fi init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 初始化所有目标（MAC、ID、偏移） */
    init_targets(count);

    /* 初始化路径引擎（每个目标独立实例） */
    init_patrol_engines(count, s_sim_config.flight_mode, s_sim_config.speed);

    /* 创建多目标 TX 任务 */
    BaseType_t task_ret = xTaskCreatePinnedToCore(sim_tx_task, "sim_tx",
                                SIM_TX_TASK_STACK, NULL, SIM_TX_TASK_PRIO,
                                &s_tx_task, 0);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create TX task");
        sim_wifi_deinit();
        return ESP_ERR_NO_MEM;
    }

    s_sim_state = SIM_STATE_RUNNING;

    /* 保存配置到 NVS */
    nvs_save_config(&s_sim_config);

    ESP_LOGI(TAG, "Simulator started: %d targets, ~%dms/round",
             count, count * SIM_PER_FRAME_DELAY_MS);
    return ESP_OK;
}

esp_err_t sim_stop(void) {
    if (s_sim_state != SIM_STATE_RUNNING) {
        ESP_LOGW(TAG, "Not running (state=%d)", s_sim_state);
        s_sim_state = SIM_STATE_STOPPED;
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Stopping simulator (%d targets)...", s_sim_config.target_count);

    /* lcdfix16: 先标记状态，再删 TX 任务，让任何还在发的帧知道我们要停了。
     * 注意不能在 s_tx_task 自己内部调 sim_stop（自删），目前都是 mode_switch_task
     * 外部调用，安全。 */
    s_sim_state = SIM_STATE_STOPPED;

    /* 停止发射任务。
     * vTaskDelete 不会等待任务退出，TX 任务可能正在 esp_wifi_80211_tx
     * 这个调用本身是阻塞且通常很快返回（<5ms），但为了保证 sim_wifi_deinit
     * 时 TX 任务已经完全退出，这里先 delete 再短暂 delay。 */
    if (s_tx_task) {
        vTaskDelete(s_tx_task);
        s_tx_task = NULL;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    /* 停止 Wi-Fi AP */
    sim_wifi_deinit();

    /* 重置所有路径引擎实例 */
    for (int i = 0; i < SIM_MAX_TARGETS; i++) {
        s_targets[i].encode_cfg.latitude = 0;
        s_targets[i].encode_cfg.longitude = 0;
    }

    ESP_LOGI(TAG, "Simulator stopped. Total TX: %u", (unsigned)s_tx_count);
    return ESP_OK;
}

sim_state_t sim_get_state(void) {
    return s_sim_state;
}

int sim_get_target_count(void) {
    return s_sim_config.target_count;
}

void sim_get_current_position(double *lat, double *lon, float *heading) {
    if (lat) *lat = (double)s_current_lat_i32 / 1e7;
    if (lon) *lon = (double)s_current_lon_i32 / 1e7;
    if (heading) *heading = s_current_heading;
}

uint32_t sim_get_tx_count(void) {
    return s_tx_count;
}

void sim_update_config(const sim_config_t *config) {
    if (!config) return;

    if (s_config_mutex) xSemaphoreTake(s_config_mutex, portMAX_DELAY);
    memcpy(&s_sim_config, config, sizeof(sim_config_t));
    if (s_config_mutex) xSemaphoreGive(s_config_mutex);

    /* 确保范围 */
    if (s_sim_config.target_count < 1) s_sim_config.target_count = 1;
    if (s_sim_config.target_count > SIM_MAX_TARGETS) s_sim_config.target_count = SIM_MAX_TARGETS;

    /* 实时更新路径引擎 */
    int count = s_sim_config.target_count;
    for (int i = 0; i < count; i++) {
        sim_target_t *t = &s_targets[i];
        double inst_lat = config->base_lat + t->base_lat_offset;
        double inst_lon = config->base_lon + t->base_lon_offset;
        float inst_speed = config->speed * 0.1f * t->speed_offset;
        float phase = (float)i * (2.0f * 3.14159f / (float)count);

        sim_patrol_update_instance(i, inst_lat, inst_lon, inst_speed,
                                    (sim_patrol_mode_t)config->flight_mode, phase);
    }

    /* 保存到 NVS */
    nvs_save_config(config);

    ESP_LOGI(TAG, "Config updated: lat=%.6f lon=%.6f mode=%s targets=%d tx_power=%d",
             config->base_lat, config->base_lon,
             sim_flight_mode_name(config->flight_mode),
             config->target_count, config->tx_power);
}
