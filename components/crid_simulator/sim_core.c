/**
 * sim_core.c — RID 伪造模拟器核心逻辑 v2.6.1
 *
 * 新增：
 *   - 300 目标上限（v2.6.1）
 *   - 真实 SN 前缀库（Crockford Base32 后缀）
 *   - 多品牌：CRID / DJI / Autel / Parrot / Skydio / MIXED
 *   - 三信道轮发 ch1/6/11
 *   - 暂停/恢复
 *   - 串口 CLI
 *   - 运行时统计
 *   - 城市坐标表
 */

#include <string.h>
#include <strings.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_random.h"
#include "esp_mac.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "freertos/idf_additions.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "sim_core.h"
#include "sim_encode.h"
#include "sim_encode_gb46750.h"
#include "sim_wifi.h"
#include "sim_patrol.h"
#include "drone_sn_db.h"
#include "sim_cities.h"

static const char *TAG = "SIM_CORE";

/* ================================================================
 * 内部常量
 * ================================================================ */
#define SIM_NVS_NAMESPACE "crid_sim"
#define SIM_TX_TASK_STACK 8192
#define SIM_TX_TASK_PRIO  5
#define SIM_FRAME_BUF_SIZE 512
#define SIM_CLI_BUF_SIZE   128

#define SIM_ROTATE_CHANNELS {6, 1, 11}
#define SIM_ROTATE_CH_COUNT 3

/* Crockford Base32（DJI SN 实际使用，排除 I/O/L 易混字符） */
static const char CROCKFORD32[] = "0123456789ABCDEFGHJKMNPQRSTVWXYZ";

/* 品牌 OUI 映射 */
static const uint8_t BRAND_OUI[][3] = {
    [SIM_BRAND_CRID]   = {0xFA, 0x0B, 0xBC},
    [SIM_BRAND_DJI]    = {0x26, 0x37, 0x12},
    [SIM_BRAND_AUTEL]  = {0x48, 0x15, 0xA6},
    [SIM_BRAND_PARROT] = {0x90, 0x3A, 0x72},
    [SIM_BRAND_SKYDIO] = {0xD0, 0x15, 0x5A},
};

/* ================================================================
 * 内部状态
 * ================================================================ */
static sim_state_t s_sim_state = SIM_STATE_IDLE;
static sim_config_t s_sim_config;
static SemaphoreHandle_t s_config_mutex = NULL;
static TaskHandle_t s_tx_task = NULL;
static volatile uint32_t s_tx_count = 0;
static volatile uint32_t s_tx_fail = 0;
static volatile uint32_t s_rounds = 0;
static uint8_t s_msg_counter = 0;
static uint32_t s_start_tick = 0;
static uint8_t s_current_channel = 6;

static volatile bool s_tx_should_stop = false;
static volatile bool s_tx_paused = false;

/* 第一架无人机的实时位置 */
static volatile double s_current_lat = 0;
static volatile double s_current_lon = 0;
static volatile float s_current_heading = 0.0f;

/* CLI */
static char s_cli_buf[SIM_CLI_BUF_SIZE];
static int s_cli_len = 0;

/* ================================================================
 * 每个目标的独立状态
 * ================================================================ */
typedef struct {
    uint8_t mac[6];
    char uas_id[21];
    const drone_sn_entry_t *sn_entry;  /* 指向 SN 库条目（MIXED 模式） */
    sim_encode_config_t encode_cfg;
    double current_lat;
    double current_lon;
    float current_heading;
    float alt_offset;
    float speed_offset;
    double base_lat_offset;
    double base_lon_offset;
    double op_lat_offset;
    double op_lon_offset;
    uint8_t channel;
    sim_brand_t brand;
    char upc[GB46750_UPC_LEN + 1];  /* v2.7.0: GB46750 UPC (20 chars) */
} sim_target_t;

static EXT_RAM_BSS_ATTR sim_target_t s_targets[SIM_MAX_TARGETS];

/* ================================================================
 * 名称工具
 * ================================================================ */
const char *sim_brand_name(sim_brand_t b) {
    switch (b) {
        case SIM_BRAND_CRID:   return "CRID";
        case SIM_BRAND_DJI:    return "DJI";
        case SIM_BRAND_AUTEL:  return "Autel";
        case SIM_BRAND_PARROT: return "Parrot";
        case SIM_BRAND_SKYDIO: return "Skydio";
        case SIM_BRAND_MIXED:  return "Mixed";
        default:               return "?";
    }
}

const char *sim_chan_mode_name(sim_chan_mode_t m) {
    switch (m) {
        case SIM_CHAN_SINGLE:         return "single";
        case SIM_CHAN_ROTATE_1_6_11:  return "rotate 1/6/11";
        default:                       return "?";
    }
}

const char *sim_protocol_name(sim_protocol_t p) {
    switch (p) {
        case SIM_PROTO_GB42590: return "GB42590";
        case SIM_PROTO_GB46750: return "GB46750";
        case SIM_PROTO_MIXED:   return "Mixed";
        default:                return "?";
    }
}

/* ================================================================
 * 默认配置
 * ================================================================ */
void sim_get_default_config(sim_config_t *config) {
    if (!config) return;
    memset(config, 0, sizeof(sim_config_t));
    config->base_lat = g_sim_cities[12].lat;   /* 大连 */
    config->base_lon = g_sim_cities[12].lon;
    config->altitude_msl = 50.0f;
    config->altitude_agl = 50.0f;
    config->speed = 5.0f;
    config->channel = 6;
    config->flight_mode = SIM_MODE_CIRCLE;
    config->target_count = 300;
    config->tx_power = 20;
    strncpy(config->uas_id, "SIM-C5-001", sizeof(config->uas_id) - 1);
    strncpy(config->operator_id, "OP-C5-001", sizeof(config->operator_id) - 1);
    strncpy(config->ssid, "RID-SIM-C5", sizeof(config->ssid) - 1);
    config->brand = SIM_BRAND_MIXED;
    config->chan_mode = SIM_CHAN_ROTATE_1_6_11;
    config->protocol = SIM_PROTO_GB42590;  /* 默认旧国标，不回归 */
    config->frame_interval_ms = 3;
    config->round_interval_ms = 1000;
    config->city_index = 12;
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
 * Crockford Base32 随机后缀生成
 * ================================================================ */
static void crockford_append(char *out, size_t out_len, int n_chars) {
    size_t cur = strlen(out);
    for (int i = 0; i < n_chars && cur + 1 < out_len; i++) {
        out[cur++] = CROCKFORD32[esp_random() % 32];
    }
    out[cur] = '\0';
}

/* ================================================================
 * 为目标分配品牌
 * ================================================================ */
static sim_brand_t pick_brand(sim_brand_t configured) {
    if (configured != SIM_BRAND_MIXED) return configured;
    /* MIXED 模式：70% DJI，10% Autel，8% CRID，7% Parrot，5% Skydio */
    uint32_t r = esp_random() % 100;
    if (r < 70) return SIM_BRAND_DJI;
    if (r < 80) return SIM_BRAND_AUTEL;
    if (r < 88) return SIM_BRAND_CRID;
    if (r < 95) return SIM_BRAND_PARROT;
    return SIM_BRAND_SKYDIO;
}

/* ================================================================
 * 生成唯一 MAC（按品牌 OUI）
 * ================================================================ */
static void generate_unique_mac(uint8_t *mac, sim_brand_t brand, int target_idx) {
    const uint8_t *oui = BRAND_OUI[brand];
    memcpy(mac, oui, 3);
    for (int attempt = 0; attempt < 20; attempt++) {
        uint32_t r = esp_random();
        mac[3] = (r >> 16) & 0xFF;
        mac[4] = (r >> 8) & 0xFF;
        mac[5] = r & 0xFF;
        bool dup = false;
        for (int j = 0; j < target_idx; j++) {
            if (memcmp(s_targets[j].mac, mac, 6) == 0) { dup = true; break; }
        }
        if (!dup) return;
    }
    mac[3] = (uint8_t)(target_idx & 0xFF);
    mac[4] = (uint8_t)(esp_random() & 0xFF);
    mac[5] = (uint8_t)(esp_random() & 0xFF);
}

/* ================================================================
 * 生成真实 SN（从 SN 前缀库 + Crockford 后缀）
 * ================================================================ */
static const drone_sn_entry_t *pick_sn_entry(sim_brand_t brand) {
    /* 如果是 DJI/CRID（大疆为主），从库里按类别随机选 */
    /* 其他品牌也在库里有前缀（Autel/Parrot/Skydio），按品牌过滤 */
    static int indices[DRONE_SN_DB_COUNT];
    static bool init = false;
    if (!init) {
        for (int i = 0; i < DRONE_SN_DB_COUNT; i++) indices[i] = i;
        init = true;
    }
    /* 简单做法：随机取一个前缀，不做严格品牌过滤（库里前缀本身对应品牌） */
    return &drone_sn_db[esp_random() % DRONE_SN_DB_COUNT];
}

static void generate_realistic_sn(char *out, size_t out_len, const drone_sn_entry_t *entry) {
    /* 前缀 8 字符 + Crockford 随机后缀，总长 15~18（与真机 SN 长度一致） */
    snprintf(out, out_len, "%s", entry->prefix);
    int suffix_len = 8 + (esp_random() % 4);  /* 8~11 字符后缀 */
    crockford_append(out, out_len, suffix_len);
}

/* ================================================================
 * 初始化所有目标
 * ================================================================ */
static void init_targets(int target_count) {
    if (target_count < 1) target_count = 1;
    if (target_count > SIM_MAX_TARGETS) target_count = SIM_MAX_TARGETS;

    for (int i = 0; i < target_count; i++) {
        sim_target_t *t = &s_targets[i];
        memset(t, 0, sizeof(*t));

        t->brand = pick_brand(s_sim_config.brand);
        generate_unique_mac(t->mac, t->brand, i);
        t->sn_entry = pick_sn_entry(t->brand);
        generate_realistic_sn(t->uas_id, sizeof(t->uas_id), t->sn_entry);

        /* v2.7.0: 为 GB46750 预生成 20 位 UPC */
        sim_gb46750_gen_upc(t->brand, (uint32_t)i, t->upc, sizeof(t->upc));

        float angle_rand = (float)((esp_random() % 36000) / 100.0f);
        float dist_rand = (float)((esp_random() % 2000) / 1000.0f);
        t->base_lat_offset = 0.001 * dist_rand * cosf(angle_rand * 3.14159f / 180.0f);
        t->base_lon_offset = 0.001 * dist_rand * sinf(angle_rand * 3.14159f / 180.0f);

        float op_angle = (float)((esp_random() % 36000) / 100.0f) * 3.14159f / 180.0f;
        float op_dist_deg = 0.0005f + (float)((esp_random() % 4500) / 10000.0f);
        t->op_lat_offset = op_dist_deg * cosf(op_angle);
        t->op_lon_offset = op_dist_deg * sinf(op_angle);

        t->alt_offset = ((float)(esp_random() % 2000) / 100.0f) - 10.0f;
        t->speed_offset = 0.8f + ((float)(esp_random() % 400) / 1000.0f);

        if (s_sim_config.chan_mode == SIM_CHAN_ROTATE_1_6_11) {
            static const uint8_t chs[] = SIM_ROTATE_CHANNELS;
            t->channel = chs[i % SIM_ROTATE_CH_COUNT];
        } else {
            t->channel = s_sim_config.channel;
        }

        t->current_lat = s_sim_config.base_lat + t->base_lat_offset;
        t->current_lon = s_sim_config.base_lon + t->base_lon_offset;
        t->current_heading = 0.0f;

        ESP_LOGI(TAG, "T%d: brand=%s mac=%02X:%02X:%02X:%02X:%02X:%02X sn=%s model=%s ch=%u",
                 i, sim_brand_name(t->brand),
                 t->mac[0], t->mac[1], t->mac[2], t->mac[3], t->mac[4], t->mac[5],
                 t->uas_id, t->sn_entry->model, t->channel);
    }
}

static void init_patrol_engines(int target_count, sim_flight_mode_t mode, float speed) {
    for (int i = 0; i < target_count; i++) {
        sim_target_t *t = &s_targets[i];
        double inst_lat = s_sim_config.base_lat + t->base_lat_offset;
        double inst_lon = s_sim_config.base_lon + t->base_lon_offset;
        float inst_speed = speed * 0.1f * t->speed_offset;
        float phase = (float)i * (2.0f * 3.14159f / (float)target_count);
        sim_patrol_init_instance(i, inst_lat, inst_lon, inst_speed,
                                 (sim_patrol_mode_t)mode, phase);
    }
}

static void build_target_encode_config(sim_target_t *t, const sim_config_t *cfg,
                                        double lat, double lon, float heading,
                                        float base_speed) {
    sim_encode_config_t *enc = &t->encode_cfg;
    memset(enc, 0, sizeof(sim_encode_config_t));
    memcpy(enc->mac_address, t->mac, 6);
    strncpy(enc->uas_id, t->uas_id, SIM_UAS_ID_MAX_LEN);
    enc->uas_id[SIM_UAS_ID_MAX_LEN] = '\0';
    enc->id_type = 1;
    enc->ua_type = 2;
    enc->latitude = (float)lat;
    enc->longitude = (float)lon;
    enc->altitude_msl = cfg->altitude_msl + t->alt_offset;
    enc->altitude_agl = cfg->altitude_agl + t->alt_offset;
    enc->speed_horizontal = base_speed * t->speed_offset;
    enc->speed_vertical = 0.0f;
    enc->heading = heading;
    enc->status = 1;
    enc->operator_lat = (float)(cfg->base_lat + t->op_lat_offset);
    enc->operator_lon = (float)(cfg->base_lon + t->op_lon_offset);
    enc->operator_alt = 5.0f + (float)(esp_random() % 300) / 10.0f;
    enc->operator_location_type = 1;
    enc->classification_type = 2;
    enc->category_eu = 2;
    enc->class_eu = 0;
    enc->height_type = 1;
    strncpy(enc->ssid, cfg->ssid, sizeof(enc->ssid) - 1);
    enc->ssid[sizeof(enc->ssid) - 1] = '\0';
    enc->channel = t->channel;
}

/* ================================================================
 * NVS
 * ================================================================ */
static esp_err_t nvs_load_config(sim_config_t *cfg) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(SIM_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        sim_get_default_config(cfg);
        return ESP_OK;
    }
    size_t size = sizeof(sim_config_t);
    err = nvs_get_blob(handle, "sim_cfg", cfg, &size);
    nvs_close(handle);
    if (err != ESP_OK || size != sizeof(sim_config_t)) {
        /* v2.7.0: 结构体新增 protocol 字段，旧 NVS blob 尺寸不匹配，
         * 走默认值避免读取越界/字段错位 */
        sim_get_default_config(cfg);
    } else {
        if (cfg->target_count < 1 || cfg->target_count > SIM_MAX_TARGETS)
            cfg->target_count = 10;
        if (cfg->tx_power < 0) cfg->tx_power = 20;
        if (cfg->frame_interval_ms <= 0) cfg->frame_interval_ms = 5;
        if (cfg->round_interval_ms <= 0) cfg->round_interval_ms = 1000;
        /* 兜底：protocol 枚举越界（异常 NVS）*/
        if (cfg->protocol > SIM_PROTO_MIXED)
            cfg->protocol = SIM_PROTO_GB42590;
    }
    return err;
}

static esp_err_t nvs_save_config(const sim_config_t *cfg) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(SIM_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;
    err = nvs_set_blob(handle, "sim_cfg", cfg, sizeof(sim_config_t));
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    return err;
}

/* ================================================================
 * TX 任务（v2.6.2：信道分组轮发，避免逐架切信道）
 *
 * v2.6.0/v2.6.1 在 ROTATE 模式下对每个目标都调用 esp_wifi_set_channel，
 * 300 架 = 300 次切信道，每次 1~5ms，整轮被拖到 1.5~2.5s。
 * 本版按 init_targets() 里同样的 i%3 分组规则，把同一信道的目标
 * 集中在一个分组里连续发送，每轮只切 2 次信道。
 * ================================================================ */

/* 发送单个目标的一帧（已持锁/不持锁由调用方决定），成功返回 true */
static inline bool tx_send_one(int i, uint8_t *frame_buf) {
    sim_target_t *t = &s_targets[i];
    double lat, lon;
    float heading;
    sim_patrol_next(i, &lat, &lon, &heading);
    t->current_lat = lat;
    t->current_lon = lon;
    t->current_heading = heading;

    /* 读取当前协议选择（v2.7.0） */
    sim_protocol_t proto;
    if (s_config_mutex) xSemaphoreTake(s_config_mutex, portMAX_DELAY);
    build_target_encode_config(t, &s_sim_config, lat, lon, heading,
                               s_sim_config.speed);
    proto = s_sim_config.protocol;
    if (s_config_mutex) xSemaphoreGive(s_config_mutex);

    float alt_wave = 3.0f * sinf((float)(s_tx_count + i * 7) * 0.1f);
    t->encode_cfg.altitude_msl += alt_wave;
    t->encode_cfg.altitude_agl += alt_wave;
    float spd_wave = 0.9f + 0.1f * sinf((float)(s_tx_count + i * 13) * 0.05f);
    t->encode_cfg.speed_horizontal *= spd_wave;

    if (i == 0) {
        s_current_lat = lat;
        s_current_lon = lon;
        s_current_heading = heading;
    }

    uint16_t frame_len = 0;
    uint8_t mc = s_msg_counter++;
    bool ok;

    if (proto == SIM_PROTO_GB46750) {
        ok = sim_encode_gb46750_beacon_frame(&t->encode_cfg, t->upc,
                                              mc, frame_buf,
                                              SIM_FRAME_BUF_SIZE, &frame_len);
    } else if (proto == SIM_PROTO_MIXED) {
        /* 轮替：偶数 counter 发 GB42590，奇数发 GB46750 */
        if (mc & 1) {
            ok = sim_encode_gb46750_beacon_frame(&t->encode_cfg, t->upc,
                                                  mc, frame_buf,
                                                  SIM_FRAME_BUF_SIZE, &frame_len);
        } else {
            ok = sim_encode_beacon_frame(&t->encode_cfg, mc, frame_buf,
                                          SIM_FRAME_BUF_SIZE, &frame_len);
        }
    } else {
        ok = sim_encode_beacon_frame(&t->encode_cfg, mc, frame_buf,
                                      SIM_FRAME_BUF_SIZE, &frame_len);
    }

    if (!ok) return false;
    return sim_wifi_send_raw_frame(frame_buf, frame_len) == ESP_OK;
}

static void sim_tx_task(void *arg) {
    ESP_LOGI(TAG, "TX task started (v2.6.2 grouped rotate)");
    uint8_t frame_buf[SIM_FRAME_BUF_SIZE];
    static const uint8_t ROTATE_CH[] = SIM_ROTATE_CHANNELS;

    while (!s_tx_should_stop) {
        if (s_tx_paused) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        int count;
        int frame_interval;
        int round_interval;
        sim_chan_mode_t chan_mode;
        if (s_config_mutex) xSemaphoreTake(s_config_mutex, portMAX_DELAY);
        count = s_sim_config.target_count;
        frame_interval = s_sim_config.frame_interval_ms;
        round_interval = s_sim_config.round_interval_ms;
        chan_mode = s_sim_config.chan_mode;
        if (s_config_mutex) xSemaphoreGive(s_config_mutex);

        if (count < 1) count = 1;
        if (count > SIM_MAX_TARGETS) count = SIM_MAX_TARGETS;

        TickType_t round_start = xTaskGetTickCount();

        if (chan_mode == SIM_CHAN_ROTATE_1_6_11) {
            /* 分组轮发：ch6 组 (i%3==0) -> ch1 组 (i%3==1) -> ch11 组 (i%3==2)
             * 与 init_targets() 中 t->channel = chs[i%3] 的分配严格一致 */
            for (int g = 0; g < SIM_ROTATE_CH_COUNT && !s_tx_should_stop; g++) {
                if (s_tx_paused) break;
                uint8_t ch = ROTATE_CH[g];
                if (s_current_channel != ch) {
                    sim_wifi_set_channel(ch);
                    s_current_channel = ch;
                }
                for (int i = g; i < count; i += SIM_ROTATE_CH_COUNT) {
                    if (s_tx_paused || s_tx_should_stop) break;
                    if (tx_send_one(i, frame_buf)) s_tx_count++;
                    else s_tx_fail++;
                    vTaskDelay(pdMS_TO_TICKS(frame_interval));
                }
            }
        } else {
            /* 单信道：只在开始时切一次 */
            uint8_t ch = s_sim_config.channel;
            if (s_current_channel != ch) {
                sim_wifi_set_channel(ch);
                s_current_channel = ch;
            }
            for (int i = 0; i < count && !s_tx_should_stop; i++) {
                if (s_tx_paused) break;
                if (tx_send_one(i, frame_buf)) s_tx_count++;
                else s_tx_fail++;
                vTaskDelay(pdMS_TO_TICKS(frame_interval));
            }
        }

        s_rounds++;

        if (!s_tx_should_stop) {
            TickType_t elapsed = xTaskGetTickCount() - round_start;
            TickType_t target = pdMS_TO_TICKS(round_interval);
            if (elapsed < target) {
                vTaskDelay(target - elapsed);
            }
        }
    }

    s_tx_task = NULL;
    vTaskDelete(NULL);
}

/* ================================================================
 * 公开 API
 * ================================================================ */
esp_err_t sim_init(void) {
    if (s_sim_state != SIM_STATE_IDLE) return ESP_OK;
    ESP_LOGI(TAG, "Initializing simulator v2.7.0...");

    s_config_mutex = xSemaphoreCreateMutex();
    if (!s_config_mutex) return ESP_ERR_NO_MEM;

    nvs_load_config(&s_sim_config);
    s_sim_state = SIM_STATE_STOPPED;
    s_tx_count = 0;
    s_tx_fail = 0;
    s_rounds = 0;
    s_cli_len = 0;

    ESP_LOGI(TAG, "Init OK: default city=%s targets=%d brand=%s chan=%s",
             g_sim_cities[s_sim_config.city_index >= 0 && s_sim_config.city_index < SIM_CITY_COUNT
                          ? s_sim_config.city_index : 12].name,
             s_sim_config.target_count,
             sim_brand_name(s_sim_config.brand),
             sim_chan_mode_name(s_sim_config.chan_mode));
    return ESP_OK;
}

esp_err_t sim_start(const sim_config_t *config) {
    if (s_sim_state == SIM_STATE_RUNNING) return ESP_ERR_INVALID_STATE;

    if (config) {
        if (s_config_mutex) xSemaphoreTake(s_config_mutex, portMAX_DELAY);
        memcpy(&s_sim_config, config, sizeof(sim_config_t));
        if (s_config_mutex) xSemaphoreGive(s_config_mutex);
    }

    if (s_sim_config.target_count < 1) s_sim_config.target_count = 1;
    if (s_sim_config.target_count > SIM_MAX_TARGETS) s_sim_config.target_count = SIM_MAX_TARGETS;
    if (s_sim_config.frame_interval_ms <= 0) s_sim_config.frame_interval_ms = 5;
    if (s_sim_config.round_interval_ms <= 0) s_sim_config.round_interval_ms = 1000;

    /* 城市索引 -> 坐标（如果 city_index >= 0） */
    if (s_sim_config.city_index >= 0 && s_sim_config.city_index < SIM_CITY_COUNT) {
        s_sim_config.base_lat = g_sim_cities[s_sim_config.city_index].lat;
        s_sim_config.base_lon = g_sim_cities[s_sim_config.city_index].lon;
    }

    int count = s_sim_config.target_count;
    ESP_LOGI(TAG, "Starting: %s (%.6f,%.6f) ch=%d brand=%s chan=%s count=%d power=%d",
             s_sim_config.city_index >= 0 ? g_sim_cities[s_sim_config.city_index].name : "custom",
             s_sim_config.base_lat, s_sim_config.base_lon,
             s_sim_config.channel, sim_brand_name(s_sim_config.brand),
             sim_chan_mode_name(s_sim_config.chan_mode),
             count, s_sim_config.tx_power);

    esp_err_t ret = sim_wifi_init(s_sim_config.channel, s_sim_config.ssid,
                                   s_sim_config.tx_power);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    s_current_channel = s_sim_config.channel;
    init_targets(count);
    init_patrol_engines(count, s_sim_config.flight_mode, s_sim_config.speed);

    s_tx_should_stop = false;
    s_tx_paused = false;
    s_start_tick = xTaskGetTickCount();

    BaseType_t ok = xTaskCreatePinnedToCore(sim_tx_task, "sim_tx",
                                SIM_TX_TASK_STACK, NULL, SIM_TX_TASK_PRIO,
                                &s_tx_task, 0);
    if (ok != pdPASS) {
        /* PSRAM 栈 fallback：xTaskCreateWithCaps 不支持 pinned，
         * 但 TX 任务绑核不关键，用无 pin 版本在 PSRAM 上创建 */
        ok = xTaskCreateWithCaps(sim_tx_task, "sim_tx",
                                SIM_TX_TASK_STACK, NULL, SIM_TX_TASK_PRIO,
                                &s_tx_task, MALLOC_CAP_SPIRAM);
    }
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create TX task");
        sim_wifi_deinit();
        return ESP_ERR_NO_MEM;
    }

    s_sim_state = SIM_STATE_RUNNING;
    nvs_save_config(&s_sim_config);
    ESP_LOGI(TAG, "Started: %d targets", count);
    return ESP_OK;
}

esp_err_t sim_stop(void) {
    if (s_sim_state != SIM_STATE_RUNNING && s_sim_state != SIM_STATE_PAUSED) {
        s_sim_state = SIM_STATE_STOPPED;
        return ESP_OK;
    }
    ESP_LOGI(TAG, "Stopping...");
    s_sim_state = SIM_STATE_STOPPED;
    s_tx_should_stop = true;
    s_tx_paused = false;
    if (s_tx_task) {
        for (int i = 0; i < 50 && s_tx_task; i++)
            vTaskDelay(pdMS_TO_TICKS(20));
    }
    sim_wifi_deinit();
    for (int i = 0; i < SIM_MAX_TARGETS; i++) {
        s_targets[i].encode_cfg.latitude = 0;
        s_targets[i].encode_cfg.longitude = 0;
    }
    ESP_LOGI(TAG, "Stopped. TX=%lu fail=%lu rounds=%lu",
             (unsigned long)s_tx_count, (unsigned long)s_tx_fail,
             (unsigned long)s_rounds);
    return ESP_OK;
}

esp_err_t sim_pause(void) {
    if (s_sim_state != SIM_STATE_RUNNING) return ESP_ERR_INVALID_STATE;
    s_tx_paused = true;
    s_sim_state = SIM_STATE_PAUSED;
    ESP_LOGI(TAG, "Paused");
    return ESP_OK;
}

esp_err_t sim_resume(void) {
    if (s_sim_state != SIM_STATE_PAUSED) return ESP_ERR_INVALID_STATE;
    s_tx_paused = false;
    s_sim_state = SIM_STATE_RUNNING;
    ESP_LOGI(TAG, "Resumed");
    return ESP_OK;
}

sim_state_t sim_get_state(void) { return s_sim_state; }
int sim_get_target_count(void) { return s_sim_config.target_count; }

void sim_get_current_position(double *lat, double *lon, float *heading) {
    if (lat) *lat = s_current_lat;
    if (lon) *lon = s_current_lon;
    if (heading) *heading = s_current_heading;
}

uint32_t sim_get_tx_count(void) { return s_tx_count; }

void sim_get_stats(sim_stats_t *out) {
    if (!out) return;
    memset(out, 0, sizeof(*out));
    out->tx_count = s_tx_count;
    out->tx_fail = s_tx_fail;
    out->rounds = s_rounds;
    out->active_count = s_sim_config.target_count;
    out->current_channel = s_current_channel;
    out->state = s_sim_state;
    out->brand = s_sim_config.brand;
    out->chan_mode = s_sim_config.chan_mode;
    out->protocol = s_sim_config.protocol;
    out->speed = s_sim_config.speed;
    out->base_lat = s_sim_config.base_lat;
    out->base_lon = s_sim_config.base_lon;
    out->uptime_s = (xTaskGetTickCount() - s_start_tick) * portTICK_PERIOD_MS / 1000;
}

void sim_update_config(const sim_config_t *config) {
    if (!config) return;
    if (s_config_mutex) xSemaphoreTake(s_config_mutex, portMAX_DELAY);
    int old_count = s_sim_config.target_count;
    memcpy(&s_sim_config, config, sizeof(sim_config_t));
    if (s_config_mutex) xSemaphoreGive(s_config_mutex);
    if (s_sim_config.target_count < 1) s_sim_config.target_count = 1;
    if (s_sim_config.target_count > SIM_MAX_TARGETS) s_sim_config.target_count = SIM_MAX_TARGETS;
    if (s_sim_config.target_count > old_count) {
        int new_count = s_sim_config.target_count;
        for (int i = old_count; i < new_count; i++) {
            sim_target_t *t = &s_targets[i];
            memset(t, 0, sizeof(*t));
            t->brand = pick_brand(s_sim_config.brand);
            generate_unique_mac(t->mac, t->brand, i);
            t->sn_entry = pick_sn_entry(t->brand);
            generate_realistic_sn(t->uas_id, sizeof(t->uas_id), t->sn_entry);
            float a = (float)((esp_random() % 36000) / 100.0f);
            float d = (float)((esp_random() % 2000) / 1000.0f);
            t->base_lat_offset = 0.001 * d * cosf(a * 3.14159f / 180.0f);
            t->base_lon_offset = 0.001 * d * sinf(a * 3.14159f / 180.0f);
            float oa = (float)((esp_random() % 36000) / 100.0f) * 3.14159f / 180.0f;
            float od = 0.0005f + (float)((esp_random() % 4500) / 10000.0f);
            t->op_lat_offset = od * cosf(oa);
            t->op_lon_offset = od * sinf(oa);
            t->alt_offset = ((float)(esp_random() % 2000) / 100.0f) - 10.0f;
            t->speed_offset = 0.8f + ((float)(esp_random() % 400) / 1000.0f);
            if (s_sim_config.chan_mode == SIM_CHAN_ROTATE_1_6_11) {
                static const uint8_t chs[] = SIM_ROTATE_CHANNELS;
                t->channel = chs[i % SIM_ROTATE_CH_COUNT];
            } else {
                t->channel = s_sim_config.channel;
            }
            t->current_lat = s_sim_config.base_lat + t->base_lat_offset;
            t->current_lon = s_sim_config.base_lon + t->base_lon_offset;
            double il = s_sim_config.base_lat + t->base_lat_offset;
            double in = s_sim_config.base_lon + t->base_lon_offset;
            float isp = s_sim_config.speed * 0.1f * t->speed_offset;
            float ph = (float)i * (2.0f * 3.14159f / (float)new_count);
            sim_patrol_init_instance(i, il, in, isp,
                                     (sim_patrol_mode_t)s_sim_config.flight_mode, ph);
        }
    }
    nvs_save_config(config);
}

/* ================================================================
 * 运行时控制（CLI 用）
 * ================================================================ */
void sim_set_count(int count) {
    if (count < 1) count = 1;
    if (count > SIM_MAX_TARGETS) count = SIM_MAX_TARGETS;
    if (s_config_mutex) xSemaphoreTake(s_config_mutex, portMAX_DELAY);
    s_sim_config.target_count = count;
    if (s_config_mutex) xSemaphoreGive(s_config_mutex);
    ESP_LOGI(TAG, "count -> %d", count);
}

void sim_set_speed(float speed) {
    if (s_config_mutex) xSemaphoreTake(s_config_mutex, portMAX_DELAY);
    s_sim_config.speed = speed;
    if (s_config_mutex) xSemaphoreGive(s_config_mutex);
    ESP_LOGI(TAG, "speed -> %.1f m/s", speed);
}

void sim_set_center(double lat, double lon) {
    if (s_config_mutex) xSemaphoreTake(s_config_mutex, portMAX_DELAY);
    s_sim_config.base_lat = lat;
    s_sim_config.base_lon = lon;
    s_sim_config.city_index = -1;
    if (s_config_mutex) xSemaphoreGive(s_config_mutex);
    ESP_LOGI(TAG, "center -> %.6f,%.6f", lat, lon);
}

void sim_set_channel(uint8_t ch) {
    if (s_config_mutex) xSemaphoreTake(s_config_mutex, portMAX_DELAY);
    s_sim_config.channel = ch;
    s_sim_config.chan_mode = SIM_CHAN_SINGLE;
    if (s_config_mutex) xSemaphoreGive(s_config_mutex);
    sim_wifi_set_channel(ch);
    ESP_LOGI(TAG, "channel -> %d (single)", ch);
}

void sim_set_brand(sim_brand_t brand) {
    if (s_config_mutex) xSemaphoreTake(s_config_mutex, portMAX_DELAY);
    s_sim_config.brand = brand;
    if (s_config_mutex) xSemaphoreGive(s_config_mutex);
    ESP_LOGI(TAG, "brand -> %s", sim_brand_name(brand));
}

void sim_set_chan_mode(sim_chan_mode_t mode) {
    if (s_config_mutex) xSemaphoreTake(s_config_mutex, portMAX_DELAY);
    s_sim_config.chan_mode = mode;
    if (s_config_mutex) xSemaphoreGive(s_config_mutex);
    ESP_LOGI(TAG, "chan_mode -> %s", sim_chan_mode_name(mode));
}

void sim_set_protocol(sim_protocol_t proto) {
    if (s_config_mutex) xSemaphoreTake(s_config_mutex, portMAX_DELAY);
    s_sim_config.protocol = proto;
    if (s_config_mutex) xSemaphoreGive(s_config_mutex);
    nvs_save_config(&s_sim_config);
    ESP_LOGI(TAG, "protocol -> %s", sim_protocol_name(proto));
}

/* ================================================================
 * 省/市两级辅助（v2.6.1）
 * 省份列表在首次扫描城市表时按出现顺序建立。
 * ================================================================ */
static const char *s_prov_names[SIM_PROVINCE_COUNT];
static int s_prov_first[SIM_PROVINCE_COUNT];
static int s_prov_count_cnt[SIM_PROVINCE_COUNT];
static bool s_prov_built = false;

static void build_province_table(void) {
    if (s_prov_built) return;
    int n = 0;
    for (int i = 0; i < SIM_CITY_COUNT && n < SIM_PROVINCE_COUNT; i++) {
        const char *p = g_sim_cities[i].province;
        int k = -1;
        for (int j = 0; j < n; j++) {
            if (s_prov_names[j] == p) { k = j; break; } /* 指针比较，表内同名字符串需去重；安全起见也用 strcmp */
        }
        /* 字符串可能不是同一指针，用 strcmp 兜底 */
        if (k < 0) {
            for (int j = 0; j < n; j++) {
                if (strcmp(s_prov_names[j], p) == 0) { k = j; break; }
            }
        }
        if (k < 0) {
            k = n++;
            s_prov_names[k] = p;
            s_prov_first[k] = i;
            s_prov_count_cnt[k] = 0;
        }
        s_prov_count_cnt[k]++;
    }
    s_prov_built = true;
}

int sim_city_province(int city_idx) {
    if (city_idx < 0 || city_idx >= SIM_CITY_COUNT) return -1;
    build_province_table();
    const char *p = g_sim_cities[city_idx].province;
    for (int j = 0; j < SIM_PROVINCE_COUNT; j++) {
        if (s_prov_names[j] && strcmp(s_prov_names[j], p) == 0) return j;
    }
    return -1;
}

const char *sim_get_province_name(int prov_idx) {
    build_province_table();
    if (prov_idx < 0 || prov_idx >= SIM_PROVINCE_COUNT) return "?";
    return s_prov_names[prov_idx];
}

int sim_get_province_count(void) { build_province_table(); return SIM_PROVINCE_COUNT; }

int sim_province_first_city(int prov_idx) {
    build_province_table();
    if (prov_idx < 0 || prov_idx >= SIM_PROVINCE_COUNT) return -1;
    return s_prov_first[prov_idx];
}

int sim_province_city_count(int prov_idx) {
    build_province_table();
    if (prov_idx < 0 || prov_idx >= SIM_PROVINCE_COUNT) return 0;
    return s_prov_count_cnt[prov_idx];
}

int sim_city_step_within_province(int city_idx, int step) {
    if (city_idx < 0 || city_idx >= SIM_CITY_COUNT) city_idx = 0;
    int prov = sim_city_province(city_idx);
    if (prov < 0) return city_idx;
    int first = s_prov_first[prov];
    int cnt = s_prov_count_cnt[prov];
    int last = first + cnt - 1;
    int next = city_idx + step;
    if (step > 0) {
        if (next > last) {
            /* 跳到下一省第一个城市；到末尾回到 0 */
            int np = prov + 1;
            if (np >= SIM_PROVINCE_COUNT) np = 0;
            next = s_prov_first[np];
        }
    } else if (step < 0) {
        if (next < first) {
            int np = prov - 1;
            if (np < 0) np = SIM_PROVINCE_COUNT - 1;
            next = s_prov_first[np] + s_prov_count_cnt[np] - 1;
        }
    }
    return next;
}


/* ================================================================
 * 串口 CLI
 * ================================================================ */
static void cli_print_help(void) {
    printf("\n=== RID Simulator CLI v2.7.0 ===\n");
    printf("  status              - show status\n");
    printf("  count N             - set target count (1-%d)\n", SIM_MAX_TARGETS);
    printf("  speed X.X           - set speed m/s\n");
    printf("  center LAT LON      - set center coords\n");
    printf("  city N              - select city (0-%d, -1=list)\n", SIM_CITY_COUNT - 1);
    printf("  province [N]        - list provinces, or jump to province N\n");
    printf("  channel N           - set single channel (1-13)\n");
    printf("  rotate              - enable ch1/6/11 rotation\n");
    printf("  brand NAME          - crid/dji/autel/parrot/skydio/mixed\n");
    printf("  protocol NAME       - gb42590/gb46750/mixed (v2.7.0)\n");
    printf("  pause / resume      - pause/resume TX\n");
    printf("  start               - start TX from standby\n");
    printf("  stop                - stop simulator\n");
    printf("  help                - this help\n");
    printf("================================\n\n");
}

static void cli_print_status(void) {
    sim_stats_t st;
    sim_get_stats(&st);
    const char *state_str = "?";
    switch (st.state) {
        case SIM_STATE_IDLE:    state_str = "IDLE"; break;
        case SIM_STATE_RUNNING: state_str = "RUNNING"; break;
        case SIM_STATE_PAUSED:  state_str = "PAUSED"; break;
        case SIM_STATE_STOPPED: state_str = "STOPPED"; break;
    }
    printf("\n--- Status ---\n");
    printf("  State:    %s\n", state_str);
    printf("  Targets:  %d\n", st.active_count);
    printf("  Brand:    %s\n", sim_brand_name(st.brand));
    printf("  Protocol: %s\n", sim_protocol_name(st.protocol));
    printf("  Channel:  %d (%s)\n", st.current_channel, sim_chan_mode_name(st.chan_mode));
    printf("  Speed:    %.1f m/s\n", st.speed);
    printf("  Center:   %.6f, %.6f\n", st.base_lat, st.base_lon);
    printf("  TX ok:    %lu\n", (unsigned long)st.tx_count);
    printf("  TX fail:  %lu\n", (unsigned long)st.tx_fail);
    printf("  Rounds:   %lu\n", (unsigned long)st.rounds);
    printf("  Uptime:   %lus\n", (unsigned long)st.uptime_s);
    printf("--------------\n\n");
}

static void cli_print_cities(void) {
    printf("\n--- Cities (by province) ---\n");
    const char *cur = NULL;
    for (int i = 0; i < SIM_CITY_COUNT; i++) {
        if (cur == NULL || strcmp(g_sim_cities[i].province, cur) != 0) {
            cur = g_sim_cities[i].province;
            printf("  [%s]\n", cur);
        }
        printf("    %2d  %-8s  %.4f,%.4f\n", i, g_sim_cities[i].name,
               g_sim_cities[i].lat, g_sim_cities[i].lon);
    }
    printf("Use: city N to select\n\n");
}

static void cli_print_provinces(void) {
    build_province_table();
    printf("\n--- Provinces ---\n");
    for (int i = 0; i < SIM_PROVINCE_COUNT; i++) {
        int first = s_prov_first[i];
        printf("  %2d  %-8s  %d city(ies), e.g. %s (idx %d)\n",
               i, s_prov_names[i], s_prov_count_cnt[i],
               g_sim_cities[first].name, first);
    }
    printf("Use: province N to jump to first city of province N\n\n");
}

bool sim_cli_handle_line(const char *line) {
    if (!line) return false;
    /* Trim leading spaces */
    while (*line == ' ' || *line == '\t') line++;
    if (*line == '\0' || *line == '\n' || *line == '\r') return false;

    if (strncmp(line, "help", 4) == 0 || strncmp(line, "?", 1) == 0) {
        cli_print_help(); return true;
    }
    if (strncmp(line, "status", 6) == 0) {
        cli_print_status(); return true;
    }
    if (strncmp(line, "pause", 5) == 0) {
        sim_pause(); return true;
    }
    if (strncmp(line, "resume", 6) == 0) {
        sim_resume(); return true;
    }
    if (strncmp(line, "start", 5) == 0) {
        /* 从待机启动；已在运行/暂停时分别返回错误或提示 */
        sim_state_t s = sim_get_state();
        if (s == SIM_STATE_STOPPED || s == SIM_STATE_IDLE) {
            esp_err_t e = sim_start(NULL);
            printf("start -> %s\n", esp_err_to_name(e));
        } else if (s == SIM_STATE_PAUSED) {
            sim_resume();
        } else {
            printf("already running\n");
        }
        return true;
    }
    if (strncmp(line, "stop", 4) == 0) {
        sim_stop(); return true;
    }
    if (strncmp(line, "rotate", 6) == 0) {
        sim_set_chan_mode(SIM_CHAN_ROTATE_1_6_11); return true;
    }

    int n; float f; double d1, d2;
    if (sscanf(line, "count %d", &n) == 1) {
        sim_set_count(n); return true;
    }
    if (sscanf(line, "speed %f", &f) == 1) {
        sim_set_speed(f); return true;
    }
    if (sscanf(line, "center %lf %lf", &d1, &d2) == 2) {
        sim_set_center(d1, d2); return true;
    }
    if (sscanf(line, "channel %d", &n) == 1) {
        sim_set_channel((uint8_t)n); return true;
    }
    if (sscanf(line, "city %d", &n) == 1) {
        if (n < 0) { cli_print_cities(); return true; }
        if (n >= 0 && n < SIM_CITY_COUNT) {
            if (s_config_mutex) xSemaphoreTake(s_config_mutex, portMAX_DELAY);
            s_sim_config.city_index = n;
            s_sim_config.base_lat = g_sim_cities[n].lat;
            s_sim_config.base_lon = g_sim_cities[n].lon;
            if (s_config_mutex) xSemaphoreGive(s_config_mutex);
            printf("City -> %s (%.6f,%.6f)\n", g_sim_cities[n].name,
                   g_sim_cities[n].lat, g_sim_cities[n].lon);
            return true;
        }
        printf("Invalid city index. Use city -1 to list.\n");
        return true;
    }
    if (strncmp(line, "province", 8) == 0) {
        const char *arg = line + 8;
        while (*arg == ' ' || *arg == '\t') arg++;
        if (*arg == '\0') { cli_print_provinces(); return true; }
        int pn;
        if (sscanf(arg, "%d", &pn) == 1) {
            build_province_table();
            if (pn >= 0 && pn < SIM_PROVINCE_COUNT) {
                int ci = s_prov_first[pn];
                sim_set_city(ci);
                printf("Province -> %s, first city %s (idx %d)\n",
                       s_prov_names[pn], g_sim_cities[ci].name, ci);
            } else {
                printf("Invalid province (0-%d)\n", SIM_PROVINCE_COUNT - 1);
            }
        } else {
            cli_print_provinces();
        }
        return true;
    }
    if (strncmp(line, "brand ", 6) == 0) {
        const char *name = line + 6;
        while (*name == ' ') name++;
        if (strncasecmp(name, "crid", 4) == 0) sim_set_brand(SIM_BRAND_CRID);
        else if (strncasecmp(name, "dji", 3) == 0) sim_set_brand(SIM_BRAND_DJI);
        else if (strncasecmp(name, "autel", 5) == 0) sim_set_brand(SIM_BRAND_AUTEL);
        else if (strncasecmp(name, "parrot", 6) == 0) sim_set_brand(SIM_BRAND_PARROT);
        else if (strncasecmp(name, "skydio", 6) == 0) sim_set_brand(SIM_BRAND_SKYDIO);
        else if (strncasecmp(name, "mixed", 5) == 0) sim_set_brand(SIM_BRAND_MIXED);
        else printf("Unknown brand. Use: crid/dji/autel/parrot/skydio/mixed\n");
        return true;
    }
    if (strncmp(line, "protocol ", 9) == 0) {
        const char *name = line + 9;
        while (*name == ' ') name++;
        if (strncasecmp(name, "gb42590", 7) == 0 || strncasecmp(name, "old", 3) == 0)
            sim_set_protocol(SIM_PROTO_GB42590);
        else if (strncasecmp(name, "gb46750", 7) == 0 || strncasecmp(name, "new", 3) == 0)
            sim_set_protocol(SIM_PROTO_GB46750);
        else if (strncasecmp(name, "mixed", 5) == 0 || strncasecmp(name, "both", 4) == 0)
            sim_set_protocol(SIM_PROTO_MIXED);
        else
            printf("Unknown protocol. Use: gb42590/gb46750/mixed\n");
        return true;
    }
    return false;
}

/* 串口字节喂入（从 uart task 调用） */
void sim_cli_feed(const char *data, int len) {
    for (int i = 0; i < len; i++) {
        char c = data[i];
        if (c == '\n' || c == '\r') {
            if (s_cli_len > 0) {
                s_cli_buf[s_cli_len] = '\0';
                if (!sim_cli_handle_line(s_cli_buf)) {
                    printf("Unknown command: %s (type 'help')\n", s_cli_buf);
                }
                s_cli_len = 0;
            }
        } else if (c == 0x08 || c == 0x7F) {
            /* backspace */
            if (s_cli_len > 0) s_cli_len--;
        } else if (s_cli_len < SIM_CLI_BUF_SIZE - 1 && c >= 0x20) {
            s_cli_buf[s_cli_len++] = c;
        }
    }
}

/* ================================================================
 * 目标信息查询（供 UI 使用）
 * ================================================================ */
bool sim_get_target_info(int idx, char *sn_out, size_t sn_len,
                         uint8_t *mac_out, char *model_out,
                         size_t model_len, uint8_t *ch_out) {
    if (idx < 0 || idx >= SIM_MAX_TARGETS) return false;
    sim_target_t *t = &s_targets[idx];
    if (t->mac[0] == 0 && t->mac[1] == 0 && t->mac[2] == 0) return false;
    if (sn_out && sn_len > 0) {
        strncpy(sn_out, t->uas_id, sn_len - 1);
        sn_out[sn_len - 1] = '\0';
    }
    if (mac_out) memcpy(mac_out, t->mac, 6);
    if (model_out && model_len > 0) {
        const char *m = t->sn_entry ? t->sn_entry->model : "Unknown";
        strncpy(model_out, m, model_len - 1);
        model_out[model_len - 1] = '\0';
    }
    if (ch_out) *ch_out = t->channel;
    return true;
}

/* 城市索引管理 */
static int s_city_index = 12;
void sim_set_city(int idx) {
    if (idx < 0 || idx >= SIM_CITY_COUNT) return;
    s_city_index = idx;
    if (s_config_mutex) xSemaphoreTake(s_config_mutex, portMAX_DELAY);
    s_sim_config.city_index = idx;
    s_sim_config.base_lat = g_sim_cities[idx].lat;
    s_sim_config.base_lon = g_sim_cities[idx].lon;
    if (s_config_mutex) xSemaphoreGive(s_config_mutex);
    ESP_LOGI(TAG, "City -> %s", g_sim_cities[idx].name);
}
int sim_get_city_index(void) { return s_city_index; }

