/**
 * sim_core.h — RID 伪造模拟器对外 API（v2.6.0 多目标+多品牌+多信道+CLI）
 *
 * 标准：GB 42590-2023 (OUI=FA:0B:BC, VendorType=0x0D)
 * 扩展：DJI Beacon VID (OUI=26:37:12)
 *
 * v2.6.0 新增：
 *   - SIM_MAX_TARGETS 64 -> 300（v2.6.1）
 *   - 真实 SN 前缀库（117 条目，Crockford Base32 后缀）
 *   - 三信道轮发（ch1/6/11）
 *   - 串口 CLI 实时控制（count/speed/pause/resume/center/status/help）
 *   - 暂停/恢复
 *   - 运行时统计（帧数、失败数、目标数、当前信道）
 */

#ifndef SIM_CORE_H
#define SIM_CORE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SIM_MAX_TARGETS 300

/* ================================================================
 * 模拟器工作模式
 * ================================================================ */
typedef enum {
    SIM_MODE_CIRCLE = 0,
    SIM_MODE_PINGPONG = 1,
    SIM_MODE_S_SEARCH = 2,
    SIM_MODE_MAX
} sim_flight_mode_t;

/* ================================================================
 * 模拟器状态
 * ================================================================ */
typedef enum {
    SIM_STATE_IDLE = 0,
    SIM_STATE_RUNNING,
    SIM_STATE_PAUSED,
    SIM_STATE_STOPPED,
} sim_state_t;

/* ================================================================
 * 品牌/协议类型
 * ================================================================ */
typedef enum {
    SIM_BRAND_CRID = 0,    /* GB42590 标准 (FA:0B:BC) */
    SIM_BRAND_DJI = 1,     /* DJI Beacon VID (26:37:12) */
    SIM_BRAND_AUTEL = 2,   /* Autel EVO */
    SIM_BRAND_PARROT = 3,  /* Parrot Anafi */
    SIM_BRAND_SKYDIO = 4,  /* Skydio */
    SIM_BRAND_MIXED = 99,  /* 混合模式：按 SN 库分布随机 */
} sim_brand_t;

/* ================================================================
 * 信道轮发模式
 * ================================================================ */
typedef enum {
    SIM_CHAN_SINGLE = 0,   /* 单信道 */
    SIM_CHAN_ROTATE_1_6_11 = 1,  /* 三信道轮发 ch1/6/11 */
} sim_chan_mode_t;

/* ================================================================
 * 模拟器配置（v2.6.0 扩展）
 * ================================================================ */
typedef struct {
    double base_lat;
    double base_lon;
    float altitude_msl;
    float altitude_agl;
    float speed;
    uint8_t channel;               /* 主信道（单信道模式） */
    sim_flight_mode_t flight_mode;
    int target_count;
    int8_t tx_power;
    char uas_id[21];
    char operator_id[21];
    char ssid[33];
    /* v2.6.0 扩展 */
    sim_brand_t brand;             /* 品牌/协议 */
    sim_chan_mode_t chan_mode;     /* 信道模式 */
    int frame_interval_ms;         /* 每架帧间隔 ms（默认 5） */
    int round_interval_ms;         /* 轮间隔 ms（默认 1000） */
    int city_index;                /* 城市索引（-1=自定义坐标） */
} sim_config_t;

/* ================================================================
 * 运行时统计（串口 status 命令用）
 * ================================================================ */
typedef struct {
    uint32_t tx_count;             /* 累计成功帧数 */
    uint32_t tx_fail;              /* 累计失败帧数 */
    uint32_t rounds;               /* 完成的轮次 */
    int active_count;              /* 实际活跃目标数 */
    uint8_t current_channel;       /* 当前信道 */
    sim_state_t state;
    sim_brand_t brand;
    sim_chan_mode_t chan_mode;
    float speed;
    double base_lat;
    double base_lon;
    uint32_t uptime_s;             /* 运行时长（秒） */
} sim_stats_t;

/* ================================================================
 * API
 * ================================================================ */

esp_err_t sim_init(void);
esp_err_t sim_start(const sim_config_t *config);
esp_err_t sim_stop(void);
esp_err_t sim_pause(void);
esp_err_t sim_resume(void);

sim_state_t sim_get_state(void);
int sim_get_target_count(void);
void sim_get_current_position(double *lat, double *lon, float *heading);
uint32_t sim_get_tx_count(void);
void sim_update_config(const sim_config_t *config);
void sim_get_default_config(sim_config_t *config);
const char *sim_flight_mode_name(sim_flight_mode_t mode);
const char *sim_brand_name(sim_brand_t brand);
const char *sim_chan_mode_name(sim_chan_mode_t mode);

/* v2.6.0 运行时控制（CLI 调用） */
void sim_set_count(int count);
void sim_set_speed(float speed);
void sim_set_center(double lat, double lon);
void sim_set_channel(uint8_t ch);
void sim_set_brand(sim_brand_t brand);
void sim_set_chan_mode(sim_chan_mode_t mode);
void sim_get_stats(sim_stats_t *out);

/* 串口 CLI 处理：解析一行命令，返回是否已处理 */
bool sim_cli_handle_line(const char *line);

/* 串口 CLI 字节喂入（从 UART 任务逐字节调用） */
void sim_cli_feed(const char *data, int len);

/* 目标信息查询（供 UI 使用） */
bool sim_get_target_info(int idx, char *sn_out, size_t sn_len,
                         uint8_t *mac_out, char *model_out,
                         size_t model_len, uint8_t *ch_out);

/* 城市选择 */
void sim_set_city(int idx);
int sim_get_city_index(void);

/* 城市表 */
typedef struct {
    const char *name;       /* 城市中文名 */
    const char *province;   /* 省份/直辖市 */
    double lat;
    double lon;
} sim_city_t;

#define SIM_CITY_COUNT 67
#define SIM_PROVINCE_COUNT 31

/* ================================================================
 * 省/市两级选择 API（v2.6.1）
 * 省份按 g_sim_cities 中首次出现顺序排列，索引 0..SIM_PROVINCE_COUNT-1。
 * ================================================================ */
const char *sim_get_province_name(int prov_idx);
int  sim_get_province_count(void);
/* 返回某省下第一个城市在 g_sim_cities 中的索引，无则 -1 */
int  sim_province_first_city(int prov_idx);
/* 返回某省的城市数量 */
int  sim_province_city_count(int prov_idx);
/* 把城市在某省范围内按 step 移动（+1 下一城市/-1 上一城市），
 * 自动跨省循环，返回新的全局城市索引。 */
int  sim_city_step_within_province(int city_idx, int step);
/* 获取城市所属省份索引 */
int  sim_city_province(int city_idx);

#ifdef __cplusplus
}
#endif

#endif // SIM_CORE_H
