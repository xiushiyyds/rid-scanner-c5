/**
 * sim_core.h — RID 伪造模拟器对外 API（多目标版）
 *
 * v1.4 扩展：支持最多 SIM_MAX_TARGETS 架无人机同时模拟，
 * 每架无人机有独立 MAC、UAS_ID、轨迹和飞行参数。
 *
 * 标准：GB 42590-2023 (OUI=FA:0B:BC, VendorType=0x0D)
 */

#ifndef SIM_CORE_H
#define SIM_CORE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SIM_MAX_TARGETS 64

/* ================================================================
 * 模拟器工作模式
 * ================================================================ */
typedef enum {
    SIM_MODE_CIRCLE = 0,      // 圆形巡游
    SIM_MODE_PINGPONG = 1,    // 直线往返
    SIM_MODE_S_SEARCH = 2,    // S型搜索
    SIM_MODE_MAX
} sim_flight_mode_t;

/* ================================================================
 * 模拟器状态
 * ================================================================ */
typedef enum {
    SIM_STATE_IDLE = 0,       // 未初始化或已释放
    SIM_STATE_RUNNING,        // 正在发射
    SIM_STATE_STOPPED,        // 已停止（可重新启动）
} sim_state_t;

/* ================================================================
 * 模拟器配置（多目标版）
 * ================================================================ */
typedef struct {
    double base_lat;           // 基准纬度（所有目标的中心点）
    double base_lon;           // 基准经度
    float altitude_msl;        // 基准气压高度 (m)
    float altitude_agl;        // 基准相对高度 (m)
    float speed;               // 基准飞行速度 (m/s)
    uint8_t channel;           // Wi-Fi 信道
    sim_flight_mode_t flight_mode;  // 飞行模式
    int target_count;          // 模拟目标数量 (1~64)
    int8_t tx_power;           // 发射功率 (单位0.25dBm, 20=5dBm, 80=20dBm)
    char uas_id[21];           // 主 UAS ID（多目标时自动覆盖）
    char operator_id[21];      // 操作员 ID
    char ssid[33];             // SoftAP SSID
} sim_config_t;

/* ================================================================
 * API
 * ================================================================ */

/**
 * 初始化模拟器（一次性初始化，NVS 等）
 * 不启动 Wi-Fi 或发射，仅做资源准备。
 * @return ESP_OK 成功
 */
esp_err_t sim_init(void);

/**
 * 启动模拟发射
 * 初始化 Wi-Fi AP 模式 + 创建多目标发射任务。
 * 调用前必须确保 Wi-Fi sniffer 已停止。
 * @param config 模拟器配置
 * @return ESP_OK 成功
 */
esp_err_t sim_start(const sim_config_t *config);

/**
 * 停止模拟发射
 * 停止发射任务，关闭 Wi-Fi AP，释放资源。
 * @return ESP_OK 成功
 */
esp_err_t sim_stop(void);

/**
 * 获取当前状态
 */
sim_state_t sim_get_state(void);

/**
 * 获取当前目标数量
 */
int sim_get_target_count(void);

/**
 * 获取第一架无人机的当前位置和航向（飞行中实时更新）
 * @param lat 输出纬度
 * @param lon 输出经度
 * @param heading 输出航向角 (0~360°)
 */
void sim_get_current_position(double *lat, double *lon, float *heading);

/**
 * 获取已发射帧数（所有目标累计）
 */
uint32_t sim_get_tx_count(void);

/**
 * 动态更新配置（飞行中可用）
 * 线程安全，会立即生效到下一次发射。
 * @param config 新配置
 */
void sim_update_config(const sim_config_t *config);

/**
 * 获取默认配置（大连市中心坐标，单目标）
 * @param config 输出配置
 */
void sim_get_default_config(sim_config_t *config);

/**
 * 获取飞行模式名称字符串
 */
const char *sim_flight_mode_name(sim_flight_mode_t mode);

#ifdef __cplusplus
}
#endif

#endif // SIM_CORE_H
