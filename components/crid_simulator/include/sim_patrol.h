/**
 * sim_patrol.h — 模拟器飞行路径计算引擎（多实例版）
 *
 * 支持最多 SIM_PATROL_MAX_INSTANCES 个独立实例，
 * 每个实例有独立的基准坐标、速度、模式和相位偏移，
 * 用于多目标（多无人机）模拟场景。
 */

#ifndef SIM_PATROL_H
#define SIM_PATROL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SIM_PATROL_MAX_INSTANCES 64

/**
 * 飞行模式枚举
 */
typedef enum {
    SIM_PATROL_CIRCLE = 0,     // 圆形巡游
    SIM_PATROL_PINGPONG = 1,   // 直线往返
    SIM_PATROL_S_SEARCH = 2,   // S型搜索
} sim_patrol_mode_t;

/**
 * 单个路径引擎实例的独立状态
 */
typedef struct {
    double base_lat;
    double base_lon;
    float speed_factor;
    sim_patrol_mode_t mode;
    uint32_t tick;
    bool active;
    float phase_offset;  // 每个目标的初始相位偏移，让轨迹不同
} sim_patrol_instance_t;

/**
 * 初始化指定实例的路径引擎
 * @param id 实例 ID (0 ~ SIM_PATROL_MAX_INSTANCES-1)
 * @param base_lat 基准纬度
 * @param base_lon 基准经度
 * @param speed 飞行速度因子
 * @param mode 飞行模式
 * @param phase_offset 初始相位偏移（弧度）
 */
void sim_patrol_init_instance(int id, double base_lat, double base_lon,
                               float speed, sim_patrol_mode_t mode,
                               float phase_offset);

/**
 * 计算指定实例的下一个位置点
 * @param id 实例 ID
 * @param out_lat 输出纬度
 * @param out_lon 输出经度
 * @param out_heading 输出航向角 (0~360°)
 */
void sim_patrol_next(int id, double *out_lat, double *out_lon, float *out_heading);

/**
 * 重置指定实例的路径引擎计时器
 * @param id 实例 ID
 */
void sim_patrol_reset_instance(int id);

/**
 * 更新指定实例的路径参数（飞行中可用）
 * @param id 实例 ID
 * @param base_lat 新基准纬度
 * @param base_lon 新基准经度
 * @param speed 新速度因子
 * @param mode 新飞行模式
 * @param phase_offset 新相位偏移
 */
void sim_patrol_update_instance(int id, double base_lat, double base_lon,
                                 float speed, sim_patrol_mode_t mode,
                                 float phase_offset);

/* ================================================================
 * 向后兼容的旧接口（内部映射到 instance 0）
 * ================================================================ */
void sim_patrol_init(double base_lat, double base_lon, float speed, sim_patrol_mode_t mode);
void sim_patrol_calculate_next(double *out_lat, double *out_lon, float *out_heading);
void sim_patrol_update(double base_lat, double base_lon, float speed, sim_patrol_mode_t mode);
void sim_patrol_reset(void);

#ifdef __cplusplus
}
#endif

#endif // SIM_PATROL_H
