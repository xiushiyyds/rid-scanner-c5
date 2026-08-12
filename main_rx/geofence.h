/**
 * geofence.h — 地理围栏与告警模块
 *
 * ESP32 Remote ID Scanner — 空域合规检测
 *
 * 功能：
 *   1. 超高告警：无人机高度超过法定限高（默认120m）
 *   2. 禁飞区告警：无人机进入预定义的禁飞/管制空域
 *   3. 机场净空区告警：无人机进入机场净空保护区
 *
 * 围栏数据来源：
 *   - 大连市人民政府公告（2026-05-22）
 *   - 大连公安治安支队执法案例
 *   - 民航局UOM平台公开信息
 *
 * 坐标系统：WGS-84
 */

#ifndef GEOFENCE_H
#define GEOFENCE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 * 围栏类型
 * ================================================================ */

typedef enum {
    FENCE_TYPE_NOFLY     = 0,   // 禁飞区（军事/政府核心/核设施）
    FENCE_TYPE_AIRPORT   = 1,   // 机场净空保护区
    FENCE_TYPE_RESTRICTED = 2,  // 管制空域（高铁/港口/基础设施）
} fence_type_t;

/* ================================================================
 * 围栏形状
 * ================================================================ */

typedef enum {
    FENCE_SHAPE_RECT   = 0,     // 矩形（axis-aligned bounding box）
    FENCE_SHAPE_CIRCLE = 1,     // 圆形（中心+半径）
} fence_shape_t;

/* ================================================================
 * 围栏区域定义
 * ================================================================ */

typedef struct {
    const char   *name;         // 区域名称（中文，UTF-8）
    fence_type_t  type;         // 围栏类型
    fence_shape_t shape;        // 形状
    union {
        struct {
            double min_lat, min_lon;
            double max_lat, max_lon;
        } rect;
        struct {
            double center_lat, center_lon;
            float  radius_m;    // 半径（米）
        } circle;
    };
    float max_alt_m;            // 最大允许高度（m），0=全域禁飞
} fence_zone_t;

/* ================================================================
 * 告警等级
 * ================================================================ */

typedef enum {
    ALERT_NONE        = 0,      // 无告警
    ALERT_ALTITUDE    = 1,      // 超高（>120m）
    ALERT_NOFLY       = 2,      // 禁飞区
    ALERT_AIRPORT     = 3,      // 机场净空区
    ALERT_RESTRICTED  = 4,      // 管制空域
} alert_level_t;

/* ================================================================
 * 告警结果
 * ================================================================ */

typedef struct {
    alert_level_t level;            // 告警等级（取最高）
    char zone_name[32];             // 触发区域名称
    float uav_alt_m;                // 当前无人机高度
    float max_allowed_alt_m;        // 该区域最大允许高度
    bool in_fence;                  // 是否在围栏内
} geofence_alert_t;

/* ================================================================
 * 全局配置
 * ================================================================ */

#define GEOFENCE_DEFAULT_ALT_LIMIT    120.0f   // 轻型无人机限高120m
#define GEOFENCE_MICRO_ALT_LIMIT      50.0f    // 微型无人机限高50m

/* ================================================================
 * API
 * ================================================================ */

void geofence_init(void);

bool geofence_check(double uav_lat, double uav_lon, float uav_alt_m,
                    geofence_alert_t *alert);

bool geofence_check_altitude(float alt_m, float limit_m);

int geofence_get_zone_count(void);

const fence_zone_t *geofence_get_zone(int index);

const char *geofence_alert_name(alert_level_t level);

int geofence_alert_color(alert_level_t level);

#ifdef __cplusplus
}
#endif

#endif // GEOFENCE_H
