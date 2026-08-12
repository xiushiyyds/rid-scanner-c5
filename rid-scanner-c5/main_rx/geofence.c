/**
 * geofence.c — 地理围栏与告警模块实现
 *
 * 包含大连地区禁飞区/管制空域数据（基于公开政府公告）
 *
 * 数据来源：
 *   1. 大连市人民政府公告（2026-05-22）：
 *      中共大连市委军民融合发展委员会办公室关于公布微型、轻型、小型
 *      无人驾驶航空器适飞空域范围的公告
 *   2. 大连公安治安支队执法案例（2026年1-6月）
 *   3. 大连市关于进一步加强机场净空保护区升空物体管理的通告
 *   4. 《无人驾驶航空器飞行管理暂行条例》
 *
 * 坐标精度说明：
 *   围栏坐标基于公开文字描述近似推算，与实际精确坐标可能存在
 *   数百米偏差。实际部署时应根据民航局UOM平台数据或公安部门
 *   提供的精确坐标进行校准。
 */

#include <string.h>
#include <math.h>
#include "geofence.h"
#include "gps_module.h"
#include "esp_log.h"

static const char *TAG = "GEOFENCE";

/* ================================================================
 * 大连地区围栏数据（存储在 Flash 常量区）
 *
 * 优先级排序：NOFLY > AIRPORT > RESTRICTED
 * 当同时命中多个区域时，取最高优先级告警
 * ================================================================ */

/* 大连周水子机场净空保护区
 *
 * 官方描述：
 *   跑道中心线两侧各10公里、跑道两端外各20公里的矩形区域
 *   北到金州五一路至开发区金马路
 *   南到黄浦路至旅顺南路
 *   东到姜家沟至老虎滩海岸沿线
 *   西到拉树房至营城子湾以西旅顺北路
 *
 * 坐标近似（WGS-84）：
 *   北边界: 39.090°N (金州五一路/开发区金马路一线)
 *   南边界: 38.860°N (黄浦路/旅顺南路一线)
 *   东边界: 121.700°E (老虎滩海岸)
 *   西边界: 121.450°E (营城子湾以西旅顺北路)
 *
 * 注：此区域覆盖大连主城区绝大部分（甘井子、沙河口、西岗、中山、高新园）
 */
#define DL_AIRPORT_MIN_LAT  38.860
#define DL_AIRPORT_MAX_LAT  39.090
#define DL_AIRPORT_MIN_LON  121.450
#define DL_AIRPORT_MAX_LON  121.700

/* 旅顺军港区域
 * 军事管理区，严禁无人机飞行
 * 中心约(38.810, 121.260)
 */
#define DL_LVSHUN_MIN_LAT  38.780
#define DL_LVSHUN_MAX_LAT  38.840
#define DL_LVSHUN_MIN_LON  121.220
#define DL_LVSHUN_MAX_LON  121.300

/* 金石滩旅游度假区
 * 景区及周边管控区域
 */
#define DL_JINSHITAN_MIN_LAT  38.990
#define DL_JINSHITAN_MAX_LAT  39.060
#define DL_JINSHITAN_MIN_LON  121.920
#define DL_JINSHITAN_MAX_LON  122.030

/* 红沿河核电站（瓦房店）
 * 核设施控制区域，半径5km
 */
#define DL_NUCLEAR_LAT  39.810
#define DL_NUCLEAR_LON  121.440
#define DL_NUCLEAR_R    5000.0f

/* 大连港核心作业区（在机场净空区内，单独标注） */
#define DL_PORT_MIN_LAT  38.905
#define DL_PORT_MAX_LAT  38.940
#define DL_PORT_MIN_LON  121.635
#define DL_PORT_MAX_LON  121.690

/* 星海广场（在机场净空区内，高频违规区域） */
#define DL_XINGHAI_MIN_LAT  38.862
#define DL_XINGHAI_MAX_LAT  38.878
#define DL_XINGHAI_MIN_LON  121.544
#define DL_XINGHAI_MAX_LON  121.564

/* 东港商务区（在机场净空区内，高频违规区域） */
#define DL_DONGGANG_MIN_LAT  38.900
#define DL_DONGGANG_MAX_LAT  38.920
#define DL_DONGGANG_MIN_LON  121.635
#define DL_DONGGANG_MAX_LON  121.665

/* 中山广场（在机场净空区内，执法案例高发） */
#define DL_ZHONGSHAN_MIN_LAT  38.908
#define DL_ZHONGSHAN_MAX_LAT  38.922
#define DL_ZHONGSHAN_MIN_LON  121.625
#define DL_ZHONGSHAN_MAX_LON  121.645

/* 人民广场/西岗区（在机场净空区内，执法案例高发） */
#define DL_RENMIN_MIN_LAT  38.905
#define DL_RENMIN_MAX_LAT  38.920
#define DL_RENMIN_MIN_LON  121.600
#define DL_RENMIN_MAX_LON  121.625

/* 梭鱼湾足球场（在机场净空区内） */
#define DL_SUOYU_MIN_LAT  38.930
#define DL_SUOYU_MAX_LAT  38.950
#define DL_SUOYU_MIN_LON  121.585
#define DL_SUOYU_MAX_LON  121.615

/* 大连北站及高铁沿线（在机场净空区内） */
#define DL_STATION_MIN_LAT  38.950
#define DL_STATION_MAX_LAT  38.970
#define DL_STATION_MIN_LON  121.575
#define DL_STATION_MAX_LON  121.605

/* 金州湾新机场（在建，预留） */
#define DL_JZ_AIRPORT_MIN_LAT  39.020
#define DL_JZ_AIRPORT_MAX_LAT  39.060
#define DL_JZ_AIRPORT_MIN_LON  121.440
#define DL_JZ_AIRPORT_MAX_LON  121.510

/* ================================================================
 * 围栏数据表（const → 存储在 Flash）
 *
 * 注意：顺序影响检测优先级，NOFLY 在前，AIRPORT 居中，RESTRICTED 在后
 * ================================================================ */

static const fence_zone_t s_zones[] = {
    /* ---- 禁飞区 (NOFLY) ---- */
    {
        .name = "旅顺军港",
        .type = FENCE_TYPE_NOFLY,
        .shape = FENCE_SHAPE_RECT,
        .rect = { DL_LVSHUN_MIN_LAT, DL_LVSHUN_MIN_LON,
                  DL_LVSHUN_MAX_LAT, DL_LVSHUN_MAX_LON },
        .max_alt_m = 0,
    },
    {
        .name = "红沿河核电站",
        .type = FENCE_TYPE_NOFLY,
        .shape = FENCE_SHAPE_CIRCLE,
        .circle = { DL_NUCLEAR_LAT, DL_NUCLEAR_LON, DL_NUCLEAR_R },
        .max_alt_m = 0,
    },

    /* ---- 机场净空区 (AIRPORT) ---- */
    {
        .name = "周水子机场净空区",
        .type = FENCE_TYPE_AIRPORT,
        .shape = FENCE_SHAPE_RECT,
        .rect = { DL_AIRPORT_MIN_LAT, DL_AIRPORT_MIN_LON,
                  DL_AIRPORT_MAX_LAT, DL_AIRPORT_MAX_LON },
        .max_alt_m = 0,  // 净空区内全域禁止
    },
    {
        .name = "金州湾机场(在建)",
        .type = FENCE_TYPE_AIRPORT,
        .shape = FENCE_SHAPE_RECT,
        .rect = { DL_JZ_AIRPORT_MIN_LAT, DL_JZ_AIRPORT_MIN_LON,
                  DL_JZ_AIRPORT_MAX_LAT, DL_JZ_AIRPORT_MAX_LON },
        .max_alt_m = 0,
    },

    /* ---- 管制空域 (RESTRICTED) — 机场净空区内的重点标注区域 ---- */
    {
        .name = "星海广场",
        .type = FENCE_TYPE_RESTRICTED,
        .shape = FENCE_SHAPE_RECT,
        .rect = { DL_XINGHAI_MIN_LAT, DL_XINGHAI_MIN_LON,
                  DL_XINGHAI_MAX_LAT, DL_XINGHAI_MAX_LON },
        .max_alt_m = 0,
    },
    {
        .name = "东港商务区",
        .type = FENCE_TYPE_RESTRICTED,
        .shape = FENCE_SHAPE_RECT,
        .rect = { DL_DONGGANG_MIN_LAT, DL_DONGGANG_MIN_LON,
                  DL_DONGGANG_MAX_LAT, DL_DONGGANG_MAX_LON },
        .max_alt_m = 0,
    },
    {
        .name = "中山广场",
        .type = FENCE_TYPE_RESTRICTED,
        .shape = FENCE_SHAPE_RECT,
        .rect = { DL_ZHONGSHAN_MIN_LAT, DL_ZHONGSHAN_MIN_LON,
                  DL_ZHONGSHAN_MAX_LAT, DL_ZHONGSHAN_MAX_LON },
        .max_alt_m = 0,
    },
    {
        .name = "人民广场",
        .type = FENCE_TYPE_RESTRICTED,
        .shape = FENCE_SHAPE_RECT,
        .rect = { DL_RENMIN_MIN_LAT, DL_RENMIN_MIN_LON,
                  DL_RENMIN_MAX_LAT, DL_RENMIN_MAX_LON },
        .max_alt_m = 0,
    },
    {
        .name = "梭鱼湾足球场",
        .type = FENCE_TYPE_RESTRICTED,
        .shape = FENCE_SHAPE_RECT,
        .rect = { DL_SUOYU_MIN_LAT, DL_SUOYU_MIN_LON,
                  DL_SUOYU_MAX_LAT, DL_SUOYU_MAX_LON },
        .max_alt_m = 0,
    },
    {
        .name = "大连港核心区",
        .type = FENCE_TYPE_RESTRICTED,
        .shape = FENCE_SHAPE_RECT,
        .rect = { DL_PORT_MIN_LAT, DL_PORT_MIN_LON,
                  DL_PORT_MAX_LAT, DL_PORT_MAX_LON },
        .max_alt_m = 0,
    },
    {
        .name = "大连北站/高铁",
        .type = FENCE_TYPE_RESTRICTED,
        .shape = FENCE_SHAPE_RECT,
        .rect = { DL_STATION_MIN_LAT, DL_STATION_MIN_LON,
                  DL_STATION_MAX_LAT, DL_STATION_MAX_LON },
        .max_alt_m = 0,
    },
    {
        .name = "金石滩度假区",
        .type = FENCE_TYPE_RESTRICTED,
        .shape = FENCE_SHAPE_RECT,
        .rect = { DL_JINSHITAN_MIN_LAT, DL_JINSHITAN_MIN_LON,
                  DL_JINSHITAN_MAX_LAT, DL_JINSHITAN_MAX_LON },
        .max_alt_m = 0,
    },
};

#define ZONE_COUNT  (sizeof(s_zones) / sizeof(s_zones[0]))

/* ================================================================
 * 内部函数
 * ================================================================ */

static bool point_in_rect(double lat, double lon,
                          double min_lat, double min_lon,
                          double max_lat, double max_lon)
{
    return (lat >= min_lat && lat <= max_lat &&
            lon >= min_lon && lon <= max_lon);
}

static bool point_in_circle(double lat, double lon,
                            double center_lat, double center_lon,
                            float radius_m)
{
    double dist = gps_distance(lat, lon, center_lat, center_lon);
    return (dist <= (double)radius_m);
}

static bool point_in_zone(double lat, double lon, const fence_zone_t *zone)
{
    if (zone->shape == FENCE_SHAPE_RECT) {
        return point_in_rect(lat, lon,
                            zone->rect.min_lat, zone->rect.min_lon,
                            zone->rect.max_lat, zone->rect.max_lon);
    } else {
        return point_in_circle(lat, lon,
                              zone->circle.center_lat, zone->circle.center_lon,
                              zone->circle.radius_m);
    }
}

/**
 * 获取围栏类型对应的告警等级
 */
static alert_level_t type_to_alert(fence_type_t type)
{
    switch (type) {
        case FENCE_TYPE_NOFLY:      return ALERT_NOFLY;
        case FENCE_TYPE_AIRPORT:    return ALERT_AIRPORT;
        case FENCE_TYPE_RESTRICTED: return ALERT_RESTRICTED;
        default:                    return ALERT_NONE;
    }
}

/**
 * 比较告警等级（返回更高等级）
 * 优先级: NOFLY(2) > AIRPORT(3) > RESTRICTED(4) > ALTITUDE(1) > NONE(0)
 *
 * 注意：枚举值大小不代表优先级，这里按实际严重程度排序
 */
static alert_level_t higher_alert(alert_level_t a, alert_level_t b)
{
    // 优先级权重（越高越严重）
    static const int priority[] = {
        [ALERT_NONE]       = 0,
        [ALERT_ALTITUDE]   = 1,
        [ALERT_NOFLY]      = 4,
        [ALERT_AIRPORT]    = 3,
        [ALERT_RESTRICTED] = 2,
    };
    
    if (a >= sizeof(priority)/sizeof(priority[0])) return b;
    if (b >= sizeof(priority)/sizeof(priority[0])) return a;
    
    return (priority[a] >= priority[b]) ? a : b;
}

/* ================================================================
 * 公开 API
 * ================================================================ */

void geofence_init(void)
{
    ESP_LOGI(TAG, "Geofence loaded: %d zones (Dalian)", (int)ZONE_COUNT);
    for (int i = 0; i < (int)ZONE_COUNT; i++) {
        ESP_LOGI(TAG, "  [%d] %s (type=%d)", i, s_zones[i].name, s_zones[i].type);
    }
}

bool geofence_check(double uav_lat, double uav_lon, float uav_alt_m,
                    geofence_alert_t *alert)
{
    if (alert == NULL) return false;
    
    // 初始化
    memset(alert, 0, sizeof(geofence_alert_t));
    alert->level = ALERT_NONE;
    alert->uav_alt_m = uav_alt_m;
    alert->in_fence = false;
    
    // 1. 检查超高
    if (uav_alt_m > GEOFENCE_DEFAULT_ALT_LIMIT) {
        alert->level = ALERT_ALTITUDE;
        alert->uav_alt_m = uav_alt_m;
        alert->max_allowed_alt_m = GEOFENCE_DEFAULT_ALT_LIMIT;
        strncpy(alert->zone_name, "超高告警", sizeof(alert->zone_name) - 1);
        alert->zone_name[sizeof(alert->zone_name) - 1] = '\0';
    }
    
    // 2. 检查围栏区域（遍历所有区域，取最高优先级）
    for (int i = 0; i < (int)ZONE_COUNT; i++) {
        const fence_zone_t *zone = &s_zones[i];
        
        if (!point_in_zone(uav_lat, uav_lon, zone)) {
            continue;
        }
        
        // 在围栏内
        alert->in_fence = true;
        alert_level_t zone_alert = type_to_alert(zone->type);
        
        if (higher_alert(zone_alert, alert->level) == zone_alert) {
            // 更高优先级的告警
            alert->level = zone_alert;
            strncpy(alert->zone_name, zone->name, sizeof(alert->zone_name) - 1);
            alert->zone_name[sizeof(alert->zone_name) - 1] = '\0';
            alert->max_allowed_alt_m = zone->max_alt_m;
        }
    }
    
    return (alert->level != ALERT_NONE);
}

bool geofence_check_altitude(float alt_m, float limit_m)
{
    if (limit_m <= 0) limit_m = GEOFENCE_DEFAULT_ALT_LIMIT;
    return (alt_m > limit_m);
}

int geofence_get_zone_count(void)
{
    return (int)ZONE_COUNT;
}

const fence_zone_t *geofence_get_zone(int index)
{
    if (index < 0 || index >= (int)ZONE_COUNT) return NULL;
    return &s_zones[index];
}

const char *geofence_alert_name(alert_level_t level)
{
    switch (level) {
        case ALERT_NONE:       return "安全";
        case ALERT_ALTITUDE:   return "超高";
        case ALERT_NOFLY:      return "禁飞区";
        case ALERT_AIRPORT:    return "机场净空区";
        case ALERT_RESTRICTED: return "管制空域";
        default:               return "未知";
    }
}

int geofence_alert_color(alert_level_t level)
{
    switch (level) {
        case ALERT_NONE:       return 0;  // 绿色
        case ALERT_ALTITUDE:   return 1;  // 黄色
        case ALERT_RESTRICTED: return 1;  // 黄色
        case ALERT_AIRPORT:    return 2;  // 红色
        case ALERT_NOFLY:      return 2;  // 红色
        default:               return 0;
    }
}
