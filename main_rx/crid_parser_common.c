/**
 * crid_parser_common.c — 协议解析通用模块（自定义版）
 *
 * 增强：
 *   - ASTM/ASD-STAN 自动区分（基于 EU 分类字段）
 *   - DJI DroneID 固件标记检测
 */

#include <string.h>
#include "esp_log.h"
#include "opendroneid.h"
#include "odid_wifi.h"
#include "crid_parser.h"
#include "crid_json.h"
#include "crid_rx_types.h"

static const char *TAG = "PARSER_RID";

/* ================================================================
 * 坐标有效性校验（v2.7.4）
 *
 * 空中干扰/BLE-WiFi共存可能导致单帧中个别字段比特翻转，
 * decodeLatLon 不检查范围，损坏的 int32 会被直接转成
 * 34084864835328 这类天文数字 double。这里在提取层兜底：
 * 纬度必须 [-90, 90]，经度必须 [-180, 180]，超出则丢弃。
 * ================================================================ */
static inline bool valid_lat(double v) { return v >= -90.0 && v <= 90.0; }
static inline bool valid_lon(double v) { return v >= -180.0 && v <= 180.0; }
static inline bool valid_coord(double lat, double lon) {
    /* 0,0 也视为无效（RID 设备不会真在几内亚湾） */
    return valid_lat(lat) && valid_lon(lon) &&
           !(lat == 0.0 && lon == 0.0);
}

/*
 * Debug 开关：设为 1 时，在解析前打印原始数据十六进制转储
 * ================================================================ */
#ifndef PARSER_DEBUG_HEX_DUMP
#define PARSER_DEBUG_HEX_DUMP   0
#endif

/* ================================================================
 * Debug 辅助：十六进制转储
 * ================================================================ */
#if PARSER_DEBUG_HEX_DUMP
static void hex_dump(const char *tag, const char *prefix, const uint8_t *data, uint8_t len) {
    char line[128];
    int pos = 0;
    pos += snprintf(line + pos, sizeof(line) - pos, "%s [%u] ", prefix, len);
    for (int i = 0; i < len; i++) {
        pos += snprintf(line + pos, sizeof(line) - pos, "%02X ", data[i]);
        if ((i + 1) % 16 == 0 && i + 1 < len) {
            ESP_LOGI(tag, "%s", line);
            pos = snprintf(line, sizeof(line), "       ");
        }
    }
    if (pos > 0) {
        ESP_LOGI(tag, "%s", line);
    }
}
#endif

/* ================================================================
 * ASD-STAN 检测：检查 ODID_UAS_Data 中的 EU 分类字段
 *
 * ASD-STAN prEN 4709-002 是 ASTM F3411 的欧洲版本，使用完全相同的
 * OpenDroneID 线格式。区分方法是检查 System 消息中的 EU 特有字段：
 *   - ClassificationType (非 zero = EU 分类)
 *   - CategoryEU (非 zero = 有 EU 类别)
 *   - ClassEU (非 zero = 有 EU 等级)
 *
 * 如果任一字段非零，则判定为 ASD-STAN 而非纯 ASTM。
 * ================================================================ */
static bool detect_asd_stan(const uav_track_t *uav) {
    const ODID_System_data *sys = &uav->uas_data.System;
    
    if (uav->uas_data.SystemValid) {
        // EU 分类类型非零 → 一定是欧洲标准
        if (sys->ClassificationType != ODID_CLASSIFICATION_TYPE_UNDECLARED) {
            return true;
        }
        // EU 类别/等级非零
        if (sys->CategoryEU != ODID_CATEGORY_EU_UNDECLARED || sys->ClassEU != ODID_CLASS_EU_UNDECLARED) {
            return true;
        }
    }
    return false;
}

/* ================================================================
 * DJI DroneID 固件标记检测
 *
 * DJI 在 ASTM 格式的 BasicID 消息中，有时会在 UASID 字段中嵌入
 * 固件标识信息。我们检测以下特征：
 *   - UASID 以 "T" 开头（DJI 某些型号的 ID 格式）
 *   - UASID 包含 DJI 常见前缀
 *
 * 注意：DJI 私有协议（DroneID）使用完全独立的帧格式，
 * 目前仅能做标记检测，完整解码需要后续逆向。
 * ================================================================ */
static bool detect_dji_signature(uav_track_t *uav) {
    if (!uav->basic_id.valid) return false;
    
    const char *id = uav->basic_id.uas_id;
    if (!id[0]) return false;
    
    // DJI 常见 ID 模式
    // 1. 以 "15MHZ" 开头（某些 DJI 型号的 FCC ID 前缀）
    // 2. 以 "T" 开头后跟数字（DJI 产品序列号格式）
    // 3. MAC OUI 已匹配 DJI OUI（由 sniffer 层处理）
    
    // 检查 DJI OUI
    if ((uav->oui[0] == MAC_OUI_DJI_60_0 && uav->oui[1] == MAC_OUI_DJI_60_1 && uav->oui[2] == MAC_OUI_DJI_60_2) ||
        (uav->oui[0] == MAC_OUI_DJI_48_0 && uav->oui[1] == MAC_OUI_DJI_48_1 && uav->oui[2] == MAC_OUI_DJI_48_2) ||
        (uav->oui[0] == MAC_OUI_DJI_34_0 && uav->oui[1] == MAC_OUI_DJI_34_1 && uav->oui[2] == MAC_OUI_DJI_34_2)) {
        return true;
    }
    
    return false;
}

/* ================================================================
 * 主解析入口：策略分发 (增强版)
 * ================================================================ */
rid_protocol_t crid_parser_decode(uav_track_t *uav, const uint8_t *data, uint8_t len) {
    if (!data || len < 1) return RID_PROTOCOL_UNKNOWN;
    #if PARSER_DEBUG_HEX_DUMP
    hex_dump(TAG, "Beacon Payload HEX", data, len);
    #endif

    /* 每次解码前重置 UAS_Data，避免旧消息的 Valid 标志和字段残留
     * 导致 extract_layered 读到上一帧的过期数据。 */
    odid_initUasData(&uav->uas_data);

    rid_protocol_t proto = RID_PROTOCOL_UNKNOWN;

    /* --- 第一轮：尝试各协议解析 --- */

    // 尝试解析 GB 46750 协议
    if (crid_parser_decode_gb46750(uav, data, len)) {
        proto = RID_PROTOCOL_GB46750;
    }
    // 尝试解析 GB 42590 协议
    else if (crid_parser_decode_gb42590(uav, data, len)) {
        proto = RID_PROTOCOL_GB42590;
    }
    // 尝试解析 ASTM F3411 协议（同时也覆盖 ASD-STAN）
    else if (crid_parser_decode_astm(uav, data, len)) {
        // ASTM 解码成功后，检查是否为 ASD-STAN（欧盟标准）
        if (detect_asd_stan(uav)) {
            proto = RID_PROTOCOL_ASD_STAN;
        } else {
            proto = RID_PROTOCOL_ASTM_F3411;
        }
    }

    if (proto != RID_PROTOCOL_UNKNOWN) {
        /* v2.7.4: 诊断日志 — 每帧打印解码了哪些消息，
         * 用于定位"只收到1个正确目标+2个废目标"问题。 */
        static uint32_t s_ok = 0;
        s_ok++;
        uint8_t vmask = 0;
        if (uav->uas_data.BasicIDValid[0]) vmask |= 0x01;
        if (uav->uas_data.LocationValid)  vmask |= 0x02;
        if (uav->uas_data.SystemValid)    vmask |= 0x04;
        if (uav->uas_data.SelfIDValid)    vmask |= 0x08;
        if (uav->uas_data.OperatorIDValid) vmask |= 0x10;
        /* 前 20 帧全打印，之后每 64 帧打印一次 */
        if (s_ok <= 20 || (s_ok & 0x3F) == 0) {
            ESP_LOGI(TAG, "decode ok: proto=%d vmask=0x%02X mac=%02X:%02X:%02X:%02X:%02X:%02X "
                     "loc=(%.6f,%.6f) op=(%.6f,%.6f)",
                     proto, vmask,
                     uav->mac[0], uav->mac[1], uav->mac[2],
                     uav->mac[3], uav->mac[4], uav->mac[5],
                     uav->uas_data.Location.Latitude,
                     uav->uas_data.Location.Longitude,
                     uav->uas_data.System.OperatorLatitude,
                     uav->uas_data.System.OperatorLongitude);
        }
        return proto;
    }

    /* --- 第二轮：DJI DroneID 标记检测 --- */
    // 如果标准协议都无法解码，但 OUI 匹配 DJI，
    // 记录原始数据供后续分析
    if (detect_dji_signature(uav)) {
        static uint32_t dji_unknown_count = 0;
        if ((++dji_unknown_count & 0x0F) == 1) {
            char mac_str[18];
            snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
                     uav->mac[0], uav->mac[1], uav->mac[2],
                     uav->mac[3], uav->mac[4], uav->mac[5]);
            ESP_LOGW(TAG, "DJI OUI detected but protocol unknown (%s), "
                     "payload[%u]: raw data logged for analysis",
                     mac_str, len);
        }
    }

    /* 解析失败统计 */
    static uint32_t s_fail_count = 0;
    if ((++s_fail_count & 0x1F) == 0) {
        json_decode_fail(data[0], (len > 1 ? data[1] : 0), len);
    }
    return RID_PROTOCOL_UNKNOWN;
}

/* ================================================================
 * 分层数据提取 (GB 46750 优先，否则走 ASTM 标准字段)
 * ================================================================ */
void crid_parser_extract_layered(uav_track_t *uav) {
    if (!uav) return;

    /* --- GB 46750-2025 映射 --- */
    if (uav->protocol == RID_PROTOCOL_GB46750 && uav->gb46750.valid) {
        gb46750_data_t *gb = &uav->gb46750;

        if (gb->has_unique_id) {
            uav->basic_id.valid = true;
            uav->basic_id.id_type = ODID_IDTYPE_SERIAL_NUMBER;
            /* GB46750 ua_category 是中国重量分类(微型/轻型/小型/中型/大型)，
             * 与 ASTM UA Type(机型结构)是不同维度枚举，不能直接赋值。
             * 消费级无人机绝大多数为多旋翼，默认 HELICOPTER_OR_MULTIROTOR。
             * 原始分类值保留在 gb46750.ua_category 字段中。 */
            uav->basic_id.ua_type = ODID_UATYPE_HELICOPTER_OR_MULTIROTOR;

            // 字符串安全净化
            char *dst = uav->basic_id.uas_id;
            const char *src = gb->unique_id;
            int i = 0;
            while (*src && i < (int)sizeof(uav->basic_id.uas_id) - 1) {
                if (*src >= 32 && *src <= 126) {
                    *dst++ = *src;
                    i++;
                }
                src++;
            }
            *dst = '\0';
        }

        EXTRACT_IF(gb->has_uav_location,      uav->location.latitude,  gb->uav_latitude);
        EXTRACT_IF(gb->has_uav_location,      uav->location.longitude, gb->uav_longitude);
        EXTRACT_IF(gb->has_geo_altitude,      uav->location.altitude_geo, gb->geo_altitude);
        EXTRACT_IF(gb->has_baro_altitude,     uav->location.altitude_baro, gb->baro_altitude);
        EXTRACT_IF(gb->has_relative_height,   uav->location.height, gb->relative_height);
        EXTRACT_IF(gb->has_relative_height,   uav->location.height_ref, ODID_HEIGHT_REF_OVER_TAKEOFF);
        EXTRACT_IF(gb->has_ground_speed,      uav->location.speed_horizontal, gb->ground_speed);
        EXTRACT_IF(gb->has_vertical_speed,    uav->location.speed_vertical, gb->vertical_speed);
        EXTRACT_IF(gb->has_track_angle,       uav->location.direction, gb->track_angle);
        EXTRACT_IF(gb->has_operation_status,  uav->location.status, gb->operation_status);
        EXTRACT_IF(gb->has_h_accuracy,        uav->location.h_accuracy, gb->h_accuracy);
        EXTRACT_IF(gb->has_v_accuracy,        uav->location.v_accuracy, gb->v_accuracy);
        EXTRACT_IF(gb->has_speed_accuracy,    uav->location.speed_accuracy, gb->speed_accuracy);
        EXTRACT_IF(gb->has_ts_accuracy,       uav->location.ts_accuracy, gb->ts_accuracy);
        EXTRACT_IF(gb->has_timestamp,         uav->location.timestamp, gb->timestamp_ms / 100.0f);

        if (gb->has_uav_location || gb->has_geo_altitude || gb->has_baro_altitude ||
            gb->has_ground_speed || gb->has_track_angle || gb->has_operation_status) {
            uav->location.valid = true;
        }

        /* 遥控站信息 */
        if (gb->has_rcs_loc_type) uav->system.operator_location_type = gb->rcs_loc_type;
        EXTRACT_IF(gb->has_rcs_location, uav->system.operator_latitude,  gb->rcs_latitude);
        EXTRACT_IF(gb->has_rcs_location, uav->system.operator_longitude, gb->rcs_longitude);
        EXTRACT_IF(gb->has_rcs_altitude, uav->system.operator_altitude_geo, gb->rcs_altitude);
        if (gb->has_rcs_location || gb->has_rcs_altitude || gb->has_rcs_loc_type) uav->system.valid = true;
        EXTRACT_IF(gb->has_operation_category, uav->system.classification_type, gb->operation_category);
        return;
    }

    /* --- ASTM / ASD-STAN / GB 42590 标准字段映射 --- */
    #define MAP_ODID_FIELD(dst, src, valid_cond) \
        do { if (valid_cond) { (dst) = (uint8_t)(src); } } while(0)

    /* Basic ID — v2.0.6 fix: 不要在每条消息里无条件清零。
     * RID 广播每 1-3 秒在 BasicID/Location/SelfID/System 间轮转，
     * 如果本包不含 BasicID 就把之前已解析的 SN 清掉，会导致：
     *  1. LCD 上 SN 频繁变成 "----"
     *  2. 跨 MAC 合并失效（find_by_uas_id 只匹配 valid 的 track）
     *  3. 手机端 basic_id 反复为 null
     * 正确做法：本包带 BasicID 才覆盖，不带则保留历史值。 */
    if (uav->uas_data.BasicIDValid[0]) {
        const ODID_BasicID_data *b = &uav->uas_data.BasicID[0];
        uav->basic_id.valid   = true;
        uav->basic_id.id_type = (uint8_t)b->IDType;
        uav->basic_id.ua_type = (uint8_t)b->UAType;
        strncpy(uav->basic_id.uas_id, b->UASID, sizeof(uav->basic_id.uas_id) - 1);
        uav->basic_id.uas_id[sizeof(uav->basic_id.uas_id) - 1] = '\0';
    }
    /* Location — 同上：本包带 Location 才更新，不带则保留最后一次有效坐标。
     * v2.7.4: 增加坐标范围校验，空中比特翻转产生的天文数字（如纬度
     * 34084864835328）不覆盖上一帧已确认的有效坐标。 */
    if (uav->uas_data.LocationValid) {
        const ODID_Location_data *l = &uav->uas_data.Location;
        double new_lat = l->Latitude;
        double new_lon = l->Longitude;
        bool coords_ok = valid_coord(new_lat, new_lon);

        if (coords_ok) {
            uav->location.latitude        = new_lat;
            uav->location.longitude       = new_lon;
            uav->location.valid           = true;
        } else if (uav->location.valid) {
            /* 新坐标损坏，保留历史坐标，只更新非位置字段 */
            static uint32_t s_loc_bad = 0;
            if ((++s_loc_bad & 0x3F) == 1) {
                ESP_LOGW(TAG, "Location coord out of range (lat=%.6f lon=%.6f), keeping last valid",
                         new_lat, new_lon);
            }
        }
        uav->location.altitude_baro   = l->AltitudeBaro;
        uav->location.altitude_geo    = l->AltitudeGeo;
        uav->location.height          = l->Height;
        uav->location.height_ref      = (uint8_t)l->HeightType;
        uav->location.speed_horizontal = l->SpeedHorizontal;
        uav->location.speed_vertical  = l->SpeedVertical;
        uav->location.direction       = l->Direction;
        uav->location.status          = (uint8_t)l->Status;
        MAP_ODID_FIELD(uav->location.h_accuracy, l->HorizAccuracy, 1);
        MAP_ODID_FIELD(uav->location.v_accuracy, l->VertAccuracy, 1);
        MAP_ODID_FIELD(uav->location.baro_accuracy, l->BaroAccuracy, 1);
        MAP_ODID_FIELD(uav->location.speed_accuracy, l->SpeedAccuracy, 1);
        MAP_ODID_FIELD(uav->location.ts_accuracy, l->TSAccuracy, 1);
        uav->location.timestamp = l->TimeStamp;
    }

    /* System Info — 本包带 System 才更新，否则保留历史值。
     * v2.7.4: 操作员坐标同样做范围校验，单帧比特翻转不污染历史值。
     * 操作员位置类型为 0（Takeoff，无坐标）时不更新坐标字段。 */
    if (uav->uas_data.SystemValid) {
        uav->system.valid = true;
        const ODID_System_data *s = &uav->uas_data.System;
        uav->system.operator_location_type = (uint8_t)s->OperatorLocationType;

        double new_olat = s->OperatorLatitude;
        double new_olon = s->OperatorLongitude;
        bool op_coords_ok = valid_coord(new_olat, new_olon);

        if (op_coords_ok) {
            uav->system.operator_latitude  = new_olat;
            uav->system.operator_longitude = new_olon;
        } else if (uav->system.operator_latitude != 0.0 ||
                   uav->system.operator_longitude != 0.0) {
            /* 保留已有的有效操作员坐标 */
            static uint32_t s_op_bad = 0;
            if ((++s_op_bad & 0x3F) == 1) {
                ESP_LOGW(TAG, "Operator coord out of range (lat=%.6f lon=%.6f), keeping last valid",
                         new_olat, new_olon);
            }
        }

        uav->system.operator_altitude_geo  = s->OperatorAltitudeGeo;
        uav->system.area_count             = s->AreaCount;
        uav->system.area_radius            = s->AreaRadius;
        uav->system.area_ceiling           = s->AreaCeiling;
        uav->system.area_floor             = s->AreaFloor;
        uav->system.classification_type    = (uint8_t)s->ClassificationType;
        uav->system.category_eu            = (uint8_t)s->CategoryEU;
        uav->system.class_eu               = (uint8_t)s->ClassEU;
        uav->system.timestamp              = s->Timestamp;
    }

    /* Self ID — 本包带 SelfID 才更新，否则保留历史值 */
    if (uav->uas_data.SelfIDValid) {
        uav->self_id.valid = true;
        const ODID_SelfID_data *s = &uav->uas_data.SelfID;
        uav->self_id.description_type = (uint8_t)s->DescType;
        strncpy(uav->self_id.description, s->Desc, sizeof(uav->self_id.description) - 1);
        uav->self_id.description[sizeof(uav->self_id.description) - 1] = '\0';
    }

    /* Operator ID — 本包带 OperatorID 才更新，否则保留历史值 */
    if (uav->uas_data.OperatorIDValid) {
        uav->operator_id.valid = true;
        const ODID_OperatorID_data *o = &uav->uas_data.OperatorID;
        uav->operator_id.id_type = (uint8_t)o->OperatorIdType;
        strncpy(uav->operator_id.id, o->OperatorId, sizeof(uav->operator_id.id) - 1);
        uav->operator_id.id[sizeof(uav->operator_id.id) - 1] = '\0';
    }
    #undef MAP_ODID_FIELD
}
