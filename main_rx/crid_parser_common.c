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

    /* --- 第一轮：尝试各协议解析 --- */

    // 尝试解析 GB 46750 协议
    if (crid_parser_decode_gb46750(uav, data, len)) {
        return RID_PROTOCOL_GB46750;
    }

    // 尝试解析 GB 42590 协议
    if (crid_parser_decode_gb42590(uav, data, len)) {
        return RID_PROTOCOL_GB42590;
    }

    // 尝试解析 ASTM F3411 协议（同时也覆盖 ASD-STAN）
    if (crid_parser_decode_astm(uav, data, len)) {
        // ASTM 解码成功后，检查是否为 ASD-STAN（欧盟标准）
        if (detect_asd_stan(uav)) {
            return RID_PROTOCOL_ASD_STAN;
        }
        return RID_PROTOCOL_ASTM_F3411;
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
            uav->basic_id.ua_type = gb->has_ua_category ? gb->ua_category : ODID_UATYPE_HELICOPTER_OR_MULTIROTOR;

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
        EXTRACT_IF(gb->has_timestamp,         uav->location.timestamp, gb->timestamp_ms / 1000.0f);

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

    /* Basic ID */
    uav->basic_id.valid = false;
    if (uav->uas_data.BasicIDValid[0]) {
        const ODID_BasicID_data *b = &uav->uas_data.BasicID[0];
        uav->basic_id.valid   = true;
        uav->basic_id.id_type = (uint8_t)b->IDType;
        uav->basic_id.ua_type = (uint8_t)b->UAType;
        strncpy(uav->basic_id.uas_id, b->UASID, sizeof(uav->basic_id.uas_id) - 1);
        uav->basic_id.uas_id[sizeof(uav->basic_id.uas_id) - 1] = '\0';
    }
    /* Location */
    uav->location.valid = uav->uas_data.LocationValid;
    if (uav->location.valid) {
        const ODID_Location_data *l = &uav->uas_data.Location;
        uav->location.latitude        = l->Latitude;
        uav->location.longitude       = l->Longitude;
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

    /* System Info */
    uav->system.valid = uav->uas_data.SystemValid;
    if (uav->system.valid) {
        const ODID_System_data *s = &uav->uas_data.System;
        uav->system.operator_location_type = (uint8_t)s->OperatorLocationType;
        uav->system.operator_latitude      = s->OperatorLatitude;
        uav->system.operator_longitude     = s->OperatorLongitude;
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

    /* Self ID */
    uav->self_id.valid = uav->uas_data.SelfIDValid;
    if (uav->self_id.valid) {
        const ODID_SelfID_data *s = &uav->uas_data.SelfID;
        uav->self_id.description_type = (uint8_t)s->DescType;
        strncpy(uav->self_id.description, s->Desc, sizeof(uav->self_id.description) - 1);
        uav->self_id.description[sizeof(uav->self_id.description) - 1] = '\0';
    }

    /* Operator ID */
    uav->operator_id.valid = uav->uas_data.OperatorIDValid;
    if (uav->operator_id.valid) {
        const ODID_OperatorID_data *o = &uav->uas_data.OperatorID;
        uav->operator_id.id_type = (uint8_t)o->OperatorIdType;
        strncpy(uav->operator_id.id, o->OperatorId, sizeof(uav->operator_id.id) - 1);
        uav->operator_id.id[sizeof(uav->operator_id.id) - 1] = '\0';
    }
    #undef MAP_ODID_FIELD
}
