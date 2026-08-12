/**
 * dji_droneid.c — DJI DroneID 私有协议解析实现
 *
 * 交叉验证三个来源：
 *   - Kismet KSY: 字段定义和坐标换算
 *   - DJIDroneIDspoofer Drone.py: 实际字节编码顺序（yaw/roll/pitch, pilot GPS）
 *   - DroneSecurity droneid_packet.py: 实际解码真实信号的 struct 格式
 *
 * payload 布局（OUI 26:37:12 之后）：
 *   [0..2] common header: 0x58, 0x62, 0x13 ("Xb\x13")
 *   [3]    subcommand: 0x10=telemetry, 0x11=flight purpose
 *   [4..]  record
 *
 * Telemetry record (subcommand 0x10), 两种长度：
 *
 * 短格式 (54 字节, KSY 定义，部分旧固件)：
 *   [0]    version (u8, 通常 0x02)
 *   [1..2] seq (u16 LE)
 *   [3..4] state_info (u16 LE)
 *   [5..20] serialnumber (16 ASCII)
 *   [21..24] raw_lon (s32 LE, /174533.0)
 *   [25..28] raw_lat (s32 LE, /174533.0)
 *   [29..30] altitude (s16 LE, m 或 ft*0.3048)
 *   [31..32] height (s16 LE)
 *   [33..34] v_north (s16 LE, cm/s)
 *   [35..36] v_east (s16 LE, cm/s)
 *   [37..38] v_up (s16 LE, cm/s)
 *   [39..40] raw_yaw (s16 LE, (deg-180)*100)
 *   [41..42] raw_roll (s16 LE, deg*100)
 *   [43..44] raw_pitch (s16 LE, deg*100)
 *   [45..48] raw_home_lon (s32 LE, /174533.0)
 *   [49..52] raw_home_lat (s32 LE, /174533.0)
 *   [53]    product_type (u8)
 *
 * 完整格式 (87 字节 record，DJIDroneIDspoofer/DroneSecurity，V2 固件)：
 *   在 yaw/roll/pitch 之后、home 坐标之前插入 pilot GPS 块：
 *   [45..52] phone_app_gps_time (u64 LE, Unix ms)
 *   [53..56] pilot_lat (s32 LE, /174533.0)
 *   [57..60] pilot_lon (s32 LE, /174533.0)
 *   [61..64] raw_home_lon
 *   [65..68] raw_home_lat
 *   [69]    product_type
 *   [70]    uuid_len
 *   [71..90] uuid (20 bytes)
 *   末尾还有 2 字节 CRC（DroneSecurity 校验）
 *
 * 注意：KSY 的字段顺序是 pitch/roll/yaw，但 DJIDroneIDspoofer 的实际
 * 编码顺序是 yaw/roll/pitch。DroneSecurity 只提取第一个（yaw），
 * 其余两个未使用。我们以 spoofer 的实际编码为准。
 */

#include <string.h>
#include <stdio.h>
#include <math.h>
#include "dji_droneid.h"
#include "esp_log.h"

static const char *TAG = "DJI_DRONEID";

/* ================================================================
 * 小端读取辅助
 * ================================================================ */
static inline uint16_t rd_u16le(const uint8_t *p) {
    return (uint16_t)(p[0] | (p[1] << 8));
}
static inline int16_t rd_s16le(const uint8_t *p) {
    return (int16_t)(p[0] | (p[1] << 8));
}
static inline uint32_t rd_u32le(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static inline int32_t rd_s32le(const uint8_t *p) {
    return (int32_t)rd_u32le(p);
}
static inline uint64_t rd_u64le(const uint8_t *p) {
    return (uint64_t)rd_u32le(p) | ((uint64_t)rd_u32le(p + 4) << 32);
}

/* ================================================================
 * DJI 机型名称映射
 *
 * 注意：DroneSecurity 和 Kismet 的机型映射有差异！
 * 这里采用 DroneSecurity（实际解码真实信号）的映射，
 * 部分 Kismet 独有的条目作为补充。
 * ================================================================ */
static const char *dji_product_type_name(uint8_t pt) {
    switch (pt) {
        case 1:  return "Inspire 1";
        case 2:
        case 3:  return "Phantom 3 Series";
        case 4:  return "Phantom 3 Standard";
        case 5:  return "Matrice 100";
        case 6:  return "ACEONE";
        case 7:  return "WKM";
        case 8:  return "NAZA";
        case 9:  return "A2";
        case 10: return "A3";
        case 11: return "Phantom 4";
        case 12: return "MG1";
        case 14: return "M600";
        case 15: return "Phantom 3 4K";
        case 16: return "Mavic Pro";
        case 17: return "Inspire 2";
        case 18: return "Phantom 4 Pro";
        case 20: return "N2";
        case 21: return "Spark";
        case 23: return "M600 Pro";
        case 24: return "Mavic Air";
        case 25: return "Matrice 200";
        case 26: return "Phantom 4 Series";
        case 27: return "Phantom 4 Advanced";
        case 28: return "Matrice 210";
        case 30: return "M210 RTK";
        case 31: return "A3 AG";
        case 32: return "MG2";
        case 34: return "MG1A";
        case 35: return "Phantom 4 RTK";
        case 36: return "Phantom 4 Pro V2";
        case 38: return "M210 V2 / MG1P";
        case 40: return "M210 RTK V2 / MG1P-RTK";
        case 41: return "Mavic 2";
        case 44: return "M200 V2 Series";
        case 45: return "Mavic 2 Pro";
        case 51: return "Mavic 2 Enterprise";
        case 53: return "Mavic Mini";
        case 58: return "Mavic Air 2";
        case 59: return "P4M / Mini SE";
        case 60: return "M300 RTK";
        case 61: return "DJI FPV";
        case 63: return "Mini 2 / Air 2S";
        case 64: return "AGRAS T10";
        case 65: return "AGRAS T30";
        case 66: return "Air 2S / Mavic 3";
        case 67: return "M30";
        case 68: return "Mavic 3";
        case 69: return "Mavic 2 Enterprise Advanced";
        case 70: return "Mini SE";
        /* 以下为 Kismet 补充映射（新固件机型） */
        case 74: return "M30";
        case 75: return "M30T";
        case 76: return "Mini 3";
        case 77: return "Mini 3 Pro";
        case 79: return "Mavic 3 Classic";
        case 81: return "Mavic 3M";
        case 83: return "M3E/M3T";
        case 86: return "M300 RTK";
        case 87: return "M350 RTK";
        case 88: return "Avata";
        case 90: return "FPV";
        case 96: return "Mini 4 Pro";
        case 98: return "Air 3";
        case 100: return "Mavic 3 Pro";
        case 102: return "Avata 2";
        case 103: return "Mini 4K";
        case 104: return "Neo";
        default: return "Unknown";
    }
}

/* ================================================================
 * 初始化
 * ================================================================ */
void dji_droneid_init(void) {
    ESP_LOGI(TAG, "DJI DroneID parser initialized (v2, cross-validated)");
}

/* ================================================================
 * 检查是否为 DJI OUI
 * ================================================================ */
bool dji_droneid_is_dji_oui(const uint8_t *oui, uint8_t oui_type) {
    (void)oui_type;
    if (!oui) return false;
    return IS_DJI_OUI(oui[0], oui[1], oui[2]);
}

/* ================================================================
 * 获取机型名称
 * ================================================================ */
const char* dji_droneid_get_model_name(uint8_t product_type) {
    return dji_product_type_name(product_type);
}

/* ================================================================
 * 解析 Telemetry (subcommand 0x10)
 *
 * record 从 payload[4] (version) 开始。
 * 根据 record 长度自动判断短格式(54)或完整格式(87)。
 * ================================================================ */
static int parse_telemetry(const uint8_t *record, uint16_t rec_len,
                           dji_droneid_data_t *data) {
    /* 短格式最小长度：54 字节到 product_type */
    if (rec_len < DJI_TELEMETRY_MIN_LEN) {
        ESP_LOGD(TAG, "Telemetry too short: %d bytes (min %d)",
                 rec_len, DJI_TELEMETRY_MIN_LEN);
        return -1;
    }

    data->version         = record[0];
    data->sequence_number = rd_u16le(&record[1]);
    data->state_info      = rd_u16le(&record[3]);

    /* 序列号：16 字节 ASCII */
    memcpy(data->serial, &record[5], 16);
    data->serial[16] = '\0';
    for (int i = 15; i >= 0; i--) {
        if (data->serial[i] == '\0' || data->serial[i] == ' ')
            data->serial[i] = '\0';
        else
            break;
    }

    /* 经纬度：s32 LE / 174533.0 */
    int32_t raw_lon = rd_s32le(&record[21]);
    int32_t raw_lat = rd_s32le(&record[25]);
    data->longitude = (double)raw_lon / DJI_COORD_SCALE;
    data->latitude  = (double)raw_lat / DJI_COORD_SCALE;

    /* 高度：s16 LE
     * 注意：DroneSecurity 解码时 /3.281 (英尺→米)，
     * 但 DJIDroneIDspoofer 直接 pack_uint16(altitude_meters)。
     * 不同固件版本可能不同，暂按米处理，真机验证后调整。 */
    int16_t alt = rd_s16le(&record[29]);
    int16_t hgt = rd_s16le(&record[31]);
    data->altitude = (float)alt;
    data->height   = (float)hgt;

    /* 速度：s16 LE, cm/s → m/s */
    int16_t v_north = rd_s16le(&record[33]);
    int16_t v_east  = rd_s16le(&record[35]);
    int16_t v_up    = rd_s16le(&record[37]);
    data->speed_north = (float)v_north / 100.0f;
    data->speed_east  = (float)v_east / 100.0f;
    data->speed_up    = (float)v_up / 100.0f;
    data->speed_h     = sqrtf(data->speed_north * data->speed_north +
                              data->speed_east * data->speed_east);

    /*
     * 姿态角（以 DJIDroneIDspoofer 编码顺序为准）：
     *   record[39..40] = yaw:   (deg - 180) * 100
     *   record[41..42] = roll:  deg * 100
     *   record[43..44] = pitch: deg * 100
     *
     * KSY 写的顺序是 pitch/roll/yaw，但 spoofer 实际编码是 yaw/roll/pitch。
     * DroneSecurity 只提取 d_1_angle（第一个，即 yaw）。
     */
    int16_t raw_yaw   = rd_s16le(&record[39]);
    int16_t raw_roll  = rd_s16le(&record[41]);
    int16_t raw_pitch = rd_s16le(&record[43]);

    /* yaw: raw = (deg - 180) * 100, 还原: deg = raw/100 + 180 */
    float yaw = (float)raw_yaw / 100.0f + 180.0f;
    if (yaw >= 360.0f) yaw -= 360.0f;
    if (yaw < 0.0f)    yaw += 360.0f;
    data->heading = yaw;

    data->roll  = (float)raw_roll / 100.0f;
    data->pitch = (float)raw_pitch / 100.0f;

    /*
     * 判断是否包含 pilot GPS 块（16 字节）：
     *   完整格式 record >= 70 字节（到 product_type + uuid_len）
     *   短格式 record 54 字节（到 product_type）
     *
     * 完整格式在 attitude 之后：
     *   [45..52] phone_app_gps_time (u64)
     *   [53..56] pilot_lat (s32)
     *   [57..60] pilot_lon (s32)
     *   [61..64] home_lon (s32)
     *   [65..68] home_lat (s32)
     *   [69]     product_type
     *   [70]     uuid_len
     *   [71..90] uuid (20)
     */
    if (rec_len >= 70) {
        /* 完整格式：含 pilot GPS */
        data->has_pilot_gps = true;

        /* phone_app_gps_time 在 record[45..52]，暂不使用 */
        (void)rd_u64le(&record[45]);

        int32_t pilot_lat = rd_s32le(&record[53]);
        int32_t pilot_lon = rd_s32le(&record[57]);
        data->pilot_latitude  = (double)pilot_lat / DJI_COORD_SCALE;
        data->pilot_longitude = (double)pilot_lon / DJI_COORD_SCALE;

        int32_t raw_home_lon = rd_s32le(&record[61]);
        int32_t raw_home_lat = rd_s32le(&record[65]);
        data->home_longitude = (double)raw_home_lon / DJI_COORD_SCALE;
        data->home_latitude  = (double)raw_home_lat / DJI_COORD_SCALE;

        data->product_type = record[69];
    } else {
        /* 短格式：无 pilot GPS */
        data->has_pilot_gps = false;
        data->pilot_latitude  = 0.0;
        data->pilot_longitude = 0.0;

        int32_t raw_home_lon = rd_s32le(&record[45]);
        int32_t raw_home_lat = rd_s32le(&record[49]);
        data->home_longitude = (double)raw_home_lon / DJI_COORD_SCALE;
        data->home_latitude  = (double)raw_home_lat / DJI_COORD_SCALE;

        data->product_type = record[53];
    }

    strncpy(data->model_name, dji_product_type_name(data->product_type),
            sizeof(data->model_name) - 1);
    data->model_name[sizeof(data->model_name) - 1] = '\0';

    ESP_LOGI(TAG, "DJI Telemetry: SN=%s model=%s(pt=%d) v%d seq=%d "
             "pos=%.6f,%.6f alt=%.1fm hgt=%.1fm "
             "spd=%.1fm/s hdg=%.1f° roll=%.1f pitch=%.1f "
             "pilot=%.6f,%.6f home=%.6f,%.6f state=0x%04X",
             data->serial, data->model_name, data->product_type,
             data->version, data->sequence_number,
             data->latitude, data->longitude,
             data->altitude, data->height,
             data->speed_h, data->heading,
             data->roll, data->pitch,
             data->pilot_latitude, data->pilot_longitude,
             data->home_latitude, data->home_longitude,
             data->state_info);

    return 0;
}

/* ================================================================
 * 解析 Flight Purpose (subcommand 0x11)
 *
 * record 布局（从 payload[4] 开始）：
 *   [0..15] serialnumber (16 ASCII)
 *   [16]    len
 *   [17..26] drone_id (10 ASCII)
 *   [27]    purpose_len
 *   [28..]  purpose text
 * ================================================================ */
static int parse_flight_purpose(const uint8_t *record, uint16_t rec_len,
                                 dji_droneid_data_t *data) {
    if (rec_len < DJI_FLIGHT_INFO_MIN_LEN) {
        ESP_LOGD(TAG, "Flight purpose too short: %d bytes", rec_len);
        return -1;
    }

    /* 序列号 */
    memcpy(data->serial, &record[0], 16);
    data->serial[16] = '\0';
    for (int i = 15; i >= 0; i--) {
        if (data->serial[i] == '\0' || data->serial[i] == ' ')
            data->serial[i] = '\0';
        else
            break;
    }

    /* drone_id 10 字节 */
    memcpy(data->drone_id, &record[17], 10);
    data->drone_id[10] = '\0';

    /* purpose 文本 */
    uint8_t purpose_len = record[27];
    uint16_t avail = rec_len - 28;
    uint16_t copy_len = purpose_len;
    if (copy_len > avail) copy_len = avail;
    if (copy_len >= sizeof(data->purpose)) copy_len = sizeof(data->purpose) - 1;
    memcpy(data->purpose, &record[28], copy_len);
    data->purpose[copy_len] = '\0';

    ESP_LOGI(TAG, "DJI Flight Purpose: SN=%s drone_id=%s purpose=\"%s\"",
             data->serial, data->drone_id, data->purpose);
    return 0;
}

/* ================================================================
 * 解析 DJI DroneID payload（OUI 之后）
 *
 * payload[0..2] = common header (0x58, 0x62, 0x13)
 * payload[3]    = subcommand (0x10 telemetry / 0x11 flight purpose)
 * payload[4..]  = record
 * ================================================================ */
int dji_droneid_parse(const uint8_t *payload, uint16_t len,
                      dji_droneid_data_t *data) {
    if (!payload || !data || len < 4) {
        return -1;
    }

    memset(data, 0, sizeof(dji_droneid_data_t));

    /*
     * 校验 common header magic: 0x58, 0x62, 0x13
     * DJIDroneIDspoofer: DJI_COMMON_HEADER = b'Xb\x13'
     * DroneSecurity: struct starts with pkt_len(B), unk(B), version(B)
     *   其中 pkt_len 字段在 Wi-Fi Beacon 中对应 0x58... 实际
     *   DroneSecurity 处理的是 OcuSync DUML 帧，前两字节含义不同。
     *
     * 对于 Wi-Fi Beacon 中的 Vendor IE payload，前 3 字节在
     * DJIDroneIDspoofer 中是固定 magic，我们做宽松校验：
     * 如果首字节是 0x58，按 magic 格式解析；
     * 否则直接把 payload[3] 当 subcommand（兼容 KSY 格式）。
     */
    uint8_t subcommand;
    const uint8_t *record;
    uint16_t rec_len;

    if (payload[0] == DJI_MAGIC_0 && payload[1] == DJI_MAGIC_1 &&
        payload[2] == DJI_MAGIC_2) {
        /* 标准 DJI 帧：magic(3) + subcommand(1) + record */
        subcommand = payload[3];
        record = payload + 4;
        rec_len = len - 4;
        ESP_LOGD(TAG, "DJI magic header detected, subcommand=0x%02X", subcommand);
    } else {
        /*
         * 兼容 KSY 格式：payload[0]=vendor_type, [1]=unk1, [2]=unk2,
         * [3]=subcommand。此时 subcommand 仍在 offset 3。
         */
        subcommand = payload[3];
        record = payload + 4;
        rec_len = len - 4;
        ESP_LOGD(TAG, "KSY format (no magic), subcommand=0x%02X", subcommand);
    }

    data->type = subcommand;

    int rc = -1;
    if (subcommand == DJI_DRONEID_TYPE_TELEMETRY) {
        rc = parse_telemetry(record, rec_len, data);
    } else if (subcommand == DJI_DRONEID_TYPE_FLIGHT_INFO) {
        rc = parse_flight_purpose(record, rec_len, data);
    } else {
        ESP_LOGD(TAG, "Unknown DJI subcommand: 0x%02X", subcommand);
        return -2;
    }

    if (rc == 0) {
        data->valid = true;
    }
    return rc;
}
