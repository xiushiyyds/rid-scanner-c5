/**
 * dji_droneid.h — DJI DroneID 私有协议解析模块
 *
 * 参考来源（交叉验证）：
 *   1. Kismet KSY: dot11_ie_221_dji_droneid.ksy
 *   2. DJIDroneIDspoofer: Drone.py (实际编码逻辑)
 *   3. DroneSecurity: droneid_packet.py (实际解码 OcuSync 信号)
 *
 * DJI 无人机通过 Wi-Fi Beacon 帧的 Vendor IE (ID=221) 广播 DroneID：
 * - OUI: 26:37:12
 * - 两种 subcommand：
 *     0x10 = flight telemetry（位置/速度/姿态/序列号/机型）
 *     0x11 = flight purpose（用户输入的飞行目的文本）
 *
 * payload（OUI 之后）布局：
 *   [0..2]  common header (0x58, 0x62, 0x13 — "Xb\x13")
 *   [3]     subcommand (0x10 / 0x11)
 *   [4..]   record
 *
 * 注意：Kismet KSY 将前 3 字节标记为 vendor_type/unk1/unk2，
 * 但 DJIDroneIDspoofer 和 DroneSecurity 均证实这是固定 magic header。
 */

#ifndef DJI_DRONEID_H
#define DJI_DRONEID_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 * DJI OUI 定义
 * ================================================================ */
#define DJI_OUI_0  0x26
#define DJI_OUI_1  0x37
#define DJI_OUI_2  0x12

#define IS_DJI_OUI(o0, o1, o2) \
    ((o0) == DJI_OUI_0 && (o1) == DJI_OUI_1 && (o2) == DJI_OUI_2)

/* DJI Common Header magic: 0x58 0x62 0x13 ("Xb\x13") */
#define DJI_MAGIC_0  0x58
#define DJI_MAGIC_1  0x62
#define DJI_MAGIC_2  0x13

/* ================================================================
 * DJI DroneID subcommand 类型
 * ================================================================ */
#define DJI_DRONEID_TYPE_TELEMETRY   0x10
#define DJI_DRONEID_TYPE_FLIGHT_INFO 0x11

/* ================================================================
 * DJI DroneID state_info 位域（来自 KSY + DroneSecurity 交叉验证）
 * ================================================================ */
#define DJI_STATE_SERIAL_VALID     0x0001
#define DJI_STATE_PRIVATE_DISABLED 0x0002
#define DJI_STATE_HOMEPOINT_SET    0x0004
#define DJI_STATE_UUID_SET         0x0008
#define DJI_STATE_MOTOR_ON         0x0010
#define DJI_STATE_IN_AIR           0x0020
#define DJI_STATE_GPS_VALID        0x0040
#define DJI_STATE_ALT_VALID        0x0080
#define DJI_STATE_HEIGHT_VALID     0x0100
#define DJI_STATE_HORIZ_VALID      0x0200
#define DJI_STATE_VUP_VALID        0x0400
#define DJI_STATE_ATT_VALID        0x0800

/* ================================================================
 * DJI 坐标编码因子
 * ================================================================ */
#define DJI_COORD_SCALE  174533.0

/* ================================================================
 * Telemetry payload 长度
 *   KSY 短格式：version..product_type = 54 字节（无 pilot GPS）
 *   完整格式：common_header(3) + subcommand(1) + record(87) = 91 字节
 * ================================================================ */
#define DJI_TELEMETRY_MIN_LEN  54   /* record 最小长度 */
#define DJI_TELEMETRY_FULL_LEN 87   /* record 完整长度（含 pilot GPS） */
#define DJI_FLIGHT_INFO_MIN_LEN 28

/* ================================================================
 * DJI DroneID 解析结果
 * ================================================================ */
typedef struct {
    bool     valid;
    uint8_t  type;               // 0x10=telemetry, 0x11=flight purpose

    // 通用
    char     serial[17];         // 序列号 (16字节 ASCII, null-terminated)
    uint8_t  product_type;       // 机型代码 (u8)
    char     model_name[32];     // 机型名称
    uint16_t state_info;         // 状态位域

    // Telemetry (subcommand 0x10)
    uint8_t  version;           // 协议版本（通常为 2）
    uint16_t sequence_number;    // 帧序号
    double   latitude;           // 纬度 (degrees)
    double   longitude;          // 经度 (degrees)
    float    altitude;           // 海拔高度 (m)
    float    height;             // 相对起飞点高度 (m)
    float    speed_north;        // 北向速度 (m/s)
    float    speed_east;         // 东向速度 (m/s)
    float    speed_up;           // 垂直速度 (m/s, 正=上升)
    float    speed_h;            // 合成水平速度 (m/s)
    float    heading;            // 航向 (度, 0~360)
    float    pitch;              // 俯仰角 (度)
    float    roll;               // 横滚角 (度)
    double   pilot_latitude;     // 操作员/遥控器纬度
    double   pilot_longitude;    // 操作员/遥控器经度
    double   home_latitude;      // 返航点纬度
    double   home_longitude;     // 返航点经度
    bool     has_pilot_gps;      // 是否包含操作员 GPS 数据

    // Flight Purpose (subcommand 0x11)
    char     drone_id[11];       // 无人机 ID (10字节 ASCII)
    char     purpose[64];        // 飞行目的文本

    // RSSI 和时间戳（由调用者填充）
    int8_t   rssi;
    uint32_t timestamp_ms;
} dji_droneid_data_t;

/* ================================================================
 * API 函数
 * ================================================================ */

void dji_droneid_init(void);

bool dji_droneid_is_dji_oui(const uint8_t *oui, uint8_t oui_type);

/**
 * 解析 DJI DroneID payload
 *
 * payload 指向 OUI(3字节) 之后的数据，即从 common header 开始。
 * 布局：
 *   [0..2] common header (0x58,0x62,0x13)
 *   [3]    subcommand
 *   [4..]  record
 *
 * @param payload  原始 payload（OUI 之后）
 * @param len  payload 长度
 * @param[out] data  解析结果
 * @return 0 成功，负数失败
 */
int dji_droneid_parse(const uint8_t *payload, uint16_t len, dji_droneid_data_t *data);

const char* dji_droneid_get_model_name(uint8_t product_type);

#ifdef __cplusplus
}
#endif

#endif // DJI_DRONEID_H
