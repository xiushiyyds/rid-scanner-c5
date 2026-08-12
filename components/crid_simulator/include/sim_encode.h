/**
 * sim_encode.h — GB42590 RID 信标帧编码器
 *
 * 从原始 esp32-crid-sim-OTA 项目的 encode_gb42590.c 移植。
 * 仅保留 GB42590 的 3 条消息版本（BasicID + Location + System）。
 * 函数前缀改为 sim_encode_* 避免与扫描端冲突。
 */

#ifndef SIM_ENCODE_H
#define SIM_ENCODE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --- 报文常量 --- */
#define SIM_MESSAGE_SIZE      25     // 每条报文 25 字节
#define SIM_UAS_ID_MAX_LEN    20     // UAS ID 最大长度

/* --- 中国 C-RID 标准 OUI 和类型 --- */
#define SIM_OUI_0 0xFA
#define SIM_OUI_1 0x0B
#define SIM_OUI_2 0xBC
#define SIM_VENDOR_TYPE 0x0D

/* --- 报文类型 (符合 ASTM F3411 / ASD-STAN 4709-002) --- */
#define SIM_MSG_TYPE_BASIC_ID    0x0
#define SIM_MSG_TYPE_LOCATION    0x1
#define SIM_MSG_TYPE_SYSTEM      0x4

/* ================================================================
 * 编码器内部使用的配置结构体
 * （与 sim_core.h 的 sim_config_t 不同，这里包含编码所需的全部字段）
 * ================================================================ */
typedef struct {
    uint8_t mac_address[6];      // 源 MAC 地址
    char uas_id[SIM_UAS_ID_MAX_LEN + 1];  // UAS ID
    uint8_t id_type;             // ID 类型 (1=序列号)
    uint8_t ua_type;             // UA 类型 (1=直升机)
    float latitude;              // 当前纬度
    float longitude;             // 当前经度
    float altitude_msl;          // 气压高度 (m)
    float altitude_agl;          // 相对高度 (m)
    float speed_horizontal;      // 水平速度 (m/s)
    float speed_vertical;        // 垂直速度 (m/s)
    float heading;               // 航向 (0~360°)
    uint8_t status;              // 飞行状态 (1=空中)
    float operator_lat;          // 操作员纬度
    float operator_lon;          // 操作员经度
    float operator_alt;          // 操作员高度 (m)
    uint8_t operator_location_type;
    uint8_t classification_type;
    uint8_t category_eu;
    uint8_t class_eu;
    uint8_t height_type;
    char ssid[33];               // SSID
    uint8_t channel;             // Wi-Fi 信道
} sim_encode_config_t;

/**
 * 构建 Basic ID 报文 (25 字节)
 */
void sim_encode_basic_id(const sim_encode_config_t *cfg, uint8_t *message);

/**
 * 构建 Location 报文 (25 字节)
 */
void sim_encode_location(const sim_encode_config_t *cfg, uint8_t *message);

/**
 * 构建 System 报文 (25 字节)
 */
void sim_encode_system(const sim_encode_config_t *cfg, uint8_t *message);

/**
 * 构建完整的 Beacon 帧（含 3 条打包消息）
 * @param cfg 编码配置
 * @param message_counter 消息计数器
 * @param frame 输出帧缓冲区
 * @param max_len 缓冲区最大长度
 * @param out_len 实际帧长度
 * @return true 成功
 */
bool sim_encode_beacon_frame(const sim_encode_config_t *cfg,
                              uint8_t message_counter,
                              uint8_t *frame, uint16_t max_len,
                              uint16_t *out_len);

#ifdef __cplusplus
}
#endif

#endif // SIM_ENCODE_H
