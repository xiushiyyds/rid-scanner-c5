/**
 * sim_encode_gb46750.h — GB 46750-2025 新国标广播帧编码器
 *
 * Wire 格式 (BLE/WiFi Beacon payload after OUI+VendorType+Counter):
 *   [Magic=0xFF][Ver:1B][DataLen:1B][Flags:3B][Content:变长]
 *
 * 版本号: bit7..5=major(=1 V1.0), bit4..0=minor(=0) → 0x20
 * Flags: 3字节位图, bit0=扩展标志, bit1..7 分别映射数据项
 *
 * 仅做加法，不影响现有 GB42590 编码器路径。
 */

#ifndef SIM_ENCODE_GB46750_H
#define SIM_ENCODE_GB46750_H

#include <stdint.h>
#include <stdbool.h>
#include "sim_encode.h"   /* 复用 sim_encode_config_t */
#include "sim_core.h"     /* sim_brand_t */

#ifdef __cplusplus
extern "C" {
#endif

/* GB46750 常量 */
#define GB46750_MAGIC            0xFF
#define GB46750_VERSION_1_0      0x20   /* V1.0: major=1, minor=0 */
#define GB46750_HEADER_LEN       6      /* Magic+Ver+Len+Flags(3) */
#define GB46750_UPC_LEN          20
#define GB46750_REALNAME_LEN     8

/**
 * 生成 20 位 UPC (14 位厂商码 + 6 位流水号)。
 * @param brand    品牌枚举（用于选择厂商前缀）
 * @param seq      流水号（取低 6 位十进制）
 * @param out      输出缓冲区，至少 21 字节
 * @param out_len  缓冲区长度
 */
void sim_gb46750_gen_upc(sim_brand_t brand, uint32_t seq,
                          char *out, size_t out_len);

/**
 * 构建 GB46750 完整 Beacon 帧（802.11 Beacon + Vendor IE + GB46750 PDU）。
 *
 * 与 GB42590 版本签名一致，方便 sim_core 按协议分发。
 *
 * @param cfg             编码配置（复用 sim_encode_config_t）
 * @param upc             20 位 UPC 字符串
 * @param message_counter 消息计数器（Vendor IE 头后 1B）
 * @param frame           输出帧缓冲
 * @param max_len         缓冲最大长度
 * @param out_len         实际帧长
 * @return true 成功
 */
bool sim_encode_gb46750_beacon_frame(const sim_encode_config_t *cfg,
                                      const char *upc,
                                      uint8_t message_counter,
                                      uint8_t *frame, uint16_t max_len,
                                      uint16_t *out_len);

#ifdef __cplusplus
}
#endif

#endif /* SIM_ENCODE_GB46750_H */
