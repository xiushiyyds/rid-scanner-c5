/**
 * crid_parser_astm.c — ASTM F3411 / ASD-STAN 协议解析模块
 *
 * 处理 OpenDroneID MessagePack 格式：
 *   Byte 0: [MessageType:4][ProtoVersion:4]
 *           MessageType = 0xF (PACKED), ProtoVersion = 0x0 (ASTM) or 0x1 (GB42590)
 *   Byte 1: SingleMessageSize (必须 = 25)
 *   Byte 2: MsgPackSize (消息数量, 1..10)
 *   Byte 3+: 各 25 字节消息
 *
 * v1.9 修复：之前错误地检查 data[0] == 0xF2，实际标准值是 0xF0/0xF1，
 * 导致所有标准 RID Beacon 都无法解析。
 */
#include <string.h>
#include "esp_log.h"
#include "opendroneid.h"
#include "odid_wifi.h"
#include "crid_parser.h"
#include "crid_json.h"
#include "crid_rx_types.h"

static const char *TAG = "RID_ASTM";

#define ASTM_MSG_SIZE           25
#define ASTM_PACK_MAX_MSGS      ODID_PACK_MAX_MESSAGES
#define ASTM_MIN_LEN            3   /* header: type/ver + size + count */

/* ================================================================
 * 内部辅助函数
 * ================================================================ */

/* ================================================================
 * ASTM F3411 / ASD-STAN Packed 格式解析
 *
 * 直接调用 opendroneid 的 odid_message_process_pack，它会：
 *   1. 校验 MessageType == 0xF
 *   2. 校验 SingleMessageSize == 25
 *   3. 调用 decodeMessagePack 解码所有消息
 * ================================================================ */
bool crid_parser_decode_astm(uav_track_t *uav, const uint8_t *data, uint8_t len) {
    if (!data || len < ASTM_MIN_LEN) return false;

    uint8_t msg_type    = (data[0] >> 4) & 0x0F;
    uint8_t proto_ver   = data[0] & 0x0F;
    uint8_t single_size = data[1];
    uint8_t msg_count   = data[2];

    /* 必须是 PACKED 消息类型 */
    if (msg_type != ODID_MESSAGETYPE_PACKED) {
        return false;
    }

    /* 协议版本：0x0 = ASTM F3411, 0x1 = GB42590
     * 这里只处理 ASTM (proto_ver == 0)，GB42590 由专门模块处理 */
    if (proto_ver != 0x00) {
        return false;
    }

    if (single_size != ASTM_MSG_SIZE) {
        ESP_LOGD(TAG, "Unexpected SingleMessageSize=%d (expect %d)",
                 single_size, ASTM_MSG_SIZE);
        return false;
    }

    if (msg_count < 1 || msg_count > 10) {  /* ASTM/ASD-STAN 允许 1..10 条 */
        ESP_LOGD(TAG, "Invalid MsgPackSize=%d", msg_count);
        return false;
    }

    /* 校验长度足够容纳声明的消息数 */
    uint16_t needed = 3 + (uint16_t)msg_count * ASTM_MSG_SIZE;
    if (len < needed) {
        ESP_LOGD(TAG, "Data too short: %d < %d", len, needed);
        return false;
    }

    int ret = odid_message_process_pack(&uav->uas_data, (uint8_t *)data, len);
    if (ret > 0) {
        return true;
    }

    ESP_LOGD(TAG, "odid_message_process_pack failed ret=%d", ret);
    return false;
}
