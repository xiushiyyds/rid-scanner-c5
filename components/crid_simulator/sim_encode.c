/**
 * sim_encode.c — GB42590 RID 信标帧编码器实现
 *
 * 从原始 esp32-crid-sim-OTA 项目的 encode_gb42590.c 移植。
 * 仅保留 3 条消息版本：BasicID + Location + System。
 * OUI=FA:0B:BC, VendorType=0x0D。
 */

#include <string.h>
#include <math.h>
#include <time.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sys/time.h"

#include "sim_encode.h"

static const char *TAG = "SIM_ENCODE";

/* ================================================================
 * 辅助函数
 * ================================================================ */

/* 写入 int32_t 为小端序（安全处理负数） */
static inline void write_le32(uint8_t *buf, int32_t val) {
    uint32_t uval = (uint32_t)val;
    buf[0] = uval & 0xFF;
    buf[1] = (uval >> 8) & 0xFF;
    buf[2] = (uval >> 16) & 0xFF;
    buf[3] = (uval >> 24) & 0xFF;
}

/* 写入 uint32_t 为小端序 */
static inline void write_le32_u32(uint8_t *buf, uint32_t val) {
    buf[0] = val & 0xFF;
    buf[1] = (val >> 8) & 0xFF;
    buf[2] = (val >> 16) & 0xFF;
    buf[3] = (val >> 24) & 0xFF;
}

/* 写入 uint16_t 为小端序 */
static inline void write_le16(uint8_t *buf, uint16_t val) {
    buf[0] = val & 0xFF;
    buf[1] = (val >> 8) & 0xFF;
}

/* 编码高度 (0.5m 精度，偏移 -1000m) */
static uint16_t encode_altitude(float altitude_m) {
    float val = (altitude_m + 1000.0f) * 2.0f;
    if (val < 0.0f) val = 0.0f;
    if (val > 65535.0f) val = 65535.0f;
    return (uint16_t)(val + 0.5f);
}

/* 编码地速 (符合 ASTM F3411-22a) */
static uint8_t encode_ground_speed(float speed_ms) {
    if (speed_ms < 0.0f) speed_ms = 0.0f;
    if (speed_ms < 63.75f) {
        return (uint8_t)(speed_ms * 4.0f + 0.5f);
    } else if (speed_ms <= 254.25f) {
        return (uint8_t)(255.0f + (speed_ms - 63.75f) / 0.75f + 0.5f);
    } else {
        return 254;
    }
}

/* ================================================================
 * Basic ID 报文 (25 字节)
 * ================================================================ */
void sim_encode_basic_id(const sim_encode_config_t *cfg, uint8_t *message) {
    memset(message, 0, SIM_MESSAGE_SIZE);
    message[0] = (SIM_MSG_TYPE_BASIC_ID << 4) | 0x01;
    message[1] = (cfg->ua_type & 0x0F) | ((cfg->id_type & 0x0F) << 4);

    memset(&message[2], 0x00, SIM_UAS_ID_MAX_LEN);
    size_t id_len = strlen(cfg->uas_id);
    if (id_len > SIM_UAS_ID_MAX_LEN) id_len = SIM_UAS_ID_MAX_LEN;
    memcpy(&message[2], cfg->uas_id, id_len);

    ESP_LOGD(TAG, "BasicID built (UAS: %s)", cfg->uas_id);
}

/* ================================================================
 * Location 报文 (25 字节)
 * ================================================================ */
void sim_encode_location(const sim_encode_config_t *cfg, uint8_t *message) {
    memset(message, 0, SIM_MESSAGE_SIZE);
    message[0] = (SIM_MSG_TYPE_LOCATION << 4) | 0x01;

    uint8_t speed_mult = (cfg->speed_horizontal >= 63.75f) ? 1 : 0;
    uint8_t height_type = cfg->height_type & 0x01;
    message[1] = (cfg->status << 4) | (height_type << 2) | speed_mult;

    uint8_t track_angle;
    if (cfg->heading < 0 || cfg->heading >= 360.0f) {
        track_angle = 255;
    } else {
        track_angle = (uint8_t)(cfg->heading + 0.5f);
        if (track_angle > 254) track_angle = 254;
    }
    message[2] = track_angle;
    message[3] = encode_ground_speed(cfg->speed_horizontal);
    /* 垂直速度：ASTM 编码 = speed / 0.5 (int8_t)，decodeSpeedVertical 返回 enc * 0.5。
     * 旧代码 *2 导致解码结果是实际值的 4 倍。 */
    int8_t vs_enc = (int8_t)(cfg->speed_vertical / 0.5f);
    message[4] = (uint8_t)vs_enc;

    write_le32_u32(&message[5], (uint32_t)(int32_t)round(cfg->latitude * 1e7));
    write_le32_u32(&message[9], (uint32_t)(int32_t)round(cfg->longitude * 1e7));

    write_le16(&message[13], encode_altitude(cfg->altitude_msl));
    write_le16(&message[15], encode_altitude(cfg->altitude_msl));
    write_le16(&message[17], encode_altitude(cfg->altitude_agl));

    /* 精度字段 — 模拟 DJI 消费级无人机典型精度 */
    /* HorizAcc=11(3m) | VertAcc=11(3m) */
    message[19] = (11 << 4) | 11;
    /* BaroAcc=7(1m) | SpeedAcc=4(0.3m/s) */
    message[20] = (7 << 4) | 4;

    struct timeval tv_loc;
    gettimeofday(&tv_loc, NULL);
    struct tm *tm_utc = gmtime(&tv_loc.tv_sec);
    uint16_t ts = (uint16_t)(tm_utc->tm_min * 600 + tm_utc->tm_sec * 10 + tv_loc.tv_usec / 100000);
    write_le16(&message[21], ts);

    /* TSAccuracy=2(~0.1s) | Reserved=0 */
    message[23] = (2 << 4) | 0;

    ESP_LOGD(TAG, "Location built (%.6f, %.6f)", cfg->latitude, cfg->longitude);
}

/* ================================================================
 * System 报文 (25 字节)
 * ================================================================ */
void sim_encode_system(const sim_encode_config_t *cfg, uint8_t *message) {
    memset(message, 0, SIM_MESSAGE_SIZE);
    message[0] = (SIM_MSG_TYPE_SYSTEM << 4) | 0x01;

    message[1] = (cfg->classification_type << 2) | (cfg->operator_location_type & 0x03);

    write_le32(&message[2], (int32_t)round(cfg->operator_lat * 1e7));
    write_le32(&message[6], (int32_t)round(cfg->operator_lon * 1e7));

    write_le16(&message[10], 1);  /* AreaCount = 1 */
    /* AreaRadius: ASTM 编码 = radius_m / 10 (单位 10m)。
     * 100m 半径应编码为 10 (0x0A)，之前写 0x64(100) 会被解码为 1000m。 */
    message[12] = 10;

    /* AreaCeiling(2B) + AreaFloor(2B)：ASTM 高度编码 (alt+1000)*2 */
    write_le16(&message[13], encode_altitude(100.0f));
    write_le16(&message[15], encode_altitude(50.0f));

    /* Byte 17: [ClassEU:4][CategoryEU:4] */
    message[17] = (cfg->class_eu << 4) | (cfg->category_eu & 0x0F);

    /* OperatorAltitudeGeo(2B, byte 18-19)：ASTM 高度编码 */
    write_le16(&message[18], encode_altitude(cfg->operator_alt));

    /* Timestamp(4B, byte 20-23)：自 2019-01-01 00:00:00 UTC 的秒数 */
    struct timeval tv_sys;
    gettimeofday(&tv_sys, NULL);
    uint32_t ts_since_2019 = (uint32_t)(tv_sys.tv_sec - 1546300800);
    write_le32_u32(&message[20], ts_since_2019);

    /* Byte 24: Reserved */
    message[24] = 0;

    ESP_LOGD(TAG, "System built");
}

/* ================================================================
 * 完整 Beacon 帧构建（含 3 条打包消息）
 * ================================================================ */
bool sim_encode_beacon_frame(const sim_encode_config_t *cfg,
                              uint8_t message_counter,
                              uint8_t *frame, uint16_t max_len,
                              uint16_t *out_len) {
    if (cfg == NULL || frame == NULL || out_len == NULL) return false;

    uint16_t pos = 0;
    #define REQUIRE_SPACE(need) \
    do { \
        if ((uint32_t)pos + (uint32_t)(need) > (uint32_t)max_len) { \
            ESP_LOGE(TAG, "Frame buffer too small: pos=%u need=%u max=%u", \
                     pos, (unsigned)(need), max_len); \
            return false; \
        } \
    } while (0)

    /* 802.11 Beacon 帧头 (24 bytes) */
    REQUIRE_SPACE(24);
    frame[pos++] = 0x80;
    frame[pos++] = 0x00;
    frame[pos++] = 0x00;
    frame[pos++] = 0x00;
    memset(&frame[pos], 0xFF, 6); pos += 6;
    memcpy(&frame[pos], cfg->mac_address, 6); pos += 6;
    memcpy(&frame[pos], cfg->mac_address, 6); pos += 6;
    frame[pos++] = 0x00;
    frame[pos++] = 0x00;

    /* 固定参数 (8 bytes) */
    REQUIRE_SPACE(8);
    memset(&frame[pos], 0, 8); pos += 8;

    /* Beacon Interval */
    REQUIRE_SPACE(2);
    write_le16(&frame[pos], 100); pos += 2;

    /* Capability Info */
    REQUIRE_SPACE(2);
    frame[pos++] = 0x21;
    frame[pos++] = 0x04;

    /* SSID IE */
    size_t ssid_len = strlen(cfg->ssid);
    if (ssid_len > 32) ssid_len = 32;
    REQUIRE_SPACE(2 + ssid_len);
    frame[pos++] = 0x00;
    frame[pos++] = (uint8_t)ssid_len;
    memcpy(&frame[pos], cfg->ssid, ssid_len); pos += ssid_len;

    /* Supported Rates IE */
    REQUIRE_SPACE(2 + 8);
    frame[pos++] = 0x01;
    frame[pos++] = 0x08;
    uint8_t rates[] = {0x82, 0x84, 0x8b, 0x96, 0x24, 0x30, 0x48, 0x6c};
    memcpy(&frame[pos], rates, 8); pos += 8;

    /* DS Set (Channel) IE */
    REQUIRE_SPACE(3);
    frame[pos++] = 0x03;
    frame[pos++] = 0x01;
    frame[pos++] = cfg->channel;

    /* Vendor Specific IE (RID Packed Message) */
    #define PACKED_MSG_COUNT 3
    /* packed payload = header(3: F1+size+count) + messages(75) = 78 bytes */
    #define PACKED_PAYLOAD_LEN (3 + PACKED_MSG_COUNT * SIM_MESSAGE_SIZE)
    /* IE content = OUI(3) + VendorType(1) + Counter(1) + packed_payload(78) = 83 bytes */
    #define IE_CONTENT_LEN (3 + 1 + 1 + PACKED_PAYLOAD_LEN)

    REQUIRE_SPACE(2 + IE_CONTENT_LEN);
    frame[pos++] = 0xDD;
    frame[pos++] = IE_CONTENT_LEN;  /* IE length = 83 (OUI+Type+Counter+packed) */
    frame[pos++] = SIM_OUI_0;
    frame[pos++] = SIM_OUI_1;
    frame[pos++] = SIM_OUI_2;
    frame[pos++] = SIM_VENDOR_TYPE;
    frame[pos++] = message_counter;

    /* 打包消息体 */
    REQUIRE_SPACE(PACKED_PAYLOAD_LEN);
    uint8_t packed[PACKED_PAYLOAD_LEN];
    uint8_t pp = 0;

    packed[pp++] = 0xF1;
    packed[pp++] = SIM_MESSAGE_SIZE;
    packed[pp++] = PACKED_MSG_COUNT;

    uint8_t basic_msg[SIM_MESSAGE_SIZE];
    sim_encode_basic_id(cfg, basic_msg);
    memcpy(&packed[pp], basic_msg, SIM_MESSAGE_SIZE);
    pp += SIM_MESSAGE_SIZE;

    uint8_t loc_msg[SIM_MESSAGE_SIZE];
    sim_encode_location(cfg, loc_msg);
    memcpy(&packed[pp], loc_msg, SIM_MESSAGE_SIZE);
    pp += SIM_MESSAGE_SIZE;

    uint8_t sys_msg[SIM_MESSAGE_SIZE];
    sim_encode_system(cfg, sys_msg);
    memcpy(&packed[pp], sys_msg, SIM_MESSAGE_SIZE);
    pp += SIM_MESSAGE_SIZE;

    (void)pp;

    memcpy(&frame[pos], packed, PACKED_PAYLOAD_LEN);
    pos += PACKED_PAYLOAD_LEN;

    *out_len = pos;

    static uint32_t s_frame_count = 0;
    s_frame_count++;
    if ((s_frame_count % 100U) == 1U) {
        ESP_LOGI(TAG, "=== Beacon Frame: %u bytes, Counter=%u ===", *out_len, message_counter);
    }

    return true;
#undef REQUIRE_SPACE
}
