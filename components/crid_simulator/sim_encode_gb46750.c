/**
 * sim_encode_gb46750.c — GB 46750-2025 新国标广播帧编码器实现
 *
 * 数据项位图 (与侦测侧 crid_parser_gb46750.c 严格对应):
 *   Flags byte0 (MSB→LSB):
 *     bit7=item07 UPC(20B)      bit6=item06 实名标志(8B)
 *     bit5=item05 运行类别(1B)  bit4=item04 UA分类(1B)
 *     bit3=item03 遥控站位置类型(1B)
 *     bit2=item02 遥控站经纬度(8B)
 *     bit1=item01 遥控站高度(2B)
 *     bit0=扩展标志
 *   Flags byte1:
 *     bit7=item15 无人机位置(8B) bit6=item14 航迹角(2B)
 *     bit5=item13 地速(2B)       bit4=item12 相对高度(2B)
 *     bit3=item11 垂直速度(1B)   bit2=item10 大地高度(2B)
 *     bit1=item09 气压高度(2B)
 *   Flags byte2:
 *     bit7=item23 运行状态(1B)   bit6=item22 坐标系(1B)
 *     bit5=item21 水平精度(1B)   bit4=item20 垂直精度(1B)
 *     bit3=item19 速度精度(1B)   bit2=item18 时间戳(6B)
 *     bit1=item17 时间戳精度(1B)
 *
 * 注意：item ID 是按 (byte_idx<<3)|bit 编码的，与国标"数据项编号001~021"
 * 不同。侦测侧已按此实现并验证，编码器必须保持一致。
 */

#include <string.h>
#include <math.h>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include "esp_log.h"
#include "sim_encode.h"
#include "sim_encode_gb46750.h"

static const char *TAG = "SIM_GB46750";

/* ================================================================
 * 小端写入辅助
 * ================================================================ */
static inline void w_le16(uint8_t *p, uint16_t v) {
    p[0] = v & 0xFF;
    p[1] = (v >> 8) & 0xFF;
}

static inline void w_le32(uint8_t *p, int32_t v) {
    uint32_t u = (uint32_t)v;
    p[0] = u & 0xFF;
    p[1] = (u >> 8) & 0xFF;
    p[2] = (u >> 16) & 0xFF;
    p[3] = (u >> 24) & 0xFF;
}

static inline void w_le48(uint8_t *p, uint64_t v) {
    for (int i = 0; i < 6; i++)
        p[i] = (v >> (i * 8)) & 0xFF;
}

/* ================================================================
 * 物理量编码（与 parser 解码公式严格对称）
 * ================================================================ */

/* 大地/气压/遥控站高度: raw = (actual + 1000) * 2, 分辨率 0.5m */
static uint16_t enc_alt_std(float alt_m) {
    float v = (alt_m + 1000.0f) * 2.0f;
    if (v < 0) v = 0;
    if (v > 65535.0f) v = 65535.0f;
    return (uint16_t)(v + 0.5f);
}

/* 相对高度: raw = (actual + 9000) * 2, 分辨率 0.5m */
static uint16_t enc_alt_rel(float alt_m) {
    float v = (alt_m + 9000.0f) * 2.0f;
    if (v < 0) v = 0;
    if (v > 65535.0f) v = 65535.0f;
    return (uint16_t)(v + 0.5f);
}

/* 经纬度: int32 LE × 1e7 */
static int32_t enc_latlon(double deg) {
    return (int32_t)llround(deg * 1e7);
}

/* 航迹角: uint16 LE × 0.1° */
static uint16_t enc_track(float deg) {
    if (deg < 0) deg = 0;
    if (deg >= 360.0f) deg = 359.9f;
    return (uint16_t)(deg * 10.0f + 0.5f);
}

/* 地速: uint16 LE × 0.1 m/s
 * 注意：parser 中 raw/10.0f 得到 m/s，与任务描述的 0.01 不同。
 * 以已验证的 parser 为准（真实肩灯抓包 raw/10）。
 * 任务描述中的 0.01 系数记录为待抓包验证项。 */
static uint16_t enc_speed(float mps) {
    if (mps < 0) mps = 0;
    if (mps > 300.0f) mps = 300.0f;
    return (uint16_t)(mps * 10.0f + 0.5f);
}

/* 垂直速度: 1B, bit7=方向(1=下降), bit6..0=幅值/2 (分辨率0.5m/s) */
static uint8_t enc_vspeed(float mps) {
    uint8_t sign = 0;
    float abs_v = mps;
    if (mps < 0) { sign = 0x80; abs_v = -mps; }
    if (abs_v > 63.5f) abs_v = 63.5f;
    return sign | (uint8_t)(abs_v * 2.0f + 0.5f);
}

/* ================================================================
 * 厂商 UPC 前缀（14位占位符，后续可按真实厂商码替换）
 * ================================================================ */
typedef struct {
    sim_brand_t brand;
    const char *prefix14;
} upc_prefix_t;

static const upc_prefix_t UPC_PREFIXES[] = {
    {SIM_BRAND_CRID,   "10000000000000"},  /* 国标通用占位 */
    {SIM_BRAND_DJI,    "10001581F00000"},  /* 大疆占位（含厂商码1581） */
    {SIM_BRAND_AUTEL,  "10004815A60000"},  /* 道通/Autel 占位 */
    {SIM_BRAND_PARROT, "1000903A720000"},  /* Parrot 占位 */
    {SIM_BRAND_SKYDIO, "1000D0155A0000"},  /* Skydio 占位 */
};
#define UPC_PREFIX_COUNT (sizeof(UPC_PREFIXES) / sizeof(UPC_PREFIXES[0]))

void sim_gb46750_gen_upc(sim_brand_t brand, uint32_t seq,
                          char *out, size_t out_len) {
    if (!out || out_len < GB46750_UPC_LEN + 1) return;

    const char *prefix = UPC_PREFIXES[0].prefix14;
    for (size_t i = 0; i < UPC_PREFIX_COUNT; i++) {
        if (UPC_PREFIXES[i].brand == brand) {
            prefix = UPC_PREFIXES[i].prefix14;
            break;
        }
    }

    /* 14 位前缀 + 6 位流水号（000000~999999） */
    snprintf(out, out_len, "%s%06lu", prefix, (unsigned long)(seq % 1000000));
}

/* ================================================================
 * GB46750 PDU 构建
 *
 * 返回 PDU 长度（含 6B header），0 表示失败。
 * ================================================================ */
static uint16_t build_gb46750_pdu(const sim_encode_config_t *cfg,
                                   const char *upc,
                                   uint8_t *pdu, uint16_t max_len) {
    /* 必填 15 项内容长度:
     * 01 UPC 20 + 02 实名 8 + 04 分类 1 + 05 RCS位置类型 1
     * + 06 RCS经纬度 8 + 07 RCS高度 2 + 08 飞机位置 8
     * + 09 航迹角 2 + 10 地速 2 + 13 大地高度 2
     * + 15 运行状态 1 + 16 坐标系 1 + 17 水平精度 1
     * + 18 垂直精度 1 + 20 时间戳 6
     * = 64 bytes
     * 可选 6 项: 03 运行类别 1 + 11 相对高度 2 + 12 垂直速度 1
     *           + 14 气压高度 2 + 19 速度精度 1 + 21 时间戳精度 1 = 8
     * 全发 = 72 bytes content */
    uint8_t content[80];
    uint16_t cp = 0;

    /* Flags: 全发 21 项 */
    uint8_t flags[3] = { 0xFE, 0xFF, 0xFE };
    /* byte0 bit0=0(无扩展), bit1~7=1 → 0xFE
     * byte1 bit0=0, bit1~7=1 → 0xFF
     * byte2 bit0=0, bit1~6=1, bit7=1 → 0xFE (item23=run status) */

    /* --- 01 UPC (20B ASCII) --- */
    memset(&content[cp], '1', GB46750_UPC_LEN);
    if (upc && upc[0]) {
        size_t ulen = strlen(upc);
        if (ulen > GB46750_UPC_LEN) ulen = GB46750_UPC_LEN;
        memcpy(&content[cp], upc, ulen);
    }
    cp += GB46750_UPC_LEN;

    /* --- 02 实名登记标志 (8B ASCII, item 0x06) ---
     * 警告：模拟器场景禁止使用真实身份证号，使用测试值 "11111111"。
     * 侦测侧 UI 显示遮罩在 v2.8 backlog 中处理。 */
    memset(&content[cp], '1', GB46750_REALNAME_LEN);  /* 测试值 0x31×8 */
    cp += GB46750_REALNAME_LEN;

    /* --- 03 运行类别 (1B, item 0x05, 可选)
     * 0=未知,1=开放,2=批准,3=通告等 */
    content[cp++] = 0;  /* 未知 */

    /* --- 04 UA 分类 (1B, item 0x04)
     * 国标分类: 1=微型,2=轻型,3=小型,4=中型,5=大型
     * 模拟器默认轻型=2 */
    content[cp++] = 2;

    /* --- 05 遥控站位置类型 (1B, item 0x03)
     * 0=未知, 1=T/O位置, 2=动态, 3=固定 */
    content[cp++] = 1;  /* 起飞位置 */

    /* --- 06 遥控站经纬度 (8B LE int32×1e7) --- */
    w_le32(&content[cp], enc_latlon(cfg->operator_lon));
    cp += 4;
    w_le32(&content[cp], enc_latlon(cfg->operator_lat));
    cp += 4;

    /* --- 07 遥控站高度 (2B, std encoding) --- */
    w_le16(&content[cp], enc_alt_std(cfg->operator_alt));
    cp += 2;

    /* --- 15 飞机位置 (8B LE int32×1e7) --- */
    w_le32(&content[cp], enc_latlon(cfg->longitude));
    cp += 4;
    w_le32(&content[cp], enc_latlon(cfg->latitude));
    cp += 4;

    /* --- 14 航迹角 (2B, uint16×0.1°) --- */
    w_le16(&content[cp], enc_track(cfg->heading));
    cp += 2;

    /* --- 13 地速 (2B, uint16×0.1 m/s) --- */
    w_le16(&content[cp], enc_speed(cfg->speed_horizontal));
    cp += 2;

    /* --- 12 相对高度 (2B, rel encoding) --- */
    w_le16(&content[cp], enc_alt_rel(cfg->altitude_agl));
    cp += 2;

    /* --- 11 垂直速度 (1B) --- */
    content[cp++] = enc_vspeed(cfg->speed_vertical);

    /* --- 10 大地高度 (2B, std encoding) --- */
    w_le16(&content[cp], enc_alt_std(cfg->altitude_msl));
    cp += 2;

    /* --- 09 气压高度 (2B, std encoding) — 可选 --- */
    w_le16(&content[cp], enc_alt_std(cfg->altitude_msl));
    cp += 2;

    /* --- 23 运行状态 (1B)
     * 0=未起飞,1=滑行,2=空中,4=起降,5=未知等
     * cfg->status: 0=ground,1=air,2=emergency... */
    content[cp++] = (cfg->status == 1) ? 2 : 0;

    /* --- 22 坐标系 (1B): 1=CGCS2000 --- */
    content[cp++] = 1;

    /* --- 21 水平精度 (1B) 枚举值 --- */
    content[cp++] = 11;  /* ~3m */

    /* --- 20 垂直精度 (1B) --- */
    content[cp++] = 11;

    /* --- 19 速度精度 (1B) --- */
    content[cp++] = 4;  /* ~0.3 m/s */

    /* --- 18 时间戳 (6B LE Unix ms) --- */
    struct timeval tv;
    gettimeofday(&tv, NULL);
    uint64_t ts_ms = (uint64_t)tv.tv_sec * 1000ULL + tv.tv_usec / 1000ULL;
    w_le48(&content[cp], ts_ms);
    cp += 6;

    /* --- 17 时间戳精度 (1B) --- */
    content[cp++] = 2;  /* ~0.1s */

    /* content 长度应为 72 */
    if (cp != 72) {
        ESP_LOGE(TAG, "PDU content length mismatch: %u (expected 72)", cp);
        return 0;
    }

    /* 组装 PDU */
    if (GB46750_HEADER_LEN + cp > max_len) return 0;

    uint16_t p = 0;
    pdu[p++] = GB46750_MAGIC;
    pdu[p++] = GB46750_VERSION_1_0;
    pdu[p++] = (uint8_t)cp;          /* data length */
    pdu[p++] = flags[0];
    pdu[p++] = flags[1];
    pdu[p++] = flags[2];
    memcpy(&pdu[p], content, cp);
    p += cp;

    return p;
}

/* ================================================================
 * 完整 Beacon 帧构建
 * ================================================================ */
bool sim_encode_gb46750_beacon_frame(const sim_encode_config_t *cfg,
                                      const char *upc,
                                      uint8_t message_counter,
                                      uint8_t *frame, uint16_t max_len,
                                      uint16_t *out_len) {
    if (!cfg || !frame || !out_len) return false;

    uint16_t pos = 0;
    #define NEED(n) do { \
        if ((uint32_t)pos + (uint32_t)(n) > (uint32_t)max_len) { \
            ESP_LOGE(TAG, "Frame buf small: pos=%u need=%u max=%u", pos, (unsigned)(n), max_len); \
            return false; \
        } \
    } while (0)

    /* ---- 802.11 Beacon 帧头 (24B) ---- */
    NEED(24);
    frame[pos++] = 0x80; frame[pos++] = 0x00;
    frame[pos++] = 0x00; frame[pos++] = 0x00;
    memset(&frame[pos], 0xFF, 6); pos += 6;   /* DA broadcast */
    memcpy(&frame[pos], cfg->mac_address, 6); pos += 6;  /* SA */
    memcpy(&frame[pos], cfg->mac_address, 6); pos += 6;  /* BSSID */
    frame[pos++] = 0x00; frame[pos++] = 0x00;  /* seq */

    /* ---- 固定参数 (8B) ---- */
    NEED(8);
    memset(&frame[pos], 0, 8); pos += 8;

    /* ---- Beacon Interval (2B) ---- */
    NEED(2);
    w_le16(&frame[pos], 100); pos += 2;

    /* ---- Capability Info (2B) ---- */
    NEED(2);
    frame[pos++] = 0x21;
    frame[pos++] = 0x04;

    /* ---- SSID IE ---- */
    size_t ssid_len = strlen(cfg->ssid);
    if (ssid_len > 32) ssid_len = 32;
    NEED(2 + ssid_len);
    frame[pos++] = 0x00;
    frame[pos++] = (uint8_t)ssid_len;
    memcpy(&frame[pos], cfg->ssid, ssid_len);
    pos += ssid_len;

    /* ---- Supported Rates IE ---- */
    NEED(2 + 8);
    frame[pos++] = 0x01;
    frame[pos++] = 0x08;
    {
        uint8_t rates[] = {0x82,0x84,0x8b,0x96,0x24,0x30,0x48,0x6c};
        memcpy(&frame[pos], rates, 8); pos += 8;
    }

    /* ---- DS Set (Channel) IE ---- */
    NEED(3);
    frame[pos++] = 0x03;
    frame[pos++] = 0x01;
    frame[pos++] = cfg->channel;

    /* ---- Vendor Specific IE (0xDD) ----
     * 内容: OUI(3) + VendorType(1) + Counter(1) + GB46750 PDU(78)
     * PDU = 6B header + 72B content = 78B
     * IE content len = 3+1+1+78 = 83 */
    uint8_t pdu_buf[90];
    uint16_t pdu_len = build_gb46750_pdu(cfg, upc, pdu_buf, sizeof(pdu_buf));
    if (pdu_len == 0) return false;

    uint8_t ie_content_len = 3 + 1 + 1 + pdu_len;
    NEED(2 + ie_content_len);
    frame[pos++] = 0xDD;
    frame[pos++] = ie_content_len;
    frame[pos++] = SIM_OUI_0;   /* FA */
    frame[pos++] = SIM_OUI_1;   /* 0B */
    frame[pos++] = SIM_OUI_2;   /* BC */
    frame[pos++] = SIM_VENDOR_TYPE; /* 0x0D */
    frame[pos++] = message_counter;
    memcpy(&frame[pos], pdu_buf, pdu_len);
    pos += pdu_len;

    *out_len = pos;

    static uint32_t s_cnt = 0;
    s_cnt++;
    if ((s_cnt % 100U) == 1U) {
        ESP_LOGI(TAG, "GB46750 Beacon: %u bytes, counter=%u", *out_len, message_counter);
    }
    return true;
#undef NEED
}
