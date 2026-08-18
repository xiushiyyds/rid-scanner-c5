/* drone_sn_db.h — 无人机 SN 前缀数据库（侦测/模拟器共用）
 *
 * 来源：厂商 SN 前缀映射表（2026-08 整理）。
 * 侦测侧用 prefix→brand/model 反查机型；
 * 模拟器侧用 prefix+model 生成逼真 SN。
 * 修改此表后两边同时生效，避免不同步。
 *
 * 注意：model 字段会送 LCD（仅 ASCII 字库），不要含中文。
 */
#ifndef DRONE_SN_DB_H
#define DRONE_SN_DB_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DRONE_CAT_CONSUMER = 0,   /* 消费级 */
    DRONE_CAT_FPV,            /* 穿越机/FPV */
    DRONE_CAT_INDUSTRIAL,     /* 行业级 */
    DRONE_CAT_AGRICULTURE,    /* 植保级 */
    DRONE_CAT_LOGISTICS,      /* 物流级 */
    DRONE_CAT_MAPPING,        /* 测绘级 */
    DRONE_CAT_VIDEO,          /* 影视级 */
    DRONE_CAT_UNKNOWN
} drone_category_t;

typedef struct {
    const char *prefix;       /* SN 前缀 */
    const char *brand;        /* 品牌（ASCII） */
    const char *model;        /* 机型名称（ASCII，会送 LCD） */
    drone_category_t category;
} drone_sn_entry_t;

#define DRONE_SN_DB_COUNT 117

static const drone_sn_entry_t drone_sn_db[] = {
    {"11223344", "Parrot",      "Anafi",                DRONE_CAT_CONSUMER},
    {"1567A000", "microdrones", "md4-1000",             DRONE_CAT_MAPPING},
    {"1567A001", "microdrones", "md4-3000",             DRONE_CAT_MAPPING},
    {"1581F0M6", "DJI",         "Mavic 2 Zoom",         DRONE_CAT_CONSUMER},
    {"1581F0UY", "DJI",         "Phantom 4 RTK",        DRONE_CAT_INDUSTRIAL},
    {"1581F11V", "DJI",         "Phantom 4 Pro V2.0",   DRONE_CAT_CONSUMER},
    {"1581F163", "DJI",         "Mavic 2 Pro",          DRONE_CAT_CONSUMER},
    {"1581F1WN", "DJI",         "Mavic Air 2",          DRONE_CAT_CONSUMER},
    {"1581F1ZN", "DJI",         "M300 RTK",             DRONE_CAT_INDUSTRIAL},
    {"1581F37Q", "DJI",         "FPV",                  DRONE_CAT_FPV},
    {"1581F385", "DJI",         "Air 2S",               DRONE_CAT_CONSUMER},
    {"1581F3CQ", "DJI",         "DJI FPV",              DRONE_CAT_FPV},
    {"1581F3N3", "DJI",         "Mavic Air 2",          DRONE_CAT_CONSUMER},
    {"1581F3YT", "DJI",         "Air 2S",               DRONE_CAT_CONSUMER},
    {"1581F446", "DJI",         "Agras T10/T30",        DRONE_CAT_AGRICULTURE},
    {"1581F45Q", "DJI",         "Mavic 3",              DRONE_CAT_CONSUMER},
    {"1581F45T", "DJI",         "Mavic 3",              DRONE_CAT_CONSUMER},
    {"1581F4BN", "DJI",         "M30",                  DRONE_CAT_INDUSTRIAL},
    {"1581F4CQ", "DJI",         "Avata",                DRONE_CAT_FPV},
    {"1581F4GC", "DJI",         "Mavic 2 Ent. Adv.",    DRONE_CAT_INDUSTRIAL},
    {"1581F4QW", "DJI",         "Avata",                DRONE_CAT_FPV},
    {"1581F4XF", "DJI",         "Mini 3 Pro",           DRONE_CAT_CONSUMER},
    {"1581F4Z4", "DJI",         "Inspire 3",            DRONE_CAT_INDUSTRIAL},
    {"1581F52Q", "DJI",         "Mavic 3E/3T",          DRONE_CAT_INDUSTRIAL},
    {"1581F574", "DJI",         "Agras T20P/T40",       DRONE_CAT_AGRICULTURE},
    {"1581F578", "DJI",         "Inspire 3",            DRONE_CAT_INDUSTRIAL},
    {"1581F5BK", "DJI",         "Matrice 30",           DRONE_CAT_INDUSTRIAL},
    {"1581F5BL", "DJI",         "M30 Dock",             DRONE_CAT_INDUSTRIAL},
    {"1581F5BM", "DJI",         "M30T Dock",            DRONE_CAT_INDUSTRIAL},
    {"1581F5FH", "DJI",         "M3E",                  DRONE_CAT_INDUSTRIAL},
    {"1581F5FJ", "DJI",         "M3T",                  DRONE_CAT_INDUSTRIAL},
    {"1581F5FK", "DJI",         "M3M",                  DRONE_CAT_INDUSTRIAL},
    {"1581F5QJ", "DJI",         "Mini 4 Pro",           DRONE_CAT_CONSUMER},
    {"1581F5Y8", "DJI",         "Mavic 3 Classic",      DRONE_CAT_CONSUMER},
    {"1581F5YH", "DJI",         "Mini 3",               DRONE_CAT_CONSUMER},
    {"1581F62H", "DJI",         "M30T",                 DRONE_CAT_INDUSTRIAL},
    {"1581F67P", "DJI",         "Mavic 3 Classic",      DRONE_CAT_CONSUMER},
    {"1581F67Q", "DJI",         "Mavic 3 Pro",          DRONE_CAT_CONSUMER},
    {"1581F6BU", "DJI",         "Agras T50",            DRONE_CAT_AGRICULTURE},
    {"1581F6CD", "DJI",         "Mini 2 SE",            DRONE_CAT_CONSUMER},
    {"1581F6GK", "DJI",         "M350 RTK",             DRONE_CAT_INDUSTRIAL},
    {"1581F6H8", "DJI",         "Matrice 350 RTK",      DRONE_CAT_INDUSTRIAL},
    {"1581F6MK", "DJI",         "Mavic 3 Pro Cine",     DRONE_CAT_CONSUMER},
    {"1581F6N8", "DJI",         "Air 3",                DRONE_CAT_CONSUMER},
    {"1581F6Q8", "DJI",         "M3TD",                 DRONE_CAT_INDUSTRIAL},
    {"1581F6QA", "DJI",         "M3D",                  DRONE_CAT_INDUSTRIAL},
    {"1581F6W8", "DJI",         "Avata 2",              DRONE_CAT_FPV},
    {"1581F6Z9", "DJI",         "Mini 4 Pro",           DRONE_CAT_CONSUMER},
    {"1581F7C6", "DJI",         "FlyCart 30",           DRONE_CAT_LOGISTICS},
    {"1581F7FV", "DJI",         "Matrice 4E",           DRONE_CAT_INDUSTRIAL},
    {"1581F7K3", "DJI",         "Matrice 4T",           DRONE_CAT_INDUSTRIAL},
    {"1581F7V2", "DJI",         "DJI Flip",             DRONE_CAT_FPV},
    {"1581F836", "DJI",         "Agras T25P",           DRONE_CAT_AGRICULTURE},
    {"1581F87L", "DJI",         "Neo",                  DRONE_CAT_FPV},
    {"1581F895", "DJI",         "Air 3S",               DRONE_CAT_CONSUMER},
    {"1581F8A1", "DJI",         "Neo",                  DRONE_CAT_FPV},
    {"1581F8C8", "DJI",         "Mini 4K",              DRONE_CAT_CONSUMER},
    {"1581F8DB", "DJI",         "Matrice 400",          DRONE_CAT_INDUSTRIAL},
    {"1581F8HG", "DJI",         "Matrice 4TD",          DRONE_CAT_INDUSTRIAL},
    {"1581F8HH", "DJI",         "Matrice 4D",           DRONE_CAT_INDUSTRIAL},
    {"1581F8LQ", "DJI",         "Mavic 4 Pro",          DRONE_CAT_CONSUMER},
    {"1581F8PJ", "DJI",         "Mini 4K",              DRONE_CAT_CONSUMER},
    {"1581F8ZL", "DJI",         "Agras T100",           DRONE_CAT_AGRICULTURE},
    {"1581F8ZX", "DJI",         "Agras T70P",           DRONE_CAT_AGRICULTURE},
    {"1581F986", "DJI",         "Mavic 4 Pro 512GB",    DRONE_CAT_CONSUMER},
    {"1581F9DE", "DJI",         "Mini 5 Pro",           DRONE_CAT_CONSUMER},
    {"1581FA6Q", "DJI",         "Neo 2",                DRONE_CAT_FPV},
    {"1581FA8J", "DJI",         "Avata 360",            DRONE_CAT_CONSUMER},
    {"1581FAE3", "DJI",         "Agras T70S",           DRONE_CAT_AGRICULTURE},
    {"1581FAN4", "DJI",         "FlyCart 100",          DRONE_CAT_LOGISTICS},
    {"1581FANL", "DJI",         "Mini 5 Pro",           DRONE_CAT_CONSUMER},
    {"1581FB34", "DJI",         "Lito X1",              DRONE_CAT_CONSUMER},
    {"1581FBEX", "DJI",         "M3TA",                 DRONE_CAT_INDUSTRIAL},
    {"1581FBLK", "DJI",         "Avata 360 (DVN3NT)",   DRONE_CAT_CONSUMER},
    {"1581FBV5", "DJI",         "Lito 1",               DRONE_CAT_CONSUMER},
    {"1587A000", "AgEagle",     "eBee X",               DRONE_CAT_MAPPING},
    {"1587A001", "AgEagle",     "eBee TAC",             DRONE_CAT_MAPPING},
    {"1587A002", "AgEagle",     "eBee Geo",             DRONE_CAT_MAPPING},
    {"1587A003", "AgEagle",     "eBee RTK",             DRONE_CAT_MAPPING},
    {"1588E040", "Parrot",      "ANAMK3 (Anafi AI)",    DRONE_CAT_INDUSTRIAL},
    {"1688W000", "Wingtra",     "WingtraOne Gen II",    DRONE_CAT_MAPPING},
    {"1688W001", "Wingtra",     "WingtraOne GEN I",     DRONE_CAT_MAPPING},
    {"1701A000", "Percepto",    "Max",                  DRONE_CAT_INDUSTRIAL},
    {"1701A001", "Percepto",    "Sparrow",              DRONE_CAT_INDUSTRIAL},
    {"1701A002", "Percepto",    "Air",                  DRONE_CAT_INDUSTRIAL},
    {"1748CHL2", "Autel",       "EVO II",               DRONE_CAT_CONSUMER},
    {"1748CHL7", "Autel",       "EVO II",               DRONE_CAT_CONSUMER},
    {"1748CJD1", "Autel",       "Dragonfish Standard",  DRONE_CAT_INDUSTRIAL},
    {"1748CJD2", "Autel",       "Dragonfish Lite",      DRONE_CAT_INDUSTRIAL},
    {"1748CJD3", "Autel",       "Dragonfish Pro",       DRONE_CAT_INDUSTRIAL},
    {"1748CLT0", "Autel",       "EVO Lite",             DRONE_CAT_CONSUMER},
    {"1748CLTC", "Autel",       "EVO Lite+",            DRONE_CAT_CONSUMER},
    {"1748CLTG", "Autel",       "EVO Lite",             DRONE_CAT_CONSUMER},
    {"1748CNA0", "Autel",       "EVO Nano",             DRONE_CAT_CONSUMER},
    {"1748CNAG", "Autel",       "EVO Nano",             DRONE_CAT_CONSUMER},
    {"1748FEV2", "Autel",       "EVO II V3",            DRONE_CAT_CONSUMER},
    {"1748FEV3", "Autel",       "EVO Max/MDX-1",        DRONE_CAT_INDUSTRIAL},
    {"1748FEV4", "Autel",       "Alpha",                DRONE_CAT_INDUSTRIAL},
    {"1758X000", "XAG",         "P150 Max",             DRONE_CAT_AGRICULTURE},
    {"1758X001", "XAG",         "P100 Pro",             DRONE_CAT_AGRICULTURE},
    {"1758X002", "XAG",         "P100",                 DRONE_CAT_AGRICULTURE},
    {"1758X003", "XAG",         "V40",                  DRONE_CAT_AGRICULTURE},
    {"1778S000", "SwellPro",    "SplashDrone 4",        DRONE_CAT_CONSUMER},
    {"18179000", "Freefly",     "Alta X",               DRONE_CAT_VIDEO},
    {"18179001", "Freefly",     "Alta X Gen2",          DRONE_CAT_VIDEO},
    {"18179002", "Freefly",     "Alta 8",               DRONE_CAT_VIDEO},
    {"44332211", "Parrot",      "Anafi USA",            DRONE_CAT_CONSUMER},
    {"P208904",  "Skydio",      "Skydio 2/2+",          DRONE_CAT_CONSUMER},
    {"PF723000", "Parrot",      "Rolling Spider",       DRONE_CAT_CONSUMER},
    {"PF727078", "Parrot",      "Mambo FLY",            DRONE_CAT_CONSUMER},
    {"SDR21V1",  "Skydio",      "X2E",                  DRONE_CAT_INDUSTRIAL},
    {"SDR35V1",  "Skydio",      "X2",                   DRONE_CAT_INDUSTRIAL},
    {"SR47PCV",  "Skydio",      "X10",                  DRONE_CAT_INDUSTRIAL},
    {"YUNH5ETWET",   "Yuneec",  "H520 Hexacopter",      DRONE_CAT_INDUSTRIAL},
    {"YUNH920202EU", "Yuneec",  "Tornado H920",         DRONE_CAT_INDUSTRIAL},
    {"YUNQ4KPUS",    "Yuneec",  "Q500 4K Typhoon",      DRONE_CAT_CONSUMER},
    {"YUNTYH3EU",    "Yuneec",  "Typhoon H3",           DRONE_CAT_INDUSTRIAL},
};

#ifdef __cplusplus
}
#endif

#endif /* DRONE_SN_DB_H */
