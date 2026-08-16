/**
 * crid_brand.c — SN 前缀 → 品牌/型号映射
 *
 * 注意：LCD 字库仅含 ASCII，型号字段必须用英文/数字，不能含中文。
 * 中文类型名仅用于 USB JSON 输出和串口日志，不直接送 LCD。
 */
#include <crid_brand.h>
#include <string.h>
#include <stdio.h>

typedef struct {
    const char *prefix;
    const char *brand;
    const char *model;
} sn_model_t;

/* 高频机型表，按前缀长度从长到短排列，优先精确匹配 */
static const sn_model_t s_sn_table[] = {
    /* DJI 1581F 系列（来自 DroneScanner scanner.ino DJI_Model 表）*/
    {"1581F8LQ", "DJI", "Mavic 4 Pro"},
    {"1581F67Q", "DJI", "Mavic 3 Pro"},
    {"1581F5Y8", "DJI", "Mavic 3 Classic"},
    {"1581F45Q", "DJI", "Mavic 3"},
    {"1581F45T", "DJI", "Mavic 3"},
    {"1581F895", "DJI", "Air 3S"},
    {"1581F6N8", "DJI", "Air 3"},
    {"1581F385", "DJI", "Air 2S"},
    {"1581FANL", "DJI", "Mini 5 Pro"},
    {"1581F9DE", "DJI", "Mini 5 Pro"},
    {"1581F5QJ", "DJI", "Mini 4 Pro"},
    {"1581F6Z9", "DJI", "Mini 4 Pro"},
    {"1581F8C8", "DJI", "Mini 4K"},
    {"1581F4XF", "DJI", "Mini 3 Pro"},
    {"1581F6CD", "DJI", "Mini 2 SE"},
    {"1581F5YH", "DJI", "Mini 3"},
    {"1581FBV5", "DJI", "Lito 1"},
    {"1581FB34", "DJI", "Lito X1"},
    {"1581FA8J", "DJI", "Avata 360"},
    {"1581F6W8", "DJI", "Avata 2"},
    {"1581F4CQ", "DJI", "Avata"},
    {"1581F4QW", "DJI", "Avata"},
    {"1581FA6Q", "DJI", "Neo 2"},
    {"1581F8A1", "DJI", "Neo"},
    {"1581F87L", "DJI", "Neo"},
    {"1581F3CQ", "DJI", "DJI FPV"},
    {"1581F7V2", "DJI", "Flip"},
    {"1581F6H8", "DJI", "Matrice 350 RTK"},
    {"1581F5BK", "DJI", "Matrice 30"},
    {"1581F5BM", "DJI", "Matrice 30T"},
    {"1581F52Q", "DJI", "Mavic 3E/3T"},
    {"1581F5FH", "DJI", "Mavic 3E/3T"},
    {"1581F5FJ", "DJI", "Mavic 3M"},
    {"1581F578", "DJI", "Inspire 3"},
    {"1581F4Z4", "DJI", "Inspire 3"},
    {"1581F6GK", "DJI", "Matrice 300"},
    {"1581F6Q8", "DJI", "Matrice 3D"},
    {"1581F6QA", "DJI", "Matrice 3TD"},
    {"1581F9HE", "DJI", "Matrice 4T/4E"},
    {"1581F7K3", "DJI", "Matrice 4T/4E"},
    {"1581F8HH", "DJI", "Matrice 4D/TD"},
    {"1581F8HG", "DJI", "Matrice 4D/TD"},
    {"1581F8DB", "DJI", "Matrice 400"},
    {"1581F986", "DJI", "Mavic 4 Ser."},
    {"1581FAN4", "DJI", "FlyCart 100"},
    /* 其他品牌 */
    {"1748FEV3", "Autel", "EVO MAX"},
    /* 国标 RID 设备（GB42590/GB46750），已知肩灯前缀 */
    {"2051FE",   "GB-RID", "RID Beacon"},
};

int crid_brand_lookup_sn(const char *sn,
                         char *brand, size_t bsize,
                         char *model, size_t msize)
{
    if (!sn || !sn[0]) return 0;
    if (brand && bsize) brand[0] = '\0';
    if (model && msize) model[0] = '\0';

    for (size_t i = 0; i < sizeof(s_sn_table)/sizeof(s_sn_table[0]); i++) {
        const char *p = s_sn_table[i].prefix;
        size_t plen = strlen(p);
        /* 大小写不敏感匹配：SN 可能是大小写混合 */
        if (strncasecmp(sn, p, plen) == 0) {
            if (brand && bsize) snprintf(brand, bsize, "%s", s_sn_table[i].brand);
            if (model && msize) snprintf(model, msize, "%s", s_sn_table[i].model);
            return 1;
        }
    }

    /* DJI 通用前缀兜底（历史格式）*/
    if (!strncasecmp(sn, "4TAD", 4) || !strncasecmp(sn, "8UU", 3)) {
        if (brand && bsize) snprintf(brand, bsize, "DJI");
        if (model && msize) snprintf(model, msize, "DJI UAV");
        return 1;
    }
    if (!strncasecmp(sn, "89XH", 4)) {
        if (brand && bsize) snprintf(brand, bsize, "Autel");
        if (model && msize) snprintf(model, msize, "Autel UAV");
        return 1;
    }
    if (!strncasecmp(sn, "6AXF", 4)) {
        if (brand && bsize) snprintf(brand, bsize, "FIMI");
        if (model && msize) snprintf(model, msize, "FIMI UAV");
        return 1;
    }

    /* 纯 16+ 位十六进制 → 国标 RID 合规设备 */
    {
        size_t slen = strlen(sn);
        int hex_only = 1;
        if (slen >= 16) {
            for (size_t k = 0; k < slen; k++) {
                char c = sn[k];
                if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F'))) {
                    hex_only = 0; break;
                }
            }
            if (hex_only) {
                if (brand && bsize) snprintf(brand, bsize, "GB-RID");
                if (model && msize) snprintf(model, msize, "Compliant UAV");
                return 1;
            }
        }
    }

    return 0;
}

/* ODID_UATYPE 枚举顺序（opendroneid.h）：
 * 0=None, 1=Aeroplane, 2=Helicopter_or_Multirotor, 3=Gyroplane,
 * 4=Hybrid_Lift, 5=Ornithopter, 6=Glider, 7=Kite,
 * 8=Free_Balloon, 9=Captive_Balloon, 10=Airship,
 * 11=Free_Fall_Parachute, 12=Rocket, 13=Tethered_Powered_Aircraft,
 * 14=Ground_Obstacle, 15=Other */
static const char *const s_ua_type_cn[] = {
    "未声明", "固定翼", "多旋翼/直升机", "自转旋翼机",
    "复合翼", "扑翼机", "滑翔机", "风筝",
    "自由气球", "系留气球", "飞艇",
    "自由落体降落伞", "火箭", "系留动力飞行器",
    "地面障碍物", "其他"
};

const char *crid_brand_ua_type_cn(uint8_t ua_type) {
    if (ua_type >= sizeof(s_ua_type_cn)/sizeof(s_ua_type_cn[0]))
        return "未知";
    return s_ua_type_cn[ua_type];
}
