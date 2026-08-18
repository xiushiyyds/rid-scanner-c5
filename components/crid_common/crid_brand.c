/**
 * crid_brand.c — SN 前缀 → 品牌/型号映射
 *
 * 数据源：components/crid_common/include/drone_sn_db.h（117 条，侦测/模拟器共用）。
 * 该表同时被模拟器用于生成逼真 SN，两边永远同步。
 *
 * 注意：LCD 字库仅含 ASCII，型号字段必须用英文/数字，不能含中文。
 * 中文类型名仅用于 USB JSON 输出和串口日志，不直接送 LCD。
 */
#include <crid_brand.h>
#include <drone_sn_db.h>
#include <string.h>
#include <stdio.h>

int crid_brand_lookup_sn(const char *sn,
                         char *brand, size_t bsize,
                         char *model, size_t msize)
{
    if (!sn || !sn[0]) return 0;
    if (brand && bsize) brand[0] = '\0';
    if (model && msize) model[0] = '\0';

    /* 公共表：117 条，按精确前缀匹配（大小写不敏感） */
    for (size_t i = 0; i < DRONE_SN_DB_COUNT; i++) {
        const char *p = drone_sn_db[i].prefix;
        size_t plen = strlen(p);
        if (strncasecmp(sn, p, plen) == 0) {
            if (brand && bsize) snprintf(brand, bsize, "%s", drone_sn_db[i].brand);
            if (model && msize) snprintf(model, msize, "%s", drone_sn_db[i].model);
            return 1;
        }
    }

    /* 历史格式 / 通用前缀兜底 */
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

    /* 国标 RID 已知肩灯前缀 */
    if (!strncasecmp(sn, "2051FE", 6)) {
        if (brand && bsize) snprintf(brand, bsize, "GB-RID");
        if (model && msize) snprintf(model, msize, "RID Beacon");
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
