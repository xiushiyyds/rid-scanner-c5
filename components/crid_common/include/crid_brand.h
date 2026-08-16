/**
 * crid_brand.h — SN/UA Type → 品牌型号映射
 *
 * 用于 LCD 列表/详情页显示可读机型名。
 * 与网页端 SN_BRAND_MAP 保持同源，固件端仅保留高频条目，节省 flash。
 */
#ifndef CRID_BRAND_H
#define CRID_BRAND_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 通过 SN 前缀匹配品牌+型号。
 *
 * @param sn     UAS ID / SN 字符串（以 '\0' 结尾）
 * @param brand  输出品牌，如 "DJI" / "Autel" / "GB-RID"
 * @param model  输出型号，如 "Mavic 3" / "EVO MAX"
 * @param bsize  brand 缓冲大小
 * @param msize  model 缓冲大小
 * @return 1=命中精确前缀，0=未命中（输出保持空串）
 */
int crid_brand_lookup_sn(const char *sn,
                         char *brand, size_t bsize,
                         char *model, size_t msize);

/**
 * ASTM/GB 标准的 UA Type → 中文类型名。
 * @param ua_type ODID_UATYPE_* 数值（0~15）
 * @return 静态字符串，如 "多旋翼" / "固定翼"
 */
const char *crid_brand_ua_type_cn(uint8_t ua_type);

#ifdef __cplusplus
}
#endif

#endif /* CRID_BRAND_H */
