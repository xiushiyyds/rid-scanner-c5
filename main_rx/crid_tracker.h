/**
 * crid_tracker.h — 无人机追踪表接口（线程安全）
 */

#ifndef CRID_TRACKER_H
#define CRID_TRACKER_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "crid_rx_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 初始化追踪表
 */
void crid_tracker_init(void);

/**
 * 获取追踪互斥锁句柄
 */
SemaphoreHandle_t crid_tracker_get_mutex(void);

/**
 * 通过 MAC 地址查找或创建追踪条目
 * @param mac  6 字节 MAC 地址
 * @return 追踪条目指针，表满返回 NULL
 */
uav_track_t *crid_tracker_find_or_create(const uint8_t *mac);

/**
 * 获取活跃无人机数量
 */
int crid_tracker_get_active_count(void);

/**
 * 获取追踪表数组（用于遍历）
 */
uav_track_t *crid_tracker_get_table(void);

/**
 * 清理超时条目
 * @param timeout_ms  超时阈值（毫秒）
 */
void crid_tracker_cleanup(uint32_t timeout_ms);

/**
 * 更新某目标的 RSSI 采样并计算趋势
 * @param uav   追踪条目
 * @param rssi  本次 RSSI 值
 */
void crid_tracker_update_rssi(uav_track_t *uav, int8_t rssi);

#ifdef __cplusplus
}
#endif

#endif // CRID_TRACKER_H

/**
 * 通过 Basic ID (UAS ID / SN) 查找已存在的追踪条目
 * 用于跨传输方式（WiFi/BLE）去重：同一架无人机的 SN 相同，
 * 即使 MAC 不同也应合并到同一个条目。
 *
 * @param uas_id  UAS ID 字符串（序列号）
 * @return 已有条目指针，未找到返回 NULL
 */
uav_track_t *crid_tracker_find_by_uas_id(const char *uas_id);
