/**
 * crid_ble_scan.h — BLE Remote ID 扫描模块 (detector v2.1.0)
 *
 * 扫描蓝牙 Legacy Advertising，匹配 ASTM F3411 Service UUID 0xFFFA，
 * 提取 ODID / GB46750 payload 投递到 sniffer 队列。
 *
 * v2.1.0: 删除 TDD 时分复用。BLE 扫描使用 window < itvl 占空比，
 * 控制器自动在间隙调度 GATT 连接事件。WiFi/BLE 共存由 PTA 硬件仲裁。
 *
 * 占空比：
 *   HIGH（未连接）：80%，window=80ms / itvl=100ms
 *   LOW （已连接）：40%，window=40ms / itvl=100ms
 */

#ifndef CRID_BLE_SCAN_H
#define CRID_BLE_SCAN_H

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t crid_ble_scan_init(void);
esp_err_t crid_ble_scan_start(void);
void crid_ble_scan_stop(void);
bool crid_ble_scan_is_running(void);

/**
 * 设置扫描允许开关。
 * 侦测板始终 true，但保留开关供未来使用。
 */
void crid_ble_scan_set_allowed(bool allowed);

/**
 * 切换扫描占空比模式。
 * HIGH：未连接，80% 占空比，高 RID 捕获。
 * LOW： 已连接，40% 占空比，GATT 连接事件稳定。
 *
 * 注意：这些函数只设置标志。如果扫描正在运行，调用者（必须在独立
 * task 中，不能在 NimBLE host task 回调里）需要 stop → delay → start
 * 让新参数生效。
 */
void crid_ble_scan_set_duty_high(void);
void crid_ble_scan_set_duty_low(void);

/**
 * WiFi 目标锁定时：10% 占空比，把空中时间让给 WiFi sniffer。
 * 与 set_duty_high/low 相同，需要调用者 stop → delay → start 生效。
 */
void crid_ble_scan_set_duty_wifi_locked(void);

#ifdef __cplusplus
}
#endif

#endif // CRID_BLE_SCAN_H
