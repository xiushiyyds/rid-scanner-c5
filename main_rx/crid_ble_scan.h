/**
 * crid_ble_scan.h — BLE Remote ID 扫描模块 (detector variant)
 *
 * 扫描蓝牙 Legacy Advertising 和 BT5 Extended Advertising (Coded PHY / Long Range)，
 * 匹配 ASTM F3411 标准 Service UUID 0xFFFA，提取 ODID payload 投递到 sniffer 队列。
 * 纯侦测板：BLE 连续扫描，无 TDD 分时。
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
 * HIGH：未连接时连续扫描（~100% duty），最大化 RID 捕获。
 * LOW：已连接时低占空比（~20% duty），给 GATT 通信留射频。
 */
void crid_ble_scan_set_duty_high(void);
void crid_ble_scan_set_duty_low(void);

#ifdef __cplusplus
}
#endif

#endif // CRID_BLE_SCAN_H
