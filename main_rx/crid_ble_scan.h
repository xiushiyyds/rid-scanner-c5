/**
 * crid_ble_scan.h — BLE Remote ID 扫描模块
 *
 * 扫描蓝牙 Legacy Advertising 和 BT5 Extended Advertising (Coded PHY / Long Range)，
 * 匹配 ASTM F3411 标准 Service UUID 0xFFFA，提取 ODID payload 投递到 sniffer 队列，
 * 复用现有的标准 RID 解析流程。
 *
 * v1.8: 新增 BLE RID 接收能力，与 WiFi sniffer 并行工作。
 */

#ifndef CRID_BLE_SCAN_H
#define CRID_BLE_SCAN_H

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 初始化 BLE RID 扫描器
 * 必须在 NimBLE 控制器初始化之后调用。
 * @return ESP_OK 成功
 */
esp_err_t crid_ble_scan_init(void);

/**
 * 启动 BLE 扫描
 * @return ESP_OK 成功
 */
esp_err_t crid_ble_scan_start(void);

/**
 * 停止 BLE 扫描
 */
void crid_ble_scan_stop(void);

/**
 * 检查 BLE 扫描是否正在运行
 */
bool crid_ble_scan_is_running(void);

/**
 * lcdfix28: 设置扫描允许开关。
 * 进入模拟发射页时传 false（停扫并阻止 BLE 事件重启扫描），
 * 回到侦测页时传 true 恢复。
 */
void crid_ble_scan_set_allowed(bool allowed);

#ifdef __cplusplus
}
#endif

#endif // CRID_BLE_SCAN_H
