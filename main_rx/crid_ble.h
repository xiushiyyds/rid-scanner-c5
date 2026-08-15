/**
 * crid_ble.h — BLE NUS 数据通道接口 (detector variant v2.0.0)
 *
 * 使用 NimBLE 实现 Nordic UART Service (NUS) 外设，
 * 将 JSON 数据流通过 BLE 通知发送到手机端。
 * 纯侦测板：无模拟发射控制命令，仅保留配对。
 */

#ifndef CRID_BLE_H
#define CRID_BLE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t crid_ble_init(void);
void crid_ble_write_cb(const char *data, size_t len, void *ctx);
bool crid_ble_is_connected(void);

/* ================================================================
 * BLE 配对确认
 * ================================================================ */

typedef void (*ble_pair_display_cb_t)(const char *pin_code);
void crid_ble_register_pair_display(ble_pair_display_cb_t cb);
bool crid_ble_is_paired(void);
void crid_ble_reset_pair(void);

/**
 * 延迟重启 BLE RID 扫描
 * BLE 连接/断开后 controller 可能停止扫描，调用此函数安全重启。
 */
void crid_ble_delayed_scan_restart(uint32_t delay_ms);

#ifdef __cplusplus
}
#endif

#endif // CRID_BLE_H
