/**
 * crid_ble.h — BLE NUS 数据通道接口
 *
 * 使用 NimBLE 实现 Nordic UART Service (NUS) 外设，
 * 将 JSON 数据流通过 BLE 通知发送到 Android app。
 *
 * v1.4：模拟器控制回调更新为多目标模式。
 * v1.5：新增 BLE 配对确认机制。
 *
 * 内存优化：NimBLE 内存池分配在 SPIRAM 中。
 */

#ifndef CRID_BLE_H
#define CRID_BLE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 初始化 BLE NUS 外设
 */
esp_err_t crid_ble_init(void);

/**
 * BLE JSON 数据写入回调
 */
void crid_ble_write_cb(const char *data, size_t len, void *ctx);

/**
 * 检查 BLE 是否已连接
 */
bool crid_ble_is_connected(void);

/* ================================================================
 * v1.4：BLE 模拟器控制命令（多目标）
 * ================================================================ */

typedef void (*sim_ble_start_cb_t)(int target_count);
typedef void (*sim_ble_stop_cb_t)(void);
typedef void (*sim_ble_config_cb_t)(double lat, double lon, int mode, int channel,
                                     int count, int tx_power);
typedef void (*sim_ble_status_cb_t)(char *buf, size_t buf_size);

void crid_ble_register_sim_callbacks(sim_ble_start_cb_t start_cb,
                                      sim_ble_stop_cb_t stop_cb,
                                      sim_ble_config_cb_t config_cb,
                                      sim_ble_status_cb_t status_cb);

/* ================================================================
 * v1.5：BLE 配对确认
 * ================================================================ */

/**
 * 配对码显示回调：当 BLE 客户端连接时触发，
 * 由主程序在 LCD 屏幕上显示 4 位配对码
 * @param pin_code 4 位配对码字符串 (以 \0 结尾)
 */
typedef void (*ble_pair_display_cb_t)(const char *pin_code);

/**
 * 注册配对码显示回调
 */
void crid_ble_register_pair_display(ble_pair_display_cb_t cb);

/**
 * 检查是否已完成配对
 */
bool crid_ble_is_paired(void);

/**
 * 重置配对状态（断开连接时调用）
 */
void crid_ble_reset_pair(void);

#ifdef __cplusplus
}
#endif

#endif // CRID_BLE_H
