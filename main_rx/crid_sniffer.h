/**
 * crid_sniffer.h — Wi-Fi Sniffer 模块接口
 */

#ifndef CRID_SNIFFER_H
#define CRID_SNIFFER_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "crid_rx_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 获取 sniffer 消息队列句柄
 */
QueueHandle_t crid_sniffer_get_queue(void);

/**
 * lcdfix31: 仅创建 sniffer 消息队列（不初始化 WiFi）。
 * 用于在 WiFi 初始化前先创建 parser/monitor 任务，避免 WiFi 占用
 * 大量内部 SRAM 后任务栈分配失败。
 */
esp_err_t crid_sniffer_queue_create(void);

/**
 * 获取全局统计指针
 */
sniffer_stats_t *crid_sniffer_get_stats(void);

/**
 * 初始化 Wi-Fi 监控模式（NULL mode + Promiscuous）
 * @return ESP_OK 成功，否则失败
 */
esp_err_t crid_sniffer_init(void);

/**
 * 启动信道保持任务（锁定信道，防止漂移）
 */
void crid_sniffer_start_channel_hold(void);

/** lcdfix15: 停止信道保持任务（切到模拟模式前必须调用） */
void crid_sniffer_stop_channel_hold(void);

/** lcdfix19: 获取当前扫描信道（1/6/11 跳频中） */
uint8_t crid_sniffer_get_current_channel(void);

/** lcdfix30: 获取当前 TDD 阶段（0=WiFi, 1=BLE） */
int crid_sniffer_get_tdd_phase(void);

/** lcdfix30: 注册 TDD BLE 暂停/恢复回调 */
void crid_sniffer_set_ble_tdd_callbacks(void (*pause_cb)(void), void (*resume_cb)(void));

/**
 * v1.3: 停止 sniffer 并释放 Wi-Fi 资源
 * 用于模式切换到模拟模式前调用。
 * @return ESP_OK 成功
 */
esp_err_t crid_sniffer_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // CRID_SNIFFER_H
