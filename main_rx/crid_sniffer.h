/**
 * crid_sniffer.h — Wi-Fi Sniffer 模块接口 (detector variant)
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
 * 仅创建 sniffer 消息队列（不初始化 WiFi）。
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
 * 启动信道轮转任务（ch6 长驻 + ch1/ch11 快速扫描）
 */
void crid_sniffer_start_channel_hold(void);

/** 停止信道轮转任务 */
void crid_sniffer_stop_channel_hold(void);

/** 获取当前扫描信道（1/6/11 轮转中） */
uint8_t crid_sniffer_get_current_channel(void);

#ifdef __cplusplus
}
#endif

#endif // CRID_SNIFFER_H

/** 暂停/恢复 WiFi promiscuous 接收（BLE GATT 连接期间让射频给 BLE TX） */
void crid_sniffer_pause(bool pause);
