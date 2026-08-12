/**
 * sim_wifi.h — 模拟器 Wi-Fi AP 模式 + raw frame 发送
 *
 * 支持可配置的发射功率。
 */

#ifndef SIM_WIFI_H
#define SIM_WIFI_H

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 初始化 Wi-Fi AP 模式（用于 raw 802.11 帧发送）
 * 不连接网络，仅用于发射 Beacon 帧。
 * @param channel Wi-Fi 信道
 * @param ssid AP 的 SSID
 * @param tx_power 发射功率 (单位0.25dBm, 20=5dBm, 80=20dBm)
 * @return ESP_OK 成功
 */
esp_err_t sim_wifi_init(uint8_t channel, const char *ssid, int8_t tx_power);

/**
 * 发送 raw 802.11 帧
 * @param frame 帧数据
 * @param len 帧长度
 * @return ESP_OK 成功
 */
esp_err_t sim_wifi_send_raw_frame(const uint8_t *frame, uint16_t len);

/**
 * 停止 Wi-Fi 并释放资源
 * 用于模式切换时从模拟模式回到扫描模式。
 * @return ESP_OK 成功
 */
esp_err_t sim_wifi_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // SIM_WIFI_H

/**
 * 动态切换 Wi-Fi 信道（多目标分散信道时使用）
 * @param channel 目标信道 (1~13)
 * @return ESP_OK 成功
 */
esp_err_t sim_wifi_set_channel(uint8_t channel);

/**
 * 获取当前 Wi-Fi 信道
 * @return 当前信道号
 */
uint8_t sim_wifi_get_channel(void);
