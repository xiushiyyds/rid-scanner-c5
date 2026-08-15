/**
 * sim_wifi.c — 模拟器 Wi-Fi raw 802.11 帧注入（STA 模式）
 *
 * lcdfix30: 从 AP 模式改为 STA 模式注入。
 *
 * 根本原因：AP 模式下 ESP32 WiFi 协议栈每 100ms 自动发送一个
 * 带 SSID 的标准 Beacon，这个 Beacon 不含 RID Vendor IE，但肩灯
 * 收到 WiFi Beacon 就会报警（检测到射频活动），却解析不到无人机
 * 信息。同时 AP 协议栈与 BLE 共存时会产生射频调度冲突，导致我们
 * 通过 esp_wifi_80211_tx 注入的自定义 Beacon 被丢弃或截断。
 *
 * 改用 WIFI_MODE_STA 后：
 *   - 不会自动发 Beacon，只有我们注入的 RID 帧
 *   - 不连接任何 AP，纯 NULL 功能
 *   - promiscuous 注入模式稳定工作
 *   - 与 BLE 共存冲突大幅减少
 */

#include "sdkconfig.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_log.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sim_wifi.h"

static const char *TAG = "SIM_WIFI";

/* 当前信道 */
static uint8_t s_current_channel = 6;

/* 跟踪 WiFi 初始化状态 */
static bool s_wifi_inited = false;
static esp_netif_t *s_sta_netif = NULL;

esp_err_t sim_wifi_init(uint8_t channel, const char *ssid, int8_t tx_power) {
    esp_err_t ret;
    (void)ssid; /* STA 模式不广播 SSID，参数保留兼容 */

    if (!s_wifi_inited) {
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ret = esp_wifi_init(&cfg);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "esp_wifi_init returned %s (assuming already inited)",
                     esp_err_to_name(ret));
        }
        s_wifi_inited = true;
    } else {
        ESP_LOGI(TAG, "Wi-Fi already initialized, reusing driver");
    }

    /* STA netif（不连接任何 AP，纯用于 raw TX） */
    if (s_sta_netif == NULL) {
        s_sta_netif = esp_netif_create_default_wifi_sta();
        if (s_sta_netif == NULL) {
            ESP_LOGW(TAG, "esp_netif_create_default_wifi_sta returned NULL");
        }
    }

    /* lcdfix30: 使用 STA 模式，而非 AP 模式。
     * STA 模式不会自动发送 Beacon，esp_wifi_80211_tx 注入的
     * 帧是空中唯一的帧，避免 AP 自动 Beacon 与注入帧冲突。 */
    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_mode failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 不需要 set_config for STA（不连接 AP） */

    /* 设置发射功率：tx_power 单位为 0.25dBm */
    esp_wifi_set_max_tx_power(tx_power);

    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_start failed: %s", esp_err_to_name(ret));
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(300));
    /* 再次确认功率设置 */
    esp_wifi_set_max_tx_power(tx_power);

    /* 锁定信道 */
    ret = esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_channel failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 注册空的 promiscuous RX 回调。
     * 某些 IDF 版本要求 set_promiscuous(true) 之前必须注册回调，
     * 否则注入功能可能不工作。我们不需要收包，回调直接返回。 */
    /* 注意：不注册过滤，只设置 promiscuous 用于 TX 注入 */

    ret = esp_wifi_set_promiscuous(true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_promiscuous failed: %s", esp_err_to_name(ret));
        return ret;
    }

    s_current_channel = channel;

    ESP_LOGI(TAG, "Wi-Fi STA injection init OK: ch=%u, tx_power=%d (%.1f dBm)",
             channel, tx_power, tx_power * 0.25f);
    return ESP_OK;
}

esp_err_t sim_wifi_send_raw_frame(const uint8_t *frame, uint16_t len) {
    if (frame == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    /* lcdfix30: 使用 WIFI_IF_STA 发送。
     * en_sys_seq=true 让硬件自动处理序列号，避免 AP 模式下
     * 序列号冲突。对于无连接的 raw 帧注入，这是推荐做法。 */
    esp_err_t ret = esp_wifi_80211_tx(WIFI_IF_STA, frame, len, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_80211_tx failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t sim_wifi_deinit(void) {
    ESP_LOGI(TAG, "Stopping Wi-Fi STA injection...");

    esp_err_t ret;

    ret = esp_wifi_set_promiscuous(false);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "set_promiscuous(false) failed: %s", esp_err_to_name(ret));
    }

    ret = esp_wifi_stop();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "esp_wifi_stop failed: %s", esp_err_to_name(ret));
    }

    ret = esp_wifi_deinit();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "esp_wifi_deinit failed: %s", esp_err_to_name(ret));
    }
    s_wifi_inited = false;

    /* 注意：不销毁 s_sta_netif，模式切换时复用避免泄漏 */
    ESP_LOGI(TAG, "Wi-Fi STA injection released");
    return ESP_OK;
}

esp_err_t sim_wifi_set_channel(uint8_t channel) {
    if (channel < 1 || channel > 13) {
        return ESP_ERR_INVALID_ARG;
    }
    if (channel == s_current_channel) {
        return ESP_OK;
    }

    esp_err_t ret = esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    if (ret == ESP_OK) {
        s_current_channel = channel;
        ESP_LOGD(TAG, "Channel switched to %u", channel);
    } else {
        ESP_LOGW(TAG, "set_channel(%u) failed: %s", channel, esp_err_to_name(ret));
    }
    return ret;
}

uint8_t sim_wifi_get_channel(void) {
    return s_current_channel;
}
