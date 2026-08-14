/**
 * sim_wifi.c — 模拟器 Wi-Fi AP 模式实现
 *
 * 支持可配置发射功率，适配多目标场景。
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

#define AP_DEFAULT_PASSWORD "12345678"

/* 当前信道（动态切换时记录） */
static uint8_t s_current_channel = 6;

/* lcdfix16: 记录 AP netif 是否已创建，避免每次 sim_wifi_init 都
 * 调 esp_netif_create_default_wifi_ap() 造成 netif 对象泄漏。
 * sniffer deinit 不会销毁 default netif，重复创建会让 AP 起不来。 */
static esp_netif_t *s_ap_netif = NULL;

/* lcdfix16: 跟踪 esp_wifi_init 状态。
 * 模式切换时如果上一次 esp_wifi_deinit 没有真正完全释放，
 * 再调 esp_wifi_init 会失败，但 WiFi 实际上仍然可用。
 * 用静态标志避免重复 init，比依赖不存在的 ESP_ERR_WIFI_INITED 可靠。 */
static bool s_wifi_inited = false;

esp_err_t sim_wifi_init(uint8_t channel, const char *ssid, int8_t tx_power) {
    esp_err_t ret;

    if (!s_wifi_inited) {
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ret = esp_wifi_init(&cfg);
        if (ret != ESP_OK) {
            /* 某些模式切换路径下 controller 没完全 deinit，
             * 继续走 set_mode/start，通常能恢复。 */
            ESP_LOGW(TAG, "esp_wifi_init returned %s (assuming already inited)",
                     esp_err_to_name(ret));
        }
        s_wifi_inited = true;
    } else {
        ESP_LOGI(TAG, "Wi-Fi already initialized, reusing driver");
    }

    /* 只在第一次创建 AP netif，后续模式切换复用 */
    if (s_ap_netif == NULL) {
        s_ap_netif = esp_netif_create_default_wifi_ap();
        if (s_ap_netif == NULL) {
            ESP_LOGW(TAG, "esp_netif_create_default_wifi_ap returned NULL (may already exist)");
        }
    }

    wifi_config_t ap_config = { 0 };
    const char *ap_ssid = (ssid != NULL && ssid[0] != '\0') ? ssid : "NekolunaRID-SIM";

    snprintf((char *)ap_config.ap.ssid, sizeof(ap_config.ap.ssid), "%s", ap_ssid);
    ap_config.ap.ssid_len = strlen((char *)ap_config.ap.ssid);
    ap_config.ap.channel = channel;
    ap_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    strncpy((char *)ap_config.ap.password, AP_DEFAULT_PASSWORD,
            sizeof(ap_config.ap.password) - 1);
    ap_config.ap.password[sizeof(ap_config.ap.password) - 1] = '\0';
    ap_config.ap.max_connection = 4;
    ap_config.ap.beacon_interval = 100;

    ret = esp_wifi_set_mode(WIFI_MODE_AP);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_mode failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_wifi_set_config(WIFI_IF_AP, &ap_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_config(AP) failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* 设置发射功率：tx_power 单位为 0.25dBm */
    esp_wifi_set_max_tx_power(tx_power);

    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_start failed: %s", esp_err_to_name(ret));
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(500));
    /* 再次确认功率设置 */
    esp_wifi_set_max_tx_power(tx_power);

    s_current_channel = channel;

    /* lcdfix16: 移除这里的 esp_wifi_set_promiscuous(true)。
     * AP 模式下不需要 promiscuous，之前开了也没注册回调，
     * 反而可能让控制器状态异常。80211_tx 帧在 AP 模式下直接发送。 */

    ESP_LOGI(TAG, "Wi-Fi AP init OK: ch=%u, SSID=%s, tx_power=%d (0.25dBm) = %.1f dBm",
             channel, ap_config.ap.ssid, tx_power, tx_power * 0.25f);
    return ESP_OK;
}

esp_err_t sim_wifi_send_raw_frame(const uint8_t *frame, uint16_t len) {
    if (frame == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = esp_wifi_80211_tx(WIFI_IF_AP, frame, len, false);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_80211_tx failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t sim_wifi_deinit(void) {
    ESP_LOGI(TAG, "Stopping Wi-Fi AP mode...");

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
    /* lcdfix16: 复位初始化标志。即使 deinit 返回错误，也认为
     * WiFi 驱动已释放；下次 init 会重新 esp_wifi_init。 */
    s_wifi_inited = false;

    ESP_LOGI(TAG, "Wi-Fi AP mode released");
    return ESP_OK;
}

esp_err_t sim_wifi_set_channel(uint8_t channel) {
    if (channel < 1 || channel > 13) {
        return ESP_ERR_INVALID_ARG;
    }
    if (channel == s_current_channel) {
        return ESP_OK;  /* 无需切换 */
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
