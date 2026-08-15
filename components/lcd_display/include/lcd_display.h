/**
 * lcd_display.h — ST7789 LCD 显示模块 (detector variant v2.0.0)
 *
 * 硬件目标：LILYGO T-Display-C5
 *   - ESP32-C5 (RISC-V 32-bit @ 240MHz)
 *   - 1.9" ST7789 IPS LCD, 170×320, RGB565, SPI
 *   - CST816S 电容触摸 (I2C)
 *   - AXP2602 电池管理
 *   - 16MB Flash + 8MB PSRAM
 *
 * 纯侦测板页面：HOME → LIST → DETAIL → HOME（无 SIM 页面）
 */

#ifndef LCD_DISPLAY_H
#define LCD_DISPLAY_H

#include <stdint.h>
#include <stdbool.h>
#include "crid_rx_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 * 引脚配置 — LILYGO T-Display-C5
 * ================================================================ */
#define LCD_PIN_SCLK        7
#define LCD_PIN_MOSI        9
#define LCD_PIN_CS          26
#define LCD_PIN_DC          8
#define LCD_PIN_RST         23
#define LCD_PIN_BK          25

#define TOUCH_PIN_SDA       2
#define TOUCH_PIN_SCL       3
#define TOUCH_PIN_INT       27
#define TOUCH_PIN_RST       24
#define TOUCH_I2C_ADDR      0x15

#define BTN_USER_PIN        0
#define BTN_BOOT_PIN        28

#define AXP2602_I2C_ADDR    0x62
#define AXP2602_INT_PIN     10

/* ================================================================
 * 屏幕参数
 * ================================================================ */
#define LCD_WIDTH            170
#define LCD_HEIGHT           320
#define LCD_SPI_FREQ_HZ     (20 * 1000 * 1000)
#define LCD_FB_SIZE         (LCD_WIDTH * LCD_HEIGHT * 2)

/* ================================================================
 * 显示页面定义（纯侦测板：3 页）
 * ================================================================ */
typedef enum {
    LCD_PAGE_HOME = 0,
    LCD_PAGE_LIST,
    LCD_PAGE_DETAIL,
    LCD_PAGE_COUNT
} lcd_page_t;

/* ================================================================
 * 按键事件
 * ================================================================ */
typedef enum {
    LCD_KEY_NONE = 0,
    LCD_KEY_PREV,
    LCD_KEY_NEXT,
    LCD_KEY_SELECT,
    LCD_KEY_BACK,
    LCD_KEY_POWER,
} lcd_key_event_t;

/* ================================================================
 * API 函数
 * ================================================================ */
int lcd_display_init(void);

void lcd_display_set_source(uav_track_t *tracker_table,
                            void *tracker_mutex,
                            int max_uavs);

void lcd_display_send_key(lcd_key_event_t key);
void lcd_display_set_page(lcd_page_t page);
lcd_page_t lcd_display_get_page(void);
void lcd_display_set_selection(int index);
int lcd_display_get_selection(void);
void lcd_display_command(const char *cmd);
void lcd_display_show_pair_pin(const char *pin_code);

int lcd_display_get_battery_voltage(uint16_t *voltage_mv);

/* GPS 状态提供回调 */
typedef bool (*lcd_gps_provider_cb_t)(double *lat, double *lon,
                                       double *alt, int *sats_out);
void lcd_display_register_gps_provider(lcd_gps_provider_cb_t cb);

/* 当前扫描信道回调 */
typedef uint8_t (*lcd_channel_provider_cb_t)(void);
void lcd_display_register_channel_provider(lcd_channel_provider_cb_t cb);

#ifdef __cplusplus
}
#endif

#endif // LCD_DISPLAY_H
