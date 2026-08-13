/**
 * lcd_display.c — ST7789 LCD 显示模块（T-Display-C5 适配版）
 *
 * 硬件目标：LILYGO T-Display-C5
 *   - 1.9" ST7789, 170×320, SPI
 *   - 帧缓冲 108,800 bytes, 优先内部 SRAM（PSRAM 时分块刷新）
 *   - 5fps 刷新 (200ms)
 *
 * 屏幕布局（170×320 竖屏）：
 *   - 状态栏：顶部 16px
 *   - 内容区：16~304px（288px）
 *   - 底栏：304~320px（16px）
 *   - 每行文字：7px高（5px字+2px间距）
 *   - 内容区可显示约 41 行
 */

#include <stdio.h>
#include <string.h>
#include "lcd_display.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "esp_cache.h"

/* sniffer 锁定信道（与 crid_sniffer.c 中 FIXED_CHANNEL 一致） */
#ifndef FIXED_CHANNEL
#define FIXED_CHANNEL 6
#endif

static const char *TAG = "lcd_display";

/* ================================================================
 * 模拟器状态信息（由 app_main 通过 lcd_display_set_sim_info 更新）
 * ================================================================ */
static sim_display_info_t s_sim_info = { 0 };

void lcd_display_set_sim_info(const sim_display_info_t *info) {
    if (info) {
        memcpy(&s_sim_info, info, sizeof(sim_display_info_t));
    }
}

/* ================================================================
 * 内部状态
 * ================================================================ */
static esp_lcd_panel_handle_t s_panel = NULL;
static uint16_t *s_fb = NULL;          // 帧缓冲
static bool s_fb_in_psram = false;     // 帧缓冲是否在 PSRAM（决定刷新策略）
static TaskHandle_t s_refresh_task = NULL;
static QueueHandle_t s_key_queue = NULL;

static uav_track_t *s_tracker = NULL;
static void *s_tracker_mutex = NULL;
static int s_max_uavs = 0;

static volatile lcd_page_t s_page = LCD_PAGE_HOME;
static volatile int s_selection = 0;
static volatile int s_scroll_offset = 0;

/* ================================================================
 * 5x7 ASCII 字体（与之前相同）
 * ================================================================ */
static const uint8_t font_5x7[96][5] = {
    {0x00,0x00,0x00,0x00,0x00}, // space
    {0x00,0x00,0x5F,0x00,0x00}, // !
    {0x00,0x07,0x00,0x07,0x00}, // "
    {0x14,0x7F,0x14,0x7F,0x14}, // #
    {0x24,0x2A,0x7F,0x2A,0x12}, // $
    {0x23,0x13,0x08,0x64,0x62}, // %
    {0x36,0x49,0x55,0x22,0x50}, // &
    {0x00,0x05,0x03,0x00,0x00}, // '
    {0x00,0x1C,0x22,0x41,0x00}, // (
    {0x00,0x41,0x22,0x1C,0x00}, // )
    {0x08,0x2A,0x1C,0x2A,0x08}, // *
    {0x08,0x08,0x3E,0x08,0x08}, // +
    {0x00,0x50,0x30,0x00,0x00}, // ,
    {0x08,0x08,0x08,0x08,0x08}, // -
    {0x00,0x60,0x60,0x00,0x00}, // .
    {0x20,0x10,0x08,0x04,0x02}, // /
    {0x3E,0x51,0x49,0x45,0x3E}, // 0
    {0x00,0x42,0x7F,0x40,0x00}, // 1
    {0x72,0x49,0x49,0x49,0x46}, // 2
    {0x21,0x41,0x45,0x4B,0x31}, // 3
    {0x18,0x14,0x12,0x7F,0x10}, // 4
    {0x27,0x45,0x45,0x45,0x39}, // 5
    {0x3C,0x4A,0x49,0x49,0x30}, // 6
    {0x01,0x71,0x09,0x05,0x03}, // 7
    {0x36,0x49,0x49,0x49,0x36}, // 8
    {0x06,0x49,0x49,0x29,0x1E}, // 9
    {0x00,0x36,0x36,0x00,0x00}, // :
    {0x00,0x56,0x36,0x00,0x00}, // ;
    {0x08,0x14,0x22,0x41,0x00}, // <
    {0x14,0x14,0x14,0x14,0x14}, // =
    {0x00,0x41,0x22,0x14,0x08}, // >
    {0x02,0x01,0x51,0x09,0x06}, // ?
    {0x32,0x49,0x79,0x41,0x3E}, // @
    {0x7E,0x11,0x11,0x11,0x7E}, // A
    {0x7F,0x49,0x49,0x49,0x36}, // B
    {0x3E,0x41,0x41,0x41,0x22}, // C
    {0x7F,0x41,0x41,0x22,0x1C}, // D
    {0x7F,0x49,0x49,0x49,0x41}, // E
    {0x7F,0x09,0x09,0x09,0x01}, // F
    {0x3E,0x41,0x49,0x49,0x7A}, // G
    {0x7F,0x08,0x08,0x08,0x7F}, // H
    {0x00,0x41,0x7F,0x41,0x00}, // I
    {0x20,0x40,0x41,0x3F,0x01}, // J
    {0x7F,0x08,0x14,0x22,0x41}, // K
    {0x7F,0x40,0x40,0x40,0x40}, // L
    {0x7F,0x02,0x04,0x02,0x7F}, // M
    {0x7F,0x04,0x08,0x10,0x7F}, // N
    {0x3E,0x41,0x41,0x41,0x3E}, // O
    {0x7F,0x09,0x09,0x09,0x06}, // P
    {0x3E,0x41,0x51,0x21,0x5E}, // Q
    {0x7F,0x09,0x19,0x29,0x46}, // R
    {0x46,0x49,0x49,0x49,0x31}, // S
    {0x01,0x01,0x7F,0x01,0x01}, // T
    {0x3F,0x40,0x40,0x40,0x3F}, // U
    {0x1F,0x20,0x40,0x20,0x1F}, // V
    {0x7F,0x20,0x18,0x20,0x7F}, // W
    {0x63,0x14,0x08,0x14,0x63}, // X
    {0x03,0x04,0x78,0x04,0x03}, // Y
    {0x61,0x51,0x49,0x45,0x43}, // Z
    {0x00,0x00,0x7F,0x41,0x41}, // [
    {0x02,0x04,0x08,0x10,0x20}, // backslash
    {0x41,0x41,0x7F,0x00,0x00}, // ]
    {0x04,0x02,0x01,0x02,0x04}, // ^
    {0x40,0x40,0x40,0x40,0x40}, // _
    {0x00,0x01,0x02,0x04,0x00}, // `
    {0x20,0x54,0x54,0x54,0x78}, // a
    {0x7F,0x48,0x44,0x44,0x38}, // b
    {0x38,0x44,0x44,0x44,0x20}, // c
    {0x38,0x44,0x44,0x48,0x7F}, // d
    {0x38,0x54,0x54,0x54,0x18}, // e
    {0x08,0x7E,0x09,0x01,0x02}, // f
    {0x08,0x14,0x54,0x54,0x3C}, // g
    {0x7F,0x08,0x04,0x04,0x78}, // h
    {0x00,0x44,0x7D,0x40,0x00}, // i
    {0x20,0x40,0x44,0x3D,0x00}, // j
    {0x00,0x7F,0x10,0x28,0x44}, // k
    {0x00,0x41,0x7F,0x40,0x00}, // l
    {0x7C,0x04,0x18,0x04,0x78}, // m
    {0x7C,0x08,0x04,0x04,0x78}, // n
    {0x38,0x44,0x44,0x44,0x38}, // o
    {0x7C,0x14,0x14,0x14,0x08}, // p
    {0x08,0x14,0x14,0x18,0x7C}, // q
    {0x7C,0x08,0x04,0x04,0x08}, // r
    {0x48,0x54,0x54,0x54,0x20}, // s
    {0x04,0x3F,0x44,0x40,0x20}, // t
    {0x3C,0x40,0x40,0x20,0x7C}, // u
    {0x1C,0x20,0x40,0x20,0x1C}, // v
    {0x3C,0x40,0x30,0x40,0x3C}, // w
    {0x44,0x28,0x10,0x28,0x44}, // x
    {0x0C,0x50,0x50,0x50,0x3C}, // y
    {0x44,0x64,0x54,0x4C,0x44}, // z
    {0x00,0x08,0x36,0x41,0x00}, // {
    {0x00,0x00,0x7F,0x00,0x00}, // |
    {0x00,0x41,0x36,0x08,0x00}, // }
    {0x08,0x08,0x2A,0x1C,0x08}, // ~
};

/* ================================================================
 * 颜色定义（RGB565）
 * ================================================================ */
#define COLOR_BLACK     0x0000
#define COLOR_WHITE     0xFFFF
#define COLOR_RED       0xF800
#define COLOR_GREEN     0x07E0
#define COLOR_BLUE      0x001F
#define COLOR_CYAN      0x07FF
#define COLOR_YELLOW    0xFFE0
#define COLOR_ORANGE    0xFD20
#define COLOR_GRAY      0x8410
#define COLOR_DARK_GREEN 0x03E0

/* ================================================================
 * 屏幕布局常量（170×320 竖屏）
 * ================================================================ */
#define STATUS_BAR_H    16      // 顶部状态栏高度
#define FOOTER_BAR_H    16      // 底部栏高度
#define CONTENT_Y0      STATUS_BAR_H
#define CONTENT_Y1      (LCD_HEIGHT - FOOTER_BAR_H)  // 304
#define CONTENT_H       (CONTENT_Y1 - CONTENT_Y0)    // 288

/* 文字参数：5x7字体，每字符占6×7像素（含1px间距） */
#define CHAR_W          6
#define CHAR_H          7
#define LINE_H          9       // 每行高（7px字+2px行距）
#define CHARS_PER_LINE  (LCD_WIDTH / CHAR_W)  // 170/6 = 28字符

/* ================================================================
 * SPI + LCD 初始化
 * ================================================================ */
static int init_lcd(void)
{
    ESP_LOGI(TAG, "初始化 SPI 总线 (T-Display-C5)");

    /* 配置 SPI 总线 */
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = LCD_PIN_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = LCD_PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 170 * 10 * 2 + 8,  /* 单块 10 行 RGB565 + 余量，避免 bounce buffer 过大 */
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    /* 配置 LCD IO */
    esp_lcd_panel_io_handle_t io_handle = NULL;
    /* 手动复位 LCD (reset_gpio_num 在 ESP-IDF v5.4 的 io_cfg 中已移除) */
    gpio_config_t rst_conf = {
        .pin_bit_mask = (1ULL << LCD_PIN_RST),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&rst_conf);
    gpio_set_level(LCD_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(LCD_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(LCD_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));

    esp_lcd_panel_io_spi_config_t io_cfg = {
        .dc_gpio_num = LCD_PIN_DC,
        .cs_gpio_num = LCD_PIN_CS,
        .pclk_hz = LCD_SPI_FREQ_HZ,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = NULL,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI2_HOST, &io_cfg, &io_handle));

    /* 配置 ST7789 驱动 — reset_gpio_num=-1 因为上面已手动复位 */
    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = -1,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .data_endian = LCD_RGB_DATA_ENDIAN_LITTLE,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_cfg, &s_panel));

    /* 初始化并配置屏幕（参数参照 LILYGO factory.ino 官方示例） */
    esp_lcd_panel_init(s_panel);
    esp_lcd_panel_mirror(s_panel, true, false);
    esp_lcd_panel_swap_xy(s_panel, true);
    esp_lcd_panel_invert_color(s_panel, true);
    esp_lcd_panel_set_gap(s_panel, 0, 35);
    esp_lcd_panel_disp_on_off(s_panel, true);

    ESP_LOGI(TAG, "LCD 初始化完成: %dx%d", LCD_WIDTH, LCD_HEIGHT);
    return 0;
}

/* ================================================================
 * 背光初始化
 * ================================================================ */
static void init_backlight(void)
{
    /* LEDC PWM 背光（参照 LILYGO factory.ino：5kHz, 8-bit, 最大亮度） */
    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    ledc_channel_config_t ch_cfg = {
        .gpio_num = LCD_PIN_BK,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 255,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));
    ESP_LOGI(TAG, "背光开启 (GPIO%d, PWM 255)", LCD_PIN_BK);
}

/* ================================================================
 * 帧缓冲初始化（优先 PSRAM）
 * ================================================================ */
static int init_framebuffer(void)
{
    /* 与 LILYGO factory.ino 保持一致：PSRAM | DMA | 8BIT。
       PSRAM 配合分块刷新 (10行/块) + esp_cache_msync，避免 DMA 读到 cache 脏数据。
       内部 SRAM 108KB 会挤占 WiFi/BT 协议栈，可能导致共存不稳定，因此仅作 fallback。 */
    s_fb = heap_caps_malloc(LCD_FB_SIZE,
                            MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (s_fb) {
        s_fb_in_psram = true;
        ESP_LOGI(TAG, "帧缓冲在 PSRAM: %d bytes @ %p（分块刷新模式）", LCD_FB_SIZE, s_fb);
    } else {
        ESP_LOGW(TAG, "PSRAM 分配失败，尝试内部 SRAM");
        s_fb = heap_caps_malloc(LCD_FB_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
        if (s_fb) {
            s_fb_in_psram = false;
            ESP_LOGI(TAG, "帧缓冲在内部 SRAM: %d bytes @ %p", LCD_FB_SIZE, s_fb);
        }
    }
    if (!s_fb) {
        ESP_LOGE(TAG, "帧缓冲分配失败！需要 %d bytes", LCD_FB_SIZE);
        return -1;
    }
    memset(s_fb, 0, LCD_FB_SIZE);
    return 0;
}

/* ================================================================
 * 绘图原语
 * ================================================================ */
static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b)
{
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

/* 安全的帧缓冲推送：分块刷新，每块 10 行 = 3400 字节。
   实测全屏一次性 DMA 传输在 ESP32-C5 上约 60% 后数据出错（雪花），
   分块可避免 SPI DMA 描述符链过长、cache 未同步等问题。 */
static void flush_framebuffer(void)
{
    if (!s_panel || !s_fb) return;

    const int chunk_rows = 10;
    for (int y = 0; y < LCD_HEIGHT; y += chunk_rows) {
        int h = chunk_rows;
        if (y + h > LCD_HEIGHT) h = LCD_HEIGHT - y;
        uint16_t *p = &s_fb[y * LCD_WIDTH];
        size_t len = LCD_WIDTH * h * sizeof(uint16_t);
        /* PSRAM 上的数据要先做 cache 写回，DMA 才能读到最新内容 */
        if (s_fb_in_psram) {
            esp_cache_msync(p, len, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
        }
        esp_lcd_panel_draw_bitmap(s_panel, 0, y, LCD_WIDTH, y + h, p);
    }
}

static void fb_fill(uint16_t color)
{
    for (int i = 0; i < LCD_WIDTH * LCD_HEIGHT; i++) {
        s_fb[i] = color;
    }
}

static void fb_draw_pixel(int x, int y, uint16_t color)
{
    if (x < 0 || x >= LCD_WIDTH || y < 0 || y >= LCD_HEIGHT) return;
    s_fb[y * LCD_WIDTH + x] = color;
}

static void fb_draw_hline(int x, int y, int w, uint16_t color)
{
    if (y < 0 || y >= LCD_HEIGHT) return;
    if (x < 0) { w += x; x = 0; }
    if (x + w > LCD_WIDTH) w = LCD_WIDTH - x;
    for (int i = 0; i < w; i++) {
        s_fb[y * LCD_WIDTH + x + i] = color;
    }
}

static void fb_draw_vline(int x, int y, int h, uint16_t color)
{
    if (x < 0 || x >= LCD_WIDTH) return;
    if (y < 0) { h += y; y = 0; }
    if (y + h > LCD_HEIGHT) h = LCD_HEIGHT - y;
    for (int i = 0; i < h; i++) {
        s_fb[(y + i) * LCD_WIDTH + x] = color;
    }
}

static void fb_draw_rect(int x, int y, int w, int h, uint16_t color)
{
    fb_draw_hline(x, y, w, color);
    fb_draw_hline(x, y + h - 1, w, color);
    fb_draw_vline(x, y, h, color);
    fb_draw_vline(x + w - 1, y, h, color);
}

static void fb_fill_rect(int x, int y, int w, int h, uint16_t color)
{
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > LCD_WIDTH) w = LCD_WIDTH - x;
    if (y + h > LCD_HEIGHT) h = LCD_HEIGHT - y;
    for (int row = y; row < y + h; row++) {
        for (int col = x; col < x + w; col++) {
            s_fb[row * LCD_WIDTH + col] = color;
        }
    }
}

/* 绘制 5x7 字符 */
static void fb_draw_char(int x, int y, char c, uint16_t color)
{
    if (c < 32 || c > 127) c = '?';
    const uint8_t *glyph = font_5x7[c - 32];
    for (int col = 0; col < 5; col++) {
        for (int row = 0; row < 7; row++) {
            if (glyph[col] & (1 << row)) {
                fb_draw_pixel(x + col, y + row, color);
            }
        }
    }
}

/* 绘制字符串 */
static void fb_draw_string(int x, int y, const char *str, uint16_t color)
{
    while (*str) {
        fb_draw_char(x, y, *str, color);
        x += CHAR_W;
        str++;
        if (x + CHAR_W > LCD_WIDTH) break;
    }
}

/* 绘制固定宽度截断字符串（尾部补空格） */
static void fb_draw_string_fixed(int x, int y, const char *str, int max_chars, uint16_t color)
{
    int i = 0;
    while (i < max_chars && str[i]) {
        fb_draw_char(x + i * CHAR_W, y, str[i], color);
        i++;
    }
    /* 补空格 */
    while (i < max_chars) {
        fb_draw_char(x + i * CHAR_W, y, ' ', color);
        i++;
    }
}

/* ================================================================
 * 页面渲染
 * ================================================================ */

/* 渲染状态栏 */
static void render_status_bar(uint32_t scan_count, int wifi_ch)
{
    fb_fill_rect(0, 0, LCD_WIDTH, STATUS_BAR_H, COLOR_BLUE);
    char buf[64];
    snprintf(buf, sizeof(buf), "CH:%d  Total:%lu", wifi_ch, (unsigned long)scan_count);
    fb_draw_string(2, 4, buf, COLOR_WHITE);
}

/* 渲染底部栏 */
static void render_footer(lcd_page_t page, int sel, int total)
{
    fb_fill_rect(0, CONTENT_Y1, LCD_WIDTH, FOOTER_BAR_H, COLOR_DARK_GREEN);
    char buf[64];
    const char *page_names[] = {"HOME", "LIST", "DETAIL", "SIM_CFG", "SIM"};
    int idx = (int)page;
    if (idx < 0 || idx >= 5) idx = 0;
    if (page <= LCD_PAGE_DETAIL) {
        snprintf(buf, sizeof(buf), "%s %d/%d", page_names[idx], sel + 1, total > 0 ? total : 1);
    } else {
        snprintf(buf, sizeof(buf), "%s", page_names[idx]);
    }
    fb_draw_string(2, CONTENT_Y1 + 4, buf, COLOR_WHITE);
}

/* 渲染主页 */
static void render_home_page(void)
{
    if (!s_tracker || !s_tracker_mutex) return;

    int active = 0;
    int gb_count = 0, astm_count = 0, asdstan_count = 0, dji_count = 0;

    xSemaphoreTake(s_tracker_mutex, portMAX_DELAY);
    for (int i = 0; i < s_max_uavs; i++) {
        if (s_tracker[i].active) {
            active++;
            if (s_tracker[i].is_dji) {
                dji_count++;
            }
            switch (s_tracker[i].protocol) {
                case RID_PROTOCOL_ASTM_F3411: astm_count++; break;
                case RID_PROTOCOL_ASD_STAN:   asdstan_count++; break;
                case RID_PROTOCOL_GB42590:
                case RID_PROTOCOL_GB46750:    gb_count++; break;
                default: break;
            }
        }
    }
    xSemaphoreGive(s_tracker_mutex);

    int y = CONTENT_Y0 + 8;

    /* 标题 */
    fb_draw_string(20, y, "RID SCANNER", COLOR_CYAN);
    y += LINE_H + 4;
    fb_draw_string(16, y, "T-Display-C5", COLOR_GRAY);
    y += LINE_H + 8;

    /* 分割线 */
    fb_draw_hline(4, y, LCD_WIDTH - 8, COLOR_GRAY);
    y += 6;

    /* 统计信息 */
    char buf[64];
    snprintf(buf, sizeof(buf), "Active UAVs: %d", active);
    fb_draw_string(4, y, buf, active > 0 ? COLOR_GREEN : COLOR_WHITE);
    y += LINE_H + 4;

    fb_draw_hline(4, y, LCD_WIDTH - 8, COLOR_GRAY);
    y += 6;

    /* 协议分类 */
    fb_draw_string(4, y, "Protocol Breakdown:", COLOR_YELLOW);
    y += LINE_H + 2;

    snprintf(buf, sizeof(buf), "GB42590/46750: %d", gb_count);
    fb_draw_string(8, y, buf, COLOR_WHITE);
    y += LINE_H;

    snprintf(buf, sizeof(buf), "ASTM F3411:   %d", astm_count);
    fb_draw_string(8, y, buf, COLOR_WHITE);
    y += LINE_H;

    snprintf(buf, sizeof(buf), "ASD-STAN:     %d", asdstan_count);
    fb_draw_string(8, y, buf, COLOR_WHITE);
    y += LINE_H;

    snprintf(buf, sizeof(buf), "DJI DroneID:  %d", dji_count);
    fb_draw_string(8, y, buf, COLOR_ORANGE);
    y += LINE_H + 8;

    /* 提示 */
    fb_draw_string(4, LCD_HEIGHT - 70, "[User] = Next page", COLOR_GRAY);
    fb_draw_string(4, LCD_HEIGHT - 60, "[Boot] = Select/Scroll", COLOR_GRAY);

    /* SIM 模式入口提示 */
    if (s_sim_info.is_sim_running) {
        fb_draw_string(4, LCD_HEIGHT - 44, ">> SIM TX ACTIVE <<", COLOR_GREEN);
    } else {
        fb_draw_string(4, LCD_HEIGHT - 44, "SIM MODE available", COLOR_ORANGE);
    }
    fb_draw_string(4, LCD_HEIGHT - 34, "(Scroll to SIM pages)", COLOR_GRAY);
}

/* 渲染列表页 */
static void render_list_page(void)
{
    if (!s_tracker || !s_tracker_mutex) return;

    /* 收集活跃无人机 */
    int indices[32];
    int count = 0;

    xSemaphoreTake(s_tracker_mutex, portMAX_DELAY);
    for (int i = 0; i < s_max_uavs && count < 32; i++) {
        if (s_tracker[i].active) {
            indices[count++] = i;
        }
    }
    xSemaphoreGive(s_tracker_mutex);

    if (count == 0) {
        fb_draw_string(20, CONTENT_Y0 + 40, "No UAVs detected", COLOR_GRAY);
        return;
    }

    /* 每行显示一架无人机，行高20px（两行文字：ID + RSSI/协议） */
    int item_h = 28;
    int items_per_page = CONTENT_H / item_h;  // 约10个

    /* 确保选中项在可见范围 */
    if (s_selection >= count) s_selection = count - 1;
    if (s_selection < 0) s_selection = 0;

    int start = s_scroll_offset;
    int end = start + items_per_page;
    if (end > count) end = count;

    int y = CONTENT_Y0;
    for (int idx = start; idx < end; idx++) {
        int i = indices[idx];
        bool selected = (idx == s_selection);
        uint16_t bg = selected ? rgb565(0, 80, 160) : COLOR_BLACK;
        uint16_t fg = selected ? COLOR_WHITE : COLOR_GREEN;

        /* 背景 */
        fb_fill_rect(0, y, LCD_WIDTH, item_h - 1, bg);
        fb_draw_hline(0, y + item_h - 1, LCD_WIDTH, COLOR_GRAY);

        /* 第一行：序号 + ID */
        char buf[64];
        snprintf(buf, sizeof(buf), "#%d %s", idx + 1,
                 s_tracker[i].basic_id.uas_id[0] ? s_tracker[i].basic_id.uas_id : "----");
        fb_draw_string_fixed(2, y + 2, buf, CHARS_PER_LINE - 1, fg);

        /* 第二行：RSSI + 协议 */
        const char *proto_str[] = {"??", "ASTM", "EUST", "GB42", "GB46"};
        const char *ps = (s_tracker[i].protocol <= 4) ? proto_str[s_tracker[i].protocol] : "??";

        snprintf(buf, sizeof(buf), "%ddB %s", s_tracker[i].last_rssi, ps);
        uint16_t sub_color = selected ? rgb565(180, 220, 255) : COLOR_CYAN;
        fb_draw_string_fixed(2, y + 12, buf, CHARS_PER_LINE - 1, sub_color);

        y += item_h;
    }
}

/* 渲染详情页 */
static void render_detail_page(void)
{
    if (!s_tracker || !s_tracker_mutex) return;

    /* 找到选中活跃无人机 */
    int idx = -1;
    int sel = s_selection;
    int active_idx = 0;

    xSemaphoreTake(s_tracker_mutex, portMAX_DELAY);
    for (int i = 0; i < s_max_uavs; i++) {
        if (s_tracker[i].active) {
            if (active_idx == sel) { idx = i; break; }
            active_idx++;
        }
    }
    xSemaphoreGive(s_tracker_mutex);

    if (idx < 0) {
        fb_draw_string(20, CONTENT_Y0 + 40, "No selection", COLOR_GRAY);
        return;
    }

    int y = CONTENT_Y0 + 4;

    /* 标题 */
    fb_fill_rect(0, y - 2, LCD_WIDTH, LINE_H + 4, rgb565(0, 60, 120));
    fb_draw_string(4, y, "UAV DETAIL", COLOR_WHITE);
    y += LINE_H + 6;

    /* 字段列表 */
    char buf[64];
    const char *proto_names[] = {"Unknown", "ASTM F3411", "ASD-STAN", "GB 42590", "GB 46750"};

    /* 协议 */
    const char *pname = (s_tracker[idx].protocol <= 4) ? proto_names[s_tracker[idx].protocol] : "Unknown";
    snprintf(buf, sizeof(buf), "Proto: %s", pname);
    fb_draw_string(2, y, buf, COLOR_YELLOW);
    y += LINE_H + 2;

    /* ID */
    snprintf(buf, sizeof(buf), "ID: %s", s_tracker[idx].basic_id.uas_id[0] ? s_tracker[idx].basic_id.uas_id : "N/A");
    fb_draw_string(2, y, buf, COLOR_WHITE);
    y += LINE_H + 2;

    /* RSSI */
    snprintf(buf, sizeof(buf), "RSSI: %d dBm", s_tracker[idx].last_rssi);
    fb_draw_string(2, y, buf, COLOR_CYAN);
    y += LINE_H + 2;

    /* 信道 */
    snprintf(buf, sizeof(buf), "CH: %d", s_tracker[idx].last_channel);
    fb_draw_string(2, y, buf, COLOR_GREEN);
    y += LINE_H + 2;

    /* 高度 - 优先显示相对高度 (height)，备选气压高度、大地高度 */
    if ((int)s_tracker[idx].location.height != 0) {
        /* 相对高度：标注参考类型 T=Takeoff G=Ground */
        const char *ref_tag = (s_tracker[idx].location.height_ref == 0) ? "T" : "G";
        snprintf(buf, sizeof(buf), "RelAlt%s: %d m", ref_tag, (int)s_tracker[idx].location.height);
        fb_draw_string(2, y, buf, COLOR_YELLOW);
        y += LINE_H + 2;
    } else if ((int)s_tracker[idx].location.altitude_baro != 0) {
        snprintf(buf, sizeof(buf), "BaroAlt: %d m", (int)s_tracker[idx].location.altitude_baro);
        fb_draw_string(2, y, buf, COLOR_GREEN);
        y += LINE_H + 2;
    } else if ((int)s_tracker[idx].location.altitude_geo != 0) {
        snprintf(buf, sizeof(buf), "GeoAlt: %d m", (int)s_tracker[idx].location.altitude_geo);
        fb_draw_string(2, y, buf, COLOR_GRAY);
        y += LINE_H + 2;
    }

    /* 速度 */
    if ((int)s_tracker[idx].location.speed_horizontal > 0) {
        snprintf(buf, sizeof(buf), "Speed: %d m/s", (int)s_tracker[idx].location.speed_horizontal);
        fb_draw_string(2, y, buf, COLOR_GREEN);
        y += LINE_H + 2;
    }

    /* MAC 地址 */
    snprintf(buf, sizeof(buf), "MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             s_tracker[idx].mac[0], s_tracker[idx].mac[1],
             s_tracker[idx].mac[2], s_tracker[idx].mac[3],
             s_tracker[idx].mac[4], s_tracker[idx].mac[5]);
    fb_draw_string(2, y, buf, COLOR_GRAY);
    y += LINE_H + 2;

    /* 首次/最后出现 */
    if (s_tracker[idx].last_seen_ms > 0) {
        /* [Bug FF 修复] 使用 last_seen 显示"多久前见过"，而非 first_seen(追踪时长) */
        snprintf(buf, sizeof(buf), "Seen: %lus ago",
                 (unsigned long)((uint32_t)(esp_timer_get_time() / 1000 - s_tracker[idx].last_seen_ms) / 1000));
        fb_draw_string(2, y, buf, COLOR_GRAY);
        y += LINE_H + 2;
    }

    /* 分割线 */
    y += 4;
    fb_draw_hline(2, y, LCD_WIDTH - 4, COLOR_GRAY);
}

/* ================================================================
 * 模拟配置页
 * ================================================================ */
static void render_sim_config_page(void)
{
    int y = CONTENT_Y0 + 4;

    /* 标题 */
    fb_fill_rect(0, y - 2, LCD_WIDTH, LINE_H + 4, rgb565(120, 0, 60));
    fb_draw_string(4, y, "SIM CONFIG", COLOR_WHITE);
    y += LINE_H + 8;

    char buf[64];
    const char *mode_names[] = {"CIRCLE", "PINGPONG", "S_SEARCH"};
    int mode_idx = s_sim_info.sim_flight_mode;
    if (mode_idx < 0 || mode_idx >= 3) mode_idx = 0;

    /* 状态 */
    if (s_sim_info.is_sim_running) {
        fb_draw_string(4, y, "Status: RUNNING", COLOR_GREEN);
    } else {
        fb_draw_string(4, y, "Status: STOPPED", COLOR_GRAY);
    }
    y += LINE_H + 4;

    /* 飞行模式 */
    snprintf(buf, sizeof(buf), "Mode: %s", mode_names[mode_idx]);
    fb_draw_string(4, y, buf, COLOR_YELLOW);
    y += LINE_H + 2;

    /* 信道 */
    snprintf(buf, sizeof(buf), "Channel: %d", s_sim_info.sim_channel);
    fb_draw_string(4, y, buf, COLOR_CYAN);
    y += LINE_H + 2;

    /* UAS ID */
    snprintf(buf, sizeof(buf), "ID: %s",
             s_sim_info.sim_uas_id[0] ? s_sim_info.sim_uas_id : "SIM-ESP32C5");
    fb_draw_string(4, y, buf, COLOR_WHITE);
    y += LINE_H + 2;

    /* SSID */
    snprintf(buf, sizeof(buf), "SSID: %s",
             s_sim_info.sim_ssid[0] ? s_sim_info.sim_ssid : "NekolunaRID-SIM");
    fb_draw_string(4, y, buf, COLOR_WHITE);
    y += LINE_H + 2;

    /* 坐标 */
    snprintf(buf, sizeof(buf), "Lat: %.4f", s_sim_info.sim_lat);
    fb_draw_string(4, y, buf, COLOR_GREEN);
    y += LINE_H + 2;

    snprintf(buf, sizeof(buf), "Lon: %.4f", s_sim_info.sim_lon);
    fb_draw_string(4, y, buf, COLOR_GREEN);
    y += LINE_H + 8;

    /* 操作提示 */
    fb_draw_hline(4, y, LCD_WIDTH - 8, COLOR_GRAY);
    y += 6;
    fb_draw_string(4, y, "[Boot]=Toggle Mode", COLOR_ORANGE);
    y += LINE_H;
    fb_draw_string(4, y, "[User]=Next page", COLOR_GRAY);
}

/* ================================================================
 * 模拟状态页
 * ================================================================ */
static void render_sim_status_page(void)
{
    int y = CONTENT_Y0 + 4;

    /* 标题 */
    fb_fill_rect(0, y - 2, LCD_WIDTH, LINE_H + 4, rgb565(0, 80, 40));
    fb_draw_string(4, y, "SIM STATUS", COLOR_WHITE);
    y += LINE_H + 8;

    char buf[64];

    /* 状态指示灯 */
    if (s_sim_info.is_sim_running) {
        fb_draw_string(4, y, ">> TX ACTIVE <<", COLOR_GREEN);
    } else {
        fb_draw_string(4, y, "   TX STOPPED   ", COLOR_GRAY);
    }
    y += LINE_H + 6;

    /* 实时位置 */
    snprintf(buf, sizeof(buf), "Lat: %.6f", s_sim_info.sim_lat);
    fb_draw_string(4, y, buf, COLOR_CYAN);
    y += LINE_H + 2;

    snprintf(buf, sizeof(buf), "Lon: %.6f", s_sim_info.sim_lon);
    fb_draw_string(4, y, buf, COLOR_CYAN);
    y += LINE_H + 2;

    /* 航向 */
    snprintf(buf, sizeof(buf), "Hdg: %.1f deg", s_sim_info.sim_heading);
    fb_draw_string(4, y, buf, COLOR_YELLOW);
    y += LINE_H + 2;

    /* 高度 */
    snprintf(buf, sizeof(buf), "Alt: %.1f m", s_sim_info.sim_alt);
    fb_draw_string(4, y, buf, COLOR_GREEN);
    y += LINE_H + 2;

    /* 信道 */
    snprintf(buf, sizeof(buf), "CH: %d", s_sim_info.sim_channel);
    fb_draw_string(4, y, buf, COLOR_WHITE);
    y += LINE_H + 2;

    /* 发射计数 */
    snprintf(buf, sizeof(buf), "TX: %lu frames", (unsigned long)s_sim_info.sim_tx_count);
    fb_draw_string(4, y, buf, COLOR_ORANGE);
    y += LINE_H + 2;

    /* MAC */
    snprintf(buf, sizeof(buf), "MAC: FA:0B:BC:xx:xx:xx");
    fb_draw_string(4, y, buf, COLOR_GRAY);
    y += LINE_H + 6;

    /* 分割线 */
    fb_draw_hline(4, y, LCD_WIDTH - 8, COLOR_GRAY);
    y += 6;
    fb_draw_string(4, y, "OUI: FA:0B:BC", COLOR_GRAY);
    y += LINE_H;
    fb_draw_string(4, y, "Type: 0x0D (GB42590)", COLOR_GRAY);
}

/* ================================================================
 * 刷新任务
 * ================================================================ */
static void render_pair_overlay(void);
static void refresh_task(void *arg)
{
    static uint32_t frame_count = 0;

    while (1) {
        /* 清屏 */
        fb_fill(COLOR_BLACK);

        /* 获取扫描计数和当前 WiFi 信道 */
        uint32_t scan_count = 0;
        int wifi_channel = 1;
        if (s_tracker && s_tracker_mutex) {
            xSemaphoreTake(s_tracker_mutex, portMAX_DELAY);
            for (int i = 0; i < s_max_uavs; i++) {
                if (s_tracker[i].active) scan_count++;
            }
            xSemaphoreGive(s_tracker_mutex);
        }
        /* [Bug EE 修复] sniffer 锁定信道使用 FIXED_CHANNEL，不再硬编码 */
        wifi_channel = FIXED_CHANNEL;

        /* 渲染状态栏 */
        render_status_bar(scan_count, wifi_channel);

        /* 渲染内容区 */
        switch (s_page) {
            case LCD_PAGE_HOME:       render_home_page(); break;
            case LCD_PAGE_LIST:       render_list_page(); break;
            case LCD_PAGE_DETAIL:     render_detail_page(); break;
            case LCD_PAGE_SIM_CONFIG: render_sim_config_page(); break;
            case LCD_PAGE_SIM_STATUS: render_sim_status_page(); break;
            default: break;
        }

        /* 渲染底部栏 */
        int total_active = 0;
        if (s_tracker && s_tracker_mutex) {
            xSemaphoreTake(s_tracker_mutex, portMAX_DELAY);
            for (int i = 0; i < s_max_uavs; i++) {
                if (s_tracker[i].active) total_active++;
            }
            xSemaphoreGive(s_tracker_mutex);
        }
        render_footer(s_page, s_selection, total_active);

        /* 推送到屏幕 */
        render_pair_overlay();
        flush_framebuffer();

        frame_count++;

        /* 处理按键队列 */
        lcd_key_event_t key;
        while (xQueueReceive(s_key_queue, &key, 0) == pdTRUE) {
            switch (key) {
                case LCD_KEY_PREV:
                    if (s_page > LCD_PAGE_HOME) {
                        s_page--;
                        s_scroll_offset = 0;
                    }
                    break;
                case LCD_KEY_NEXT:
                    if (s_page < LCD_PAGE_COUNT - 1) {
                        s_page++;
                        s_scroll_offset = 0;
                    }
                    break;
                case LCD_KEY_SELECT:
                    if (s_page == LCD_PAGE_LIST) {
                        s_page = LCD_PAGE_DETAIL;
                    } else if (s_page == LCD_PAGE_DETAIL) {
                        s_page = LCD_PAGE_LIST;
                    }
                    /* Boot 键在 SIM_CONFIG 页：循环切换飞行模式（由外部回调处理） */
                    break;
                case LCD_KEY_BACK:
                    if (s_page != LCD_PAGE_HOME) {
                        s_page--;
                        s_scroll_offset = 0;
                    }
                    break;
                default:
                    break;
            }
        }

        /* 5fps = 200ms */
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/* ================================================================
 * 按键轮询任务（替代中断，更稳定）
 * ================================================================ */
static void button_poll_task(void *arg)
{
    /* [Bug GG 修复] 两个按键使用独立的时间戳，避免按一个键后另一个键被屏蔽 300ms */
    static uint32_t last_user_time = 0;
    static uint32_t last_boot_time = 0;
    const uint32_t debounce_ms = 300;

    while (1) {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

        /* User 按键（GPIO0）= 下一页 */
        if (gpio_get_level(BTN_USER_PIN) == 0 && (now - last_user_time > debounce_ms)) {
            lcd_display_send_key(LCD_KEY_NEXT);
            last_user_time = now;
        }

        /* Boot 按键（GPIO28）= 选择/确认 */
        if (gpio_get_level(BTN_BOOT_PIN) == 0 && (now - last_boot_time > debounce_ms)) {
            lcd_display_send_key(LCD_KEY_SELECT);
            last_boot_time = now;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ================================================================
 * 公开 API
 * ================================================================ */

int lcd_display_init(void)
{
    ESP_LOGI(TAG, "=== T-Display-C5 LCD 模块初始化 ===");

    /* 初始化帧缓冲 */
    if (init_framebuffer() != 0) return -1;

    /* 初始化 LCD */
    if (init_lcd() != 0) return -1;

    /* 初始化背光 */
    init_backlight();

    /* 开机纯色填充测试：确认 SPI 链路和 ST7789 初始化正常 */
    for (int i = 0; i < LCD_WIDTH * LCD_HEIGHT; i++) {
        s_fb[i] = rgb565(0, 30, 60);  // 深蓝色
    }
    flush_framebuffer();
    ESP_LOGI(TAG, "开机纯色测试已发送 (深蓝, fb_in_psram=%d)", s_fb_in_psram);

    /* 初始化按键（User + Boot 两个物理按键） */
    gpio_config_t btn_user_cfg = {
        .pin_bit_mask = (1ULL << BTN_USER_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn_user_cfg);

    gpio_config_t btn_boot_cfg = {
        .pin_bit_mask = (1ULL << BTN_BOOT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn_boot_cfg);

    /* 创建按键队列 */
    s_key_queue = xQueueCreate(8, sizeof(lcd_key_event_t));
    if (!s_key_queue) {
        ESP_LOGE(TAG, "按键队列创建失败");
        return -1;
    }

    /* 创建刷新任务 */
    xTaskCreatePinnedToCore(refresh_task, "lcd_refresh", 4096, NULL, 5, &s_refresh_task, 0);

    /* 创建按键轮询任务 */
    xTaskCreatePinnedToCore(button_poll_task, "btn_poll", 2048, NULL, 6, NULL, 0);

    ESP_LOGI(TAG, "=== LCD 模块就绪 ===");
    return 0;
}

void lcd_display_set_source(uav_track_t *tracker_table, void *tracker_mutex, int max_uavs)
{
    s_tracker = tracker_table;
    s_tracker_mutex = tracker_mutex;
    s_max_uavs = max_uavs;
    ESP_LOGI(TAG, "数据源设置: max_uavs=%d", max_uavs);
}

void lcd_display_send_key(lcd_key_event_t key)
{
    if (s_key_queue) {
        xQueueSend(s_key_queue, &key, 0);
    }
}

void lcd_display_set_page(lcd_page_t page)
{
    if (page < LCD_PAGE_COUNT) {
        s_page = page;
        s_scroll_offset = 0;
    }
}

lcd_page_t lcd_display_get_page(void)
{
    return s_page;
}

void lcd_display_set_selection(int index)
{
    s_selection = index;
}

int lcd_display_get_selection(void)
{
    return s_selection;
}

void lcd_display_command(const char *cmd)
{
    ESP_LOGI(TAG, "CMD: %s", cmd);
    /* 扩展用：可通过字符串命令控制显示 */
}

int lcd_display_get_battery_voltage(uint16_t *voltage_mv)
{
    /* AXP2602 读取 - 通过 I2C */
    /* 简化实现：后续可扩展完整 I2C 读取 */
    if (voltage_mv) *voltage_mv = 0;
    return -1;  // 暂未实现，需要 AXP2602 驱动
}

/* ================================================================
 * BLE 配对码显示
 * ================================================================ */
static char s_pair_pin[8] = {0};
static uint32_t s_pair_show_time = 0;

void lcd_display_show_pair_pin(const char *pin_code) {
    if (pin_code) {
        snprintf(s_pair_pin, sizeof(s_pair_pin), "%s", pin_code);
        s_pair_show_time = (uint32_t)(esp_timer_get_time() / 1000);  // ms
        ESP_LOGI(TAG, "Pair PIN displayed: %s", pin_code);
    }
}

static void render_pair_overlay(void) {
    /* 如果配对码显示超过60秒，自动清除 */
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    if (s_pair_pin[0] == '\0' || s_pair_show_time == 0) return;
    if (now_ms - s_pair_show_time > 60000) {
        s_pair_pin[0] = '\0';
        s_pair_show_time = 0;
        return;
    }

    /* 半透明遮罩效果（深色背景框） */
    int box_x = 20, box_y = 100, box_w = 130, box_h = 80;
    fb_fill_rect(box_x, box_y, box_w, box_h, rgb565(10, 10, 40));
    fb_draw_rect(box_x, box_y, box_w, box_h, COLOR_CYAN);

    /* 标题 */
    fb_draw_string(box_x + 16, box_y + 8, "BLE PAIR", COLOR_CYAN);

    /* PIN 码大字显示 */
    fb_draw_string(box_x + 24, box_y + 30, s_pair_pin, COLOR_WHITE);

    /* 提示 */
    fb_draw_string(box_x + 6, box_y + 55, "Enter on phone", COLOR_GRAY);
}
