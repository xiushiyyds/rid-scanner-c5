/**
 * lcd_display.c — ST7789 LCD 显示模块（T-Display-C5 中文 UI 版）
 *
 * 硬件：LILYGO T-Display-C5 (1.9" ST7789, 170×320, RGB565, SPI)
 * 字体：16×16 CJK + 8×16 ASCII 混合排版
 * UI 风格：深色主题，青绿标签，卡片式布局，信号条形图
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "lcd_display.h"
#include "lcd_font.h"
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
#include "driver/ledc.h"
#include "esp_cache.h"

#ifndef FIXED_CHANNEL
#define FIXED_CHANNEL 6
#endif

static const char *TAG = "lcd_display";

/* ================================================================
 * 模拟器状态
 * ================================================================ */
static sim_display_info_t s_sim_info = {0};

void lcd_display_set_sim_info(const sim_display_info_t *info) {
    if (info) memcpy(&s_sim_info, info, sizeof(sim_display_info_t));
}

/* ================================================================
 * 内部状态
 * ================================================================ */
static esp_lcd_panel_handle_t s_panel = NULL;
static uint16_t *s_fb = NULL;           /* 渲染帧缓冲（PSRAM，不做DMA） */
static uint16_t *s_dma_buf = NULL;      /* DMA bounce buffer（内部SRAM） */
static int s_dma_lines = 40;            /* 每次DMA传输的行数 */
static TaskHandle_t s_refresh_task = NULL;
static QueueHandle_t s_key_queue = NULL;

static uav_track_t *s_tracker = NULL;
static void *s_tracker_mutex = NULL;
static int s_max_uavs = 0;

static volatile lcd_page_t s_page = LCD_PAGE_HOME;
static volatile int s_selection = 0;
static volatile int s_scroll_offset = 0;

/* ================================================================
 * 配色方案（深色主题）
 * ================================================================ */
#define C_BG        0x0000   /* 纯黑背景 */
#define C_CARD      0x10A2   /* 深灰蓝卡片底 0x08,0x10,0x20 */
#define C_WHITE     0xFFFF
#define C_GREEN     0x07E0
#define C_CYAN      0x07FF
#define C_YELLOW    0xFFE0
#define C_ORANGE    0xFD20
#define C_RED       0xF800
#define C_BLUE      0x441F   /* 深蓝 0x42,0x80,0xFF */
#define C_DARKGREEN 0x03E0
#define C_GRAY      0x8410
#define C_LTGRAY    0xC618
#define C_PURPLE    0x8010
#define C_TEAL      0x0410   /* 青绿标签色 0x00,0x80,0x80 */
#define C_AMBER     0xFD20
#define C_DIM       0x4208   /* 暗灰 */

/* 状态栏/底栏高度 */
#define STATUSBAR_H   22
#define FOOTER_H      18
#define CONTENT_Y0    STATUSBAR_H
#define CONTENT_Y1    (LCD_HEIGHT - FOOTER_H)
#define CONTENT_H     (CONTENT_Y1 - CONTENT_Y0)

/* ================================================================
 * SPI + LCD 初始化
 * ================================================================ */
static int init_lcd(void)
{
    ESP_LOGI(TAG, "初始化 SPI 总线");

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = LCD_PIN_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = LCD_PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 170 * 40 * 2 + 8,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    /* 硬件复位 */
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

    esp_lcd_panel_io_handle_t io_handle = NULL;
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

    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = -1,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .data_endian = LCD_RGB_DATA_ENDIAN_LITTLE,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_cfg, &s_panel));

    esp_lcd_panel_init(s_panel);
    /* 与 LILYGO 官方 lcd.ino 一致：竖屏 170×320，无需 swap_xy。
     * 真机实测需要 x 不镜像、y 镜像，才能正向显示。 */
    esp_lcd_panel_mirror(s_panel, false, true);
    esp_lcd_panel_invert_color(s_panel, true);
    esp_lcd_panel_set_gap(s_panel, 35, 0);
    esp_lcd_panel_disp_on_off(s_panel, true);

    ESP_LOGI(TAG, "LCD 初始化完成 %dx%d", LCD_WIDTH, LCD_HEIGHT);
    return 0;
}

static void init_backlight(void)
{
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
}

/* ================================================================
 * 帧缓冲
 * ================================================================ */
static int init_framebuffer(void)
{
    /* 主帧缓冲放 PSRAM（108KB，内部 SRAM 装不下），仅做 CPU 渲染，不用于 DMA */
    s_fb = heap_caps_malloc(LCD_FB_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_fb) {
        s_fb = heap_caps_malloc(LCD_FB_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
        ESP_LOGW(TAG, "PSRAM 不可用，帧缓冲退到 SRAM");
    }
    if (!s_fb) {
        ESP_LOGE(TAG, "帧缓冲分配失败!");
        return -1;
    }
    memset(s_fb, 0, LCD_FB_SIZE);

    /* DMA bounce buffer：内部 SRAM，每次拷 N 行再发，避开 PSRAM DMA 问题 */
    size_t bounce_size = LCD_WIDTH * s_dma_lines * 2;
    s_dma_buf = heap_caps_malloc(bounce_size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!s_dma_buf) {
        ESP_LOGE(TAG, "DMA bounce buffer 分配失败 (%d bytes)", bounce_size);
        return -1;
    }
    ESP_LOGI(TAG, "帧缓冲: PSRAM 渲染 + SRAM DMA bounce %d bytes (每次 %d 行)",
             bounce_size, s_dma_lines);
    return 0;
}

static void flush_framebuffer(void)
{
    if (!s_panel || !s_fb || !s_dma_buf) return;
    for (int y = 0; y < LCD_HEIGHT; y += s_dma_lines) {
        int h = (y + s_dma_lines > LCD_HEIGHT) ? LCD_HEIGHT - y : s_dma_lines;
        size_t len = LCD_WIDTH * h * 2;
        memcpy(s_dma_buf, &s_fb[y * LCD_WIDTH], len);
        esp_lcd_panel_draw_bitmap(s_panel, 0, y, LCD_WIDTH, y + h, s_dma_buf);
    }
}

/* ================================================================
 * 绘图原语
 * ================================================================ */
static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

static void fb_fill(uint16_t c) {
    for (int i = 0; i < LCD_WIDTH * LCD_HEIGHT; i++) s_fb[i] = c;
}

static void fb_setpixel(int x, int y, uint16_t c) {
    if (x >= 0 && x < LCD_WIDTH && y >= 0 && y < LCD_HEIGHT)
        s_fb[y * LCD_WIDTH + x] = c;
}

static void fb_hline(int x, int y, int w, uint16_t c) {
    if (y < 0 || y >= LCD_HEIGHT) return;
    if (x < 0) { w += x; x = 0; }
    if (x + w > LCD_WIDTH) w = LCD_WIDTH - x;
    for (int i = 0; i < w; i++) s_fb[y * LCD_WIDTH + x + i] = c;
}

static void fb_vline(int x, int y, int h, uint16_t c) {
    if (x < 0 || x >= LCD_WIDTH) return;
    if (y < 0) { h += y; y = 0; }
    if (y + h > LCD_HEIGHT) h = LCD_HEIGHT - y;
    for (int i = 0; i < h; i++) s_fb[(y + i) * LCD_WIDTH + x] = c;
}

static void fb_fillrect(int x, int y, int w, int h, uint16_t c) {
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > LCD_WIDTH) w = LCD_WIDTH - x;
    if (y + h > LCD_HEIGHT) h = LCD_HEIGHT - y;
    for (int r = y; r < y + h; r++)
        for (int col = x; col < x + w; col++)
            s_fb[r * LCD_WIDTH + col] = c;
}

static void fb_drawrect(int x, int y, int w, int h, uint16_t c) {
    fb_hline(x, y, w, c);
    fb_hline(x, y + h - 1, w, c);
    fb_vline(x, y, h, c);
    fb_vline(x + w - 1, y, h, c);
}

/* 绘制文本（封装字体引擎） */
static int fb_text(int x, int y, const char *s, uint16_t c) {
    return lcd_font_draw_text(s_fb, LCD_WIDTH, LCD_HEIGHT, x, y, s, c);
}

static int fb_text_center(int y, const char *s, uint16_t c) {
    return lcd_font_draw_text_centered(s_fb, LCD_WIDTH, LCD_HEIGHT, y, s, c);
}

static int fb_text_right(int x_right, int y, const char *s, uint16_t c) {
    return lcd_font_draw_text_right(s_fb, LCD_WIDTH, LCD_HEIGHT, x_right, y, s, c);
}

/* ================================================================
 * 信号强度条形图
 * ================================================================ */
static void draw_signal_bars(int x, int y, int rssi, int max_bars)
{
    /* RSSI 范围 -100 (无信号) 到 -30 (满格)，映射到 0~max_bars */
    int level = 0;
    if (rssi >= -30) level = max_bars;
    else if (rssi > -100) level = (rssi + 100) * max_bars / 70;
    if (level < 0) level = 0;
    if (level > max_bars) level = max_bars;

    int bar_w = 4;
    int gap = 2;
    for (int i = 0; i < max_bars; i++) {
        int bh = 3 + i * 3;  /* 递增高度 */
        int by = y + (max_bars * 3) - bh;
        uint16_t color = (i < level) ?
            (level >= max_bars - 1 ? C_GREEN : (level >= max_bars / 2 ? C_YELLOW : C_RED))
            : C_DIM;
        fb_fillrect(x + i * (bar_w + gap), by, bar_w, bh, color);
    }
}

/* 协议类型转中文标签 */
static const char *protocol_label(uint8_t proto)
{
    switch (proto) {
        case RID_PROTOCOL_ASTM_F3411: return "ASTM";
        case RID_PROTOCOL_ASD_STAN:   return "欧盟";
        case RID_PROTOCOL_GB42590:    return "国标";
        case RID_PROTOCOL_GB46750:    return "国标";
        default:                      return "未知";
    }
}

static uint16_t protocol_color(uint8_t proto)
{
    switch (proto) {
        case RID_PROTOCOL_ASTM_F3411: return C_CYAN;
        case RID_PROTOCOL_ASD_STAN:   return C_PURPLE;
        case RID_PROTOCOL_GB42590:
        case RID_PROTOCOL_GB46750:    return C_GREEN;
        default:                      return C_GRAY;
    }
}

/* ================================================================
 * 状态栏
 * ================================================================ */
static void render_statusbar(int active_count, int channel)
{
    fb_fillrect(0, 0, LCD_WIDTH, STATUSBAR_H, C_BLUE);
    /* 左侧标题 */
    fb_text(3, 3, "无人机侦测", C_WHITE);
    /* 右侧信道+计数 */
    char buf[24];
    snprintf(buf, sizeof(buf), "CH%d", channel);
    int tw = lcd_font_text_width(buf);
    fb_text(LCD_WIDTH - tw - 3, 3, buf, C_CYAN);
}

/* ================================================================
 * 底栏
 * ================================================================ */
static void render_footer(lcd_page_t page)
{
    fb_fillrect(0, CONTENT_Y1, LCD_WIDTH, FOOTER_H, rgb565(20, 30, 20));
    const char *hints[] = {
        "User:翻页  Boot:选择",   /* HOME */
        "User:翻页  Boot:详情",   /* LIST */
        "User:翻页  Boot:返回",   /* DETAIL */
        "User:翻页  Boot:切换",   /* SIM_CFG */
        "User:翻页  Boot:停止",   /* SIM_STATUS */
    };
    int idx = (int)page;
    if (idx < 0 || idx >= 5) idx = 0;
    fb_text(2, CONTENT_Y1 + 1, hints[idx], C_LTGRAY);
}

/* ================================================================
 * 主页
 * ================================================================ */
static void render_home(void)
{
    int active = 0, gb = 0, astm = 0, asd = 0, dji = 0;

    if (s_tracker && s_tracker_mutex) {
        xSemaphoreTake(s_tracker_mutex, portMAX_DELAY);
        for (int i = 0; i < s_max_uavs; i++) {
            if (s_tracker[i].active) {
                active++;
                if (s_tracker[i].is_dji) dji++;
                switch (s_tracker[i].protocol) {
                    case RID_PROTOCOL_ASTM_F3411: astm++; break;
                    case RID_PROTOCOL_ASD_STAN:   asd++; break;
                    case RID_PROTOCOL_GB42590:
                    case RID_PROTOCOL_GB46750:    gb++; break;
                    default: break;
                }
            }
        }
        xSemaphoreGive(s_tracker_mutex);
    }

    int y = CONTENT_Y0 + 8;

    /* 大数字：活跃目标数 */
    char num[8];
    snprintf(num, sizeof(num), "%d", active);
    int num_w = lcd_font_text_width(num) * 2;
    /* 简化：画正常大小数字加标签 */
    fb_drawrect(8, y, LCD_WIDTH - 16, 50, C_TEAL);
    fb_fillrect(8, y, LCD_WIDTH - 16, 50, rgb565(5, 20, 30));

    fb_text(16, y + 6, "活跃目标", C_CYAN);
    fb_text_right(LCD_WIDTH - 16, y + 6, num, active > 0 ? C_GREEN : C_GRAY);
    fb_text(16, y + 28, active > 0 ? "● 监测中" : "○ 未发现",
            active > 0 ? C_GREEN : C_GRAY);

    y += 62;

    /* 协议统计卡片 */
    fb_text(8, y, "协议分类", C_YELLOW);
    y += FONT_LINE_H + 2;
    fb_hline(8, y, LCD_WIDTH - 16, C_DIM);
    y += 4;

    char buf[24];
    snprintf(buf, sizeof(buf), "国标  %d", gb);
    fb_text(12, y, buf, C_GREEN);
    snprintf(buf, sizeof(buf), "%d", gb);
    fb_text_right(LCD_WIDTH - 12, y, buf, C_GREEN);
    y += FONT_LINE_H;

    snprintf(buf, sizeof(buf), "ASTM  %d", astm);
    fb_text(12, y, buf, C_CYAN);
    fb_text_right(LCD_WIDTH - 12, y, buf + 6, C_CYAN);
    y += FONT_LINE_H;

    snprintf(buf, sizeof(buf), "欧盟  %d", asd);
    fb_text(12, y, buf, C_PURPLE);
    fb_text_right(LCD_WIDTH - 12, y, buf + 6, C_PURPLE);
    y += FONT_LINE_H;

    snprintf(buf, sizeof(buf), "大疆  %d", dji);
    fb_text(12, y, buf, C_ORANGE);
    fb_text_right(LCD_WIDTH - 12, y, buf + 6, C_ORANGE);
    y += FONT_LINE_H + 8;

    fb_hline(8, y, LCD_WIDTH - 16, C_DIM);
    y += 8;

    /* 模拟器状态 */
    if (s_sim_info.is_sim_running) {
        fb_text(8, y, "● 模拟发射中", C_GREEN);
        char cnt[16];
        snprintf(cnt, sizeof(cnt), "%lu帧", (unsigned long)s_sim_info.sim_tx_count);
        fb_text_right(LCD_WIDTH - 8, y, cnt, C_GREEN);
    } else {
        fb_text(8, y, "○ 模拟器就绪", C_GRAY);
    }
}

/* ================================================================
 * 列表页
 * ================================================================ */
static void render_list(void)
{
    if (!s_tracker || !s_tracker_mutex) return;

    int indices[32];
    int count = 0;

    xSemaphoreTake(s_tracker_mutex, portMAX_DELAY);
    for (int i = 0; i < s_max_uavs && count < 32; i++) {
        if (s_tracker[i].active) indices[count++] = i;
    }
    xSemaphoreGive(s_tracker_mutex);

    if (count == 0) {
        fb_text_center(CONTENT_Y0 + 60, "未发现目标", C_GRAY);
        fb_text_center(CONTENT_Y0 + 84, "请按User翻页", C_DIM);
        return;
    }

    if (s_selection >= count) s_selection = count - 1;
    if (s_selection < 0) s_selection = 0;

    int item_h = 58;
    int visible = CONTENT_H / item_h;
    if (s_selection < s_scroll_offset) s_scroll_offset = s_selection;
    if (s_selection >= s_scroll_offset + visible) s_scroll_offset = s_selection - visible + 1;
    if (s_scroll_offset > count - visible) s_scroll_offset = count - visible;
    if (s_scroll_offset < 0) s_scroll_offset = 0;

    int y = CONTENT_Y0 + 2;
    for (int idx = s_scroll_offset; idx < s_scroll_offset + visible && idx < count; idx++) {
        int i = indices[idx];
        bool sel = (idx == s_selection);

        /* 卡片背景 */
        uint16_t card_bg = sel ? rgb565(10, 40, 70) : C_CARD;
        fb_fillrect(2, y, LCD_WIDTH - 4, item_h - 4, card_bg);
        if (sel) fb_drawrect(2, y, LCD_WIDTH - 4, item_h - 4, C_CYAN);

        int iy = y + 3;

        /* 第一行：编号 + 型号/ID */
        char buf[64];
        if (s_tracker[i].is_dji && s_tracker[i].dji_model[0]) {
            snprintf(buf, sizeof(buf), "%d.%s", idx + 1, s_tracker[i].dji_model);
        } else {
            const char *id = s_tracker[i].basic_id.uas_id[0] ?
                s_tracker[i].basic_id.uas_id : "----";
            snprintf(buf, sizeof(buf), "%d.%s", idx + 1, id);
        }
        fb_text(5, iy, buf, sel ? C_WHITE : C_GREEN);
        iy += FONT_LINE_H;

        /* 第二行：信号强度条 + RSSI 数值 + 协议标签 */
        draw_signal_bars(5, iy + 2, s_tracker[i].last_rssi, 5);
        snprintf(buf, sizeof(buf), "%ddBm", s_tracker[i].last_rssi);
        fb_text(45, iy, buf, C_CYAN);

        const char *plabel = s_tracker[i].is_dji ? "DJI" : protocol_label(s_tracker[i].protocol);
        uint16_t pcolor = s_tracker[i].is_dji ? C_ORANGE : protocol_color(s_tracker[i].protocol);
        fb_text_right(LCD_WIDTH - 5, iy, plabel, pcolor);
        iy += FONT_LINE_H + 2;

        /* 第三行：高度 + 速度 */
        if (s_tracker[i].is_dji) {
            snprintf(buf, sizeof(buf), "高%d.%dm",
                     (int)s_tracker[i].dji_altitude,
                     (int)((s_tracker[i].dji_altitude - (int)s_tracker[i].dji_altitude) * 10));
        } else if ((int)s_tracker[i].location.height != 0) {
            snprintf(buf, sizeof(buf), "高%dm", (int)s_tracker[i].location.height);
        } else {
            snprintf(buf, sizeof(buf), "高--m");
        }
        fb_text(5, iy, buf, C_YELLOW);

        if (s_tracker[i].is_dji) {
            snprintf(buf, sizeof(buf), "速%.1fm/s", s_tracker[i].dji_speed_h);
        } else {
            snprintf(buf, sizeof(buf), "速%dm/s", (int)s_tracker[i].location.speed_horizontal);
        }
        fb_text_right(LCD_WIDTH - 5, iy, buf, C_YELLOW);

        y += item_h;
    }

    /* 滚动条 */
    if (count > visible) {
        int bar_h = CONTENT_H * visible / count;
        int bar_y = CONTENT_Y0 + CONTENT_H * s_scroll_offset / count;
        fb_fillrect(LCD_WIDTH - 2, bar_y, 2, bar_h, C_CYAN);
    }
}

/* ================================================================
 * 详情页
 * ================================================================ */
static void render_detail(void)
{
    if (!s_tracker || !s_tracker_mutex) return;

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
        fb_text_center(CONTENT_Y0 + 60, "未选中目标", C_GRAY);
        return;
    }

    int y = CONTENT_Y0 + 4;
    char buf[48];

    /* 标题区：型号/ID + 信号条 */
    if (s_tracker[idx].is_dji) {
        fb_fillrect(0, y - 2, LCD_WIDTH, FONT_LINE_H + 6, rgb565(60, 30, 0));
        const char *model = s_tracker[idx].dji_model[0] ?
            s_tracker[idx].dji_model : "DJI Drone";
        fb_text(4, y, model, C_ORANGE);
    } else {
        fb_fillrect(0, y - 2, LCD_WIDTH, FONT_LINE_H + 6, rgb565(0, 30, 60));
        fb_text(4, y, "目标详情", C_CYAN);
    }
    y += FONT_LINE_H + 8;

    /* ID */
    if (s_tracker[idx].is_dji && s_tracker[idx].dji_serial[0]) {
        fb_text(4, y, "SN", C_GREEN);
        fb_text(36, y, s_tracker[idx].dji_serial, C_WHITE);
    } else if (s_tracker[idx].basic_id.uas_id[0]) {
        fb_text(4, y, "ID", C_GREEN);
        fb_text(36, y, s_tracker[idx].basic_id.uas_id, C_WHITE);
    }
    y += FONT_LINE_H + 2;

    /* 信号 */
    fb_text(4, y, "信号", C_GREEN);
    draw_signal_bars(38, y + 2, s_tracker[idx].last_rssi, 5);
    snprintf(buf, sizeof(buf), "%ddBm", s_tracker[idx].last_rssi);
    fb_text(78, y, buf, C_WHITE);
    y += FONT_LINE_H + 4;

    fb_hline(4, y, LCD_WIDTH - 8, C_DIM);
    y += 4;

    /* 位置信息 */
    if (s_tracker[idx].is_dji) {
        snprintf(buf, sizeof(buf), "纬度  %.6f", s_tracker[idx].dji_latitude);
        fb_text(4, y, buf, C_CYAN); y += FONT_LINE_H;
        snprintf(buf, sizeof(buf), "经度  %.6f", s_tracker[idx].dji_longitude);
        fb_text(4, y, buf, C_CYAN); y += FONT_LINE_H;
        snprintf(buf, sizeof(buf), "高度  %.1fm", s_tracker[idx].dji_altitude);
        fb_text(4, y, buf, C_YELLOW); y += FONT_LINE_H;
        snprintf(buf, sizeof(buf), "速度  %.1fm/s", s_tracker[idx].dji_speed_h);
        fb_text(4, y, buf, C_YELLOW); y += FONT_LINE_H;
        if (s_tracker[idx].dji_heading >= 0) {
            snprintf(buf, sizeof(buf), "航向  %.1f°", s_tracker[idx].dji_heading);
            fb_text(4, y, buf, C_YELLOW); y += FONT_LINE_H;
        }
    } else {
        if (s_tracker[idx].location.latitude != 0) {
            snprintf(buf, sizeof(buf), "纬度  %.6f", s_tracker[idx].location.latitude);
            fb_text(4, y, buf, C_CYAN); y += FONT_LINE_H;
            snprintf(buf, sizeof(buf), "经度  %.6f", s_tracker[idx].location.longitude);
            fb_text(4, y, buf, C_CYAN); y += FONT_LINE_H;
        }
        if ((int)s_tracker[idx].location.height != 0) {
            snprintf(buf, sizeof(buf), "高度  %dm", (int)s_tracker[idx].location.height);
            fb_text(4, y, buf, C_YELLOW); y += FONT_LINE_H;
        }
        if (s_tracker[idx].location.speed_horizontal > 0) {
            snprintf(buf, sizeof(buf), "速度  %.1fm/s", s_tracker[idx].location.speed_horizontal);
            fb_text(4, y, buf, C_YELLOW); y += FONT_LINE_H;
        }
        if (s_tracker[idx].location.direction != 0) {
            snprintf(buf, sizeof(buf), "航向  %.1f°", s_tracker[idx].location.direction);
            fb_text(4, y, buf, C_YELLOW); y += FONT_LINE_H;
        }
    }

    y += 2;
    fb_hline(4, y, LCD_WIDTH - 8, C_DIM);
    y += 4;

    /* MAC */
    snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
             s_tracker[idx].mac[0], s_tracker[idx].mac[1],
             s_tracker[idx].mac[2], s_tracker[idx].mac[3],
             s_tracker[idx].mac[4], s_tracker[idx].mac[5]);
    fb_text(4, y, "MAC", C_GREEN);
    fb_text(36, y, buf, C_GRAY);
    y += FONT_LINE_H;

    /* 信道 */
    snprintf(buf, sizeof(buf), "信道  %d", s_tracker[idx].last_channel);
    fb_text(4, y, buf, C_GREEN);
    y += FONT_LINE_H;

    /* 最后出现 */
    if (s_tracker[idx].last_seen_ms > 0) {
        uint32_t ago = (uint32_t)(esp_timer_get_time() / 1000 - s_tracker[idx].last_seen_ms) / 1000;
        if (ago < 60)
            snprintf(buf, sizeof(buf), "%lu秒前", (unsigned long)ago);
        else
            snprintf(buf, sizeof(buf), "%lu分%lu秒前",
                     (unsigned long)(ago / 60), (unsigned long)(ago % 60));
        fb_text(4, y, "更新", C_GREEN);
        fb_text(36, y, buf, C_LTGRAY);
    }
}

/* ================================================================
 * 模拟配置页
 * ================================================================ */
static void render_simconfig(void)
{
    int y = CONTENT_Y0 + 4;
    char buf[48];

    fb_fillrect(0, y - 2, LCD_WIDTH, FONT_LINE_H + 4, rgb565(60, 0, 30));
    fb_text_center(y, "模拟设置", C_WHITE);
    y += FONT_LINE_H + 8;

    fb_text(4, y, "状态", C_GREEN);
    fb_text_right(LCD_WIDTH - 4, y,
        s_sim_info.is_sim_running ? "运行中" : "已停止",
        s_sim_info.is_sim_running ? C_GREEN : C_GRAY);
    y += FONT_LINE_H + 2;

    fb_text(4, y, "模式", C_GREEN);
    const char *modes[] = {"圆周", "往返", "搜索"};
    int mi = s_sim_info.sim_flight_mode;
    if (mi < 0 || mi >= 3) mi = 0;
    fb_text_right(LCD_WIDTH - 4, y, modes[mi], C_YELLOW);
    y += FONT_LINE_H + 2;

    snprintf(buf, sizeof(buf), "信道  %d", s_sim_info.sim_channel);
    fb_text(4, y, buf, C_CYAN);
    y += FONT_LINE_H + 2;

    fb_text(4, y, "ID", C_GREEN);
    fb_text(36, y,
        s_sim_info.sim_uas_id[0] ? s_sim_info.sim_uas_id : "SIM-C5",
        C_WHITE);
    y += FONT_LINE_H + 8;

    fb_hline(4, y, LCD_WIDTH - 8, C_DIM);
    y += 6;

    snprintf(buf, sizeof(buf), "纬度  %.4f", s_sim_info.sim_lat);
    fb_text(4, y, buf, C_GREEN); y += FONT_LINE_H;
    snprintf(buf, sizeof(buf), "经度  %.4f", s_sim_info.sim_lon);
    fb_text(4, y, buf, C_GREEN); y += FONT_LINE_H + 8;

    fb_hline(4, y, LCD_WIDTH - 8, C_DIM);
    y += 6;
    fb_text(4, y, "Boot键切换模式", C_ORANGE);
}

/* ================================================================
 * 模拟状态页
 * ================================================================ */
static void render_simstatus(void)
{
    int y = CONTENT_Y0 + 4;
    char buf[48];

    fb_fillrect(0, y - 2, LCD_WIDTH, FONT_LINE_H + 4, rgb565(0, 50, 25));
    fb_text_center(y, "模拟状态", C_WHITE);
    y += FONT_LINE_H + 8;

    if (s_sim_info.is_sim_running) {
        fb_text(4, y, "● 发射中", C_GREEN);
    } else {
        fb_text(4, y, "○ 已停止", C_GRAY);
    }
    y += FONT_LINE_H + 6;

    snprintf(buf, sizeof(buf), "纬度  %.6f", s_sim_info.sim_lat);
    fb_text(4, y, buf, C_CYAN); y += FONT_LINE_H;
    snprintf(buf, sizeof(buf), "经度  %.6f", s_sim_info.sim_lon);
    fb_text(4, y, buf, C_CYAN); y += FONT_LINE_H;
    snprintf(buf, sizeof(buf), "航向  %.1f°", s_sim_info.sim_heading);
    fb_text(4, y, buf, C_YELLOW); y += FONT_LINE_H;
    snprintf(buf, sizeof(buf), "高度  %.1fm", s_sim_info.sim_alt);
    fb_text(4, y, buf, C_GREEN); y += FONT_LINE_H;
    snprintf(buf, sizeof(buf), "信道  %d", s_sim_info.sim_channel);
    fb_text(4, y, buf, C_WHITE); y += FONT_LINE_H + 4;

    fb_hline(4, y, LCD_WIDTH - 8, C_DIM);
    y += 6;
    snprintf(buf, sizeof(buf), "已发射  %lu帧", (unsigned long)s_sim_info.sim_tx_count);
    fb_text(4, y, buf, C_ORANGE);
}

/* ================================================================
 * BLE 配对码覆盖层
 * ================================================================ */
static char s_pin[8] = {0};
static uint32_t s_pin_time = 0;

void lcd_display_show_pair_pin(const char *pin) {
    if (pin) {
        snprintf(s_pin, sizeof(s_pin), "%s", pin);
        s_pin_time = (uint32_t)(esp_timer_get_time() / 1000);
    }
}

static void render_pair_overlay(void)
{
    if (!s_pin[0] || !s_pin_time) return;
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    if (now - s_pin_time > 60000) {
        s_pin[0] = 0;
        s_pin_time = 0;
        return;
    }

    int bx = 15, by = 90, bw = LCD_WIDTH - 30, bh = 100;
    fb_fillrect(bx, by, bw, bh, rgb565(5, 10, 35));
    fb_drawrect(bx, by, bw, bh, C_CYAN);

    fb_text_center(by + 10, "蓝牙配对", C_CYAN);

    /* PIN 码用大号字（ASCII 8x16，放大2倍暂不支持，正常显示） */
    int pin_w = lcd_font_text_width(s_pin);
    lcd_font_draw_text(s_fb, LCD_WIDTH, LCD_HEIGHT,
                       (LCD_WIDTH - pin_w) / 2, by + 38, s_pin, C_WHITE);

    fb_text_center(by + 68, "请在手机输入", C_GRAY);
}

/* ================================================================
 * 刷新任务
 * ================================================================ */
static void refresh_task(void *arg)
{
    while (1) {
        fb_fill(C_BG);

        int active = 0;
        if (s_tracker && s_tracker_mutex) {
            xSemaphoreTake(s_tracker_mutex, portMAX_DELAY);
            for (int i = 0; i < s_max_uavs; i++)
                if (s_tracker[i].active) active++;
            xSemaphoreGive(s_tracker_mutex);
        }

        render_statusbar(active, FIXED_CHANNEL);

        switch (s_page) {
            case LCD_PAGE_HOME:       render_home(); break;
            case LCD_PAGE_LIST:       render_list(); break;
            case LCD_PAGE_DETAIL:     render_detail(); break;
            case LCD_PAGE_SIM_CONFIG: render_simconfig(); break;
            case LCD_PAGE_SIM_STATUS: render_simstatus(); break;
            default: break;
        }

        render_footer(s_page);
        render_pair_overlay();
        flush_framebuffer();

        /* 处理按键 */
        lcd_key_event_t key;
        while (xQueueReceive(s_key_queue, &key, 0) == pdTRUE) {
            switch (key) {
                case LCD_KEY_NEXT:
                    if (s_page < LCD_PAGE_COUNT - 1) {
                        s_page++;
                        s_scroll_offset = 0;
                    }
                    break;
                case LCD_KEY_PREV:
                case LCD_KEY_BACK:
                    if (s_page > LCD_PAGE_HOME) {
                        s_page--;
                        s_scroll_offset = 0;
                    }
                    break;
                case LCD_KEY_SELECT:
                    if (s_page == LCD_PAGE_LIST) {
                        s_page = LCD_PAGE_DETAIL;
                    } else if (s_page == LCD_PAGE_DETAIL) {
                        s_page = LCD_PAGE_LIST;
                    }
                    break;
                default: break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

/* ================================================================
 * 按键轮询
 * ================================================================ */
static void button_poll_task(void *arg)
{
    static uint32_t t_user = 0, t_boot = 0;
    const uint32_t debounce = 300;

    while (1) {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

        if (gpio_get_level(BTN_USER_PIN) == 0 && (now - t_user > debounce)) {
            lcd_display_send_key(LCD_KEY_NEXT);
            t_user = now;
        }
        if (gpio_get_level(BTN_BOOT_PIN) == 0 && (now - t_boot > debounce)) {
            lcd_display_send_key(LCD_KEY_SELECT);
            t_boot = now;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ================================================================
 * 公开 API
 * ================================================================ */
int lcd_display_init(void)
{
    ESP_LOGI(TAG, "=== LCD 模块初始化（中文UI）===");

    if (init_framebuffer() != 0) return -1;
    if (init_lcd() != 0) return -1;
    init_backlight();

    /* 开机纯色测试：整屏填充，用于快速判断屏幕方向/满屏是否正常 */
    fb_fill(rgb565(0, 20, 50));
    /* 四角画白色方块 + 屏幕中央画十字，方便肉眼确认满屏和方向 */
    fb_fillrect(0, 0, 20, 20, C_WHITE);             /* 左上 */
    fb_fillrect(LCD_WIDTH - 20, 0, 20, 20, C_WHITE);  /* 右上 */
    fb_fillrect(0, LCD_HEIGHT - 20, 20, 20, C_WHITE); /* 左下 */
    fb_fillrect(LCD_WIDTH - 20, LCD_HEIGHT - 20, 20, 20, C_WHITE); /* 右下 */
    fb_fillrect(LCD_WIDTH/2 - 1, 0, 2, LCD_HEIGHT, C_WHITE); /* 垂直中线 */
    fb_fillrect(0, LCD_HEIGHT/2 - 1, LCD_WIDTH, 2, C_WHITE); /* 水平中线 */
    flush_framebuffer();
    vTaskDelay(pdMS_TO_TICKS(1500));

    /* 开机 Logo */
    fb_fill(C_BG);
    fb_text_center(130, "无人机侦测器", C_CYAN);
    fb_text_center(154, "T-Display-C5", C_GRAY);
    flush_framebuffer();
    vTaskDelay(pdMS_TO_TICKS(1000));

    /* 按键 */
    gpio_config_t btn_cfg = {
        .pin_bit_mask = (1ULL << BTN_USER_PIN) | (1ULL << BTN_BOOT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn_cfg);

    s_key_queue = xQueueCreate(8, sizeof(lcd_key_event_t));
    if (!s_key_queue) return -1;

    xTaskCreatePinnedToCore(refresh_task, "lcd_refresh", 6144, NULL, 5, &s_refresh_task, 0);
    xTaskCreatePinnedToCore(button_poll_task, "btn_poll", 2048, NULL, 6, NULL, 0);

    ESP_LOGI(TAG, "=== LCD 就绪 ===");
    return 0;
}

void lcd_display_set_source(uav_track_t *t, void *m, int n) {
    s_tracker = t; s_tracker_mutex = m; s_max_uavs = n;
}

void lcd_display_send_key(lcd_key_event_t key) {
    if (s_key_queue) xQueueSend(s_key_queue, &key, 0);
}

void lcd_display_set_page(lcd_page_t p) {
    if (p < LCD_PAGE_COUNT) { s_page = p; s_scroll_offset = 0; }
}

lcd_page_t lcd_display_get_page(void) { return s_page; }
void lcd_display_set_selection(int i) { s_selection = i; }
int lcd_display_get_selection(void) { return s_selection; }
void lcd_display_command(const char *cmd) { (void)cmd; }

int lcd_display_get_battery_voltage(uint16_t *v) {
    if (v) *v = 0;
    return -1;
}
