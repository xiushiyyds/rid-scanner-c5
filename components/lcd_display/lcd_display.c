/**
 * lcd_display.c — ST7789 LCD 显示模块（T-Display-C5 中文 UI 版）
 *
 * lcdfix10：
 *   - DMA 传输完成信号量同步（on_color_trans_done），消除撕裂/杂线
 *   - LCD 初始化必须在 WiFi/BLE 之前，保证 108KB 内部 SRAM 可用
 *   - 修复列表 UAS ID / 详情 MAC 溢出截断
 *   - 字库补 "准"、几何符号区(●○★→等)
 *   - 详情页布局重排，信息完整显示
 *
 * 硬件：LILYGO T-Display-C5 (1.9" ST7789, 170×320, RGB565, SPI)
 * 字体：16×16 CJK + 8×16 ASCII 混合排版
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
    if (info) memcpy(&s_sim_info, info, sizeof(sim_info));
}

/* ================================================================
 * 内部状态
 * ================================================================ */
static esp_lcd_panel_handle_t s_panel = NULL;
static uint16_t *s_fb = NULL;
static uint16_t *s_dma_buf = NULL;
static bool s_use_full_dma = false;
static int s_dma_lines = 40;
static TaskHandle_t s_refresh_task = NULL;
static QueueHandle_t s_key_queue = NULL;

/* DMA 完成同步信号量：on_color_trans_done 回调中 give，
 * flush 结束前 take，确保 DMA 不读正在被重写的帧缓冲 */
static SemaphoreHandle_t s_dma_done_sem = NULL;

static uav_track_t *s_tracker = NULL;
static void *s_tracker_mutex = NULL;
static int s_max_uavs = 0;

static volatile lcd_page_t s_page = LCD_PAGE_HOME;
static volatile int s_selection = 0;
static volatile int s_scroll_offset = 0;

/* ================================================================
 * 配色方案（深色主题）
 * ================================================================ */
#define C_BG        0x0000
#define C_CARD      0x10A2
#define C_WHITE     0xFFFF
#define C_GREEN     0x07E0
#define C_CYAN      0x07FF
#define C_YELLOW    0xFFE0
#define C_ORANGE    0xFD20
#define C_RED       0xF800
#define C_BLUE      0x441F
#define C_DARKGREEN 0x03E0
#define C_GRAY      0x8410
#define C_LTGRAY    0xC618
#define C_PURPLE    0x8010
#define C_TEAL      0x0410
#define C_DIM       0x4208

/* 状态栏/底栏高度 */
#define STATUSBAR_H   22
#define FOOTER_H      18
#define CONTENT_Y0    STATUSBAR_H
#define CONTENT_Y1    (LCD_HEIGHT - FOOTER_H)
#define CONTENT_H     (CONTENT_Y1 - CONTENT_Y0)

#define SB_TEXT_Y     ((STATUSBAR_H - FONT_LINE_H) / 2)
#define FT_TEXT_Y     (CONTENT_Y1 + (FOOTER_H - FONT_LINE_H) / 2)

/* ================================================================
 * DMA 完成回调（ISR 安全）
 * ================================================================ */
static bool IRAM_ATTR lcd_color_trans_done_cb(esp_lcd_panel_io_handle_t panel_io,
                                               esp_lcd_panel_io_event_data_t *edata,
                                               void *user_ctx)
{
    BaseType_t higher_priority_task_wakeup = pdFALSE;
    if (s_dma_done_sem) {
        xSemaphoreGiveFromISR(s_dma_done_sem, &higher_priority_task_wakeup);
    }
    return higher_priority_task_wakeup == pdTRUE;
}

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
        .max_transfer_sz = LCD_FB_SIZE + 8,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    /* 硬件复位（与 LILYGO 官方时序一致） */
    gpio_config_t rst_conf = {
        .pin_bit_mask = (1ULL << LCD_PIN_RST),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&rst_conf);
    gpio_set_level(LCD_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(25));
    gpio_set_level(LCD_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(25));
    gpio_set_level(LCD_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(125));

    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_cfg = {
        .dc_gpio_num = LCD_PIN_DC,
        .cs_gpio_num = LCD_PIN_CS,
        .pclk_hz = LCD_SPI_FREQ_HZ,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = lcd_color_trans_done_cb,
        .user_ctx = NULL,
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

    /* 竖屏 170×320，set_gap(35,0)，与 LILYGO 官方一致。
     * mirror(true,true) = 双镜像 = 180° 旋转，真机验证文字正向+logo在顶 */
    esp_lcd_panel_mirror(s_panel, true, true);
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
    /* 创建 DMA 完成信号量 */
    s_dma_done_sem = xSemaphoreCreateBinary();
    if (!s_dma_done_sem) {
        ESP_LOGE(TAG, "DMA done semaphore creation failed");
        return -1;
    }

    /* 优先全屏内部 SRAM DMA（必须在 WiFi/BLE 初始化前调用） */
    s_fb = heap_caps_malloc(LCD_FB_SIZE,
                            MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (s_fb) {
        s_use_full_dma = true;
        ESP_LOGI(TAG, "帧缓冲: 全屏内部 SRAM DMA (%d bytes)", LCD_FB_SIZE);
    } else {
        ESP_LOGW(TAG, "全屏 SRAM 不足，回退 PSRAM + 分块 DMA");
        s_fb = heap_caps_malloc(LCD_FB_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_fb) {
            s_fb = heap_caps_malloc(LCD_FB_SIZE, MALLOC_CAP_8BIT);
        }
        if (!s_fb) {
            ESP_LOGE(TAG, "帧缓冲分配失败!");
            return -1;
        }

        size_t bounce_size = LCD_WIDTH * s_dma_lines * 2;
        s_dma_buf = heap_caps_malloc(bounce_size,
                                     MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!s_dma_buf) {
            ESP_LOGE(TAG, "DMA bounce buffer 分配失败 (%d bytes)", bounce_size);
            return -1;
        }
        s_use_full_dma = false;
        ESP_LOGI(TAG, "帧缓冲: PSRAM 渲染 + SRAM DMA bounce %d bytes (每次 %d 行)",
                 bounce_size, s_dma_lines);
    }
    memset(s_fb, 0, LCD_FB_SIZE);
    return 0;
}

static void flush_framebuffer(void)
{
    if (!s_panel || !s_fb || !s_dma_done_sem) return;

    if (s_use_full_dma) {
        /* 全屏一次 DMA 推送，等待传输完成再返回 */
        xSemaphoreTake(s_dma_done_sem, 0);  /* 清理残留 */
        esp_lcd_panel_draw_bitmap(s_panel, 0, 0, LCD_WIDTH, LCD_HEIGHT, s_fb);
        /* 等待 DMA 完成回调，最多 100ms（20MHz 下 108KB 约 43ms） */
        xSemaphoreTake(s_dma_done_sem, pdMS_TO_TICKS(100));
    } else {
        /* 回退：分块 bounce——每块等 DMA 完成后再 memcpy 下一块 */
        for (int y = 0; y < LCD_HEIGHT; y += s_dma_lines) {
            int h = (y + s_dma_lines > LCD_HEIGHT) ? LCD_HEIGHT - y : s_dma_lines;
            size_t len = LCD_WIDTH * h * 2;
            xSemaphoreTake(s_dma_done_sem, 0);
            memcpy(s_dma_buf, &s_fb[y * LCD_WIDTH], len);
            esp_lcd_panel_draw_bitmap(s_panel, 0, y, LCD_WIDTH, y + h, s_dma_buf);
            xSemaphoreTake(s_dma_done_sem, pdMS_TO_TICKS(100));
        }
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

static void fb_fillrect(int x, int y, int w, int h, uint16_t c) {
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > LCD_WIDTH) w = LCD_WIDTH - x;
    if (y + h > LCD_HEIGHT) h = LCD_HEIGHT - y;
    if (w <= 0 || h <= 0) return;
    for (int r = y; r < y + h; r++)
        for (int col = x; col < x + w; col++)
            s_fb[r * LCD_WIDTH + col] = c;
}

static void fb_drawrect(int x, int y, int w, int h, uint16_t c) {
    fb_fillrect(x, y, w, 1, c);
    fb_fillrect(x, y + h - 1, w, 1, c);
    fb_fillrect(x, y, 1, h, c);
    fb_fillrect(x + w - 1, y, 1, h, c);
}

static void fb_hline(int x, int y, int w, uint16_t c) {
    fb_fillrect(x, y, w, 1, c);
}

static int fb_text(int x, int y, const char *s, uint16_t c) {
    return lcd_font_draw_text(s_fb, LCD_WIDTH, LCD_HEIGHT, x, y, s, c);
}

static int fb_text_center(int y, const char *s, uint16_t c) {
    return lcd_font_draw_text_centered(s_fb, LCD_WIDTH, LCD_HEIGHT, y, s, c);
}

static int fb_text_right(int x_right, int y, const char *s, uint16_t c) {
    return lcd_font_draw_text_right(s_fb, LCD_WIDTH, LCD_HEIGHT, x_right, y, s, c);
}

/* 截断字符串到指定像素宽度，末尾加 "~"。
 * ASCII 按字节、CJK 按 3 字节 UTF-8 逐字符测量。 */
static void fb_text_trunc(int x, int y, const char *s, uint16_t c, int max_w) {
    int full_w = lcd_font_text_width(s);
    if (full_w <= max_w) {
        fb_text(x, y, s, c);
        return;
    }
    /* 逐字符截断，~ 占 8px */
    char buf[48];
    int i = 0, w = 0;
    const char *p = s;
    while (*p && i < (int)sizeof(buf) - 2) {
        int cw, step;
        unsigned char ch = (unsigned char)*p;
        if (ch < 0x80) { cw = FONT_ASCII_W; step = 1; }
        else if ((ch & 0xE0) == 0xC0) { cw = FONT_CJK_W; step = 2; }
        else { cw = FONT_CJK_W; step = 3; }
        if (w + cw + FONT_ASCII_W > max_w) break;
        for (int k = 0; k < step && p[k]; k++) buf[i++] = p[k];
        w += cw;
        p += step;
    }
    buf[i++] = '~';
    buf[i] = '\0';
    fb_text(x, y, buf, c);
}

/* ================================================================
 * 信号强度条形图
 * ================================================================ */
static void draw_signal_bars(int x, int y, int rssi, int max_bars)
{
    int level = 0;
    if (rssi >= -30) level = max_bars;
    else if (rssi > -100) level = (rssi + 100) * max_bars / 70;
    if (level < 0) level = 0;
    if (level > max_bars) level = max_bars;

    int bar_w = 3;
    int gap = 2;
    for (int i = 0; i < max_bars; i++) {
        int bh = 3 + i * 2;
        int by = y + (max_bars * 2 + 3) - bh;
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
        case RID_PROTOCOL_ASTM_F3411:
        case RID_PROTOCOL_ASD_STAN:
        case RID_PROTOCOL_GB42590:
        case RID_PROTOCOL_GB46750:
            return "标准";
        default:
            return "未知";
    }
}

static uint16_t protocol_color(uint8_t proto)
{
    switch (proto) {
        case RID_PROTOCOL_ASTM_F3411:
        case RID_PROTOCOL_ASD_STAN:
        case RID_PROTOCOL_GB42590:
        case RID_PROTOCOL_GB46750:
            return C_CYAN;
        default:
            return C_GRAY;
    }
}

/* ================================================================
 * 状态栏
 * ================================================================ */
static void render_statusbar(int active_count, int channel)
{
    fb_fillrect(0, 0, LCD_WIDTH, STATUSBAR_H, C_BLUE);
    fb_text(4, SB_TEXT_Y, "无人机侦测", C_WHITE);

    char buf[24];
    snprintf(buf, sizeof(buf), "CH%d  %d机", channel, active_count);
    fb_text_right(LCD_WIDTH - 4, SB_TEXT_Y, buf, C_CYAN);
}

/* ================================================================
 * 底栏
 * ================================================================ */
static void render_footer(lcd_page_t page)
{
    fb_fillrect(0, CONTENT_Y1, LCD_WIDTH, FOOTER_H, rgb565(20, 30, 20));
    const char *hints[] = {
        "User下翻 长按上翻",
        "User翻页 Boot详情",
        "Boot返回列表",
        "User翻页 Boot切换",
        "User翻页 长按返回",
    };
    int idx = (int)page;
    if (idx < 0 || idx >= 5) idx = 0;
    fb_text(3, FT_TEXT_Y, hints[idx], C_LTGRAY);
}

/* ================================================================
 * 主页
 * ================================================================ */
static void render_home(void)
{
    int active = 0, standard = 0, dji = 0;

    if (s_tracker && s_tracker_mutex) {
        xSemaphoreTake(s_tracker_mutex, portMAX_DELAY);
        for (int i = 0; i < s_max_uavs; i++) {
            if (s_tracker[i].active) {
                active++;
                if (s_tracker[i].is_dji) dji++;
                else standard++;
            }
        }
        xSemaphoreGive(s_tracker_mutex);
    }

    int y = CONTENT_Y0 + 10;

    char num[8];
    snprintf(num, sizeof(num), "%d", active);
    fb_drawrect(8, y, LCD_WIDTH - 16, 52, C_TEAL);
    fb_fillrect(9, y + 1, LCD_WIDTH - 18, 50, rgb565(5, 20, 30));

    fb_text(16, y + 8, "活跃目标", C_CYAN);
    fb_text_right(LCD_WIDTH - 16, y + 6, num, active > 0 ? C_GREEN : C_GRAY);
    fb_text(16, y + 30, active > 0 ? "● 监测中" : "○ 未发现",
            active > 0 ? C_GREEN : C_GRAY);

    y += 64;

    fb_text(8, y, "协议分类", C_YELLOW);
    y += FONT_LINE_H + 4;
    fb_hline(8, y, LCD_WIDTH - 16, C_DIM);
    y += 6;

    char buf[24];
    snprintf(buf, sizeof(buf), "标准协议  %d", standard);
    fb_text(12, y, buf, C_CYAN);
    snprintf(buf, sizeof(buf), "%d", standard);
    fb_text_right(LCD_WIDTH - 12, y, buf, C_CYAN);
    y += FONT_LINE_H + 4;

    snprintf(buf, sizeof(buf), "DJI大疆  %d", dji);
    fb_text(12, y, buf, C_ORANGE);
    snprintf(buf, sizeof(buf), "%d", dji);
    fb_text_right(LCD_WIDTH - 12, y, buf, C_ORANGE);
    y += FONT_LINE_H + 10;

    fb_hline(8, y, LCD_WIDTH - 16, C_DIM);
    y += 10;

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
        fb_text_center(CONTENT_Y0 + 100, "未发现目标", C_GRAY);
        fb_text_center(CONTENT_Y0 + 124, "User键翻页", C_DIM);
        return;
    }

    if (s_selection >= count) s_selection = count - 1;
    if (s_selection < 0) s_selection = 0;

    int item_h = 40;
    int visible = CONTENT_H / item_h;
    if (s_selection < s_scroll_offset) s_scroll_offset = s_selection;
    if (s_selection >= s_scroll_offset + visible) s_scroll_offset = s_selection - visible + 1;
    if (s_scroll_offset > count - visible) s_scroll_offset = count - visible;
    if (s_scroll_offset < 0) s_scroll_offset = 0;

    int y = CONTENT_Y0 + 1;
    for (int idx = s_scroll_offset; idx < s_scroll_offset + visible && idx < count; idx++) {
        int i = indices[idx];
        bool sel = (idx == s_selection);

        uint16_t card_bg = sel ? rgb565(10, 40, 70) : C_CARD;
        fb_fillrect(2, y, LCD_WIDTH - 4, item_h - 2, card_bg);
        if (sel) fb_drawrect(2, y, LCD_WIDTH - 4, item_h - 2, C_CYAN);

        int iy = y + 3;

        /* 第一行：编号 + 型号/ID（截断），协议标签右侧 */
        const char *label_text;
        char id_buf[24];
        if (s_tracker[i].is_dji && s_tracker[i].dji_model[0]) {
            snprintf(id_buf, sizeof(id_buf), "%d.%s", idx + 1, s_tracker[i].dji_model);
            label_text = id_buf;
        } else {
            const char *id = s_tracker[i].basic_id.uas_id[0] ?
                s_tracker[i].basic_id.uas_id : "----";
            snprintf(id_buf, sizeof(id_buf), "%d.%s", idx + 1, id);
            label_text = id_buf;
        }

        const char *plabel = s_tracker[i].is_dji ? "DJI" : protocol_label(s_tracker[i].protocol);
        uint16_t pcolor = s_tracker[i].is_dji ? C_ORANGE : protocol_color(s_tracker[i].protocol);
        int plabel_w = lcd_font_text_width(plabel);

        /* ID 最大宽度：屏幕宽 - 左右边距 - 协议标签宽 - 间距 */
        int max_id_w = LCD_WIDTH - 10 - plabel_w - 6;
        fb_text_trunc(5, iy, label_text, sel ? C_WHITE : C_GREEN, max_id_w);
        fb_text_right(LCD_WIDTH - 5, iy, plabel, pcolor);
        iy += FONT_LINE_H + 1;

        /* 第二行：信号条 + RSSI + 高度 */
        draw_signal_bars(5, iy + 1, s_tracker[i].last_rssi, 4);
        char buf[32];
        snprintf(buf, sizeof(buf), "%ddBm", s_tracker[i].last_rssi);
        fb_text(38, iy, buf, C_CYAN);

        if (s_tracker[i].is_dji) {
            snprintf(buf, sizeof(buf), "%.0fm", s_tracker[i].dji_altitude);
        } else if ((int)s_tracker[i].location.height != 0) {
            snprintf(buf, sizeof(buf), "%dm", (int)s_tracker[i].location.height);
        } else {
            snprintf(buf, sizeof(buf), "--m");
        }
        fb_text_right(LCD_WIDTH - 5, iy, buf, C_YELLOW);

        y += item_h;
    }

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
        fb_text_center(CONTENT_Y0 + 100, "未选中目标", C_GRAY);
        return;
    }

    int y = CONTENT_Y0 + 4;
    char buf[48];

    /* 标题行 */
    if (s_tracker[idx].is_dji) {
        fb_fillrect(0, y - 2, LCD_WIDTH, FONT_LINE_H + 6, rgb565(60, 30, 0));
        const char *model = s_tracker[idx].dji_model[0] ?
            s_tracker[idx].dji_model : "DJI Drone";
        fb_text_trunc(4, y, model, C_ORANGE, LCD_WIDTH - 8);
    } else {
        fb_fillrect(0, y - 2, LCD_WIDTH, FONT_LINE_H + 6, rgb565(0, 30, 60));
        fb_text(4, y, "目标详情", C_CYAN);
    }
    y += FONT_LINE_H + 8;

    /* ID / SN — 截断到屏幕宽度 */
    if (s_tracker[idx].is_dji && s_tracker[idx].dji_serial[0]) {
        fb_text(4, y, "SN", C_GREEN);
        fb_text_trunc(30, y, s_tracker[idx].dji_serial, C_WHITE, LCD_WIDTH - 34);
    } else if (s_tracker[idx].basic_id.uas_id[0]) {
        fb_text(4, y, "ID", C_GREEN);
        fb_text_trunc(30, y, s_tracker[idx].basic_id.uas_id, C_WHITE, LCD_WIDTH - 34);
    }
    y += FONT_LINE_H + 4;

    /* 信号 */
    fb_text(4, y, "信号", C_GREEN);
    draw_signal_bars(38, y + 2, s_tracker[idx].last_rssi, 5);
    snprintf(buf, sizeof(buf), "%ddBm", s_tracker[idx].last_rssi);
    fb_text(80, y, buf, C_WHITE);
    y += FONT_LINE_H + 6;

    fb_hline(4, y, LCD_WIDTH - 8, C_DIM);
    y += 6;

    /* 位置信息 */
    if (s_tracker[idx].is_dji) {
        snprintf(buf, sizeof(buf), "纬度  %.6f", s_tracker[idx].dji_latitude);
        fb_text(4, y, buf, C_CYAN); y += FONT_LINE_H + 2;
        snprintf(buf, sizeof(buf), "经度  %.6f", s_tracker[idx].dji_longitude);
        fb_text(4, y, buf, C_CYAN); y += FONT_LINE_H + 2;
        snprintf(buf, sizeof(buf), "高度  %.1fm", s_tracker[idx].dji_altitude);
        fb_text(4, y, buf, C_YELLOW); y += FONT_LINE_H + 2;
        snprintf(buf, sizeof(buf), "速度  %.1fm/s", s_tracker[idx].dji_speed_h);
        fb_text(4, y, buf, C_YELLOW); y += FONT_LINE_H + 2;
        if (s_tracker[idx].dji_heading >= 0) {
            snprintf(buf, sizeof(buf), "航向  %.1f度", s_tracker[idx].dji_heading);
            fb_text(4, y, buf, C_YELLOW); y += FONT_LINE_H + 2;
        }
    } else {
        if (s_tracker[idx].location.latitude != 0) {
            snprintf(buf, sizeof(buf), "纬度  %.6f", s_tracker[idx].location.latitude);
            fb_text(4, y, buf, C_CYAN); y += FONT_LINE_H + 2;
            snprintf(buf, sizeof(buf), "经度  %.6f", s_tracker[idx].location.longitude);
            fb_text(4, y, buf, C_CYAN); y += FONT_LINE_H + 2;
        }
        if ((int)s_tracker[idx].location.height != 0) {
            snprintf(buf, sizeof(buf), "高度  %dm", (int)s_tracker[idx].location.height);
            fb_text(4, y, buf, C_YELLOW); y += FONT_LINE_H + 2;
        }
        if (s_tracker[idx].location.speed_horizontal > 0) {
            snprintf(buf, sizeof(buf), "速度  %.1fm/s", s_tracker[idx].location.speed_horizontal);
            fb_text(4, y, buf, C_YELLOW); y += FONT_LINE_H + 2;
        }
        if (s_tracker[idx].location.direction != 0) {
            snprintf(buf, sizeof(buf), "航向  %.1f度", s_tracker[idx].location.direction);
            fb_text(4, y, buf, C_YELLOW); y += FONT_LINE_H + 2;
        }
    }

    y += 4;
    fb_hline(4, y, LCD_WIDTH - 8, C_DIM);
    y += 6;

    /* MAC — 标签+值，标签占 4 字符宽(32px)，值从 36px 开始 */
    snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
             s_tracker[idx].mac[0], s_tracker[idx].mac[1],
             s_tracker[idx].mac[2], s_tracker[idx].mac[3],
             s_tracker[idx].mac[4], s_tracker[idx].mac[5]);
    fb_text(4, y, "MAC", C_GREEN);
    fb_text(34, y, buf, C_GRAY);
    y += FONT_LINE_H + 2;

    snprintf(buf, sizeof(buf), "信道  %d", s_tracker[idx].last_channel & 0x7F);
    fb_text(4, y, buf, C_GREEN);
    y += FONT_LINE_H + 2;

    if (s_tracker[idx].last_seen_ms > 0) {
        uint32_t ago = (uint32_t)(esp_timer_get_time() / 1000 - s_tracker[idx].last_seen_ms) / 1000;
        if (ago < 60)
            snprintf(buf, sizeof(buf), "%lu秒前", (unsigned long)ago);
        else
            snprintf(buf, sizeof(buf), "%lu分%lu秒前",
                     (unsigned long)(ago / 60), (unsigned long)(ago % 60));
        fb_text(4, y, "更新", C_GREEN);
        fb_text(34, y, buf, C_LTGRAY);
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
    y += FONT_LINE_H + 4;

    fb_text(4, y, "模式", C_GREEN);
    const char *modes[] = {"圆周", "往返", "搜索"};
    int mi = s_sim_info.sim_flight_mode;
    if (mi < 0 || mi >= 3) mi = 0;
    fb_text_right(LCD_WIDTH - 4, y, modes[mi], C_YELLOW);
    y += FONT_LINE_H + 4;

    snprintf(buf, sizeof(buf), "信道  %d", s_sim_info.sim_channel);
    fb_text(4, y, buf, C_CYAN);
    y += FONT_LINE_H + 4;

    fb_text(4, y, "ID", C_GREEN);
    fb_text(30, y,
        s_sim_info.sim_uas_id[0] ? s_sim_info.sim_uas_id : "SIM-C5",
        C_WHITE);
    y += FONT_LINE_H + 8;

    fb_hline(4, y, LCD_WIDTH - 8, C_DIM);
    y += 6;

    snprintf(buf, sizeof(buf), "纬度  %.4f", s_sim_info.sim_lat);
    fb_text(4, y, buf, C_GREEN); y += FONT_LINE_H + 2;
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
    fb_text(4, y, buf, C_CYAN); y += FONT_LINE_H + 2;
    snprintf(buf, sizeof(buf), "经度  %.6f", s_sim_info.sim_lon);
    fb_text(4, y, buf, C_CYAN); y += FONT_LINE_H + 2;
    snprintf(buf, sizeof(buf), "航向  %.1f度", s_sim_info.sim_heading);
    fb_text(4, y, buf, C_YELLOW); y += FONT_LINE_H + 2;
    snprintf(buf, sizeof(buf), "高度  %.1fm", s_sim_info.sim_alt);
    fb_text(4, y, buf, C_GREEN); y += FONT_LINE_H + 2;
    snprintf(buf, sizeof(buf), "信道  %d", s_sim_info.sim_channel);
    fb_text(4, y, buf, C_WHITE); y += FONT_LINE_H + 6;

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

    int bx = 15, by = 100, bw = LCD_WIDTH - 30, bh = 100;
    fb_fillrect(bx, by, bw, bh, rgb565(5, 10, 35));
    fb_drawrect(bx, by, bw, bh, C_CYAN);

    fb_text_center(by + 12, "蓝牙配对", C_CYAN);

    int pin_w = lcd_font_text_width(s_pin);
    lcd_font_draw_text(s_fb, LCD_WIDTH, LCD_HEIGHT,
                       (LCD_WIDTH - pin_w) / 2, by + 42, s_pin, C_WHITE);

    fb_text_center(by + 72, "请在手机输入", C_GRAY);
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

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/* ================================================================
 * 按键轮询
 * ================================================================ */
static void button_poll_task(void *arg)
{
    static uint32_t t_user_down = 0, t_boot_down = 0;
    static bool user_down = false, boot_down = false;
    static bool user_long_sent = false, boot_long_sent = false;
    const uint32_t debounce = 30;
    const uint32_t long_press = 600;

    while (1) {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

        if (gpio_get_level(BTN_USER_PIN) == 0) {
            if (!user_down) {
                user_down = true;
                t_user_down = now;
                user_long_sent = false;
            } else if (!user_long_sent && (now - t_user_down > long_press)) {
                lcd_display_send_key(LCD_KEY_PREV);
                user_long_sent = true;
            }
        } else if (user_down) {
            if (!user_long_sent && (now - t_user_down > debounce)) {
                lcd_display_send_key(LCD_KEY_NEXT);
            }
            user_down = false;
        }

        if (gpio_get_level(BTN_BOOT_PIN) == 0) {
            if (!boot_down) {
                boot_down = true;
                t_boot_down = now;
                boot_long_sent = false;
            } else if (!boot_long_sent && (now - t_boot_down > long_press)) {
                lcd_display_send_key(LCD_KEY_BACK);
                boot_long_sent = true;
            }
        } else if (boot_down) {
            if (!boot_long_sent && (now - t_boot_down > debounce)) {
                lcd_display_send_key(LCD_KEY_SELECT);
            }
            boot_down = false;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ================================================================
 * 公开 API
 * ================================================================ */
int lcd_display_init(void)
{
    ESP_LOGI(TAG, "=== LCD 模块初始化（lcdfix10）===");

    if (init_framebuffer() != 0) return -1;
    if (init_lcd() != 0) return -1;
    init_backlight();

    /* 开机测试：四角方块 + 十字线 */
    fb_fill(rgb565(0, 20, 50));
    fb_fillrect(0, 0, 20, 20, C_WHITE);
    fb_fillrect(LCD_WIDTH - 20, 0, 20, 20, C_WHITE);
    fb_fillrect(0, LCD_HEIGHT - 20, 20, 20, C_WHITE);
    fb_fillrect(LCD_WIDTH - 20, LCD_HEIGHT - 20, 20, 20, C_WHITE);
    fb_fillrect(LCD_WIDTH/2 - 1, 0, 2, LCD_HEIGHT, C_WHITE);
    fb_fillrect(0, LCD_HEIGHT/2 - 1, LCD_WIDTH, 2, C_WHITE);
    flush_framebuffer();
    vTaskDelay(pdMS_TO_TICKS(1500));

    fb_fill(C_BG);
    fb_text_center(140, "无人机侦测器", C_CYAN);
    fb_text_center(164, "T-Display-C5", C_GRAY);
    flush_framebuffer();
    vTaskDelay(pdMS_TO_TICKS(1000));

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
