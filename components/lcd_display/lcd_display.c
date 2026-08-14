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
#include "driver/gpio.h"
#include "esp_intr_alloc.h"
#include "driver/i2c.h"

#ifndef FIXED_CHANNEL
#define FIXED_CHANNEL 6
#endif

static const char *TAG = "lcd_display";

/* ================================================================
 * 模拟器状态
 * ================================================================ */
static sim_display_info_t s_sim_info = {0};

void lcd_display_set_sim_info(const sim_display_info_t *info) {
    if (info) memcpy(&s_sim_info, info, sizeof(s_sim_info));
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
static volatile int s_detail_sel = 0;  /* 详情页当前查看的目标序号 */
static volatile bool s_display_off = false;  /* 熄屏/睡眠状态 */
static volatile uint32_t s_last_activity_ms = 0;  /* 最后一次按键时间(ms) */
static bool s_both_pressed = false;  /* 双键同按检测 */

/* 模拟器操作回调（由 app_main 注册） */
static sim_action_cb_t s_sim_start_cb = NULL;
static sim_action_cb_t s_sim_stop_cb = NULL;
static sim_mode_cb_t s_sim_mode_cb = NULL;

/* lcdfix16: GPS 状态回调（由 app_main 注入，避免 lcd_display 组件
 * 反向依赖 main_rx/gps_module.h 造成组件循环依赖）。
 * 返回 true=有定位；输出经纬度/海拔/卫星数。 */
static lcd_gps_provider_cb_t s_gps_provider_cb = NULL;

void lcd_display_register_gps_provider(lcd_gps_provider_cb_t cb) {
    s_gps_provider_cb = cb;
}

void lcd_display_register_sim_callbacks(sim_action_cb_t start,
                                         sim_action_cb_t stop,
                                         sim_mode_cb_t cycle_mode) {
    s_sim_start_cb = start;
    s_sim_stop_cb = stop;
    s_sim_mode_cb = cycle_mode;
}

#define DISPLAY_TIMEOUT_MS  60000  /* 60秒无操作自动熄屏 */
#define PWR_BOTH_HOLD_MS    1500   /* 双键同按1.5秒关机 */

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
 * AXP2602 PMIC 电池电压读取（I2C）
 * T-Display-C5: SDA=GPIO2, SCL=GPIO3, addr=0x62
 * ================================================================ */
#define AXP_I2C_PORT       I2C_NUM_0
#define AXP_I2C_FREQ       100000
static bool s_axp_inited = false;

static esp_err_t axp_read_reg(uint8_t reg, uint8_t *val) {
    return i2c_master_write_read_device(AXP_I2C_PORT, AXP2602_I2C_ADDR,
                                        &reg, 1, val, 1, pdMS_TO_TICKS(50));
}

static void axp2602_init(void) {
    if (s_axp_inited) return;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = TOUCH_PIN_SDA,
        .scl_io_num = TOUCH_PIN_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = AXP_I2C_FREQ,
    };
    if (i2c_param_config(AXP_I2C_PORT, &conf) != ESP_OK) return;
    if (i2c_driver_install(AXP_I2C_PORT, conf.mode, 0, 0, 0) != ESP_OK) return;
    vTaskDelay(pdMS_TO_TICKS(50));
    /* 读取芯片 ID 寄存器 0x03 校验（AXP2602 通常返回 0x34 或类似） */
    uint8_t id = 0;
    if (axp_read_reg(0x03, &id) == ESP_OK) {
        ESP_LOGI(TAG, "AXP2602 detected, chip ID=0x%02X", id);
        s_axp_inited = true;
    } else {
        ESP_LOGW(TAG, "AXP2602 not responding on I2C");
    }
}

/* 返回电池电压 (mV)，失败返回 -1 */
static int axp2602_read_battery_mv(void) {
    if (!s_axp_inited) return -1;
    /* AXP2602 电池电压高字节：0x34, 低字节：0x35 (bit7-4 有效)
     * 公式: voltage = (H << 4 | L >> 4) * 1.1mV  (参考 LILYGO AXP2602 demo) */
    uint8_t hi = 0, lo = 0;
    if (axp_read_reg(0x34, &hi) != ESP_OK) return -1;
    if (axp_read_reg(0x35, &lo) != ESP_OK) return -1;
    int raw = ((int)hi << 4) | ((lo >> 4) & 0x0F);
    int mv = (int)(raw * 1.1f);
    return mv;
}

static int battery_mv_to_pct(int mv) {
    if (mv <= 3300) return 0;
    if (mv >= 4200) return 100;
    /* 简单线性映射；锂电实际曲线 3.3V~4.2V */
    return (mv - 3300) * 100 / 900;
}

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

    /* 竖屏 170×320，set_gap(35,0)，与 LILYGO 官方 demo 一致。
     * 无镜像：文字正向，LILYGO logo 在顶部（与 lcdfix7 真机验证一致） */
    esp_lcd_panel_mirror(s_panel, false, false);
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

static inline void fb_pixel(int x, int y, uint16_t c) {
    if (x >= 0 && x < LCD_WIDTH && y >= 0 && y < LCD_HEIGHT)
        s_fb[y * LCD_WIDTH + x] = c;
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
/* ================================================================
 * 状态栏（含 GPS 图标 + 电量图标）
 * ================================================================ */

/* lcdfix16: 8x10 GPS 定位针图标
 * 点阵设计：圆形针头 + 针尖朝下，1=有色，0=透明 */
static const uint8_t GPS_ICON_W = 8, GPS_ICON_H = 10;
static const uint8_t gps_icon_bits[] = {
    0b00111100,
    0b01111110,
    0b11111111,
    0b11111111,
    0b11111111,
    0b01111110,
    0b00111100,
    0b00011000,
    0b00111100,
    0b00111100,
};

static void draw_gps_icon(int x, int y, bool has_fix, int sats) {
    /* 有定位绿色，无定位灰色；>=4 颗卫星显示绿色 */
    uint16_t c = has_fix ? C_GREEN : C_GRAY;
    for (int row = 0; row < GPS_ICON_H; row++) {
        uint8_t bits = gps_icon_bits[row];
        for (int col = 0; col < GPS_ICON_W; col++) {
            if (bits & (0x80 >> col)) {
                fb_pixel(x + col, y + row, c);
            }
        }
    }
    /* 卫星数（有定位才显示）。缓冲区给 12 字节，避免 GCC -Wformat-truncation 报警。 */
    if (has_fix && sats > 0) {
        char s[12];
        snprintf(s, sizeof(s), "%d", sats);
        /* 图标 8px 宽，右侧画数字 */
        fb_text(x + GPS_ICON_W + 1, y - 2, s, C_GREEN);
    }
}

static void draw_battery_icon(int x, int y, int pct, uint16_t color)
{
    /* 电池外框 16×8，正极 2×4 */
    int w = 16, h = 8;
    fb_drawrect(x, y, w, h, color);
    fb_fillrect(x + w, y + 2, 2, h - 4, color);  /* 正极头 */
    if (pct > 0) {
        int fill_w = (w - 4) * pct / 100;
        if (fill_w < 1) fill_w = 1;
        uint16_t fc = (pct > 30) ? C_GREEN : (pct > 15 ? C_YELLOW : C_RED);
        fb_fillrect(x + 2, y + 2, fill_w, h - 4, fc);
    }
}

static void render_statusbar(int active_count, int channel)
{
    fb_fillrect(0, 0, LCD_WIDTH, STATUSBAR_H, C_BLUE);
    fb_text(4, SB_TEXT_Y, "无人机侦测", C_WHITE);

    char buf[24];
    snprintf(buf, sizeof(buf), "CH%d %d机", channel, active_count);
    int tw = lcd_font_text_width(buf);

    /* lcdfix16: GPS 图标 + 卫星状态（通过回调从 app_main 获取，
     * 避免 lcd_display 组件直接 include main_rx/gps_module.h）。
     * 没接GPS/未定位时灰色，定位后绿色+卫星数。 */
    bool gps_fix = false;
    int gps_sats = 0;
    double dummy_lat, dummy_lon, dummy_alt;
    if (s_gps_provider_cb) {
        gps_fix = s_gps_provider_cb(&dummy_lat, &dummy_lon, &dummy_alt, &gps_sats);
    }
    int gps_right = LCD_WIDTH - 4 - tw;
    int gps_icon_x = gps_right - (gps_fix ? 22 : 12);  /* 有卫星数时多留位置 */
    if (gps_icon_x < 80) gps_icon_x = 80;
    draw_gps_icon(gps_icon_x, (STATUSBAR_H - GPS_ICON_H) / 2, gps_fix, gps_sats);

    fb_text(LCD_WIDTH - 4 - tw - 40, SB_TEXT_Y, buf, C_CYAN);

    /* lcdfix15: 真实电量（AXP2602 I2C） */
    static int s_batt_pct = -1;
    static uint32_t s_last_batt_read = 0;
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (s_batt_pct < 0 || now - s_last_batt_read > 5000) {
        int mv = axp2602_read_battery_mv();
        if (mv > 0) {
            s_batt_pct = battery_mv_to_pct(mv);
        } else if (s_batt_pct < 0) {
            s_batt_pct = 75;  /* I2C 未就绪时用占位 */
        }
        s_last_batt_read = now;
    }
    draw_battery_icon(LCD_WIDTH - 20, (STATUSBAR_H - 8) / 2, s_batt_pct, C_WHITE);
    snprintf(buf, sizeof(buf), "%d%%", s_batt_pct);
    int pw = lcd_font_text_width(buf);
    fb_text(LCD_WIDTH - 24 - pw, SB_TEXT_Y, buf,
            s_batt_pct > 30 ? C_WHITE : C_RED);
}

/* ================================================================
 * 底栏
 * ================================================================ */
static void render_footer(lcd_page_t page)
{
    fb_fillrect(0, CONTENT_Y1, LCD_WIDTH, FOOTER_H, rgb565(20, 30, 20));
    const char *hints[] = {
        "短按列表 长按模拟",
        "User选择 Boot详情",
        "User切换 Boot返回",
        "User选项 Boot启停",
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
        fb_text_center(CONTENT_Y0 + 124, "Boot键返回", C_DIM);
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
        char id_buf[48];
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
 * 详情页（重排，紧凑两列布局，显示完整飞行数据）
 * ================================================================ */
static void render_detail(void)
{
    if (!s_tracker || !s_tracker_mutex) return;

    int idx = -1;
    int sel = s_detail_sel;
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

    int y = CONTENT_Y0 + 2;
    char buf[48];

    /* 标题：型号 + (n/N) */
    char title_buf[48];
    int total_active = 0;
    xSemaphoreTake(s_tracker_mutex, portMAX_DELAY);
    for (int i = 0; i < s_max_uavs; i++)
        if (s_tracker[i].active) total_active++;
    xSemaphoreGive(s_tracker_mutex);

    if (s_tracker[idx].is_dji) {
        fb_fillrect(0, y - 2, LCD_WIDTH, FONT_LINE_H + 4, rgb565(60, 30, 0));
        const char *model = s_tracker[idx].dji_model[0] ?
            s_tracker[idx].dji_model : "DJI Drone";
        snprintf(title_buf, sizeof(title_buf), "%s (%d/%d)", model, s_detail_sel + 1, total_active);
        fb_text_trunc(4, y, title_buf, C_ORANGE, LCD_WIDTH - 8);
    } else {
        fb_fillrect(0, y - 2, LCD_WIDTH, FONT_LINE_H + 4, rgb565(0, 30, 60));
        const char *id = s_tracker[idx].basic_id.uas_id[0] ?
            s_tracker[idx].basic_id.uas_id : "RID";
        snprintf(title_buf, sizeof(title_buf), "%s (%d/%d)", id, s_detail_sel + 1, total_active);
        fb_text_trunc(4, y, title_buf, C_CYAN, LCD_WIDTH - 8);
    }
    y += FONT_LINE_H + 4;

    /* 第二行：信号条 + RSSI + 电量（两列） */
    draw_signal_bars(4, y + 1, s_tracker[idx].last_rssi, 4);
    snprintf(buf, sizeof(buf), "%ddBm", s_tracker[idx].last_rssi);
    fb_text(30, y, buf, C_CYAN);

    if (s_tracker[idx].is_dji && s_tracker[idx].dji_battery > 0 && s_tracker[idx].dji_battery <= 100) {
        snprintf(buf, sizeof(buf), "电量%d%%", s_tracker[idx].dji_battery);
        fb_text_right(LCD_WIDTH - 4, y, buf, s_tracker[idx].dji_battery > 30 ? C_GREEN : C_RED);
    }
    y += FONT_LINE_H + 3;

    fb_hline(4, y, LCD_WIDTH - 8, C_DIM);
    y += 4;

    /* 飞行数据：lcdfix14 始终显示字段，0 值也显示（用户要全字段） */
    if (s_tracker[idx].is_dji) {
        double lat = s_tracker[idx].dji_latitude;
        double lon = s_tracker[idx].dji_longitude;
        snprintf(buf, sizeof(buf), "纬度 %.6f", lat);
        fb_text(4, y, buf, lat != 0 ? C_CYAN : C_GRAY); y += FONT_LINE_H + 1;
        snprintf(buf, sizeof(buf), "经度 %.6f", lon);
        fb_text(4, y, buf, lon != 0 ? C_CYAN : C_GRAY); y += FONT_LINE_H + 1;

        float alt = s_tracker[idx].dji_altitude;
        snprintf(buf, sizeof(buf), "海拔 %.1fm", alt);
        fb_text(4, y, buf, alt != 0 ? C_YELLOW : C_GRAY); y += FONT_LINE_H + 1;

        float spd = s_tracker[idx].dji_speed_h;
        snprintf(buf, sizeof(buf), "速度 %.1fm/s", spd);
        fb_text(4, y, buf, spd > 0 ? C_YELLOW : C_GRAY); y += FONT_LINE_H + 1;

        float hdg = s_tracker[idx].dji_heading;
        snprintf(buf, sizeof(buf), "航向 %.1f度", hdg);
        fb_text(4, y, buf, C_YELLOW); y += FONT_LINE_H + 1;

        /* 飞手位置 */
        double plat = s_tracker[idx].dji_pilot_lat;
        double plon = s_tracker[idx].dji_pilot_lon;
        y += 1;
        fb_text(4, y, "飞手位置", C_GREEN); y += FONT_LINE_H + 1;
        snprintf(buf, sizeof(buf), "  %.6f", plat);
        fb_text(4, y, buf, plat != 0 ? C_LTGRAY : C_GRAY); y += FONT_LINE_H + 1;
        snprintf(buf, sizeof(buf), "  %.6f", plon);
        fb_text(4, y, buf, plon != 0 ? C_LTGRAY : C_GRAY); y += FONT_LINE_H + 1;
    } else {
        /* 标准 RID */
        double lat = s_tracker[idx].location.latitude;
        double lon = s_tracker[idx].location.longitude;
        snprintf(buf, sizeof(buf), "纬度 %.6f", lat);
        fb_text(4, y, buf, lat != 0 ? C_CYAN : C_GRAY); y += FONT_LINE_H + 1;
        snprintf(buf, sizeof(buf), "经度 %.6f", lon);
        fb_text(4, y, buf, lon != 0 ? C_CYAN : C_GRAY); y += FONT_LINE_H + 1;

        float rh = s_tracker[idx].location.height;
        snprintf(buf, sizeof(buf), "相对高 %.0fm", rh);
        fb_text(4, y, buf, rh != 0 ? C_YELLOW : C_GRAY); y += FONT_LINE_H + 1;

        float ga = s_tracker[idx].location.altitude_geo;
        snprintf(buf, sizeof(buf), "海拔 %.0fm", ga);
        fb_text(4, y, buf, ga != 0 ? C_YELLOW : C_GRAY); y += FONT_LINE_H + 1;

        float spd = s_tracker[idx].location.speed_horizontal;
        snprintf(buf, sizeof(buf), "速度 %.1fm/s", spd);
        fb_text(4, y, buf, spd > 0 ? C_YELLOW : C_GRAY); y += FONT_LINE_H + 1;

        float dir = s_tracker[idx].location.direction;
        snprintf(buf, sizeof(buf), "航向 %.1f度", dir);
        fb_text(4, y, buf, C_YELLOW); y += FONT_LINE_H + 1;

        /* 操作员位置 */
        double plat = s_tracker[idx].system.operator_latitude;
        double plon = s_tracker[idx].system.operator_longitude;
        y += 1;
        fb_text(4, y, "操作员位置", C_GREEN); y += FONT_LINE_H + 1;
        snprintf(buf, sizeof(buf), "  %.6f", plat);
        fb_text(4, y, buf, plat != 0 ? C_LTGRAY : C_GRAY); y += FONT_LINE_H + 1;
        snprintf(buf, sizeof(buf), "  %.6f", plon);
        fb_text(4, y, buf, plon != 0 ? C_LTGRAY : C_GRAY); y += FONT_LINE_H + 1;
    }

    y += 2;
    fb_hline(4, y, LCD_WIDTH - 8, C_DIM);
    y += 3;

    /* 底部：MAC + 信道 + 更新时间（小字段落） */
    snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
             s_tracker[idx].mac[0], s_tracker[idx].mac[1],
             s_tracker[idx].mac[2], s_tracker[idx].mac[3],
             s_tracker[idx].mac[4], s_tracker[idx].mac[5]);
    fb_text(4, y, "MAC", C_GREEN);
    fb_text_trunc(30, y, buf, C_GRAY, LCD_WIDTH - 34);
    y += FONT_LINE_H + 1;

    snprintf(buf, sizeof(buf), "信道%d", s_tracker[idx].last_channel & 0x7F);
    fb_text(4, y, buf, C_GREEN);

    if (s_tracker[idx].last_seen_ms > 0) {
        uint32_t ago = (uint32_t)(esp_timer_get_time() / 1000 - s_tracker[idx].last_seen_ms) / 1000;
        if (ago < 60)
            snprintf(buf, sizeof(buf), "%lu秒前", (unsigned long)ago);
        else
            snprintf(buf, sizeof(buf), "%lu分%lu秒前",
                     (unsigned long)(ago / 60), (unsigned long)(ago % 60));
        fb_text_right(LCD_WIDTH - 4, y, buf, C_LTGRAY);
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
    y += FONT_LINE_H + 6;

    fb_text(4, y, "飞行模式", C_GREEN);
    const char *modes[] = {"圆周", "往返", "搜索"};
    int mi = s_sim_info.sim_flight_mode;
    if (mi < 0 || mi >= 3) mi = 0;
    fb_text_right(LCD_WIDTH - 4, y, modes[mi], C_YELLOW);
    y += FONT_LINE_H + 6;

    snprintf(buf, sizeof(buf), "信道%d", s_sim_info.sim_channel);
    fb_text(4, y, buf, C_CYAN);
    y += FONT_LINE_H + 4;

    fb_text(4, y, "ID", C_GREEN);
    fb_text(30, y,
        s_sim_info.sim_uas_id[0] ? s_sim_info.sim_uas_id : "SIM-C5",
        C_WHITE);
    y += FONT_LINE_H + 8;

    fb_hline(4, y, LCD_WIDTH - 8, C_DIM);
    y += 8;

    snprintf(buf, sizeof(buf), "纬度%.4f", s_sim_info.sim_lat);
    fb_text(4, y, buf, C_GREEN); y += FONT_LINE_H + 4;
    snprintf(buf, sizeof(buf), "经度%.4f", s_sim_info.sim_lon);
    fb_text(4, y, buf, C_GREEN); y += FONT_LINE_H + 4;
    snprintf(buf, sizeof(buf), "目标数%d", s_sim_info.sim_target_count);
    fb_text(4, y, buf, C_GREEN); y += FONT_LINE_H + 12;

    fb_hline(4, y, LCD_WIDTH - 8, C_DIM);
    y += 6;
    fb_text(4, y, "User切换模式", C_LTGRAY);
    y += FONT_LINE_H + 2;
    if (s_sim_info.is_sim_running)
        fb_text(4, y, "Boot键停止", C_ORANGE);
    else
        fb_text(4, y, "Boot键发射", C_GREEN);
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
    if (pin && pin[0]) {
        snprintf(s_pin, sizeof(s_pin), "%s", pin);
        s_pin_time = (uint32_t)(esp_timer_get_time() / 1000);
    } else {
        /* 空字符串=清除提示（连接成功后调用）*/
        s_pin[0] = 0;
        s_pin_time = 0;
    }
}

static void render_pair_overlay(void)
{
    if (!s_pin[0] || !s_pin_time) return;
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    if (now - s_pin_time > 3000) {
        s_pin[0] = 0;
        s_pin_time = 0;
        return;
    }

    int bx = 15, by = 120, bw = LCD_WIDTH - 30, bh = 60;
    fb_fillrect(bx, by, bw, bh, rgb565(5, 10, 35));
    fb_drawrect(bx, by, bw, bh, C_CYAN);

    fb_text_center(by + 10, "蓝牙已连接", C_CYAN);
    fb_text_center(by + 34, s_pin, C_WHITE);
}

/* ================================================================
 * 刷新任务
 * ================================================================ */
static void refresh_task(void *arg)
{
    s_last_activity_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    while (1) {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

        /* 熄屏时仅轮询按键，不渲染，省电流畅 */
        if (s_display_off) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        /* 60秒无操作自动熄屏（背光关、画面保持） */
        if (now - s_last_activity_ms > DISPLAY_TIMEOUT_MS) {
            ESP_LOGI(TAG, "Display timeout, turning off backlight");
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            s_display_off = true;
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

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
            /* 熄屏状态：第一次按键只唤醒屏幕，不执行操作（防误触） */
            if (s_display_off) {
                s_display_off = false;
                s_last_activity_ms = now;
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 255);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
                ESP_LOGI(TAG, "Display woken by key");
                continue;
            }
            s_last_activity_ms = now;

            /* POWER 键（暂未从按键轮询发送，预留） */
            if (key == LCD_KEY_POWER) continue;

            int active = 0;
            if (s_tracker && s_tracker_mutex) {
                xSemaphoreTake(s_tracker_mutex, portMAX_DELAY);
                for (int i = 0; i < s_max_uavs; i++)
                    if (s_tracker[i].active) active++;
                xSemaphoreGive(s_tracker_mutex);
            }

            if (s_page == LCD_PAGE_LIST) {
                /* 列表页：User短按=下一个目标，长按=上一个目标；Boot短按=进入详情，长按=返回主页 */
                if (key == LCD_KEY_NEXT && active > 0) {
                    s_selection = (s_selection + 1) % active;
                } else if (key == LCD_KEY_PREV && active > 0) {
                    s_selection = (s_selection - 1 + active) % active;
                } else if (key == LCD_KEY_SELECT) {
                    s_detail_sel = s_selection;
                    s_page = LCD_PAGE_DETAIL;
                } else if (key == LCD_KEY_BACK) {
                    s_page = LCD_PAGE_HOME;
                    s_scroll_offset = 0;
                }
            } else if (s_page == LCD_PAGE_DETAIL) {
                /* 详情页：User短按=下一个目标，长按=上一个目标；Boot短按=返回列表，长按=主页 */
                if (key == LCD_KEY_NEXT && active > 0) {
                    s_detail_sel = (s_detail_sel + 1) % active;
                } else if (key == LCD_KEY_PREV && active > 0) {
                    s_detail_sel = (s_detail_sel - 1 + active) % active;
                } else if (key == LCD_KEY_SELECT) {
                    s_page = LCD_PAGE_LIST;
                    s_selection = s_detail_sel;
                } else if (key == LCD_KEY_BACK) {
                    s_page = LCD_PAGE_HOME;
                    s_scroll_offset = 0;
                }
            } else if (s_page == LCD_PAGE_SIM_CONFIG) {
                /* 模拟配置页：User短按=切换模式，Boot短按=启动/停止模拟；长按=返回主页 */
                if (key == LCD_KEY_NEXT) {
                    int new_mode = (s_sim_info.sim_flight_mode + 1) % 3;
                    s_sim_info.sim_flight_mode = (uint8_t)new_mode;
                    if (s_sim_mode_cb) s_sim_mode_cb(new_mode);
                } else if (key == LCD_KEY_PREV) {
                    int new_mode = (s_sim_info.sim_flight_mode + 2) % 3;
                    s_sim_info.sim_flight_mode = (uint8_t)new_mode;
                    if (s_sim_mode_cb) s_sim_mode_cb(new_mode);
                } else if (key == LCD_KEY_SELECT) {
                    if (s_sim_info.is_sim_running) {
                        if (s_sim_stop_cb) s_sim_stop_cb();
                    } else {
                        if (s_sim_start_cb) s_sim_start_cb();
                    }
                } else if (key == LCD_KEY_BACK) {
                    s_page = LCD_PAGE_HOME;
                    s_scroll_offset = 0;
                }
            } else if (s_page == LCD_PAGE_SIM_STATUS) {
                /* 模拟状态页：Boot短按=停止并返回，长按=主页 */
                if (key == LCD_KEY_SELECT) {
                    if (s_sim_info.is_sim_running && s_sim_stop_cb)
                        s_sim_stop_cb();
                    s_page = LCD_PAGE_HOME;
                } else if (key == LCD_KEY_BACK) {
                    s_page = LCD_PAGE_HOME;
                }
            } else {
                /* 主页按键逻辑（lcdfix14 优化）：
                 *   User短按 = 列表（有目标进列表，无目标提示扫描中）
                 *   User长按 = 模拟配置（手动进入模拟器）
                 *   Boot短按 = 列表（有目标进列表，无目标停在主页提示）
                 *   Boot长按 = 模拟配置
                 * 这样不会因为没目标就误进模拟器，模拟器需要明确长按才能进入。 */
                if (key == LCD_KEY_NEXT) {
                    if (active > 0) {
                        s_page = LCD_PAGE_LIST;
                    }
                    /* 无目标时停留在主页（状态栏会显示"搜索中"） */
                } else if (key == LCD_KEY_PREV) {
                    s_page = LCD_PAGE_SIM_CONFIG;
                    s_scroll_offset = 0;
                } else if (key == LCD_KEY_SELECT) {
                    if (active > 0) {
                        s_page = LCD_PAGE_LIST;
                    }
                    /* 无目标时停留在主页 */
                } else if (key == LCD_KEY_BACK) {
                    s_page = LCD_PAGE_SIM_CONFIG;
                    s_scroll_offset = 0;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/* ================================================================
 * 按键轮询
 * ================================================================ */
/* ================================================================
 * 熄屏（lcdfix16 替换原来的 light sleep）
 *
 * 之前的 light sleep + gpio_wakeup_enable 在 ESP32-C5 + WiFi/BLE
 * 同时运行的场景下不可靠（controller 持锁时按键事件被射频中断吞掉），
 * 表现为"跟关机一样唤不醒"。
 *
 * 改为只关背光、CPU/RF/LCD 全部保持运行。button_poll_task 一直在轮询，
 * 任意键 100% 秒唤醒。1.9 寸 IPS 背光占绝大头功耗，熄屏已经能省电。
 *
 * 调用方：双键同按 1.5 秒。进入此函数后会阻塞，直到任意键被按下并释放，
 * 然后恢复背光返回。refresh_task 在 s_display_off=true 时会自动跳过重绘。
 * ================================================================ */
static void enter_display_off(void)
{
    ESP_LOGW(TAG, "=== 熄屏（背光关，按键唤醒）===");

    /* 关闭背光 */
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

    s_display_off = true;

    /* 阻塞等待按键唤醒（button_poll_task 里会把 s_display_off 置 false） */
    while (s_display_off) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    ESP_LOGI(TAG, "Woken from display-off by key");
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 255);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    s_last_activity_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
}

/* ================================================================
 * 按键轮询
 *
 * 按键定义：
 *   User(GPIO0)：短按=NEXT，长按=PREV（在列表/详情=上下切换目标）
 *   Boot(GPIO28)：短按=SELECT（确认/进入），长按=BACK（返回主页）
 *   双键同按1.5秒=熄屏（背光关，任意键唤醒）
 * ================================================================ */
static void button_poll_task(void *arg)
{
    static uint32_t t_user_down = 0, t_boot_down = 0;
    static bool user_down = false, boot_down = false;
    static bool user_long_sent = false, boot_long_sent = false;
    static uint32_t both_down_time = 0;
    const uint32_t debounce = 30;
    const uint32_t long_press = 600;

    while (1) {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

        int user_lvl = gpio_get_level(BTN_USER_PIN);
        int boot_lvl = gpio_get_level(BTN_BOOT_PIN);

        /* ---- User 键 ---- */
        if (user_lvl == 0) {
            if (!user_down) {
                user_down = true;
                t_user_down = now;
                user_long_sent = false;
                s_last_activity_ms = now;
                /* 熄屏状态下任意键唤醒（等键释放后清标志，避免误触发操作） */
                if (s_display_off) {
                    while (gpio_get_level(BTN_USER_PIN) == 0 ||
                           gpio_get_level(BTN_BOOT_PIN) == 0) {
                        vTaskDelay(pdMS_TO_TICKS(20));
                    }
                    vTaskDelay(pdMS_TO_TICKS(50));
                    s_display_off = false;
                }
            } else if (!user_long_sent && (now - t_user_down > long_press)) {
                if (!s_both_pressed && !s_display_off)
                    lcd_display_send_key(LCD_KEY_PREV);
                user_long_sent = true;
            }
        } else if (user_down) {
            if (!user_long_sent && !s_both_pressed && !s_display_off &&
                (now - t_user_down > debounce)) {
                lcd_display_send_key(LCD_KEY_NEXT);
            }
            user_down = false;
        }

        /* ---- Boot 键 ---- */
        if (boot_lvl == 0) {
            if (!boot_down) {
                boot_down = true;
                t_boot_down = now;
                boot_long_sent = false;
                s_last_activity_ms = now;
                if (s_display_off) {
                    while (gpio_get_level(BTN_USER_PIN) == 0 ||
                           gpio_get_level(BTN_BOOT_PIN) == 0) {
                        vTaskDelay(pdMS_TO_TICKS(20));
                    }
                    vTaskDelay(pdMS_TO_TICKS(50));
                    s_display_off = false;
                }
            } else if (!boot_long_sent && (now - t_boot_down > long_press)) {
                if (!s_both_pressed && !s_display_off)
                    lcd_display_send_key(LCD_KEY_BACK);
                boot_long_sent = true;
            }
        } else if (boot_down) {
            if (!boot_long_sent && !s_both_pressed && !s_display_off &&
                (now - t_boot_down > debounce)) {
                lcd_display_send_key(LCD_KEY_SELECT);
            }
            boot_down = false;
        }

        /* ---- 双键同按检测（熄屏） ---- */
        if (user_lvl == 0 && boot_lvl == 0) {
            if (!s_both_pressed) {
                s_both_pressed = true;
                both_down_time = now;
            } else if (now - both_down_time > PWR_BOTH_HOLD_MS) {
                /* 等按键释放后再熄屏，避免立即被同一次按下唤醒 */
                while (gpio_get_level(BTN_USER_PIN) == 0 ||
                       gpio_get_level(BTN_BOOT_PIN) == 0) {
                    vTaskDelay(pdMS_TO_TICKS(20));
                }
                vTaskDelay(pdMS_TO_TICKS(100));
                /* lcdfix16: 改为纯熄屏，不再 light sleep。
                 * enter_display_off 内部会阻塞等待唤醒 */
                enter_display_off();
            }
        } else {
            s_both_pressed = false;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ================================================================
 * 公开 API
 * ================================================================ */
int lcd_display_init(void)
{
    ESP_LOGI(TAG, "=== LCD 模块初始化（lcdfix13）===");

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

    /* lcdfix15: 初始化 AXP2602 PMIC（真实电量） */
    axp2602_init();

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
    int mv = axp2602_read_battery_mv();
    if (mv <= 0) return -1;
    if (v) *v = (uint16_t)mv;
    return 0;
}
