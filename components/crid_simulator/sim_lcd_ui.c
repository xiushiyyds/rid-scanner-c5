/**
 * sim_lcd_ui.c — 模拟发射模式专属 LCD UI（v2.6.0）
 *
 * 显示：
 *   - 顶部：模拟发射 + 状态（发射/暂停）
 *   - 统计：已发 / 失败 / 轮次
 *   - 参数：目标数 / 速度 / 信道 / 品牌
 *   - 位置：经纬度 + 城市名
 *   - 底部：A键 设置 / B键 暂停
 *
 * 按键（运行中）：
 *   A 短按：循环切换显示页（状态 / 目标列表 / 关于）
 *   B 短按：暂停/恢复
 *
 * 设置模式：
 *   A 短按：切换设置项（目标数/速度/城市/品牌/信道模式）
 *   B 短按：调整当前项
 *   长按 B：退出设置
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_attr.h"

#include "sim_lcd_ui.h"
#include "sim_core.h"
#include "sim_cities.h"
#include "lcd_display.h"

/* 字库字符占位（确保 gen_font.py 提取到这些字） */
static const char *FONT_CHARS = "机型关于";

static const char *TAG = "SIM_UI";

/* 颜色（RGB565） */
#define UI_BG       0x0000
#define UI_WHITE    0xFFFF
#define UI_GREEN    0x07E0
#define UI_YELLOW   0xFFE0
#define UI_RED      0xF800
#define UI_CYAN     0x07FF
#define UI_GRAY     0x8410
#define UI_DIM      0x4208
#define UI_BLUE     0x001F
#define UI_HILIGHT  0x03DF  /* 高亮蓝条 */

#define UI_REFRESH_MS   500
#define BTN_DEBOUNCE_MS 30
#define BTN_LONG_MS     1500

typedef enum {
    PAGE_STATUS = 0,
    PAGE_TARGETS,
    PAGE_SETTINGS,
    PAGE_COUNT
} ui_page_t;

typedef enum {
    SET_COUNT = 0,
    SET_SPEED,
    SET_CITY,
    SET_BRAND,
    SET_CHANMODE,
    SET_COUNT_ITEMS
} setting_item_t;

static TaskHandle_t s_ui_task = NULL;
static TaskHandle_t s_btn_task = NULL;
static volatile bool s_should_stop = false;

static ui_page_t s_page = PAGE_STATUS;
static setting_item_t s_setting_sel = SET_COUNT;

/* 城市编辑子模式：true 时 A 切省、B 切市；长按 B 退出 */
static bool s_city_edit = false;
/* 当前选中的省份索引（城市编辑用） */
static int s_city_prov = -1;

/* 目标列表滚动偏移 */
static int s_list_scroll = 0;

/* ================================================================
 * 按键状态（独立于侦测模式的 btn_poll，因为侦测模式的 button task 未启动）
 * ================================================================ */
typedef struct {
    bool user_was_low;
    bool boot_was_low;
    uint32_t boot_down_ms;
    bool boot_long_fired;
} btn_state_t;

static void config_buttons(void) {
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << BTN_USER_PIN) | (1ULL << BTN_BOOT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
}

/* ================================================================
 * 设置项调整
 * ================================================================ */
static void adjust_setting(setting_item_t item, bool forward) {
    sim_config_t cfg;
    /* 读当前配置：从 stats 拿不全，直接用 get_default + 手动覆盖 */
    sim_stats_t st;
    sim_get_stats(&st);

    switch (item) {
    case SET_COUNT: {
        int cur = st.active_count;
        if (forward) cur += 10; else cur -= 10;
        if (cur < 1) cur = 1;
        if (cur > SIM_MAX_TARGETS) cur = SIM_MAX_TARGETS;
        sim_set_count(cur);
        break;
    }
    case SET_SPEED: {
        float cur = st.speed;
        if (forward) cur += 1.0f; else cur -= 1.0f;
        if (cur < 1.0f) cur = 1.0f;
        if (cur > 30.0f) cur = 30.0f;
        sim_set_speed(cur);
        break;
    }
    case SET_CITY: {
        /* B 短按：在当前省内切换城市，跨省循环 */
        int ci = sim_get_city_index();
        ci = sim_city_step_within_province(ci, forward ? 1 : -1);
        s_city_prov = sim_city_province(ci);
        sim_set_city(ci);
        break;
    }
    case SET_BRAND: {
        int b = (int)st.brand;
        if (forward) b++; else b--;
        if (b < 0) b = SIM_BRAND_MIXED;
        if (b > SIM_BRAND_MIXED) b = 0;
        sim_set_brand((sim_brand_t)b);
        break;
    }
    case SET_CHANMODE: {
        int m = (int)st.chan_mode;
        if (forward) m++; else m--;
        if (m < 0) m = SIM_CHAN_ROTATE_1_6_11;
        if (m > SIM_CHAN_ROTATE_1_6_11) m = 0;
        sim_set_chan_mode((sim_chan_mode_t)m);
        break;
    }
    default: break;
    }
}

/* ================================================================
 * 页面绘制
 * ================================================================ */
static void draw_bar(int y, uint16_t c) {
    lcd_fb_fillrect(0, y, LCD_WIDTH, 1, c);
}

static void draw_status_page(void) {
    sim_stats_t st;
    sim_get_stats(&st);

    const char *state_str = "?";
    uint16_t state_color = UI_GRAY;
    switch (st.state) {
        case SIM_STATE_RUNNING: state_str = "发射中"; state_color = UI_GREEN; break;
        case SIM_STATE_PAUSED:  state_str = "已暂停"; state_color = UI_YELLOW; break;
        case SIM_STATE_STOPPED: state_str = "已停止"; state_color = UI_RED; break;
        default: break;
    }

    char buf[48];

    /* 标题栏 */
    lcd_fb_fillrect(0, 0, LCD_WIDTH, 24, UI_BLUE);
    lcd_fb_text(4, 4, "RID 模拟发射", UI_WHITE);
    snprintf(buf, sizeof(buf), "%s", state_str);
    lcd_fb_text_right(LCD_WIDTH - 4, 4, buf, state_color);
    draw_bar(24, UI_CYAN);

    int y = 32;

    /* 发射统计 */
    lcd_fb_text(4, y, "已发帧数", UI_GRAY);
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)st.tx_count);
    lcd_fb_text_right(LCD_WIDTH - 4, y, buf, UI_WHITE);
    y += 20;

    lcd_fb_text(4, y, "失败", UI_GRAY);
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)st.tx_fail);
    lcd_fb_text_right(LCD_WIDTH - 4, y, buf, st.tx_fail > 0 ? UI_RED : UI_DIM);
    y += 20;

    lcd_fb_text(4, y, "轮次", UI_GRAY);
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)st.rounds);
    lcd_fb_text_right(LCD_WIDTH - 4, y, buf, UI_WHITE);
    y += 20;

    lcd_fb_text(4, y, "运行", UI_GRAY);
    unsigned h = st.uptime_s / 3600;
    unsigned m = (st.uptime_s % 3600) / 60;
    unsigned s = st.uptime_s % 60;
    if (h) snprintf(buf, sizeof(buf), "%uh%um", h, m);
    else if (m) snprintf(buf, sizeof(buf), "%um%us", m, s);
    else snprintf(buf, sizeof(buf), "%us", s);
    lcd_fb_text_right(LCD_WIDTH - 4, y, buf, UI_WHITE);
    y += 24;

    draw_bar(y, UI_DIM); y += 6;

    /* 参数区 */
    lcd_fb_text(4, y, "目标数", UI_GRAY);
    snprintf(buf, sizeof(buf), "%d", st.active_count);
    lcd_fb_text_right(LCD_WIDTH - 4, y, buf, UI_CYAN);
    y += 20;

    lcd_fb_text(4, y, "速度", UI_GRAY);
    snprintf(buf, sizeof(buf), "%.1f m/s", st.speed);
    lcd_fb_text_right(LCD_WIDTH - 4, y, buf, UI_CYAN);
    y += 20;

    lcd_fb_text(4, y, "信道", UI_GRAY);
    snprintf(buf, sizeof(buf), "%d (%s)", st.current_channel,
             sim_chan_mode_name(st.chan_mode));
    lcd_fb_text_right(LCD_WIDTH - 4, y, buf, UI_CYAN);
    y += 20;

    lcd_fb_text(4, y, "品牌", UI_GRAY);
    snprintf(buf, sizeof(buf), "%s", sim_brand_name(st.brand));
    lcd_fb_text_right(LCD_WIDTH - 4, y, buf, UI_CYAN);
    y += 24;

    draw_bar(y, UI_DIM); y += 6;

    /* 位置 */
    lcd_fb_text(4, y, "纬度", UI_GRAY); y += 16;
    snprintf(buf, sizeof(buf), "%.5f", st.base_lat);
    lcd_fb_text(4, y, buf, UI_WHITE); y += 18;
    lcd_fb_text(4, y, "经度", UI_GRAY); y += 16;
    snprintf(buf, sizeof(buf), "%.5f", st.base_lon);
    lcd_fb_text(4, y, buf, UI_WHITE); y += 22;

    /* 底部提示 */
    draw_bar(LCD_HEIGHT - 24, UI_DIM);
    lcd_fb_text_center(LCD_HEIGHT - 18, "A翻页 B暂停", UI_GRAY);
    lcd_fb_text(4, LCD_HEIGHT - 40, "v2.6.0", UI_DIM);
}

static void draw_targets_page(void) {
    /* 标题栏 */
    lcd_fb_fillrect(0, 0, LCD_WIDTH, 24, UI_BLUE);
    lcd_fb_text(4, 4, "目标列表", UI_WHITE);
    char buf[24];
    snprintf(buf, sizeof(buf), "%d架", sim_get_target_count());
    lcd_fb_text_right(LCD_WIDTH - 4, 4, buf, UI_CYAN);
    draw_bar(24, UI_CYAN);

    /* 显示最多 12 个目标的概要 */
    int y = 30;
    int count = sim_get_target_count();
    int visible = 12;
    int start = s_list_scroll;
    if (start > count - visible) start = count > visible ? count - visible : 0;

    for (int i = 0; i < visible && (start + i) < count; i++) {
        int idx = start + i;
        char sn[22] = "?";
        char model[24] = "?";
        uint8_t mac[6] = {0};
        uint8_t ch = 0;
        sim_get_target_info(idx, sn, sizeof(sn), mac, model, sizeof(model), &ch);

        snprintf(buf, sizeof(buf), "#%02d", idx + 1);
        lcd_fb_text(2, y, buf, UI_YELLOW);
        lcd_fb_text(34, y, model, UI_WHITE);
        snprintf(buf, sizeof(buf), "ch%d", ch);
        lcd_fb_text_right(LCD_WIDTH - 2, y, buf, UI_GREEN);
        y += 14;
        lcd_fb_text(34, y, sn, UI_GRAY);
        snprintf(buf, sizeof(buf), "%02X%02X%02X", mac[3], mac[4], mac[5]);
        lcd_fb_text_right(LCD_WIDTH - 2, y, buf, UI_DIM);
        y += 16;
        if (y > LCD_HEIGHT - 40) break;
    }

    draw_bar(LCD_HEIGHT - 24, UI_DIM);
    if (start > 0)
        lcd_fb_text(4, LCD_HEIGHT - 18, "^ 更多", UI_GRAY);
    else
        lcd_fb_text(4, LCD_HEIGHT - 18, "A:状态 B:暂停", UI_GRAY);
    if (start + visible < count)
        lcd_fb_text_right(LCD_WIDTH - 4, LCD_HEIGHT - 18, "更多 v", UI_GRAY);
}

static const char *setting_name(setting_item_t item) {
    switch (item) {
        case SET_COUNT:    return "目标数";
        case SET_SPEED:    return "速度";
        case SET_CITY:     return "城市";
        case SET_BRAND:    return "品牌";
        case SET_CHANMODE: return "信道模式";
        default:           return "?";
    }
}

static void setting_value_str(setting_item_t item, char *out, size_t len) {
    sim_stats_t st;
    sim_get_stats(&st);
    switch (item) {
        case SET_COUNT:
            snprintf(out, len, "%d", st.active_count);
            break;
        case SET_SPEED:
            snprintf(out, len, "%.1f m/s", st.speed);
            break;
        case SET_CITY: {
            int ci = sim_get_city_index();
            if (ci >= 0 && ci < SIM_CITY_COUNT) {
                if (s_city_edit)
                    snprintf(out, len, "%s·%s", g_sim_cities[ci].province,
                             g_sim_cities[ci].name);
                else
                    snprintf(out, len, "%s", g_sim_cities[ci].name);
            } else {
                snprintf(out, len, "自定义");
            }
            break;
        }
        case SET_BRAND:
            snprintf(out, len, "%s", sim_brand_name(st.brand));
            break;
        case SET_CHANMODE:
            snprintf(out, len, "%s", sim_chan_mode_name(st.chan_mode));
            break;
        default:
            out[0] = '?'; out[1] = 0; break;
    }
}

static void draw_settings_page(void) {
    lcd_fb_fillrect(0, 0, LCD_WIDTH, 24, UI_BLUE);
    lcd_fb_text(4, 4, "设置", UI_WHITE);
    if (s_city_edit && s_setting_sel == SET_CITY)
        lcd_fb_text_right(LCD_WIDTH - 4, 4, "A省 B市", UI_YELLOW);
    else
        lcd_fb_text_right(LCD_WIDTH - 4, 4, "A切换 B调", UI_GRAY);
    draw_bar(24, UI_CYAN);

    int y = 34;
    char val[32];

    for (int i = 0; i < SET_COUNT_ITEMS; i++) {
        setting_item_t item = (setting_item_t)i;
        bool sel = (item == s_setting_sel);

        if (sel) {
            lcd_fb_fillrect(0, y - 2, LCD_WIDTH, 22, UI_HILIGHT);
        }
        lcd_fb_text(6, y + 2, setting_name(item), sel ? UI_WHITE : UI_GRAY);
        setting_value_str(item, val, sizeof(val));
        lcd_fb_text_right(LCD_WIDTH - 6, y + 2, val, sel ? UI_YELLOW : UI_CYAN);
        y += 24;
        if (y > LCD_HEIGHT - 40) break;
    }

    draw_bar(LCD_HEIGHT - 24, UI_DIM);
    if (s_city_edit && s_setting_sel == SET_CITY)
        lcd_fb_text_center(LCD_HEIGHT - 18, "A切省 B切市 长按退出", UI_GRAY);
    else
        lcd_fb_text_center(LCD_HEIGHT - 18, "B长按退出", UI_GRAY);
}

static void draw_about_page(void) {
    lcd_fb_fill(UI_BG);
    lcd_fb_text_center(60, "RID 模拟发射", UI_CYAN);
    lcd_fb_text_center(86, "v2.6.1", UI_WHITE);
    lcd_fb_text_center(120, "GB42590 + DJI", UI_GRAY);
    lcd_fb_text_center(140, "多品牌 多信道", UI_GRAY);
    lcd_fb_text_center(160, "300目标 省市选择", UI_GRAY);
    lcd_fb_text_center(200, "ESP32-C5", UI_DIM);
    lcd_fb_text_center(220, "LILYGO T-Display", UI_DIM);
    lcd_fb_text_center(LCD_HEIGHT - 30, "A返回", UI_GRAY);
}

/* ================================================================
 * UI 刷新任务
 * ================================================================ */
static void ui_refresh_task(void *arg) {
    (void)arg;
    while (!s_should_stop) {
        lcd_fb_fill(UI_BG);
        switch (s_page) {
            case PAGE_STATUS:   draw_status_page(); break;
            case PAGE_TARGETS:  draw_targets_page(); break;
            case PAGE_SETTINGS: draw_settings_page(); break;
            default:            draw_about_page(); break;
        }
        lcd_flush();
        vTaskDelay(pdMS_TO_TICKS(UI_REFRESH_MS));
    }
    s_ui_task = NULL;
    vTaskDelete(NULL);
}

/* ================================================================
 * 按键任务
 * ================================================================ */
static void button_task(void *arg) {
    (void)arg;
    btn_state_t bs = {0};
    uint32_t last_tick = xTaskGetTickCount() * portTICK_PERIOD_MS;

    while (!s_should_stop) {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        int user_lvl = gpio_get_level(BTN_USER_PIN);
        int boot_lvl = gpio_get_level(BTN_BOOT_PIN);

        /* A 键：短按切换 */
        if (user_lvl == 0) {
            bs.user_was_low = true;
        } else if (bs.user_was_low) {
            bs.user_was_low = false;
            if (s_page == PAGE_SETTINGS) {
                if (s_city_edit && s_setting_sel == SET_CITY) {
                    /* 城市子模式：A 切换省份（跳到该省第一个城市） */
                    int np = s_city_prov + 1;
                    if (np >= sim_get_province_count()) np = 0;
                    s_city_prov = np;
                    int ci = sim_province_first_city(np);
                    sim_set_city(ci);
                } else if (s_setting_sel == SET_CITY) {
                    /* 首次按 A：进入城市编辑子模式 */
                    s_city_edit = true;
                    s_city_prov = sim_city_province(sim_get_city_index());
                } else {
                    s_city_edit = false;
                    s_setting_sel = (setting_item_t)((s_setting_sel + 1) % SET_COUNT_ITEMS);
                }
            } else {
                s_page = (ui_page_t)((s_page + 1) % PAGE_COUNT);
            }
        }

        /* B 键：短按确认/暂停，长按退出设置 */
        if (boot_lvl == 0) {
            if (!bs.boot_was_low) {
                bs.boot_was_low = true;
                bs.boot_down_ms = now;
                bs.boot_long_fired = false;
            } else if (!bs.boot_long_fired && (now - bs.boot_down_ms >= BTN_LONG_MS)) {
                bs.boot_long_fired = true;
                /* 长按：城市子模式先退出子模式；否则设置页退回状态页 */
                if (s_page == PAGE_SETTINGS) {
                    if (s_city_edit && s_setting_sel == SET_CITY) {
                        s_city_edit = false;
                    } else {
                        s_city_edit = false;
                        s_page = PAGE_STATUS;
                    }
                }
            }
        } else if (bs.boot_was_low) {
            bs.boot_was_low = false;
            if (!bs.boot_long_fired) {
                /* 短按 */
                if (s_page == PAGE_SETTINGS) {
                    if (s_setting_sel == SET_CITY) {
                        /* 城市项：B 短按在省内切市（自动进入子模式） */
                        s_city_edit = true;
                        if (s_city_prov < 0)
                            s_city_prov = sim_city_province(sim_get_city_index());
                        adjust_setting(SET_CITY, true);
                    } else {
                        adjust_setting(s_setting_sel, true);
                    }
                } else {
                    /* 暂停/恢复 */
                    if (sim_get_state() == SIM_STATE_RUNNING)
                        sim_pause();
                    else if (sim_get_state() == SIM_STATE_PAUSED)
                        sim_resume();
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(BTN_DEBOUNCE_MS));
    }
    s_btn_task = NULL;
    vTaskDelete(NULL);
}

/* ================================================================
 * 公开 API
 * ================================================================ */
int sim_lcd_ui_start(void) {
    s_should_stop = false;
    s_page = PAGE_STATUS;
    config_buttons();

    BaseType_t ok = xTaskCreatePinnedToCore(ui_refresh_task, "sim_ui",
                                4096, NULL, 4, &s_ui_task, 0);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create UI task");
        return -1;
    }
    ok = xTaskCreatePinnedToCore(button_task, "sim_btn",
                                2048, NULL, 6, &s_btn_task, 0);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create button task");
        s_should_stop = true;
        return -1;
    }
    ESP_LOGI(TAG, "Simulator UI started");
    return 0;
}

void sim_lcd_ui_stop(void) {
    s_should_stop = true;
    for (int i = 0; i < 30 && (s_ui_task || s_btn_task); i++)
        vTaskDelay(pdMS_TO_TICKS(20));
}
