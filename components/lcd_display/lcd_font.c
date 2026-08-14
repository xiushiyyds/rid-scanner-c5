/**
 * lcd_font.c — 文本渲染引擎（8×16 ASCII + 16×16 CJK 混合排版）
 *
 * 支持中英文混排：
 *   - ASCII 字符 8×16 像素
 *   - CJK 字符 16×16 像素
 *   - 自动检测字符范围选择字体
 *   - 支持自动换行
 */

#include <string.h>
#include "lcd_font.h"
#include "font_data.h"
#include "esp_heap_caps.h"

/* 二分查找 CJK 字符索引 */
static int find_cjk_index(uint16_t codepoint)
{
    int lo = 0, hi = FONT_CJK_COUNT - 1;
    while (lo <= hi) {
        int mid = (lo + hi) / 2;
        uint16_t cp = font_cjk_codepoints[mid];
        if (cp == codepoint) return mid;
        if (cp < codepoint) lo = mid + 1;
        else hi = mid - 1;
    }
    return -1;
}

/* 判断是否为 CJK 字符（简单范围检测） */
static bool is_cjk(uint32_t cp)
{
    return (cp >= 0x4E00 && cp <= 0x9FFF) ||   /* CJK Unified Ideographs */
           (cp >= 0x3000 && cp <= 0x303F) ||    /* CJK Symbols and Punctuation */
           (cp >= 0xFF00 && cp <= 0xFFEF) ||    /* Halfwidth and Fullwidth Forms */
           (cp >= 0x2000 && cp <= 0x206F) ||    /* General Punctuation */
           (cp >= 0x25A0 && cp <= 0x25FF) ||    /* Geometric Shapes (●○■□▲▼ etc.) */
           (cp >= 0x2600 && cp <= 0x26FF) ||    /* Misc Symbols (★ etc.) */
           (cp >= 0x2190 && cp <= 0x21FF);      /* Arrows (→←↑↓ etc.) */
}

/* UTF-8 解码：返回 codepoint，*bytes_consumed 为消耗的字节数 */
static uint32_t utf8_decode(const char *s, int *bytes_consumed)
{
    uint8_t c = (uint8_t)s[0];
    if (c < 0x80) {
        *bytes_consumed = 1;
        return c;
    } else if ((c & 0xE0) == 0xC0) {
        *bytes_consumed = 2;
        return ((c & 0x1F) << 6) | ((uint8_t)s[1] & 0x3F);
    } else if ((c & 0xF0) == 0xE0) {
        *bytes_consumed = 3;
        return ((c & 0x0F) << 12) | (((uint8_t)s[1] & 0x3F) << 6) | ((uint8_t)s[2] & 0x3F);
    } else if ((c & 0xF8) == 0xF0) {
        *bytes_consumed = 4;
        return ((c & 0x07) << 18) | (((uint8_t)s[1] & 0x3F) << 12) |
               (((uint8_t)s[2] & 0x3F) << 6) | ((uint8_t)s[3] & 0x3F);
    }
    *bytes_consumed = 1;
    return '?';
}

/* 获取字符的显示宽度 */
int lcd_font_char_width(uint32_t codepoint)
{
    if (is_cjk(codepoint)) return FONT_CJK_W;
    return FONT_ASCII_W;
}

/* 获取字符串的像素宽度 */
int lcd_font_text_width(const char *text)
{
    int w = 0;
    const char *p = text;
    while (*p) {
        int consumed;
        uint32_t cp = utf8_decode(p, &consumed);
        w += lcd_font_char_width(cp);
        p += consumed;
    }
    return w;
}

/* 渲染单个字符到帧缓冲 */
static void render_glyph(uint16_t *fb, int fb_w, int fb_h,
                         int x, int y, const uint8_t *glyph,
                         int glyph_w, int glyph_h, uint16_t color)
{
    int bytes_per_row = (glyph_w + 7) / 8;
    for (int row = 0; row < glyph_h; row++) {
        for (int col = 0; col < glyph_w; col++) {
            int byte_idx = row * bytes_per_row + col / 8;
            int bit_idx = 7 - (col % 8);
            if (glyph[byte_idx] & (1 << bit_idx)) {
                int px = x + col;
                int py = y + row;
                if (px >= 0 && px < fb_w && py >= 0 && py < fb_h) {
                    fb[py * fb_w + px] = color;
                }
            }
        }
    }
}

/* 在帧缓冲上绘制单个字符 */
void lcd_font_draw_char(uint16_t *fb, int fb_w, int fb_h,
                        int x, int y, uint32_t codepoint, uint16_t color)
{
    if (is_cjk(codepoint)) {
        int idx = find_cjk_index(codepoint);
        if (idx >= 0) {
            render_glyph(fb, fb_w, fb_h, x, y,
                        font_cjk_16x16[idx], FONT_CJK_W, FONT_CJK_H, color);
        } else {
            /* 字库中没有的字，画一个占位方块 */
            for (int r = 0; r < FONT_CJK_H; r++) {
                for (int c = 0; c < FONT_CJK_W; c++) {
                    int px = x + c, py = y + r;
                    if (px >= 0 && px < fb_w && py >= 0 && py < fb_h) {
                        if (r < 2 || r >= FONT_CJK_H-2 || c < 2 || c >= FONT_CJK_W-2)
                            fb[py * fb_w + px] = color;
                    }
                }
            }
        }
    } else if (codepoint >= 0x20 && codepoint < 0x7F) {
        render_glyph(fb, fb_w, fb_h, x, y,
                    font_ascii_8x16[codepoint - 0x20],
                    FONT_ASCII_W, FONT_ASCII_H, color);
    }
}

/* 在帧缓冲上绘制字符串 */
int lcd_font_draw_text(uint16_t *fb, int fb_w, int fb_h,
                       int x, int y, const char *text, uint16_t color)
{
    int cx = x;
    const char *p = text;
    while (*p) {
        int consumed;
        uint32_t cp = utf8_decode(p, &consumed);
        int cw = lcd_font_char_width(cp);
        if (cx + cw > fb_w) break;
        lcd_font_draw_char(fb, fb_w, fb_h, cx, y, cp, color);
        cx += cw;
        p += consumed;
    }
    return cx - x;
}

/* 绘制居中字符串 */
int lcd_font_draw_text_centered(uint16_t *fb, int fb_w, int fb_h,
                                int y, const char *text, uint16_t color)
{
    int w = lcd_font_text_width(text);
    int x = (fb_w - w) / 2;
    if (x < 0) x = 0;
    return lcd_font_draw_text(fb, fb_w, fb_h, x, y, text, color);
}

/* 绘制右对齐字符串 */
int lcd_font_draw_text_right(uint16_t *fb, int fb_w, int fb_h,
                             int x_right, int y, const char *text, uint16_t color)
{
    int w = lcd_font_text_width(text);
    int x = x_right - w;
    if (x < 0) x = 0;
    return lcd_font_draw_text(fb, fb_w, fb_h, x, y, text, color);
}
