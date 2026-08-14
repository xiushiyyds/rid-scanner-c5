/**
 * lcd_font.h — 文本渲染引擎接口
 */
#ifndef LCD_FONT_H
#define LCD_FONT_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FONT_CJK_W    16
#define FONT_CJK_H    16
#define FONT_ASCII_W  8
#define FONT_ASCII_H  16

/* 字体行高（混合排版时取较大值） */
#define FONT_LINE_H   18

/* 计算单字符显示宽度 */
int lcd_font_char_width(uint32_t codepoint);

/* 计算字符串像素宽度 */
int lcd_font_text_width(const char *text);

/* 在帧缓冲上绘制单字符 */
void lcd_font_draw_char(uint16_t *fb, int fb_w, int fb_h,
                        int x, int y, uint32_t codepoint, uint16_t color);

/* 在帧缓冲上绘制字符串，返回绘制宽度 */
int lcd_font_draw_text(uint16_t *fb, int fb_w, int fb_h,
                       int x, int y, const char *text, uint16_t color);

/* 居中绘制字符串 */
int lcd_font_draw_text_centered(uint16_t *fb, int fb_w, int fb_h,
                                int y, const char *text, uint16_t color);

/* 右对齐绘制字符串，x_right 为右边界 */
int lcd_font_draw_text_right(uint16_t *fb, int fb_w, int fb_h,
                             int x_right, int y, const char *text, uint16_t color);

#ifdef __cplusplus
}
#endif

#endif /* LCD_FONT_H */
