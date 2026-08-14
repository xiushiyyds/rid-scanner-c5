#!/usr/bin/env python3
"""Generate bitmap font C arrays for LCD display (lcdfix15).

改进：
- 自动从 lcd_display.c 提取所有 UI 中文字符，杜绝缺字
- 渲染算法：先在 24px 画布渲染，再 LANCZOS 降采样到 16px，二值化阈值 128
  → 笔画更均匀、不歪扭、不缺角
- ASCII 字号 12px，CJK 字号 18px（渲染时），降采样后更清晰
"""
from PIL import Image, ImageDraw, ImageFont
import os, re

CJK_FONT_CANDIDATES = [
    "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
]

def _find_font():
    for p in CJK_FONT_CANDIDATES:
        if os.path.exists(p):
            return p
    return CJK_FONT_CANDIDATES[-1]

CJK_FONT_PATH = _find_font()
OUTPUT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "font_data.h")

# 额外符号（非中文但UI会用到）
EXTRA_SYMBOLS = "：，。、（）%/-+→←↑↓★●○◆◇▲▼■□°"

def extract_ui_chars():
    """从 lcd_display.c 中提取所有字符串字面量里的 CJK 字符"""
    src_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "lcd_display.c")
    chars = set()
    if os.path.exists(src_path):
        with open(src_path, encoding='utf-8') as f:
            src = f.read()
        for m in re.finditer(r'"([^"]*)"', src):
            for c in m.group(1):
                cp = ord(c)
                if (0x4E00 <= cp <= 0x9FFF or 0x3000 <= cp <= 0x303F or
                    0xFF00 <= cp <= 0xFFEF or 0x2000 <= cp <= 0x206F or
                    0x25A0 <= cp <= 0x25FF or 0x2600 <= cp <= 0x26FF or
                    0x2190 <= cp <= 0x21FF or c in EXTRA_SYMBOLS):
                    chars.add(c)
    # 确保基础字
    for c in "无人机侦测器":
        chars.add(c)
    return sorted(chars)

CJK_LIST = extract_ui_chars()
# 加上额外符号
for c in EXTRA_SYMBOLS:
    if c not in CJK_LIST:
        CJK_LIST.append(c)
CJK_LIST = sorted(set(CJK_LIST))

print(f"CJK font: {CJK_FONT_PATH}")
print(f"CJK chars: {len(CJK_LIST)}")

RENDER_SCALE = 2  # 2x supersampling
GLYPH_W, GLYPH_H = 16, 16
RENDER_W, RENDER_H = GLYPH_W * RENDER_SCALE, GLYPH_H * RENDER_SCALE

def render_glyph(font, char, w, h):
    """用2x超采样+LANCZOS降采样渲染字形，得到更均匀的点阵"""
    big = Image.new('L', (RENDER_W, RENDER_H), 0)
    draw = ImageDraw.Draw(big)
    # 大字号渲染，居中
    font_size = int(GLYPH_H * 1.15) * RENDER_SCALE
    try:
        f = font.font_variant(size=font_size)
    except:
        f = ImageFont.truetype(CJK_FONT_PATH, font_size)
    bbox = draw.textbbox((0,0), char, font=f)
    tw = bbox[2] - bbox[0]
    th = bbox[3] - bbox[1]
    x = (RENDER_W - tw) // 2 - bbox[0]
    y = (RENDER_H - th) // 2 - bbox[1]
    draw.text((x, y), char, fill=255, font=f)
    # LANCZOS 降采样
    small = big.resize((w, h), Image.LANCZOS)
    # 二值化
    pixels = list(small.getdata())
    return [1 if p > 128 else 0 for p in pixels]

def render_ascii(font_char, char, w, h):
    """ASCII: 直接 8x16 渲染，字号 12"""
    img = Image.new('L', (w, h), 0)
    draw = ImageDraw.Draw(img)
    bbox = draw.textbbox((0,0), char, font=font_char)
    tw = bbox[2] - bbox[0]
    th = bbox[3] - bbox[1]
    x = (w - tw) // 2 - bbox[0]
    y = (h - th) // 2 - bbox[1] + 1
    draw.text((x, y), char, fill=255, font=font_char)
    pixels = list(img.getdata())
    return [1 if p > 128 else 0 for p in pixels]

def glyph_bytes(bitmap, w, h):
    bpr = (w + 7) // 8
    result = []
    for row in range(h):
        for bi in range(bpr):
            val = 0
            for bit in range(8):
                col = bi * 8 + bit
                if col < w and bitmap[row * w + col]:
                    val |= (1 << (7 - bit))
            result.append(val)
    return result

def main():
    ascii_font = ImageFont.truetype(CJK_FONT_PATH, 12)
    cjk_font = ImageFont.truetype(CJK_FONT_PATH, 18)

    ascii_data = []
    for code in range(0x20, 0x7F):
        bm = render_ascii(ascii_font, chr(code), 8, 16)
        ascii_data.append((chr(code), glyph_bytes(bm, 8, 16)))

    cjk_data = []
    for c in CJK_LIST:
        bm = render_glyph(cjk_font, c, GLYPH_W, GLYPH_H)
        cjk_data.append((c, glyph_bytes(bm, GLYPH_W, GLYPH_H)))

    cjk_data.sort(key=lambda x: ord(x[0]))

    with open(OUTPUT_PATH, 'w', encoding='utf-8') as f:
        f.write("""/*
 * font_data.h — Auto-generated bitmap fonts (lcdfix15)
 * 8x16 ASCII + 16x16 CJK (WenQuanYi Micro Hei, 2x supersampled)
 * DO NOT EDIT MANUALLY
 */
#ifndef FONT_DATA_H
#define FONT_DATA_H
#include <stdint.h>

""")
        f.write(f"#define FONT_ASCII_FIRST 0x20\n")
        f.write(f"#define FONT_ASCII_COUNT {len(ascii_data)}\n")
        f.write(f"#define FONT_ASCII_W 8\n")
        f.write(f"#define FONT_ASCII_H 16\n")
        f.write("static const uint8_t font_ascii_8x16[][16] = {\n")
        for c, data in ascii_data:
            label = c if c != '\\' else '\\\\'
            hexstr = ','.join(f'0x{b:02X}' for b in data)
            f.write(f"    {{{hexstr}}}, /* '{label}' */\n")
        f.write("};\n\n")

        f.write(f"#define FONT_CJK_COUNT {len(cjk_data)}\n")
        f.write(f"#define FONT_CJK_W 16\n")
        f.write(f"#define FONT_CJK_H 16\n")
        f.write("static const uint8_t font_cjk_16x16[][32] = {\n")
        for c, data in cjk_data:
            hexstr = ','.join(f'0x{b:02X}' for b in data)
            f.write(f"    {{{hexstr}}}, /* U+{ord(c):04X} '{c}' */\n")
        f.write("};\n\n")

        f.write("static const uint16_t font_cjk_codepoints[] = {\n")
        for i in range(0, len(cjk_data), 10):
            chunk = cjk_data[i:i+10]
            f.write("    " + ",".join(f"0x{ord(c):04X}" for c, _ in chunk) + ",\n")
        f.write("};\n\n")
        f.write("#endif\n")

    print(f"Written: {OUTPUT_PATH} ({os.path.getsize(OUTPUT_PATH)} bytes)")

if __name__ == "__main__":
    main()
