#!/usr/bin/env python3
"""Generate bitmap font C arrays for LCD display (fixed brace version)."""
from PIL import Image, ImageDraw, ImageFont
import os

CJK_FONT_PATH = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
OUTPUT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "font_data.h")

CJK_CHARS = """
无人机侦测器就绪活跃目标协议国标大疆暂无信号扫描中按键翻页
列表详情编号信号信道高度速度经度纬度距离航向电量最后发现
模拟设置状态运行停止模式圆周往返搜索坐标
发射中已停止帧数地址
蓝牙配对请在手机输入
主页设置
已发现未发现米秒度毫瓦分贝
警告错误信息强弱
收发开关确认返回上下选择
飞行器类型距离方向
水平垂直爬升
经纬度位置
时间戳版本
设备名称序列号
电池电压温度
在线离线锁定
百分比帧计数
自动手动
北南西东西北东北西南东南
开机关机重启
"""
EXTRA_CHARS = "：，。、（）%/-+→←↑↓★●○◆◇▲▼■□"

def dedup(s):
    seen = set()
    r = []
    for c in s:
        if c not in seen and not c.isspace():
            seen.add(c); r.append(c)
    return r

CJK_LIST = dedup(CJK_CHARS + EXTRA_CHARS)
print(f"CJK chars: {len(CJK_LIST)}")

def render(font, char, w, h):
    img = Image.new('1', (w, h), 0)
    draw = ImageDraw.Draw(img)
    bbox = draw.textbbox((0, 0), char, font=font)
    tw = bbox[2] - bbox[0]
    th = bbox[3] - bbox[1]
    x = (w - tw) // 2 - bbox[0]
    y = (h - th) // 2 - bbox[1]
    draw.text((x, y), char, fill=1, font=font)
    return list(img.getdata())

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
    ascii_font = ImageFont.truetype(CJK_FONT_PATH, 13)
    cjk_font = ImageFont.truetype(CJK_FONT_PATH, 15)

    ascii_data = []
    for code in range(0x20, 0x7F):
        bm = render(ascii_font, chr(code), 8, 16)
        ascii_data.append((chr(code), glyph_bytes(bm, 8, 16)))

    cjk_data = []
    for c in CJK_LIST:
        bm = render(cjk_font, c, 16, 16)
        cjk_data.append((c, glyph_bytes(bm, 16, 16)))

    # 关键：按码点排序，lcd_font.c 的 find_cjk_index 使用二分查找
    cjk_data.sort(key=lambda x: ord(x[0]))

    with open(OUTPUT_PATH, 'w', encoding='utf-8') as f:
        f.write("""/*
 * font_data.h — Auto-generated bitmap fonts
 * 8x16 ASCII + 16x16 CJK
 * DO NOT EDIT MANUALLY
 */
#ifndef FONT_DATA_H
#define FONT_DATA_H
#include <stdint.h>

""")
        # ASCII
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

        # CJK
        f.write(f"#define FONT_CJK_COUNT {len(cjk_data)}\n")
        f.write(f"#define FONT_CJK_W 16\n")
        f.write(f"#define FONT_CJK_H 16\n")
        f.write("static const uint8_t font_cjk_16x16[][32] = {\n")
        for c, data in cjk_data:
            hexstr = ','.join(f'0x{b:02X}' for b in data)
            f.write(f"    {{{hexstr}}}, /* U+{ord(c):04X} '{c}' */\n")
        f.write("};\n\n")

        # Codepoints
        f.write("static const uint16_t font_cjk_codepoints[] = {\n")
        for i in range(0, len(cjk_data), 10):
            chunk = cjk_data[i:i+10]
            f.write("    " + ",".join(f"0x{ord(c):04X}" for c, _ in chunk) + ",\n")
        f.write("};\n\n")
        f.write("#endif\n")

    print(f"Written: {OUTPUT_PATH} ({os.path.getsize(OUTPUT_PATH)} bytes)")

if __name__ == "__main__":
    main()
