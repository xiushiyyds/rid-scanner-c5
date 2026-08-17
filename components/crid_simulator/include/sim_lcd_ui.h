/**
 * sim_lcd_ui.h — 模拟发射模式专属 LCD UI（v2.6.0）
 *
 * 显示：状态、已发帧数、失败数、目标数、速度、信道、位置、品牌
 * 按键：A 键切换设置项，B 键调整/确认（进入设置菜单后）
 *
 * 独立模块，不依赖 lcd_display.c 的侦测页面逻辑。
 */
#ifndef SIM_LCD_UI_H
#define SIM_LCD_UI_H

#include <stdint.h>
#include <stdbool.h>
#include "sim_core.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 初始化模拟器 UI（在 sim_start 之后调用）
 * 创建 LCD 刷新任务和按键轮询任务。
 */
int sim_lcd_ui_start(void);

/**
 * 停止模拟器 UI
 */
void sim_lcd_ui_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* SIM_LCD_UI_H */
