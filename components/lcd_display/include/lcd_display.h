/**
 * lcd_display.h — ST7789 LCD 显示模块（T-Display-C5 适配版）
 *
 * 硬件目标：LILYGO T-Display-C5
 *   - ESP32-C5 (RISC-V 32-bit @ 240MHz)
 *   - 1.9" ST7789 IPS LCD, 170×320, RGB565, SPI
 *   - CST816S 电容触摸 (I2C)
 *   - AXP2602 电池管理
 *   - 16MB Flash + 8MB PSRAM
 *
 * 功能：
 *   - 主页：无人机计数 + 扫描状态
 *   - 列表页：所有活跃无人机摘要（ID、RSSI、协议）
 *   - 详情页：选中无人机的完整信息
 *   - 状态栏：信道、RSSI、时间
 *   - 触摸切换页面（可选，默认用物理按键）
 */

#ifndef LCD_DISPLAY_H
#define LCD_DISPLAY_H

#include <stdint.h>
#include <stdbool.h>
#include "crid_rx_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 * 引脚配置 — LILYGO T-Display-C5
 *
 * 来源：官方 Wiki https://wiki.lilygo.cc/products/t-display-series/t-display-c5/
 * ================================================================ */

/* LCD SPI 引脚 */
#define LCD_PIN_SCLK        7       // SPI 时钟
#define LCD_PIN_MOSI        9       // SPI 数据（MOSI）
#define LCD_PIN_CS          26      // 片选
#define LCD_PIN_DC          8       // 数据/命令
#define LCD_PIN_RST         23      // 复位
#define LCD_PIN_BK          25      // 背光 PWM

/* 触摸 I2C 引脚（CST816S） */
#define TOUCH_PIN_SDA       2
#define TOUCH_PIN_SCL       3
#define TOUCH_PIN_INT       27      // 触摸中断
#define TOUCH_PIN_RST       24      // 触摸复位
#define TOUCH_I2C_ADDR      0x15

/* 物理按键 */
#define BTN_USER_PIN        0       // 用户按键（低有效）
#define BTN_BOOT_PIN        28      // Boot 按键（低有效）

/* 电源管理 I2C（AXP2602，与触摸共用 I2C 总线） */
#define AXP2602_I2C_ADDR    0x62    // 7位地址（8位写地址 0xC4）
#define AXP2602_INT_PIN     10

/* ================================================================
 * 屏幕参数 — T-Display-C5
 * ================================================================ */
#define LCD_WIDTH            170
#define LCD_HEIGHT           320
#define LCD_SPI_FREQ_HZ     (20 * 1000 * 1000)  // 20MHz (与 LILYGO 官方一致，已验证稳定)

/* 帧缓冲大小：170×320×2 = 108,800 bytes ≈ 106KB */
#define LCD_FB_SIZE         (LCD_WIDTH * LCD_HEIGHT * 2)

/* ================================================================
 * 显示页面定义
 * ================================================================ */
typedef enum {
    LCD_PAGE_HOME = 0,        // 主页：概览 + 扫描统计
    LCD_PAGE_LIST,            // 列表页：无人机列表
    LCD_PAGE_DETAIL,          // 详情页：选中无人机完整信息
    LCD_PAGE_SIM_CONFIG,      // 新增：模拟配置页
    LCD_PAGE_SIM_STATUS,      // 新增：模拟状态页
    LCD_PAGE_COUNT
} lcd_page_t;

/* ================================================================
 * 按键事件（用于页面切换和滚动）
 * ================================================================ */
typedef enum {
    LCD_KEY_NONE = 0,
    LCD_KEY_PREV,            // 上一项 / 上一页面
    LCD_KEY_NEXT,            // 下一项 / 下一页面
    LCD_KEY_SELECT,          // 确认 / 进入详情
    LCD_KEY_BACK,            // 返回
    LCD_KEY_POWER,           // 电源/睡眠
} lcd_key_event_t;

/* ================================================================
 * API 函数
 * ================================================================ */

/**
 * 初始化 LCD 显示模块
 * - 初始化 SPI 总线（使用 T-Display-C5 引脚）
 * - 初始化 ST7789 驱动（170×320）
 * - 开启背光
 * - 创建显示刷新任务
 * @return 0 成功, -1 失败
 */
int lcd_display_init(void);

/**
 * 设置显示刷新数据源
 * @param tracker_table  追踪表数组指针
 * @param tracker_mutex  追踪表互斥锁
 * @param max_uavs       最大追踪数量
 */
void lcd_display_set_source(uav_track_t *tracker_table,
                            void *tracker_mutex,
                            int max_uavs);

/**
 * 发送按键事件
 */
void lcd_display_send_key(lcd_key_event_t key);

/**
 * 切换页面
 */
void lcd_display_set_page(lcd_page_t page);

/**
 * 获取当前页面
 */
lcd_page_t lcd_display_get_page(void);

/**
 * 设置列表页当前选中索引
 */
void lcd_display_set_selection(int index);

/**
 * 获取当前选中索引
 */
int lcd_display_get_selection(void);

/**
 * 发送自定义显示命令（调试用）
 */
void lcd_display_command(const char *cmd);

/**
 * 显示BLE配对码（由BLE配对回调触发）
 * @param pin_code 4位数字配对码字符串
 */
void lcd_display_show_pair_pin(const char *pin_code);

/**
 * 模拟器操作回调类型（由 app_main 注册）
 */
typedef void (*sim_action_cb_t)(void);
typedef void (*sim_mode_cb_t)(int mode);

/**
 * 注册模拟器按键操作回调
 */
void lcd_display_register_sim_callbacks(sim_action_cb_t start,
                                         sim_action_cb_t stop,
                                         sim_mode_cb_t cycle_mode);

/**
 * 读取电池电压（通过 AXP2602）
 * @param voltage_mv  输出电压指针（mV）
 * @return 0 成功, -1 失败
 */
int lcd_display_get_battery_voltage(uint16_t *voltage_mv);

/**
 * lcdfix16: GPS 状态提供回调。
 *
 * lcd_display 是独立 component，不能直接 #include "gps_module.h"
 * （那会造成 main_rx ↔ lcd_display 组件循环依赖）。
 * 由 app_main 注册一个轻量回调，LCD 需要绘制状态栏时调用它拿 GPS 快照。
 * 返回 true 表示有有效定位，false 表示无定位/未接模块。
 * 若 sats_out 非 NULL，写入卫星数。
 */
typedef bool (*lcd_gps_provider_cb_t)(double *lat, double *lon,
                                       double *alt, int *sats_out);

void lcd_display_register_gps_provider(lcd_gps_provider_cb_t cb);

/**
 * 模拟器显示信息结构体（多目标版）
 */
typedef struct {
    bool is_sim_running;
    double sim_lat;
    double sim_lon;
    float sim_heading;
    float sim_alt;
    uint32_t sim_tx_count;
    uint8_t sim_channel;
    uint8_t sim_flight_mode;
    int sim_target_count;        // 模拟目标数量
    int8_t sim_tx_power;         // 发射功率 (0.25dBm 单位)
    char sim_uas_id[21];
    char sim_ssid[33];
} sim_display_info_t;

/**
 * 更新模拟器显示信息（由 app_main 调用）
 */
void lcd_display_set_sim_info(const sim_display_info_t *info);

#ifdef __cplusplus
}
#endif

#endif // LCD_DISPLAY_H
