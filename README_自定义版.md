# NekolunaRIDScanner 自定义版（公安图侦增强版）

## 修改概述

基于 [NekolunaRIDScanner](https://github.com/Nekoluna-dot/NekolunaRIDScanner) 项目，面向国内公安图侦实战需求做了以下增强：

### 新增功能
1. **LCD 屏幕显示**（ST7789 240x320 TFT）
   - 主页：实时无人机计数、扫描统计、协议分布
   - 列表页：所有活跃无人机摘要（MAC/ID、RSSI、协议类型）
   - 详情页：坐标、高度、速度、航向、操作员位置、GB46750专有字段
   - 3个GPIO按键控制页面切换和滚动

2. **国外标准增强识别**
   - ASTM F3411-22a（美国）
   - ASD-STAN prEN 4709-002（欧洲）：自动检测EU分类字段区分
   - GB 42590-2023（中国国标）
   - GB 46750-2025（中国新标）
   - DJI OUI 监控标记（60:60:1F / 48:1C:B9 / 34:D2:62）

3. **协议自动识别**
   - GB 46750（magic 0xFF）→ GB 42590（magic 0xF1）→ ASTM（magic 0xF2）→ ASD-STAN（ASTM+EU字段）
   - DJI私有协议OUI标记 + 原始数据记录（供后续逆向）

### 硬件平台
- **主控**: ESP32-C5（WiFi 6 双频 + BLE 5.0）
- **屏幕**: ST7789 240x320 TFT（SPI接口）
- **按键**: 3个GPIO按键（上/下/确认）

### 文件结构
```
NekolunaRIDScanner_custom/
├── CMakeLists.txt                    # 根构建文件
├── components/
│   ├── lcd_display/                  # [新增] LCD 显示组件
│   │   ├── CMakeLists.txt
│   │   ├── include/lcd_display.h
│   │   └── lcd_display.c
│   └── opendroneid/                  # [保留] OpenDroneID 库
├── main_rx/
│   ├── app_main.c                    # [修改] 集成LCD + 按键
│   ├── crid_parser_common.c          # [修改] ASD-STAN检测 + DJI标记
│   ├── crid_parser_astm.c            # [保留]
│   ├── crid_parser_gb42590.c         # [保留]
│   ├── crid_parser_gb46750.c         # [保留]
│   ├── crid_rx_types.h               # [保留] 已含ASD-STAN枚举
│   ├── crid_sniffer.c                # [保留]
│   ├── crid_tracker.c                # [保留]
│   ├── crid_display.c                # [保留] 终端输出
│   ├── crid_json.c                   # [保留]
│   └── crid_ble.c                    # [保留]
└── partition_table/                  # [保留]
```

## 快速开始

详见：
- `编译指南.md` - ESP-IDF 环境搭建 + 编译步骤
- `硬件接线指南.md` - LCD屏幕和按键接线

## 原始协议
MIT License - 原作者 Nekoluna
