# NekolunaRID Scanner v1.3 — 自定义版

## 版本变更日志

### v1.3 — RID 伪造模拟器集成

**新增功能：**
- 新增 `crid_simulator` 组件，支持 GB42590 RID 信标伪造发射
- 支持三种飞行路径模式：圆形巡游、直线往返、S型搜索
- Wi-Fi AP 模式 + raw 802.11 Beacon 帧注入
- OUI=FA:0B:BC, VendorType=0x0D（符合 GB42590 标准）
- 1Hz 发射频率，MAC 地址 FA:0B:BC 前缀 + 随机后缀

**模式切换机制：**
- 设备支持两种工作模式：扫描模式（SCAN）和模拟模式（SIMULATE）
- ESP32-C5 单 Wi-Fi radio，不能同时扫描和发射，需要模式切换
- 扫描→模拟：停止 sniffer → 释放 Wi-Fi → 初始化 AP 模式 → 开始发射
- 模拟→扫描：停止发射 → 释放 AP → 重新初始化 sniffer

**LCD 显示新增页面：**
- SIM_CONFIG 页：显示/修改模拟参数（飞行模式、信道、坐标等）
- SIM_STATUS 页：实时显示位置、航向、发射计数、运行状态
- 主页增加 SIM MODE 入口提示

**BLE 控制命令：**
- `SIM_START` — 启动模拟发射
- `SIM_STOP` — 停止模拟发射
- `SIM_CONFIG <lat> <lon> <mode> <channel>` — 配置模拟参数
- `SIM_STATUS` — 查询模拟状态

**默认参数：**
- 大连市中心坐标 (38.9140, 121.6147)
- 高度 50m, 速度 5 m/s
- 信道 6
- 圆形巡游模式

**文件结构变更：**
```
components/crid_simulator/    ← 新增
├── CMakeLists.txt
├── include/
│   ├── sim_core.h            ← 模拟器对外 API
│   ├── sim_encode.h          ← GB42590 帧编码器
│   ├── sim_wifi.h            ← Wi-Fi AP 模式
│   └── sim_patrol.h          ← 飞行路径引擎
├── sim_core.c
├── sim_encode.c
├── sim_wifi.c
└── sim_patrol.c
```

**修改的文件：**
- `main_rx/app_main.c` — 模式切换逻辑
- `main_rx/crid_sniffer.c/h` — 新增 `crid_sniffer_deinit()`
- `main_rx/crid_ble.c/h` — 新增 BLE SIM 控制命令
- `components/lcd_display/include/lcd_display.h` — 新增 SIM 页面枚举
- `components/lcd_display/lcd_display.c` — 新增 SIM 页面渲染
- `CMakeLists.txt` — 版本号更新
- `main_rx/CMakeLists.txt` — 新增 `crid_simulator` 依赖

---

### v1.2 — 基础扫描功能

完整的多协议 RID 扫描功能，支持 GB42590/GB46750、ASTM F3411、ASD-STAN、DJI DroneID。

## 编译方法

```bash
# 安装 ESP-IDF v6.0.1
. $HOME/esp/esp-idf/export.sh

# 设置目标
cd NekolunaRIDScanner_custom
idf.py set-target esp32c5

# 编译
idf.py build

# 烧录
idf.py -p /dev/ttyACM0 flash monitor
```

## 硬件要求

- LILYGO T-Display-C5
- ESP32-C5 (RISC-V, Wi-Fi 6 双频, BLE 5, 16MB Flash, 8MB PSRAM)
- 1.9" ST7789 IPS LCD, 170×320
