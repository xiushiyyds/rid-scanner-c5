# ESP32-C5 RID Scanner

基于 ESP32-C5 (LILYGO T-Display-C5) 的手持无人机 Remote ID 侦测工具。

## 功能特性

- **4协议RID识别** - 支持 ASTM F3411、GB/T 42590、GB/T 46750、OpenDroneID 协议解析
- **地理围栏告警** - 自定义禁飞区域，实时距离检测与分级告警
- **RID伪造模拟器** - 多目标并行模拟，支持多种飞行模式（圆形/悬停/直线/8字）
- **Web BLE控制** - 通过手机浏览器无线连接设备，实时控制模拟器
- **LCD实时显示** - 板载屏幕展示侦测结果与目标信息
- **USB网络桥接** - 支持通过USB将数据转发到PC端监控系统

## 硬件要求

| 组件 | 型号 | 说明 |
|------|------|------|
| 主控 | LILYGO T-Display-C5 | ESP32-C5 + 1.9" LCD |
| GPS | ATGM336H / 兼容NMEA模块 | UART连接，9600bps |
| 天线 | 2.4GHz WiFi天线 | SMA接口 |

## 项目结构

```
├── main_rx/              # 主程序（RID接收/解析/追踪）
├── components/
│   ├── crid_simulator/   # RID模拟器组件
│   ├── lcd_display/      # LCD显示驱动
│   └── opendroneid/      # OpenDroneID协议库
├── partition_table/      # 分区表
└── docs/                 # GitHub Pages（Web BLE控制）
```

## 编译方式

### 本地编译

```bash
# 安装 ESP-IDF v5.2.1
. $IDF_PATH/export.sh

# 编译
idf.py set-target esp32c5
idf.py build

# 烧录
idf.py -p /dev/ttyACM0 flash monitor
```

### 云端编译 (GitHub Actions)

推送代码到 `main` 分支即可自动触发编译，固件产物可在 Actions 页面下载。

## Web BLE 控制

通过 GitHub Pages 部署的 Web 页面可以无线控制 ESP32-C5 的模拟器功能：

🔗 **[打开 Web BLE 控制面板](https://xiushiyyds.github.io/rid-scanner-c5/monitor.html)**

支持功能：
- BLE扫描与连接
- 目标数量调节（1-64）
- 发射功率调节（0.5-20 dBm）
- 飞行模式切换（圆形/悬停/直线/8字）
- Wi-Fi信道选择（CH 1/6/11）
- 实时状态查看

> ⚠️ Web BLE 需要 HTTPS 环境 + Chrome/Edge 浏览器

## License

MIT
