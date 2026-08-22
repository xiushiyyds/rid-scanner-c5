# ESP32-C5 RID Scanner

基于 ESP32-C5 (LILYGO T-Display-C5) 的手持无人机 Remote ID 纯侦测固件。

## 功能特性

- **4协议RID识别**：支持 ASTM F3411、GB/T 42590、GB/T 46750、OpenDroneID 协议解析
- **DJI DroneID识别**：支持 DJI Beacon Vendor IE 解析、SN/机型/位置/高度/速度提取
- **WiFi Sniffer优先**：ch6 1500ms 主听，ch1/ch11 各 200ms 快速扫漏；发现目标后锁定 ch6
- **BLE 低占空比保活**：未连接手机 10% duty，手机连接后 40% duty，避免破坏 PTA 共存
- **ISR层去重**：同一 MAC 50ms 内重复帧直接丢弃，降低 burst 模式队列压力
- **LCD实时显示**：板载 170×320 屏幕展示侦测列表、目标详情、信号强度与高度
- **手机数据推送**：BLE NUS 通道输出 JSON，配合 `rid-monitor` 网页查看雷达/轨迹/KML
- **地理围栏告警**：自定义禁飞区域，实时距离检测与分级告警
- **证据日志**：新目标和周期性数据写入 Flash，便于事后追溯

## 硬件要求

| 组件 | 型号 | 说明 |
|------|------|------|
| 主控 | LILYGO T-Display-C5 | ESP32-C5 + 1.9" ST7789 LCD |
| GPS | ATGM336H / 兼容NMEA模块 | UART连接，9600bps（可选） |
| 天线 | 2.4GHz WiFi天线 | SMA接口 |

## 项目结构

```text
├── main_rx/              # RID接收、解析、追踪、BLE、LCD业务逻辑
├── components/
│   ├── lcd_display/      # ST7789 LCD显示驱动与UI
│   ├── opendroneid/      # OpenDroneID协议库
│   └── crid_common/      # 品牌/型号查表等公共组件
├── partition_table/      # 分区表
└── .github/workflows/    # GitHub Actions 云端编译
```

## 编译方式

### 本地编译

```bash
. $IDF_PATH/export.sh
idf.py set-target esp32c5
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

要求 ESP-IDF v5.5.2 或兼容版本。

### 云端编译

推送到 `main` 分支后，GitHub Actions 自动编译纯侦测固件，产物为 `firmware-detector`。

## 刷写

```bash
esptool --chip esp32-c5 --port /dev/ttyACM0 --baud 921600 write-flash 0x10000 remoteid_scanner.bin
```

## License

MIT
