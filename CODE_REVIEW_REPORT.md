# rid-scanner-c5 固件源代码审查报告

> 审查范围：全量 C 源码 + 构建配置  
> 硬件平台：ESP32-C5 (LILYGO T-Display-C5), ESP-IDF v5.4  
> 审查日期：2025年

---

## 问题汇总

| 严重程度 | 数量 |
|---------|------|
| 🔴 致命 | 8 |
| 🟡 中等 | 8 |
| 🟢 轻微 | 7 |
| **合计** | **23** |

---

## 🔴 致命问题 (8)

### BUG-01: BLE NUS Characteristic Flags 反转 — TX/RX 特性完全搞反

**文件：** `main_rx/crid_ble.c`  
**行号：** GATT 服务定义 (~行 240-250)

**问题描述：**  
NUS 的 TX 和 RX 特性的访问标志完全搞反了：
- TX characteristic（服务器→客户端）标记为 `BLE_GATT_CHR_F_WRITE`（写入），应为 `BLE_GATT_CHR_F_NOTIFY`
- RX characteristic（客户端→服务器）标记为 `BLE_GATT_CHR_F_NOTIFY`（通知），应为 `BLE_GATT_CHR_F_WRITE`

```c
// 当前代码（错误）：
{
    .uuid = &gatt_svr_chr_nus_tx_uuid.u,
    .flags = BLE_GATT_CHR_F_WRITE,    // ← 应该是 NOTIFY
},
{
    .uuid = &gatt_svr_chr_nus_rx_uuid.u,
    .flags = BLE_GATT_CHR_F_NOTIFY,   // ← 应该是 WRITE
},
```

**影响：** 标准 NUS 客户端无法发现正确的通知/写入特性，BLE 数据通道完全无法工作。

**修复建议：**
```c
{
    .uuid = &gatt_svr_chr_nus_tx_uuid.u,
    .flags = BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_READ,
},
{
    .uuid = &gatt_svr_chr_nus_rx_uuid.u,
    .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
},
```

---

### BUG-02: BLE 通知发送在错误的 Characteristic Handle 上

**文件：** `main_rx/crid_ble.c`  
**行号：** `ble_tx_task()` 函数 (~行 340)

**问题描述：**  
`ble_gattc_notify_custom()` 使用了 `g_nus_rx_handle` 发送通知，但通知应该从 TX characteristic 发送。当前代码查找并存储的是 RX handle，TX handle 从未被查找。

```c
int rc = ble_gattc_notify_custom(g_nus_conn_handle,
                                  g_nus_rx_handle, om);  // ← 应该是 TX handle
```

**影响：** 即使 FLAG 修正后，通知仍然发送在错误的 characteristic 上，客户端不会收到数据。

**修复建议：**
```c
// 在 ble_on_sync() 中同时查找 TX handle:
static uint16_t g_nus_tx_handle;  // 新增

ble_gatts_find_chr(&gatt_svr_svc_nus_uuid.u,
                    &gatt_svr_chr_nus_tx_uuid.u,
                    NULL, &g_nus_tx_handle);

// 在 ble_tx_task 中使用 TX handle:
ble_gattc_notify_custom(g_nus_conn_handle, g_nus_tx_handle, om);
```

---

### BUG-03: `uav->last_seen_ms` 从未更新 — 超时清理失效

**文件：** `main_rx/app_main.c`  
**行号：** `parser_task()` 函数 (~行 180-200)

**问题描述：**  
`parser_task` 处理每帧数据时更新了大量字段（`msg_count`, `last_rssi`, `last_channel` 等），但遗漏了关键的 `last_seen_ms` 字段。该字段在 `crid_tracker_find_or_create()` 中被 `memset(0)` 初始化后从未被赋值。

**影响：**
- `crid_tracker_cleanup(UAV_TIMEOUT_MS)` 使用 `now - last_seen_ms` 判断超时，`last_seen_ms` 为 0 导致所有条目在 ~49 天后才会超时（uint32_t 溢出），实际上永远不会被清理
- `json_uav_status()` 和 LCD 详情页计算的 `age_ms` 始终显示为系统运行时间（巨大值）
- 追踪表会被永不超时的死条目填满

**修复建议：** 在 parser_task 中 `uav->msg_count++` 之后添加：
```c
uav->last_seen_ms = esp_log_timestamp();
```

---

### BUG-04: 模式切换后 Sniffer Queue 句柄失效 — 解析器永久失聪

**文件：** `main_rx/crid_sniffer.c` + `main_rx/app_main.c`  
**行号：** `crid_sniffer_init()` 和 `parser_task()`

**问题描述：**  
`parser_task` 在启动时调用 `crid_sniffer_get_queue()` 获取队列句柄并缓存。但 `crid_sniffer_init()` 每次调用都执行 `xQueueCreate()` 创建新队列：

```c
// crid_sniffer.c - 每次 init 都创建新队列
g_sniffer_queue = xQueueCreate(SNIFFER_QUEUE_SIZE, sizeof(sniffer_msg_t));
```

当从模拟模式切回扫描模式时：
1. `crid_sniffer_init()` 创建 **新队列 B**
2. ISR 回调向 **队列 B** 写入数据
3. `parser_task` 仍然从 **旧队列 A** 读取 → 永远收不到数据

同时旧队列 A 从未被 `vQueueDelete()` 释放，造成内存泄漏。

**修复建议：**
```c
// 方案1：init 时检查队列是否已存在
if (g_sniffer_queue == NULL) {
    g_sniffer_queue = xQueueCreate(SNIFFER_QUEUE_SIZE, sizeof(sniffer_msg_t));
}

// 方案2：parser_task 每次循环都重新获取队列句柄
QueueHandle_t queue = crid_sniffer_get_queue();  // 移到 while(1) 内部或定期刷新
```

---

### BUG-05: Sniffer ISR 中 Vendor IE 长度校验不足 → 缓冲区溢出

**文件：** `main_rx/crid_sniffer.c`  
**行号：** `wifi_sniffer_cb()` ISR 回调 (~行 100-120)

**问题描述：**  
Vendor IE (ID=0xDD) 的长度校验只检查了 `len >= 5`，但后续代码计算：

```c
uint8_t odid_len = len - 5;   // 当 len=5 时 odid_len=0，安全
memcpy(msg.data, odid_payload, copy_len);  // msg.data[256]
```

问题在于 `ie_ptr[i + 5]` 的访问：当 `len == 5` 时，`ie_ptr[i + 2..i+6]` 中 `i+6 = i+2+len = i+2+5 = i+7`，而外层检查是 `i + 2 + len <= ie_total_len`，这意味着 `i+7 <= ie_total_len`，所以 `ie_ptr[i+5]` (即 oui_type) 的访问是安全的。

**但存在另一个问题：** `decodeMessagePack` 调用没有校验 `odid_len` 是否足够包含一个最小的 MessagePack header：

```c
int decode_ret = decodeMessagePack(&uas_data, (ODID_MessagePack_encoded *)odid_payload);
```

如果 `odid_len` 很小（如 1-2 字节），`decodeMessagePack` 会读取超出边界的数据。

**修复建议：**
```c
if (len >= 5 + 3) {  // 至少需要 OUI(3) + Type(1) + Counter(1) + 最小消息(25)
    // ...
    if (odid_len < 28) break;  // MessagePack header(3) + 至少1条消息(25)
    int decode_ret = decodeMessagePack(...);
}
```

---

### BUG-06: 分区表严重浪费 Flash 空间 + 潜在空间不足

**文件：** `partition_table/partitionTable.csv`

**问题描述：**  
```csv
nvs,data,nvs,0x9000,24K,
phy_init,data,phy,0xf000,4K,
factory,app,factory,0x10000,3M,
```

- 总 Flash 16MB (0x1000000)，但分区表只用到 0x310000 (3.2MB)
- **12.8MB Flash 完全未使用**
- 3MB app 分区对于包含 NimBLE + Wi-Fi + LCD + GPS + 模拟器的项目可能偏小
- `sdkconfig.defaults` 注释说"增大 app 分区到 3MB"，但未说明为何不利用剩余空间

**修复建议：**
```csv
nvs,data,nvs,0x9000,24K,
phy_init,data,phy,0xf000,4K,
factory,app,factory,0x10000,4M,
ota_0,app,ota_0,0x510000,4M,
ota_1,app,ota_1,0x910000,4M,
nvs_key,data,nvs_keys,0xD10000,4K,
storage,data,spiffs,0xD20000,256K,
```

或至少将 factory 扩大到 4-6MB。

---

### BUG-07: GPS 数据竞态 — 写入无保护，读取有保护

**文件：** `main_rx/gps_module.c`

**问题描述：**  
- `gps_get_data()` 使用 `taskENTER_CRITICAL()` 保护读取 ✓
- `gps_has_fix()` 直接读 `gps_state.valid` **无任何保护** ✗
- `gps_task` → `process_byte()` → `parse_gga()/parse_rmc()` 修改 `gps_state` 的多个字段 **无临界区保护** ✗

```c
// parse_gga 修改多个字段，没有原子性保证
gps_state.latitude = parse_nmea_lat(fields[1], fields[2]);   // 写入1
gps_state.longitude = parse_nmea_lon(fields[3], fields[4]);  // 写入2
gps_state.fix_quality = atoi(fields[5]);                       // 写入3
gps_state.satellites = atoi(fields[6]);                        // 写入4
gps_state.valid = (gps_state.fix_quality > 0 && ...);          // 写入5
```

读取方可能看到 latitude 已更新但 longitude 还是旧值的中间状态。

**修复建议：** 在 `parse_gga()` / `parse_rmc()` 完成所有字段更新后，一次性用 critical section 保护提交：
```c
static gps_data_t gps_pending;  // 本地暂存
// ... 填充 gps_pending ...
taskENTER_CRITICAL(NULL);
gps_state = gps_pending;  // 原子性提交
taskEXIT_CRITICAL(NULL);
```

---

### BUG-08: 模拟器 Beacon 帧 Vendor IE 长度字段多了 2 字节

**文件：** `components/crid_simulator/sim_encode.c`  
**行号：** `sim_encode_beacon_frame()` (~行 180-210)

**问题描述：**  
Vendor IE 的 length 字节计算错误：

```c
#define PACKED_MSG_TOTAL (3 + 1 + 1 + PACKED_MSG_COUNT * SIM_MESSAGE_SIZE)
// = 3 + 1 + 1 + 75 = 80... 不对
// 实际 = 3(header) + 1(OUI count??) + 1(vendor type) + 75 = 80

frame[pos++] = 0xDD;
frame[pos++] = 3 + 1 + 1 + PACKED_MSG_TOTAL;  // = 3 + 1 + 1 + 80 = 85??
```

重新计算实际写入的内容：
- OUI (3) + VendorType (1) + MsgCounter (1) + packed data (3+75=78) = **83 bytes**
- 但 IE length 字段值为 `3 + 1 + 1 + (3+1+1+75)` = `5 + 80` = **85**

IE length 比实际数据多 2 字节。标准 compliant 的 sniffer 会按 IE length 读取，导致读取越界到 FCS 或后续 IE。

**修复建议：**
```c
// IE content = OUI(3) + VendorType(1) + MsgCounter(1) + packed(78) = 83
frame[pos++] = 0xDD;
frame[pos++] = 3 + 1 + 1 + 3 + PACKED_MSG_COUNT * SIM_MESSAGE_SIZE;  // = 83
```

---

## 🟡 中等问题 (8)

### BUG-09: `crid_tracker_cleanup` 未释放 `ODID_UAS_Data` 内部资源

**文件：** `main_rx/crid_tracker.c`  
**行号：** `crid_tracker_cleanup()` (~行 70)

**问题描述：**  
清理超时条目时仅设置 `active = false`，但未调用 `odid_freeUasData()` 释放 `uas_data` 内部可能动态分配的内存。虽然当前 opendroneid 库的 `ODID_UAS_Data` 是纯结构体（无动态分配），但这是一个不好的模式，且未来库升级可能引入动态分配。

---

### BUG-10: Wi-Fi 混杂模式过滤器接收所有帧类型

**文件：** `main_rx/crid_sniffer.c`  
**行号：** `crid_sniffer_init()` (~行 200)

**问题描述：**
```c
wifi_promiscuous_filter_t filter = {
    .filter_mask = WIFI_PROMIS_FILTER_MASK_ALL  // 接收 MGMT + DATA + CTRL
};
```

ISR 回调只处理 MGMT 帧，但 `MASK_ALL` 导致 DATA 和 CTRL 帧也进入 ISR，浪费 CPU 周期并增加队列溢出风险。

**修复建议：**
```c
.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT
```

---

### BUG-11: JSON 输出缓冲区 512 字节可能溢出

**文件：** `main_rx/crid_json.c`  
**行号：** 多处 `DAT_PRINTF` / `DBG_PRINTF` 宏

**问题描述：**  
`json_uav_update()` 连续调用多个 `*_impl` 函数，每个函数内部使用 512 字节栈缓冲区。对于包含大量字段的完整 UAV 数据（特别是 GB46750 的 23 个字段），单次 `snprintf` 的 512 字节限制可能被超过，导致数据截断和 JSON 格式损坏。

`json_uav_gb46750_impl()` 中大量 `GB_FIELD_*` 宏展开后，单条 JSON 可能超过 512 字节。

**修复建议：** 将缓冲区增大到 1024 字节，或改用流式输出。

---

### BUG-12: `sim_tx_task` 读取 `s_sim_config.target_count` 无锁保护

**文件：** `components/crid_simulator/sim_core.c`  
**行号：** `sim_tx_task()` (~行 200)

**问题描述：**
```c
static void sim_tx_task(void *arg) {
    int count = s_sim_config.target_count;  // 函数入口读取一次
    // ...
    while (1) {
        for (int i = 0; i < count; i++) {  // 使用缓存的 count
```

`count` 在任务启动时读取一次，后续 `sim_update_config()` 修改 `target_count` 不会反映到运行中的任务。同时 `s_sim_config` 的其他字段在 `s_config_mutex` 保护下读取，但 `count` 不在保护范围内。

**修复建议：** 在 while 循环内部重新读取 `count`（加锁），或将其作为独立 volatile 变量。

---

### BUG-13: `ESP_ERROR_CHECK` 在 GPS 初始化中使用 — 模块故障将导致设备重启

**文件：** `main_rx/gps_module.c`  
**行号：** `gps_init()` (~行 220)

**问题描述：**
```c
ESP_ERROR_CHECK(uart_driver_install(GPS_UART_NUM, ...));
ESP_ERROR_CHECK(uart_param_config(GPS_UART_NUM, &uart_config));
ESP_ERROR_CHECK(uart_set_pin(GPS_UART_NUM, ...));
```

`ESP_ERROR_CHECK` 在错误时调用 `esp_system_abort()`，直接重启设备。GPS 是非关键外设，其故障不应导致整个扫描仪崩溃。

**修复建议：** 改为日志警告 + 优雅降级：
```c
if (uart_driver_install(...) != ESP_OK) {
    ESP_LOGE(TAG, "GPS UART init failed, GPS disabled");
    return;
}
```

---

### BUG-14: `parser_task` 超时清理逻辑有缺陷

**文件：** `main_rx/app_main.c`  
**行号：** `parser_task()` (~行 190-210)

**问题描述：**
```c
if (uav == NULL) {
    uint32_t now = esp_log_timestamp();
    if (now - last_cleanup_ms >= 10000) {
        crid_tracker_cleanup(UAV_TIMEOUT_MS);
        last_cleanup_ms = now;
        uav = crid_tracker_find_or_create(msg.src_mac);  // 重试
    }
    if (uav == NULL) {
        // ...
        continue;
    }
}
```

由于 BUG-03 (`last_seen_ms` 从未更新)，`crid_tracker_cleanup` 永远不会清理任何条目（所有条目的 age = now - 0 ≈ 极大值... 等等，实际上 `now - 0` 会大于 `UAV_TIMEOUT_MS`，所以所有条目都会被清理！）。

**修正分析：** 如果 `last_seen_ms == 0`，则 `age_ms = now - 0 = now`（可能数百万毫秒），远大于 `UAV_TIMEOUT_MS (30000)`。因此**所有条目都会在第一次 cleanup 时被删除**，包括活跃无人机。这意味着追踪表每 10 秒就被清空一次。

**这实际上比预期更严重** — 与 BUG-03 结合的效果是：要么永远不清理（如果 `last_seen_ms` 碰巧被设为当前时间），要么全部清理（如果保持为 0）。当前代码是后者。

---

### BUG-15: `ble_rx_command_handler` 使用栈上 256 字节缓冲区 + sscanf 无边界检查

**文件：** `main_rx/crid_ble.c`  
**行号：** `ble_rx_command_handler()` (~行 100)

**问题描述：**
```c
char cmd_buf[256];
if (len >= sizeof(cmd_buf)) len = sizeof(cmd_buf) - 1;
memcpy(cmd_buf, data, len);

// SIM_CONFIG 解析：sscanf 无输入验证
int parsed = sscanf(cmd_buf + 10, "%lf %lf %d %d %d %d",
                    &lat, &lon, &mode, &channel, &count, &tx_power);
```

`sscanf` 对 `%lf` 没有输入长度限制，恶意 BLE 客户端可发送极大数值导致浮点异常。`mode`/`channel`/`count` 等参数虽有后续范围检查，但 `sscanf` 返回的 `parsed` 值被 `(void)parsed` 忽略了。

---

### BUG-16: 模拟器 `sim_wifi_init` 使用 WPA2 密码但信标帧不含安全相关 IE

**文件：** `components/crid_simulator/sim_wifi.c`  
**行号：** `sim_wifi_init()` (~行 40)

**问题描述：**  
AP 配置使用 `WIFI_AUTH_WPA2_PSK` + 密码 `"12345678"`，但 `sim_encode_beacon_frame()` 手工构建的信标帧不包含 RSN IE（WPA2 安全信息元素）。这导致：
- 真实客户端扫描到的 Beacon 显示为开放网络
- 但 AP 模式实际要求 WPA2 认证
- 如果有客户端尝试连接，会因 Beacon 与 Probe Response 不一致而失败

由于模拟器主要使用 `esp_wifi_80211_tx` 发送原始帧而非通过 AP 框架，AP 配置的实际作用有限。

---

## 🟢 轻微问题 (7)

### BUG-17: 重复 `#include "crid_ble.h"`

**文件：** `main_rx/app_main.c`  
**行号：** 第 28 行和第 34 行

**问题描述：** `crid_ble.h` 被 include 了两次。虽然有 include guard 不会导致编译错误，但应清理。

---

### BUG-18: `crid_parser_gb42590.c` 和 `crid_parser_gb46750.c` 中 `le16`/`le32s` 函数重复定义

**文件：** `main_rx/crid_parser_gb42590.c`, `main_rx/crid_parser_gb46750.c`, `main_rx/crid_parser_astm.c`

**问题描述：** 三个解析器文件各自定义了 `le16()` 和 `le32s()` 静态函数。如果未来改为非 static 会产生命名冲突。应提取到公共头文件。

---

### BUG-19: `app_main` 中 `json_startup_banner` 被调用两次

**文件：** `main_rx/app_main.c`  
**行号：** `app_main()` 函数 (~行 280 和 ~行 330)

**问题描述：** 启动横幅在初始化早期和任务创建完成后各调用一次，输出重复的启动信息。

---

### BUG-20: `monitor_task` 统计计算在 60 秒窗口内精度较低

**文件：** `main_rx/app_main.c`  
**行号：** `monitor_task()` (~行 230)

**问题描述：**
```c
float pkt_rate = (total_pkts - last_packets) / 60.0f;
```

使用整数除法后转 float，如果差值较小（如 < 60），精度损失明显。应使用 `(float)(total_pkts - last_packets) / 60.0f`。

---

### BUG-21: `geofence_check` 的 `higher_alert` 使用静态数组，枚举值需严格匹配

**文件：** `main_rx/geofence.c`  
**行号：** `higher_alert()` (~行 180)

**问题描述：**
```c
static const int priority[] = {
    [ALERT_NONE]       = 0,
    [ALERT_ALTITUDE]   = 1,
    [ALERT_NOFLY]      = 4,
    [ALERT_AIRPORT]    = 3,
    [ALERT_RESTRICTED] = 2,
};
```

使用指定初始化器，但如果枚举值在 `geofence.h` 中被修改（如添加新值），此数组不会自动更新，导致越界访问。代码有边界检查 (`a >= sizeof(priority)/sizeof(priority[0])`)，但这意味着新增的告警类型会被静默忽略。

---

### BUG-22: LCD `render_detail_page` 时间计算使用了不同的时间源

**文件：** `components/lcd_display/lcd_display.c`  
**行号：** `render_detail_page()` (~行 450)

**问题描述：**
```c
(unsigned long)((esp_timer_get_time() / 1000 - s_tracker[idx].first_seen_ms) / 1000)
```

`esp_timer_get_time()` 返回微秒级时间戳（从启动开始），而 `first_seen_ms` 来自 `esp_log_timestamp()`（毫秒级）。两者使用不同的时间源，可能存在基准偏移。

**修复建议：** 统一使用 `esp_log_timestamp()` 或 `esp_timer_get_time() / 1000`。

---

### BUG-23: `sim_patrol.c` 中 `s_patrol_mutex` 初始化存在竞态

**文件：** `components/crid_simulator/sim_patrol.c`  
**行号：** `sim_patrol_init_instance()` (~行 40)

**问题描述：**
```c
if (s_patrol_mutex == NULL) {
    s_patrol_mutex = xSemaphoreCreateMutex();
}
```

无原子性检查-创建操作。如果两个任务同时调用 `sim_patrol_init_instance()`，可能创建两个 mutex，导致一个泄漏。

**修复建议：** 在初始化阶段（`sim_init()`）显式创建 mutex，而非在首次使用时懒初始化。

---

## 附录：架构级建议

### 1. ISR 安全优化
`wifi_sniffer_cb` 在 ISR 上下文中执行了 `decodeMessagePack`（OpenDroneID 库解码），这是**极其危险**的操作。ISR 应该尽可能短，只做数据拷贝和队列发送。建议：
- ISR 中仅做最小数据拷贝
- 将 `decodeMessagePack` 移到 `parser_task` 中执行

### 2. 线程模型优化
当前多个任务直接共享 `g_uavs[]` 全局数组，通过外部 mutex 保护。建议：
- 将 mutex 封装在 tracker 内部，对外提供线程安全的 API
- 避免外部模块直接操作 tracker 内部数据

### 3. 内存策略
- `sniffer_msg_t` 结构体约 300 字节，队列 32 深度 = ~10KB 内部 RAM
- 建议将 sniffer 队列数据也放入 PSRAM，或使用指针传递 + 独立分配

### 4. BLE UUID 兼容性
当前 NUS UUID 字节序（`BLE_UUID128_INIT` 参数）需确认与前端 app 的 UUID 匹配方式。`BLE_UUID128_INIT` 按小端序存储，如果前端使用标准 NUS UUID 字符串比较，需确保字节序一致。

---

*报告生成工具：静态代码审查*  
*审查覆盖：23 个源文件，约 5900 行代码*
