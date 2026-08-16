/**
 * evlog.h — 本机证据日志（flash 环形存储）
 *
 * 每条记录固定 64 字节，包含时间戳、MAC、SN、坐标、高度、速度、航向、RSSI、信道、告警等级。
 * 记录写入 evlog 分区（2MB），掉电不丢失。
 * 满了自动覆盖最旧记录（环形）。
 * 可通过 USB 命令导出。
 */
#ifndef EVLOG_H
#define EVLOG_H

#include <stdint.h>
#include <stdbool.h>
#include "crid_rx_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define EVLOG_RECORD_SIZE   64
#define EVLOG_MAGIC         0x4556   // "EV"
#define EVLOG_MAX_RECORDS   ((2 * 1024 * 1024) / EVLOG_RECORD_SIZE)  // ~32768 条

/* 证据记录结构体（固定 64 字节，紧凑排列） */
typedef struct __attribute__((packed)) {
    uint16_t magic;           // 0x4556
    uint16_t flags;           // bit0=is_dji, bit1-3=alert_level, bit4=has_location
    uint32_t timestamp;       // Unix 时间戳（秒，由 GPS 提供，0=未同步）
    uint8_t  mac[6];          // 源 MAC
    int8_t   rssi;            // 信号强度 dBm
    uint8_t  channel;         // 信道
    uint8_t  ua_type;         // UA 类型
    char     sn[24];          // 序列号/SN
    double   latitude;        // 纬度
    double   longitude;       // 经度
    float    altitude;        // 海拔高度 (m)
    float    speed;           // 水平速度 (m/s)
    uint16_t heading;         // 航向 (度, 0-360)
    uint8_t  battery;         // 电量 (DJI, 0=未知)
    uint8_t  reserved[5];     // 预留对齐
} evlog_record_t;

_Static_assert(sizeof(evlog_record_t) == EVLOG_RECORD_SIZE, "evlog record must be 64 bytes");

/**
 * 初始化证据日志模块
 * @return ESP_OK 或错误码
 */
int evlog_init(void);

/**
 * 写入一条证据记录
 * @param track  UAV 追踪条目
 * @return ESP_OK 或错误码
 */
int evlog_write(const uav_track_t *track);

/**
 * 获取已存储的记录数
 */
uint32_t evlog_count(void);

/**
 * 读取一条记录
 * @param index  记录索引 (0 = 最旧)
 * @param rec    [out] 记录
 * @return ESP_OK 或 ESP_ERR_NOT_FOUND
 */
int evlog_read(uint32_t index, evlog_record_t *rec);

/**
 * 清空所有记录
 */
int evlog_clear(void);

/**
 * 获取日志状态摘要
 * @param oldest_ts  [out] 最旧记录时间戳（秒）
 * @param newest_ts  [out] 最新记录时间戳（秒）
 * @param count      [out] 记录数
 */
void evlog_status(uint32_t *oldest_ts, uint32_t *newest_ts, uint32_t *count);

#ifdef __cplusplus
}
#endif

#endif // EVLOG_H
