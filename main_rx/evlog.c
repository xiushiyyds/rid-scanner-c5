/**
 * evlog.c — 本机证据日志实现
 *
 * 使用独立的 evlog 分区（2MB, subtype 0x80），裸 flash 读写。
 * 环形缓冲：head 指向下一个写入位置，count 记录有效条数。
 * 每次写入单条 64 字节记录，通过 esp_partition_write 追加。
 * 擦除按 4KB sector 进行，只有当 head 越过 sector 边界时才擦除下一个 sector。
 */
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_timer.h"
#include "evlog.h"
#include "gps_module.h"

static const char *TAG = "EVLOG";

static const esp_partition_t *s_part = NULL;
static uint32_t s_head = 0;       // 下一个写入的记录索引
static uint32_t s_count = 0;      // 有效记录数
static uint32_t s_last_erase_sector = UINT32_MAX;
static bool s_initialized = false;

#define PART_SIZE_BYTES   (2 * 1024 * 1024)
#define SECTOR_SIZE       4096
#define RECS_PER_SECTOR   (SECTOR_SIZE / EVLOG_RECORD_SIZE)

/* 头部元数据存在分区第一个 sector，记录数据从第二个 sector 开始。
 * 简化：直接用最后一个 sector 存 head/count，前面存数据。
 * 更简化：启动时扫描 magic 恢复 head/count。 */

static uint32_t total_data_records(void) {
    /* 前 N 个 sector 存数据，最后 1 个 sector 保留 */
    return ((PART_SIZE_BYTES - SECTOR_SIZE) / EVLOG_RECORD_SIZE);
}

static void load_meta(void) {
    /* 扫描数据区，统计有效记录并定位 head */
    uint32_t total = total_data_records();
    s_count = 0;
    s_head = 0;

    /* 快速扫描：找到最后一条 magic 有效的记录 */
    uint32_t last_valid = UINT32_MAX;
    for (uint32_t i = 0; i < total; i++) {
        evlog_record_t rec;
        esp_err_t err = esp_partition_read(s_part,
            (size_t)i * EVLOG_RECORD_SIZE, &rec, EVLOG_RECORD_SIZE);
        if (err == ESP_OK && rec.magic == EVLOG_MAGIC) {
            last_valid = i;
            s_count++;
        }
    }

    if (last_valid != UINT32_MAX) {
        s_head = (last_valid + 1) % total;
        /* 检查是否有环绕（记录数 == total 说明满了） */
        if (s_count < total) {
            /* 未满，count 就是扫描到的数量 */
        }
    } else {
        s_head = 0;
        s_count = 0;
    }

    ESP_LOGI(TAG, "Restored: count=%lu head=%lu total=%lu",
             (unsigned long)s_count, (unsigned long)s_head, (unsigned long)total);
}

int evlog_init(void) {
    if (s_initialized) return ESP_OK;

    s_part = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, (esp_partition_subtype_t)0x80, "evlog");
    if (!s_part) {
        ESP_LOGW(TAG, "evlog partition not found, evidence logging disabled");
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "evlog partition: %lu bytes at 0x%lx",
             (unsigned long)s_part->size, (unsigned long)s_part->address);

    load_meta();
    s_initialized = true;
    return ESP_OK;
}

static void ensure_sector_erased(uint32_t record_idx) {
    uint32_t sector = record_idx / RECS_PER_SECTOR;
    if (sector == s_last_erase_sector) return;

    /* 检查该 sector 是否已擦除（读第一个 word 判断） */
    uint32_t first_word = 0;
    esp_partition_read(s_part, sector * SECTOR_SIZE, &first_word, 4);
    if (first_word != 0xFFFFFFFF) {
        esp_err_t err = esp_partition_erase_range(s_part,
            sector * SECTOR_SIZE, SECTOR_SIZE);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "erase sector %lu failed: %s",
                     (unsigned long)sector, esp_err_to_name(err));
        }
    }
    s_last_erase_sector = sector;
}

int evlog_write(const uav_track_t *track) {
    if (!s_initialized || !s_part || !track) return ESP_ERR_INVALID_STATE;

    evlog_record_t rec;
    memset(&rec, 0, sizeof(rec));
    rec.magic = EVLOG_MAGIC;

    uint16_t flags = 0;
    if (track->is_dji) flags |= 0x0001;
    flags |= ((uint16_t)(track->alert_level & 0x07) << 1);

    /* SN */
    const char *sn = NULL;
    if (track->is_dji) {
        sn = track->dji_serial[0] ? track->dji_serial : NULL;
        rec.battery = track->dji_battery;
    } else {
        sn = track->basic_id.uas_id[0] ? track->basic_id.uas_id : NULL;
    }
    if (sn) {
        strncpy(rec.sn, sn, sizeof(rec.sn) - 1);
    }
    rec.ua_type = track->basic_id.ua_type;

    /* 坐标 */
    double lat = 0, lon = 0;
    float alt = 0, spd = 0;
    uint16_t hdg = 0;
    bool has_loc = false;

    if (track->is_dji) {
        lat = track->dji_latitude;
        lon = track->dji_longitude;
        alt = track->dji_altitude;
        spd = track->dji_speed_h;
        hdg = (uint16_t)((int)track->dji_heading % 360);
        if (lat != 0 && lon != 0) has_loc = true;
    } else if (track->location.valid) {
        lat = track->location.latitude;
        lon = track->location.longitude;
        alt = track->location.altitude_geo;
        spd = track->location.speed_horizontal;
        hdg = (uint16_t)((int)track->location.direction % 360);
        if (lat != 0 && lon != 0) has_loc = true;
    }

    if (has_loc) {
        flags |= 0x0010;
        rec.latitude = lat;
        rec.longitude = lon;
        rec.altitude = alt;
        rec.speed = spd;
        rec.heading = hdg;
    }

    rec.flags = flags;
    memcpy(rec.mac, track->mac, 6);
    rec.rssi = track->last_rssi;
    rec.channel = track->last_channel & 0x7F;

    /* GPS 时间戳 */
    rec.timestamp = (uint32_t)gps_get_unix_time();

    /* 写入 flash */
    uint32_t total = total_data_records();
    ensure_sector_erased(s_head);

    esp_err_t err = esp_partition_write(s_part,
        (size_t)s_head * EVLOG_RECORD_SIZE, &rec, EVLOG_RECORD_SIZE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "write failed: %s", esp_err_to_name(err));
        return err;
    }

    s_head = (s_head + 1) % total;
    if (s_count < total) s_count++;

    return ESP_OK;
}

uint32_t evlog_count(void) {
    return s_count;
}

int evlog_read(uint32_t index, evlog_record_t *rec) {
    if (!s_initialized || !s_part || !rec) return ESP_ERR_INVALID_STATE;
    uint32_t total = total_data_records();
    if (index >= s_count || index >= total) return ESP_ERR_NOT_FOUND;

    /* ring buffer: 最旧记录在 head - count */
    uint32_t pos;
    if (s_count < total) {
        pos = index;
    } else {
        pos = (s_head + index) % total;
    }

    esp_err_t err = esp_partition_read(s_part,
        (size_t)pos * EVLOG_RECORD_SIZE, rec, EVLOG_RECORD_SIZE);
    if (err != ESP_OK) return err;
    if (rec->magic != EVLOG_MAGIC) return ESP_ERR_NOT_FOUND;
    return ESP_OK;
}

int evlog_clear(void) {
    if (!s_initialized || !s_part) return ESP_ERR_INVALID_STATE;

    uint32_t data_sectors = (PART_SIZE_BYTES - SECTOR_SIZE) / SECTOR_SIZE;
    for (uint32_t s = 0; s < data_sectors; s++) {
        esp_partition_erase_range(s_part, s * SECTOR_SIZE, SECTOR_SIZE);
    }
    s_head = 0;
    s_count = 0;
    s_last_erase_sector = UINT32_MAX;
    ESP_LOGI(TAG, "evidence log cleared");
    return ESP_OK;
}

void evlog_status(uint32_t *oldest_ts, uint32_t *newest_ts, uint32_t *count) {
    if (count) *count = s_count;
    if (oldest_ts) {
        *oldest_ts = 0;
        if (s_count > 0) {
            evlog_record_t rec;
            if (evlog_read(0, &rec) == ESP_OK) *oldest_ts = rec.timestamp;
        }
    }
    if (newest_ts) {
        *newest_ts = 0;
        if (s_count > 0) {
            evlog_record_t rec;
            if (evlog_read(s_count - 1, &rec) == ESP_OK) *newest_ts = rec.timestamp;
        }
    }
}
