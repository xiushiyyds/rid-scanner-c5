/**
 * crid_tracker.c — 无人机追踪表（线程安全）
 */

#include <string.h>
#include "esp_log.h"
#include "crid_tracker.h"
#include "crid_json.h"

/* ---- 模块内部状态 ---- */

static SemaphoreHandle_t g_tracker_mutex = NULL;
static uav_track_t       g_uavs[MAX_TRACKED_UAVS];
static uint32_t          g_start_time_ms = 0;

/* ---- MAC 比较 ---- */

static inline bool mac_equal(const uint8_t *a, const uint8_t *b) {
    return memcmp(a, b, 6) == 0;
}

/* ---- 公开接口 ---- */

void crid_tracker_init(void) {
    memset(g_uavs, 0, sizeof(g_uavs));
    g_start_time_ms = esp_log_timestamp();

    g_tracker_mutex = xSemaphoreCreateMutex();
    if (g_tracker_mutex == NULL) {
        json_error("RID_TRACK", "Failed to create tracker mutex!");
    }
}

SemaphoreHandle_t crid_tracker_get_mutex(void) {
    return g_tracker_mutex;
}

uav_track_t *crid_tracker_find_or_create(const uint8_t *mac) {
    uav_track_t *free_slot = NULL;

    for (int i = 0; i < MAX_TRACKED_UAVS; i++) {
        if (g_uavs[i].active) {
            if (mac_equal(g_uavs[i].mac, mac)) {
                return &g_uavs[i];
            }
        } else if (free_slot == NULL) {
            free_slot = &g_uavs[i];
        }
    }

    if (free_slot) {
        memset(free_slot, 0, sizeof(uav_track_t));
        memcpy(free_slot->mac, mac, 6);
        free_slot->active = true;
        free_slot->first_seen_ms = esp_log_timestamp();
        free_slot->last_seen_ms = free_slot->first_seen_ms;
        odid_initUasData(&free_slot->uas_data);
        return free_slot;
    }

    return NULL;  // 追踪表已满
}

int crid_tracker_get_active_count(void) {
    int count = 0;
    for (int i = 0; i < MAX_TRACKED_UAVS; i++) {
        if (g_uavs[i].active) count++;
    }
    return count;
}

uav_track_t *crid_tracker_get_table(void) {
    return g_uavs;
}

void crid_tracker_update_rssi(uav_track_t *uav, int8_t rssi) {
    if (!uav) return;
    uav->rssi_hist[uav->rssi_hist_idx] = rssi;
    uav->rssi_hist_idx = (uav->rssi_hist_idx + 1) % 6;
    if (uav->rssi_hist_cnt < 6) uav->rssi_hist_cnt++;

    if (uav->rssi_hist_cnt >= 4) {
        /* 前半窗口均值 vs 后半窗口均值 */
        int old_sum = 0, new_sum = 0;
        int half = uav->rssi_hist_cnt / 2;
        int start = (uav->rssi_hist_idx - uav->rssi_hist_cnt + 6) % 6;
        for (int k = 0; k < half; k++) {
            old_sum += uav->rssi_hist[(start + k) % 6];
        }
        for (int k = half; k < uav->rssi_hist_cnt; k++) {
            new_sum += uav->rssi_hist[(start + k) % 6];
        }
        int old_avg = old_sum / half;
        int new_avg = new_sum / (uav->rssi_hist_cnt - half);
        int diff = new_avg - old_avg;  /* RSSI 变大(数值更正)=信号增强=接近 */
        if (diff >= 3) uav->rssi_trend = 1;        /* 接近 */
        else if (diff <= -3) uav->rssi_trend = -1; /* 远离 */
        else uav->rssi_trend = 0;
    }
}

void crid_tracker_cleanup(uint32_t timeout_ms) {
    uint32_t now = esp_log_timestamp();
    for (int i = 0; i < MAX_TRACKED_UAVS; i++) {
        if (!g_uavs[i].active) continue;
        uint32_t age_ms = now - g_uavs[i].last_seen_ms;
        if (age_ms > timeout_ms) {
            json_uav_timeout(g_uavs[i].mac);
            g_uavs[i].active = false;
        }
    }
}

/* ---- 跨传输方式去重：通过 UAS ID 查找 ---- */

uav_track_t *crid_tracker_find_by_uas_id(const char *uas_id) {
    if (uas_id == NULL || uas_id[0] == '\0') {
        return NULL;
    }

    for (int i = 0; i < MAX_TRACKED_UAVS; i++) {
        if (g_uavs[i].active && g_uavs[i].basic_id.valid) {
            if (strcmp(g_uavs[i].basic_id.uas_id, uas_id) == 0) {
                return &g_uavs[i];
            }
        }
    }
    return NULL;
}
