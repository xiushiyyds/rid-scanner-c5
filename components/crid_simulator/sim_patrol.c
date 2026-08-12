/**
 * sim_patrol.c — 模拟器飞行路径计算引擎实现（多实例版）
 *
 * 支持 64 个独立路径引擎实例，每个实例有独立的基准坐标、速度、
 * 飞行模式和相位偏移，实现多无人机轨迹不重叠。
 */

#include "sim_patrol.h"
#include <math.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *TAG = "SIM_PATROL";

/* 多实例数组 */
static sim_patrol_instance_t s_instances[SIM_PATROL_MAX_INSTANCES];
static SemaphoreHandle_t s_patrol_mutex = NULL;

/* ================================================================
 * 多实例 API
 * ================================================================ */

void sim_patrol_init_instance(int id, double base_lat, double base_lon,
                               float speed, sim_patrol_mode_t mode,
                               float phase_offset) {
    if (id < 0 || id >= SIM_PATROL_MAX_INSTANCES) {
        ESP_LOGE(TAG, "Invalid instance id: %d", id);
        return;
    }

    if (s_patrol_mutex == NULL) {
        s_patrol_mutex = xSemaphoreCreateMutex();
    }
    if (s_patrol_mutex) {
        xSemaphoreTake(s_patrol_mutex, portMAX_DELAY);
    }

    sim_patrol_instance_t *inst = &s_instances[id];
    inst->base_lat = base_lat;
    inst->base_lon = base_lon;
    inst->speed_factor = speed;
    inst->mode = mode;
    inst->tick = 0;
    inst->active = true;
    inst->phase_offset = phase_offset;

    if (s_patrol_mutex) {
        xSemaphoreGive(s_patrol_mutex);
    }

    ESP_LOGD(TAG, "Instance %d init: lat=%.6f lon=%.6f speed=%.3f mode=%d phase=%.2f",
             id, base_lat, base_lon, speed, mode, phase_offset);
}

void sim_patrol_next(int id, double *out_lat, double *out_lon, float *out_heading) {
    if (id < 0 || id >= SIM_PATROL_MAX_INSTANCES) {
        if (out_lat) *out_lat = 0;
        if (out_lon) *out_lon = 0;
        if (out_heading) *out_heading = 0;
        return;
    }

    if (s_patrol_mutex) {
        xSemaphoreTake(s_patrol_mutex, portMAX_DELAY);
    }

    sim_patrol_instance_t *inst = &s_instances[id];
    if (!inst->active) {
        if (s_patrol_mutex) xSemaphoreGive(s_patrol_mutex);
        if (out_lat) *out_lat = inst->base_lat;
        if (out_lon) *out_lon = inst->base_lon;
        if (out_heading) *out_heading = 0;
        return;
    }

    inst->tick++;

    double base_lat = inst->base_lat;
    double base_lon = inst->base_lon;
    float speed = inst->speed_factor;
    sim_patrol_mode_t mode = inst->mode;
    float phase = inst->phase_offset;
    uint32_t tick = inst->tick;

    if (s_patrol_mutex) {
        xSemaphoreGive(s_patrol_mutex);
    }

    switch (mode) {
        case SIM_PATROL_CIRCLE: {
            double radius = 0.0005;
            double angle = (tick * speed * 0.1) + phase;
            *out_lat = base_lat + radius * sin(angle);
            *out_lon = base_lon + radius * cos(angle);
            *out_heading = (float)fmod((angle * 180.0 / M_PI) + 90.0, 360.0);
            break;
        }

        case SIM_PATROL_PINGPONG: {
            double max_distance = 0.001;
            double p = sin(tick * speed * 0.05 + phase);
            *out_lat = base_lat;
            *out_lon = base_lon + (max_distance * p);
            *out_heading = (cos(tick * speed * 0.05 + phase) >= 0) ? 90.0f : 270.0f;
            break;
        }

        case SIM_PATROL_S_SEARCH: {
            double scale_x = 0.001;
            double scale_y = 0.0003;
            double t = tick * speed * 0.02 + phase;
            *out_lat = base_lat + scale_y * sin(t * 4.0);
            *out_lon = base_lon + scale_x * sin(t);
            *out_heading = (float)fmod(t * 180.0 / M_PI, 360.0);
            break;
        }

        default:
            *out_lat = base_lat;
            *out_lon = base_lon;
            *out_heading = 0.0f;
            break;
    }
}

void sim_patrol_reset_instance(int id) {
    if (id < 0 || id >= SIM_PATROL_MAX_INSTANCES) return;

    if (s_patrol_mutex) {
        xSemaphoreTake(s_patrol_mutex, portMAX_DELAY);
    }
    s_instances[id].tick = 0;
    if (s_patrol_mutex) {
        xSemaphoreGive(s_patrol_mutex);
    }
    ESP_LOGD(TAG, "Instance %d reset", id);
}

void sim_patrol_update_instance(int id, double base_lat, double base_lon,
                                 float speed, sim_patrol_mode_t mode,
                                 float phase_offset) {
    if (id < 0 || id >= SIM_PATROL_MAX_INSTANCES) return;

    if (s_patrol_mutex) {
        xSemaphoreTake(s_patrol_mutex, portMAX_DELAY);
    }

    sim_patrol_instance_t *inst = &s_instances[id];
    inst->base_lat = base_lat;
    inst->base_lon = base_lon;
    inst->speed_factor = speed;
    inst->mode = mode;
    inst->phase_offset = phase_offset;

    if (s_patrol_mutex) {
        xSemaphoreGive(s_patrol_mutex);
    }

    ESP_LOGD(TAG, "Instance %d update: lat=%.6f lon=%.6f speed=%.3f mode=%d",
             id, base_lat, base_lon, speed, mode);
}

/* ================================================================
 * 向后兼容的旧接口（映射到 instance 0）
 * ================================================================ */

void sim_patrol_init(double base_lat, double base_lon, float speed, sim_patrol_mode_t mode) {
    sim_patrol_init_instance(0, base_lat, base_lon, speed, mode, 0.0f);
    ESP_LOGI(TAG, "Legacy init (instance 0): lat=%.6f lon=%.6f speed=%.2f mode=%d",
             base_lat, base_lon, speed, mode);
}

void sim_patrol_calculate_next(double *out_lat, double *out_lon, float *out_heading) {
    sim_patrol_next(0, out_lat, out_lon, out_heading);
}

void sim_patrol_update(double base_lat, double base_lon, float speed, sim_patrol_mode_t mode) {
    float phase = 0.0f;
    if (s_patrol_mutex) {
        xSemaphoreTake(s_patrol_mutex, portMAX_DELAY);
    }
    phase = s_instances[0].phase_offset;
    if (s_patrol_mutex) {
        xSemaphoreGive(s_patrol_mutex);
    }
    sim_patrol_update_instance(0, base_lat, base_lon, speed, mode, phase);
    ESP_LOGI(TAG, "Legacy update (instance 0): lat=%.6f lon=%.6f speed=%.2f mode=%d",
             base_lat, base_lon, speed, mode);
}

void sim_patrol_reset(void) {
    sim_patrol_reset_instance(0);
    ESP_LOGI(TAG, "Legacy reset (instance 0)");
}
