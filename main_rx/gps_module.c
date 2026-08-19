/**
 * GPS Module - ATGM336H via UART1
 * GPIO5 (RX) <- GPS TX
 * GPIO6 (TX) -> GPS RX
 * Baud: 9600, NMEA-0183 protocol
 */

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "gps_module.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "GPS";

#define GPS_UART_NUM    UART_NUM_1
#define GPS_TX_PIN      6
#define GPS_RX_PIN      5
#define GPS_BAUD_RATE   9600
#define GPS_BUF_SIZE    1024

// NMEA sentence buffer
static char nmea_buf[256];
static int nmea_pos = 0;

// GPS state
static gps_data_t gps_state = {
    .valid = false,
    .fix_quality = 0,
    .latitude = 0.0,
    .longitude = 0.0,
    .altitude = 0.0,
    .speed_knots = 0.0,
    .satellites = 0,
    .hdop = 99.9,
};

static bool gps_lock_acquired = false;

/* GPS 同步的 Unix 时间戳（秒），0 = 未同步 */
static volatile time_t gps_unix_time = 0;

/* 保护 gps_state 的自旋锁（不能传 NULL 给 taskENTER_CRITICAL） */
static portMUX_TYPE gps_spinlock = portMUX_INITIALIZER_UNLOCKED;

/**
 * Parse NMEA latitude: "3854.12345" -> 38.902057
 */
static double parse_nmea_lat(const char *lat, const char *ns)
{
    if (strlen(lat) < 4) return 0.0;
    
    double degrees = (lat[0] - '0') * 10 + (lat[1] - '0');
    double minutes = atof(lat + 2);
    double result = degrees + minutes / 60.0;
    
    if (ns[0] == 'S') result = -result;
    return result;
}

/**
 * Parse NMEA longitude: "12133.12345" -> 121.552057
 */
static double parse_nmea_lon(const char *lon, const char *ew)
{
    if (strlen(lon) < 5) return 0.0;
    
    double degrees = (lon[0] - '0') * 100 + (lon[1] - '0') * 10 + (lon[2] - '0');
    double minutes = atof(lon + 3);
    double result = degrees + minutes / 60.0;
    
    if (ew[0] == 'W') result = -result;
    return result;
}

/**
 * Calculate checksum of NMEA sentence
 */
static uint8_t nmea_checksum(const char *sentence)
{
    uint8_t crc = 0;
    while (*sentence && *sentence != '*') {
        crc ^= *sentence++;
    }
    return crc;
}

/**
 * Parse GGA sentence (Global Positioning System Fix Data)
 * $GPGGA,hhmmss.ss,ddmm.mmmm,n,dddmm.mmmm,e,q,nn,h.h,a.a,M,g.g,M,,*cc
 */
static void parse_gga(const char *sentence)
{
    char fields[20][32];
    int field_count = 0;
    const char *p = sentence + 7; // skip "$GPGGA,"
    
    // Split by comma
    while (*p && field_count < 20) {
        int i = 0;
        while (*p && *p != ',' && *p != '*' && i < 31) {
            fields[field_count][i++] = *p++;
        }
        fields[field_count][i] = '\0';
        field_count++;
        if (*p == ',') p++;
        else break;
    }
    
    if (field_count < 9) return;
    
    // 使用本地暂存，最后原子性提交（避免读取方看到中间状态）
    gps_data_t pending;
    taskENTER_CRITICAL(&gps_spinlock);
    memcpy(&pending, &gps_state, sizeof(gps_data_t));
    taskEXIT_CRITICAL(&gps_spinlock);
    
    // Field 2: Latitude
    if (strlen(fields[1]) >= 4 && strlen(fields[2]) >= 1) {
        pending.latitude = parse_nmea_lat(fields[1], fields[2]);
    }
    
    // Field 4: Longitude
    if (strlen(fields[3]) >= 5 && strlen(fields[4]) >= 1) {
        pending.longitude = parse_nmea_lon(fields[3], fields[4]);
    }
    
    // Field 6: Fix quality (0=invalid, 1=GPS, 2=DGPS)
    pending.fix_quality = atoi(fields[5]);
    
    // Field 7: Number of satellites
    pending.satellites = atoi(fields[6]);
    
    // Field 8: HDOP
    pending.hdop = atof(fields[7]);
    
    // Field 9: Altitude
    pending.altitude = atof(fields[8]);
    
    // Update valid flag
    pending.valid = (pending.fix_quality > 0 && pending.satellites >= 3);
    
    // 原子性提交
    taskENTER_CRITICAL(&gps_spinlock);
    gps_state = pending;
    taskEXIT_CRITICAL(&gps_spinlock);
    
    if (pending.valid && !gps_lock_acquired) {
        gps_lock_acquired = true;
        ESP_LOGI(TAG, "GPS lock acquired! Lat=%.6f, Lon=%.6f, Sats=%d",
                 pending.latitude, pending.longitude, pending.satellites);
    }
}

/**
 * Parse RMC sentence (Recommended Minimum Navigation Information)
 * $GPRMC,hhmmss.ss,A,ddmm.mmmm,n,dddmm.mmmm,e,s.s,c.c,ddmmyy,d.d,e,m,*cc
 */
static void parse_rmc(const char *sentence)
{
    char fields[15][32];
    int field_count = 0;
    const char *p = sentence + 7; // skip "$GPRMC,"
    
    while (*p && field_count < 15) {
        int i = 0;
        while (*p && *p != ',' && *p != '*' && i < 31) {
            fields[field_count][i++] = *p++;
        }
        fields[field_count][i] = '\0';
        field_count++;
        if (*p == ',') p++;
        else break;
    }
    
    if (field_count < 10) return;
    
    // Field 1: Status (A=active, V=void)
    if (fields[1][0] != 'A') return;

    /* Field 0: UTC time hhmmss.ss → 解析时分秒
     * Field 9: date ddmmyy → 解析日期 */
    if (strlen(fields[0]) >= 6 && strlen(fields[9]) == 6) {
        int hh = (fields[0][0]-'0')*10 + (fields[0][1]-'0');
        int mm = (fields[0][2]-'0')*10 + (fields[0][3]-'0');
        int ss = (fields[0][4]-'0')*10 + (fields[0][5]-'0');
        int day   = (fields[9][0]-'0')*10 + (fields[9][1]-'0');
        int month = (fields[9][2]-'0')*10 + (fields[9][3]-'0');
        int year  = (fields[9][4]-'0')*10 + (fields[9][5]-'0') + 2000;
        /* 手动计算 UTC Unix 时间戳，避免 mktime 时区问题 */
        static const uint16_t dom[] = {0,31,59,90,120,151,181,212,243,273,304,334};
        uint32_t days = 0;
        for (int y = 1970; y < year; y++) {
            days += ((y % 4 == 0 && y % 100 != 0) || y % 400 == 0) ? 366 : 365;
        }
        days += dom[month - 1] + day - 1;
        if (month > 2 && ((year % 4 == 0 && year % 100 != 0) || year % 400 == 0))
            days++;
        gps_unix_time = (time_t)(days * 86400 + hh * 3600 + mm * 60 + ss);
    }
    
    // 使用本地暂存，原子性提交
    gps_data_t pending;
    taskENTER_CRITICAL(&gps_spinlock);
    memcpy(&pending, &gps_state, sizeof(gps_data_t));
    taskEXIT_CRITICAL(&gps_spinlock);
    
    // Field 3: Latitude
    if (strlen(fields[2]) >= 4 && strlen(fields[3]) >= 1) {
        pending.latitude = parse_nmea_lat(fields[2], fields[3]);
    }
    
    // Field 5: Longitude
    if (strlen(fields[4]) >= 5 && strlen(fields[5]) >= 1) {
        pending.longitude = parse_nmea_lon(fields[4], fields[5]);
    }

    // Field 7: Speed in knots
    pending.speed_knots = atof(fields[6]);

    /* v2.6.3: 不能仅凭 RMC status=A 就设 valid=true。
     * GGA 是定位质量的权威来源（fix_quality > 0 且 sats >= 3）。
     * RMC 的 A/V 标志可能在仅有1-2颗星时仍为 A，导致假定位。
     * 保留 pending 中从 GGA 继承的 valid 状态，不覆盖。 */

    taskENTER_CRITICAL(&gps_spinlock);
    gps_state = pending;
    taskEXIT_CRITICAL(&gps_spinlock);
}

/**
 * Process a complete NMEA sentence
 */
static void process_nmea_sentence(const char *sentence)
{
    // Verify checksum
    const char *star = strchr(sentence, '*');
    if (star && strlen(star) >= 3) {
        uint8_t expected = (uint8_t)strtol(star + 1, NULL, 16);
        uint8_t actual = nmea_checksum(sentence + 1);
        if (expected != actual) {
            ESP_LOGW(TAG, "NMEA checksum error: expected %02X, got %02X", expected, actual);
            return;
        }
    }
    
    // Route to parser
    if (strncmp(sentence, "$GPGGA", 6) == 0 || strncmp(sentence, "$GNGGA", 6) == 0) {
        parse_gga(sentence);
    } else if (strncmp(sentence, "$GPRMC", 6) == 0 || strncmp(sentence, "$GNRMC", 6) == 0) {
        parse_rmc(sentence);
    }
}

/**
 * Process incoming UART data byte by byte
 */
static void process_byte(uint8_t byte)
{
    if (byte == '$') {
        nmea_pos = 0;
        nmea_buf[nmea_pos++] = byte;
    } else if (byte == '\n' || byte == '\r') {
        if (nmea_pos > 6) {
            nmea_buf[nmea_pos] = '\0';
            process_nmea_sentence(nmea_buf);
        }
        nmea_pos = 0;
    } else {
        if (nmea_pos < 255) {
            nmea_buf[nmea_pos++] = byte;
        }
    }
}

/**
 * GPS task - runs continuously reading UART
 */
static void gps_task(void *arg)
{
    uint8_t data[128];
    
    while (1) {
        int len = uart_read_bytes(GPS_UART_NUM, data, sizeof(data), pdMS_TO_TICKS(100));
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                process_byte(data[i]);
            }
        }
        
        // Periodic status log every 30 seconds if no lock
        static int tick_count = 0;
        if (++tick_count % 300 == 0 && !gps_state.valid) {
            ESP_LOGI(TAG, "Waiting for GPS lock... Sats=%d, Fix=%d", 
                     gps_state.satellites, gps_state.fix_quality);
        }
    }
}

/**
 * Initialize GPS module
 */
void gps_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = GPS_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    /* GPS 是非关键外设，故障不应导致整个设备重启 */
    esp_err_t ret = uart_driver_install(GPS_UART_NUM, GPS_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPS UART driver install failed: %s", esp_err_to_name(ret));
        return;
    }
    ret = uart_param_config(GPS_UART_NUM, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPS UART param config failed: %s", esp_err_to_name(ret));
        return;
    }
    ret = uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, 
                        UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPS UART set pin failed: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "GPS initialized: UART1, TX=GPIO%d, RX=GPIO%d, baud=%d",
             GPS_TX_PIN, GPS_RX_PIN, GPS_BAUD_RATE);
    
    // Create GPS reading task
    xTaskCreate(gps_task, "gps_task", 4096, NULL, 5, NULL);
}

/**
 * Get current GPS data (thread-safe copy)
 */
gps_data_t gps_get_data(void)
{
    gps_data_t copy;
    taskENTER_CRITICAL(&gps_spinlock);
    memcpy(&copy, &gps_state, sizeof(gps_data_t));
    taskEXIT_CRITICAL(&gps_spinlock);
    return copy;
}

/**
 * Check if GPS has valid fix
 */
bool gps_has_fix(void)
{
    bool fix;
    taskENTER_CRITICAL(&gps_spinlock);
    fix = gps_state.valid;
    taskEXIT_CRITICAL(&gps_spinlock);
    return fix;
}

/**
 * Calculate distance between two GPS coordinates (Haversine formula, meters)
 */
double gps_distance(double lat1, double lon1, double lat2, double lon2)
{
    const double R = 6371000.0; // Earth radius in meters
    double dlat = (lat2 - lat1) * M_PI / 180.0;
    double dlon = (lon2 - lon1) * M_PI / 180.0;
    lat1 = lat1 * M_PI / 180.0;
    lat2 = lat2 * M_PI / 180.0;
    
    double a = sin(dlat/2) * sin(dlat/2) +
               cos(lat1) * cos(lat2) * sin(dlon/2) * sin(dlon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return R * c;
}

/**
 * Calculate bearing between two GPS coordinates (degrees)
 */
double gps_bearing(double lat1, double lon1, double lat2, double lon2)
{
    lat1 = lat1 * M_PI / 180.0;
    lat2 = lat2 * M_PI / 180.0;
    double dlon = (lon2 - lon1) * M_PI / 180.0;
    
    double x = sin(dlon) * cos(lat2);
    double y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon);
    double bearing = atan2(x, y) * 180.0 / M_PI;
    return fmod(bearing + 360.0, 360.0);
}

time_t gps_get_unix_time(void)
{
    if (gps_unix_time == 0) return 0;
    /* 基础时间 + 系统运行时间差（近似，GPS 每秒更新一次 RMC） */
    static time_t last_sync = 0;
    static int64_t last_sync_us = 0;
    time_t t = gps_unix_time;
    if (t != last_sync) {
        last_sync = t;
        last_sync_us = esp_timer_get_time();
    }
    if (last_sync_us > 0) {
        t += (time_t)((esp_timer_get_time() - last_sync_us) / 1000000);
    }
    return t;
}
