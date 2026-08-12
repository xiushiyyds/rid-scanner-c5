/**
 * GPS Module Header - ATGM336H
 */

#ifndef GPS_MODULE_H
#define GPS_MODULE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool valid;         // Has valid fix
    int fix_quality;    // 0=invalid, 1=GPS, 2=DGPS
    double latitude;    // WGS84 degrees
    double longitude;   // WGS84 degrees
    double altitude;    // Meters above sea level
    double speed_knots; // Speed in knots
    int satellites;     // Number of satellites tracked
    double hdop;        // Horizontal dilution of precision
} gps_data_t;

/**
 * Initialize GPS module (UART1, 9600 baud, GPIO5/GPIO6)
 * Call this once at startup.
 */
void gps_init(void);

/**
 * Get current GPS data snapshot (thread-safe)
 */
gps_data_t gps_get_data(void);

/**
 * Check if GPS has a valid position fix
 */
bool gps_has_fix(void);

/**
 * Calculate distance in meters between two WGS84 coordinates (Haversine)
 */
double gps_distance(double lat1, double lon1, double lat2, double lon2);

/**
 * Calculate bearing in degrees (0-360) from point 1 to point 2
 */
double gps_bearing(double lat1, double lon1, double lat2, double lon2);

#endif // GPS_MODULE_H
