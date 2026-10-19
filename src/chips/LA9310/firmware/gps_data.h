#ifndef LIME_GPS_DATA_H
#define LIME_GPS_DATA_H

#include <stdbool.h>
#include <stdint.h>

// Copy of the M4 firmware's GPS state layout (arm_m4/src/ppsdo/gps_data.h, nmea_data.h), keep them in sync.

// Constellation, derived from the talker ID.
typedef enum nmea_system {
    NMEA_SYSTEM_UNKNOWN = 0,
    NMEA_SYSTEM_GPS, // GP
    NMEA_SYSTEM_GLONASS, // GL
    NMEA_SYSTEM_GALILEO, // GA
    NMEA_SYSTEM_BEIDOU, // GB, BD
    NMEA_SYSTEM_QZSS, // GQ
    NMEA_SYSTEM_COMBINED, // GN, a solution from several constellations
    NMEA_SYSTEM_COUNT
} nmea_system_t;

typedef struct nmea_time {
    uint16_t millisecond;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} nmea_time_t;

typedef struct nmea_date {
    uint16_t year; // full year, e.g. 2026
    uint8_t month; // 1-12
    uint8_t day; // 1-31
} nmea_date_t;

// Degrees scaled by 1e7, positive is north/east.
typedef struct nmea_position {
    int32_t latitude_1e7;
    int32_t longitude_1e7;
} nmea_position_t;

typedef enum gps_link_state {
    GPS_LINK_IDLE,
    GPS_LINK_DETECTING,
    GPS_LINK_FAIL, // Unable to determine baudrate
    GPS_LINK_RUNNING,
} gps_link_state_t;

// gps_data_aggregate_t.valid: sections holding data from the latest sentence that carries them
#define GPS_DATA_VALID_TIME (1 << 0)
#define GPS_DATA_VALID_DATE (1 << 1)
#define GPS_DATA_VALID_POSITION (1 << 2)
#define GPS_DATA_VALID_ALTITUDE (1 << 3) // altitude and geoid separation
#define GPS_DATA_VALID_SPEED (1 << 4)
#define GPS_DATA_VALID_COURSE (1 << 5)
#define GPS_DATA_VALID_DOP (1 << 6)

// Latest navigation data, merged from RMC, GGA, GSA, GSV and ZDA sentences.
typedef struct gps_data_aggregate {
    nmea_time_t time; // UTC
    nmea_date_t date; // UTC
    nmea_position_t position;
    int32_t altitude_mm;
    int32_t geoid_separation_mm;
    uint32_t speed_knots_milli;
    uint32_t course_centideg;

    uint8_t fix_quality; // GGA
    uint8_t fix_type; // GSA: 1 none, 2 2D, 3 3D
    uint8_t satellites_used;
    uint8_t satellites_in_view[NMEA_SYSTEM_COUNT];
    uint16_t pdop_centi;
    uint16_t hdop_centi;
    uint16_t vdop_centi;

    uint8_t valid; // GPS_DATA_VALID_*
    bool active; // RMC status 'A'
} gps_data_aggregate_t;

typedef struct gps_stats {
    uint32_t rx_bytes;
    uint32_t rx_line_errors; // UART framing, parity, overrun and break errors
    uint32_t rx_buffer_overflows;
    uint32_t sentences_ok;
    uint32_t checksum_errors;
    uint32_t framing_errors;
    uint32_t parse_errors;
} gps_stats_t;

typedef struct gps_state {
    uint32_t baudrate; // detected rate, 0 while not detected
    gps_data_aggregate_t data;
    gps_link_state_t link;
    gps_stats_t stats;
} gps_state_t;

#endif // LIME_GPS_DATA_H
