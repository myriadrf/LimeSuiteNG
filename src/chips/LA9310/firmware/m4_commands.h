#ifndef LIME_M4_COMMANDS_H
#define LIME_M4_COMMANDS_H

#include "gps_data.h"

enum M4_Command {
    // first commands with dedicated timers
    LIME_M4_TX_DAC_ALLOWED,
    LIME_M4_RO0_ADC_ALLOWED,
    LIME_M4_RO1_ADC_ALLOWED,
    LIME_M4_RX0_ADC_ALLOWED,
    LIME_M4_RX1_ADC_ALLOWED,
    LIME_M4_TX_BAND_SWITCH,
    LIME_M4_PA_ENABLE,
    LIME_M4_TX_WINDOW,
    LIME_M4_TIMED_CMD_COUNT = LIME_M4_TX_WINDOW,
    LIME_M4_SCHEDULE_CMD,

    LIME_M4_EMPTY, // 10
    LIME_M4_LMS64C_PACKET,
    LIME_M4_SET_SYSTEM_CLOCK_FREQUENCY,
    LIME_M4_GET_REFERENCE_CLOCK_FREQUENCY,
    LIME_M4_SET_REFERENCE_CLOCK_FREQUENCY,
    LIME_M4_BOOTLOADER,
    LIME_M4_HEARTHBEAT,

    LIME_M4_HARDWARE_COUNTER_GET,
    LIME_M4_HARDWARE_COUNTER_RESET,
    LIME_M4_DIGITAL_LOOPBACK,

    LIME_M4_TX_SET_OVERSAMPLE, //20
    LIME_M4_RX_SET_OVERSAMPLE,
    LIME_M4_GET_FEATURES,
    LIME_M4_DMA,
    LIME_M4_IQSTREAM_CTRL,
    LIME_M4_RX_CHANNEL_SELECT,
    LIME_M4_RX_GET_OVERSAMPLE,
    LIME_M4_TX_GET_OVERSAMPLE,
    LIME_M4_PPSDO_CTRL,
    LIME_M4_GPS_GET_STATUS,
    LIME_M4_GPS_SEND
};

struct tx_dac_allowed_payload {
    uint64_t vspa_cmd;
};

struct tx_band_switch_payload {
    uint32_t tx_rf_switch_control;
};

struct iqstream_control_payload {
    uint32_t enable;
    uint32_t rxmask;
    uint32_t txmask;
};

struct simple_response_payload {
    uint32_t status;
};

struct iqstream_channel_select {
    uint8_t lane;
    uint8_t channel;
};

struct iqstream_channel_config {
    uint8_t channel;
    uint8_t oversample_pow2;
};

struct ppsdo_control_payload {
    uint32_t enable;
};

struct ppsdo_status_payload {
    uint32_t result; // lime_Result
    uint32_t enabled;
    uint32_t tune_state; // ppsdo_tune_state_t
    uint32_t xo_dac_value;
};

// GPS payloads, sized to fit la9310_sw_cmd_desc.data (128 bytes)
enum gps_link_state_payload {
    GPS_LINK_STATE_IDLE,
    GPS_LINK_STATE_DETECTING,
    GPS_LINK_STATE_FAIL, // no baud rate worked
    GPS_LINK_STATE_RUNNING,
};

// Index into gps_status_payload.data.satellites_in_view
enum gps_system_payload {
    GPS_SYSTEM_UNKNOWN,
    GPS_SYSTEM_GPS,
    GPS_SYSTEM_GLONASS,
    GPS_SYSTEM_GALILEO,
    GPS_SYSTEM_BEIDOU,
    GPS_SYSTEM_QZSS,
    GPS_SYSTEM_COMBINED,
    GPS_SYSTEM_COUNT_PAYLOAD = 8
};

// LIME_M4_GPS_GET_STATUS response
struct gps_status_payload {
    uint32_t result; // lime_Result
    uint32_t link_state; // gps_link_state_payload
    uint32_t baudrate; // detected UART baud rate, 0 if not detected

    gps_data_aggregate_t data; // data.valid holds the GPS_DATA_VALID_* flags
    gps_stats_t stats;
};

#define GPS_SEND_MAX_TEXT 120

// LIME_M4_GPS_SEND request: sentence body without '$' and checksum, e.g. "PMTK161,0", NUL terminated.
// The response is a uint32_t lime_Result.
struct gps_send_payload {
    char body[GPS_SEND_MAX_TEXT];
};

#endif
