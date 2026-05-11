#ifndef LIME_M4_COMMANDS_H
#define LIME_M4_COMMANDS_H

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

    LIME_M4_EMPTY,
    LIME_M4_LMS64C_PACKET,
    LIME_M4_SET_SYSTEM_CLOCK_FREQUENCY,
    LIME_M4_GET_REFERENCE_CLOCK_FREQUENCY,
    LIME_M4_SET_REFERENCE_CLOCK_FREQUENCY,
    LIME_M4_BOOTLOADER,
    LIME_M4_HEARTHBEAT,

    LIME_M4_HARDWARE_COUNTER_GET,
    LIME_M4_HARDWARE_COUNTER_RESET,
    LIME_M4_DIGITAL_LOOPBACK,

    LIME_M4_TX_CONTROL,
};

struct tx_dac_allowed_payload {
    uint64_t vspa_cmd;
};

struct tx_band_switch_payload {
    uint32_t tx_rf_switch_control;
};

struct scheduled_cmd {
    uint64_t timepoint;
    uint32_t cmd;
    uint32_t data[32];
};

struct tx_window_payload {
    uint64_t vspa_cmd;
    uint32_t tx_rf_switch_control;
    int32_t rf_switch_offset;
    int32_t pa_switch_offset;
};

enum tx_control_flags {
    TX_CONTROL_START = (1 << 0),
    TX_CONTROL_STOP = (1 << 1),
    TX_CONTROL_HAS_TIME = (1 << 2),
    TX_CONTROL_HAS_LENGTH = (1 << 3),
};

struct tx_control_payload {
    int64_t timepoint;
    uint32_t data_src_offset;
    uint32_t data_length;
    uint8_t flags;
};

#endif