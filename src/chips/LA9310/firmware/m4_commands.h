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

    LIME_M4_TX_CONTROL, //20
    LIME_M4_RX_CONTROL,
    LIME_M4_GET_FEATURES,
    LIME_M4_DMA,
    LIME_M4_IQSTREAM_CTRL
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

#endif