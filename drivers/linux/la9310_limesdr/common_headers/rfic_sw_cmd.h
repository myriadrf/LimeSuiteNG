/* SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0)
 * Copyright 2021-2024 NXP
 */
#ifndef __RFIC_SW_CMD_H
#define __RFIC_SW_CMD_H

#pragma pack(push)
#define RF_SWCMD_DATA_SIZE 12

/* Supported RF Band */
typedef enum rf_band { RF_SW_BAND_N77, RF_SW_BAND_B13, RF_SW_BAND_B3, RF_SW_BAND_GNSS, RF_SW_BAND_MAX } rf_band_t;

/* Supported Bandwidth */
typedef enum rf_bw {
    RF_BW_36MHZ,
    RF_BW_72MHZ,
    RF_BW_144MHZ,
    RF_BW_288MHZ,
    RF_BW_432MHZ,
    RF_BW_576MHZ,
    RF_BW_720MHZ,
    RF_BW_BYPASS
} rf_bw_t;

/* RFIC Software commands */
typedef enum RfSwCmdId {
    RF_SW_CMD_SET_BAND,
    RF_SW_CMD_ADJUST_PLL_FREQ,
    RF_SW_CMD_CTRL_GAIN,
    RF_SW_CMD_GET_ABS_GAIN,
    RF_SW_CMD_REG_READ,
    RF_SW_CMD_REG_WRITE,
    RF_SW_CMD_CTRL_VGA_GAIN,
    RF_SW_CMD_CTRL_DEMOD_GAIN,
    RF_SW_CMD_CTRL_LNA,
    RF_SW_CMD_DUMP_IQ_DATA,
    RF_SW_CMD_SINGLE_TONE_TX,
    RF_SW_CMD_SET_LOOPBACK,
    RF_SW_CMD_FAST_CALIB,
    RF_SW_CMD_TX_IQ_DATA,
    RF_SW_GET_RX_DC_OFFSET,
    RF_SW_SET_RX_DC_OFFSET,
    RF_SW_SET_CHANNEL,
    RF_SW_SET_IQ_IMBALANCE,
    RF_SW_CMD_END
} RfSwCmdId_t;

typedef enum rf_sw_cmd_status {
    RF_SW_CMD_STATUS_FREE,
    RF_SW_CMD_STATUS_POSTED,
    RF_SW_CMD_STATUS_PENDING,
    RF_SW_CMD_STATUS_IN_PROGRESS,
    RF_SW_CMD_STATUS_TIMEOUT,
    RF_SW_CMD_STATUS_DONE
} rf_sw_cmd_status_t;

typedef enum rf_sw_cmd_result {
    RF_SW_CMD_RESULT_OK = 0,
    RF_SW_CMD_RESULT_CMD_INVALID,
    RF_SW_CMD_RESULT_CMD_PARAMS_INVALID,
    RF_SW_CMD_RESULT_ERROR,
    RF_SW_CMD_RESULT_NOT_IMPLEMENTED
} rf_sw_cmd_result_t;

typedef enum rf_sw_cmd_type { RF_SW_CMD_LOCAL = 1, RF_SW_CMD_REMOTE } rf_sw_cmd_type_t;

/* RFIC Loopback types */
typedef enum loopback_type { NO_LOOPBACK = 0, AXIQ_LOOPBACK, LOOPBACK_END } rf_loopback_type_t;

/* CMD: RF_SWCMD_SET_BAND */
struct sw_cmddata_set_band {
    uint32_t band;
};

/* CMD: RF_SWCMD_SET_FREQ */
struct sw_cmddata_adjust_pll_freq {
    uint32_t freq_khz;
};

/* CMD: RF_SWCMD_CTRL_GAIN */
/* CMD: RF_SWCMD_GET-ABS_GAIN */
struct sw_cmddata_gain {
    uint32_t gain_in_db;
};

/* CMD: RF_SWCMD_REG_READ */
/* CMD: RF_SWCMD_REG_WRITE */
struct sw_cmddata_reg_opr {
    uint32_t dspi_slv_id;
    uint32_t addr;
    uint32_t val;
};

/* CMD: RF_SWCMD_CTRL_VGA_GAIN */
struct sw_cmddata_vga_gain {
    uint32_t dac1_val;
    uint32_t dac2_val;
};

/* CMD: RF_SWCMD_CTRL_DEMOD_GAIN */
struct sw_cmddata_demod_gain {
    uint32_t rf_attn;
    uint32_t bb_gain;
};

/* CMD: RF_SWCMD_CTRL_LNA */
struct sw_cmddata_ctrl_lna {
    uint32_t state;
};

/* CMD: RF_SWCMD_DUMP_IQ_DATA */
struct sw_cmddata_dump_iq {
    uint32_t addr;
    uint32_t size;
    uint32_t start_stop;
};

/* CMD: RF_SW_CMD_SINGLE_TONE_TX */
struct sw_cmddata_single_tone_TX {
    uint32_t tone_index;
    uint32_t start_stop;
};

/* CMD: RF_SW_CMD_SET_LOOPBACK */
struct sw_cmddata_set_loopback {
    uint32_t loopback_type;
};

/* Software commands parameters */
typedef struct rf_sw_cmd_desc {
    uint32_t cmd; /* command id */
    uint32_t flags; /* remote or local */
    uint32_t timeout; /* command specific timeout in us */
    uint32_t core_id; /* Descriptor owner */
    uint32_t ipi_event_id;
    uint32_t status; /* desc status */
    uint32_t result;
    uint8_t data[RF_SWCMD_DATA_SIZE]; /* command specific data */
} rf_sw_cmd_desc_t;

#pragma pack(pop)
#endif //__RFIC_SW_CMD_H
