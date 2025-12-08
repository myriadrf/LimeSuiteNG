/* SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0)
 * Copyright 2021-2022 NXP
 */
#ifndef __RFIC_HIF_H
#define __RFIC_HIF_H
#include "rfic_sw_cmd.h"

#pragma pack(push)

enum rf_state { RF_NOT_READY, RF_READY };

/* RFIC Host SWCMD  interface */
typedef struct rf_common_mdata {
    rf_sw_cmd_desc_t host_swcmd;
} rf_common_mdata_t;

/* RFIC current info */
typedef struct rf_priv_mdata {
    uint32_t band;
    uint32_t freq_khz;
    uint32_t bw;
    uint32_t lna_state; /* bit0 - N77, bit1 - B13, bit2 - B3 */
    uint32_t gain_in_db;
    uint32_t demod_rf_attn;
    uint32_t demod_bb_gain;
    uint32_t vga_dac1_val;
    uint32_t vga_dac2_val;
} rf_priv_mdata_t;

/* RF current statistics */
typedef struct rf_stats {
    uint32_t local_cmd_count;
    uint32_t remote_cmd_count;
    uint32_t local_cmd_failed_count;
    uint32_t remote_cmd_failed_count;
} rf_stats_t;

/* RFIC HIF */
typedef struct rf_host_if {
    rf_common_mdata_t rf_mdata;
    rf_priv_mdata_t rf_priv_mdata;
    rf_stats_t rf_stats;
    uint32_t ready;
} rf_host_if_t;

#pragma pack(pop)
#endif //__RFIC_HIF_H
