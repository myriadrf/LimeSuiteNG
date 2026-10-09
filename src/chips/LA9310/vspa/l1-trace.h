// SPDX-License-Identifier: BSD-3-Clause
// Copyright 2026 Lime Microsystems

#ifndef LIME_L1_TRACE_H
#define LIME_L1_TRACE_H

#include <stdint.h>

typedef struct l1_trace_data_s {
    uint64_t cnt;
    uint32_t msg;
    uint32_t param;
} l1_trace_data_t;

enum {
    T_BUSY = 0,
    T_GO,
    T_XFER_BUFFER,
    T_EXTERNAL_GO,
    T_TRACE_PUSH,
    T_DDR_WR_ENQ,
    T_DDR_RD_ENQ,
    T_DDR_WR_COMPLETE,
    T_DDR_RD_COMPLETE,
    T_ADC_COMPLETE,
    T_DAC_COMPLETE,
    T_RX_WORK,
    T_TX_WORK,
    T_ADC_ENQ,
    T_DAC_ENQ,
    T_DAC_AXIQ_RST,
    T_ADC_AXIQ_RST,
};

typedef struct l1_trace_state_s {
    uint32_t la9310_mem_address;
    uint32_t buffer_size;
    uint32_t bytes_produced;
    uint32_t event_count;
    uint32_t event_drops;
} l1_trace_hif_t; // L1 trace host interface

extern l1_trace_hif_t trace_hif;

extern void l1_trace_init(void);
extern void l1_trace_clear(void);
void l1_trace_upload(void);

#endif // LIME_L1_TRACE_H
