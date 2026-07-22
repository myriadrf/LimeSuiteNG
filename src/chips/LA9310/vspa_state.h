// SPDX-License-Identifier: BSD-3-Clause
// Copyright 2026 Lime Microsystems

#ifndef VSPA_STATE_H
#define VSPA_STATE_H

#include "pipelines.h"
#include "VSPA_DMA.h"

enum {
    PROXY_UPDATE_FLOW = (1 << 0),
    PROXY_UPDATE_INFO = (1 << 1),
    PROXY_UPDATE_INTERRUPT = (1 << 2),
    PROXY_UPDATE_INTERNALS = (1 << 3),
    PROXY_UPDATE_TRACE = (1 << 4),
    PROXY_UPDATE_ALL = 0xFF,
};

struct flow_control {
    uint32_t produced;
    uint32_t consumed;
};

struct flow_issues {
    uint32_t underrun;
    uint32_t overrun;
    uint32_t xfer_errors;
    uint32_t xfer_config_errors;
};

struct vspa_flow_control {
    struct flow_control tx;
    struct flow_control rx[4];
    struct flow_issues tx_issues;
    struct flow_issues rx_issues[4];
};

struct vspa_interface_info {
    tx_config_t tx_config;
    rx_config_t rx_config[4];
    uint32_t proxy_fetch;
    uint32_t dmemProxyOffset;
    uint32_t l1_trace_offset;
    uint16_t l1_trace_size;
    uint16_t rx_num_chan;
};

struct vspa_internals {
    tx_pipeline_t txpipe;
    rx_pipeline_t rxpipe[4];
    tx_control_t tx_control;
    rx_control_t rx_control[4];
    dma_table_t tx_dma_schedule;
    uint32_t go_count;
};

// proxy divided into sections for convenient partial updates using DMA
// DMA transfers must be 128 bit aligned
typedef struct VSPAState {
    struct vspa_flow_control data_flow;
    struct vspa_interface_info info;
    struct vspa_internals internals;
} vspa_state_t;

extern vspa_state_t player_state __attribute__((aligned(32)));
extern void dmem_proxy_set_offset(uint32_t addr_offset);
extern void EnqueueProxyUpdate(uint32_t flags);
extern void VSPA_PROXY_update(void);

#endif // VSPA_STATE_H
