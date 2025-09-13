/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2024 NXP
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <inttypes.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/timerfd.h>
#include <sys/mman.h>
#include <errno.h>
#include <stdbool.h>
#include <pthread.h>
#include <sched.h>
#include <assert.h>
#include <semaphore.h>
#include <signal.h>
#include <argp.h>

#include "vspa_dmem_proxy.h"
// #include "imx8-host.h"
// #include "l1-trace-host.h"

extern "C" {

#define CACHE_LINE_SIZE 64

enum l1_trace_msg_host {
    /* 0x100 */ L1_TRACE_MSG_DMA_XFR = 0x100,
    /* 0x101 */ L1_TRACE_MSG_DMA_CFGERR,
    /* 0x102 */ L1_TRACE_MSG_DMA_DDR_RD_START,
    /* 0x103 */ L1_TRACE_MSG_DMA_DDR_RD_COMP,
    /* 0x104 */ L1_TRACE_MSG_DMA_DDR_RD_UNDERRUN,
    /* 0x105 */ L1_TRACE_MSG_DMA_DDR_WR_START,
    /* 0x106 */ L1_TRACE_MSG_DMA_DDR_WR_COMP,
    /* 0x107 */ L1_TRACE_MSG_DMA_DDR_WR_OVERRUN,
};

// Clean and Invalidate data cache
// #define dccivac(p) { asm volatile("dc civac, %0" : : "r"(p) : "memory"); }
inline void dccivac(void* mem)
{
}

// Data Cache Flush/Clean
// #define dcbf(p) { asm volatile("dc cvac, %0" : : "r"(p) : "memory"); }
inline void dcbf(void* mem)
{
}

static void invalidate_region(volatile void* region, uint32_t size)
{
    // uint32_t* ptr;
    // uint32_t index;

    // for (ptr = (uint32_t*)region, index = 0; index < size / CACHE_LINE_SIZE; ptr += 16, index++)
    // {
    //     dccivac((uint32_t*)(ptr));
    // }
}

inline void flush_region(void* region, uint32_t size)
{
    // uint32_t* ptr;
    // uint32_t index;

    // for (ptr = (uint32_t*)region, index = 0; index < size / CACHE_LINE_SIZE; ptr += 16, index++)
    // {
    //     dcbf((uint32_t*)(ptr));
    // }
}

inline void l1_trace_clear(void)
{
}

inline void l1_trace(uint32_t msg, uint32_t param)
{
}

uint32_t* vl_iqflood_ddr_addr = NULL;
volatile uint32_t* v_vspa_dmem_proxy_ro = NULL;
const volatile t_tx_ch_host_proxy* tx_vspa_proxy_ro = NULL;
const volatile t_rx_ch_host_proxy* rx_vspa_proxy_ro = NULL;

volatile t_tx_ch_host_proxy* tx_vspa_proxy_wo = NULL;
volatile uint32_t* v_tx_vspa_proxy_wo = nullptr;
volatile uint32_t* v_rx_vspa_proxy_wo = nullptr;
t_stats* app_stats = NULL;

#define TX_DDR_STEP (((t_vspa_dmem_proxy*)v_vspa_dmem_proxy_ro)->tx_state_readonly.tx_ddr_step)
#define RX_DDR_STEP (((t_vspa_dmem_proxy*)v_vspa_dmem_proxy_ro)->tx_state_readonly.rx_ddr_step)
#define RX_DECIM (((t_vspa_dmem_proxy*)v_vspa_dmem_proxy_ro)->tx_state_readonly.rx_decim)
#define TX_UPSMP (((t_vspa_dmem_proxy*)v_vspa_dmem_proxy_ro)->tx_state_readonly.tx_upsmp)
#define RX_NUM_CHAN (((t_vspa_dmem_proxy*)v_vspa_dmem_proxy_ro)->tx_state_readonly.rx_num_chan)

int iq_player_init(uint32_t* v_iqflood, uint32_t iqflood_size, uint32_t* v_la9310_pci_bar2, uint32_t* dmem_proxy_vaddr)
{
    vl_iqflood_ddr_addr = v_iqflood;

    // use last 1024 bytes of iqflood as shared vspa dmem proxy , vspa will write mirrored dmem value to avoid PCI read from host
    v_vspa_dmem_proxy_ro = dmem_proxy_vaddr;
    rx_vspa_proxy_ro = &(((t_vspa_dmem_proxy*)v_vspa_dmem_proxy_ro)->rx_state_readonly[0]);
    tx_vspa_proxy_ro = &(((t_vspa_dmem_proxy*)v_vspa_dmem_proxy_ro)->tx_state_readonly);

    app_stats = &(((t_vspa_dmem_proxy*)v_vspa_dmem_proxy_ro)->app_stats);
    uint32_t* BAR2_addr = v_la9310_pci_bar2;

    /* use dmem structure at hardcoded address to write host status/request */
    dccivac((uint32_t*)tx_vspa_proxy_ro);
    if (tx_vspa_proxy_ro->rx_num_chan == 1)
    {
        v_tx_vspa_proxy_wo = (uint32_t*)((uint64_t)BAR2_addr + 0x400000 + 0x00000000);
        v_rx_vspa_proxy_wo = (uint32_t*)((uint64_t)BAR2_addr + 0x400000 + 0x00000040);
    }
    else
    {
        v_tx_vspa_proxy_wo = (uint32_t*)((uint64_t)BAR2_addr + 0x500000 + 0x00004000);
        v_rx_vspa_proxy_wo = (uint32_t*)((uint64_t)BAR2_addr + 0x500000 + 0x00004040);
    }
    tx_vspa_proxy_wo = reinterpret_cast<volatile t_tx_ch_host_proxy*>(v_tx_vspa_proxy_wo);

    return 1;
}

uint32_t app_TX_total_produced_size = 0; /* Bytes copied into modem tx Fifo */
uint32_t app_TX_total_consumed_size = 0; /* Bytes sent out of modem tx Fifo */
uint32_t tx_modem_ddr_fifo_start;
uint32_t tx_modem_ddr_fifo_size;
uint32_t tx_modem_fifo_offset;

int iq_player_init_tx(uint32_t fifo_start, uint32_t fifo_size)
{
    (void)VSPA_stat_gbl_string; /* unused variables  */
    (void)VSPA_stat_tx_string;
    (void)VSPA_stat_rx_string;

    if (vl_iqflood_ddr_addr == NULL)
        return 0;

    dccivac((uint32_t*)(tx_vspa_proxy_ro));

    /* check firmware is idle waiting for new data */
    //if (tx_vspa_proxy_ro->host_produced_size != tx_vspa_proxy_ro->la9310_fifo_enqueued_size) {
    //  printf("\n TX : modem is already running \n");
    //  fflush(stdout);
    //  return 0;
    //}

    // init fifo pointers
    tx_modem_ddr_fifo_start = fifo_start;
    tx_modem_ddr_fifo_size = fifo_size;
    tx_modem_fifo_offset = tx_vspa_proxy_ro->la9310_fifo_enqueued_size % fifo_size;
    tx_vspa_proxy_wo->host_produced_size = tx_vspa_proxy_ro->la9310_fifo_enqueued_size;

    // init flow control
    app_TX_total_consumed_size = tx_vspa_proxy_ro->la9310_fifo_enqueued_size;
    app_TX_total_produced_size = tx_vspa_proxy_ro->la9310_fifo_enqueued_size;

    return 1;
}

int iq_player_send_data(uint32_t* v_buffer, uint32_t size)
{
    uint32_t fifoWaterMark = 0;
    uint32_t busy_size = 0;
    uint32_t empty_size = 0;
    void* ddr_dst;

    dccivac((uint32_t*)(tx_vspa_proxy_ro));

    // check stop/restart
    if (tx_vspa_proxy_ro->DDR_rd_base_address == 0xdeadbeef)
    {
        tx_modem_fifo_offset = 0;
        app_TX_total_consumed_size = 0;
        app_TX_total_produced_size = 0;
        tx_vspa_proxy_wo->host_produced_size = 0;
        return 0;
    }

    // Check new transfer opty
    app_TX_total_consumed_size = tx_vspa_proxy_ro->la9310_fifo_enqueued_size;
    busy_size = app_TX_total_produced_size - app_TX_total_consumed_size;
    if (busy_size > tx_modem_ddr_fifo_size)
    {
        printf("\n TX underrun , exit (busy=0x%08x app_TX_total_produced_size=0x%08x app_TX_total_consumed_size=0x%08x)\n",
            busy_size,
            app_TX_total_produced_size,
            app_TX_total_consumed_size);
        fflush(stdout);
        // exit(0);
    }
    fifoWaterMark = tx_modem_ddr_fifo_size - tx_modem_fifo_offset;
    empty_size = tx_modem_ddr_fifo_size - busy_size;
    if (empty_size >= TX_DDR_STEP)
    {
        if (empty_size > fifoWaterMark)
        {
            empty_size = fifoWaterMark;
        }
        if (empty_size > size)
        {
            empty_size = size;
        }
        // ready to send new data
        app_TX_total_produced_size += empty_size;
        app_stats->tx_stats[STAT_EXT_DMA_DDR_RD] += empty_size / TX_DDR_STEP;
    }
    else
    {
        // no room in tx fifo
        return 0;
    }

    // xfer data
    ddr_dst = (void*)((uint64_t)vl_iqflood_ddr_addr + tx_modem_ddr_fifo_start + tx_modem_fifo_offset);
    l1_trace(L1_TRACE_MSG_DMA_DDR_RD_START, tx_modem_ddr_fifo_start + tx_modem_fifo_offset);
    memcpy(ddr_dst, v_buffer, empty_size);
    flush_region(ddr_dst, empty_size);
    l1_trace(L1_TRACE_MSG_DMA_DDR_RD_COMP, empty_size);

    // update modem flow control
    tx_vspa_proxy_wo->host_produced_size = app_TX_total_produced_size;

    tx_modem_fifo_offset += empty_size;
    if (tx_modem_fifo_offset >= tx_modem_ddr_fifo_size)
    {
        tx_modem_fifo_offset = 0;
    }

    return empty_size;
}

uint32_t rx_modem_ddr_fifo_start[RX_NUM_MAX_CHAN] = { 0, 0, 0, 0 };
uint32_t rx_modem_ddr_fifo_size[RX_NUM_MAX_CHAN] = { 0, 0, 0, 0 };
uint32_t rx_modem_fifo_offset[RX_NUM_MAX_CHAN] = { 0, 0, 0, 0 };
uint32_t app_RX_total_consumed_size[RX_NUM_MAX_CHAN] = { 0, 0, 0, 0 }; /* Bytes copied from modem rx Fifo */
uint32_t app_RX_total_produced_size[RX_NUM_MAX_CHAN] = { 0, 0, 0, 0 }; /* Bytes received in modem rx Fifo */

int iq_player_init_rx(uint32_t chan, uint32_t fifo_start, uint32_t fifo_size)
{
    if (vl_iqflood_ddr_addr == NULL)
        return -1;

    dccivac((uint32_t*)(rx_vspa_proxy_ro));

    // init fifo pointers
    rx_modem_ddr_fifo_start[chan] = fifo_start;
    rx_modem_ddr_fifo_size[chan] = fifo_size;
    rx_modem_fifo_offset[chan] = rx_vspa_proxy_ro[chan].la9310_fifo_consumed_size % fifo_size;

    // init flow counters
    app_RX_total_consumed_size[chan] = rx_vspa_proxy_ro[chan].la9310_fifo_consumed_size;
    app_RX_total_produced_size[chan] = rx_vspa_proxy_ro[chan].la9310_fifo_consumed_size;
    tx_vspa_proxy_wo->host_consumed_size[chan] = rx_vspa_proxy_ro[chan].la9310_fifo_consumed_size;

    return 1;
}

int iq_player_receive_data(uint32_t chan, uint32_t* v_buffer, uint32_t max_size)
{
    uint32_t fifoWaterMark = 0;
    uint32_t data_size = 0;
    volatile void* ddr_src;

    dccivac((uint32_t*)(rx_vspa_proxy_ro));

    // check stop/restart
    if (rx_vspa_proxy_ro[chan].DDR_wr_base_address == 0xdeadbeef)
    {
        app_RX_total_produced_size[chan] = 0;
        app_RX_total_consumed_size[chan] = 0;
        rx_modem_fifo_offset[chan] = 0;
        tx_vspa_proxy_wo->host_consumed_size[chan] = 0;
        return 0;
    }

    // Check new transfer
    // printf("Rx prod: %li  cons: %li\n", app_RX_total_produced_size[chan], rx_vspa_proxy_ro[chan].la9310_fifo_consumed_size);
    app_RX_total_produced_size[chan] = rx_vspa_proxy_ro[chan].la9310_fifo_consumed_size;
    data_size = app_RX_total_produced_size[chan] - app_RX_total_consumed_size[chan];
    if (data_size >= rx_modem_ddr_fifo_size[chan])
    {
        printf("\n RX underrun , exit (data_size=0x%08x app_RX_total_produced_size=0x%08x app_RX_total_consumed_size=0x%08x)\n",
            data_size,
            app_RX_total_produced_size[chan],
            app_RX_total_consumed_size[chan]);
        fflush(stdout);
        // exit(0);
    }

    fifoWaterMark = rx_modem_ddr_fifo_size[chan] - rx_modem_fifo_offset[chan];
    if (data_size >= RX_DDR_STEP)
    {
        if (data_size > fifoWaterMark)
            data_size = fifoWaterMark;
        if (data_size > max_size)
            data_size = max_size;
        // ready to fetch new data
        app_RX_total_consumed_size[chan] += data_size;
        app_stats->rx_stats[chan][STAT_EXT_DMA_DDR_WR] += data_size / RX_DDR_STEP;
    }
    else
    {
        // no data available
        return 0;
    }

    // xfer data
    ddr_src = (volatile void*)((uint64_t)vl_iqflood_ddr_addr + rx_modem_ddr_fifo_start[chan] + rx_modem_fifo_offset[chan]);
    l1_trace(L1_TRACE_MSG_DMA_DDR_RD_START, rx_modem_ddr_fifo_start[chan] + rx_modem_fifo_offset[chan]);
    invalidate_region(ddr_src, data_size);
    memcpy(v_buffer, (const void*)ddr_src, data_size);
    l1_trace(L1_TRACE_MSG_DMA_DDR_RD_COMP, data_size);

    // update modem flow control
    tx_vspa_proxy_wo->host_consumed_size[chan] = app_RX_total_consumed_size[chan];

    rx_modem_fifo_offset[chan] += data_size;
    if (rx_modem_fifo_offset[chan] >= rx_modem_ddr_fifo_size[chan])
    {
        rx_modem_fifo_offset[chan] = 0;
    }

    return data_size;
}
}
