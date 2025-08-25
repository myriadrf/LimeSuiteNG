/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2024 NXP
 */

#ifndef __STATS_H__
#define __STATS_H__

#define RX_NUM_MAX_CHAN 4

typedef enum {
    STAT_DMA_AXIQ_READ,
    STAT_DMA_DDR_WR,
    STAT_EXT_DMA_DDR_WR,
    ERROR_DMA_DDR_WR_OVERRUN,
    ERROR_AXIQ_FIFO_RX_UNDERRUN,
    ERROR_AXIQ_FIFO_RX_OVERRUN,
    ERROR_AXIQ_DMA_RX_CMD_OVERRUN,
    ERROR_EXT_DMA_DDR_WR_OVERRUN,
    STATS_RX_MAX
} stats_rx_e;

typedef enum {
    STAT_DMA_AXIQ_WRITE = 0,
    STAT_DMA_DDR_RD,
    STAT_EXT_DMA_DDR_RD,
    ERROR_DMA_DDR_RD_UNDERRUN,
    ERROR_AXIQ_FIFO_TX_UNDERRUN,
    ERROR_AXIQ_FIFO_TX_OVERRUN,
    ERROR_AXIQ_DMA_TX_CMD_UNDERRUN,
    ERROR_EXT_DMA_DDR_RD_UNDERRUN,
    STATS_TX_MAX
} stats_tx_e;

typedef enum { ERROR_DMA_CONFIG_ERROR, ERROR_DMA_XFER_ERROR, STATS_GBL_MAX } stats_gbl_e;

typedef struct s_stats {
    uint32_t gbl_stats[STATS_GBL_MAX];
    uint32_t unused[8 - STATS_GBL_MAX];
    uint32_t tx_stats[STATS_TX_MAX];
    uint32_t rx_stats[RX_NUM_MAX_CHAN][STATS_RX_MAX];
} t_stats;

#if !defined(__VSPA__) && !defined(__M7__)

static const char* VSPA_stat_rx_string[STATS_RX_MAX + 1] = { "DMA_AXIQ_RD",
    "DMA_DDR_WR",
    "EXT_DDR_WR",
    "DDR_WR_OVR",
    "FIFO_RX_UDR",
    "FIFO_RX_OVR",
    "DMA_RX_CMD_OVR",
    "EXT_DDR_WR_OVR",
    "STATS_RX_MAX" };

static const char* VSPA_stat_tx_string[STATS_TX_MAX + 1] = { "DMA_AXIQ_WR",
    "DMA_DDR_RD",
    "EXT_DDR_RD",
    "DDR_RD_UDR",
    "FIFO_TX_UDR",
    "FIFO_TX_OVR",
    "DMA_TX_CMD_UDR",
    "EXT_DDR_RD_UDR",
    "STATS_TX_MAX" };

static const char* VSPA_stat_gbl_string[STATS_GBL_MAX + 1] = { "DMA_CFG_ERROR", "DMA_XFER_ERROR", "STATS_GBL_MAX" };

#endif

#ifdef __VSPA__
extern volatile t_stats g_stats __attribute__((aligned(64)));
#endif

#endif // __STATS_H__
