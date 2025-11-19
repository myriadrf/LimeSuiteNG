#include "stats.h"

const char* VSPA_stat_rx_string[STATS_RX_MAX + 1] = { "DMA_AXIQ_RD",
    "DMA_DDR_WR",
    "APP_DDR_WR",
    "EXT_DMA_DDR_WR",
    "DMA_DDR_WR_OVR",
    "AXIQ_FIFO_RX_UDR",
    "AXIQ_FIFO_RX_OVR",
    "EXT_DMA_DDR_WR_OVR",
    "STATS_RX_MAX" };

const char* VSPA_stat_tx_string[STATS_TX_MAX + 1] = { "DMA_AXIQ_WR",
    "DMA_DDR_RD",
    "APP_DDR_WR",
    "EXT_DMA_DDR_RD",
    "DMA_DDR_RD_UDR",
    "AXI_FIFO_TX_UDR",
    "AXI_FIFO_TX_OVR",
    "EXT_DMA_DDR_RD_UDR",
    "STATS_TX_MAX" };

const char* VSPA_stat_gbl_string[STATS_GBL_MAX + 1] = { "DMA_CFG_ERROR", "DMA_XFER_ERROR", "STATS_GBL_MAX" };
