#ifndef LIME_VSPA_DMA_HIF_H
#define LIME_VSPA_DMA_HIF_H

// Packet flags
enum {
    IQSTREAM_HAS_TIMESTAMP = (1 << 0),
    IQSTREAM_TRX_START = (1 << 1),
    IQSTREAM_TRX_END = (1 << 2),
    IQSTREAM_IRQ = (1 << 3),
    IQSTREAM_DMA_TCD_END = (1 << 4),

    // TCD was completed early because following samples were lost, size is less than requested.
    IQSTREAM_TRUNCATED = (1 << 5),
};

#include <stdint.h>

#include "dma_tcd_fifo.h"

// Directly accessable data from host for TCD submission and status readback
typedef struct VSPA_DMA_HIF {
    dma_tcd_fifo_t tcd_table;
    uint32_t htv_tcd_pending_flag_mask; // Host to VSPA signal that input TCD is prepared
    uint32_t vth_tcd_done_flag_mask; // VSPA to host, signal that TCD has been completed
} vspa_dma_hif_t;

#endif // LIME_VSPA_DMA_HIF_H
