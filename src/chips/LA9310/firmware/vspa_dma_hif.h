#ifndef LIME_VSPA_DMA_HIF_H
#define LIME_VSPA_DMA_HIF_H

// Packet flags
enum {
    PKT_HAS_TIMESTAMP = (1 << 0),
    PKT_START = (1 << 1),
    PKT_END = (1 << 2),
    PKT_IRQ = (1 << 3),
    PKT_DMA_TCD_END = (1 << 4),
};

#include <stdint.h>
#include <stdbool.h>

#include "dma_tcd_fifo.h"

// Directly accessable data from host for TCD submission and status readback
typedef struct VSPA_DMA_HIF {
    dma_tcd_fifo_t tcd_fifo;
    uint32_t htv_tcd_pending_flag_mask; // Host to VSPA signal that input TCD is prepared
    uint32_t vth_tcd_done_flag_mask; // VSPA to host, signal that TCD has been completed
} vspa_dma_hif_t;

void signal_to_vspa(uint32_t flags);
uint32_t vspa_signal_status();

#endif // LIME_VSPA_DMA_HIF_H
