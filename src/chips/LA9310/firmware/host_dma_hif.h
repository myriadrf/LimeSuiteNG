#ifndef LIME_M4_HOST_DMA_HIF_H
#define LIME_M4_HOST_DMA_HIF_H

#include "dma_tcd_fifo.h"

#include <stdint.h>
#include <stdbool.h>

// Packet flags
enum {
    PKT_HAS_TIMESTAMP = (1 << 0),
    PKT_START = (1 << 1),
    PKT_END = (1 << 2),
    PKT_IRQ = (1 << 3),
    PKT_DMA_TCD_END = (1 << 4),
};

typedef struct DMA_Frontend_hif {
    dma_tcd_fifo_t tcd_fifo;

    uint32_t bytes_xferred;
    uint32_t error;
    uint32_t enable;
    uint32_t loop_mode;
    uint32_t clear;
    uint32_t pending;
} host_dma_hif_t;

#endif // LIME_M4_HOST_DMA_HIF_H