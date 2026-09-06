// SPDX-License-Identifier: BSD-3-Clause
// Copyright 2026 Lime Microsystems

#ifndef LIME_DMA_TCD_FIFO_H
#define LIME_DMA_TCD_FIFO_H

#include <stdint.h>
#include <stdbool.h>

#define MFIFO_SIZE 32 // must be power of 2
#define MFIFO_SIZE_MASK (MFIFO_SIZE - 1)

typedef volatile struct DMA_TCD {
    uint32_t timestamp_lsb; // user facing time, samples count
    uint32_t timestamp_msb; // user facing time, samples count
    uint32_t la9310_mem_address;
    uint32_t size;
    uint32_t flags;
} dma_tcd_t;

typedef volatile struct TCD_FIFO {
    dma_tcd_t items[MFIFO_SIZE];
    uint32_t head;
    uint32_t tail;
    uint32_t done;
} dma_tcd_fifo_t;

static inline void tcd_fifo_reset(dma_tcd_fifo_t* fifo)
{
    fifo->head = 0;
    fifo->tail = 0;
    fifo->done = 0;
}

static inline void tcd_fifo_push(dma_tcd_fifo_t* fifo)
{
    ++fifo->tail;
}

static inline void tcd_fifo_pop(dma_tcd_fifo_t* fifo)
{
    ++fifo->head;
}

static inline dma_tcd_t* tcd_fifo_front(dma_tcd_fifo_t* fifo)
{
    return &fifo->items[fifo->head & MFIFO_SIZE_MASK];
}
static inline dma_tcd_t* tcd_fifo_back(dma_tcd_fifo_t* fifo)
{
    return &fifo->items[fifo->tail & MFIFO_SIZE_MASK];
}

static inline uint16_t tcd_fifo_size(const dma_tcd_fifo_t* fifo)
{
    return fifo->tail - fifo->head;
}

static inline bool tcd_fifo_isfull(const dma_tcd_fifo_t* fifo)
{
    return tcd_fifo_size(fifo) == MFIFO_SIZE;
}

static inline bool tcd_fifo_isempty(const dma_tcd_fifo_t* fifo)
{
    return tcd_fifo_size(fifo) == 0;
}

#endif // LIME_DMA_TCD_FIFO_H
