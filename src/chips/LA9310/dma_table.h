// SPDX-License-Identifier: BSD-3-Clause
// Copyright 2026 Lime Microsystems

#ifndef LIME_DMA_TABLE_H
#define LIME_DMA_TABLE_H

#include "stdint.h"
#include "stdbool.h"

#define DMA_TABLE_LINE_COUNT 32 // expected to be power of 2

typedef struct DMA_LINE {
    uint64_t timestamp;
    uint32_t addr;
    uint16_t size;
    uint16_t flags;
} dma_line_t;

typedef struct DMA_TABLE {
    dma_line_t items[DMA_TABLE_LINE_COUNT];
    uint32_t head;
    uint32_t tail;
} dma_table_t;

void dma_table_reset(dma_table_t* fifo);
bool dma_table_push(dma_table_t* fifo, dma_line_t* line);
dma_line_t* dma_table_front(dma_table_t* fifo);
void dma_table_pop(dma_table_t* fifo);
bool dma_table_isfull(const dma_table_t* fifo);
bool dma_table_isempty(const dma_table_t* fifo);
uint32_t dma_table_size(const dma_table_t* fifo);

#endif // LIME_DMA_TABLE_H