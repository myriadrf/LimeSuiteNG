// SPDX-License-Identifier: BSD-3-Clause
// Copyright 2026 Lime Microsystems

#include "dma_table.h"

void dma_table_reset(dma_table_t* fifo)
{
    fifo->head = 0;
    fifo->tail = 0;
    // memclr(fifo->items, sizeof(fifo->items));
}

bool dma_table_push(dma_table_t* fifo, dma_line_t* block)
{
    if (dma_table_isfull(fifo))
        return false;

    fifo->items[fifo->tail & (DMA_TABLE_LINE_COUNT - 1)] = *block;
    ++fifo->tail;
    return true;
}

dma_line_t* dma_table_front(dma_table_t* fifo)
{
    return &fifo->items[fifo->head & (DMA_TABLE_LINE_COUNT - 1)];
}

void dma_table_pop(dma_table_t* fifo)
{
    ++fifo->head;
}

uint32_t dma_table_size(const dma_table_t* fifo)
{
    return fifo->tail - fifo->head;
}

bool dma_table_isfull(const dma_table_t* fifo)
{
    return dma_table_size(fifo) == (DMA_TABLE_LINE_COUNT - 1);
}
bool dma_table_isempty(const dma_table_t* fifo)
{
    return dma_table_size(fifo) == 0;
}
