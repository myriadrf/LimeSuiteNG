/* SPDX-License-Identifier: BSD-2-Clause
 *
 * LimePCIe driver
 *
 * This file is part of LimePCIe.
 *
 * Copyright (C) 2018-2020 / EnjoyDigital  / florent@enjoy-digital.fr
 *
 */

#ifndef _LINUX_LIMEPCIE_H
#define _LINUX_LIMEPCIE_H

struct limepcie_ioctl_dma_writer {
    uint8_t enable;
    int64_t hw_count;
    int64_t sw_count;
    uint32_t write_size;
    uint8_t irqFreq;
};

struct limepcie_ioctl_dma_reader {
    uint8_t enable;
    int64_t hw_count;
    int64_t sw_count;
    uint64_t interruptCounter;
    uint8_t irqFreq;
};

struct limepcie_ioctl_lock {
    uint8_t dma_reader_request;
    uint8_t dma_writer_request;
    uint8_t dma_reader_release;
    uint8_t dma_writer_release;
    uint8_t dma_reader_status;
    uint8_t dma_writer_status;
};

struct limepcie_ioctl_mmap_dma_info {
    uint64_t dma_tx_buf_offset;
    uint64_t dma_tx_buf_size;
    uint64_t dma_tx_buf_count;

    uint64_t dma_rx_buf_offset;
    uint64_t dma_rx_buf_size;
    uint64_t dma_rx_buf_count;
};

struct limepcie_ioctl_mmap_dma_update {
    int64_t sw_count;
    int32_t buffer_size;
    bool genIRQ;
};

struct limepcie_cache_flush {
    uint16_t bufferIndex;
    bool isTx;
    bool toDevice;
};

struct limepcie_version {
    int major;
    int minor;
    int patch;
};

struct limepcie_control_packet {
    uint8_t request[64];
    uint8_t response[64];
    uint32_t length;
    int timeout_ms;
};

#define LIMEPCIE_IOCTL 'S'

#define LIMEPCIE_IOCTL_DMA_WRITER _IOWR(LIMEPCIE_IOCTL, 21, struct limepcie_ioctl_dma_writer)
#define LIMEPCIE_IOCTL_DMA_READER _IOWR(LIMEPCIE_IOCTL, 22, struct limepcie_ioctl_dma_reader)
#define LIMEPCIE_IOCTL_MMAP_DMA_INFO _IOR(LIMEPCIE_IOCTL, 24, struct limepcie_ioctl_mmap_dma_info)
#define LIMEPCIE_IOCTL_LOCK _IOWR(LIMEPCIE_IOCTL, 25, struct limepcie_ioctl_lock)
#define LIMEPCIE_IOCTL_MMAP_DMA_WRITER_UPDATE _IOW(LIMEPCIE_IOCTL, 26, struct limepcie_ioctl_mmap_dma_update)
#define LIMEPCIE_IOCTL_MMAP_DMA_READER_UPDATE _IOW(LIMEPCIE_IOCTL, 27, struct limepcie_ioctl_mmap_dma_update)
#define LIMEPCIE_IOCTL_CACHE_FLUSH _IOW(LIMEPCIE_IOCTL, 28, struct limepcie_cache_flush)
#define LIMEPCIE_IOCTL_VERSION _IOWR(LIMEPCIE_IOCTL, 29, struct limepcie_version)
#define LIMEPCIE_IOCTL_RUN_CONTROL_COMMAND _IOWR(LIMEPCIE_IOCTL, 30, struct limepcie_control_packet)

#endif /* _LINUX_LIMEPCIE_H */
