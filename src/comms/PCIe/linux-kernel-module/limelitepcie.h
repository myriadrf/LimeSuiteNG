/* SPDX-License-Identifier: BSD-2-Clause
 *
 * LitePCIe driver
 *
 * This file is part of LitePCIe.
 *
 * Copyright (C) 2018-2020 / EnjoyDigital  / florent@enjoy-digital.fr
 *
 */

#ifndef _LINUX_LIMELITEPCIE_H
#define _LINUX_LIMELITEPCIE_H

struct limelitepcie_ioctl_dma_writer {
    uint8_t enable;
    int64_t hw_count;
    int64_t sw_count;
    uint32_t write_size;
    uint8_t irqFreq;
};

struct limelitepcie_ioctl_dma_reader {
    uint8_t enable;
    int64_t hw_count;
    int64_t sw_count;
    uint64_t interruptCounter;
    uint8_t irqFreq;
};

struct limelitepcie_ioctl_lock {
    uint8_t dma_reader_request;
    uint8_t dma_writer_request;
    uint8_t dma_reader_release;
    uint8_t dma_writer_release;
    uint8_t dma_reader_status;
    uint8_t dma_writer_status;
};

struct limelitepcie_ioctl_mmap_dma_info {
    uint64_t dma_tx_buf_offset;
    uint64_t dma_tx_buf_size;
    uint64_t dma_tx_buf_count;

    uint64_t dma_rx_buf_offset;
    uint64_t dma_rx_buf_size;
    uint64_t dma_rx_buf_count;
};

struct limelitepcie_ioctl_mmap_dma_update {
    int64_t sw_count;
    int32_t buffer_size;
    bool genIRQ;
};

struct limelitepcie_cache_flush {
    uint16_t bufferIndex;
    bool isTx;
    bool toDevice;
};

struct limelitepcie_version {
    int major;
    int minor;
    int patch;
};

struct limelitepcie_control_packet {
    uint8_t request[64];
    uint8_t response[64];
    uint32_t length;
    int timeout_ms;
};

#define LIMELITEPCIE_IOCTL 'S'

#define LIMELITEPCIE_IOCTL_DMA_WRITER _IOWR(LIMELITEPCIE_IOCTL, 21, struct limelitepcie_ioctl_dma_writer)
#define LIMELITEPCIE_IOCTL_DMA_READER _IOWR(LIMELITEPCIE_IOCTL, 22, struct limelitepcie_ioctl_dma_reader)
#define LIMELITEPCIE_IOCTL_MMAP_DMA_INFO _IOR(LIMELITEPCIE_IOCTL, 24, struct limelitepcie_ioctl_mmap_dma_info)
#define LIMELITEPCIE_IOCTL_LOCK _IOWR(LIMELITEPCIE_IOCTL, 25, struct limelitepcie_ioctl_lock)
#define LIMELITEPCIE_IOCTL_MMAP_DMA_WRITER_UPDATE _IOW(LIMELITEPCIE_IOCTL, 26, struct limelitepcie_ioctl_mmap_dma_update)
#define LIMELITEPCIE_IOCTL_MMAP_DMA_READER_UPDATE _IOW(LIMELITEPCIE_IOCTL, 27, struct limelitepcie_ioctl_mmap_dma_update)
#define LIMELITEPCIE_IOCTL_CACHE_FLUSH _IOW(LIMELITEPCIE_IOCTL, 28, struct limelitepcie_cache_flush)
#define LIMELITEPCIE_IOCTL_VERSION _IOWR(LIMELITEPCIE_IOCTL, 29, struct limelitepcie_version)
#define LIMELITEPCIE_IOCTL_RUN_CONTROL_COMMAND _IOWR(LIMELITEPCIE_IOCTL, 30, struct limelitepcie_control_packet)

#endif /* _LINUX_LIMELITEPCIE_H */
