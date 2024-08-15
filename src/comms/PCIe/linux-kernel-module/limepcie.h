/* SPDX-License-Identifier: BSD-2-Clause
 */

#ifndef _LINUX_LIMEPCIE_H
#define _LINUX_LIMEPCIE_H

struct limepcie_ioctl_dma_control {
    bool directionFromDevice;
    bool enabled;
};

struct limepcie_ioctl_dma_control_continuous {
    uint32_t transferSize;
    uint8_t irqPeriod;
    struct limepcie_ioctl_dma_control control;
};

struct limepcie_ioctl_dma_status {
    uint64_t fromDeviceCounter;
    uint64_t toDeviceCounter;
    bool wait_for_read;
    bool wait_for_write;
};

struct limepcie_ioctl_dma_request {
    uint32_t bufferIndex;
    uint32_t transferSize;
    bool generateIRQ;
    bool directionFromDevice;
};

struct limepcie_cache_flush {
    uint32_t bufferIndex;
    bool directionFromDevice;
    bool sync_to_cpu;
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

struct limepcie_control_packet {
    uint8_t request[64];
    uint8_t response[64];
    uint32_t length;
    int timeout_ms;
};

#define LIMEPCIE_IOCTL 'S'

#define LIMEPCIE_IOCTL_DMA_CONTROL _IOWR(LIMEPCIE_IOCTL, 21, struct limepcie_ioctl_dma_control)
#define LIMEPCIE_IOCTL_DMA_CONTROL_CONTINUOUS _IOWR(LIMEPCIE_IOCTL, 22, struct limepcie_ioctl_dma_control_continuous)
#define LIMEPCIE_IOCTL_DMA_STATUS _IOR(LIMEPCIE_IOCTL, 23, struct limepcie_ioctl_dma_status)
#define LIMEPCIE_IOCTL_DMA_REQUEST _IOW(LIMEPCIE_IOCTL, 26, struct limepcie_ioctl_dma_request)
#define LIMEPCIE_IOCTL_CACHE_FLUSH _IOW(LIMEPCIE_IOCTL, 28, struct limepcie_cache_flush)

#define LIMEPCIE_IOCTL_MMAP_DMA_INFO _IOR(LIMEPCIE_IOCTL, 24, struct limepcie_ioctl_mmap_dma_info)
#define LIMEPCIE_IOCTL_LOCK _IOWR(LIMEPCIE_IOCTL, 25, struct limepcie_ioctl_lock)

#define LIMEPCIE_IOCTL_RUN_CONTROL_COMMAND _IOWR(LIMEPCIE_IOCTL, 30, struct limepcie_control_packet)

#endif /* _LINUX_LIMEPCIE_H */
