/* SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0)
 * Copyright 2017-2018, 2021-2024 NXP
 */
#ifndef __LA9310_BASE_H__
#define __LA9310_BASE_H__

#include "common_headers/la9310_host_if.h"
#include "la9310_events.h"
#include "la9310_ioctl.h"

/**
 *  LA9310 mem region types
 *  NOTE:XXX: Bar regions should be defined
 *  before LA9310_MEM_REGION_BAR_END. these regions
 *  are stored in la9310_dev->mem_regions
 *  NOTE:XXX: DMA BUF regions must be defined
 *  after LA9310_MEM_REGION_BARD_END. these
 *  regions are stored in la9310_dev->dma_info
 */
enum la9310_mem_region_t {
    LA9310_MEM_REGION_CCSR = 0,
    LA9310_MEM_REGION_TCML,
    LA9310_MEM_REGION_TCMU,
    LA9310_MEM_REGION_BAR_END,
};

struct la9310_mem_region_info {
    phys_addr_t phys_addr;
    u8 __iomem* vaddr;
    size_t size;
    struct page* page; /* set for cacheable DMA buffers (cached_dma=1): mapped with dma_mmap_pages() */
};

/* cacheable cached DMA pool (module parameter cached_dma); user space syncs around CPU accesses */
extern bool la9310_cached_dma;

/* Host DMA memory pools for user space, see la9310_dma_pool.h */
enum la9310_dma_pool_id {
    LA9310_DMA_POOL_CACHED, /* sample data, user space syncs it around CPU accesses */
    LA9310_DMA_POOL_UNCACHED, /* firmware staging, status that the device posts, traces */
    LA9310_DMA_POOL_COUNT
};
struct la9310_dma_pool;

struct la9310_ccsr_dcr {
    u32 porsr1; /* POR status 1 */
    u32 porsr2; /* POR status 2 */
    u8 res_008[0x70 - 0x08];
    u32 devdisr1; /* Device disable control 1 */
    u32 devdisr2; /* Device disable control 2 */
    u32 devdisr3; /* Device disable control 3 */
    u32 devdisr4; /* Device disable control 4 */
    u32 devdisr5; /* Device disable control 5 */
    u8 res_084[0x94 - 0x84];
    u32 coredisr; /* core disable register */
    u8 res_098[0xa4 - 0x98];
    u32 svr; /* System version */
    u8 res_0a8[0x200 - 0xa8];
    /*
     * Scratch Read/Write, The host and the M4 firmware use them to communicate:
     *   0      LA9310 ROM/bootloader error code
     *   1      LA9310_BOOT_HSHAKE_SCRATCH_REG: boot handshake state (enum la9310_boot_fsm)
     *   2      LA9310_SCRATCH_MSI_EVENTS_ADDR: M4 address of struct la9310_msi_events for events reporting
     *   3      LA9310_BOOT_HSHAKE_HIF_REG: offset of struct la9310_hif in the M4 TCML
     *   4      LA9310_BOOT_HSHAKE_HIF_SIZ_REG: sizeof(struct la9310_hif)
     *   5-8    unused
     *   9      LA9310_SCRATCH_MLOG_ADDR: M4 address of the firmware's memory log (la9310_sysfs.c)
     *   10-30  unused
     *   31     M4 address of the VSPA debug proxy
     * The M4 bootloader clears 0-8 and 31 when the M4 firmware is reloaded.
     */
    u32 scratchrw[32];
    u8 res_280[0x300 - 0x280];
    u32 scratchw1r[4]; /* Scratch Read (Write once) */
    u8 res_858[0xbfc - 0x310];
};

/*
 * struct la9310_dma_info
 *
 * @host_buf: the persistent DMA buffer, allocated coherent (uncached) for the device's lifetime. The device reaches it
 * through an outbound window at LA9310_EP_DMA_BUF_PHYS_ADDR. The driver doesn't use its contents, see
 * LA9310_PERSISTENT_SIZE in la9310_ioctl.h.
 */
struct la9310_dma_info {
    struct la9310_mem_region_info host_buf;
};

struct MemoryLog_hif {
    uint32_t buffer_addr;
    uint32_t buffer_size;
    uint32_t produced;
    uint32_t host_consumed;
    uint32_t log_level;
};

struct la9310_ep_log {
    struct MemoryLog_hif* hif;
    char* buf;
};

/**
 * struct la9310_dev - The LA9310 device private structure.
 *
 * This sturcure keeps all the context information of a LA9310 device
 */

struct la9310_dev {
    struct pci_dev* pdev;
    struct device* dev;
    struct class* class;
    struct la9310_mem_region_info mem_regions[LA9310_MEM_REGION_BAR_END];
    struct la9310_dma_info dma_info; /* persistent memory */
    struct la9310_dma_pool* dma_pools[LA9310_DMA_POOL_COUNT];
    struct la9310_hif* hif;

    struct platform_device* uart;
    struct la9310_events events;
    bool rcw_done; // sticky: RCW_COMPLETION_DONE stays set until the device is power cycled
    bool buggy_ats; // ATS is enabled in a translating IOMMU domain: unmapping buffers makes IOMMU timeout
};

int la9310_map_mem_regions(struct la9310_dev* la9310_dev);
void la9310_unmap_mem_regions(struct la9310_dev* la9310_dev);
int la9310_base_probe(struct la9310_dev* la9310_dev);
int la9310_base_remove(struct la9310_dev* la9310_dev);
int la9310_init_global_sysfs(void);
void la9310_remove_global_sysfs(void);

void la9310_set_host_ready(struct la9310_dev* la9310_dev, u32 set_bit);

void* endpoint_pa_to_va(struct la9310_dev* la9310_dev, uint32_t ep_pa);

int la9310_init_sysfs(struct la9310_dev* la9310_dev);
void la9310_remove_sysfs(struct la9310_dev* la9310_dev);

// RCW completion register in CCSR (BAR0). Until RCW_COMPLETION_DONE is set, accessing VSPA registers or VSPA memory reboots the host.
#define RCW_COMPLETIONR_ADDR (0x1E60000 + 0x104)
#define RCW_COMPLETION_DONE 0x1
bool la9310_rcw_completed(struct la9310_dev* la9310_dev);

// VSPA DMEM window inside BAR2 (endpoint 0x20400000 - 0x207FFFFF). The rest of BAR2 (TCM) is safe to access before RCW completion.
#define LA9310_BAR2_VSPA_OFFSET 0x400000UL
#define LA9310_BAR2_VSPA_SIZE 0x400000UL

// true if BAR2 byte range [offset, offset + len) overlaps the VSPA DMEM window
static inline bool la9310_bar2_overlaps_vspa(unsigned long offset, unsigned long len)
{
    return offset < LA9310_BAR2_VSPA_OFFSET + LA9310_BAR2_VSPA_SIZE && offset + len > LA9310_BAR2_VSPA_OFFSET;
}
#endif
