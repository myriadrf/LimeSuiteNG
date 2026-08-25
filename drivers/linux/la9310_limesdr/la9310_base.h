/* SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0)
 * Copyright 2017-2018, 2021-2024 NXP
 */
#ifndef __LA9310_BASE_H__
#define __LA9310_BASE_H__

#include "common_headers/la9310_host_if.h"
#include "la9310_softirq.h"
#include "la9310_ioctl.h"

/*Enable the multiple MSIs support */
#define LA9310_REAL_MSI_FLAG (1 << 0)
/* Macros to mark status of IRQ line */
#define LA931XA_MSI_IRQ_FREE 0x45455246 /* 'F''R''E''E' */
#define LA931XA_MSI_IRQ_BUSY 0x59535542 /* 'B''U''S''Y' */

enum la9310_init_stage {
    LA9310_SCRATCH_DMA_INIT_STAGE = 1,
    LA9310_SYSFS_INIT_STAGE,
    LA9310_HANDSHAKE_INIT_STAGE,
    LA9310_IRQ_INIT_STAGE,
    LA9310_SUBDRV_PROBE_STAGE
};

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
    LA9310_VSPA_OVERLAY,
    LA9310_MEM_REGION_FW,
    LA9310_MEM_REGION_DBG_LOG,
    LA9310_MEM_REGION_MAX,
};

/* Number of DMA regions*/
#define LA9310_DMA_REGIONS (LA9310_MEM_REGION_MAX - LA9310_MEM_REGION_BAR_END - 1)

struct la9310_mem_region_info {
    phys_addr_t phys_addr;
    u8 __iomem* vaddr;
    size_t size;
};

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
    u32 scratchrw[32]; /* Scratch Read/Write */
    u8 res_280[0x300 - 0x280];
    u32 scratchw1r[4]; /* Scratch Read (Write once) */
    u8 res_858[0xbfc - 0x310];
};

/*
 * struct la9310_dma_info
 *
 * LA9310 DMA buffer contains details of
 * DMAable buffer shared with LA9310 over PCI. The buffer exsits
 * in Host memory.
 *
 * @host_buf : DMA buffer allocated by linux DMA API (pci_alloc_consistent/
 * dma_alloc_coherent). host_buf.vaddr/host_buf.phys_addr point
 * to virtual and physical address in Host address space. This buffer
 * is mapped to the End-point (LA9310) PCIe address space by creating
 * a outbound window.
 *
 * @ep_phys_addr_start: Physical address of buffer (host_buf.vaddr) in
 * end point (LA9310) address space.
 *
 * @ep_bufs: DMA buffers for sub-drivers. These are initialized by cutting of
 * chunks of buffers from host_buf, thus these buffers are not allocated by
 * Linux DMA API (pci_alloc_consistent/dma_alloc_coherent) and they should
 * not be freed using Linux DMA API. eb_buf[SUBDRV].phys_addr is the physical
 * address of buffer in End-point PCIe address space. It can be programmed
 * directly as source/destination for DMAs inside DMA without any conversion.
 */
struct la9310_dma_info {
    struct la9310_mem_region_info host_buf;
    phys_addr_t ep_pcie_addr;
    struct la9310_mem_region_info ep_bufs[LA9310_DMA_REGIONS];
    uint32_t dma_region_used;
};

#define LA9310_SUBDRV_DMA_REGION_IDX(i) (i - LA9310_MEM_REGION_BAR_END - 1)

/* DMA buffer size definitions */
#define LA9310_FW_DMA_SIZE 0x20000 // TCML memory
#define LA9310_DBUG_LOG_SIZE (4 * 1024)
/* Mem region separator */
#define LA9310_DMA_SEPARATOR_SIZE (64)
#define LA9310_DMA_SEPARATOR_PAINT_CHAR (0xFC)

#define LA9310_DMA_SEPARATOR_TOTAL_SIZE (LA9310_DMA_SEPARATOR_SIZE * LA9310_DMA_REGIONS)
#define LA9310_DMA_ALIGNMENT (64)
#define LA9310_DMA_BUF_SIZE \
    (LA9310_VSPA_FW_SIZE + LA9310_FW_DMA_SIZE + LA9310_DBUG_LOG_SIZE + \
        LA9310_DMA_SEPARATOR_TOTAL_SIZE + LA9310_DMA_ALIGNMENT)

struct la9310_ep_log {
    u8* buf;
    int len;
    int offset;
};

struct irq_info {
    int irq_val;
    int msi_val;
    int free;
};

/**
 * struct la9310_dev - The LA9310 device private structure.
 *
 * This sturcure keeps all the context information of a LA9310 device
 * TBD:XXX: explain other fields as well. Currently priv is explained
 * so that sub-driver developers can understand usage.
 *
 * @vspa_priv:	Sub driver Priv pointers are supposted to point to
 *		sub-driver private data structure that they need to maintain
 *		for all data/info regarding this device. The priv structure
 *		will be allocated by sub-driver during the sub-driver's probe
 *		called by LA9310 core (la9310_sub_driver_ops->probe), and sub-
 *		driver has to initialize la9310_dev-><sub_drv>_priv with pointer
 *		to its private data structure. The sub driver SHOULD & MUST NOT
 *		keep any data global, it should be always part of private
 *		structure to make sure that driver scales when when more than
 *		one LA9310 device are found on system.
 */

struct la9310_dev {
    struct pci_dev* pdev;
    struct device* dev;
    struct class* class;
    struct la9310_sub_driver* sub_drvs;
    struct la9310_mem_region_info mem_regions[LA9310_MEM_REGION_BAR_END];
    struct la9310_msg_unit* msg_units[LA9310_MSG_UNIT_CNT];
    struct la9310_dma_info dma_info;
    struct la9310_mem_region_info iqflood_region;
    struct la9310_hif* hif;
    u32 hif_size;
    struct la9310_ep_log ep_log;
    struct irq_info irq[LA9310_MSI_MAX_CNT];
    int irq_count;
    void* vspa_priv;

    struct platform_device* uart;
    struct la9310_softirq soft_irq;
    struct la9310_userspace_dma user_dma;
};

/*la9310_dev->flags*/
#define LA9310_FLG_PCI_MSI_EN (1 << 0)
#define LA9310_FLG_PCI_MSIX_EN (1 << 1)
#define LA9310_FLG_PCI_INT_LEGACY (1 << 2)
#define LA9310_FLG_PCI_8MSI_EN (1 << 3)

#define LA9310_SET_FLG(flag_var, flg) (flag_var |= flg)
#define LA9310_CHK_FLG(flag_var, flg) (flag_var & flg)
#define LA9310_CLR_FLG(flag_var, flg) (flag_var &= (~flg))

typedef int (*sub_drv_probe_t)(struct la9310_dev* la9310_dev);

typedef int (*sub_drv_remove_t)(struct la9310_dev* la9310_dev);
typedef int (*sub_drv_mod_init_t)(void);
typedef int (*sub_drv_mod_exit_t)(void);

struct la9310_sub_driver_ops {
    sub_drv_probe_t probe;
    sub_drv_remove_t remove;
    sub_drv_mod_init_t mod_init;
    sub_drv_mod_exit_t mod_exit;
};

typedef enum {
    LA9310_SUBDRV_TYPE_VSPA = 1,
} la9310_subdrv_type_t;

struct la9310_sub_driver {
    char* name;
    la9310_subdrv_type_t type;
    struct la9310_sub_driver_ops ops;
};

int vspa_probe(struct la9310_dev* la9310_dev);
int vspa_remove(struct la9310_dev* la9310_dev);

extern int la9310_subdrv_mod_init(void);
extern void la9310_subdrv_mod_exit(void);

void la9310_subdrv_remove(struct la9310_dev* la9310_dev);
int la9310_base_deinit(struct la9310_dev* la9310_dev, int stage, int drv_index);
int la9310_load_rtos_img(struct la9310_dev* la9310_dev, const char __user* fw_data, size_t fw_length);
int la9310_do_reset_handshake(struct la9310_dev* la9310_dev);

int la9310_map_mem_regions(struct la9310_dev* la9310_dev);
void la9310_unmap_mem_regions(struct la9310_dev* la9310_dev);
int la9310_base_probe(struct la9310_dev* la9310_dev);
int la9310_base_remove(struct la9310_dev* la9310_dev);
struct la9310_mem_region_info* la9310_get_dma_region(struct la9310_dev* la9310_dev, enum la9310_mem_region_t type);
int la9310_init_global_sysfs(void);
void la9310_remove_global_sysfs(void);
extern int la9310_get_msi_irq(struct la9310_dev*, enum la9310_msi_id);

void la9310_set_host_ready(struct la9310_dev* la9310_dev, u32 set_bit);
int la9310_raise_msgunit_irq(struct la9310_dev* la9310_dev, int msg_unit_idx, int bit_num);
void raise_msg_interrupt(struct la9310_dev* la9310_dev, uint32_t msg_unit_index, uint32_t ibs);

int la9310_load_m4_firmware(struct la9310_dev* la9310_dev, const char __user* fw_data, size_t fw_length);
#endif
