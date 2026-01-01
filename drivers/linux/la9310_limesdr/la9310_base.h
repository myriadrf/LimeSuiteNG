/* SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0)
 * Copyright 2017-2018, 2021-2024 NXP
 */
#ifndef __LA9310_BASE_H__
#define __LA9310_BASE_H__

#include <linux/if.h>
#include <linux/cdev.h>
#include <linux/semaphore.h>
#include "common_headers/la9310_host_if.h"
#include <linux/firmware.h>

#ifndef LA9310_RESET_HANDSHAKE_POLLING_ENABLE
    #include <linux/completion.h>
#endif

#define FIRMWARE_RTOS "la9310.bin"
#define FIRMWARE_NAME_SIZE 100

/*Boot HandShake timeout in jiffies and retry count */
#if defined(SEEVE)
    #define LA9310_HOST_BOOT_HSHAKE_TIMEOUT 10
    #define LA9310_HOST_BOOT_HSHAKE_RETRIES 40
#else
    #define LA9310_HOST_BOOT_HSHAKE_TIMEOUT 100
    #define LA9310_HOST_BOOT_HSHAKE_RETRIES 60
#endif
#define LA9310_IPC_INIT_WAIT_TIMEOUT 100
#define LA9310_IPC_INIT_WAIT_RETRIES 50
#define NXP_ERRATUM_A008822 1

/*Enable the multiple MSIs support */
#define LA9310_REAL_MSI_FLAG (1 << 0)
/* Macros to mark status of IRQ line */
#define LA931XA_MSI_IRQ_FREE 0x45455246 /* 'F''R''E''E' */
#define LA931XA_MSI_IRQ_BUSY 0x59535542 /* 'B''U''S''Y' */

#define TCML_MEM_SIZE 0x20000
#define TCMU_MEM_SIZE 0x10000
#define MAX_MODEM_INSTANCES 1

#define IN_MB(x) ((x) / (1024 * 1024))
#define IN_KB(x) ((x) / (1024))

#ifndef LA9310_RESET_HANDSHAKE_POLLING_ENABLE
extern struct completion ScratchRegisterHandshake;
#endif

struct la9310_global {
    bool active;
    char dev_name[64];
};
extern struct la9310_global g_la9310_global[];

enum la9310_init_stage {
    LA9310_SCRATCH_DMA_INIT_STAGE = 1,
    LA9310_SYSFS_INIT_STAGE,
    LA9310_HANDSHAKE_INIT_STAGE,
    LA9310_IRQ_INIT_STAGE,
    LA9310_SUBDRV_PROBE_STAGE
};

struct virq_evt_map {
    la9310_irq_evt_bits_t evt;
    int virq;
};

struct la9310_dev;

/**
 *  LA9310 host stats cookie to be used all module for stats sysfs interface
 *  la9310_show_stats: function pointer for show stats counter, module using
 *  this function should set the func ptr with valid stats counter show func
 *  NOTE:XXX: la9310_show_stats should exact value of return number of print
 *  string set i.e. it should be return value of sprintf
 *  la9310_reset_stats: to reset the stats counters module user will point with
 *  their own reset implementation function
 *  stats_arg: stats pointer callback func argument typecasted to void ptr
 */
struct la9310_stats_ops {
    ssize_t (*la9310_show_stats)(void* stats_args, char* buf, struct la9310_dev* la9310_dev);
    void (*la9310_reset_stats)(void* stats_args);
    void* stats_args;
};
struct la9310_host_stats {
    struct la9310_stats_ops stats_ops;
    struct list_head list;
};
struct la9310_irq_mux_stats {
    unsigned long num_virq_evt_raised;
    unsigned long num_hw_irq_recv;
    unsigned long num_msg_unit_irq_evt_raised;
};

struct la9310_irq_mux_pram {
    int irq_base;
    u32* irq_evt_cfg_reg;
    u32* irq_evt_en_reg;
    u32* irq_evt_sts_reg;
    u32* irq_evt_clr_reg;
    struct la9310_irq_mux_stats irq_stats;
    u32 num_irq;
    struct virq_evt_map* virq_map;
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
    LA9310_MEM_REGION_VSPA,
    LA9310_MEM_REGION_FW,
    LA9310_MEM_REGION_DBG_LOG,
    LA9310_MEM_REGION_IQ_SAMPLES,
    LA9310_MEM_REGION_NLM_OPS,
    LA9310_MEM_REGION_STD_FW,
    LA9310_MEM_REGION_RF_CAL,
    LA9310_MEM_REGION_MAX,
};

typedef enum la9310_stat_control {
    EP_CONTROL_IRQ_STATS = 0,
    EP_CONTROL_V2H_STATS,
    EP_CONTROL_AVI_STATS,
    EP_CONTROL_EDMA_STATS,
    EP_CONTROL_WDOG_STATS,
    HOST_CONTROL_IRQ_STATS,
    HOST_CONTROL_VSPA_STATS,
    HOST_CONTROL_V2H_STATS
} la9310_stat_control_t;

#define LA9310_ENABLE_STATS 1
#define LA9310_DISABLE_STATS 0
#define LA9310_STATS_DEFAULT_ENABLE_MASK \
    ((LA9310_ENABLE_STATS << EP_CONTROL_IRQ_STATS) | (LA9310_ENABLE_STATS << EP_CONTROL_V2H_STATS) | \
        (LA9310_ENABLE_STATS << EP_CONTROL_AVI_STATS) | (LA9310_ENABLE_STATS << EP_CONTROL_WDOG_STATS) | \
        (LA9310_ENABLE_STATS << HOST_CONTROL_IRQ_STATS) | (LA9310_ENABLE_STATS << HOST_CONTROL_VSPA_STATS) | \
        (LA9310_ENABLE_STATS << HOST_CONTROL_V2H_STATS))
/* Number of DMA regions*/
#define LA9310_DMA_REGIONS (LA9310_MEM_REGION_MAX - LA9310_MEM_REGION_BAR_END - 1)

struct la9310_mem_region_info {
    phys_addr_t phys_addr;
    u8 __iomem* vaddr;
    size_t size;
};

struct la9310_firmware_info {
    const struct firmware* fw;
    char name[FIRMWARE_NAME_SIZE];
    int busy;
    int size;
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
#define LA9310_VSPA_DMA_SIZE (0)
#define LA9310_FREERTOS_HEAP (24 * 1024)
#define LA9310_FREERTOS_INTERRUPT_STACK 0x1000
#define LA9310_FW_DMA_SIZE (TCML_MEM_SIZE - (LA9310_EP_HIF_SIZE + LA9310_FREERTOS_HEAP + LA9310_FREERTOS_INTERRUPT_STACK))
#define LA9310_DBUG_LOG_SIZE (4 * 1024)
#define LA9310_IQ_SAMPLES_SIZE (1 * 1024 * 1024)
#define LA9310_NLM_OPS_SIZE (256 * 1024)
#define LA9310_STD_FW_SIZE (128 * 1024)
#define LA9310_SHARE_RF_SIZE (200 * 1024) /* 200KB */
/* Mem region separator */
#define LA9310_DMA_SEPARATOR_SIZE (64)
#define LA9310_DMA_SEPARATOR_PAINT_CHAR (0xFC)

#define LA9310_DMA_SEPARATOR_TOTAL_SIZE (LA9310_DMA_SEPARATOR_SIZE * LA9310_DMA_REGIONS)
#define LA9310_DMA_ALIGNMENT (64)
#define LA9310_DMA_BUF_SIZE \
    (LA9310_VSPA_FW_SIZE + LA9310_VSPA_DMA_SIZE + LA9310_FW_DMA_SIZE + LA9310_DBUG_LOG_SIZE + LA9310_IQ_SAMPLES_SIZE + \
        LA9310_NLM_OPS_SIZE + LA9310_DMA_SEPARATOR_TOTAL_SIZE + LA9310_DMA_ALIGNMENT)
#define LA9310_V2H_PCI_ADDR_BASE 0xA4000000 //0xA0500000
#define LA9310_V2H_PCI_ADDR_MAX_SIZE (4096 * V2H_MAX_BD)

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

struct tti_priv {
    int irq;
    int tti_id;
    int gpio;
    uint64_t tti_count;
    /* TBD Timestamp; */
    struct swait_queue_head tti_wq;
    raw_spinlock_t wq_lock;
    int tti_irq_status;
    struct eventfd_ctx* evt_fd_ctxt;
};

/*TTI char Dev data holder */
struct la9310_tti_device_data {
    struct tti_priv* tti_dev;
    struct cdev cdev;
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
    struct la9310_firmware_info firmware_info;
    char name[IFNAMSIZ];
    int id;
    u32 flags;
    struct la9310_sub_driver* sub_drvs;
    struct la9310_mem_region_info mem_regions[LA9310_MEM_REGION_BAR_END];
    struct la9310_msg_unit* msg_units[LA9310_MSG_UNIT_CNT];
    struct la9310_dma_info dma_info;
    struct la9310_mem_region_info iqflood_region;
    struct la9310_boot_header __iomem* boot_header;
    struct la9310_hif* hif;
    u32 hif_offset;
    u32 hif_size;
    struct la9310_ep_log ep_log;
    struct irq_info irq[LA9310_MSI_MAX_CNT];
    int irq_count;
    struct la9310_irq_mux_pram* la9310_irq_priv;
    void* vspa_priv;
    void* ipc_priv;
    void* v2h_priv;
    void* tvd_priv;
    struct la9310_host_stats host_stats;
    struct list_head list;
    uint32_t pci_outbound_win_start_addr;
    uint32_t pci_outbound_win_current_addr;
    uint32_t pci_outbound_win_limit;
    uint32_t stats_control;

    //
    int scratch_buf_size;
    int sdr_board;
    int dac_mask;
    int adc_mask;
    int adc_rate_mask;
    int dac_rate_mask;
    int iq_mem_size;
    uint64_t iq_mem_addr;
    int modem_rf_data_size;
    uint64_t scratch_buf_phys_addr;
    char firmware_name[128];
    char la9310_dev_name[128];

    struct la9310_mem_region_info dmem_proxy;
    struct platform_device* uart;

    u8 arm_m4_fw_loaded;
    u8 vspa_fw_loaded;
};

/*la9310_dev->flags*/
#define LA9310_FLG_PCI_MSI_EN (1 << 0)
#define LA9310_FLG_PCI_MSIX_EN (1 << 1)
#define LA9310_FLG_PCI_INT_LEGACY (1 << 2)
#define LA9310_FLG_PCI_8MSI_EN (1 << 3)

#define LA9310_SET_FLG(flag_var, flg) (flag_var |= flg)
#define LA9310_CHK_FLG(flag_var, flg) (flag_var & flg)
#define LA9310_CLR_FLG(flag_var, flg) (flag_var &= (~flg))

typedef int (*sub_drv_probe_t)(struct la9310_dev* la9310_dev, int virq_count, struct virq_evt_map* virq_map);

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
    LA9310_SUBDRV_TYPE_IPC,
    LA9310_SUBDRV_TYPE_WDOG,
    LA9310_SUBDRV_TYPE_V2H,
    LA9310_SUBDRV_TYPE_TVD,
    LA9310_SUBDRV_TYPE_TTI,
    LA9310_SUBDRV_TYPE_TEST
} la9310_subdrv_type_t;

struct la9310_sub_driver {
    char* name;
    la9310_subdrv_type_t type;
    struct la9310_sub_driver_ops ops;
};

enum la9310_reset_type {
    LA9310_PO_RESET = 1,
    LA9310_COLD_RESET,
    LA9310_WARM_RESET,
};

int vspa_probe(struct la9310_dev* la9310_dev, int virq_count, struct virq_evt_map* virq_map);
int vspa_remove(struct la9310_dev* la9310_dev);

int la9310_test_probe(struct la9310_dev* la9310_dev, int virq_count, struct virq_evt_map* virq_map);
int la9310_test_remove(struct la9310_dev* la9310_dev);
int la9310_v2h_probe(struct la9310_dev* la9310_dev, int virq_count, struct virq_evt_map* virq_map);
int la9310_v2h_remove(struct la9310_dev* la9310_dev);

extern int la9310_subdrv_mod_init(void);
extern void la9310_subdrv_mod_exit(void);

void la9310_subdrv_remove(struct la9310_dev* la9310_dev);
int la9310_base_deinit(struct la9310_dev* la9310_dev, int stage, int drv_index);
int la9310_load_rtos_img(struct la9310_dev* la9310_dev, const char __user* fw_data, size_t fw_length);
int la9310_do_reset_handshake(struct la9310_dev* la9310_dev);
void la9310_hexdump(const void* ptr, size_t sz);
int la9310_map_mem_regions(struct la9310_dev* la9310_dev);
void la9310_unmap_mem_regions(struct la9310_dev* la9310_dev);
int la9310_base_probe(struct la9310_dev* la9310_dev);
int la9310_base_remove(struct la9310_dev* la9310_dev);
int la9310_load_firmware(struct la9310_dev* la9310_dev, char* buf, int buff_sz, const char __user* fw_data, size_t fw_size);
int la9310_udev_load_firmware(struct la9310_dev* la9310_dev, char* buf, int buff_sz, char* name);
struct la9310_mem_region_info* la9310_get_dma_region(struct la9310_dev* la9310_dev, enum la9310_mem_region_t type);
int la9310_dev_reserve_firmware(struct la9310_dev* la9310_dev);
void la9310_dev_free_firmware(struct la9310_dev* la9310_dev);
// int la9310_init_sysfs(struct la9310_dev *la9310_dev);
// void la9310_remove_sysfs(struct la9310_dev *la9310_dev);
int la9310_init_global_sysfs(void);
void la9310_remove_global_sysfs(void);
int la9310_create_outbound_msi(struct la9310_dev* la9310_dev);
int la9310_request_irq(struct la9310_dev* la9310_dev, struct irq_evt_regs* irq_evt_regs);
int la9310_clean_request_irq(struct la9310_dev* la9310_dev, struct irq_evt_regs* irq_evt_regs);
// int la9310_host_add_stats(struct la9310_dev *la9310_dev,
// 			  struct la9310_stats_ops *stats_ops);
int la9310_create_outbound_msi(struct la9310_dev* la9310_dev);
void la9310_create_ipc_hugepage_outbound(struct la9310_dev* la9310_dev, uint64_t phys_addr, uint32_t size);
extern int la9310_get_msi_irq(struct la9310_dev*, enum la9310_msi_id);
int v2h_callback_test_init(struct la9310_dev* la9310_dev);
int v2h_callback_test_deinit(void);
struct la9310_dev* get_la9310_dev_byname(const char* name);
void la9310_init_ep_pcie_allocator(struct la9310_dev* la9310_dev);
uint32_t la9310_alloc_ep_pcie_addr(struct la9310_dev* la9310_dev, uint32_t window_size);
void la9310_set_host_ready(struct la9310_dev* la9310_dev, u32 set_bit);
int la9310_raise_msgunit_irq(struct la9310_dev* la9310_dev, int msg_unit_idx, int bit_num);
int wdog_set_modem_status(int wdog_id, int status);
int wdog_set_pci_domain_nr(int wdog_id, int domain_nr);
void raise_msg_interrupt(struct la9310_dev* la9310_dev, uint32_t msg_unit_index, uint32_t ibs);

ssize_t la9310_show_global_status(char* buf);

int la9310_modinfo_init(struct la9310_dev* dev);
int la9310_load_m4_firmware(struct la9310_dev* la9310_dev, const char __user* fw_data, size_t fw_length);
// int la9310_modinfo_exit(struct la9310_dev *dev);
// void la9310_modinfo_get(struct la9310_dev *la9310_dev, modinfo_t *mi);
#endif
