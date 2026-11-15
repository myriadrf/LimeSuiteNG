/* SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0)
 * Copyright 2017-2024 NXP
 */

#define DEBUG

#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/of_device.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>

#include "common_headers/la9310_host_if.h"
#include "common_headers/la9310_pci_def.h"
#include "la9310_base.h"
#include "la9310_dma_pool.h"
#include "la9310_pci.h"

#define MSG_UNIT_OFFSET 0x1fc0000
#define DCR_OFFSET 0x1e00000

static int la9310_uart_id_counter = 0;

bool la9310_rcw_completed(struct la9310_dev* la9310_dev)
{
    if (READ_ONCE(la9310_dev->rcw_done))
        return true;

    const u32 val = ioread32((u8 __iomem*)la9310_dev->mem_regions[LA9310_MEM_REGION_CCSR].vaddr + RCW_COMPLETIONR_ADDR);
    if (!(val & RCW_COMPLETION_DONE))
        return false;

    // Once set, it stays set for the rest of the power cycle, so the register doesn't have to be read again */
    WRITE_ONCE(la9310_dev->rcw_done, true);
    return true;
}

void* endpoint_pa_to_va(struct la9310_dev* la9310_dev, uint32_t ep_pa)
{
    const uint32_t valid_addr_ranges[3][3] = { { 0x1F800000, 0x1F81FFFF, LA9310_MEM_REGION_TCML },
        { 0x20000000, 0x2000FFFF, LA9310_MEM_REGION_TCMU },
        { 0x20400000, 0x207FFFFF, LA9310_MEM_REGION_TCMU } };

    for (int i = 0; i < 3; ++i)
    {
        if (ep_pa >= valid_addr_ranges[i][0] && ep_pa <= valid_addr_ranges[i][1])
        {
            uint8_t __iomem* va =
                (uint8_t __iomem*)la9310_dev->mem_regions[valid_addr_ranges[i][2]].vaddr + (ep_pa - valid_addr_ranges[i][0]);
            return va;
        }
    }
    return NULL;
}

int la9310_map_mem_regions(struct la9310_dev* la9310_dev)
{
    struct la9310_mem_region_info* mem_region;
    phys_addr_t phys_addr;
    u8 __iomem* vaddr;
    int i, rc = 0, size;

    for (i = 0; i < LA9310_MEM_REGION_BAR_END; i++)
    {
        mem_region = &la9310_dev->mem_regions[i];
        phys_addr = mem_region->phys_addr;
        size = mem_region->size;
        size = ALIGN(size, PAGE_SIZE);

        if (phys_addr && size)
        {
            vaddr = ioremap(phys_addr, size);
            if (!vaddr)
            {
                dev_err(la9310_dev->dev, "ioremap failed, adr %llx, size %d", phys_addr, size);
                rc = -ENOMEM;
                goto out;
            }
            dev_info(la9310_dev->dev, "mem[%d] phy %llx, vaddr %px\n", i, phys_addr, vaddr);
            mem_region->vaddr = vaddr;
        }
    }

out:
    return rc;
}

void la9310_unmap_mem_regions(struct la9310_dev* la9310_dev)
{
    struct la9310_mem_region_info* mem_region;
    u8 __iomem* vaddr;
    int i;

    for (i = 0; i < LA9310_MEM_REGION_BAR_END; i++)
    {
        mem_region = &la9310_dev->mem_regions[i];
        vaddr = mem_region->vaddr;
        if (vaddr)
        {
            dev_info(la9310_dev->dev, "unmap: region %d, vaddr %p, phys %llx", i, vaddr, mem_region->phys_addr);
            iounmap(vaddr);
            mem_region->vaddr = NULL;
        }
    }
}

static void ls_pcie_iatu_outbound_set(void __iomem* dbi, int idx, int type, u64 cpu_addr, u64 pci_addr, u32 size)
{
    writel(PCIE_ATU_REGION_OUTBOUND | idx, dbi + PCIE_ATU_VIEWPORT);
    writel(lower_32_bits(cpu_addr), dbi + PCIE_ATU_LOWER_BASE);
    writel(upper_32_bits(cpu_addr), dbi + PCIE_ATU_UPPER_BASE);
    writel(lower_32_bits(cpu_addr + size - 1), dbi + PCIE_ATU_LIMIT);
    writel(lower_32_bits(pci_addr), dbi + PCIE_ATU_LOWER_TARGET);
    writel(upper_32_bits(pci_addr), dbi + PCIE_ATU_UPPER_TARGET);
    writel(type, dbi + PCIE_ATU_CR1);
    writel(PCIE_ATU_ENABLE, dbi + PCIE_ATU_CR2);
}

static void ls_pcie_iatu_outbound_disable(void __iomem* dbi, int idx)
{
    writel(PCIE_ATU_REGION_OUTBOUND | idx, dbi + PCIE_ATU_VIEWPORT);
    writel(0, dbi + PCIE_ATU_CR2);
}

static int la9310_create_outbound_msi(struct la9310_dev* la9310_dev)
{
    struct la9310_mem_region_info* ccsr_region;
    u64 msi_msg_addr = 0;

    ccsr_region = &la9310_dev->mem_regions[LA9310_MEM_REGION_CCSR];

    dev_info(la9310_dev->dev, "CCSR: vaddr %px, size %d\n", ccsr_region->vaddr, (int)ccsr_region->size);

    /* out bound for MSI */
    const uint32_t pcie_offset = PCIE_RHOM_DBI_BASE + PCIE_MSI_MSG_ADDR_OFF;
    msi_msg_addr = readl(ccsr_region->vaddr + pcie_offset) | (((u64)readl(ccsr_region->vaddr + pcie_offset + 4)) << 32);

    /* outbound iATU for MSI. From LA9310 to host */
    ls_pcie_iatu_outbound_set(ccsr_region->vaddr + PCIE_RHOM_DBI_BASE,
        LA9310_MSI_OUTBOUND_WIN,
        PCIE_ATU_TYPE_MEM,
        LA9310_EP_TOHOST_MSI_PHY_ADDR,
        msi_msg_addr,
        PCIE_MSI_OB_SIZE);
    dev_info(la9310_dev->dev,
        "MSI:ATU: DBI 0x%px, DMA %llx, EP %x\n",
        (ccsr_region->vaddr + pcie_offset),
        msi_msg_addr,
        LA9310_EP_TOHOST_MSI_PHY_ADDR);
    dev_dbg(la9310_dev->dev, "MSI ATU done\n");
    return 0;
}

/* Maps host memory to the LA9310 through one of its outbound windows */
static void la9310_outbound_create(
    struct la9310_dev* la9310_dev, int window, const char* name, u32 ep_addr, const struct la9310_mem_region_info* region)
{
    struct la9310_mem_region_info* ccsr_region = &la9310_dev->mem_regions[LA9310_MEM_REGION_CCSR];

    ls_pcie_iatu_outbound_set(
        ccsr_region->vaddr + PCIE_RHOM_DBI_BASE, window, PCIE_ATU_TYPE_MEM, ep_addr, region->phys_addr, region->size);
    dev_dbg(la9310_dev->dev,
        "%s ATU[%i]: 0x%08x[LA9310] - 0x%llx[host], size %zu\n",
        name,
        window,
        ep_addr,
        region->phys_addr,
        region->size);
}

static void la9310_outbound_disable(struct la9310_dev* la9310_dev, int window)
{
    struct la9310_mem_region_info* ccsr_region = &la9310_dev->mem_regions[LA9310_MEM_REGION_CCSR];

    ls_pcie_iatu_outbound_disable(ccsr_region->vaddr + PCIE_RHOM_DBI_BASE, window);
}

static int la9310_scratch_outbound_create(struct la9310_dev* la9310_dev)
{
    la9310_outbound_create(la9310_dev,
        LA9310_PERSISTENT_OUTBOUND_WIN,
        "Persistent buffer",
        LA9310_EP_DMA_BUF_PHYS_ADDR,
        &la9310_dev->dma_info.host_buf);
    return 0;
}

static unsigned int la9310_cached_pool_size = 4 * 1024 * 1024;
module_param_named(cached_pool_size, la9310_cached_pool_size, uint, 0444);
MODULE_PARM_DESC(cached_pool_size, "Size in bytes of the cached DMA memory pool");

static unsigned int la9310_uncached_pool_size = 2 * 1024 * 1024;
module_param_named(uncached_pool_size, la9310_uncached_pool_size, uint, 0444);
MODULE_PARM_DESC(uncached_pool_size, "Size in bytes of the uncached DMA memory pool");

static int la9310_dma_pools_init(struct la9310_dev* la9310_dev)
{
    struct la9310_dma_pool* pool;
    int rc;

    rc = la9310_dma_pool_create(
        la9310_dev, LA9310_DMA_POOL_CACHED, la9310_cached_pool_size, LA9310_EP_CACHED_POOL_PHYS_ADDR, la9310_cached_dma);
    if (rc)
        return rc;
    rc = la9310_dma_pool_create(
        la9310_dev, LA9310_DMA_POOL_UNCACHED, la9310_uncached_pool_size, LA9310_EP_UNCACHED_POOL_PHYS_ADDR, false);
    if (rc)
    {
        la9310_dma_pools_destroy(la9310_dev);
        return rc;
    }

    pool = la9310_dev->dma_pools[LA9310_DMA_POOL_CACHED];
    la9310_outbound_create(la9310_dev, LA9310_CACHED_POOL_OUTBOUND_WIN, pool->name, pool->ep_addr, &pool->region);
    pool = la9310_dev->dma_pools[LA9310_DMA_POOL_UNCACHED];
    la9310_outbound_create(la9310_dev, LA9310_UNCACHED_POOL_OUTBOUND_WIN, pool->name, pool->ep_addr, &pool->region);
    return 0;
}

/* The device must not reach the host memory any more, it's freed (pools: once the last user is gone) */
static void la9310_dma_free_all(struct la9310_dev* la9310_dev)
{
    la9310_outbound_disable(la9310_dev, LA9310_CACHED_POOL_OUTBOUND_WIN);
    la9310_outbound_disable(la9310_dev, LA9310_UNCACHED_POOL_OUTBOUND_WIN);
    la9310_outbound_disable(la9310_dev, LA9310_PERSISTENT_OUTBOUND_WIN);
    la9310_dma_pools_destroy(la9310_dev);
    la9310_dma_mem_free(la9310_dev->dev, "Persistent buffer", &la9310_dev->dma_info.host_buf);
}

bool la9310_cached_dma = true;
module_param_named(cached_dma, la9310_cached_dma, bool, 0444);
MODULE_PARM_DESC(cached_dma, "Allocate the cached DMA pool cacheable (synced per buffer) instead of coherent/uncached");

static int la9310_scratch_dma_buf(struct la9310_dev* la9310_dev)
{
    struct la9310_dma_info* dma_info = &la9310_dev->dma_info;
    struct la9310_mem_region_info* host_region = &dma_info->host_buf;
    int rc = 0;

    /* Uncached: the device and user space access it without cache maintenance */
    rc = la9310_dma_mem_alloc(la9310_dev->dev, "Persistent buffer", host_region, PAGE_ALIGN(LA9310_PERSISTENT_SIZE), false);
    if (rc)
        return rc;

    rc = la9310_scratch_outbound_create(la9310_dev);
    if (rc)
    {
        dev_err(la9310_dev->dev, "scratch buf outbound window creation failed\n");
        goto out;
    }

    return 0;
out:
    return rc;
}

static void la9310_init_msg_unit_ptrs(struct la9310_dev* la9310_dev)
{
    struct la9310_msg_unit* msg_unit;
    int i;

    msg_unit = (struct la9310_msg_unit*)(la9310_dev->mem_regions[LA9310_MEM_REGION_CCSR].vaddr + MSG_UNIT_OFFSET);

    for (i = 0; i < LA9310_MSG_UNIT_CNT; i++)
        la9310_dev->msg_units[i] = msg_unit + i;
}

static irqreturn_t la9310_irq_handler(int irq, void* dev)
{
    struct la9310_dev* la9310_dev = (struct la9310_dev*)dev;
    // const struct la9310_ccsr_dcr* ccsr_dcr =
    //     (struct la9310_ccsr_dcr*)(la9310_dev->mem_regions[LA9310_MEM_REGION_CCSR].vaddr + DCR_OFFSET);
    // const uint32_t* sirq_count_reg = &ccsr_dcr->scratchrw[LA9310_SCRATCH_SIRQ_COUNT_REG];
    // const uint32_t sirq_count = readl(sirq_count_reg);
    // if (la9310_dev->soft_irq.irq_counter != sirq_count)
    // {
    //     la9310_dev->soft_irq.irq_counter = sirq_count;
    //     const uint32_t* sirq_status_reg = &ccsr_dcr->scratchrw[LA9310_SCRATCH_SIRQ_STATUS_REG];
    //     const uint32_t sirq_status = readl(sirq_status_reg);
    //     uint32_t bits_to_clear = 0;
    for (int i = 0; i < LA9310_SOFTIRQ_COUNT; ++i)
    {
        // if (sirq_status & (1 << i))
        // {
        //     bits_to_clear |= (1 << i);
        la9310_softirq_signal(la9310_dev, i);
        // }
        }
        // if (bits_to_clear)
        //     la9310_softirq_clear_device(la9310_dev, bits_to_clear, bits_to_clear);
        return IRQ_HANDLED;
    // }
    // return IRQ_NONE;
}

static int la9310_init_irq(struct la9310_dev* la9310_dev)
{
    struct la9310_ccsr_dcr* ccsr_dcr =
        (struct la9310_ccsr_dcr*)(la9310_dev->mem_regions[LA9310_MEM_REGION_CCSR].vaddr + DCR_OFFSET);
    la9310_softirq_init(&la9310_dev->soft_irq, ccsr_dcr->scratchrw);

    la9310_create_outbound_msi(la9310_dev);
    int rc = request_irq(la9310_get_msi_irq(la9310_dev, MSI_IRQ_MUX), la9310_irq_handler, 0, "la9310_dev", (void*)la9310_dev);

    return rc;
}

static int la9310_register_uart(struct la9310_dev* la9310)
{
    struct resource* tty_res = NULL;
    tty_res = devm_kzalloc(la9310->dev, sizeof(struct resource), GFP_KERNEL);
    if (!tty_res)
    {
        dev_err(la9310->dev, "Failed to allocate memory for UART\n");
        return -1;
    }

    tty_res->start = (resource_size_t)la9310->mem_regions[LA9310_MEM_REGION_CCSR].vaddr + 0x21c0000;
    tty_res->flags = IORESOURCE_REG;
    char* devSymlink = devm_kzalloc(la9310->dev, 64, GFP_KERNEL);
    snprintf(devSymlink, 64, "limesdr_micro_uart");
    tty_res->name = devSymlink;
    la9310->uart = platform_device_register_simple("la9310uart", ++la9310_uart_id_counter, tty_res, 1);
    if (IS_ERR(la9310->uart))
    {
        dev_err(la9310->dev, "Failed to register UART\n");
        return -1;
    }
    return 0;
}

int la9310_base_probe(struct la9310_dev* la9310_dev)
{
    int rc = 0;

    /*LA9310 Host interface (HIF) @ TCML + LA9310_EP_HIF_OFFSET */
    struct la9310_mem_region_info* tcm_region = &la9310_dev->mem_regions[LA9310_MEM_REGION_TCML];
    la9310_dev->hif = (struct la9310_hif*)((u8*)tcm_region->vaddr + LA9310_EP_HIF_OFFSET);

    rc = la9310_scratch_dma_buf(la9310_dev);
    if (rc)
    {
        dev_err(la9310_dev->dev, "Failed to init DMA buf for outbound, err %d\n", rc);
        return rc;
    }

    rc = la9310_dma_pools_init(la9310_dev);
    if (rc)
        goto free_dma;

    la9310_init_msg_unit_ptrs(la9310_dev);

    rc = la9310_init_sysfs(la9310_dev);
    if (rc)
        goto free_dma;

    /* WDOG request_irq */
    // wdog_msi_irq = la9310_get_msi_irq(la9310_dev, MSI_IRQ_WDOG);
    /* Errata A-008822 */
    uint32_t pci_abserr = PCIE_RHOM_DBI_BASE + PCIE_ABSERR;
    struct la9310_mem_region_info* ccsr_region = &la9310_dev->mem_regions[LA9310_MEM_REGION_CCSR];
    writel(PCIE_ABSERR_SETTING, ccsr_region->vaddr + pci_abserr);
    // wdog_set_pci_domain_nr(la9310_dev->id, pci_domain_nr(la9310_dev->pdev->bus));
    // wdog_set_modem_status(0, WDOG_MODEM_READY);

    rc = la9310_init_irq(la9310_dev);
    if (rc)
        goto free_handshake;

    rc = la9310_register_uart(la9310_dev);
    if (rc)
        goto free_irq;

    return 0;

free_irq:
    free_irq(la9310_get_msi_irq(la9310_dev, MSI_IRQ_MUX), la9310_dev);
free_handshake:
    la9310_remove_sysfs(la9310_dev);
free_dma:
    la9310_dma_free_all(la9310_dev);
    return rc;
}

int la9310_base_remove(struct la9310_dev* la9310_dev)
{
    dev_info(la9310_dev->dev, "Removing LA9310 dev\n");

    free_irq(la9310_get_msi_irq(la9310_dev, MSI_IRQ_MUX), la9310_dev);

    if (la9310_dev->uart)
        platform_device_unregister(la9310_dev->uart);

    la9310_dma_free_all(la9310_dev);

    pci_free_irq_vectors(la9310_dev->pdev);

    pci_disable_msi(la9310_dev->pdev);

    la9310_unmap_mem_regions(la9310_dev);

    la9310_remove_sysfs(la9310_dev);

    return 0;
}

extern int la9310_get_msi_irq(struct la9310_dev* la9310_dev, enum la9310_msi_id type)
{
    dev_dbg(la9310_dev->dev, "return irq=%d\n", la9310_dev->irq[type].irq_val);
    return la9310_dev->irq[type].irq_val;
}

int la9310_raise_msgunit_irq(struct la9310_dev* la9310_dev, int msg_unit_idx, int bit_num)
{
    struct la9310_msg_unit* msg_unit;

    msg_unit = la9310_dev->msg_units[msg_unit_idx];
    writel(bit_num, &msg_unit->msiir);

    return 0;
}
