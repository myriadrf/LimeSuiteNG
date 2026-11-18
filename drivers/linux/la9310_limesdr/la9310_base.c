/* SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0)
 * Copyright 2017-2024 NXP
 */

#define DEBUG

#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/iommu.h>
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
    if (la9310_dev->buggy_ats && la9310_dev->dma_info.host_buf.vaddr)
        dev_warn(la9310_dev->dev,
            "Persistent buffer: intentionally leaking %zu bytes of DMA memory at bus 0x%llx (ATS is enabled)\n",
            la9310_dev->dma_info.host_buf.size,
            la9310_dev->dma_info.host_buf.phys_addr);
    else
        la9310_dma_mem_free(la9310_dev->dev, "Persistent buffer", &la9310_dev->dma_info.host_buf);
}

/*
 * The LA9310 advertises the ATS capability, but doesn't answer the IOMMU's ATS Invalidate Requests.
 * When the device is behind a translating IOMMU domain with ATS enabled, every unmap of its DMA
 * memory ends in IOTLB invalidation timeouts (AMD-Vi: "Completion-Wait loop timed out", IOTLB_INV_TIMEOUT)
 * These timeouts can affect other PCIe devices work that are on the same IOMMU group, and can result in problems like
 * the file system cache failing to flush into NVME SSD.
 * Therefore to avoid causing such problems the DMA buffers are intentionaly not unmapped and leaked.
 * The buffers are allocated once per probe, so the leak would only happen on driver remove (which typically should happen only once before shutdown)
 * Without an IOMMU, or with an identity (passthrough) domain, freeing unmaps no IOVAs and no invalidations are sent.
 */
static bool la9310_ats_check(struct la9310_dev* la9310_dev)
{
    struct pci_dev* pdev = la9310_dev->pdev;
    const int pos = pci_find_ext_capability(pdev, PCI_EXT_CAP_ID_ATS);
    struct iommu_domain* domain = iommu_get_domain_for_dev(&pdev->dev);
    u16 ctrl;

    if (!domain)
        dev_info(la9310_dev->dev, "IOMMU: not used, DMA addresses are physical\n");
    else if (domain->type & __IOMMU_DOMAIN_PAGING)
        dev_info(la9310_dev->dev, "IOMMU: translating domain (type 0x%x)\n", domain->type);
    else
        dev_info(la9310_dev->dev, "IOMMU: non translating domain (type 0x%x), e.g. identity/passthrough\n", domain->type);

    if (!pos)
        return false;
    if (pci_read_config_word(pdev, pos + PCI_ATS_CTRL, &ctrl) || !(ctrl & PCI_ATS_CTRL_ENABLE))
    {
        dev_info(la9310_dev->dev, "ATS capability present, not enabled\n");
        return false;
    }
    if (!domain || !(domain->type & __IOMMU_DOMAIN_PAGING))
    {
        dev_info(la9310_dev->dev, "ATS is enabled, but DMA addresses are not translated, no ATS invalidations expected\n");
        return false;
    }

    dev_warn(la9310_dev->dev,
        "ATS is enabled by the IOMMU, the LA9310 doesn't respond to ATS invalidations. Unmapping its DMA memory would "
        "cause IOTLB invalidation timeouts. To avoid that the driver will intentionally not unmap DMA memory and it will be leaked "
        "when the device is removed during driver unload.\n");
    dev_warn(la9310_dev->dev,
        "Workarounds: boot with 'pci=noats' (disables ATS for all devices) or 'iommu=pt', or with the driver unbound "
        "set this device's IOMMU domain to identity: echo identity > /sys/bus/pci/devices/%s/iommu_group/type\n",
        pci_name(pdev));
    return true;
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

static int la9310_init_irq(struct la9310_dev* la9310_dev)
{
    struct la9310_ccsr_dcr* ccsr_dcr =
        (struct la9310_ccsr_dcr*)(la9310_dev->mem_regions[LA9310_MEM_REGION_CCSR].vaddr + DCR_OFFSET);

    int rc = la9310_events_init(la9310_dev, ccsr_dcr->scratchrw);
    if (rc)
        return rc;
    // window for the M4 to send MSIs through, the vectors are allocated now so the host's MSI address is known
    if (la9310_dev->events.vector_count)
        la9310_create_outbound_msi(la9310_dev);
    return 0;
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

    la9310_dev->buggy_ats = la9310_ats_check(la9310_dev);

    rc = la9310_scratch_dma_buf(la9310_dev);
    if (rc)
    {
        dev_err(la9310_dev->dev, "Failed to init DMA buf for outbound, err %d\n", rc);
        return rc;
    }

    rc = la9310_dma_pools_init(la9310_dev);
    if (rc)
        goto free_dma;

    rc = la9310_init_sysfs(la9310_dev);
    if (rc)
        goto free_dma;

    /* Errata A-008822 */
    uint32_t pci_abserr = PCIE_RHOM_DBI_BASE + PCIE_ABSERR;
    struct la9310_mem_region_info* ccsr_region = &la9310_dev->mem_regions[LA9310_MEM_REGION_CCSR];
    writel(PCIE_ABSERR_SETTING, ccsr_region->vaddr + pci_abserr);

    rc = la9310_init_irq(la9310_dev);
    if (rc)
        goto free_handshake;

    rc = la9310_register_uart(la9310_dev);
    if (rc)
        goto free_irq;

    return 0;

free_irq:
    la9310_events_destroy(la9310_dev);
free_handshake:
    la9310_remove_sysfs(la9310_dev);
free_dma:
    la9310_dma_free_all(la9310_dev);
    return rc;
}

int la9310_base_remove(struct la9310_dev* la9310_dev)
{
    dev_info(la9310_dev->dev, "Removing LA9310 dev\n");

    la9310_events_destroy(la9310_dev);

    if (la9310_dev->uart)
        platform_device_unregister(la9310_dev->uart);

    la9310_dma_free_all(la9310_dev);

    la9310_unmap_mem_regions(la9310_dev);

    la9310_remove_sysfs(la9310_dev);

    return 0;
}
