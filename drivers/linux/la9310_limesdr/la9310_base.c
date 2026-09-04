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

#include "common_headers/la9310_host_if.h"
#include "common_headers/la9310_pci_def.h"
#include "la9310_base.h"
#include "la9310_vspa.h"

#define MSG_UNIT_OFFSET 0x1fc0000
#define DCR_OFFSET 0x1e00000

int la9310_init_sysfs(struct la9310_dev* la9310_dev);
void la9310_remove_sysfs(struct la9310_dev* la9310_dev);

static int la9310_uart_id_counter = 0;
static int la9310_subdrv_cnt_g;
static struct la9310_sub_driver* la9310_get_subdrv(int i);

void* endpoint_pa_to_va(struct la9310_dev* la9310_dev, uint32_t ep_pa)
{
    const uint32_t valid_addr_ranges[3][3] = { { 0x1F800000, 0x1F81FFFF, LA9310_MEM_REGION_TCML },
        { 0x20000000, 0x2000FFFF, LA9310_MEM_REGION_TCMU },
        { 0x20400000, 0x207FFFFF, LA9310_MEM_REGION_TCMU } };

    for (int i = 0; i < 3; ++i)
    {
        if (ep_pa >= valid_addr_ranges[i][0] && ep_pa <= valid_addr_ranges[i][1])
        {
            void* va = (size_t)la9310_dev->mem_regions[valid_addr_ranges[i][2]].vaddr + (ep_pa - valid_addr_ranges[i][0]);
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

static int la9310_scratch_outbound_create(struct la9310_dev* la9310_dev)
{
    struct la9310_mem_region_info* ccsr_region = &la9310_dev->mem_regions[LA9310_MEM_REGION_CCSR];

    ls_pcie_iatu_outbound_set(ccsr_region->vaddr + PCIE_RHOM_DBI_BASE,
        LA9310_SCRATCH_OUTBOUND_WIN,
        PCIE_ATU_TYPE_MEM,
        LA9310_EP_DMA_BUF_PHYS_ADDR, /*cpu addr */
        la9310_dev->dma_info.host_buf.phys_addr, /*pci addr 1 to 1 map */
        la9310_dev->dma_info.host_buf.size);
    la9310_dev->dma_info.ep_pcie_addr = LA9310_EP_DMA_BUF_PHYS_ADDR;

    dev_info(la9310_dev->dev, "Scratch buf DMA ATU done\n");

    return 0;
}

static void la9310_create_iqflood_outbound(struct la9310_dev* la9310_dev)
{
    struct la9310_mem_region_info* ccsr_region;

    ccsr_region = &la9310_dev->mem_regions[LA9310_MEM_REGION_CCSR];

    ls_pcie_iatu_outbound_set(ccsr_region->vaddr + PCIE_RHOM_DBI_BASE,
        LA9310_V2H_OUTBOUND_WIN,
        PCIE_ATU_TYPE_MEM,
        LA9310_IQFLOOD_PHYS_ADDR,
        la9310_dev->iqflood_region.phys_addr,
        la9310_dev->iqflood_region.size);
    dev_info(la9310_dev->dev,
        "IQFLOOD Buff:0x%x[H]-0x%llx[M],size %li\n",
        LA9310_IQFLOOD_PHYS_ADDR,
        la9310_dev->iqflood_region.phys_addr,
        la9310_dev->iqflood_region.size);

    struct la9310_atu* dma_region = &la9310_dev->user_dma.region[la9310_dev->user_dma.region_count++];

    dma_region->host_bus = la9310_dev->iqflood_region.phys_addr;
    dma_region->ep_pa = LA9310_IQFLOOD_PHYS_ADDR;
    dma_region->size = la9310_dev->iqflood_region.size;
    dma_region->flags = 0;
    dma_region->mmap_offset = LA9310_WINDOW_IQFLOOD * PAGE_SIZE;
}

static void la9310_init_subdrv_region(
    struct la9310_dev* la9310_dev, struct la9310_mem_region_info* ep_buf, int size, enum la9310_mem_region_t type)
{
    u8* host_vaddr;
    struct la9310_dma_info* dma_info = &la9310_dev->dma_info;
    struct la9310_mem_region_info* host_dma_region;

    host_dma_region = &la9310_dev->dma_info.host_buf;

    ep_buf->vaddr = host_dma_region->vaddr + dma_info->dma_region_used;
    ep_buf->phys_addr = dma_info->ep_pcie_addr + dma_info->dma_region_used;
    ep_buf->size = size;

    dev_info(la9310_dev->dev, "subdrv DMA region:[%d] offset %d\n", type, dma_info->dma_region_used);
    dev_info(la9310_dev->dev, "Host virtual 0x%px, EP Phys 0x%llx, size %d", ep_buf->vaddr, ep_buf->phys_addr, (int)ep_buf->size);

    /*Paint separator */
    host_vaddr = host_dma_region->vaddr + size + dma_info->dma_region_used;
    memset(host_vaddr, LA9310_DMA_SEPARATOR_PAINT_CHAR, LA9310_DMA_SEPARATOR_SIZE);

    dma_info->dma_region_used += size + LA9310_DMA_SEPARATOR_SIZE;
    dev_info(la9310_dev->dev,
        "Paint addr 0x%px, size %d New offset - 0x%x\n",
        host_vaddr,
        LA9310_DMA_SEPARATOR_SIZE,
        dma_info->dma_region_used);
}

struct la9310_mem_region_info* la9310_get_dma_region(struct la9310_dev* la9310_dev, enum la9310_mem_region_t type)
{
    struct la9310_mem_region_info* ep_buf;
    int idx;

    idx = LA9310_SUBDRV_DMA_REGION_IDX(type);
    ep_buf = &la9310_dev->dma_info.ep_bufs[idx];

    return ep_buf;
}

static void la9310_init_subdrv_dma_buf(struct la9310_dev* la9310_dev)
{
    struct la9310_mem_region_info* ep_buf;
    int idx;

    /*VSPA Overlay*/
    idx = LA9310_SUBDRV_DMA_REGION_IDX(LA9310_VSPA_OVERLAY);
    ep_buf = &la9310_dev->dma_info.ep_bufs[idx];
    la9310_init_subdrv_region(la9310_dev, ep_buf, LA9310_VSPA_FW_SIZE, LA9310_VSPA_OVERLAY);

    /*FW*/ idx = LA9310_SUBDRV_DMA_REGION_IDX(LA9310_MEM_REGION_FW);
    ep_buf = &la9310_dev->dma_info.ep_bufs[idx];
    la9310_init_subdrv_region(la9310_dev, ep_buf, LA9310_FW_DMA_SIZE, LA9310_MEM_REGION_FW);
    /*LA9310 LOG buffer */
    idx = LA9310_SUBDRV_DMA_REGION_IDX(LA9310_MEM_REGION_DBG_LOG);
    ep_buf = &la9310_dev->dma_info.ep_bufs[idx];
    la9310_init_subdrv_region(la9310_dev, ep_buf, LA9310_DBUG_LOG_SIZE, LA9310_MEM_REGION_DBG_LOG);
}

static int la9310_alloc_dma_buf(struct device* dev,
    const char* buf_name,
    struct la9310_mem_region_info* buf_info,
    int buf_size,
    enum dma_data_direction dma_dir)
{
    int status;

    buf_info->vaddr = kzalloc(buf_size, GFP_KERNEL);
    if (!buf_info->vaddr)
    {
        dev_err(dev, "Failed to allocate %s\n", buf_name);
        return -ENOMEM;
    }

    buf_info->size = buf_size;

    buf_info->phys_addr = dma_map_single(dev, buf_info->vaddr, buf_size, dma_dir);
    status = dma_mapping_error(dev, buf_info->phys_addr);
    if (status)
    {
        dev_err(dev, "dma_map_single error @ va:%p pa:%llX for %s\n", buf_info->vaddr, virt_to_phys(buf_info->vaddr), buf_name);
        kfree(buf_info->vaddr);
        return status;
    }

    dev_info(dev,
        "%s size: %i, va:0x%px pa:0x%llx bus:0x%llx\n",
        buf_name,
        buf_size,
        buf_info->vaddr,
        virt_to_phys(buf_info->vaddr),
        buf_info->phys_addr);
    return 0;
}

static int la9310_free_dma_buf(
    struct device* dev, const char* buf_name, struct la9310_mem_region_info* buf_info, enum dma_data_direction dma_dir)
{
    dev_info(dev,
        "Unmap and free %s size: %lu, va:0x%px pa:0x%llx bus:0x%llx\n",
        buf_name,
        buf_info->size,
        buf_info->vaddr,
        virt_to_phys(buf_info->vaddr),
        buf_info->phys_addr);
    if (buf_info->phys_addr)
        dma_unmap_single(dev, buf_info->phys_addr, buf_info->size, dma_dir);
    if (buf_info->vaddr)
        kfree(buf_info->vaddr);

    dev_info(dev, "Unmapped and freed %s\n", buf_name);

    return 0;
}

static int la9310_scratch_dma_buf(struct la9310_dev* la9310_dev)
{
    struct la9310_dma_info* dma_info = &la9310_dev->dma_info;
    struct la9310_mem_region_info* host_region = &dma_info->host_buf;
    int rc = 0;

    uint32_t scratch_buf_size = 4 * (1024 * 1024);

    rc = la9310_alloc_dma_buf(la9310_dev->dev, "Scratch buffer", host_region, scratch_buf_size, DMA_BIDIRECTIONAL);

    if (rc)
        return rc;

    if ((!host_region->vaddr) || (host_region->size < LA9310_DMA_BUF_SIZE))
    {
        dev_err(la9310_dev->dev, "ERR: ioremap DDR Address Failed\n");
        if (host_region->size < LA9310_DMA_BUF_SIZE)
            dev_err(la9310_dev->dev, "Scratch buffer too small (%li), expected >= %i\n", host_region->size, LA9310_DMA_BUF_SIZE);
        return -ENOMEM;
    }
    dma_info->dma_region_used = 0;

    rc = la9310_scratch_outbound_create(la9310_dev);
    if (rc)
    {
        dev_err(la9310_dev->dev, "scratch buf outbound window creation failed\n");
        goto out;
    }

    la9310_init_subdrv_dma_buf(la9310_dev);

    return 0;
out:
    return rc;
}

static int la9310_verify_hif_compatibility(struct la9310_dev* la9310_dev)
{
    struct la9310_hif* hif = la9310_dev->hif;
    u32 hif_ep_version, hif_host_version;
    int rc = 0;

    if (sizeof(struct la9310_hif) > la9310_dev->hif_size)
    {
        dev_err(la9310_dev->dev,
            "LA931x firmware not compatible with la9310shiva  driverHIF siz mismatch %d!=%d",
            (int)sizeof(struct la9310_hif),
            la9310_dev->hif_size);
        rc = -EINVAL;
        goto out;
    }

    hif_host_version = LA9310_VER_MAKE(LA9310_HIF_MAJOR_VERSION, LA9310_HIF_MINOR_VERSION);
    hif_ep_version = readl(&hif->hif_ver);

    if (hif_ep_version != hif_host_version)
    {
        dev_err(la9310_dev->dev, "HIF ver mismatch, Host 0x%x, LA9310 0x%x\n", hif_host_version, hif_ep_version);
        rc = -EINVAL;
        goto out;
    }
    dev_info(la9310_dev->dev, "HIF Version : %d.%d\n", LA9310_VER_MAJOR(hif_ep_version), LA9310_VER_MINOR(hif_ep_version));
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

static int la9310_base_cleanup_subdrv(struct la9310_dev* la9310_dev, int drv_index)
{
    int i = 0, rc = 0;
    struct la9310_sub_driver* subdrv;
    struct la9310_sub_driver_ops* ops;

    dev_info(la9310_dev->dev, "Removing sub-drivers because of error\n");
    for (i = drv_index; i >= 0; i--)
    {
        subdrv = la9310_get_subdrv(i);
        ops = &subdrv->ops;
        if (ops->remove)
        {
            pr_info("%s: subdrv remove : %s\n", __func__, &subdrv->name[0]);

            rc = ops->remove(la9310_dev);
            if (rc)
            {
                pr_err("%s: %s: Remove failed, err %d\n", __func__, &subdrv->name[0], rc);
                /*Other drivers to be removed so continue */
            }
        }
    }
    return rc;
}

static int la9310_subdrv_init(struct la9310_dev* la9310_dev)
{
    int i, rc;
    struct la9310_sub_driver* subdrv;
    struct la9310_sub_driver_ops* ops;

    dev_info(la9310_dev->dev, "Initiating sub-drivers\n");
    for (i = 0; i < la9310_subdrv_cnt_g; i++)
    {
        subdrv = la9310_get_subdrv(i);
        ops = &subdrv->ops;
        if (ops->probe)
        {
            pr_info("%s: subdrv probe : %s\n", __func__, &subdrv->name[0]);

            rc = ops->probe(la9310_dev);
            if (rc)
            {
                pr_err("%s: %s: probe failed, err %d\n", __func__, &subdrv->name[0], rc);
                goto free_subdrv;
            }
        }
    }

    return 0;
free_subdrv:
    la9310_base_cleanup_subdrv(la9310_dev, i);
    return rc;
}

int la9310_load_m4_firmware(struct la9310_dev* la9310_dev, const char __user* fw_data, size_t fw_length)
{
    int rc;
    dev_info(la9310_dev->dev, "Loading M4 firmware image\n");
    rc = la9310_load_rtos_img(la9310_dev, fw_data, fw_length);
    if (rc)
    {
        dev_err(la9310_dev->dev, "Failed to add M4 firmware image, err %d", rc);
        return rc;
    }

    dev_info(la9310_dev->dev, "Initiating Reset handshake\n");
    rc = la9310_do_reset_handshake(la9310_dev);

    if (rc)
    {
        dev_err(la9310_dev->dev, "Reset handshake failed, err %d", rc);
        return rc;
    }

    /* Verify that Host and target are using same version of HIF */
    rc = la9310_verify_hif_compatibility(la9310_dev);
    if (rc)
        return rc;

    if (!la9310_dev->vspa_priv)
    {
        rc = la9310_subdrv_init(la9310_dev);
        if (rc)
            return rc;
    }

    return 0;
}

static irqreturn_t la9310_irq_handler(int irq, void* dev)
{
    struct la9310_dev* la9310_dev = (struct la9310_dev*)dev;
    const struct la9310_ccsr_dcr* ccsr_dcr =
        (struct la9310_ccsr_dcr*)(la9310_dev->mem_regions[LA9310_MEM_REGION_CCSR].vaddr + DCR_OFFSET);
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

    uint32_t iq_mem_size = 4 * 1024 * 1024;

    rc = la9310_alloc_dma_buf(la9310_dev->dev, "IQ Flood Buffer", &la9310_dev->iqflood_region, iq_mem_size, DMA_BIDIRECTIONAL);
    if (rc)
        return rc;

    la9310_create_iqflood_outbound(la9310_dev);

    la9310_init_msg_unit_ptrs(la9310_dev);

    rc = la9310_init_sysfs(la9310_dev);
    if (rc)
        goto free_iqflood;

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

    const struct la9310_ccsr_dcr* ccsr_dcr =
        (struct la9310_ccsr_dcr*)(la9310_dev->mem_regions[LA9310_MEM_REGION_CCSR].vaddr + DCR_OFFSET);
    // const uint32_t* sirq_count_reg = &ccsr_dcr->scratchrw[LA9310_SCRATCH_SIRQ_COUNT_REG];
    la9310_softirq_init(&la9310_dev->soft_irq, ccsr_dcr->scratchrw);

    rc = la9310_register_uart(la9310_dev);
    if (rc)
        goto free_irq;

    // init VSPA, it will abort if M4 has not been programmed, so ignore return value.
    // will reinit after M4 upload
    rc = la9310_subdrv_init(la9310_dev);

    return 0;

free_irq:
    free_irq(la9310_get_msi_irq(la9310_dev, MSI_IRQ_MUX), la9310_dev);
free_handshake:
    la9310_remove_sysfs(la9310_dev);
free_iqflood:
    la9310_free_dma_buf(la9310_dev->dev, "IQ FLood Buffer", &la9310_dev->iqflood_region, DMA_BIDIRECTIONAL);
    return rc;
}

int la9310_base_deinit(struct la9310_dev* la9310_dev, int stage, int drv_index)
{
    struct la9310_dma_info* dma_info = &la9310_dev->dma_info;
    struct la9310_mem_region_info* host_region;

    // la9310_modinfo_exit(la9310_dev);
    switch (stage)
    {
    case LA9310_SUBDRV_PROBE_STAGE:
        la9310_base_cleanup_subdrv(la9310_dev, drv_index);
        __attribute__((__fallthrough__));
        /*Fallthrough */
    case LA9310_IRQ_INIT_STAGE:
        __attribute__((__fallthrough__));
        /*Fallthrough */
    case LA9310_HANDSHAKE_INIT_STAGE:
        __attribute__((__fallthrough__));
        /*Fallthrough */
    case LA9310_SYSFS_INIT_STAGE:
        la9310_remove_sysfs(la9310_dev);
        __attribute__((__fallthrough__));
        /*Fallthrough */
    case LA9310_SCRATCH_DMA_INIT_STAGE:
        host_region = &dma_info->host_buf;
        // iounmap(host_region->vaddr);
    }
    return 0;
}

int la9310_base_remove(struct la9310_dev* la9310_dev)
{
    struct la9310_dma_info* dma_info = &la9310_dev->dma_info;
    struct la9310_mem_region_info* host_region;

    dev_info(la9310_dev->dev, "Removing LA9310 dev\n");

    free_irq(la9310_get_msi_irq(la9310_dev, MSI_IRQ_MUX), la9310_dev);

    if (la9310_dev->uart)
        platform_device_unregister(la9310_dev->uart);

    host_region = &dma_info->host_buf;
    host_region->vaddr = NULL;

    la9310_subdrv_remove(la9310_dev);

    pci_free_irq_vectors(la9310_dev->pdev);
    dev_info(la9310_dev->dev, "Removing sub-drivers\n");

    pci_disable_msi(la9310_dev->pdev);

    la9310_unmap_mem_regions(la9310_dev);

    la9310_remove_sysfs(la9310_dev);

    return 0;
}

void la9310_subdrv_mod_exit(void)
{
    int i, rc = 0;
    struct la9310_sub_driver* subdrv;
    struct la9310_sub_driver_ops* ops;

    pr_info("%s: subdrv mod exit\n", __func__);

    for (i = 0; i < la9310_subdrv_cnt_g; i++)
    {
        subdrv = la9310_get_subdrv(i);
        ops = &subdrv->ops;
        if (ops->mod_exit)
        {
            pr_info("%s: subdrv mod exit : %s\n", __func__, &subdrv->name[0]);
            rc = ops->mod_exit();
            if (rc)
            {
                pr_err("%s: %s: mod exit failed, err %d\n", __func__, &subdrv->name[0], rc);
            }
        }
    }
}

void la9310_subdrv_remove(struct la9310_dev* la9310_dev)
{
    int i, rc = 0;
    struct la9310_sub_driver* subdrv;
    struct la9310_sub_driver_ops* ops;

    for (i = 0; i < la9310_subdrv_cnt_g; i++)
    {
        subdrv = la9310_get_subdrv(i);
        ops = &subdrv->ops;
        if (ops->remove)
        {
            pr_info("%s: subdrv remove : %s\n", __func__, &subdrv->name[0]);
            rc = ops->remove(la9310_dev);
            if (rc)
            {
                pr_err("%s: [%s] mod exit failed, err %d\n", __func__, &subdrv->name[0], rc);
            }
        }
    }
}

/*
 * Sub driver initializer table
 * Add the subdrivers in the order of desired initiazation sequence
 */
static struct la9310_sub_driver sub_drvs_g[] = {
#ifdef NLM_ENABLE_VSPA_LOAD
    {
        .name = "VSPA",
        .type = LA9310_SUBDRV_TYPE_VSPA,
        {
            .probe = vspa_probe,
            .remove = vspa_remove,
        },
    },
#endif
    {}
};

static struct la9310_sub_driver* la9310_get_subdrv(int i)
{
    return &sub_drvs_g[i];
}

extern int la9310_get_msi_irq(struct la9310_dev* la9310_dev, enum la9310_msi_id type)
{
    dev_dbg(la9310_dev->dev, "return irq=%d\n", la9310_dev->irq[type].irq_val);
    return la9310_dev->irq[type].irq_val;
}

int la9310_subdrv_mod_init(void)
{
    int i, rc = 0;
    struct la9310_sub_driver* subdrv;
    struct la9310_sub_driver_ops* ops;

    la9310_subdrv_cnt_g = ARRAY_SIZE(sub_drvs_g);
    pr_info("%s: subdrv mod init:%i\n", __func__, la9310_subdrv_cnt_g);

    for (i = 0; i < la9310_subdrv_cnt_g; i++)
    {
        subdrv = la9310_get_subdrv(i);
        ops = &subdrv->ops;
        if (ops->mod_init)
        {
            pr_info("%s: subdrv mod init : %s\n", __func__, &subdrv->name[0]);
            rc = ops->mod_init();
            if (rc)
            {
                pr_err("%s: %s: mod init failed, err %d\n", __func__, &subdrv->name[0], rc);
                goto out;
            }
        }
    }

out:
    return rc;
}

int la9310_raise_msgunit_irq(struct la9310_dev* la9310_dev, int msg_unit_idx, int bit_num)
{
    struct la9310_msg_unit* msg_unit;

    msg_unit = la9310_dev->msg_units[msg_unit_idx];
    writel(bit_num, &msg_unit->msiir);

    return 0;
}
