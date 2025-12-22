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

// #include "la9310_wdog.h"
// #include "la9310_v2h_if.h"
#ifndef LA9310_RESET_HANDSHAKE_POLLING_ENABLE
    #include <linux/completion.h>
#endif

static int la9310_subdrv_cnt_g;
#ifndef LA9310_RESET_HANDSHAKE_POLLING_ENABLE
struct completion ScratchRegisterHandshake;
#endif
static struct la9310_sub_driver* la9310_get_subdrv(int i);
static int la9310_get_subdrv_virqmap(
    struct la9310_dev* la9310_dev, struct la9310_sub_driver* subdrv, struct virq_evt_map* subdrv_virqmap, int subdrv_virqmap_size);

// int wdog_msi_irq;
// int get_wdog_msi(int wdog_id)
// {
// 	int msi_num = 0;

// 	switch (wdog_id)	{
// 	case WDOG_ID_0:
// 		msi_num = wdog_msi_irq;
// 		break;
// 	default:
// 		break;
// 	}
// 	return msi_num;

// }

#ifndef LA9310_RESET_HANDSHAKE_POLLING_ENABLE
static irqreturn_t host_handshake_handler(int irq, void* dev)
{
    struct la9310_dev* la9310_dev = (struct la9310_dev*)dev;

    dev_info(la9310_dev->dev, "Host Handshake interrupt done!! irq num %d\n", irq);
    complete(&ScratchRegisterHandshake);
    return IRQ_HANDLED;
}
#endif

// void raise_msg_interrupt(struct la9310_dev *la9310_dev,
// 			 uint32_t msg_unit_index, uint32_t ibs)
// {
// 	u32 *msiir_reg;
// 	struct la9310_msg_unit *msg_unit;

// 	msg_unit = (struct la9310_msg_unit *)
// 		(la9310_dev->mem_regions[LA9310_MEM_REGION_CCSR].vaddr
// 		 + MSG_UNIT_OFFSET);
// 	msg_unit = msg_unit + msg_unit_index;
// 	msiir_reg = &msg_unit->msiir;

// 	writel(ibs, msiir_reg);
// }

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

int la9310_dev_reserve_firmware(struct la9310_dev* la9310_dev)
{
    if (la9310_dev->firmware_info.busy)
    {
        dev_err(la9310_dev->dev, "f/w info busy, owner %s\n", la9310_dev->firmware_info.name);
        return -EBUSY;
    }
    la9310_dev->firmware_info.busy = 1;
    return 0;
}

void la9310_dev_free_firmware(struct la9310_dev* la9310_dev)
{
    memset(&la9310_dev->firmware_info.name[0], 0, FIRMWARE_NAME_SIZE);
    la9310_dev->firmware_info.busy = 0;
}

/* Caller should call use the firmware only when la9310_dev_reserve_firmware()
 * return success. After you are done with using firmware call
 * la9310_dev_free_firmware
 */
int la9310_udev_load_firmware(struct la9310_dev* la9310_dev, char* buf, int buff_sz, char* name)
{
    int rc = 0, size;
    struct la9310_firmware_info* fw_info = &la9310_dev->firmware_info;

    if (strlen(name) < FIRMWARE_NAME_SIZE)
    {
        strcpy(fw_info->name, name);
    }
    else
    {
        dev_err(la9310_dev->dev, "Invalid firmware name %s\n", name);
        rc = -ENODEV;
        goto out;
    }

    rc = request_firmware(&fw_info->fw, name, la9310_dev->dev);
    if (rc)
    {
        dev_err(la9310_dev->dev, "Failed to load %s, %d\n", name, rc);
        goto out;
    }

    size = fw_info->fw->size;
    dev_info(la9310_dev->dev, "Downloaded f/w at 0x%px size: %d\n", fw_info->fw->data, size);

    if (buff_sz < size)
    {
        dev_err(la9310_dev->dev, "Insufficient fw buff %p: siz %d\n", buf, size);
        rc = -ENOBUFS;
    }
    else
    {
        dev_info(la9310_dev->dev, "Copy fw to %px, size %d\n", buf, size);
        memcpy_toio(buf, fw_info->fw->data, size);
        la9310_dev->firmware_info.size = size;
    }

    release_firmware(fw_info->fw);
out:
    return rc;
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

int la9310_create_outbound_msi(struct la9310_dev* la9310_dev)
{

    struct la9310_mem_region_info* ccsr_region;
    u64 msi_msg_addr = 0;
    u32 pcie_offset;

    ccsr_region = &la9310_dev->mem_regions[LA9310_MEM_REGION_CCSR];

    dev_info(la9310_dev->dev, "CCSR: vaddr %px, size %d\n", ccsr_region->vaddr, (int)ccsr_region->size);

    /* out bound for MSI */
    pcie_offset = PCIE_RHOM_DBI_BASE + PCIE_MSI_MSG_ADDR_OFF;
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
        la9310_dev->scratch_buf_phys_addr, /*pci addr 1 to 1 map */
        la9310_dev->scratch_buf_size);
    la9310_dev->dma_info.ep_pcie_addr = LA9310_EP_DMA_BUF_PHYS_ADDR;

    dev_info(la9310_dev->dev, "Scratch buf DMA ATU done\n");

    return 0;
}

// void
// la9310_create_ipc_hugepage_outbound(struct la9310_dev *la9310_dev,
// 				 uint64_t phys_addr,
// 				 uint32_t size)
// {
// 	struct la9310_mem_region_info *ccsr_region;

// 	ccsr_region = &la9310_dev->mem_regions[LA9310_MEM_REGION_CCSR];

// 	ls_pcie_iatu_outbound_set(ccsr_region->vaddr + PCIE_RHOM_DBI_BASE,
// 			LA9310_IPC_OUTBOUND_WIN,
// 			PCIE_ATU_TYPE_MEM,
// 			LA9310_USER_HUGE_PAGE_PHYS_ADDR, /*cpu addr*/
// 			phys_addr, /*pci addr 1 to 1 map*/
// 			size);
// 	dev_info(la9310_dev->dev, "Huge Page Buff:0x%llx[H]-0x%x[M],size %d\n",
// 		 phys_addr, LA9310_USER_HUGE_PAGE_PHYS_ADDR, size);
// }

static void la9310_create_rfnm_iqflood_outbound(struct la9310_dev* la9310_dev)
{
    struct la9310_mem_region_info* ccsr_region;

    ccsr_region = &la9310_dev->mem_regions[LA9310_MEM_REGION_CCSR];

    ls_pcie_iatu_outbound_set(ccsr_region->vaddr + PCIE_RHOM_DBI_BASE,
        LA9310_V2H_OUTBOUND_WIN,
        PCIE_ATU_TYPE_MEM,
        LA9310_IQFLOOD_PHYS_ADDR,
        la9310_dev->iq_mem_addr,
        la9310_dev->iq_mem_size);
    dev_info(la9310_dev->dev,
        "RFNM IQFLOOD Buff:0x%x[H]-0x%llx[M],size %d\n",
        LA9310_IQFLOOD_PHYS_ADDR,
        la9310_dev->iq_mem_addr,
        la9310_dev->iq_mem_size);
}

#define IQFLOOD_OUTBOUND_ADDR 0xB0001000
#define IQFLOOD_OUTBOUND_SIZE 0xD000000
#define VSPA_DMEM_PROXY_SIZE 4096
#define VSPA_DMEM_PROXY_ADDR (IQFLOOD_OUTBOUND_ADDR + IQFLOOD_OUTBOUND_SIZE - VSPA_DMEM_PROXY_SIZE)

// Currently using IPC outbound window to map VSPA DMEM PROXY address, which is at IQFLOOD + 256MB
// using this window so that IQFLOOD would not need allocation of 256MB, to gain access to DMEM PROXY
static void la9310_create_ipc_outbound(struct la9310_dev* la9310_dev)
{
    struct la9310_mem_region_info* ccsr_region;

    ccsr_region = &la9310_dev->mem_regions[LA9310_MEM_REGION_CCSR];

    ls_pcie_iatu_outbound_set(ccsr_region->vaddr + PCIE_RHOM_DBI_BASE,
        LA9310_IPC_OUTBOUND_WIN,
        PCIE_ATU_TYPE_MEM,
        VSPA_DMEM_PROXY_ADDR,
        la9310_dev->dmem_proxy.phys_addr,
        4096);
    dev_info(la9310_dev->dev,
        "DMEM PROXY Buff:0x%x[H]-0x%llx[M],size %d\n",
        VSPA_DMEM_PROXY_ADDR,
        la9310_dev->dmem_proxy.phys_addr,
        VSPA_DMEM_PROXY_SIZE);
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
    /*VSPA */
    idx = LA9310_SUBDRV_DMA_REGION_IDX(LA9310_MEM_REGION_VSPA);
    ep_buf = &la9310_dev->dma_info.ep_bufs[idx];
    la9310_init_subdrv_region(la9310_dev, ep_buf, LA9310_VSPA_DMA_SIZE, LA9310_MEM_REGION_VSPA);

    /*FW*/ idx = LA9310_SUBDRV_DMA_REGION_IDX(LA9310_MEM_REGION_FW);
    ep_buf = &la9310_dev->dma_info.ep_bufs[idx];
    la9310_init_subdrv_region(la9310_dev, ep_buf, LA9310_FW_DMA_SIZE, LA9310_MEM_REGION_FW);
    /*LA9310 LOG buffer */
    idx = LA9310_SUBDRV_DMA_REGION_IDX(LA9310_MEM_REGION_DBG_LOG);
    ep_buf = &la9310_dev->dma_info.ep_bufs[idx];
    la9310_init_subdrv_region(la9310_dev, ep_buf, LA9310_DBUG_LOG_SIZE, LA9310_MEM_REGION_DBG_LOG);
    /*IQ Data samples*/
    idx = LA9310_SUBDRV_DMA_REGION_IDX(LA9310_MEM_REGION_IQ_SAMPLES);
    ep_buf = &la9310_dev->dma_info.ep_bufs[idx];
    la9310_init_subdrv_region(la9310_dev, ep_buf, LA9310_IQ_SAMPLES_SIZE, LA9310_MEM_REGION_IQ_SAMPLES);
    /* NLM Operations */
    idx = LA9310_SUBDRV_DMA_REGION_IDX(LA9310_MEM_REGION_NLM_OPS);
    ep_buf = &la9310_dev->dma_info.ep_bufs[idx];
    la9310_init_subdrv_region(la9310_dev, ep_buf, LA9310_NLM_OPS_SIZE, LA9310_MEM_REGION_NLM_OPS);

    idx = LA9310_SUBDRV_DMA_REGION_IDX(LA9310_MEM_REGION_STD_FW);
    ep_buf = &la9310_dev->dma_info.ep_bufs[idx];
    la9310_init_subdrv_region(la9310_dev, ep_buf, LA9310_STD_FW_SIZE, LA9310_MEM_REGION_STD_FW);

    if (la9310_dev->modem_rf_data_size)
    {
        idx = LA9310_SUBDRV_DMA_REGION_IDX(LA9310_MEM_REGION_RF_CAL);
        ep_buf = &la9310_dev->dma_info.ep_bufs[idx];
        la9310_init_subdrv_region(la9310_dev, ep_buf, la9310_dev->modem_rf_data_size, LA9310_MEM_REGION_RF_CAL);
    }
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
        "Unmap and free %s size: %llu, va:0x%px pa:0x%llx bus:0x%llx\n",
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

    la9310_dev->scratch_buf_size = 4 * (1024 * 1024);

    rc = la9310_alloc_dma_buf(la9310_dev->dev, "Scratch buffer", host_region, la9310_dev->scratch_buf_size, DMA_BIDIRECTIONAL);

    if (rc)
        return rc;

    la9310_dev->scratch_buf_phys_addr = host_region->phys_addr;

    if ((!host_region->vaddr) || (la9310_dev->scratch_buf_size < LA9310_DMA_BUF_SIZE))
    {
        dev_err(la9310_dev->dev, "ERR: ioremap DDR Address Failed\n");
        if (la9310_dev->scratch_buf_size < LA9310_DMA_BUF_SIZE)
            dev_err(la9310_dev->dev,
                "Scratch buffer to small (%i), expected >= %i\n",
                la9310_dev->scratch_buf_size,
                LA9310_DMA_BUF_SIZE);
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
    // iounmap(host_region->vaddr);
    // dma_free_coherent(la9310_dev->dev, la9310_dev->scratch_buf_size, scratchBuffer, scratchHandle);

    return rc;
}

static int la9310_verify_hif_compatibility(struct la9310_dev* la9310_dev)
{
    struct la9310_hif* hif = la9310_dev->hif;
    u32 hif_ep_version, hif_host_version;
    int rc = 0;

    if (sizeof(struct la9310_hif) != la9310_dev->hif_size)
    {
        dev_err(la9310_dev->dev,
            "LA931x firmware(%s) not compatible with la9310shiva  driverHIF siz mismatch %d!=%d",
            la9310_dev->firmware_name,
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

void la9310_set_host_ready(struct la9310_dev* la9310_dev, u32 set_bit)
{
    struct la9310_hif* hif = la9310_dev->hif;

    writel((readl(&hif->host_ready) | set_bit), &hif->host_ready);
}

static int la9310_init_hif(struct la9310_dev* la9310_dev)
{
    struct la9310_mem_region_info* tcm_region;
    int rc = 0, i;
    u32* hif_word;

    if (sizeof(struct la9310_hif) > LA9310_EP_HIF_SIZE)
    {

        dev_err(la9310_dev->dev, "HIF struct too big %d", (int)sizeof(struct la9310_hif));
        rc = -EINVAL;
        goto out;
    }

    /*LA9310 Host interface (HIF) @ TCML + LA9310_EP_HIF_OFFSET */
    tcm_region = &la9310_dev->mem_regions[LA9310_MEM_REGION_TCML];
    la9310_dev->hif = (struct la9310_hif*)((u8*)tcm_region->vaddr + LA9310_EP_HIF_OFFSET);

    /* Zeroize HIF. */
    hif_word = (u32*)la9310_dev->hif;
    for (i = 0; i < sizeof(struct la9310_hif) / sizeof(u32); i++)
        writel(0, &hif_word[i]);

out:
    return rc;
}

static void la9310_init_ep_logger(struct la9310_dev* la9310_dev)
{
    struct la9310_mem_region_info* logger_region;
    struct la9310_ep_log* ep_log = &la9310_dev->ep_log;
    struct la9310_hif* hif = la9310_dev->hif;
    struct debug_log_regs* dbg_log_regs;

    logger_region = la9310_get_dma_region(la9310_dev, LA9310_MEM_REGION_DBG_LOG);

    ep_log->buf = logger_region->vaddr;
    ep_log->len = logger_region->size;
    ep_log->offset = 0;

    dev_info(la9310_dev->dev,
        "LA9310 Logger init vaddr %px, phys %llx, size %d\n",
        logger_region->vaddr,
        logger_region->phys_addr,
        (int)logger_region->size);
    /* update HIF to tell LA9310 the pointer to log buffer */
    dbg_log_regs = &hif->dbg_log_regs;
    writel(logger_region->phys_addr, &dbg_log_regs->buf);
    writel(logger_region->size, &dbg_log_regs->len);
    /*Set Default log level to INFO */
    writel(LA9310_LOG_LEVEL_INFO, &dbg_log_regs->log_level);
}

// ssize_t
// la9310_ep_show_stats(void *stats_args, char *buf,
// 		     struct la9310_dev *la9310_dev)
// {
// 	ssize_t len = 0;
// 	int i;
// 	int cnt = 0, max_drop_stat = 0;
// 	struct la9310_stats *stats = (struct la9310_stats *) stats_args;

// 	len += sprintf((buf + len), "LA9310 End Point Stats:\n");

// 	if (la9310_dev->stats_control & (1 << EP_CONTROL_IRQ_STATS)) {
// 		len += sprintf((buf + len), "disabled_evt_try_cnt	%d\n",
// 			       stats->disabled_evt_try_cnt);
// 		len += sprintf((buf + len), "irq_evt_raised	%d\n",
// 			       stats->irq_evt_raised);
// 		len += sprintf((buf + len), "irq_evt_cleared	%d\n",
// 			       stats->irq_evt_cleared);
// 		len += sprintf((buf + len), "irq_mux_tx_msi_cnt	%d\n",
// 			       stats->irq_mux_tx_msi_cnt);
// 		len += sprintf((buf + len), "irq_mux_rx_msi_cnt	%d\n",
// 			       stats->irq_mux_rx_msi_cnt);
// 	}

// 	/* V2H stats */
// 	if (la9310_dev->stats_control & (1 << EP_CONTROL_V2H_STATS)) {
// 		len += sprintf((buf + len), "v2h_sent_pkt	%d\n",
// 			       stats->v2h_sent_pkt);
// 		len += sprintf((buf + len), "v2h_dropped_pkt	%d\n",
// 			       stats->v2h_dropped_pkt);
// 		len += sprintf((buf + len), "v2h_last_sent_pkt	%d\n",
// 			       stats->v2h_last_sent_pkt);
// 		len += sprintf((buf + len), "v2h_last_dropped_pkt	%d\n",
// 			       stats->v2h_last_dropped_pkt);
// 		len += sprintf((buf + len), "v2h_backout_count	%d\n",
// 			       stats->v2h_backout_count);
// 		len += sprintf((buf + len), "v2h_resumed	%d\n",
// 			       stats->v2h_resumed);
// 		if (stats->v2h_resumed < MAX_SENT_RESUME)
// 			max_drop_stat = stats->v2h_resumed;
// 		else
// 			max_drop_stat = MAX_SENT_RESUME;
// 		len += sprintf((buf + len), "V2H packet drop resume stats\n");
// 		for (cnt = 0; cnt < max_drop_stat; cnt++) {
// 			len += sprintf((buf + len),
// 				       "v2h_last_sent_pkt_resumed[%d] %d\n",
// 				       cnt,
// 				       stats->v2h_last_sent_pkt_resumed[cnt]);
// 			len += sprintf((buf + len),
// 				       "v2h_last_dropped_pkt_resumed[%d] %d\n",
// 				       cnt,
// 				       stats->
// 				       v2h_last_dropped_pkt_resumed[cnt]);
// 		}
// 		len += sprintf((buf + len),
// 			       "V2H Sender Ring Stats-All Done\n");

// 		for (cnt = 0; cnt < V2H_MAX_BD; cnt++) {
// 			len += sprintf((buf + len), "ring_owner[%d]	%d\n",
// 				       cnt, stats->v2h_final_ring_owner[cnt]);
// 		}
// 	}
// 	/* V2H stats end */

// 	if (la9310_dev->stats_control & (1 << EP_CONTROL_AVI_STATS)) {
// 		len += sprintf((buf + len), "avi_cm4_mbox0_tx_cnt	%d\n",
// 			       stats->avi_cm4_mbox0_tx_cnt);
// 		len += sprintf((buf + len), "avi_cm4_mbox1_tx_cnt	%d\n",
// 			       stats->avi_cm4_mbox1_tx_cnt);
// 		len += sprintf((buf + len), "avi_cm4_mbox0_rx_cnt	%d\n",
// 			       stats->avi_cm4_mbox0_rx_cnt);
// 		len += sprintf((buf + len), "avi_cm4_mbox1_rx_cnt	%d\n",
// 			       stats->avi_cm4_mbox1_rx_cnt);
// 		len += sprintf((buf + len), "avi_err_queue_full	%d\n",
// 			       stats->avi_err_queue_full);
// 		len += sprintf((buf + len), "avi_intr_raised	%d\n",
// 			       stats->avi_intr_raised);
// 		len += sprintf((buf + len), "avi_mbox_intr_raised	%d\n",
// 			       stats->avi_mbox_intr_raised);
// 	}

// 	if (la9310_dev->stats_control & (1 << EP_CONTROL_EDMA_STATS)) {
// 		len += sprintf((buf + len), "eDMA_ch_allocated	%d\n",
// 			       stats->eDMA_ch_allocated);

// 		for (i = 0; i < LA9310_eDMA_CHANNELS; i++) {
// 			len += sprintf((buf + len),
// 				     "la9310_eDMA_ch[%d]:status	%d\n", i,
// 				       stats->la9310_eDMA_ch[i].status);
// 			len += sprintf((buf + len),
// 				       "la9310_eDMA_ch[%d]:xfer_req	%d\n",
// 				       i, stats->la9310_eDMA_ch[i].xfer_req);
// 			len += sprintf((buf + len),
// 				  "la9310_eDMA_ch[%d]:success_interrupt	%d\n",
// 				       i,
// 				       stats->la9310_eDMA_ch[i].
// 				       success_interrupt);
// 			len += sprintf((buf + len),
// 				    "la9310_eDMA_ch[%d]:error_interrupt	%d\n",
// 				       i,
// 				       stats->la9310_eDMA_ch[i].
// 				       error_interrupt);
// 			len += sprintf((buf + len),
// 				    "la9310_eDMA_ch[%d]:no_callback_reg	%d\n",
// 				       i,
// 				       stats->la9310_eDMA_ch[i].
// 				       no_callback_reg);
// 		}
// 	}

// 	if (la9310_dev->stats_control & (1 << EP_CONTROL_WDOG_STATS)) {
// 		len += sprintf((buf + len), "WDOG_interrupt	%d\n",
// 			       stats->WDOG_interrupt);
// 	}

// 	return len;
// }

// void
// la9310_ep_reset_stats(void *stats_args)
// {
// 	struct la9310_stats *stats = (struct la9310_stats *) stats_args;

// 	memset_io(stats, 0, sizeof(struct la9310_stats));
// }

// int
// la9310_register_ep_stats_ops(struct la9310_dev *la9310_dev)
// {
// 	struct la9310_hif *hif = la9310_dev->hif;
// 	struct la9310_stats_ops stats_ops;

// 	stats_ops.la9310_show_stats = la9310_ep_show_stats;
// 	stats_ops.la9310_reset_stats = la9310_ep_reset_stats;
// 	stats_ops.stats_args = &hif->stats;

// 	return la9310_host_add_stats(la9310_dev, &stats_ops);
// }

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

    dev_info(la9310_dev->dev, "%s: Removing sub-drivers because of error\n", la9310_dev->name);
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

static int la9310_subdrv_init(struct la9310_dev *la9310_dev)
{
	int i, virq_count, rc;
	struct la9310_sub_driver* subdrv;
	struct la9310_sub_driver_ops* ops;
	struct virq_evt_map subdrv_virqmap[IRQ_REAL_MSI_BIT];
	struct virq_evt_map* subdrv_virqmap_ptr;

	dev_info(la9310_dev->dev, "%s: Initiating sub-drivers\n", la9310_dev->name);
	for (i = 0; i < la9310_subdrv_cnt_g; i++) {
		subdrv = la9310_get_subdrv(i);
		ops = &subdrv->ops;
		if (ops->probe) {
			pr_info("%s: subdrv probe : %s\n", __func__, &subdrv->name[0]);

			memset(&subdrv_virqmap[0], 0, sizeof(subdrv_virqmap));
			virq_count = la9310_get_subdrv_virqmap(la9310_dev, subdrv, &subdrv_virqmap[0], IRQ_REAL_MSI_BIT);
			if (virq_count)
				subdrv_virqmap_ptr = &subdrv_virqmap[0];
			else
				subdrv_virqmap_ptr = NULL;
			rc = ops->probe(la9310_dev, virq_count, subdrv_virqmap_ptr);
			if (rc) {
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

int la9310_load_m4_firmware(struct la9310_dev *la9310_dev, const char *m4_fw_name)
{
	int rc;

	strncpy(la9310_dev->firmware_name, m4_fw_name, sizeof(la9310_dev->firmware_name));
	dev_info(la9310_dev->dev, "%s: Loading RTOS image\n", la9310_dev->name);
	rc = la9310_load_rtos_img(la9310_dev);
	if (rc)	{
		dev_err(la9310_dev->dev, "Failed to add RTOS image, err %d", rc);
		return rc;
	}

#ifndef LA9310_RESET_HANDSHAKE_POLLING_ENABLE
	rc = la9310_request_irq(la9310_dev, &la9310_dev->hif->irq_evt_regs);
	if (rc)
	{
		dev_err(la9310_dev->dev, "%s: probe irq req failed, err %d\n", __func__, rc);
		return rc;
	}

	/* scrach register handshake request irq */
	init_completion(&ScratchRegisterHandshake);
	rc = request_irq(
			la9310_get_msi_irq(la9310_dev, MSI_IRQ_HOST_HANDSHAKE), host_handshake_handler, 0, "Host Handshake interrupt", la9310_dev);
	if (rc) {
		dev_err(la9310_dev->dev, "%s: request_irq failed, err %d\n", __func__, rc);
		goto free_hs_irq;
	}

#endif
	dev_info(la9310_dev->dev, "%s: Initiating Reset handshake\n", la9310_dev->name);
	rc = la9310_do_reset_handshake(la9310_dev);
	if (rc)
	{
		dev_err(la9310_dev->dev, "Reset handshake failed, err %d", rc);
		goto free_msi_irq;
	}

	/* Verify that Host and target are using same version of HIF */
	rc = la9310_verify_hif_compatibility(la9310_dev);
	if (rc)
		goto free_msi_irq;

#ifdef LA9310_RESET_HANDSHAKE_POLLING_ENABLE
	rc = la9310_request_irq(la9310_dev, &la9310_dev->hif->irq_evt_regs);
	if (rc)	{
		pr_err("%s: probe irq req failed, err %d\n", __func__, rc);
		goto free_msi_irq;
	}
#endif

	rc = la9310_subdrv_init(la9310_dev);
	if (rc)
		goto free_msi_irq;

	return 0;

free_msi_irq:
#ifdef LA9310_RESET_HANDSHAKE_POLLING_ENABLE
	free_irq(la9310_get_msi_irq(la9310_dev, MSI_IRQ_HOST_HANDSHAKE), la9310_dev);
free_hs_irq:
	la9310_clean_request_irq(la9310_dev, &la9310_dev->hif->irq_evt_regs);
	free_irq(la9310_get_msi_irq(la9310_dev, MSI_IRQ_MUX), la9310_dev);
#endif
	return rc;
}

int la9310_base_probe(struct la9310_dev* la9310_dev)
{
    int rc = 0;
    u32 pci_abserr;
    struct la9310_mem_region_info* ccsr_region;
    // struct device_node *np;
    // struct resource mem_addr;

    la9310_dev->stats_control = LA9310_STATS_DEFAULT_ENABLE_MASK;

    rc = la9310_scratch_dma_buf(la9310_dev);
    if (rc)
    {
        dev_err(la9310_dev->dev, "Failed to init DMA buf for outbound, err %d\n", rc);
	return rc;
    }

    la9310_dev->iq_mem_size = 4 * 1024 * 1024;

    rc = la9310_alloc_dma_buf(
        la9310_dev->dev, "IQ Flood Buffer", &la9310_dev->iqflood_region, la9310_dev->iq_mem_size, DMA_BIDIRECTIONAL);
    if (rc)
        return rc;
    la9310_dev->iq_mem_addr = la9310_dev->iqflood_region.phys_addr;

    la9310_create_rfnm_iqflood_outbound(la9310_dev);
    rc = la9310_alloc_dma_buf(
        la9310_dev->dev, "VSPA DMEM Proxy Buffer", &la9310_dev->dmem_proxy, VSPA_DMEM_PROXY_SIZE, DMA_BIDIRECTIONAL);
    if (rc)
        goto free_iqflood;

    la9310_create_ipc_outbound(la9310_dev);

    rc = la9310_init_hif(la9310_dev);
    if (rc)
        goto free_ipc;
    la9310_init_msg_unit_ptrs(la9310_dev);
    la9310_init_ep_logger(la9310_dev);

    // rc = la9310_init_sysfs(la9310_dev);
    // if (rc)
    // 	goto free_ipc;

    // rc = la9310_register_ep_stats_ops(la9310_dev);
    // if (rc)
    // 	goto free_ipc;

//    rc = la9310_load_m4_firmware(la9310_dev, "la9310.bin");
//    if (rc)
//	    goto free_ipc;

    /* WDOG request_irq */
    // wdog_msi_irq = la9310_get_msi_irq(la9310_dev, MSI_IRQ_WDOG);
    /* Errata A-008822 */
    pci_abserr = PCIE_RHOM_DBI_BASE + PCIE_ABSERR;
    ccsr_region = &la9310_dev->mem_regions[LA9310_MEM_REGION_CCSR];
    writel(PCIE_ABSERR_SETTING, ccsr_region->vaddr + pci_abserr);
    // wdog_set_pci_domain_nr(la9310_dev->id,
    // 		pci_domain_nr(la9310_dev->pdev->bus));
    // wdog_set_modem_status(0, WDOG_MODEM_READY);

    writel(la9310_dev->adc_mask, &la9310_dev->hif->adc_mask);
    writel(la9310_dev->adc_rate_mask, &la9310_dev->hif->adc_rate_mask);
    writel(la9310_dev->dac_mask, &la9310_dev->hif->dac_mask);
    writel(la9310_dev->dac_rate_mask, &la9310_dev->hif->dac_rate_mask);

    writel(LA9310_IQFLOOD_PHYS_ADDR, &la9310_dev->hif->iq_phys_addr);
    writel(la9310_dev->iq_mem_size, &la9310_dev->hif->iq_mem_size);
    writel(la9310_dev->iq_mem_addr, &la9310_dev->hif->iq_mem_addr);
    writel(la9310_dev->modem_rf_data_size, &la9310_dev->hif->modem_rf_data_size);

    la9310_init_ep_pcie_allocator(la9310_dev);
    // rc = la9310_modinfo_init(la9310_dev);

    struct resource* tty_res = NULL;
    tty_res = devm_kzalloc(la9310_dev->dev, sizeof(struct resource), GFP_KERNEL);
    if (!tty_res)
    {
        dev_err(la9310_dev->dev, "Failed to allocate memory for UART\n");
        rc = -1;
        goto free_la_irq;;
    }
    {
        tty_res->start = (resource_size_t)la9310_dev->mem_regions[LA9310_MEM_REGION_CCSR].vaddr + 0x21c0000;
    }
    tty_res->flags = IORESOURCE_REG;
    char* devSymlink = devm_kzalloc(la9310_dev->dev, 64, GFP_KERNEL);
    snprintf(devSymlink, 64, "limesdr_micro_uart");
    tty_res->name = devSymlink;
    la9310_dev->uart = platform_device_register_simple("la9310uart", 1, tty_res, 1);
    if (IS_ERR(la9310_dev->uart))
    {
        dev_err(la9310_dev->dev, "Failed to register UART\n");
        rc = -1;
        goto free_la_irq;;
    }

    return 0;

free_la_irq:
        la9310_clean_request_irq(la9310_dev, &la9310_dev->hif->irq_evt_regs);
free_handshake:
#ifndef LA9310_RESET_HANDSHAKE_POLLING_ENABLE
        free_irq(la9310_get_msi_irq(la9310_dev, MSI_IRQ_HOST_HANDSHAKE), la9310_dev);
        free_irq(la9310_get_msi_irq(la9310_dev, MSI_IRQ_MUX), la9310_dev);
#endif
//free_sysfs:
        // la9310_remove_sysfs(la9310_dev);
free_ipc:
    la9310_free_dma_buf(la9310_dev->dev, "VSPA DMEM Proxy Buffer", &la9310_dev->dmem_proxy, DMA_BIDIRECTIONAL);
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
#ifdef LA9310_RESET_HANDSHAKE_POLLING_ENABLE
        free_irq(la9310_get_msi_irq(la9310_dev, MSI_IRQ_MUX), la9310_dev);
#endif
        __attribute__((__fallthrough__));
        /*Fallthrough */
    case LA9310_IRQ_INIT_STAGE:
        la9310_clean_request_irq(la9310_dev, &la9310_dev->hif->irq_evt_regs);
        __attribute__((__fallthrough__));
        /*Fallthrough */
    case LA9310_HANDSHAKE_INIT_STAGE:
#ifndef LA9310_RESET_HANDSHAKE_POLLING_ENABLE
        free_irq(la9310_get_msi_irq(la9310_dev, MSI_IRQ_HOST_HANDSHAKE), la9310_dev);
        free_irq(la9310_get_msi_irq(la9310_dev, MSI_IRQ_MUX), la9310_dev);
#endif
        __attribute__((__fallthrough__));
        /*Fallthrough */
    case LA9310_SYSFS_INIT_STAGE:
        // la9310_remove_sysfs(la9310_dev);
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

    dev_info(la9310_dev->dev, "%s: Removing LA9310 dev\n", la9310_dev->name);
    host_region = &dma_info->host_buf;
    // iounmap(host_region->vaddr);
    host_region->vaddr = NULL;

    // la9310_modinfo_exit(la9310_dev);
    la9310_subdrv_remove(la9310_dev);

    la9310_clean_request_irq(la9310_dev, &la9310_dev->hif->irq_evt_regs);
    free_irq(la9310_get_msi_irq(la9310_dev, MSI_IRQ_MUX), la9310_dev);
#ifndef LA9310_RESET_HANDSHAKE_POLLING_ENABLE
    free_irq(la9310_get_msi_irq(la9310_dev, MSI_IRQ_HOST_HANDSHAKE), la9310_dev);
#endif
    dev_info(la9310_dev->dev, "%s: Removing sub-drivers\n", la9310_dev->name);

    pci_disable_msi(la9310_dev->pdev);

#if 0
    dma_free_coherent(la9310_dev->dev,
        la9310_dev->scratch_buf_size,
        la9310_dev->dma_info.host_buf.vaddr,
        la9310_dev->dma_info.host_buf.phys_addr);
#endif
    la9310_unmap_mem_regions(la9310_dev);

    // la9310_free_dma_buf(la9310_dev->dev, "VSPA DMEM Proxy Buffer", &la9310_dev->dmem_proxy, DMA_BIDIRECTIONAL);
    // la9310_free_dma_buf(la9310_dev->dev, "IQ FLood Buffer", &la9310_dev->iqflood_region, DMA_BIDIRECTIONAL);
    // la9310_free_dma_buf(la9310_dev->dev, "IQ FLood Buffer", &la9310_dev->iqflood_region, DMA_BIDIRECTIONAL);
    // la9310_free_dma_buf(la9310_dev->dev, "Scratch buffer", host_region, DMA_BIDIRECTIONAL);
    // la9310_remove_sysfs(la9310_dev);

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
            pr_info("%s:[%s] subdrv remove : %s\n", __func__, la9310_dev->name, &subdrv->name[0]);
            rc = ops->remove(la9310_dev);
            if (rc)
            {
                pr_err("%s: %s:[%s] mod exit failed, err %d\n", __func__, la9310_dev->name, &subdrv->name[0], rc);
            }
        }
    }
}

/* Dummy functions to check VSPA/IPC probe/remove invocation from the sub-driver
 * probe/remove. These functions are defined weak so they will be over-ridden
 * when real guys arive in arena.
 */
int __attribute__((weak)) la9310_ipc_probe(struct la9310_dev* la9310_dev, int virq_count, struct virq_evt_map* virq_map)
{
    dev_info(la9310_dev->dev, "[%s]Dummy IPC probe\n", la9310_dev->name);
    return 0;
}

int __attribute__((weak)) la9310_ipc_remove(struct la9310_dev* la9310_dev)
{
    dev_info(la9310_dev->dev, "[%s]Dummy IPC remove\n", la9310_dev->name);
    return 0;
}

int __attribute__((weak)) la9310_ipc_init(void)
{
    pr_debug("Dummy IPC init\n");
    return 0;
}

int __attribute__((weak)) la9310_ipc_exit(void)
{
    pr_debug("Dummy IPC exit\n");
    return 0;
}

int __attribute__((weak)) wdog_init(void)
{
    pr_debug("Dummy WDOG init %s\n", __FILE__);
    return 0;
}

int __attribute__((weak)) wdog_exit(void)
{
    pr_debug("Dummy WDOG exit\n");
    return 0;
}

int __attribute__((weak)) tvd_init(void)
{
    printk(KERN_DEBUG "Dummy TVD init\n");
    return 0;
}

int __attribute__((weak)) tvd_exit(void)
{
    printk(KERN_DEBUG "Dummy TVD exit\n");
    return 0;
}

int __attribute__((weak)) tvd_probe(struct la9310_dev* la9310_dev, int virq_count, struct virq_evt_map* virq_map)
{
    dev_dbg(la9310_dev->dev, "[%s]Dummy TVD probe\n", la9310_dev->name);
    return 0;
}

int __attribute__((weak)) tvd_remove(struct la9310_dev* la9310_dev)
{
    dev_dbg(la9310_dev->dev, "[%s]Dummy TVD remove\n", la9310_dev->name);
    return 0;
}

int __attribute__((weak)) init_tti_dev(void)
{
    printk(KERN_DEBUG "Dummy TTI init\n");
    return 0;
}

int __attribute__((weak)) remove_tti_dev(void)
{
    printk(KERN_DEBUG "Dummy TTI exit\n");
    return 0;
}

int __attribute__((weak)) tti_dev_start(struct la9310_dev* la9310_dev, int virq_count, struct virq_evt_map* virq_map)
{
    dev_dbg(la9310_dev->dev, "[%s]Dummy TTI probe\n", la9310_dev->name);
    return 0;
}

int __attribute__((weak)) tti_dev_stop(struct la9310_dev* la9310_dev)
{
    dev_dbg(la9310_dev->dev, "[%s]Dummy TTI remove\n", la9310_dev->name);
    return 0;
}
/*
 * Sub driver initializer table
 * Add the subdrivers in the order of desired initiazation sequence
 */
static struct la9310_sub_driver sub_drvs_g[] = {
// 	{
// 		.name = "TVD",
// 		.type = LA9310_SUBDRV_TYPE_TVD,
// 		{
// 			.probe = tvd_probe,
// 			.remove = tvd_remove,
// 			.mod_init = tvd_init,
// 			.mod_exit = tvd_exit,
// 		},
// 	},
// 	{
// 		.name = "TTI",
// 		.type = LA9310_SUBDRV_TYPE_TTI,
// 		{
// 			.probe = tti_dev_start,
// 			.remove = tti_dev_stop,
// 			.mod_init = init_tti_dev,
// 			.mod_exit = remove_tti_dev,
// 		},
// 	},
// 	{
// 		.name = "WDOG",
// 		.type = LA9310_SUBDRV_TYPE_WDOG,
// 		{
// 			.mod_init = wdog_init,
// 			.mod_exit = wdog_exit,
// 		},
// 	},
// 	{
// 		.name = "IPC",
// 		.type = LA9310_SUBDRV_TYPE_IPC,
// 		{
// 			.probe = la9310_ipc_probe,
// 			.remove = la9310_ipc_remove,
// 			.mod_init = la9310_ipc_init,
// 			.mod_exit = la9310_ipc_exit,
// 		},
// 	},
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
    // #ifdef NLM_ENABLE_V2H
    // 	{
    // 		.name = "V2H",
    // 		.type = LA9310_SUBDRV_TYPE_V2H,
    // 		{
    // 			.probe = la9310_v2h_probe,
    // 			.remove = la9310_v2h_remove,
    // 		},
    // 	},
    // #endif
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

static int __la9310_get_subdrv_virqmap(
    struct la9310_dev* la9310_dev, struct virq_evt_map* subdrv_virqmap, int subdrv_virqmap_size, u32 subdrv_virq_mask)
{
    int virq_count = 0, subdrv_virq_idx = 0, i;
    struct virq_evt_map* virq_map;
    struct la9310_irq_mux_pram* la9310_irq_priv = la9310_dev->la9310_irq_priv;

    if (subdrv_virq_mask != IRQ_EVT_MSI_MASK)
    {
        virq_map = la9310_irq_priv->virq_map;
        for (i = 0; i < la9310_irq_priv->num_irq; i++)
        {
            dev_dbg(la9310_dev->dev,
                "%s: virq_map: %p, evt: %d, virq: %d\n",
                __func__,
                &virq_map[i],
                virq_map[i].evt,
                virq_map[i].virq);
            if (LA9310_IRQ_EVT(virq_map[i].evt) & subdrv_virq_mask)
            {
                subdrv_virqmap[subdrv_virq_idx].evt = virq_map[i].evt;
                subdrv_virqmap[subdrv_virq_idx].virq = virq_map[i].virq;
                subdrv_virq_idx++;
                virq_count++;
            }
        }
    }
    else
    {
        subdrv_virqmap->evt = IRQ_REAL_MSI_BIT;
        dev_dbg(la9310_dev->dev, "evt = %d virq=%d\n", subdrv_virqmap->evt, subdrv_virqmap->virq);
        virq_count++;
    }

    dev_info(la9310_dev->dev, "virqmap init, evtmask %x, count %d", subdrv_virq_mask, virq_count);
    return virq_count;
}

static int la9310_get_subdrv_virqmap(
    struct la9310_dev* la9310_dev, struct la9310_sub_driver* subdrv, struct virq_evt_map* subdrv_virqmap, int subdrv_virqmap_size)
{
    int virq_count = 0;
    u32 virq_mask;

    switch (subdrv->type)
    {
    case LA9310_SUBDRV_TYPE_IPC:
        virq_mask = IRQ_EVT_IPC_EVT_MASK;
        break;
    case LA9310_SUBDRV_TYPE_VSPA:
        virq_mask = IRQ_EVT_VSPA_EVT_MASK;
        break;
    case LA9310_SUBDRV_TYPE_TEST:
        virq_mask = IRQ_EVT_TEST_EVT_MASK;
        break;
    case LA9310_SUBDRV_TYPE_WDOG:
        virq_mask = IRQ_EVT_MSI_MASK;
        subdrv_virqmap->virq = la9310_dev->irq[MSI_IRQ_WDOG].irq_val;
        break;
    case LA9310_SUBDRV_TYPE_V2H:
        virq_mask = IRQ_EVT_MSI_MASK;
        subdrv_virqmap->virq = la9310_dev->irq[MSI_IRQ_V2H].irq_val;
        break;
    default:
        dev_warn(la9310_dev->dev, "VIRQ MASK not defined for subdrv %s\n", subdrv->name);
        goto out;
    }

    dev_dbg(la9310_dev->dev, "subdrv [%s] virqmap init", subdrv->name);
    virq_count = __la9310_get_subdrv_virqmap(la9310_dev, subdrv_virqmap, subdrv_virqmap_size, virq_mask);
out:
    return virq_count;
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

/*
 * Initializes PCIe outbound window attributes in la9310_dev.
 * Note: Maximum PCIe outbound window size can be of 1 GB.
 */
void la9310_init_ep_pcie_allocator(struct la9310_dev* la9310_dev)
{
    la9310_dev->pci_outbound_win_start_addr = PCI_OUTBOUND_WINDOW_BASE_ADDR;
    la9310_dev->pci_outbound_win_current_addr = PCI_OUTBOUND_WINDOW_BASE_ADDR;
    la9310_dev->pci_outbound_win_limit = 0xE0000000;
}

uint32_t la9310_alloc_ep_pcie_addr(struct la9310_dev* la9310_dev, uint32_t window_size)
{
    uint32_t return_address = la9310_dev->pci_outbound_win_current_addr;

    if ((return_address + window_size) > la9310_dev->pci_outbound_win_limit)
    {
        pr_err("Windows size is exceeding total PCIe memory\n");
        return -1;
    }
    else
    {
        la9310_dev->pci_outbound_win_current_addr = return_address + window_size;
    }
    return return_address;
}

int la9310_raise_msgunit_irq(struct la9310_dev* la9310_dev, int msg_unit_idx, int bit_num)
{
    struct la9310_msg_unit* msg_unit;

    msg_unit = la9310_dev->msg_units[msg_unit_idx];
    writel(bit_num, &msg_unit->msiir);

    return 0;
}
