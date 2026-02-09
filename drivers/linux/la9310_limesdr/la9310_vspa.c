/* SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0)
 * VSPA device driver
 *
 * This file has been taken from ua_bringup branch
 * and appropriate changes are made to support La9310.
 *
 * Copyright 2017, 2021-2024 NXP
 */

#define DEBUG

#include <linux/pci.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/bitops.h>
#include <linux/fs.h>
#include <linux/wait.h>
#include <linux/uaccess.h>
#include <linux/signal.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/of_address.h>
#include <linux/pid.h>
#include <linux/mm.h>
#include <linux/poll.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/version.h>

#include "la9310_vspa.h"
#include "la9310_pci.h"
#include "la9310_base.h"

int la9310_load_firmware_to_dma(
    struct la9310_dev* la9310_dev, struct la9310_mem_region_info* dma_region, const char __user* fw_data, size_t fw_size);

/* Additional options for checking if Mailbox write to VSP completed */
#ifdef VSPA_DEBUG
    #define MB_CHECK_ON_WRITE /* Check when queuing a new mailbox write */
    #define MB_CHECK_IN_IRQ /* Check during Mailbox IRQ processing	  */
    #define MB_CHECK_TIMER /* Use timer to keep checking after MB OUT */
#endif

#define VSPA_HALT_TIMEOUT (100000)
#define VSPA_STARTUP_TIMEOUT (100000)
#define VSPA_OVERLAY_TIMEOUT (100)

#define CONTROL_REG_MASK (~0x000100FF)
#define CONTROL_PDN_EN (1 << 31)
#define CONTROL_HOST_MSG_GO (1 << 20 | 1 << 21 | 1 << 22 | 1 << 23)
#define CONTROL_VCPU_RESET (1 << 16)
#define CONTROL_DEBUG_MSG_GO (1 << 5)
#define CONTROL_IPPU_GO (1 << 1)
#define CONTROL_HOST_GO (1 << 0)

#define DMA_COMP_STAT_SET 0x01
#define VSPA_DMA_MAX_COUNTER 30
#define VSPA_DMA_TIMEOUT 100

static const struct _vspa_sec_info {
    uint32_t trans_mode;
    char* sec_name;
} vspa_sec_info[] = { { PRAM, ".vpram" },
    { DMEM_NC, ".vdram" },
    { NO_LOAD, ".dmem_cmdbuf" },
    { IPPUPRAM, ".ipram" },
    { DMEM_NC, ".idram" },
    { NO_LOAD, ".ocram_vspa" } },
  overlay_sec_info[] = {

      { DEFAULT_OVERLAY, ".vpram_overlay" }, { OVERLAY_1, ".IQ_data_ovl_ddr" }, { OVERLAY_2, ".CAL_ovl_ddr" }
  };

static ssize_t vspa_show_stats(void* vspa_stats, char* buf, struct la9310_dev* la9310_dev)
{
    int len = 0;
    struct vspa_stats_info* vspa_host_stats;

    if ((la9310_dev->stats_control & (1 << HOST_CONTROL_VSPA_STATS)) == 0)
        return 0;

    vspa_host_stats = (struct vspa_stats_info*)vspa_stats;
    len = sprintf(buf,
        "vspa_loading_count\
		      %d\n",
        vspa_host_stats->vspa_loading_count);

    return len;
}

static void vspa_reset_stats(void* vspa_stats)
{
    struct vspa_stats_info* vspa_host_stats_reset;

    vspa_host_stats_reset = (struct vspa_stats_info*)vspa_stats;
    vspa_host_stats_reset->vspa_loading_count = 0;
}

static int la9310_vspa_stats_init(struct la9310_dev* la9310_dev)
{
    struct la9310_stats_ops vspa_stats_ops;
    struct vspa_device* vspadev = (struct vspa_device*)la9310_dev->vspa_priv;

    vspa_stats_ops.la9310_show_stats = vspa_show_stats;
    vspa_stats_ops.la9310_reset_stats = vspa_reset_stats;
    vspa_stats_ops.stats_args = (void*)&vspadev->vspa_stats;

    // TODO:
    // return la9310_host_add_stats(la9310_dev, &vspa_stats_ops);
    return 0;
}

/* Function to fetch the vspa state for sysfs */
int full_state(struct vspa_device* vspadev)
{
    int state = vspadev->state;

    if (state == VSPA_STATE_UNPROGRAMMED_IDLE || state == VSPA_STATE_RUNNING_IDLE)
    {
        if (vspa_reg_read(vspadev->regs + STATUS_REG_OFFSET) & 0x100)
            state++;
    }
    return state;
}

/*This function will program the DMA in polling mode */
static int dma_raw_transmit(struct vspa_device* vspadev, struct vspa_dma_req* dr)
{
    int stat_abort;
    uint32_t counter = 0;
    volatile int dma_comp_stat, xfr_err, cfg_err;
    u32 __iomem* regs = vspadev->regs;

    /* Program the DMA transfer */
    vspa_reg_write(regs + DMA_DMEM_ADDR_REG_OFFSET, dr->dmem_addr);
    vspa_reg_write(regs + DMA_AXI_ADDR_REG_OFFSET, (dr->axi_addr));
    vspa_reg_write(regs + DMA_BYTE_CNT_REG_OFFSET, dr->byte_cnt);

    dma_comp_stat = vspa_reg_read(regs + DMA_COMP_STAT_REG_OFFSET);
    dev_dbg(vspadev->dev, "dma_comp_stat = %d\n", dma_comp_stat);
    dev_dbg(vspadev->dev,
        "dmem_addr=%x \t axi_addr=%x \t byte_cnt=%x \t\
		xfer_ctrl=%x\n",
        dr->dmem_addr,
        (uint32_t)dr->axi_addr,
        dr->byte_cnt,
        dr->xfr_ctrl);

    vspa_reg_write(regs + DMA_XFR_CTRL_REG_OFFSET, dr->xfr_ctrl);

    counter = VSPA_DMA_MAX_COUNTER;

    while (!(dma_comp_stat & DMA_COMP_STAT_SET))
    {
        dma_comp_stat = vspa_reg_read(regs + DMA_COMP_STAT_REG_OFFSET);
        set_current_state(TASK_INTERRUPTIBLE);
        schedule_timeout(msecs_to_jiffies(VSPA_DMA_TIMEOUT));
        counter--;
    }

    set_current_state(TASK_RUNNING);

    if (!counter)
    {
        dev_err(vspadev->dev, "Timeout Error in Raw transmit\n");
        goto error;
    }

    dma_comp_stat = vspa_reg_read(regs + DMA_COMP_STAT_REG_OFFSET);
    stat_abort = vspa_reg_read(regs + DMA_STAT_ABORT_REG_OFFSET);
    dev_dbg(vspadev->dev, "dma_comp_stat = %x stat_abort %x\n", dma_comp_stat, stat_abort);

    vspa_reg_write(regs + DMA_COMP_STAT_REG_OFFSET, DMA_COMP_STAT_SET);

    xfr_err = vspa_reg_read(regs + DMA_XFRERR_STAT_REG_OFFSET);
    cfg_err = vspa_reg_read(regs + DMA_CFGERR_STAT_REG_OFFSET);
    dev_dbg(vspadev->dev, "DMA_XFRERR value = %x CFG error = %x\n", xfr_err, cfg_err);
    return 0;
error:
    return -ETIMEDOUT;
}

static int vspa_mem_initialization(struct vspa_device* vspadev)
{
    int rc = 0;
    uint32_t* mem_addr = NULL;
    uint8_t count = 0;
    uint32_t axi_data_width, mem_size;
    uint32_t axi_align;
    struct vspa_dma_req z_dma_req;
    struct la9310_mem_region_info* vspa_dma_region = NULL;

    mem_addr = kzalloc(sizeof(uint32_t) * MAX_DMA_TRANSFER, GFP_KERNEL);
    if (!mem_addr)
        return -ENOMEM;

    axi_data_width = vspadev->hardware.axi_data_width;
    axi_align = axi_data_width / BITS_PER_BYTE;

    vspa_dma_region = vspadev->vspa_dma_region;
    if (!vspa_dma_region)
    {
        kfree(mem_addr);
        return -EADDRNOTAVAIL;
    }

    mem_size = vspadev->hardware.dmem_bytes + vspadev->hardware.ippu_bytes + PRAM_MEM_SIZE;

    z_dma_req.dmem_addr = DMEM_ADDR & PRAM_ADDR_MASK;

    while (mem_size > 0)
    {

        if (count > 6)
            break;

        switch (count)
        {
        case AXI_TO_DMEM:
            z_dma_req.byte_cnt = vspadev->hardware.dmem_bytes;
            mem_size = mem_size - z_dma_req.byte_cnt;
            z_dma_req.byte_cnt = align(z_dma_req.byte_cnt, axi_align);
            z_dma_req.xfr_ctrl = count << 8;
            break;

        case AXI_TO_PRAM:
            z_dma_req.byte_cnt = PRAM_MEM_SIZE;
            mem_size = mem_size - z_dma_req.byte_cnt;
            z_dma_req.byte_cnt = align(z_dma_req.byte_cnt, axi_align);
            z_dma_req.xfr_ctrl = count << 8;
            break;

        case AXI_TO_IPPU_PRAM:
            z_dma_req.byte_cnt = vspadev->hardware.ippu_bytes;
            mem_size = mem_size - z_dma_req.byte_cnt;
            z_dma_req.byte_cnt = align(z_dma_req.byte_cnt, axi_align);
            z_dma_req.xfr_ctrl = count << 8;
            break;
        default:
            dev_dbg(vspadev->dev, "No Need to zeroise this memory\n");
            break;
        }

        if (count == AXI_TO_IPPU_PRAM || count == AXI_TO_PRAM || count == AXI_TO_DMEM)
        {

            memcpy_toio(vspa_dma_region->vaddr, (const void*)mem_addr, z_dma_req.byte_cnt);

            // #if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 5, 0)
            // 			dma_map_page_attrs(&vspadev->pdev->dev,
            // 				virt_to_page(vspa_dma_region->vaddr),
            // 				offset_in_page(vspa_dma_region->vaddr), z_dma_req.byte_cnt,
            // 				DMA_TO_DEVICE, 0);
            // #else
            // 			pci_map_single(vspadev->pdev, vspa_dma_region->vaddr,
            // 				z_dma_req.byte_cnt, PCI_DMA_TODEVICE);
            // #endif
            dma_wmb();
            z_dma_req.axi_addr = vspa_dma_region->phys_addr;

            rc = dma_raw_transmit(vspadev, &z_dma_req);
            if (rc < 0)
            {
                dev_err(vspadev->dev, "ERR: Timeout in Raw transmit\n");
                kfree(mem_addr);
                return -ECOMM;
            }
        }
        dev_dbg(vspadev->dev, "Section %d Zeroise successful\n", count);
        count++;
    }

    kfree(mem_addr);
    return 0;
}

/* To start the vspa core after dma is programmed */
static int startup(struct la9310_dev* la9310_dev)
{
    struct vspa_device* vspadev = (struct vspa_device*)la9310_dev->vspa_priv;
    u32 __iomem* regs = vspadev->regs;
    uint32_t val, msb, lsb;
    uint32_t dma_channels;
    uint32_t vspa_sw_version, ippu_sw_version;
    int ctr;

    /* Ask the VSPA to go */
    val = vspa_reg_read(regs + CONTROL_REG_OFFSET);
    val = (val & CONTROL_REG_MASK) | CONTROL_HOST_GO;
    vspa_reg_write(regs + CONTROL_REG_OFFSET, val);
    vspadev->vspa_stats.vspa_loading_count++;
    vspa_sw_version = vspa_reg_read(regs + SWVERSION_REG_OFFSET);
    ippu_sw_version = vspa_reg_read(regs + IPPU_SWVERSION_REG_OFFSET);

    /* Wait for the 64 bit mailbox bit to be set */

    for (ctr = VSPA_STARTUP_TIMEOUT; ctr; ctr--)
    {
        if (vspa_reg_read(regs + HOST_MBOX_STATUS_REG_OFFSET) & MBOX_STATUS_IN_64_BIT)
            break;
        schedule_timeout(1000);
    }
    if (!ctr)
    {
        ERR("%d: timeout waiting for Boot Complete msg\n", vspadev->id);
        goto startup_fail;
    }
    msb = vspa_reg_read(regs + HOST_IN_64_MSB_REG_OFFSET);
    lsb = vspa_reg_read(regs + HOST_IN_64_LSB_REG_OFFSET);
    if (vspadev->debug & DEBUG_STARTUP)
        dev_info(vspadev->dev, "Boot Ok Msg: msb = %08X, lsb = %08X\n", msb, lsb);

    /* Check Boot Complete message */
    if (msb != 0xF1000000)
    {
        dev_err(vspadev->dev, "%d: Boot Complete msg did not match\n", vspadev->id);
        goto startup_fail;
    }
    else
    {
        dev_info(vspadev->dev, "Boot Ok Msg Verified: msb = %08X, lsb = %08X\n", msb, lsb);
    }

    dma_channels = lsb;

    vspa_sw_version = vspa_reg_read(regs + SWVERSION_REG_OFFSET);
    ippu_sw_version = vspa_reg_read(regs + IPPU_SWVERSION_REG_OFFSET);

    /* Set SPM buffer */
    msb = (0x70 << 24) | vspadev->spm_buf_bytes;
    lsb = (uint32_t)(dma_addr_t)vspadev->spm_buf_paddr;
    vspa_reg_write(regs + HOST_OUT_64_MSB_REG_OFFSET, msb);
    vspa_reg_write(regs + HOST_OUT_64_LSB_REG_OFFSET, lsb);
    vspa_sw_version = vspa_reg_read(regs + SWVERSION_REG_OFFSET);
    ippu_sw_version = vspa_reg_read(regs + IPPU_SWVERSION_REG_OFFSET);
    dev_info(vspadev->dev, "SW Version: vspa = %08X, ippu = %08X\n", vspa_sw_version, ippu_sw_version);
    /* Wait for the 64 bit mailbox bit to be set */
    for (ctr = VSPA_STARTUP_TIMEOUT; ctr; ctr--)
    {
        if (vspa_reg_read(regs + HOST_MBOX_STATUS_REG_OFFSET) & MBOX_STATUS_IN_64_BIT)
            break;
        schedule_timeout(100);
    }
    if (!ctr)
    {
        ERR("%d: timeout waiting for SPM Ack msg\n", vspadev->id);
        goto startup_fail;
    }
    msb = vspa_reg_read(regs + HOST_IN_64_MSB_REG_OFFSET);
    lsb = vspa_reg_read(regs + HOST_IN_64_LSB_REG_OFFSET);
    dev_info(vspadev->dev, "SPM Ack Msg: msb = %08X, lsb = %08X\n", msb, lsb);
    if (vspadev->debug & DEBUG_STARTUP)
        dev_info(vspadev->dev, "SPM Ack Msg: msb = %08X, lsb = %08X\n", msb, lsb);
    if (msb != 0xF0700000)
    {
        ERR("%d: SPM Ack error %08X\n", vspadev->id, msb);
        goto startup_fail;
    }

    /*This piece of code is for passing the PCI base address to VSPA via
	 * mailbox 0. Error Print will come since VSPA FW image is not having
	 * response support for base address implemented */

    val = vspa_reg_read(regs + CONTROL_REG_OFFSET);

    if (dma_channels)
    {
        vspadev->spm_dma_chan = (dma_channels >> 24) & 0xFF;
        vspadev->bulk_dma_chan = (dma_channels >> 16) & 0xFF;
        vspadev->reply_dma_chan = (dma_channels >> 8) & 0xFF;
        vspadev->cmd_dma_chan = (dma_channels) & 0xFF;
    }
    else
    { /* legacy images */
        vspadev->spm_dma_chan = VSPA_DMA_CHANNELS;
        vspadev->bulk_dma_chan = 0xFF;
        vspadev->reply_dma_chan = 0xFF;
        vspadev->cmd_dma_chan = vspadev->legacy_cmd_dma_chan;
    }

    if (vspadev->debug & DEBUG_STARTUP)
    {
        dev_info(vspadev->dev, "SW Version: vspa = %08X, ippu = %08X\n", vspa_sw_version, ippu_sw_version);
        dev_info(vspadev->dev,
            "DMA chan: spm %02X, bulk %02X, reply %02X, cmd %02X\n",
            vspadev->spm_dma_chan,
            vspadev->bulk_dma_chan,
            vspadev->reply_dma_chan,
            vspadev->cmd_dma_chan);
    }

    vspadev->versions.vspa_sw_version = vspa_sw_version;
    vspadev->versions.ippu_sw_version = ippu_sw_version;
    vspadev->state = VSPA_STATE_RUNNING_IDLE;
    init_completion(&vspadev->watchdog_complete);
    return 0;

startup_fail:
    vspadev->versions.vspa_sw_version = ~0;
    vspadev->versions.ippu_sw_version = ~0;
    vspadev->state = VSPA_STATE_STARTUP_ERR;
    vspa_reg_write(vspadev->regs + IRQEN_REG_OFFSET, 0);
    return -EIO;
}

static int get_lma(uint8_t* file_start, uint32_t pg_hd_off, uint32_t ph_num, uint32_t vma, uint32_t* lma)
{
    int i = 0;
    struct program_header* ph = (struct program_header*)(file_start + pg_hd_off);

    for (i = 0; i < ph_num; i++)
    {
        if (ph[i].prg_vaddr == vma)
        {
            *lma = ph[i].prg_paddr;
            /* reset 2nd nibble as requested by VSPA f/w team */
            *lma = (*lma & 0xF0FFFFFF);
            return LIBVSPA_ERR_OK;
        }
    }

    return -LIBVSPA_ERR_INVALID_FILE;
}

int _strncmp(const char* s1, const char* s2, size_t n)
{
    while (n && *s1 && (*s1 == *s2))
    {
        ++s1;
        ++s2;
        --n;
    }
    if (n == 0)
    {
        return 0;
    }
    else
    {
        return (*(unsigned char*)s1 - *(unsigned char*)s2);
    }
}
#define LA_STRNCMP _strncmp

static char* get_overlay_section(uint8_t* string_table, int sec_name_offset)
{
    int8_t i, len = 0;

    if (!string_table)
        return NULL;

    for (i = 0; i < MAX_OVERLAY_SECTIONS; i++)
    {
        len = strlen(overlay_sec_info[i].sec_name);
        if (!LA_STRNCMP((char*)(string_table + sec_name_offset), overlay_sec_info[i].sec_name, len))
            return overlay_sec_info[i].sec_name;
    }
    return NULL;
}

static int8_t getsectiontype(uint8_t* string_table, int sec_name_offset)
{
    int8_t i = 0, len = 0;

    if (!string_table)
        return -1;

    len = strlen(vspa_sec_info[i].sec_name);

    for (i = 0; i < MAX_VSP_SECTION; i++)
    {
        if (!LA_STRNCMP((char*)(string_table + sec_name_offset), vspa_sec_info[i].sec_name, len))
            return i;
    }
    return -1;
}

int vspa_fw_dma_write(struct la9310_dev* la9310_dev, struct dma_param* linfo, uint32_t flags, int overlay_enable)
{
    uint32_t size_to_xfer = linfo->size;
    static uint8_t id;
    int rc = 0;
    struct vspa_dma_req dma_req;
    int8_t err = -LIBVSPA_ERR_EAGAIN;
    struct la9310_mem_region_info* vspa_dma_region;
    struct vspa_device* vspadev = (struct vspa_device*)la9310_dev->vspa_priv;

    dev_dbg(la9310_dev->dev,
        "INFO: %s :DMA_write: mode = %x from = %0llx \
			to = %06x total %06x bytes\n",
        __func__,
        linfo->xfr_ctrl,
        linfo->phys,
        linfo->offset,
        linfo->size);

    do
    {
        size_to_xfer = linfo->size > MAX_DMA_TRANSFER ? MAX_DMA_TRANSFER : linfo->size;

        dma_req.type = 0;
        dma_req.id = id++;
        dma_req.flags = VSPA_FLAG_REPORT_DMA_COMPLETE;
        dma_req.axi_addr = linfo->phys;
        dma_req.dmem_addr = linfo->offset & PRAM_ADDR_MASK;
        dma_req.byte_cnt = size_to_xfer & DMA_BCNT_MASK;
        dma_req.xfr_ctrl = linfo->xfr_ctrl;
        dev_dbg(la9310_dev->dev,
            "DMA: %08X %08X %0llX %08X %08X\n",
            dma_req.control,
            dma_req.dmem_addr,
            dma_req.axi_addr,
            dma_req.byte_cnt,
            dma_req.xfr_ctrl);

        linfo->phys += size_to_xfer; /* Increment the src ptr */
        linfo->size -= size_to_xfer; /* Decrement the byte count */

        if (!overlay_enable)
            linfo->offset += size_to_xfer; /* Increment the dst
							 ptr */

        /*Fetching DMA region address */
        vspa_dma_region = vspadev->vspa_dma_region;
        if (!vspa_dma_region)
        {
            dev_err(la9310_dev->dev, "ERR %s: DMA region not found", __func__);
            return -ENOMEM;
        }

        memcpy(vspa_dma_region->vaddr, (const void*)dma_req.axi_addr, dma_req.byte_cnt);

        // #if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 5, 0)
        // 		dma_map_page_attrs(&vspadev->pdev->dev,
        // 				virt_to_page(vspa_dma_region->vaddr),
        // 				offset_in_page(vspa_dma_region->vaddr), dma_req.byte_cnt,
        // 				(enum dma_data_direction)DMA_TO_DEVICE, 0);
        // #else
        // 		pci_map_single(vspadev->dev, vspa_dma_region->vaddr,
        // 				dma_req.byte_cnt, PCI_DMA_TODEVICE);
        // #endif
        dma_sync_single_for_device(
            la9310_dev->dev, la9310_dev->dma_info.host_buf.phys_addr, la9310_dev->dma_info.host_buf.size, DMA_BIDIRECTIONAL);
        dma_wmb();
        dma_req.axi_addr = vspa_dma_region->phys_addr;
        dev_dbg(la9310_dev->dev,
            "vspa%d: ctrl %08x, dmem %08x,\
				axi %08llx, cnt %08x, ctrl %08x\n",
            vspadev->id,
            dma_req.control,
            dma_req.dmem_addr,
            dma_req.axi_addr,
            dma_req.byte_cnt,
            dma_req.xfr_ctrl);
        rc = dma_raw_transmit(vspadev, &dma_req);
        if (rc < 0)
        {
            dev_err(vspadev->dev, "ERR: Timeout in Raw transmit\n");
            return -ECOMM;
        }
        dma_sync_single_for_cpu(
            la9310_dev->dev, la9310_dev->dma_info.host_buf.phys_addr, la9310_dev->dma_info.host_buf.size, DMA_BIDIRECTIONAL);

    } while (linfo->size != 0 && err == 0);

    dev_dbg(la9310_dev->dev, "DMA_write: operation complete\n");

    return 0;
}

static int fw_read_and_load_sections(struct la9310_dev* la9310_dev,
    uint8_t* file_start,
    uint32_t pg_hd_off,
    uint32_t sec_header_off,
    int sec_cnt,
    int str_section_off,
    uint32_t fw_size,
    uint32_t ph_num,
    struct vspa_hardware* hardware)
{
    int i = 0, ret = -1, overlay_link_add = 1;
    int8_t sec_type = 0;
    uint32_t axi_align = 0;
    uint32_t* section_ptr = NULL;
    uint8_t* str = NULL;
    dma_addr_t aligned_addr = 0;
    uint32_t section_size = 0, section_vma = 0;
    uint32_t section_lma = 0;
    struct section_header* sec_header = NULL;
    struct dma_param dparam = { 0 };
    uint8_t section_parsed = 0x0;
    uint32_t axi_data_width;
    char* section_name;
    struct vspa_device* vspadev = (struct vspa_device*)la9310_dev->vspa_priv;

    if (file_start == NULL)
    {
        dev_err(la9310_dev->dev, "Load sections: file start is NULL");
        return -EEXIST;
    }

    dma_sync_single_for_cpu(
        la9310_dev->dev, la9310_dev->dma_info.host_buf.phys_addr, la9310_dev->dma_info.host_buf.size, DMA_BIDIRECTIONAL);

    /*Fetching Section header from firmware file */
    sec_header = (struct section_header*)(file_start + sec_header_off);

    dev_dbg(la9310_dev->dev, "INFO %s: Section Count: %d", __func__, sec_cnt);
    str = (file_start + sec_header[str_section_off].sec_offset);
    for (i = 0; i < sec_cnt; i++)
    {
        sec_type = getsectiontype(str, sec_header[i].sec_name);
        if (sec_type < 0 || sec_type >= MAX_VSP_SECTION)
        {
            dev_dbg(la9310_dev->dev, "Section %d: type 0x%02X ignored", i, sec_type);
            section_name = get_overlay_section(str, sec_header[i].sec_name);
            if (section_name)
            {
                dev_info(la9310_dev->dev, "Section Name: %s found\n", section_name);
                vspadev->overlay_sec[overlay_link_add].name = section_name;
                section_vma = sec_header[i].sec_addr;
                section_lma = 0x0;
                if (get_lma(file_start, pg_hd_off, ph_num, section_vma, &section_lma) > LIBVSPA_ERR_OK)
                {
                    dev_err(la9310_dev->dev,
                        "Load sections: Can't find LMA \
						 of %s\n",
                        vspa_sec_info[sec_type].sec_name);
                    return -LIBVSPA_ERR_INVALID_FILE;
                }

                section_size = sec_header[i].sec_size;

                dev_dbg(la9310_dev->dev,
                    "Load section: VMA 0x%08X LMA 0x%08X \
					   size 0x%X",
                    section_vma,
                    section_lma,
                    section_size);

                section_size = align(section_size, axi_align);
                aligned_addr = (dma_addr_t)file_start;
                aligned_addr = align(aligned_addr, axi_align);

                section_ptr = (uint32_t*)(file_start + sec_header[i].sec_offset);
                vspadev->overlay_sec[overlay_link_add].dpram.phys = (uint64_t)section_ptr;

                vspadev->overlay_sec[overlay_link_add].dpram.size = section_size;
                overlay_link_add++;
            }
        }
        else
        {
            dev_dbg(la9310_dev->dev, "Section %d: type 0x%02X (%s)", i, sec_type, vspa_sec_info[sec_type].sec_name);

            section_parsed |= (1 << sec_type);

            section_vma = sec_header[i].sec_addr;
            section_lma = 0x0;
            if (get_lma(file_start, pg_hd_off, ph_num, section_vma, &section_lma) > LIBVSPA_ERR_OK)
            {
                dev_err(la9310_dev->dev, "Load sections: Can't find LMA of %s\n", vspa_sec_info[sec_type].sec_name);
                return -LIBVSPA_ERR_INVALID_FILE;
            }

            section_size = sec_header[i].sec_size;

            dev_dbg(la9310_dev->dev, "Load section: VMA 0x%08X LMA 0x%08X size 0x%X", section_vma, section_lma, section_size);

            /* skip zero size section */
            if (section_size == 0)
                continue;

            /*change */
            if (sec_type == CMDBUFF)
            {
                dev_info(la9310_dev->dev,
                    "Load section: cmd_buf_addr 0x%08X, \
					  cmd_buf_size 0x%08X\n",
                    section_lma,
                    section_size);
            }
            else
            {
                if (section_size > fw_size)
                {
                    dev_err(la9310_dev->dev,
                        "Load section: Section too big \
						 0x%08X",
                        section_size);
                    return -LIBVSPA_ERR_ENOMEM;
                }
                axi_data_width = hardware->axi_data_width;
                axi_align = axi_data_width / BITS_PER_BYTE;

                if (!isaligned(section_lma, axi_align))
                {
                    dev_err(la9310_dev->dev,
                        "Load section: Unaligned LMA \
						 0x%08X for section '%s'",
                        section_lma,
                        vspa_sec_info[sec_type].sec_name);
                    return -LIBVSPA_ERR_EINVOFFSET;
                }

                section_size = align(section_size, axi_align);
                aligned_addr = (dma_addr_t)file_start;
                aligned_addr = align(aligned_addr, axi_align);

                section_ptr = (uint32_t*)(file_start + sec_header[i].sec_offset);
                dev_dbg(la9310_dev->dev, "Section header address : %p\n", section_ptr);
                if (OCRAMVSPA == sec_type)
                {
                    dev_info(la9310_dev->dev, "Inside condition OCRAMVSPA\n");
                }
                else
                {
                    dparam.phys = (uint64_t)section_ptr;
                    dparam.size = section_size;
                    dparam.offset = section_lma;
                    dparam.xfr_ctrl = vspa_sec_info[sec_type].trans_mode;
                    section_name = get_overlay_section(str, sec_header[i].sec_name);
                    if (section_name)
                    {
                        if (!LA_STRNCMP(DEFAULT_OVERLAY_SEC_NAME, section_name, strlen(DEFAULT_OVERLAY_SEC_NAME)))
                        {
                            vspadev->overlay_sec[0].name = section_name;
                            vspadev->overlay_sec[0].dpram = dparam;
                            vspadev->overlay_sec_loaded = section_name;
                        }
                    }
                    ret = vspa_fw_dma_write(la9310_dev, &dparam, BLOCK, 0);
                    if (ret < 0)
                    {
                        dev_err(la9310_dev->dev,
                            "Load sections:DMA \
							 failed (%d)",
                            ret);
                        return ret;
                    }
                }
            }
        }
    }

    dev_dbg(la9310_dev->dev, "Load sections: sections parsed 0x%08X", section_parsed);

    return LIBVSPA_ERR_OK;
}

static int la9310_load_vspa_image(struct la9310_dev* la9310_dev, char* vaddr, int vspa_fw_size)
{
    struct file_header* hd;
    struct vspa_hardware hardware = { 0 };
    struct vspa_device* vspadev;

    vspadev = (struct vspa_device*)la9310_dev->vspa_priv;

    hardware = vspadev->hardware;

    dev_dbg(la9310_dev->dev, "%s: AXI data width %d\n", __func__, hardware.axi_data_width);

    /* FW Header Initialization */
    hd = (struct file_header*)vaddr;

    /* Image Header Format Checking */
    if (GCC_MACH_CODE != hd->fh_machine && ISCAPE_MACH_CODE != hd->fh_machine)
    {
        dev_err(la9310_dev->dev, "Load_elf: bad hdr fh_machine 0x%02X", hd->fh_machine);
        // TODO: vfree(vaddr);
        return -ENOEXEC;
    }

    /* FW image parsing and load sections */
    if (fw_read_and_load_sections(
            la9310_dev, vaddr, hd->fh_phoff, hd->fh_shoff, hd->fh_shnum, hd->fh_shstrndx, vspa_fw_size, hd->fh_phnum, &hardware))
    {
        dev_err(la9310_dev->dev, "ERR %s: Image Section Loading Failed", __func__);
        return -EIO;
    }

    return 0;
}

/*Overlay DMA Function */
int overlay_initiate(struct device* dev, struct overlay_section overlay_sec)
{
    struct la9310_dev* la9310_dev = dev_get_drvdata(dev);
    struct vspa_device* vspadev = (struct vspa_device*)la9310_dev->vspa_priv;
    int ret = 0;
    u32 __iomem* vspa_reg;
    int overlay_enable_dma = 1;

    overlay_sec.dpram.xfr_ctrl = vspadev->overlay_sec[0].dpram.xfr_ctrl;
    overlay_sec.dpram.offset = vspadev->overlay_sec[0].dpram.offset;

    ret = vspa_fw_dma_write(la9310_dev, &overlay_sec.dpram, BLOCK, overlay_enable_dma);
    if (ret < 0)
    {
        dev_err(la9310_dev->dev, "Load sections:DMA failed (%d)\n", ret);
        return ret;
    }

    vspa_reg = (u32 __iomem*)(la9310_dev->mem_regions[LA9310_MEM_REGION_TCMU].vaddr + 0x400000 + overlay_sec.dpram.offset);

#ifdef DEBUG_VSPA
    ret = memcmp((const void*)overlay_sec.dpram.phys, (const void*)vspa_reg, overlay_sec.dpram.size);
    if (ret == 0)
        dev_info(la9310_dev->dev, "Section loading verified\n");
#endif

    vspadev->overlay_sec_loaded = overlay_sec.name;
    return 0;
}

static int vspa_get_fw_image(struct la9310_dev* la9310_dev, const char __user* fw_data, size_t vspa_fw_size)
{
    int ret = 0;
    int buf_size;
    uint32_t axi_align = 0, axi_data_width = 0;
    struct vspa_device* vspadev = (struct vspa_device*)la9310_dev->vspa_priv;
    struct la9310_mem_region_info* vspa_fw_region;

    vspa_fw_region = la9310_get_dma_region(la9310_dev, LA9310_VSPA_OVERLAY);

    axi_data_width = vspadev->hardware.axi_data_width;
    axi_align = axi_data_width / BITS_PER_BYTE;

    buf_size = LA9310_VSPA_FW_SIZE;

    ret = la9310_load_firmware_to_dma(la9310_dev, vspa_fw_region, fw_data, vspa_fw_size);
    if (ret < 0)
    {
        dev_err(la9310_dev->dev, "%s: load_firmware request failed\n", __func__);
        goto OUT;
    }

    dev_dbg(
        la9310_dev->dev, "Udev FW [%s]: Address:%p\tVSPA FW Size:%d\n", vspadev->eld_filename, vspa_fw_region->vaddr, vspa_fw_size);

    dev_dbg(la9310_dev->dev, "Target DDR Virtual address = %p and size = %d\n", vspa_fw_region->vaddr, vspa_fw_size);

    vspa_fw_region->vaddr = PTR_ALIGN(vspa_fw_region->vaddr, axi_data_width);

    ret = la9310_load_vspa_image(la9310_dev, vspa_fw_region->vaddr, vspa_fw_size);
    if (ret < 0)
    {
        dev_err(la9310_dev->dev, "ERR %s: Image loading failed\n", __func__);
        goto OUT;
    }
    dev_dbg(la9310_dev->dev, "DBG %s: \tOUT\n", __func__);
    return 0;

OUT:
    dev_dbg(la9310_dev->dev, "Error in Loading VSPA FW image\n");
    return -EFAULT;
}

int vspa_load_dsp(struct la9310_dev* la9310_dev, struct vspa_device* vspadev, const char __user* fw_data, size_t fw_length)
{
    int err;

    if (!la9310_is_m4_booted(la9310_dev))
    {
        dev_err(la9310_dev->dev, "Firmware for M4 is not loaded yet but required for VSPA DSP loading!\n");
        return -EPERM;
    }

    /* Check if VCPU is busy */
    if (vspa_reg_read(vspadev->regs + STATUS_REG_OFFSET) & STATUS_REG_BUSY)
    {
        dev_err(la9310_dev->dev, "Firmware for VSPA DSP is already loaded and VCPU is running!\n");
        return -EBUSY;
    }

    dev_info(la9310_dev->dev, "INFO:%s : VSPA Loading firmware initiated-\n", __func__);

    vspadev->state = VSPA_STATE_LOADING;

    err = vspa_mem_initialization(vspadev);
    if (err < 0)
    {
        dev_err(vspadev->dev, "Memory Zeroise failed\n");
        return err;
    }
    dev_info(la9310_dev->dev, "mem init done\n");

    /* Call the LA9310 base APIs to request_firmware */
    if (vspa_get_fw_image(la9310_dev, fw_data, fw_length))
    {
        dev_err(la9310_dev->dev, "ERR %s : Loading VSPA FW failed\n", __func__);
        err = -EBADRQC;
        return err;
    }

    dev_info(la9310_dev->dev, "INFO:%s :VSPA FW image %s loading finished\n", __func__, vspadev->eld_filename);

    /* Initiate the VSPA_GO to start VSPA booting */
    if (startup(la9310_dev))
    {
        dev_err(la9310_dev->dev, "ERR %s: VSPA failed to start VSPA\n", __func__);
        err = -EBADRQC;
        return err;
    }

    err = la9310_vspa_stats_init(la9310_dev);
    if (err < 0)
    {
        dev_err(la9310_dev->dev, "ERR: VSPA stats error\n");
        return err;
    }

    dev_dbg(la9310_dev->dev, "DBG: Fw image name saved: %s", vspadev->eld_filename);

    return 0;
}

/************************* Probe / Remove ***********************************/

int vspa_probe(struct la9310_dev* la9310_dev)
{
    pr_info("VSPA probe\n");
    struct vspa_device* vspadev = NULL;
    struct device* dev = la9310_dev->dev;
    struct vspa_hardware* hw;
    struct la9310_mem_region_info* vspa_dma_region;
    int err = 0;
    size_t dma_size;
    u32 __iomem* dma_vaddr;
    uint32_t param0, param1, param2;
    uint32_t val;

    /* Allocating space vspa device structure */
    vspadev = kzalloc(sizeof(struct vspa_device), GFP_KERNEL);
    if (!vspadev)
    {
        err = -ENOMEM;
        goto err_out;
    }

    /* Getting the VSPA CCSR addresses from PCI */
    vspadev->regs = (u32 __iomem*)(la9310_dev->mem_regions[LA9310_MEM_REGION_CCSR].vaddr + VSPA_CCSR_OFFSET);

    if (!vspadev->regs)
    {
        dev_err(dev, "ERR: VSPA: Illegal address mapping\n");
        err = -EINVAL;
        goto err_out;
    }

    vspadev->dev = la9310_dev->dev;
    la9310_dev->vspa_priv = vspadev;
    vspadev->pdev = la9310_dev->pdev;

    /*Setup DMA regions - From PCI outbound window */
    vspa_dma_region = la9310_get_dma_region(la9310_dev, LA9310_MEM_REGION_FW);
    if (!vspa_dma_region)
    {
        dev_err(dev, "ERR %s: Address of dma region not found\n", __func__);
        err = -EADDRNOTAVAIL;
        goto err_out;
    }

    dma_vaddr = (u32*)vspa_dma_region->vaddr;
    dma_size = vspa_dma_region->size;

    if (!(dma_vaddr && dma_size))
    {
        err = -EADDRNOTAVAIL;
        goto err_out;
    }

    /* Struct vspa_device fields initialization */
    vspadev->vspa_dma_region = vspa_dma_region;

    vspadev->state = VSPA_STATE_UNKNOWN;
    vspadev->debug = DEBUG_MESSAGES;
    vspadev->mem_size = LA9310_VSPA_SIZE;

    /* Vspa hardware initialization */
    hw = &vspadev->hardware;
    param0 = vspa_reg_read(vspadev->regs + PARAM0_REG_OFFSET);

    hw->param0 = param0;
    param1 = vspa_reg_read(vspadev->regs + PARAM1_REG_OFFSET);
    hw->param1 = param1;
    hw->axi_data_width = 32 << ((param1 >> 28) & 7);
    hw->dma_channels = (param1 >> 16) & 0xFF;
    hw->gp_out_regs = (param1 >> 8) & 0xFF;
    hw->gp_in_regs = param1 & 0xFF;
    param2 = vspa_reg_read(vspadev->regs + PARAM2_REG_OFFSET);
    hw->param2 = param2;
    hw->dmem_bytes = ((param2 >> 8) & 0x3FF) * 400;
    hw->ippu_bytes = (param2 >> 31) * 4096;
    hw->arithmetic_units = param2 & 0xFF;

    vspadev->versions.vspa_hw_version = vspa_reg_read(vspadev->regs + HWVERSION_REG_OFFSET);
    vspadev->versions.ippu_hw_version = vspa_reg_read(vspadev->regs + IPPU_HWVERSION_REG_OFFSET);
    vspadev->versions.vspa_sw_version = ~0;
    vspadev->versions.ippu_sw_version = ~0;
    vspadev->eld_filename[0] = '\0';
    vspadev->watchdog_interval_msecs = VSPA_WATCHDOG_INTERVAL_DEFAULT;

    vspadev->poll_mask = VSPA_MSG_ALL;

    /* Enable core power gating */
    val = vspa_reg_read(vspadev->regs + CONTROL_REG_OFFSET);
    val = (val & CONTROL_REG_MASK) | CONTROL_PDN_EN;
    vspa_reg_write(vspadev->regs + CONTROL_REG_OFFSET, val);

    /* Make sure all interrupts are disabled */
    vspa_reg_write(vspadev->regs + IRQEN_REG_OFFSET, 0);
    dev_info(la9310_dev->dev,
        "VSPA: hwver 0x%08x, %d AUs, dmem %d bytes\n",
        vspadev->regs[HWVERSION_REG_OFFSET],
        hw->arithmetic_units,
        hw->dmem_bytes);

    /*Clearing the VCPU_TO_HOST MBOXs */
    vspa_reg_write(vspadev->regs + HOST_FLAGS0_REG_OFFSET, 0xFFFFFFFFUL);
    vspa_reg_write(vspadev->regs + HOST_FLAGS1_REG_OFFSET, 0xFFFFFFFFUL);

    /*Clearing the mailbox status bits for AVI */
    vspa_reg_write(vspadev->regs + STATUS_REG_OFFSET, 0xF000);

    dma_wmb();
    la9310_set_host_ready(la9310_dev, LA9310_HIF_STATUS_VSPA_READY);

    return 0;

err_out:
    pr_info("VSPA probe fail\n");
    return err;
}

int vspa_remove(struct la9310_dev* la9310_dev)
{
    struct vspa_device* vspadev = la9310_dev->vspa_priv;

    if (vspadev == NULL)
        goto out_remove;

    /* shutdown timer cleanly */
    vspadev->watchdog_interval_msecs = 0;

    kfree(vspadev);
out_remove:
    return 0;
}
