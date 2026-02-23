#define DEBUG

#include <linux/version.h>
#include <linux/file.h>

#include "la9310_limesdr_device.h"
#include "la9310_base.h"

#include "common_headers/la9310_host_if.h"
#include "common_headers/la9310_pci_def.h"

#define NXP_ERRATUM_A008822 1

int la9310_do_reset_handshake(struct la9310_dev* la9310_dev)
{
    int rc = 0, retries = LA9310_HOST_BOOT_HSHAKE_RETRIES;

    struct la9310_ccsr_dcr* ccsr_dcr;
    u32 scratch_val;
    u32* scratch_reg;
    u32 hif_offset, hif_size;
    volatile u32 *hif_offset_reg, *hif_size_reg;

    ccsr_dcr = (struct la9310_ccsr_dcr*)(la9310_dev->mem_regions[LA9310_MEM_REGION_CCSR].vaddr + DCR_OFFSET);
    scratch_reg = &ccsr_dcr->scratchrw[LA9310_BOOT_HSHAKE_SCRATCH_REG];
    hif_size_reg = &ccsr_dcr->scratchrw[LA9310_BOOT_HSHAKE_HIF_SIZ_REG];
    hif_offset_reg = &ccsr_dcr->scratchrw[LA9310_BOOT_HSHAKE_HIF_REG];
    dma_rmb();

    scratch_val = readl(scratch_reg);
    writel(LA9310_HOST_COMPLETE_CLOCK_CONFIG, scratch_reg);
    dma_wmb();

    dev_info(la9310_dev->dev,
        "[Reset HS] Waiting for FreeRTOS to write %d, current value %d\n",
        LA9310_HOST_START_DRIVER_INIT,
        scratch_val);

    set_current_state(TASK_INTERRUPTIBLE);
    /* Wait for FreeRTOS to complete reset hand shake */
    schedule_timeout(msecs_to_jiffies(LA9310_HOST_BOOT_HSHAKE_TIMEOUT));
    dma_rmb();
    scratch_val = readl(scratch_reg);
    hif_offset = readl(hif_offset_reg);
    hif_size = readl(hif_size_reg);
    while ((scratch_val != LA9310_HOST_START_DRIVER_INIT) && hif_size && retries)
    {
        set_current_state(TASK_INTERRUPTIBLE);
        schedule_timeout(msecs_to_jiffies(LA9310_HOST_BOOT_HSHAKE_TIMEOUT));
        retries--;
        dma_rmb();
        scratch_val = readl(scratch_reg);
        hif_offset = readl(hif_offset_reg);
        hif_size = readl(hif_size_reg);
    }
    if (scratch_val != LA9310_HOST_START_DRIVER_INIT)
    {
        dev_err(la9310_dev->dev, "LA9310 Reset HandShake failed, scratch 0x%x\n", scratch_val);
        rc = -EINVAL;
    }
    else
    {
        dev_info(la9310_dev->dev, "LA9310 Reset HandShake done, scratch 0x%x", scratch_val);
    }

    if (hif_size)
    {
        dev_info(la9310_dev->dev, "Reset HandShake: Done. HIF 0x%x, size 0x%x (%d)\n", hif_offset, hif_size, hif_size);
        la9310_dev->hif_offset = hif_offset;
        la9310_dev->hif_size = hif_size;
    }
    else
    {
        dev_err(la9310_dev->dev, "Reset HandShake: Failed. HIF 0x%x, size 0x%x (%d)\n", hif_offset, hif_size, hif_size);
        rc = -EBUSY;
    }

    return rc;
}

static struct la9310_boot_header prepare_bootheader(
    const struct la9310_dev* la9310_dev, const char __user* fw_data, size_t fw_length)
{
    struct la9310_boot_header header;

    // set default boot values
    header.preamble = PREAMBLE;
    header.plugin_size = 0;
    header.plugin_offset = 0;
    header.bl_size = fw_length;
    header.bl_src_offset = 0;
    header.bl_dest = LA9310_EP_FREERTOS_LOAD_ADDR;
    header.bl_entry = LA9310_EP_FREERTOS_LOAD_ADDR;
    header.reserved = LA9310_BOOT_HDR_BYPASS_BOOT_PLUGIN;

    // check if firmware file contains it's own boot header
    uint32_t fw_preamble;
    if (copy_from_user(&fw_preamble, fw_data, sizeof(fw_preamble)) != 0)
        return header;

    if (fw_preamble != PREAMBLE)
    {
        dev_dbg(la9310_dev->dev, "Using default bootheader:\n");
    }
    else
    {
        dev_dbg(la9310_dev->dev, "Firmware contains bootheader:\n");
        if (copy_from_user(&header, fw_data, sizeof(header)))
            dev_err(la9310_dev->dev, "Failed to copy firmware header from userspace:\n");
    }

#if !defined(BOOTROM_USE_EDMA)
    // Write reserved to tell bootrom to pull Image using memcpy instead of edma
    header.reserved |= LA9310_BOOT_HDR_BYPASS_BOOT_EDMA;
#endif
    return header;
}

static void la9310_initiate_boot(const struct la9310_dev* la9310_dev, const struct la9310_boot_header* header)
{
    const struct la9310_mem_region_info* ccsr_region = &la9310_dev->mem_regions[LA9310_MEM_REGION_CCSR];
#if NXP_ERRATUM_A008822
    // Before writing boot header, write 0000_9401h at ccsr adrress 0x34008d0h
    const uint32_t pcie_amba_err_offset = PCIE_RHOM_DBI_BASE + AMBA_ERROR_RESPONSE_DEFAULT_OFF;
    writel(AMBA_ERROR_RESPONSE_DEFAULT_VALUE, ccsr_region->vaddr + pcie_amba_err_offset);
#endif
    struct la9310_boot_header __iomem* dest_header =
        (struct la9310_boot_header*)((uint8_t*)la9310_dev->mem_regions[LA9310_MEM_REGION_TCMU].vaddr + LA9310_EP_BOOT_HDR_OFFSET);

    writel(header->plugin_size, &dest_header->plugin_size);
    writel(header->plugin_offset, &dest_header->plugin_offset);
    writel(header->bl_src_offset, &dest_header->bl_src_offset);
    writel(header->bl_size, &dest_header->bl_size);
    writel(header->bl_dest, &dest_header->bl_dest);
    writel(header->bl_entry, &dest_header->bl_entry);
    writel(header->reserved, &dest_header->reserved);

    // ROM code polls the contents of offset 0x0 from the TCM Data Memory base address, 0x2000_0000 for a valid Boot Header Preamble.
    dma_wmb();
    dev_dbg(la9310_dev->dev, "Initiate boot, writing preamble: 0x%08X\n", header->preamble);
    writel(header->preamble, &dest_header->preamble); // initiates boot
    dma_wmb();
}

int la9310_load_firmware_to_dma(
    struct la9310_dev* la9310_dev, const struct la9310_mem_region_info* firmware_dma, const char __user* fw_data, size_t fw_size)
{
    // const struct la9310_mem_region_info* firmware_dma = la9310_get_dma_region(la9310_dev, LA9310_MEM_REGION_FW);
    if (fw_size > firmware_dma->size)
    {
        dev_err(la9310_dev->dev, "Firmware too big (%ld) available buffer size (%ld)\n", fw_size, firmware_dma->size);
        return -ENOBUFS;
    }

    // Syncing the whole dma mapped buffer, even though only part of it is used for firmware loading.
    // IMX8MP page faults if sync is attempted for subsection of the mapped buffer.
    dma_sync_single_for_cpu(
        la9310_dev->dev, la9310_dev->dma_info.host_buf.phys_addr, la9310_dev->dma_info.host_buf.size, DMA_BIDIRECTIONAL);

    // Copy firmware from userspace to DMA region
    dev_dbg(la9310_dev->dev, "Copying firmware to 0x%ldx, size %ld\n", (size_t)firmware_dma->phys_addr, fw_size);
    int rc = copy_from_user(firmware_dma->vaddr, fw_data, fw_size);
    if (rc)
    {
        dev_err(la9310_dev->dev, "Could only copy %ld of %ld bytes of firmware.\n", fw_size - rc, fw_size);
        return -EIO;
    }
    // la9310_dev->firmware_info.size = fw_size;
    dma_sync_single_for_device(
        la9310_dev->dev, la9310_dev->dma_info.host_buf.phys_addr, la9310_dev->dma_info.host_buf.size, DMA_BIDIRECTIONAL);
    return 0;
}

int la9310_load_rtos_img(struct la9310_dev* la9310_dev, const char __user* fw_data, size_t fw_length)
{
    int rc = 0;
    struct la9310_ccsr_dcr* ccsr_dcr =
        (struct la9310_ccsr_dcr*)(la9310_dev->mem_regions[LA9310_MEM_REGION_CCSR].vaddr + DCR_OFFSET);

    struct la9310_boot_header fw_bootheader = prepare_bootheader(la9310_dev, fw_data, fw_length);

    dev_dbg(la9310_dev->dev, "preamble: \t0x%08X\n", fw_bootheader.preamble);
    dev_dbg(la9310_dev->dev, "plugin_size: \t0x%08X (%u bytes)\n", fw_bootheader.plugin_size, fw_bootheader.plugin_size);
    dev_dbg(la9310_dev->dev, "plugin_offset: \t0x%08X\n", fw_bootheader.plugin_offset);
    dev_dbg(la9310_dev->dev, "bl_size: \t0x%08X (%u bytes)\n", fw_bootheader.bl_size, fw_bootheader.bl_size);
    dev_dbg(la9310_dev->dev, "bl_src_offset: \t0x%08X\n", fw_bootheader.bl_src_offset);
    dev_dbg(la9310_dev->dev, "bl_dest: \t0x%08X\n", fw_bootheader.bl_dest);
    dev_dbg(la9310_dev->dev, "bl_entry: \t0x%08X\n", fw_bootheader.bl_entry);
    dev_dbg(la9310_dev->dev, "flags: \t0x%08X\n", fw_bootheader.reserved);

    // adjust bl_src_offset into PCIe memory window
    const struct la9310_mem_region_info* dma_region = la9310_get_dma_region(la9310_dev, LA9310_MEM_REGION_FW);
    fw_bootheader.bl_src_offset += LA9310_EP_DMA_PHYS_OFFSET(dma_region->phys_addr);
    dev_dbg(la9310_dev->dev, "PCIe window adjusted bl_src_offset: 0x%8X\n", fw_bootheader.bl_src_offset);

    // load firmware data into PCIe window
    rc = la9310_load_firmware_to_dma(la9310_dev, la9310_get_dma_region(la9310_dev, LA9310_MEM_REGION_FW), fw_data, fw_length);
    if (rc)
    {
        dev_err(la9310_dev->dev, "la9310_load_firmware_data failed\n");
        return rc;
    }

    la9310_initiate_boot(la9310_dev, &fw_bootheader);

    dev_info(la9310_dev->dev, "Waiting for FreeRTOS boot.\n");
    set_current_state(TASK_INTERRUPTIBLE);
    schedule_timeout(msecs_to_jiffies(LA9310_HOST_BOOT_HSHAKE_TIMEOUT));

    dma_rmb();
    uint32_t* scratch_reg = &ccsr_dcr->scratchrw[LA9310_BOOT_HSHAKE_SCRATCH_REG];
    uint32_t scratch_val = readl(scratch_reg);

    int retries = LA9310_HOST_BOOT_HSHAKE_RETRIES;
#if LA9310_UPGRADE_TIMESYNC_FW
    char std_fw_list[STD_MAX_FW_COUNT][STD_FW_NAME_MAX_LENGTH] = { 0 };
    dev_info(la9310_dev->dev, "[Sync Fw upgrade] Waiting for FreeRTOS to write %d\n", LA9310_HOST_TIMESYNC_FW_LOAD);
    while ((scratch_val != LA9310_HOST_TIMESYNC_FW_LOAD) && retries)
    {
        set_current_state(TASK_INTERRUPTIBLE);
        schedule_timeout(msecs_to_jiffies(LA9310_HOST_BOOT_HSHAKE_TIMEOUT));
        retries--;
        scratch_val = readl(scratch_reg);
        dma_rmb();
    }

    if (scratch_val != LA9310_HOST_TIMESYNC_FW_LOAD)
    {
        dev_err(la9310_dev->dev, "LA9310 sync no load scratch 0x%x\n", scratch_val);
        rc = -EINVAL;
        goto out;
    }
    else
    {
        dev_info(la9310_dev->dev, "LA9310 sync fw load received, scratch 0x%x", scratch_val);
    }

    strncpy(std_fw_list[0], STD_PROD_FW_NAME, STD_FW_NAME_MAX_LENGTH);
    strncpy(std_fw_list[1], STD_USER_CONFIG_NAME, STD_FW_NAME_MAX_LENGTH);

    rc = sync_timing_device_load_fw(la9310_dev, std_fw_list, 2);
    if (rc)
        goto out;
    writel(LA9310_HOST_TIMESYNC_FW_LOADED, scratch_reg);
    dma_rmb();
#endif
    dev_info(la9310_dev->dev, "[Sync fw upgrade] Waiting for FreeRTOS to write %d\n", LA9310_HOST_START_CLOCK_CONFIG);
    /* Wait for FreeRTOS to ask for clock configuration */
    set_current_state(TASK_INTERRUPTIBLE);
    schedule_timeout(msecs_to_jiffies(LA9310_HOST_BOOT_HSHAKE_TIMEOUT));

    retries = LA9310_HOST_BOOT_HSHAKE_RETRIES;
    while ((scratch_val != LA9310_HOST_START_CLOCK_CONFIG) && retries)
    {
        set_current_state(TASK_INTERRUPTIBLE);
        schedule_timeout(msecs_to_jiffies(LA9310_HOST_BOOT_HSHAKE_TIMEOUT));
        retries--;
        scratch_val = readl(scratch_reg);
        dma_rmb();
    }

    if (scratch_val != LA9310_HOST_START_CLOCK_CONFIG)
    {
        dev_err(la9310_dev->dev, "LA9310 FreeRTOS boot failed: 0x%x\n", scratch_val);
        rc = -EINVAL;
        goto out;
    }
    else
    {
        dev_info(la9310_dev->dev, "LA9310 FreeRTOS booted succesfully: 0x%x", scratch_val);
    }

out:
    return rc;
}
