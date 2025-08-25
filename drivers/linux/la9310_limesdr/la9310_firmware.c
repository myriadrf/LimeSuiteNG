#define DEBUG

#include <linux/version.h>
#include <linux/file.h>

#include "la9310_limesdr_device.h"
#include "la9310_memory.h"
#include "la9310_base.h"

#include "common_headers/la9310_host_if.h"
#include "common_headers/la9310_pci_def.h"

#define NXP_ERRATUM_A008822 1

static int check_file(const char *filename)
{
   struct file *file;
   char path[FIRMWARE_NAME_SIZE];

   if (!filename || !*filename)
      return -EINVAL;

   sprintf(path, "/lib/firmware/%s", filename);

   file = filp_open(path, O_RDONLY, 0);
   if (IS_ERR(file))
      return PTR_ERR(file);

   fput(file);
   return 0;
}

int la9310_do_reset_handshake(struct la9310_dev *la9310_dev)
{
#ifndef LA9310_RESET_HANDSHAKE_POLLING_ENABLE
   int rc = 0;
#else
   int rc = 0, retries = LA9310_HOST_BOOT_HSHAKE_RETRIES;

#endif
   struct la9310_ccsr_dcr *ccsr_dcr;
   u32 scratch_val;
   u32 *scratch_reg;
   u32 hif_offset, hif_size;
   volatile u32 *hif_offset_reg, *hif_size_reg;

   ccsr_dcr = (struct la9310_ccsr_dcr *)
         (la9310_dev->mem_regions[LA9310_MEM_REGION_CCSR].vaddr
         + DCR_OFFSET);
   scratch_reg = &ccsr_dcr->scratchrw[LA9310_BOOT_HSHAKE_SCRATCH_REG];
   hif_size_reg = &ccsr_dcr->scratchrw[LA9310_BOOT_HSHAKE_HIF_SIZ_REG];
   hif_offset_reg = &ccsr_dcr->scratchrw[LA9310_BOOT_HSHAKE_HIF_REG];
   dma_rmb();

   scratch_val = readl(scratch_reg);
   writel(LA9310_HOST_COMPLETE_CLOCK_CONFIG, scratch_reg);
   dma_wmb();

   dev_info(la9310_dev->dev,
       "[Reset HS] Waiting for FreeRTOS to write %d, current value %d\n",
        LA9310_HOST_START_DRIVER_INIT, scratch_val);

#ifdef LA9310_RESET_HANDSHAKE_POLLING_ENABLE
   set_current_state(TASK_INTERRUPTIBLE);
   /* Wait for FreeRTOS to complete reset hand shake */
   schedule_timeout(msecs_to_jiffies(LA9310_HOST_BOOT_HSHAKE_TIMEOUT));
   dma_rmb();
   scratch_val = readl(scratch_reg);
   hif_offset = readl(hif_offset_reg);
   hif_size = readl(hif_size_reg);
   while ((scratch_val != LA9310_HOST_START_DRIVER_INIT) && hif_size && retries) {
      set_current_state(TASK_INTERRUPTIBLE);
      schedule_timeout(msecs_to_jiffies(
         LA9310_HOST_BOOT_HSHAKE_TIMEOUT));
      retries--;
      dma_rmb();
      scratch_val = readl(scratch_reg);
      hif_offset = readl(hif_offset_reg);
      hif_size = readl(hif_size_reg);
   }
#else
   /*waiting for interrupt from la9310 to do handshake*/
   wait_for_completion_timeout(&ScratchRegisterHandshake,
                msecs_to_jiffies
                (LA9310_HOST_BOOT_HSHAKE_TIMEOUT));

   dma_rmb();
   scratch_val = readl(scratch_reg);
   hif_offset = readl(hif_offset_reg);
   hif_size = readl(hif_size_reg);
#endif
   if (scratch_val != LA9310_HOST_START_DRIVER_INIT) {
      dev_err(la9310_dev->dev,
         "LA9310 Reset HandShake failed, scratch 0x%x\n",
         scratch_val);
      rc = -EINVAL;
   } else {
      dev_info(la9310_dev->dev,
          "LA9310 Reset HandShake done, scratch 0x%x",
          scratch_val);
   }

   if (hif_size) {
      dev_info(la9310_dev->dev, "Reset HandShake: Done. HIF 0x%x, size 0x%x (%d)\n",
            hif_offset, hif_size, hif_size);
      la9310_dev->hif_offset = hif_offset;
      la9310_dev->hif_size = hif_size;
   } else {
      dev_err(la9310_dev->dev, "Reset HandShake: Failed. HIF 0x%x, size 0x%x (%d)\n",
            hif_offset, hif_size, hif_size);
      rc = -EBUSY;
   }

   return rc;
}

int la9310_load_rtos_img(struct la9310_dev *la9310_dev)
{
   int rc = 0, retries = LA9310_HOST_BOOT_HSHAKE_RETRIES;
   struct la9310_mem_region_info *tcm_region;
   struct la9310_mem_region_info *dma_region;
   struct la9310_mem_region_info *ccsr_region;
   struct la9310_ccsr_dcr *ccsr_dcr;
   char freertos_img[FIRMWARE_NAME_SIZE];
   u32 scratch_val;
   u32 *scratch_reg;
   u8 * __iomem ccs_dcr_ptr;
   struct la9310_boot_header __iomem *boot_header;
   int size, fw_size;
   u32 ep_dma_offset, rsvd_ctrl = 0;
#if LA9310_UPGRADE_TIMESYNC_FW
   char std_fw_list[STD_MAX_FW_COUNT][STD_FW_NAME_MAX_LENGTH] = {0};
#endif

#if NXP_ERRATUM_A008822
   u32 pcie_amba_err_offset;
#endif
   ccsr_region = &la9310_dev->mem_regions[LA9310_MEM_REGION_CCSR];
   ccs_dcr_ptr = ccsr_region->vaddr + DCR_OFFSET;
   ccsr_dcr = (struct la9310_ccsr_dcr *) ccs_dcr_ptr;

   tcm_region = &la9310_dev->mem_regions[LA9310_MEM_REGION_TCMU];
#if NXP_ERRATUM_A008822
       /*Before writing boot header,write 0000_9401h at ccsr adrress
      0x34008d0h*/
   pcie_amba_err_offset = PCIE_RHOM_DBI_BASE +
            AMBA_ERROR_RESPONSE_DEFAULT_OFF;
   writel(AMBA_ERROR_RESPONSE_DEFAULT_VALUE, ccsr_region->vaddr
                  +pcie_amba_err_offset);
#endif
   /* ROM code polls the contents of offset 0x0 from the TCM Data Memory
    * base address, 0x2000_0000 for a valid Boot Header Preamble.
    */
   boot_header = (struct la9310_boot_header *)((u8 *)tcm_region->vaddr +
      LA9310_EP_BOOT_HDR_OFFSET);
   la9310_dev->boot_header = boot_header;

   dma_region = la9310_get_dma_region(la9310_dev, LA9310_MEM_REGION_FW);
   size = dma_region->size;
   rc = la9310_dev_reserve_firmware(la9310_dev);
   if (rc)
      goto out;
   sprintf(freertos_img, "%s", la9310_dev->firmware_name);
   rc = check_file(freertos_img);
   if (rc) {
      dev_err(la9310_dev->dev,
         "FreeRTOS Image file(%s) not present",
         freertos_img);
      goto out;
   }

   rc  = la9310_udev_load_firmware(la9310_dev, dma_region->vaddr,
                 size, freertos_img);
   la9310_dev_free_firmware(la9310_dev);
   if (rc) {
      dev_err(la9310_dev->dev, "load_firmware [%s] request failed\n",
          freertos_img);
      goto out;
   }

   fw_size =  la9310_dev->firmware_info.size;
// #if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 5, 0)
//    dma_map_page_attrs(&la9310_dev->pdev->dev,
//          virt_to_page(dma_region->vaddr),
//          offset_in_page(dma_region->vaddr), fw_size,
//          (enum dma_data_direction)DMA_TO_DEVICE, 0);
// #else
//    pci_map_single(la9310_dev->pdev, dma_region->vaddr,
//          fw_size, PCI_DMA_TODEVICE);
// #endif

   dev_info(la9310_dev->dev, "load_firmware[%s] - Addr %px, size %d\n",
      freertos_img, dma_region->vaddr, fw_size);

   dev_dbg(la9310_dev->dev, "BootHDR: bl_src_offset [0x%p]: 0x%llx\n",
      &boot_header->bl_src_offset, dma_region->phys_addr);

   dev_dbg(la9310_dev->dev, "BootHDR: bl_size [0x%p]: %d\n",
      &boot_header->bl_size, fw_size);

   dev_dbg(la9310_dev->dev, "BootHDR: preamble [0x%p]: 0x%x\n",
      &boot_header->preamble, PREAMBLE);

   dma_wmb();

   /*Update Firmware pointers + size in boot header */
   ep_dma_offset = LA9310_EP_DMA_PHYS_OFFSET(dma_region->phys_addr);
   writel(ep_dma_offset, &boot_header->bl_src_offset);
   writel(fw_size, &boot_header->bl_size);
   writel(LA9310_EP_FREERTOS_LOAD_ADDR, &boot_header->bl_dest);
   writel(LA9310_EP_FREERTOS_LOAD_ADDR, &boot_header->bl_entry);

   /* Write reserved to tell bootrom to pull Image using memcpy instead of
    * edma
    */
   rsvd_ctrl = LA9310_BOOT_HDR_BYPASS_BOOT_PLUGIN;
#if !defined(BOOTROM_USE_EDMA)
   rsvd_ctrl |= LA9310_BOOT_HDR_BYPASS_BOOT_EDMA;
#endif
   writel(rsvd_ctrl, &boot_header->reserved);
   dma_wmb();
   writel(PREAMBLE, &boot_header->preamble);
   dev_info(la9310_dev->dev, "Waiting for FreeRTOS boot.\n");

   dev_info(la9310_dev->dev, "Waiting.\n");
   set_current_state(TASK_INTERRUPTIBLE);
   //schedule_timeout(msecs_to_jiffies(LA9310_HOST_BOOT_HSHAKE_TIMEOUT));
   schedule_timeout(msecs_to_jiffies(3000));
   dev_info(la9310_dev->dev, "Waiting done.\n");

   dma_rmb();
   scratch_reg = &ccsr_dcr->scratchrw[LA9310_BOOT_HSHAKE_SCRATCH_REG];
   scratch_val = readl(scratch_reg);
#if LA9310_UPGRADE_TIMESYNC_FW
   dev_info(la9310_dev->dev,
      "[Sync Fw upgrade] Waiting for FreeRTOS to write %d\n",
       LA9310_HOST_TIMESYNC_FW_LOAD);
   while ((scratch_val != LA9310_HOST_TIMESYNC_FW_LOAD) && retries) {
      set_current_state(TASK_INTERRUPTIBLE);
      schedule_timeout(msecs_to_jiffies(
               LA9310_HOST_BOOT_HSHAKE_TIMEOUT));
      retries--;
      scratch_val = readl(scratch_reg);
      dma_rmb();

   }

   if (scratch_val != LA9310_HOST_TIMESYNC_FW_LOAD) {
      dev_err(la9310_dev->dev, "LA9310 sync no load scratch 0x%x\n",
         scratch_val);
      rc = -EINVAL;
      goto out;
   } else {
      dev_info(la9310_dev->dev,
          "LA9310 sync fw load received, scratch 0x%x",
          scratch_val);
   }

   strncpy(std_fw_list[0], STD_PROD_FW_NAME, STD_FW_NAME_MAX_LENGTH);
   strncpy(std_fw_list[1], STD_USER_CONFIG_NAME, STD_FW_NAME_MAX_LENGTH);

   rc = sync_timing_device_load_fw(la9310_dev, std_fw_list, 2);
   if (rc)
      goto out;
   writel(LA9310_HOST_TIMESYNC_FW_LOADED, scratch_reg);
   dma_rmb();
#endif
   dev_info(la9310_dev->dev,
      "[Sync fw upgrade] Waiting for FreeRTOS to write %d\n",
      LA9310_HOST_START_CLOCK_CONFIG);
   /* Wait for FreeRTOS to ask for clock configuration */
   set_current_state(TASK_INTERRUPTIBLE);
   schedule_timeout(msecs_to_jiffies(LA9310_HOST_BOOT_HSHAKE_TIMEOUT));

   retries = LA9310_HOST_BOOT_HSHAKE_RETRIES;
   while ((scratch_val != LA9310_HOST_START_CLOCK_CONFIG) && retries) {
      set_current_state(TASK_INTERRUPTIBLE);
      schedule_timeout(msecs_to_jiffies(
               LA9310_HOST_BOOT_HSHAKE_TIMEOUT));
      retries--;
      scratch_val = readl(scratch_reg);
      dma_rmb();
   }

   if (scratch_val != LA9310_HOST_START_CLOCK_CONFIG) {
      dev_err(la9310_dev->dev, "LA9310 FreeRTOS boot failed: 0x%x\n",
         scratch_val);
      rc = -EINVAL;
      goto out;
   } else {
      dev_info(la9310_dev->dev,
          "LA9310 FreeRTOS booted succesfully: 0x%x",
          scratch_val);
   }

out:
   return rc;
}

// Copies firmware image into device and boots it.
int custom_la9310_load_rtos_img(struct la9310_dev *la9310, const uint8_t *rtos_img, uint32_t rtos_img_size)
{
    struct device* sysDev = &la9310->pdev->dev;
    int rc = 0;

    struct la9310_mem_region_info *ccsr_region = &la9310->mem_regions[LA9310_MEM_REGION_CCSR];
    struct la9310_mem_region_info *tcm_region = &la9310->mem_regions[LA9310_MEM_REGION_TCMU];
#if NXP_ERRATUM_A008822
    {
    // Before writing boot header,write 0000_9401h at ccsr adrress 0x34008d0h
    u32 pcie_amba_err_offset = PCIE_RHOM_DBI_BASE + AMBA_ERROR_RESPONSE_DEFAULT_OFF;
    writel(AMBA_ERROR_RESPONSE_DEFAULT_VALUE, ccsr_region->vaddr + pcie_amba_err_offset);
    }
#endif

   // ROM code polls the contents of offset 0x0 from the TCM Data Memory
   // base address, 0x2000_0000 for a valid Boot Header Preamble.
   struct la9310_boot_header __iomem *boot_header = (struct la9310_boot_header *)((uint8_t *)tcm_region->vaddr + LA9310_EP_BOOT_HDR_OFFSET);

   struct la9310_mem_region_info dma_region;// = la9310_get_dma_region(la9310_dev, LA9310_MEM_REGION_FW);
   dma_region.vaddr = kmalloc(rtos_img_size, GFP_KERNEL);
   if (!dma_region.vaddr)
   {
      dev_err(sysDev, "%s: Failed to allocate temporary buffer for firmware\n", __func__);
   }
   dma_region.size = rtos_img_size;
   dma_region.phys_addr = virt_to_phys(dma_region.vaddr);

   memcpy_toio(dma_region.vaddr, rtos_img, rtos_img_size);

   dma_addr_t dma_bus = dma_map_single(sysDev, dma_region.vaddr, dma_region.size, DMA_TO_DEVICE);
   if (dma_mapping_error(sysDev, dma_bus))
   {
      dev_err(sysDev,
         "%s: dma_map_single error @ va:%p pa:%llx bus:%llx\n",
         __func__,
         dma_region.vaddr,
         dma_region.phys_addr,
         dma_bus);
      kfree(dma_region.vaddr);
      return -3;
   }

   // dev_info(sysDev, "load_firmware - Addr %px, size %d, pa:%px bus:0x%lx\n", dma_region.vaddr, rtos_img_size, dma_region.phys_addr, dma_bus);
   dev_dbg(sysDev, "BootHDR: bl_src_offset [0x%p]: 0x%llx\n", &boot_header->bl_src_offset, dma_region.phys_addr);
   dev_dbg(sysDev, "BootHDR: bl_size [0x%p]: %d\n", &boot_header->bl_size, rtos_img_size);
   dev_dbg(sysDev, "BootHDR: preamble [0x%p]: 0x%x\n", &boot_header->preamble, PREAMBLE);

   // Update Firmware pointers + size in boot header
   const uint32_t ep_dma_offset = LA9310_EP_DMA_PHYS_OFFSET(dma_bus);
   pr_info("ep_dma_offset %X\n", ep_dma_offset);
   writel(dma_bus, &boot_header->bl_src_offset);
   writel(rtos_img_size, &boot_header->bl_size);
   writel(LA9310_EP_FREERTOS_LOAD_ADDR, &boot_header->bl_dest);
   writel(LA9310_EP_FREERTOS_LOAD_ADDR, &boot_header->bl_entry);

   // Write reserved to tell bootrom to pull Image using memcpy instead of edma
   uint32_t rsvd_ctrl = LA9310_BOOT_HDR_BYPASS_BOOT_PLUGIN;
#if !defined(BOOTROM_USE_EDMA)
   rsvd_ctrl |= LA9310_BOOT_HDR_BYPASS_BOOT_EDMA;
#endif
   writel(rsvd_ctrl, &boot_header->reserved);
   dma_wmb();

   writel(PREAMBLE, &boot_header->preamble);
   dev_info(sysDev, "Waiting for FreeRTOS boot.\n");

   set_current_state(TASK_INTERRUPTIBLE);
   schedule_timeout(msecs_to_jiffies(LA9310_HOST_BOOT_HSHAKE_TIMEOUT));

   dma_rmb();
   uint8_t * __iomem ccs_dcr_ptr = ccsr_region->vaddr + DCR_OFFSET;
   struct la9310_ccsr_dcr *ccsr_dcr = (struct la9310_ccsr_dcr *) ccs_dcr_ptr;
   uint32_t *scratch_reg = &ccsr_dcr->scratchrw[LA9310_BOOT_HSHAKE_SCRATCH_REG];

   dev_info(sysDev, "[Sync fw upgrade] Waiting for FreeRTOS to write %d\n", LA9310_HOST_START_CLOCK_CONFIG);

   // Wait for FreeRTOS to ask for clock configuration
   set_current_state(TASK_INTERRUPTIBLE);
   schedule_timeout(msecs_to_jiffies(LA9310_HOST_BOOT_HSHAKE_TIMEOUT));

   int retries = LA9310_HOST_BOOT_HSHAKE_RETRIES;
   uint32_t scratch_val = readl(scratch_reg);
   while ((scratch_val != LA9310_HOST_START_CLOCK_CONFIG) && retries)
   {
      dev_info(sysDev, "Wait start clock, retries: %i\n", retries);
      set_current_state(TASK_INTERRUPTIBLE);
      schedule_timeout(msecs_to_jiffies(LA9310_HOST_BOOT_HSHAKE_TIMEOUT));
      --retries;
      scratch_val = readl(scratch_reg);
      dma_rmb();
   }

   if (scratch_val != LA9310_HOST_START_CLOCK_CONFIG)
   {
      dev_err(sysDev, "LA9310 FreeRTOS boot failed: 0x%x\n", scratch_val);
      rc = -EINVAL;
      goto out;
   }

   dev_info(sysDev, "LA9310 FreeRTOS booted succesfully: 0x%x", scratch_val);

   dma_unmap_single(sysDev, dma_bus, dma_region.size, DMA_TO_DEVICE);
   kfree(dma_region.vaddr);

out:
   return rc;
}
