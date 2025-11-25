#define DEBUG

#include <linux/kernel.h>
#include <linux/module.h>
#include "la9310_pci.h"

#include <linux/pci.h>
#include <linux/pci_regs.h>

#include "la9310_limesdr_device.h"

#include "common_headers/la9310_pci_def.h"

#define LIMEMICROSYSTEMS_VENDOR_ID 0x2058
#define PCI_DEVICE_ID_LA9310           0x1c10
#define PCI_DEVICE_ID_LA9310_DISABLE_CIP 0x1c12

static const struct pci_device_id la9310_limesdr_pci_ids[] = {
    { PCI_DEVICE(PCI_VENDOR_ID_FREESCALE, PCI_DEVICE_ID_LA9310) },
    { PCI_DEVICE(PCI_VENDOR_ID_FREESCALE, PCI_DEVICE_ID_LA9310_DISABLE_CIP) },
    {0}
};

MODULE_DEVICE_TABLE(pci, la9310_limesdr_pci_ids);

// set dma address mask and check if buffer allocation and mapping is within the mask
static int try_set_dma_bitmask(struct device *sysDev, uint32_t bitCount)
{
    const uint64_t mask = DMA_BIT_MASK(bitCount);
    int ret = dma_set_mask_and_coherent(sysDev, mask);
    if (ret)
    {
        dev_warn(sysDev, "Failed dma_set_mask_and_coherent %ubit\n", bitCount);
        return -1;
    }
    return ret;
    void *memoryBuffer = kmalloc(PAGE_SIZE, GFP_KERNEL);
    if (!memoryBuffer)
    {
        dev_err(sysDev, "%s(%i): Failed to allocate dma buffer\n", __func__, bitCount);
        return -2;
    }

    dma_addr_t dma_bus = dma_map_single(sysDev, memoryBuffer, PAGE_SIZE, DMA_FROM_DEVICE);
    if (dma_mapping_error(sysDev, dma_bus))
    {
        dev_err(sysDev,
            "%s(%i): dma_map_single error @ va:%p pa:%llx bus:%llx\n",
            __func__,
            bitCount,
            memoryBuffer,
            virt_to_phys(memoryBuffer),
            dma_bus);
        kfree(memoryBuffer);
        return -3;
    }

    ret = 0;
    uint64_t physicallAddress = virt_to_phys(memoryBuffer);
    dev_info(sysDev, "%s(%i): test buffer va:%p pa:%llx bus:%llx\n", __func__, bitCount, memoryBuffer, physicallAddress, dma_bus);
    if (dma_bus & ~mask)
    {
        dev_err(sysDev, "DMA bus address is outside of requested mask.\n");
        ret = -4;
    }

    dma_unmap_single(sysDev, dma_bus, PAGE_SIZE, DMA_FROM_DEVICE);
    kfree(memoryBuffer);
    return ret;
}

static int set_dma_addressing_mask(struct device *sysDev, const uint32_t *bitCounts, int count)
{
    const uint64_t required_dma_mask = dma_get_required_mask(sysDev);
    dev_info(sysDev, "dma_get_required_mask: %llx.\n", required_dma_mask);
    for (int i = 0; i < count; ++i)
    {
        int ret = try_set_dma_bitmask(sysDev, bitCounts[i]);
        if (ret == 0)
        {
            dev_info(sysDev, "using %ibit DMA mask\n", bitCounts[i]);
            return 0;
        }
        else
            dev_err(sysDev, "Failed to set DMA mask %ibit.\n", bitCounts[i]);
    }
    return -1;
}

static int FixBARResources(struct pci_dev *pciContext)
{
    // LA9310 ROM bootloader PCI descriptor has a PCI class type of 00
    // Linux skips devices with undefined class when assigning resources
    // https://github.com/torvalds/linux/blob/113fb03ed1d4c1877cc8817e500867616b222380/drivers/pci/setup-bus.c#L184-L186
    // hence the device is enumerated, but BARs are left unassigned.

    int ret = 0;
    struct resource* bar0 = &pciContext->resource[0];
    if (bar0->start == 0) // BAR is unassigned
        bar0->end = 0x3ffffff; // Default BAR0 size is 256MB, some systems can't map that much, so reduce BAR0 size to 64MB,

    for (int i=0; i<3; ++i)
    {
        struct resource* res = &pciContext->resource[i];
        if (res->start != 0)
            continue;

        ret = pci_assign_resource(pciContext, i);
        if (ret)
            return ret;
    }
    return ret;
}

static int pcidev_tune_caps(struct pci_dev *pdev)
{
    struct pci_dev *parent;
    u16 pcaps, ecaps, ctl;
    int rc_sup, ep_sup;

    /* Find out supported and configured values for parent (root) */
    parent = pdev->bus->self;
    if (parent->bus->parent) {
        dev_info(&pdev->dev, "Parent not root\n");
        return -EINVAL;
    }

    if (!pci_is_pcie(parent) || !pci_is_pcie(pdev))
        return -EINVAL;

    pcie_capability_read_word(parent, PCI_EXP_DEVCAP, &pcaps);
    pcie_capability_read_word(pdev, PCI_EXP_DEVCAP, &ecaps);

    /* Find max payload supported by root, endpoint */
    rc_sup = pcaps & PCI_EXP_DEVCAP_PAYLOAD;
    ep_sup = ecaps & PCI_EXP_DEVCAP_PAYLOAD;
    dev_info(&pdev->dev, "max payload size    rc:%d ep:%d\n",
            128 * (1<<rc_sup), 128 * (1<<ep_sup));
    if (rc_sup > ep_sup)
        rc_sup = ep_sup;

    pcie_capability_clear_and_set_word(parent, PCI_EXP_DEVCTL,
                       PCI_EXP_DEVCTL_PAYLOAD, rc_sup << 5);

    pcie_capability_clear_and_set_word(pdev, PCI_EXP_DEVCTL,
                       PCI_EXP_DEVCTL_PAYLOAD, rc_sup << 5);

    pcie_capability_read_word(pdev, PCI_EXP_DEVCTL, &ctl);
    dev_dbg(&pdev->dev, "MAX payload size is %dB, MAX read size is %dB.\n",
        128 << ((ctl & PCI_EXP_DEVCTL_PAYLOAD) >> 5),
        128 << ((ctl & PCI_EXP_DEVCTL_READRQ) >> 12));

    return 0;
}

static int la9310_configure_pci(struct pci_dev *pciContext)
{
    WARN_ON(pciContext == NULL);
    struct device *sysDev = &pciContext->dev;
    int ret = 0;

    if ((ret = FixBARResources(pciContext)))
    {
        dev_err(sysDev, "Failed to assign BAR resources");
        return ret;
    }

    // enable device only after BARs are assigned
    if ((ret = pcim_enable_device(pciContext)))
    {
        dev_err(sysDev, "Cannot enable device (%i)\n", ret);
        return ret;
    }

    if ((ret = pci_request_regions(pciContext, LA9310_LIMESDR_DRIVER_NAME)))
    {
        dev_err(sysDev, "failed to request pci regions (%i)\n", ret);
        return ret;
    }

    const uint32_t bitCountOrder[] = {64, 32};
    if ((ret = set_dma_addressing_mask(sysDev, bitCountOrder, 2)))
        return ret;

    pcidev_tune_caps(pciContext);
    pci_set_master(pciContext);

    // for (int i = 0; i < 3; i++)
    // {
    //     // Read the hardware address
    //     resource_size_t phys_addr = pci_resource_start(pciContext, i);
    //     uint32_t size = pci_resource_len(pciContext, i);
    //     myDevice->bar_addr[i] = pci_iomap(pciContext, i, size);
    //     dev_dbg(sysDev, "BAR:%d va:0x%px pa:0x%p len:0x%x\n", i, myDevice->bar_addr[i], (void*)phys_addr, (uint32_t)size);
    //     myDevice->la9310->mem_regions[i].vaddr = myDevice->bar_addr[i];
    //     myDevice->la9310->mem_regions[i].phys_addr = virt_to_phys(myDevice->la9310->mem_regions[i].vaddr);
    //     myDevice->la9310->mem_regions[i].size = size;
    // }

// #ifdef LA9310_FLG_PCI_8MSI_EN
//     ret = la9310_dev_set_interrupt_capability(pciContext, PCI_INT_MODE_MULTIPLE_MSI);
// #else
//     ret = la9310_dev_set_interrupt_capability(pciContext, PCI_INT_MODE_MSI);
// #endif
//     if (ret < 0)
//         return ret;

    pcie_print_link_status(pciContext);
    return 0;
}

static void enable_all_msi(struct la9310_dev *la9310_dev)
{
    u32 __iomem *pcie_vaddr, *pcie_msi_control;
    u32 val;

    pcie_vaddr = (u32 *)(la9310_dev->mem_regions[LA9310_MEM_REGION_CCSR]
                .vaddr + PCIE_RHOM_DBI_BASE
                    + PCIE_MSI_BASE);

    val = ioread32(pcie_vaddr);
    dev_dbg(la9310_dev->dev, "MSI Capability: Control Reg. -> value = %x\n",
                            val);
    pcie_msi_control = (u32 *)(la9310_dev->mem_regions
                   [LA9310_MEM_REGION_CCSR].vaddr +
                    PCIE_RHOM_DBI_BASE + PCIE_MSI_CONTROL);

    iowrite8(0xb6, pcie_msi_control);

    val = ioread32(pcie_vaddr);
    dev_dbg(la9310_dev->dev, "MSI Capability: Control Reg. -> value = %x\n"
          , val);
}

/*
 * la9310_dev_set_interrupt_capability - set MSI or MSI-X if supported
 *
 * Attempt to configure interrupts using the best available
 * capabilities of the hardware and kernel.
 */
static int la9310_dev_set_interrupt_capability(struct la9310_dev *la9310_dev, int mode)
{
    int ret = 0, i = 0;
    /* Check whether the device has MSIx cap */
    switch (mode) {
    case PCI_INT_MODE_MULTIPLE_MSI:
        enable_all_msi(la9310_dev);
        ret = pci_alloc_irq_vectors_affinity(la9310_dev->pdev,
                   MIN_MSI_ITR_LINES,
                   LA9310_MSI_MAX_CNT,
                   PCI_IRQ_MSI, NULL);
        if (ret < LA9310_MSI_MAX_CNT) {
            dev_err(la9310_dev->dev, "Cannot complete request for multiple MSI. ret=%i", ret);
            goto msi_error;
        } else {
            dev_info(la9310_dev->dev,
                 "%d MSI successfully created\n", ret);
        }
        LA9310_SET_FLG(la9310_dev->flags, LA9310_FLG_PCI_MSI_EN);
        for (i = 0; i < LA9310_MSI_MAX_CNT; i++) {
            la9310_dev->irq[i].msi_val = i;
            la9310_dev->irq[i].irq_val =
                pci_irq_vector(la9310_dev->pdev, i);
            la9310_dev->irq[i].free = LA931XA_MSI_IRQ_FREE;
        }
        la9310_dev->irq_count = LA9310_MSI_MAX_CNT;
        break;

    case PCI_INT_MODE_MSIX:
        /* TBD:XXX: LA9310 will support 8 MSIs, thus MSIx. this code
         * need to be changed
         */
        dev_err(la9310_dev->dev,
            "Unable to support MSIX for LA9310\n");
        __attribute__((__fallthrough__));
        /* Fall through */
    case PCI_INT_MODE_MSI:
        if (!pci_enable_msi(la9310_dev->pdev)) {
            LA9310_SET_FLG(la9310_dev->flags, LA9310_FLG_PCI_MSI_EN);
            la9310_dev->irq[MSI_IRQ_MUX].irq_val =
                pci_irq_vector(la9310_dev->pdev, MSI_IRQ_MUX);
            la9310_dev->irq[MSI_IRQ_MUX].free = LA931XA_MSI_IRQ_FREE;
            la9310_dev->irq_count = 1;
        } else {
            dev_warn(la9310_dev->dev,
                 "Failed to init MSI, fall bk to legacy\n");
            goto msi_error;
        }
        __attribute__((__fallthrough__));
        /* Fall through */
    case PCI_INT_MODE_LEGACY:
        la9310_dev->irq[MSI_IRQ_MUX].irq_val = la9310_dev->pdev->irq;
        la9310_dev->irq[MSI_IRQ_MUX].free = LA931XA_MSI_IRQ_FREE;
        la9310_dev->irq_count = 1;
        break;

    case PCI_INT_MODE_NONE:
        break;
    }

    return 0;

msi_error:
    return -EINTR;
}

static struct la9310_dev *la9310_pci_priv_init(struct pci_dev *pdev)
{
    int rc = 0;

    struct la9310_dev *la9310_dev = devm_kzalloc(&pdev->dev, sizeof(struct la9310_dev), GFP_KERNEL);
    if (!la9310_dev)
        goto out;

    la9310_dev->dev = &pdev->dev;
    la9310_dev->pdev = pdev;

    //
    la9310_dev->sdr_board = 0;
    la9310_dev->dac_mask = 0x1;
    la9310_dev->adc_mask = 0xF;
    la9310_dev->adc_rate_mask = 0xF;
    la9310_dev->dac_rate_mask = 1;

    la9310_dev->modem_rf_data_size = 0;

    sprintf(la9310_dev->firmware_name, "la9310.bin");
    // char vspa_fw_name[];
    // sprintf(la9310_dev_name, "devname");

    // i = get_la9310_dev_id_pcidevname(la9310_dev->dev);

    // la9310_dev->id = i;

    // sprintf(la9310_dev_name, "%s%d", LA9310_DEV_NAME_PREFIX, la9310_dev->id);
    // sprintf(&la9310_dev->name[0], "%s", la9310_dev_name);
    sprintf(la9310_dev->name, "la9310_limesdr_name");

    // /* Not allowed to create it */
    // if (i == -1) {
    //     dev_err(&pdev->dev,
    //         "exceeding max permitted (%d) la9310 devices!\n",
    //         MAX_MODEM_INSTANCES);
    //     kfree(la9310_dev);
    //     return NULL;
    // }
    // la9310_dev->id = i;

    // g_la9310_global[la9310_dev->id].active = 1;
    // sprintf(g_la9310_global[la9310_dev->id].dev_name, "%s",
    //     dev_name(la9310_dev->dev));

    // dev_info(la9310_dev->dev, "Init - %s !\n", la9310_dev->name);

    /* Get the BAR resources and remap them into the driver memory */
    for (int i = 0; i < LA9310_MEM_REGION_BAR_END; i++) {
        /* Read the hardware address */
        la9310_dev->mem_regions[i].phys_addr = pci_resource_start(pdev, i);
        la9310_dev->mem_regions[i].size = pci_resource_len(pdev, i);
        dev_info(la9310_dev->dev, "BAR:%d  addr:0x%llx len:0x%llx\n", i, la9310_dev->mem_regions[i].phys_addr, (u64)la9310_dev->mem_regions[i].size);
    }

    rc = la9310_map_mem_regions(la9310_dev);
    if (rc) {
        dev_err(la9310_dev->dev, "Failed to map mem regions, err %d\n",
            rc);
        goto out;
    }

#ifdef LA9310_FLG_PCI_8MSI_EN
    rc = la9310_dev_set_interrupt_capability(la9310_dev,
                PCI_INT_MODE_MULTIPLE_MSI);
#else
    rc = la9310_dev_set_interrupt_capability(la9310_dev,
                PCI_INT_MODE_MSI);
#endif
    if (rc < 0) {
        dev_info(la9310_dev->dev, "Cannot set the capability of device\n");
        goto out;
    }

    // list_add_tail(&la9310_dev->list, &pcidev_list);

out:
    if (rc) {
        if (la9310_dev) {
            la9310_unmap_mem_regions(la9310_dev);
            kfree(la9310_dev);
        }
        la9310_dev = NULL;
    }

    return la9310_dev;
}

static void la9310_dev_reset_interrupt_capability(struct la9310_dev *la9310_dev)
{
    if (LA9310_CHK_FLG(la9310_dev->flags, LA9310_FLG_PCI_MSI_EN)) {
        pci_disable_msi(la9310_dev->pdev);
        LA9310_CLR_FLG(la9310_dev->flags, LA9310_FLG_PCI_MSI_EN);
    }
}

static int la9310_limesdr_pci_probe(struct pci_dev *pciContext, const struct pci_device_id *id)
{
	int ret;
    struct device *sysDev = &pciContext->dev;
    dev_info(sysDev, "[%s] vid:%04X pid:%04X\n", __func__, id->vendor, id->device);

    // struct la9310_limesdr_device *myDevice = devm_kzalloc(sysDev, sizeof(struct la9310_limesdr_device), GFP_KERNEL);
    // if (!myDevice)
    // {
    //     dev_err(sysDev, "Failed to allocate memory for device");
    //     return -ENOMEM;
    // }

    if ((ret = la9310_configure_pci(pciContext)))
        return ret;

    struct la9310_dev *la9310_dev = la9310_pci_priv_init(pciContext);
    if (!la9310_dev) {
        ret = -ENOMEM;
        dev_err(sysDev, "la9310_pci_priv_init failed, err %d\n", ret);
        return ret;
    }

    ret = la9310_base_probe(la9310_dev);
    if (ret) {
        dev_err(la9310_dev->dev, "la9310_base_probe failed, err %d\n", ret);
        goto err5;
    }

    pci_set_drvdata(pciContext, la9310_dev);

    ret = la9310_limesdr_device_init(la9310_dev, pciContext);
    if (ret < 0)
    {
        dev_err(sysDev, "la9310_limesdr_device_init fail (%i)\n", ret);
        // la9310_limesdr_device_destroy(myDevice);
        return -1;
    }

err5:
    // la9310_dev_reset_interrupt_capability(la9310_dev);

    // myDevice->attr.vendor = id->vendor;
    // myDevice->attr.product = id->device;
    return ret;
}

static void la9310_limesdr_pci_device_remove(struct pci_dev *pciContext)
{
    dev_info(&pciContext->dev, "[Removing device] vid:%04X pid:%04X\n", pciContext->vendor, pciContext->device);
    struct la9310_dev* myDevice = pci_get_drvdata(pciContext);

    dev_info(&pciContext->dev, "step 1\n");
    la9310_limesdr_device_destroy(myDevice);
    dev_info(&pciContext->dev, "step 2\n");
    la9310_base_remove(myDevice);
}

static struct pci_driver la9310_limesdr_pci_driver = {
    .name = LA9310_LIMESDR_DRIVER_NAME,
    .id_table = la9310_limesdr_pci_ids,
    .probe = la9310_limesdr_pci_probe,
    .remove = la9310_limesdr_pci_device_remove,
};

int la9310_limesdr_pci_register_driver()
{
	return pci_register_driver(&la9310_limesdr_pci_driver);
}

void la9310_limesdr_pci_unregister_driver()
{
	pci_unregister_driver(&la9310_limesdr_pci_driver);
}
