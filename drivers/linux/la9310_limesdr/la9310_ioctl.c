
#include <linux/types.h>
#include <linux/fs.h>
#include "la9310_ioctl.h"
#include "la9310_vspa.h"

#include "la9310_character_device.h"
#include "la9310_limesdr_device.h"

static int la9310_get_memory_layout(const struct la9310_dev* la9310_dev, struct LA9310_IOCTL_memory_layout* layout)
{
    memset(layout, 0, sizeof(struct LA9310_IOCTL_memory_layout));

    layout->window_size[LA9310_WINDOW_BAR0] = la9310_dev->mem_regions[LA9310_MEM_REGION_CCSR].size;
    layout->window_size[LA9310_WINDOW_BAR1] = la9310_dev->mem_regions[LA9310_MEM_REGION_TCML].size;
    layout->window_size[LA9310_WINDOW_BAR2] = la9310_dev->mem_regions[LA9310_MEM_REGION_TCMU].size;
    layout->window_size[LA9310_WINDOW_SCRATCH] = la9310_dev->dma_info.host_buf.size;
    layout->window_size[LA9310_WINDOW_MSI] = PCIE_MSI_OB_SIZE;
    layout->window_size[LA9310_WINDOW_IPC] = PAGE_SIZE;
    layout->window_size[LA9310_WINDOW_IQFLOOD] = la9310_dev->iqflood_region.size;

    layout->host_interface.window_id = LA9310_WINDOW_BAR1;
    layout->host_interface.start_offset = ((uint8_t*)la9310_dev->hif - la9310_dev->mem_regions[LA9310_MEM_REGION_TCML].vaddr);
    layout->host_interface.size = sizeof(struct la9310_hif);

    return 0;
}

void la9310_flush_cache(struct la9310_dev* la9310_dev, struct LA9310_IOCTL_flush_cache* m, dma_addr_t phys_addr)
{
    if (m->sync_to_cpu)
        dma_sync_single_for_cpu(la9310_dev->dev, phys_addr + m->offset, m->size, m->dir);
    else
        dma_sync_single_for_device(la9310_dev->dev, phys_addr + m->offset, m->size, m->dir);
}

long la9310_ioctl(struct file* file, unsigned int cmd, unsigned long arg)
{
    long ret = 0;

    struct la9310_dev* la9310_dev = file->private_data;
    struct vspa_device* vspadev = (struct vspa_device*)la9310_dev->vspa_priv;
    struct LA9310_IOCTL_flush_cache cache_entry;
    struct LA9310_IOCTL_firmware fw;
    unsigned int timeout_ms;

    WARN_ON(la9310_dev == NULL);

    if (!la9310_dev)
    {
        return -ENODEV;
    }

    switch (cmd)
    {
    case LA9310_IOCTL_GET_MEMORY_LAYOUT: {
        struct LA9310_IOCTL_memory_layout layout;

        ret = la9310_get_memory_layout(la9310_dev, &layout);
        if (ret)
            return ret;

        if (copy_to_user((void*)arg, &layout, sizeof(struct LA9310_IOCTL_memory_layout)))
            return -EFAULT;

        break;
    }
    case LA9310_IOCTL_FLUSH_CACHE_VSPA_DMEM:
        // if (copy_from_user(&cache_entry, (void*)arg, sizeof(struct LA9310_IOCTL_flush_cache)))
        //     return -EFAULT;

        // cache_entry.dir = DMA_BIDIRECTIONAL;
        // la9310_flush_cache(la9310_dev, &cache_entry, la9310_dev->dmem_proxy.phys_addr);
        // ret = 0;
        break;
    case LA9310_IOCTL_FLUSH_CACHE_IQFLOOD:
        if (copy_from_user(&cache_entry, (void*)arg, sizeof(struct LA9310_IOCTL_flush_cache)))
            return -EFAULT;

        cache_entry.dir = DMA_BIDIRECTIONAL;
        la9310_flush_cache(la9310_dev, &cache_entry, la9310_dev->iqflood_region.phys_addr);
        ret = 0;
        break;
    case LA9310_IOCTL_LOAD_M4_FW:
        ret = copy_from_user(&fw, (struct LA9310_IOCTL_firmware*)arg, sizeof(fw));
        if (ret < 0)
        {
            dev_err(la9310_dev->dev, "%s copy_from_user, err %ld\n", __func__, ret);
            return ret;
        }
        ret = la9310_load_m4_firmware(la9310_dev, fw.firmware_data, fw.size);
        break;
    case LA9310_IOCTL_LOAD_VSPA_FW:
        ret = copy_from_user(&fw, (struct LA9310_IOCTL_firmware*)arg, sizeof(fw));
        if (ret < 0)
        {
            dev_err(la9310_dev->dev, "%s copy_from_user, err %ld\n", __func__, ret);
            return ret;
        }
        ret = vspa_load_dsp(la9310_dev, vspadev, fw.firmware_data, fw.size);
        break;
    case LA9310_IOCTL_CSR_OP: {
        struct LA9310_IOCTL_CSR_op op;
        if (copy_from_user(&op, (void*)arg, sizeof(op)))
            return -EFAULT;

        switch (op.window_id)
        {
        case LA9310_WINDOW_BAR0:
        case LA9310_WINDOW_BAR1:
        case LA9310_WINDOW_BAR2: {
            struct la9310_mem_region_info* ccsr_region = &la9310_dev->mem_regions[op.window_id];
            if (op.offset < 0 || op.offset > ccsr_region->size)
            {
                dev_err(la9310_dev->dev, "CSR offset %08lX out of bounds\n", op.offset);
                return -EINVAL;
            }

            if (op.write)
            {
                //dev_info(la9310_dev->dev, "CSR Write window:%i + 0x%08lX, value: %08X\n", op.window_id, op.offset, op.value);
                writel(op.value, ccsr_region->vaddr + op.offset);
                wmb();
            }
            else
            {
                rmb();
                op.value = readl(ccsr_region->vaddr + op.offset);
                //dev_info(la9310_dev->dev, "CSR Read window:%i + 0x%08lX, value: %08X\n", op.window_id, op.offset, op.value);
            }
            ret = 0;
            break;
        }
        }

        if (copy_to_user((void*)arg, &op, sizeof(op)))
            return -EFAULT;

        break;
    }
    case LA9310_IOCTL_SIRQ_WAIT: {
        struct LA9310_IOCTL_SIRQ irq_wait;
        if (copy_from_user(&irq_wait, (struct LA9310_IOCTL_SIRQ __user*)arg, sizeof(irq_wait)))
            return -EFAULT;

        ret = la9310_softirq_wait(&la9310_dev->soft_irq, irq_wait.irq_index, msecs_to_jiffies(irq_wait.timeout_ms));
        break;
    }
    case LA9310_IOCTL_SIRQ_CONTROL: {
        struct LA9310_IOCTL_SIRQ_CTRL irq_arg;
        if (copy_from_user(&irq_arg, (struct LA9310_IOCTL_SIRQ_CTRL __user*)arg, sizeof(irq_arg)))
            return -EFAULT;

        // int success = down_timeout(&la9310_dev->soft_irq.clear_semaphore, msecs_to_jiffies(100));
        // if (success != 0) // on failure
        // {
        //     ret = -EBUSY;
        //     break;
        // }

        la9310_softirq_clear_local(la9310_dev, irq_arg.clear_bits, irq_arg.clear_mask);
        // la9310_raise_msgunit_irq(la9310_dev, 0, 0);
        // up(&la9310_dev->soft_irq.clear_semaphore);
        ret = 0;
        break;
    }

    case LA9310_IOCTL_USERSPACE_DMA: {
        struct la9310_userspace_dma dma_map;
        dma_map = la9310_dev->user_dma;
        if (copy_to_user((void*)arg, &dma_map, sizeof(dma_map)))
            return -EFAULT;
        ret = 0;
        break;
    }

    default:
        dev_err(la9310_dev->dev, "Unknown command 0x%X\n", cmd);
        return -ENOTTY;
    }
    return ret;
}
