
#include <linux/types.h>
#include <linux/fs.h>
#include "la9310_ioctl.h"

#include "la9310_character_device.h"
#include "la9310_limesdr_device.h"
#include "la9310_dma_pool.h"

static int la9310_get_memory_layout(const struct la9310_dev* la9310_dev, struct LA9310_IOCTL_memory_layout* layout)
{
    memset(layout, 0, sizeof(struct LA9310_IOCTL_memory_layout));

    layout->window_size[LA9310_WINDOW_BAR0] = la9310_dev->mem_regions[LA9310_MEM_REGION_CCSR].size;
    layout->window_size[LA9310_WINDOW_BAR1] = la9310_dev->mem_regions[LA9310_MEM_REGION_TCML].size;
    layout->window_size[LA9310_WINDOW_BAR2] = la9310_dev->mem_regions[LA9310_MEM_REGION_TCMU].size;
    layout->window_size[LA9310_WINDOW_PERSISTENT] = la9310_dev->dma_info.host_buf.size;
    layout->window_size[LA9310_WINDOW_MSI] = PCIE_MSI_OB_SIZE;
    if (la9310_dev->dma_pools[LA9310_DMA_POOL_UNCACHED])
        layout->window_size[LA9310_WINDOW_UNCACHED_POOL] = la9310_dev->dma_pools[LA9310_DMA_POOL_UNCACHED]->region.size;
    if (la9310_dev->dma_pools[LA9310_DMA_POOL_CACHED])
        layout->window_size[LA9310_WINDOW_CACHED_POOL] = la9310_dev->dma_pools[LA9310_DMA_POOL_CACHED]->region.size;

    layout->host_interface.window_id = LA9310_WINDOW_BAR1;
    layout->host_interface.start_offset = ((uint8_t*)la9310_dev->hif - la9310_dev->mem_regions[LA9310_MEM_REGION_TCML].vaddr);
    layout->host_interface.size = sizeof(struct la9310_hif);

    return 0;
}

long la9310_ioctl(struct file* file, unsigned int cmd, unsigned long arg)
{
    long ret = 0;

    struct la9310_file* file_ctx = file->private_data;
    struct la9310_dev* la9310_dev = file_ctx ? file_ctx->dev : NULL;

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
            if (op.window_id == LA9310_WINDOW_BAR2 && la9310_bar2_overlaps_vspa(op.offset, sizeof(u32)) &&
                !la9310_rcw_completed(la9310_dev))
            {
                dev_err(la9310_dev->dev, "BAR2 VSPA memory access at 0x%08lX refused: RCW not completed\n", op.offset);
                return -EAGAIN;
            }
            if (ccsr_region->size < sizeof(u32) || op.offset > ccsr_region->size - sizeof(u32))
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
        default:
            dev_err(la9310_dev->dev, "CSR op on unsupported window %i\n", op.window_id);
            return -EINVAL;
        }

        if (copy_to_user((void*)arg, &op, sizeof(op)))
            return -EFAULT;

        break;
    }
    case LA9310_IOCTL_EVENT_WAIT: {
        struct LA9310_IOCTL_EVENT wait_arg;
        if (copy_from_user(&wait_arg, (struct LA9310_IOCTL_EVENT __user*)arg, sizeof(wait_arg)))
            return -EFAULT;

        ret = la9310_event_wait(file_ctx, wait_arg.event, msecs_to_jiffies(wait_arg.timeout_ms));
        break;
    }
    case LA9310_IOCTL_EVENT_CLEAR: {
        struct LA9310_IOCTL_EVENT_CTRL clear_arg;
        if (copy_from_user(&clear_arg, (struct LA9310_IOCTL_EVENT_CTRL __user*)arg, sizeof(clear_arg)))
            return -EFAULT;

        // discards pending events, enable_bits/enable_mask are ignored
        la9310_events_clear(la9310_dev, clear_arg.clear_bits, clear_arg.clear_mask);
        ret = 0;
        break;
    }
    case LA9310_IOCTL_EVENT_ENABLE: {
        struct LA9310_IOCTL_EVENT_MASK enable_arg;
        if (copy_from_user(&enable_arg, (struct LA9310_IOCTL_EVENT_MASK __user*)arg, sizeof(enable_arg)))
            return -EFAULT;

        ret = la9310_events_enable(file_ctx, enable_arg.enable_bits, enable_arg.enable_mask);
        if (ret)
            break;
        enable_arg.enabled = file_ctx->enabled_events;
        if (copy_to_user((struct LA9310_IOCTL_EVENT_MASK __user*)arg, &enable_arg, sizeof(enable_arg)))
            return -EFAULT;
        break;
    }
    case LA9310_IOCTL_EVENT_REINIT:
        ret = la9310_events_reinit(la9310_dev);
        break;

    case LA9310_IOCTL_CACHE_SYNC_FOR_CPU:
    case LA9310_IOCTL_CACHE_SYNC_FOR_DEVICE: {
        struct la9310_atu mem;
        if (copy_from_user(&mem, (struct la9310_atu __user*)arg, sizeof(mem)))
            return -EFAULT;

        ret = la9310_dma_buf_sync(file_ctx, &mem, cmd == LA9310_IOCTL_CACHE_SYNC_FOR_CPU);
        break;
    }

    case LA9310_IOCTL_DMA_ALLOC: {
        struct la9310_atu mem;
        if (copy_from_user(&mem, (struct la9310_atu __user*)arg, sizeof(mem)))
            return -EFAULT;

        ret = la9310_dma_buf_alloc(file_ctx, &mem);
        if (ret)
            break;

        if (copy_to_user((struct la9310_atu __user*)arg, &mem, sizeof(mem)))
        {
            la9310_dma_buf_free(file_ctx, mem.host_bus);
            return -EFAULT;
        }
        break;
    }

    case LA9310_IOCTL_DMA_FREE: {
        struct la9310_atu mem;
        if (copy_from_user(&mem, (struct la9310_atu __user*)arg, sizeof(mem)))
            return -EFAULT;

        ret = la9310_dma_buf_free(file_ctx, mem.host_bus);
        break;
    }

    default:
        dev_err(la9310_dev->dev, "Unknown command 0x%X\n", cmd);
        return -ENOTTY;
    }
    return ret;
}
