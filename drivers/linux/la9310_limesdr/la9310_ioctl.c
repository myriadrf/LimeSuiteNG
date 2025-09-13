
#include <linux/types.h>
#include <linux/fs.h>
#include "la9310_ioctl.h"

#include "la9310_character_device.h"
#include "la9310_limesdr_device.h"

static int la9310_get_memory_layout(const struct la9310_dev* la9310_dev, struct LA9310_IOCTL_memory_layout* layout)
{
    memset(layout, 0, sizeof(struct LA9310_IOCTL_memory_layout));

    layout->window_size[LA9310_WINDOW_BAR0] = la9310_dev->mem_regions[LA9310_MEM_REGION_CCSR].size;
    layout->window_size[LA9310_WINDOW_BAR1] = la9310_dev->mem_regions[LA9310_MEM_REGION_TCML].size;
    layout->window_size[LA9310_WINDOW_BAR2] = la9310_dev->mem_regions[LA9310_MEM_REGION_TCMU].size;
    layout->window_size[LA9310_WINDOW_SCRATCH] = la9310_dev->scratch_buf_size;
    layout->window_size[LA9310_WINDOW_MSI] = PCIE_MSI_OB_SIZE;
    layout->window_size[LA9310_WINDOW_IPC] = 4096;
    layout->window_size[LA9310_WINDOW_IQFLOOD] = la9310_dev->iqflood_region.size;

    layout->host_interface.window_id = LA9310_WINDOW_BAR1;
    layout->host_interface.start_offset = ((uint8_t*)la9310_dev->hif - la9310_dev->mem_regions[LA9310_MEM_REGION_TCML].vaddr);
    layout->host_interface.size = sizeof(struct la9310_hif);

    return 0;
}

long la9310_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    long ret = 0;

    struct la9310_dev *myDevice = file->private_data;

    WARN_ON(myDevice == NULL);

    if (!myDevice)
    {
        return -ENODEV;
    }

    switch (cmd)
    {
    case LA9310_IOCTL_GET_MEMORY_LAYOUT: {
        struct LA9310_IOCTL_memory_layout layout;

        ret = la9310_get_memory_layout(myDevice, &layout);
        if (ret)
            return ret;

        if (copy_to_user((void*)arg, &layout, sizeof(struct LA9310_IOCTL_memory_layout)))
            return -EFAULT;

        break;
    }
    default:
        return -ENOTTY;
    }
    return ret;
}