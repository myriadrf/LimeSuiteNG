
#include <linux/types.h>
#include <linux/fs.h>
#include "la9310_ioctl.h"
#include "common_headers/la9310_modinfo.h"

#include "la9310_character_device.h"
#include "la9310_limesdr_device.h"

void la9310_modinfo_get(struct la9310_dev* la9310_dev, modinfo_t* mi);

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

    struct device *sysDev = &myDevice->pdev->dev;

    switch (cmd)
    {
    case LA9310_IOCTL_FIRMWARE_UPLOAD: {
        struct LA9310_IOCTL_firmware m;

        if (copy_from_user(&m, (void *)arg, sizeof(m)))
        {
            ret = -EFAULT;
            break;
        }

        void* buffer = kmalloc(m.size, GFP_KERNEL);
        if (!buffer)
        {
            dev_err(sysDev, "%s: IOCTL Failed to allocate temporary buffer for firmware\n", __func__);
        }

        if (copy_from_user(buffer, m.firmware_data, m.size))
        {
            ret = -EFAULT;
            kfree(buffer);
            break;
        }
        ret = custom_la9310_load_rtos_img(myDevice, buffer, m.size);
        kfree(buffer);
        return ret;
    }
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