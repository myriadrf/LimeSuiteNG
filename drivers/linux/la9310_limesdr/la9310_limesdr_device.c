#include <linux/kernel.h>

#include "la9310_limesdr_device.h"
#include "la9310_character_device.h"

static int gDeviceCounter = 0;

extern struct class* la9310_limesdr_class;

int la9310_limesdr_device_init(struct la9310_dev* la9310, struct pci_dev* pciContext)
{
    WARN_ON(la9310 == NULL);
    struct device* sysDev = &la9310->pdev->dev;

    char cdev_name[128];
    snprintf(cdev_name, sizeof(cdev_name), "limesdr_micro%i/control0", gDeviceCounter);
    dev_info(sysDev, "Creating /dev/%s\n", cdev_name);

    dev_t cdev_major_minor = la9310_cdev_create(la9310);
    if (cdev_major_minor == 0)
    {
        dev_dbg(sysDev, "Failed to allocate cdev\n");
        return -EINVAL;
    }

    void* drvdata = la9310;
    struct device* trxDev = device_create(la9310_limesdr_class, sysDev, cdev_major_minor, drvdata, "%s", cdev_name);
    if (IS_ERR(trxDev))
    {
        dev_err(sysDev, "Failed to create device\n");
        la9310_cdev_destroy(la9310);
        return -EINVAL;
    }

    dev_dbg(sysDev, "Create myDevice %p\n", la9310);

    ++gDeviceCounter;
    return 0;
}

void la9310_limesdr_device_destroy(struct la9310_dev* la9310)
{
    struct device* sysDev = &la9310->pdev->dev;
    dev_info(sysDev, "%s\n", __func__);

    dev_t major_minor = la9310_get_major_minor(la9310);
    device_destroy(la9310_limesdr_class, major_minor);
    la9310_cdev_destroy(la9310);
}
