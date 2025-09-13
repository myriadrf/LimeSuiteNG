#define DEBUG

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>

#include "version.h"
#include "la9310_pci.h"
#include "la9310_character_device.h"
#include "la9310_base.h"

struct class *la9310_limesdr_class = NULL;

static int __init la9310_limesdr_module_init(void)
{
    int ret = 0;
    pr_info("%s : v%s-g%s\n", __func__, LA9310_LIMESDR_VERSION, LA9310_LIMESDR_GIT_HASH);

    ret = la9310_subdrv_mod_init();
    if (ret)
        return ret;

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 4, 0)
    la9310_limesdr_class = class_create(THIS_MODULE, LA9310_LIMESDR_DRIVER_NAME);
#else
    la9310_limesdr_class = class_create(LA9310_LIMESDR_DRIVER_NAME);
#endif
    if (!la9310_limesdr_class)
    {
        ret = -EEXIST;
        pr_err(" Failed to create class \"%s\"\n", LA9310_LIMESDR_DRIVER_NAME);
        goto fail_create_class;
    }

    if ((ret = la9310_limesdr_alloc_chrdev_region()))
    {
        pr_err("%s (%i) Error allocating character device region\n", __func__, ret);
        goto fail_create_class;
    }

    if ((ret = la9310_limesdr_pci_register_driver()))
    {
        pr_err("%s (%i) Error while registering PCI driver\n", __func__, ret);
        goto fail_register;
    }
    return 0;

fail_register:
    class_destroy(la9310_limesdr_class);
fail_create_class:
    return ret;
}

static void __exit la9310_limesdr_module_exit(void)
{
    la9310_limesdr_pci_unregister_driver();
    la9310_limesdr_unregister_chrdev_region();
    class_destroy(la9310_limesdr_class);

    la9310_subdrv_mod_exit();

    pr_info("%s\n", __func__);
}

module_init(la9310_limesdr_module_init);
module_exit(la9310_limesdr_module_exit);

MODULE_INFO(version, LA9310_LIMESDR_VERSION);
MODULE_INFO(githash, LA9310_LIMESDR_GIT_HASH);
MODULE_INFO(author, "Lime Microsystems");

MODULE_LICENSE("GPL");
