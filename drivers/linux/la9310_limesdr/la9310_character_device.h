#ifndef LA9310_LIMESDR_CHARACTER_DEVICE_H
#define LA9310_LIMESDR_CHARACTER_DEVICE_H

#include <linux/cdev.h>

struct la9310_dev;

int la9310_limesdr_alloc_chrdev_region(void);
void la9310_limesdr_unregister_chrdev_region(void);

dev_t la9310_cdev_create(struct la9310_dev *myDevice);
void la9310_cdev_destroy(struct la9310_dev *myDevice);

dev_t la9310_get_major_minor(struct la9310_dev *myDevice);

#endif // LA9310_LIMESDR_CHARACTER_DEVICE_H