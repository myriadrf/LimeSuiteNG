#ifndef LA9310_LIMESDR_DEVICE_H
#define LA9310_LIMESDR_DEVICE_H

#include <linux/pci.h>

#include "la9310_base.h"

int la9310_limesdr_device_init(struct la9310_dev* myDevice, struct pci_dev* pciContext);
void la9310_limesdr_device_destroy(struct la9310_dev* myDevice);

int custom_la9310_load_rtos_img(struct la9310_dev* la9310_dev, const uint8_t* rtos_img, uint32_t rtos_img_size);

#endif // LA9310_LIMESDR_DEVICE_H