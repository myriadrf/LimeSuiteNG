#ifndef LA9310_LIMESDR_DEVICE_H
#define LA9310_LIMESDR_DEVICE_H

#include <linux/pci.h>

#include "la9310_base.h"

int la9310_limesdr_device_init(struct la9310_dev* myDevice, struct pci_dev* pciContext);
void la9310_limesdr_device_destroy(struct la9310_dev* myDevice);

#endif // LA9310_LIMESDR_DEVICE_H