#ifndef _LINUX_LA9310_MEMORY_H
#define _LINUX_LA9310_MEMORY_H

#include <linux/genalloc.h>

#include "la9310_limesdr_device.h"

int la9310_initialize_reserved_memory_allocator(void);
void la9310_free_reserved_memory_allocator(void);

struct gen_pool* la9310_get_physical_memory_pool(void);

#endif
