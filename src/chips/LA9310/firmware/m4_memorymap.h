#ifndef LIME_LA9310_M4_FW_MEMORYMAP_H
#define LIME_LA9310_M4_FW_MEMORYMAP_H

#include <stdint.h>

typedef enum {
    M4_MMAP_NONE = 0,
    M4_MMAP_COMMAND_HIF,
    M4_MMAP_IQPLAYER_RXPIPE0,
    M4_MMAP_IQPLAYER_RXPIPE1,
    M4_MMAP_IQPLAYER_RXPIPE2,
    M4_MMAP_IQPLAYER_RXPIPE3,
    M4_MMAP_IQPLAYER_TXPIPE0,
} e_m4_mmap;

typedef struct {
    uint32_t type;
    uint32_t address;
} m4_memory_map_t;

#endif // LIME_LA9310_M4_FW_MEMORYMAP_H