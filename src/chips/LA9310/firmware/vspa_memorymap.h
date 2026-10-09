#ifndef LIME_VSPA_MEMORYMAP_H
#define LIME_VSPA_MEMORYMAP_H

#include <stdint.h>

typedef enum {
    VSPA_MMAP_NONE = 0,
    VSPA_MMAP_L1_TRACE,
    VSPA_MMAP_ADC0,
    VSPA_MMAP_ADC1,
    VSPA_MMAP_ADC2,
    VSPA_MMAP_ADC3,
    VSPA_MMAP_RXDMA_LANE0,
    VSPA_MMAP_RXDMA_LANE1,
    VSPA_MMAP_RXDMA_LANE2,
    VSPA_MMAP_RXDMA_LANE3,
    VSPA_MMAP_DAC,
    VSPA_MMAP_TXDMA_LANE0,
    VSPA_MMAP_STATS,
    VSPA_MMAP_STATS2,
    VSPA_MMAP_TX_TONE_LANE0,
    VSPA_MMAP_VSPA_CORE_STATS,
    VSPA_MMAP_TX_TDD_CONFIG,
} e_vspa_feature;

typedef struct {
    uint32_t feature;
    uint32_t address;
} vspa_feature_t;

#endif
