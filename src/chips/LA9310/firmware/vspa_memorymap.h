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
    VSPA_MMAP_RXDDR_OUTPUT_LANE0,
    VSPA_MMAP_RXDDR_OUTPUT_LANE1,
    VSPA_MMAP_RXDDR_OUTPUT_LANE2,
    VSPA_MMAP_RXDDR_OUTPUT_LANE3,
    VSPA_MMAP_FW_INFO,
} e_vspa_feature;

typedef struct {
    uint32_t feature;
    uint32_t address;
} vspa_feature_t;

// Firmware build configuration (VSPA_MMAP_FW_INFO), read only
typedef struct fw_config {
    uint32_t rx_lane_count : 4;
    uint32_t tx_lane_count : 4;
    uint32_t max_decimation_exponent : 4;
    uint32_t max_interpolation_exponent : 4;
} fw_config_t;

#endif
