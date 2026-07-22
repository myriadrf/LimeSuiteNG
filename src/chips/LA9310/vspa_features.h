#ifndef LIME_VSPA_FEATURES_H
#define LIME_VSPA_FEATURES_H

#include <stdint.h>

typedef enum {
    F_VSPA_NONE = 0,
    F_VSPA_L1_TRACE,
    F_VSPA_TX_DMA,
    F_VSPA_RX_DMA,

    F_VSPA_FEATURE_COUNT
} e_vspa_feature;

typedef struct {
    e_vspa_feature feature;
    uint32_t address;
} feature_t;

#endif