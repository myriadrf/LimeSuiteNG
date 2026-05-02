#ifndef PIPELINES_H
#define PIPELINES_H

#ifndef __VSPA__ // pointer wrapper to maintain struct layout when accessing from other platforms
    #define VSPA_PTR(x) uint32_t
#else
    #define VSPA_PTR(x) x
#endif

typedef enum {
    VSPA_RO0,
    VSPA_RO1,
    VSPA_RX0,
    VSPA_RX1,
} e_rx_channel;

typedef struct stage_dir {
    VSPA_PTR(struct MemoryFIFO*) fifo;
    uint32_t bytes_done;
} stage_dir_t;

typedef struct Stage {
    stage_dir_t input;
    stage_dir_t output;
} stage_t;

typedef struct rx_pipeline {
    e_rx_channel channelIndex;
    uint32_t adc_axi_fifo_addr;
    uint32_t adc_dma_channel;
    uint32_t ddr_dma_channel;
    struct Stage adc;
    struct Stage qec;
    struct Stage dec;
    struct Stage ddr;
} rx_pipeline_t;

typedef struct tx_pipeline {
    struct Stage ddr;
    struct Stage interp;
    struct Stage qec;
    struct Stage dac;
} tx_pipeline_t;

typedef struct RxControl {
    uint32_t ddr_enabled;
    uint32_t generate_tone;
    uint32_t host_flow_control_disable;
    uint32_t burst_start_bytes;
    uint32_t burst_end_bytes;
    uint32_t ddr_wr_dma_ch_nb;
    uint32_t ddr_wr_dma_ch_mask;
} rx_control_t;

// parameters that host needs to know
typedef struct RxConfig {
    uint32_t ddr_base_address;
    uint32_t ddr_size;
    uint32_t ddr_step;
    uint32_t oversample;
} rx_config_t;

typedef struct TxControl {
    uint32_t ddr_enabled;
    uint32_t generate_tone;
    uint32_t host_flow_control_disable;
    uint32_t burst_start_bytes;
    uint32_t burst_end_bytes;
    uint32_t ddr_rd_dma_ch_nb;
    uint32_t ddr_rd_dma_ch_mask;
    uint32_t ddr_rd_dma_mBurst;
    uint32_t burst_active;
} tx_control_t;

typedef struct TxConfig {
    uint32_t ddr_base_address;
    uint32_t ddr_size;
    uint32_t ddr_step;
    uint32_t oversample;
} tx_config_t;

#endif
