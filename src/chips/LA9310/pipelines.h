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

typedef struct MemoryBlock {
    uint32_t addr;
    uint32_t size;
    uint32_t timestamp;
    uint32_t flags;
} MemoryBlock_t;

typedef uint16_t MetaHandle_t;

#define MEM_POOL_SIZE 8
typedef struct HandlesStack {
    MetaHandle_t items[MEM_POOL_SIZE];
    uint16_t count;
} HandlesStack_t;

typedef struct rx_pipeline {
    e_rx_channel channelIndex;
    uint32_t adc_axi_fifo_addr;
    uint16_t adc_dma_channel;
    uint16_t ddr_dma_channel;
    struct Stage adc;
    struct Stage ddr;
    HandlesStack_t mem_handles_pool;
} rx_pipeline_t;

typedef struct tx_pipeline {
    struct Stage ddr;
    struct Stage interp;
    struct Stage dac;
} tx_pipeline_t;

typedef struct RxControl {
    uint16_t ddr_enabled;
    uint16_t generate_tone;
    uint16_t host_flow_control_disable;
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
    uint32_t ddr_rd_dma_mBurst;
    uint32_t dma_table_loop;
} tx_control_t;

typedef struct TxConfig {
    uint32_t oversample;
} tx_config_t;

// Packet flags
enum {
    PKT_HAS_TIMESTAMP = (1 << 0),
    PKT_START = (1 << 1),
    PKT_END = (1 << 2),
    PKT_IRQ = (1 << 3),
    PKT_DMA_TCD_END = (1 << 4),
};

#endif
