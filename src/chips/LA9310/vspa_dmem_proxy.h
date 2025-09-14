#ifndef LIME_VSPA_DMEM_PROXY_H_
#define LIME_VSPA_DMEM_PROXY_H_

#define VSPA_RX_NUM_MAX_CHAN 4

// #include "vspa_stats.h"

typedef struct s_tx_ch_host_proxy {
    uint32_t la9310_fifo_enqueued_size;
    uint32_t la9310_fifo_consumed_size;
    uint32_t DDR_rd_base_address;
    uint32_t DDR_rd_size;
    uint32_t host_produced_size;
    uint32_t rx_decim;
    uint32_t tx_upsmp;
    uint32_t rx_num_chan;
    uint32_t host_consumed_size[VSPA_RX_NUM_MAX_CHAN];
    uint32_t rx_ddr_step;
    uint32_t tx_ddr_step;
    uint32_t gbl_stats_fetch;
    uint32_t dmemProxyOffset;
} t_tx_ch_host_proxy;

typedef struct s_rx_ch_host_proxy {
    uint32_t la9310_fifo_produced_size;
    uint32_t la9310_fifo_consumed_size;
    uint32_t DDR_wr_base_address;
    uint32_t DDR_wr_size;
} t_rx_ch_host_proxy;

typedef struct s_vspa_dmem_proxy {
    t_tx_ch_host_proxy tx_state_readonly;
    t_rx_ch_host_proxy rx_state_readonly[VSPA_RX_NUM_MAX_CHAN];
    // t_stats vspa_stats;
    // t_stats host_stats;
    // t_stats app_stats;
} t_vspa_dmem_proxy;

#endif // LIME_VSPA_DMEM_PROXY_H_
