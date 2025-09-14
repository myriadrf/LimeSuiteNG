#include "VSPA_iqplayer.h"

#include <assert.h>
#include <cstring>

#include "comms/PCIe/LA9310_PCIe.h"
#include "drivers/linux/la9310_limesdr/common_headers/la9310_host_if.h"

#if 1 // print debug messages
    #define printf_dbg_log(...) \
        do \
        { \
            printf(__VA_ARGS__); \
        } while (0)
#else
    #define printf_dbg_log(format, ...)
#endif

enum mbox_opc_e {
    MBOX_OPC_EMPTY_0, // 0x0
    MBOX_OPC_SINGLE_TONE_TX, // 0x1
    MBOX_OPC_SINGLE_TONE_RX, // 0x2
    MBOX_OPC_DCOC, // 0x3
    MBOX_OPC_BW_CAL, // 0x4
    MBOX_OPC_IQ_MOD_TX, // 0x5
    MBOX_OPC_IQ_MOD_RX, // 0x6
    MBOX_OPC_MSI, // 0x7
    MBOX_OPC_IQ_CORR, // 0x8
    MBOX_OPC_EMPTY_1, // 0x9
    MBOX_OPC_EMPTY_2, // 0xA
    MBOX_OPC_TX_DCO_CORR, // 0xB
    MBOX_OPC_OVERLAY_BASE, // 0xC
    MBOX_OPC_RX_CHAN_SELECT, // 0xD
    MBOX_OPC_RX_DCO_CORR, // 0xE
    MBOX_OPC_GET_STATS_COUNT // 0xF
};

namespace lime {

static const uint32_t vspa_cpu_id = 0;
static const uint32_t vspa_mbox_id = 0;

VSPA_iqplayer::VSPA_iqplayer(std::shared_ptr<LA9310_PCIe> port)
    : port(port)
    , mailbox(port)
{
    auto v_iqflood_ddr = port->GetBar(LA9310_WINDOW_IQFLOOD);
    auto v_la9310_bar2 = port->GetBar(LA9310_WINDOW_BAR2);
    auto dmem_proxy = port->GetBar(LA9310_WINDOW_IPC);

    vl_iqflood_ddr_addr = reinterpret_cast<uint8_t*>(v_iqflood_ddr.vaddr);
    iqflood_size = v_iqflood_ddr.size;

    // use last 1024 bytes of iqflood as shared vspa dmem proxy , vspa will write mirrored dmem value to avoid PCI read from host
    uint32_t* dmem_ptr = reinterpret_cast<uint32_t*>(dmem_proxy.vaddr) + 768;
    v_vspa_dmem_proxy_ro = reinterpret_cast<const volatile t_vspa_dmem_proxy*>(dmem_ptr);
    rx_vspa_proxy_ro = &(v_vspa_dmem_proxy_ro->rx_state_readonly[0]);
    tx_vspa_proxy_ro = &(v_vspa_dmem_proxy_ro->tx_state_readonly);

    // app_stats = &(((t_vspa_dmem_proxy*)v_vspa_dmem_proxy_ro)->app_stats);

    // use dmem structure at hardcoded address to write host status/request
    // dccivac((uint32_t*)tx_vspa_proxy_ro);
    void* tx_proxy_wo = nullptr;
    // volatile uint32_t* v_rx_vspa_proxy_wo = nullptr;
    uint8_t* BAR2_addr = reinterpret_cast<uint8_t*>(v_la9310_bar2.vaddr);
    if (tx_vspa_proxy_ro->rx_num_chan == 1)
    {
        tx_proxy_wo = BAR2_addr + 0x400000 + 0x00000000;
        // v_rx_vspa_proxy_wo = (uint32_t*)(BAR2_addr + 0x400000 + 0x00000040);
    }
    else
    {
        tx_proxy_wo = BAR2_addr + 0x500000 + 0x00004000;
        // v_rx_vspa_proxy_wo = (uint32_t*)(BAR2_addr + 0x500000 + 0x00004040);
    }
    tx_vspa_proxy_wo = reinterpret_cast<volatile t_tx_ch_host_proxy*>(tx_proxy_wo);
    printf_dbg_log("VSPA_iqplayer: IQFLOOD size: %lu, Rx channels %u\n", v_iqflood_ddr.size, tx_vspa_proxy_ro->rx_num_chan);
}

OpStatus VSPA_iqplayer::SelectRxChannel(uint32_t rx_channel_index)
{
    printf_dbg_log("IQPlayer: select ADC channel %i\n", rx_channel_index);
    uint64_t value = (uint64_t(MBOX_OPC_RX_CHAN_SELECT) << (24 + 32)) | rx_channel_index;
    mailbox.Send(vspa_cpu_id, vspa_mbox_id, value);
    return mailbox.Receive(vspa_cpu_id, vspa_mbox_id);
}

OpStatus VSPA_iqplayer::StartRx()
{
    return StartRx(iqflood_size, LA9310_IQFLOOD_PHYS_ADDR + rx_fifo_start_offset_in_iqflood);
}

OpStatus VSPA_iqplayer::StartRx(uint32_t fifo_size, uint32_t fifo_base_la9310_phys_addr)
{
    const mbox_opc_e command = MBOX_OPC_IQ_MOD_RX;
    const bool start = true;
    const bool test_load_start = false;
    const bool continuous = true;
    const uint8_t ddr_wr_dma_ch_nb = 1;
    const bool host_flow_control_disable = false;

    assert((fifo_size & 0xFFFF) == 0);
    assert(fifo_size / 4096 < 0x10000);
    assert(fifo_size / 4096 > 0);
    const uint16_t chunkCount4k = fifo_size / 4096;

    uint32_t loword = fifo_base_la9310_phys_addr;
    uint32_t hiword = 0;
    hiword |= command << 24;
    hiword |= continuous ? 0x00800000 : 0;
    hiword |= host_flow_control_disable ? 0x00400000 : 0;
    hiword |= test_load_start ? 0x00200000 : 0;
    hiword |= start ? 0x00100000 : 0;
    hiword |= (ddr_wr_dma_ch_nb << 16) & 0x00070000;
    hiword |= chunkCount4k & 0x0000FFFF;

    uint64_t value = uint64_t(hiword) << 32 | loword;

    printf_dbg_log("IQPlayer: Start Rx, fifo_size:%u\n", fifo_size);
    mailbox.Send(vspa_cpu_id, vspa_mbox_id, value);
    return mailbox.Receive(vspa_cpu_id, vspa_mbox_id);
}

OpStatus VSPA_iqplayer::StopRx()
{
    printf_dbg_log("IQPlayer: Stop Rx\n");
    uint64_t value = uint64_t(MBOX_OPC_IQ_MOD_RX) << (24 + 32);
    mailbox.Send(vspa_cpu_id, vspa_mbox_id, value);
    return mailbox.Receive(vspa_cpu_id, vspa_mbox_id);
}

OpStatus VSPA_iqplayer::StopTx()
{
    printf_dbg_log("IQPlayer: Stop Tx\n");
    uint64_t value = uint64_t(MBOX_OPC_IQ_MOD_TX) << (24 + 32);
    mailbox.Send(vspa_cpu_id, vspa_mbox_id, value);
    return mailbox.Receive(vspa_cpu_id, vspa_mbox_id);
}

OpStatus VSPA_iqplayer::Setup(uint32_t rxCount, uint32_t txCount)
{
    return SetupRx(0, 0, iqflood_size);
}

OpStatus VSPA_iqplayer::SetupRx(uint32_t channel, uint32_t fifo_start_offset, uint32_t fifo_size)
{
    VSPA_FIFO_State& rxState = mRx[channel];
    assert(rx_vspa_proxy_ro);

    // dccivac((uint32_t*)(rx_vspa_proxy_ro));

    // init fifo pointers
    rxState.fifo_start_addr = fifo_start_offset;
    rxState.fifo_size = fifo_size;
    rxState.fifo_offset = rx_vspa_proxy_ro[channel].la9310_fifo_consumed_size % fifo_size;

    // init flow counters
    rxState.bytes_consumed = rx_vspa_proxy_ro[channel].la9310_fifo_consumed_size;
    rxState.bytes_produced = rx_vspa_proxy_ro[channel].la9310_fifo_consumed_size;
    tx_vspa_proxy_wo->host_consumed_size[channel] = rx_vspa_proxy_ro[channel].la9310_fifo_consumed_size;

    return OpStatus::Success;
}

OpStatus VSPA_iqplayer::SetupTx(uint32_t fifo_start_offset, uint32_t fifo_size)
{
    VSPA_FIFO_State& txState = mTx;
    assert(tx_vspa_proxy_ro);
    // dccivac((uint32_t*)(tx_vspa_proxy_ro));

    /* check firmware is idle waiting for new data */
    //if (tx_vspa_proxy_ro->host_produced_size != tx_vspa_proxy_ro->la9310_fifo_enqueued_size) {
    //  printf("\n TX : modem is already running \n");
    //  fflush(stdout);
    //  return 0;
    //}

    // init fifo pointers
    txState.fifo_start_addr = fifo_start_offset;
    txState.fifo_size = fifo_size;
    txState.fifo_offset = tx_vspa_proxy_ro->la9310_fifo_enqueued_size % fifo_size;
    tx_vspa_proxy_wo->host_produced_size = tx_vspa_proxy_ro->la9310_fifo_enqueued_size;

    // init flow control
    txState.bytes_consumed = tx_vspa_proxy_ro->la9310_fifo_enqueued_size;
    txState.bytes_produced = tx_vspa_proxy_ro->la9310_fifo_enqueued_size;

    return OpStatus::Success;
}

int32_t VSPA_iqplayer::Receive(uint32_t channel, uint32_t* destination, uint32_t read_size)
{
    VSPA_FIFO_State& rxState = mRx[channel];

    // dccivac((uint32_t*)(rx_vspa_proxy_ro));

    // check stop/restart
    if (rx_vspa_proxy_ro[channel].DDR_wr_base_address == 0xdeadbeef)
    {
        rxState.bytes_produced = 0;
        rxState.bytes_consumed = 0;
        rxState.fifo_offset = 0;
        tx_vspa_proxy_wo->host_consumed_size[channel] = 0;
        return 0;
    }

    // Check new transfer
    rxState.bytes_produced = rx_vspa_proxy_ro[channel].la9310_fifo_consumed_size;
    uint32_t data_size = rxState.bytes_produced - rxState.bytes_consumed;
    if (data_size >= rxState.fifo_size)
    {
        printf("\n RX underrun , exit (data_size=0x%08x app_RX_total_produced_size=0x%08x app_RX_total_consumed_size=0x%08x)\n",
            data_size,
            rxState.bytes_produced,
            rxState.bytes_consumed);
        return 0;
    }

    const uint32_t contiguousBytesSize = rxState.fifo_size - rxState.fifo_offset;
    const uint32_t rx_ddr_step = tx_vspa_proxy_ro->rx_ddr_step;

    if (data_size < rx_ddr_step)
        return 0; // no data available

    if (data_size > contiguousBytesSize)
        data_size = contiguousBytesSize;
    if (data_size > read_size)
        data_size = read_size;

    // ready to fetch new data
    rxState.bytes_consumed += data_size;
    // app_stats->rx_stats[chan][STAT_EXT_DMA_DDR_WR] += data_size / rx_ddr_step;

    // xfer data
    auto ddr_src = vl_iqflood_ddr_addr + rxState.fifo_start_addr + rxState.fifo_offset;
    // l1_trace(L1_TRACE_MSG_DMA_DDR_RD_START, rxState.fifo_start_addr + rxState.fifo_offset);
    // invalidate_region(ddr_src, data_size);
    memcpy(destination, ddr_src, data_size);
    // l1_trace(L1_TRACE_MSG_DMA_DDR_RD_COMP, data_size);

    // update flow control
    tx_vspa_proxy_wo->host_consumed_size[channel] = rxState.bytes_consumed;

    rxState.fifo_offset += data_size;
    if (rxState.fifo_offset >= rxState.fifo_size)
        rxState.fifo_offset = 0;

    return data_size;
}

int32_t VSPA_iqplayer::Transmit(const uint32_t* src, uint32_t write_size)
{
    VSPA_FIFO_State& txState = mTx;

    //dccivac((uint32_t*)(tx_vspa_proxy_ro));

    // check stop/restart
    if (tx_vspa_proxy_ro->DDR_rd_base_address == 0xdeadbeef)
    {
        txState.fifo_offset = 0;
        txState.bytes_consumed = 0;
        txState.bytes_produced = 0;
        tx_vspa_proxy_wo->host_produced_size = 0;
        return 0;
    }

    // Check new transfer opty
    txState.bytes_consumed = tx_vspa_proxy_ro->la9310_fifo_enqueued_size;
    uint32_t fifo_filled_bytes = txState.bytes_produced - txState.bytes_consumed;
    if (fifo_filled_bytes > txState.fifo_size)
    {
        printf("\n TX underrun , exit (busy=0x%08x txState.bytes_produced=0x%08x txState.bytes_consumed=0x%08x)\n",
            fifo_filled_bytes,
            txState.bytes_produced,
            txState.bytes_consumed);
    }
    const uint32_t contiguousBytesSize = txState.fifo_size - txState.fifo_offset;
    uint32_t empty_size = txState.fifo_size - fifo_filled_bytes;
    const uint32_t tx_ddr_step = tx_vspa_proxy_ro->tx_ddr_step;
    if (empty_size < tx_ddr_step)
        return 0;

    if (empty_size > contiguousBytesSize)
        empty_size = contiguousBytesSize;

    if (write_size > empty_size)
        write_size = empty_size;

    // ready to send new data
    txState.bytes_produced += write_size;
    // app_stats->tx_stats[STAT_EXT_DMA_DDR_RD] += empty_size / tx_ddr_step;

    // xfer data
    auto ddr_dst = vl_iqflood_ddr_addr + txState.fifo_start_addr + txState.fifo_offset;
    //l1_trace(L1_TRACE_MSG_DMA_DDR_RD_START, txState.fifo_start_addr + txState.fifo_offset);
    memcpy(ddr_dst, src, write_size);
    //flush_region(ddr_dst, empty_size);
    //l1_trace(L1_TRACE_MSG_DMA_DDR_RD_COMP, empty_size);

    // update modem flow control
    tx_vspa_proxy_wo->host_produced_size = txState.bytes_produced;

    txState.fifo_offset += empty_size;
    if (txState.fifo_offset >= txState.fifo_size)
        txState.fifo_offset = 0;

    return write_size;
}

} // namespace lime
