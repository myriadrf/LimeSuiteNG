#include "VSPA_iqplayer.h"

#include "limesuiteng/types.h"
#include "limesuiteng/Logger.h"

#include <assert.h>
#include <cstring>
#include <iostream>
#include <thread>

#include "interface/IDCCorrector.h"
#include "interface/IQuadratureErrorCorrector.h"

#include "streaming/samplesConversion.h"

#include "comms/PCIe/LA9310_PCIe.h"
#include "drivers/linux/la9310_limesdr/common_headers/la9310_host_if.h"

#ifndef M_PI
    #define M_PI 3.14159265358979323846 /* pi */
#endif

#if 0 // print debug messages
    #define printf_dbg_log(...) \
        do \
        { \
            printf(__VA_ARGS__); \
        } while (0)
#else
    #define printf_dbg_log(format, ...)
#endif

enum {
    MBOX_EMPTY = 0, // 0x0
    MBOX_IQ_CORR_FTAP0, // 0x1
    MBOX_IQ_CORR_FTAP1, // 0x2
    MBOX_IQ_CORR_FTAP2, // 0x3
    MBOX_IQ_CORR_FTAP3, // 0x4
    MBOX_IQ_CORR_FTAP4, // 0x5
    MBOX_IQ_CORR_FTAP5, // 0x6
    MBOX_IQ_CORR_FTAP6, // 0x7
    MBOX_IQ_CORR_FTAP7, // 0x8
    MBOX_IQ_CORR_FTAP8, // 0x9
    MBOX_IQ_CORR_FTAP9, // 0xA
    MBOX_IQ_CORR_FTAP10, // 0xB
    MBOX_IQ_CORR_FTAP11, // 0xC
    MBOX_IQ_CORR_FTAP12, // 0xD
    MBOX_IQ_CORR_DC_I, // 0xE
    MBOX_IQ_CORR_DC_Q, // 0xF
    MBOX_IQ_CORR_FDELAY, // 0x10
    MBOX_IQ_CORR_MAX, // 0x11
};

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
    MBOX_OPC_GET_STATS_COUNT, // 0xF
    MBOX_OPC_DONE_SWRESET, // 0x10
    MBOX_OPC_Tx_reset_counter, // 0x11
    MBOX_OPC_Tx_rst_ptr, // 0x12
    MBOX_OPC_Tx_qxiq_enable, // 0x13
    MBOX_OPC_Tx_abort,
};

namespace lime {

static const uint32_t vspa_cpu_id = 0;
static const uint32_t vspa_mbox_id = 0;

static const double refClk = 30.72e6;

VSPA_iqplayer::VSPA_iqplayer(std::shared_ptr<LA9310_PCIe> port)
    : port(port)
    , mailbox(std::make_shared<VSPA_mailbox>(port))
    , tx_dma_channel_count(1)
{
    auto v_iqflood_ddr = port->GetBar(LA9310_WINDOW_IQFLOOD);
    auto v_la9310_bar2 = port->GetBar(LA9310_WINDOW_BAR2);
    auto dmem_proxy = port->GetBar(LA9310_WINDOW_IPC);

    vl_iqflood_ddr_addr = reinterpret_cast<uint8_t*>(v_iqflood_ddr.vaddr);
    iqflood_size = v_iqflood_ddr.size;

    // use last 1024 bytes of iqflood as shared vspa dmem proxy , vspa will write mirrored dmem value to avoid PCI read from host
    uint32_t* dmem_ptr = reinterpret_cast<uint32_t*>(dmem_proxy.vaddr) + 768;
    v_vspa_dmem_proxy_ro = reinterpret_cast<volatile t_vspa_dmem_proxy*>(dmem_ptr);
    rx_vspa_proxy_ro = &(v_vspa_dmem_proxy_ro->rx_state_readonly[0]);
    tx_vspa_proxy_ro = &(v_vspa_dmem_proxy_ro->tx_state_readonly);
    app_stats = &(reinterpret_cast<volatile t_vspa_dmem_proxy*>(v_vspa_dmem_proxy_ro)->vspa_stats);

    // use dmem structure at hardcoded address to write host status/request
    // dccivac((uint32_t*)tx_vspa_proxy_ro);
    void* tx_proxy_wo = nullptr;
    void* rx_proxy_wo = nullptr;
    uint8_t* BAR2_addr = reinterpret_cast<uint8_t*>(v_la9310_bar2.vaddr);

    auto nv_tx_vspa_proxy_ro = const_cast<t_tx_ch_host_proxy*>(tx_vspa_proxy_ro);
    port->sync_dmem_proxy_after_write(reinterpret_cast<uint8_t*>(nv_tx_vspa_proxy_ro), sizeof(t_tx_ch_host_proxy));

    if (tx_vspa_proxy_ro->rx_num_chan <= 1)
    {
        tx_proxy_wo = BAR2_addr + 0x400000 + 0x00000000;
        rx_proxy_wo = BAR2_addr + 0x400000 + 0x00000040;
    }
    else
    {
        tx_proxy_wo = BAR2_addr + 0x500000 + 0x00004000;
        rx_proxy_wo = BAR2_addr + 0x500000 + 0x00004040;
    }
    tx_vspa_proxy_wo = reinterpret_cast<volatile t_tx_ch_host_proxy*>(tx_proxy_wo);
    rx_vspa_proxy_wo = reinterpret_cast<volatile t_tx_ch_host_proxy*>(rx_proxy_wo);
    printf_dbg_log("VSPA_iqplayer: IQFLOOD size: %lu, Rx channels %u\n", v_iqflood_ddr.size, tx_vspa_proxy_ro->rx_num_chan);
}

OpStatus VSPA_iqplayer::SelectRxChannel(uint32_t rx_channel_index)
{
    // const std::lock_guard<std::mutex> lock(mx);
    printf_dbg_log("IQPlayer: select ADC channel %i\n", rx_channel_index);
    uint64_t value = (uint64_t(MBOX_OPC_RX_CHAN_SELECT) << (24 + 32)) | rx_channel_index;
    return mailbox->Message(vspa_cpu_id, vspa_mbox_id, value);
}

OpStatus VSPA_iqplayer::StartRx()
{
    VSPA_FIFO_State& rxState = mRx[0];
    return StartRx(0, rxState.fifo_size);
}

OpStatus VSPA_iqplayer::StartTx()
{
    VSPA_FIFO_State& txState = mTx;
    return StartTx(txState.fifo_size, true);
}

OpStatus VSPA_iqplayer::StartTx(uint32_t fifo_size, bool flow_control)
{
    // // const std::lock_guard<std::mutex> lock(mx);

    const mbox_opc_e command = MBOX_OPC_IQ_MOD_TX;
    const bool start = true;
    const bool test_load_start = false;

    const uint8_t ddr_rd_dma_ch_nb = tx_dma_channel_count;
    const bool ddr_rd_dma_mBurst = false; // use burst to improve performance
    const bool host_flow_control_disable = !flow_control;

    assert(fifo_size / 4096 < 0x10000);
    assert(fifo_size / 4096 > 0);
    if (fifo_size == 0)
        return OpStatus::InvalidValue;

    const uint16_t chunkCount4k = fifo_size / 4096;

    uint32_t loword = LA9310_IQFLOOD_PHYS_ADDR + mTx.fifo_start_addr; // fifo_base_la9310_phys_addr
    uint32_t hiword = 0;
    hiword |= command << 24;
    hiword |= host_flow_control_disable ? 0x00400000 : 0;
    hiword |= test_load_start ? 0x00200000 : 0;
    hiword |= start ? 0x00100000 : 0;
    hiword |= ddr_rd_dma_mBurst ? 0x00080000 : 0;
    hiword |= (ddr_rd_dma_ch_nb << 16) & 0x00070000;
    hiword |= chunkCount4k & 0x0000FFFF;

    uint64_t value = uint64_t(hiword) << 32 | loword;
    printf_dbg_log("IQPlayer: Start Tx, fifo_size:%u, fifo_base:%08X disable_flow_control:%i ch_nb:%i bytesReady:%li\n",
        fifo_size,
        loword,
        host_flow_control_disable,
        ddr_rd_dma_ch_nb,
        mTx.bytes_produced);
    uint64_t response = 0;
    OpStatus status = mailbox->Message(vspa_cpu_id, vspa_mbox_id, value, &response);
    if (status != OpStatus::Success)
        return status;
    return response == 1 ? OpStatus::Success : OpStatus::Error;
}

OpStatus VSPA_iqplayer::StartTxTone(bool enabled)
{
    OpStatus status;

    if (!enabled)
        return StopTx();

    status = SetupTx(0, iqflood_size / 4);
    if (status != OpStatus::Success)
        return status;

    std::vector<complex16_t> samples(iqflood_size / 4 / sizeof(complex16_t));
    constexpr uint16_t data[16] = { 0xb045,
        0x6068,
        0x80e7,
        0x107b,
        0x9097,
        0xb045,
        0xe084,
        0x80e7,
        0x40ba,
        0x9097,
        0x7018,
        0xe084,
        0x6068,
        0x40ba,
        0x107b,
        0x7018 };
    const complex16_t* pattern = reinterpret_cast<const complex16_t*>(data);
    for (size_t i = 0; i < samples.size(); ++i)
        samples[i] = pattern[i % 8];

    // fill fifos
    Transmit(samples.data(), samples.size() * sizeof(complex16_t), 0);
    // start tx
    VSPA_FIFO_State& txState = mTx;
    return StartTx(txState.fifo_size, false);
}

OpStatus VSPA_iqplayer::StartRx(uint8_t channel, uint32_t fifo_size)
{
    // const std::lock_guard<std::mutex> lock(mx);

    VSPA_FIFO_State& rxState = mRx[channel];
    rxState.bytes_consumed = 0;
    rxState.bytes_produced = 0;
    rxState.last_produced = 0;
    rxState.last_consumed = 0;
    tx_vspa_proxy_wo->host_consumed_size[0] = 0;

    const mbox_opc_e command = MBOX_OPC_IQ_MOD_RX;
    const bool start = true;
    const bool test_load_start = false;
    const bool continuous = true;
    const uint8_t ddr_wr_dma_ch_nb = 2;
    const bool host_flow_control_disable = false;

    assert(fifo_size / 4096 < 0x10000);
    assert(fifo_size / 4096 > 0);
    if (fifo_size == 0)
        return OpStatus::InvalidValue;
    const uint16_t chunkCount4k = fifo_size / 4096;

    uint32_t loword = LA9310_IQFLOOD_PHYS_ADDR + mRx[channel].fifo_start_addr; //fifo_base_la9310_phys_addr;
    uint32_t hiword = 0;
    hiword |= command << 24;
    hiword |= continuous ? 0x00800000 : 0;
    hiword |= host_flow_control_disable ? 0x00400000 : 0;
    hiword |= test_load_start ? 0x00200000 : 0;
    hiword |= start ? 0x00100000 : 0;
    hiword |= (ddr_wr_dma_ch_nb << 16) & 0x00070000;
    hiword |= chunkCount4k & 0x0000FFFF;

    uint64_t value = uint64_t(hiword) << 32 | loword;

    printf_dbg_log("IQPlayer: Start Rx, fifo_size:%u, fifo_base:%08X\n", fifo_size, loword);
    return mailbox->Message(vspa_cpu_id, vspa_mbox_id, value);
}

OpStatus VSPA_iqplayer::StopRx()
{
    // const std::lock_guard<std::mutex> lock(mx);

    if (rx_vspa_proxy_ro->DDR_wr_base_address == 0xDEADBEEF) // Rx is not active
        return OpStatus::Success;

    printf_dbg_log("IQPlayer: Stop Rx\n");
    // t_stats stats = GetStats();
    // const uint32_t app_ddr = mRx[0].bytes_produced / tx_vspa_proxy_ro->rx_ddr_step;
    // printf("Rx AXIQ_WR:%08X\n"
    //     "Rx DDR_RD :%08X\n"
    //     "Rx APP_RD :%08X\n",
    //     stats.rx_stats[0][STAT_DMA_AXIQ_WRITE],
    //     stats.rx_stats[0][STAT_DMA_DDR_RD],
    //     app_ddr
    //     );

    uint64_t value = uint64_t(MBOX_OPC_IQ_MOD_RX) << (24 + 32);
    OpStatus status = mailbox->Message(vspa_cpu_id, vspa_mbox_id, value);

    // clear rx buffer
    // volatile auto ddr_src = vl_iqflood_ddr_addr + mRx[0].fifo_start_addr;
    // memset(ddr_src, 0, mRx[0].fifo_size);

    return status;
}

OpStatus VSPA_iqplayer::StopTx()
{
    // const std::lock_guard<std::mutex> lock(mx);
    //return OpStatus::Success;

    if (tx_vspa_proxy_ro->DDR_rd_base_address == 0xDEADBEEF) // Tx is not active
        return OpStatus::Success;

    printf_dbg_log("IQPlayer: Stop Tx\n");
    uint64_t value = uint64_t(MBOX_OPC_IQ_MOD_TX) << (24 + 32);

    OpStatus status = mailbox->Message(vspa_cpu_id, vspa_mbox_id, value);
    mTx.fifo_offset = 0;

    // tx_vspa_proxy_wo->host_produced_size = 0;

    // clear tx buffer
    // auto ddr_dst = vl_iqflood_ddr_addr + mTx.fifo_start_addr;
    // memset(ddr_dst, 0, mTx.fifo_size);
    // complex16_t* ps = reinterpret_cast<complex16_t*>(ddr_dst);
    // for (int i = 0; i < mTx.fifo_size / sizeof(complex16_t); ++i)
    //     ps[i] = complex16_t(0, 32700 * (i & 1));

    mTx.bytes_consumed = 0;
    mTx.bytes_produced = 0;
    mTx.fifo_offset = 0;

    // t_stats stats = GetStats();
    // printf("Tx AXIQ_WR:%08X\n"
    //     "Tx DDR_RD :%08X\n"
    //     "Tx APP_WR :%08X\n"
    //     "Tx DDR_UDR:%08X\n",
    //     stats.tx_stats[STAT_DMA_AXIQ_WRITE],
    //     stats.tx_stats[STAT_DMA_DDR_RD],
    //     app_ddr,
    //     stats.tx_stats[ERROR_DMA_DDR_RD_UNDERRUN]
    //     );
    return status;
}

OpStatus VSPA_iqplayer::Setup(uint32_t rxCount, uint32_t txCount, double expectedTxDataRate)
{
    OpStatus status = OpStatus::Success;
    if (txCount)
    {
        status = SetupTx(0, iqflood_size / 4, expectedTxDataRate);
        if (status != OpStatus::Success)
            return status;
    }
    if (rxCount)
    {
        status = SetupRx(0, iqflood_size / 4, iqflood_size / 4);
        if (status != OpStatus::Success)
            return status;
    }
    return status;
}

OpStatus VSPA_iqplayer::SetupRx(uint32_t channel, uint32_t fifo_start_offset, uint32_t fifo_size)
{
    // const std::lock_guard<std::mutex> lock(mx);
    VSPA_FIFO_State& rxState = mRx[channel];
    assert(rx_vspa_proxy_ro);

    if (fifo_size == 0)
        return OpStatus::InvalidValue;

    // dccivac((uint32_t*)(rx_vspa_proxy_ro));
    auto nv_rx_vspa_proxy_ro = const_cast<t_rx_ch_host_proxy*>(rx_vspa_proxy_ro);
    port->sync_dmem_proxy_before_read(reinterpret_cast<uint8_t*>(nv_rx_vspa_proxy_ro), sizeof(t_rx_ch_host_proxy));

    // init fifo pointers
    rxState.fifo_start_addr = fifo_start_offset;
    rxState.fifo_size = fifo_size;
    rxState.fifo_offset = 0;

    // init flow counters
    rxState.bytes_consumed = 0;
    rxState.bytes_produced = 0;
    rxState.last_produced = 0;
    rxState.last_consumed = 0;
    tx_vspa_proxy_wo->host_consumed_size[0] = 0;

    auto ddr_dst = vl_iqflood_ddr_addr + rxState.fifo_start_addr;
    memset(ddr_dst, 0, rxState.fifo_size); // clear RAM, not to confuse with old data from previous runs

    auto nv_app_stats = const_cast<t_stats*>(app_stats);
    port->sync_dmem_proxy_after_write(reinterpret_cast<uint8_t*>(nv_app_stats), sizeof(t_stats));
    return OpStatus::Success;
}

OpStatus VSPA_iqplayer::SetupTx(uint32_t fifo_start_offset, uint32_t fifo_size, double expectedTxDataRate)
{
    // const std::lock_guard<std::mutex> lock(mx);
    VSPA_FIFO_State& txState = mTx;
    assert(tx_vspa_proxy_ro);

    if (fifo_size == 0)
        return OpStatus::InvalidValue;

    // TODO: choose 1 or 2 based on system clock frequency
    // When 2 is used VSPA Tx DMA transfer migth get stuck in running state when system clock <30MHz
    // Needs power cycle to recover from that.
    if (expectedTxDataRate < 200e6)
        tx_dma_channel_count = 1;
    else
        tx_dma_channel_count = 2;

    // dccivac((uint32_t*)(tx_vspa_proxy_ro));
    auto nv_tx_vspa_proxy_ro = const_cast<t_tx_ch_host_proxy*>(tx_vspa_proxy_ro);
    port->sync_dmem_proxy_before_read(reinterpret_cast<uint8_t*>(nv_tx_vspa_proxy_ro), sizeof(t_tx_ch_host_proxy));

    /* check firmware is idle waiting for new data */
    //if (tx_vspa_proxy_ro->host_produced_size != tx_vspa_proxy_ro->la9310_fifo_enqueued_size) {
    //  printf("\n TX : modem is already running \n");
    //  fflush(stdout);
    //  return 0;
    //}

    //dccivac((uint32_t*)(tx_vspa_proxy_ro));

    // init fifo pointers
    txState.fifo_start_addr = fifo_start_offset;
    txState.fifo_size = fifo_size;
    txState.fifo_offset = 0;

    auto ddr_dst = vl_iqflood_ddr_addr + txState.fifo_start_addr;
    memset(ddr_dst, 0, txState.fifo_size);
    port->sync_iq_flood_after_write(ddr_dst, txState.fifo_size);

    // complex16_t* ps = reinterpret_cast<complex16_t*>(ddr_dst);
    // for (int i = 0; i < txState.fifo_size / sizeof(complex16_t); ++i)
    //     ps[i] = complex16_t(0, 32700 * (i & 1));

    // init flow control
    txState.bytes_consumed = 0;
    txState.bytes_produced = 0;

    tx_vspa_proxy_wo->host_produced_size = 0;
    app_stats->tx_stats[STAT_APP_DDR_RD] = 0;

    auto nv_app_stats = const_cast<t_stats*>(app_stats);
    port->sync_dmem_proxy_after_write(reinterpret_cast<uint8_t*>(nv_app_stats), sizeof(t_stats));

    return OpStatus::Success;
}

int32_t VSPA_iqplayer::Receive(uint32_t channel, uint32_t* destination, uint32_t read_size, uint64_t* timestamp)
{
    // const std::lock_guard<std::mutex> lock(mx);
    VSPA_FIFO_State& rxState = mRx[channel];

    port->wait_for_new_data();

    // dccivac((uint32_t*)(rx_vspa_proxy_ro));
    auto nv_rx_vspa_proxy_ro = const_cast<t_rx_ch_host_proxy*>(rx_vspa_proxy_ro);
    port->sync_dmem_proxy_before_read(reinterpret_cast<uint8_t*>(nv_rx_vspa_proxy_ro), sizeof(t_rx_ch_host_proxy));

    // Check new transfer
    const uint32_t dev_produced = rx_vspa_proxy_ro[channel].la9310_fifo_consumed_size;
    const uint32_t produceDiff = dev_produced - rxState.last_produced;
    printf_dbg_log("devp:%u diff:%u\n", dev_produced, produceDiff);
    rxState.last_produced = dev_produced;

    rxState.bytes_produced += produceDiff;
    uint32_t data_size = rxState.bytes_produced - rxState.bytes_consumed;
    printf_dbg_log("Receive - p:%i c:%i, sz:%i\n", rxState.bytes_produced, rxState.bytes_consumed, data_size);
    if (data_size >= rxState.fifo_size)
    {
        lime::error("VSPA RX overrun, (data_size=0x%08x app_RX_total_produced_size=0x%08lx app_RX_total_consumed_size=0x%08lx)\n",
            data_size,
            rxState.bytes_produced,
            rxState.bytes_consumed);

        rxState.bytes_consumed = rxState.bytes_produced;
        tx_vspa_proxy_wo->host_consumed_size[channel] = rxState.bytes_consumed;
        return 0;
    }

    const uint32_t contiguousBytesSize = rxState.fifo_size - rxState.fifo_offset;
    const uint32_t rx_ddr_step = tx_vspa_proxy_ro->rx_ddr_step;
    if (rx_ddr_step == 0)
    {
        lime::error("VSPA Rx ddr step = 0\n");
        return 0;
    }

    // TODO: bug in firmware, race condition? Sometimes Rx hangs with flow control after if host marks just the first rx_ddr_step as consumed
    // if (data_size < rx_ddr_step)
    if (data_size < rx_ddr_step * 2)
    {
        return 0; // no data available
    }

    if (data_size > contiguousBytesSize)
        data_size = contiguousBytesSize;
    if (data_size > read_size)
        data_size = read_size;

    if (timestamp)
    {
        constexpr int iqSampleSize = 4;
        *timestamp = rxState.bytes_consumed / iqSampleSize;
    }

    // ready to fetch new data
    rxState.bytes_consumed += data_size;
    app_stats->rx_stats[channel][STAT_APP_DDR_WR] = rxState.bytes_consumed / rx_ddr_step;

    auto nv_app_stats = const_cast<t_stats*>(app_stats);
    port->sync_dmem_proxy_after_write(reinterpret_cast<uint8_t*>(nv_app_stats), sizeof(t_stats));

    // xfer data
    volatile auto ddr_src = vl_iqflood_ddr_addr + rxState.fifo_start_addr + rxState.fifo_offset;
    port->sync_iq_flood_before_read(ddr_src, data_size);
    memcpy(destination, ddr_src, data_size);

    // update flow control
    tx_vspa_proxy_wo->host_consumed_size[channel] = rxState.bytes_consumed;

    rxState.fifo_offset += data_size;
    if (rxState.fifo_offset >= rxState.fifo_size)
        rxState.fifo_offset = 0;

    return data_size;
}

int32_t VSPA_iqplayer::Transmit(const void* src, uint32_t write_size, uint64_t timestamp)
{
    // const std::lock_guard<std::mutex> lock(mx);
    volatile VSPA_FIFO_State& txState = mTx;

    auto nv_tx_vspa_proxy_ro = const_cast<t_tx_ch_host_proxy*>(tx_vspa_proxy_ro);
    port->sync_dmem_proxy_before_read(reinterpret_cast<uint8_t*>(nv_tx_vspa_proxy_ro), sizeof(t_tx_ch_host_proxy));

    // Check new transfer opty
    txState.bytes_consumed = tx_vspa_proxy_ro->la9310_fifo_enqueued_size;
    const uint64_t fifo_filled_bytes = txState.bytes_produced - txState.bytes_consumed;
    // if (fifo_filled_bytes > txState.fifo_size)
    // {
    //     printf("!!!IQPlayer tx fifo overflow\n");
    // }

    const uint32_t contiguousBytesSize = txState.fifo_size - txState.fifo_offset;
    uint32_t empty_size = txState.fifo_size - fifo_filled_bytes;
    const uint32_t tx_ddr_step = tx_vspa_proxy_ro->tx_ddr_step;
    // if (empty_size < tx_ddr_step)
    //     return 0;

    if (empty_size > contiguousBytesSize)
        empty_size = contiguousBytesSize;

    if (write_size > empty_size)
        write_size = empty_size;

    if (fifo_filled_bytes > txState.fifo_size)
    {
        // printf("\n TX underrun , exit (busy=0x%08x txState.bytes_produced=0x%08x txState.bytes_consumed=0x%08x)\n",
        //     fifo_filled_bytes,
        //     txState.bytes_produced,
        //     txState.bytes_consumed);
    }

    if (write_size <= 0)
        return 0;

    // xfer data
    auto ddr_dst = vl_iqflood_ddr_addr + txState.fifo_start_addr + txState.fifo_offset;
    memcpy(ddr_dst, src, write_size);
    port->sync_iq_flood_after_write(ddr_dst, write_size);
    //flush_region(ddr_dst, empty_size);

    // ready to send new data
    txState.bytes_produced += write_size;
    tx_vspa_proxy_wo->host_produced_size = txState.bytes_produced;
    app_stats->tx_stats[STAT_APP_DDR_RD] = txState.bytes_produced / tx_ddr_step;
    // printf("ss %i %i\n", txState.bytes_produced, txState.bytes_produced / tx_ddr_step);

    auto nv_app_stats = const_cast<t_stats*>(app_stats);
    port->sync_dmem_proxy_after_write(reinterpret_cast<uint8_t*>(nv_app_stats), sizeof(t_stats));

    txState.fifo_offset += write_size;
    if (txState.fifo_offset >= txState.fifo_size)
        txState.fifo_offset = 0;

    return write_size;
}

OpStatus VSPA_iqplayer::ClearStats()
{
    // const std::lock_guard<std::mutex> lock(mx);
    printf_dbg_log("IQPlayer: clear stats\n");
    const mbox_opc_e command = MBOX_OPC_GET_STATS_COUNT;
    constexpr uint32_t reset_counter = (1 << 20);
    uint32_t counter_idx = 0;

    uint32_t hiword = command << 24 | reset_counter | (counter_idx & 0xFFFF);
    uint32_t loword = 0;
    uint64_t value = (uint64_t(hiword) << 32) | loword;

    app_stats->tx_stats[STAT_APP_DDR_RD] = 0;
    // tx_vspa_proxy_wo->host_produced_size = 0;

    return mailbox->Message(vspa_cpu_id, vspa_mbox_id, value);
}

OpStatus VSPA_iqplayer::SetDCOffset(complex16_t offset)
{
    printf_dbg_log("IQPlayer: SetDCOffset %i %i\n", offset.real(), offset.imag());
    const mbox_opc_e command = MBOX_OPC_RX_DCO_CORR;
    uint32_t hiword = command << 24;
    uint32_t loword = int32_t(offset.real()) << 16;
    loword |= offset.imag() & 0xFFFF;
    uint64_t value = (uint64_t(hiword) << 32) | loword;
    return mailbox->Message(vspa_cpu_id, vspa_mbox_id, value);
}

t_stats VSPA_iqplayer::GetStats()
{
    t_stats stats_snapshot;
    memset(&stats_snapshot, 0, sizeof(t_stats));
    tx_vspa_proxy_wo->gbl_stats_fetch = 1;

    // std::this_thread::sleep_for(std::chrono::milliseconds(1));

    volatile uint32_t* src = (reinterpret_cast<volatile uint32_t*>(&v_vspa_dmem_proxy_ro->vspa_stats));
    uint32_t* dest = reinterpret_cast<uint32_t*>(&stats_snapshot);
    for (size_t i = 0; i < sizeof(t_stats) / sizeof(uint32_t); ++i)
    {
        *dest += *src;
        ++dest;
        ++src;
    }

    src = (reinterpret_cast<volatile uint32_t*>(&v_vspa_dmem_proxy_ro->host_stats));
    dest = reinterpret_cast<uint32_t*>(&stats_snapshot);
    for (size_t i = 0; i < sizeof(t_stats) / sizeof(uint32_t); ++i)
    {
        *dest += *src;
        ++dest;
        ++src;
    }

    src = (reinterpret_cast<volatile uint32_t*>(&v_vspa_dmem_proxy_ro->app_stats));
    dest = reinterpret_cast<uint32_t*>(&stats_snapshot);
    for (size_t i = 0; i < sizeof(t_stats) / sizeof(uint32_t); ++i)
    {
        *dest += *src;
        ++dest;
        ++src;
    }
    return stats_snapshot;
}

class VSPA_DC_Offset : public IDCCorrector
{
  public:
    VSPA_DC_Offset(std::shared_ptr<VSPA_mailbox> mailbox, mbox_opc_e command)
        : mailbox(mailbox)
        , command(command)
    {
    }

    virtual ~VSPA_DC_Offset() {}

    OpStatus SetDCOffset(complex16_t offset) override
    {
        printf_dbg_log("IQPlayer: SetDCOffset %i %i\n", offset.real(), offset.imag());
        OpStatus status = SetDCI(offset.real());
        if (status != OpStatus::Success)
            return status;
        return SetDCQ(offset.imag());
    }

    OpStatus SetDCI(int16_t offset) override
    {
        printf_dbg_log("IQPlayer: SetDCOffsetI %i\n", offset);
        uint32_t iq_channel_id = 0;
        bool iq_tx_rx = command == MBOX_OPC_TX_DCO_CORR;
        bool iq_rst = false;

        uint32_t hiword = MBOX_OPC_IQ_CORR << 24;
        hiword |= (iq_channel_id & 0x3) << 16;
        hiword |= iq_tx_rx << 21;
        hiword |= iq_rst << 20;

        uint32_t loword = 0;
        float fval = float(offset / 32768.0);
        memcpy(&loword, &fval, sizeof(uint32_t));
        uint64_t value = (uint64_t(hiword | (MBOX_IQ_CORR_DC_I & 0xFFFF)) << 32) | loword;
        return mailbox->Message(vspa_cpu_id, vspa_mbox_id, value);
    }
    OpStatus SetDCQ(int16_t offset) override
    {
        printf_dbg_log("IQPlayer: SetDCOffsetQ %i\n", offset);
        uint32_t iq_channel_id = 0;
        bool iq_tx_rx = command == MBOX_OPC_TX_DCO_CORR;
        bool iq_rst = false;

        uint32_t hiword = MBOX_OPC_IQ_CORR << 24;
        hiword |= (iq_channel_id & 0x3) << 16;
        hiword |= iq_tx_rx << 21;
        hiword |= iq_rst << 20;

        uint32_t loword = 0;
        float fval = float(offset / 32768.0);
        memcpy(&loword, &fval, sizeof(uint32_t));
        uint64_t value = (uint64_t(hiword | (MBOX_IQ_CORR_DC_Q & 0xFFFF)) << 32) | loword;
        return mailbox->Message(vspa_cpu_id, vspa_mbox_id, value);
    }

    complex16_t GetDCOffset() override { return complex16_t(0, 0); }

  private:
    std::shared_ptr<VSPA_mailbox> mailbox;
    const mbox_opc_e command;
};

std::shared_ptr<IDCCorrector> VSPA_iqplayer::GetRxDCCorrector()
{
    return std::make_shared<VSPA_DC_Offset>(mailbox, MBOX_OPC_RX_DCO_CORR);
}

std::shared_ptr<IDCCorrector> VSPA_iqplayer::GetTxDCCorrector()
{
    return std::make_shared<VSPA_DC_Offset>(mailbox, MBOX_OPC_TX_DCO_CORR);
}

class VSPA_QEC : public IQuadratureErrorCorrector
{
  public:
    VSPA_QEC(std::shared_ptr<VSPA_mailbox> mailbox, lime::TRXDir dir)
        : mailbox(mailbox)
        , dir(dir)
    {
    }

    virtual ~VSPA_QEC() {}
    OpStatus SetImbalance(float iq_gain_imb, float phase_imb_deg)
    {
        float f1, f2, f4;
        if (dir == TRXDir::Rx)
        {
            float gamma = pow(10.0, (iq_gain_imb / 20.0));
            float theta_z = phase_imb_deg * (M_PI / 180.0);
            f1 = 1.0 / gamma;
            f2 = std::tan(theta_z) / gamma;
            f4 = 1.0 / std::cos(theta_z);
        }
        else
        {
            float alpha = pow(10.0, (iq_gain_imb / 20.0));
            float phi_z = phase_imb_deg * (M_PI / 180.0);

            f1 = 1.0 / std::cos(phi_z);
            f2 = -std::tan(phi_z) / alpha;
            f4 = 1.0 / alpha;
        }
        printf_dbg_log("IQPlayer: SetIQImbalance %f %f\nf1:%f f2:%f f4:%f\n", iq_gain_imb, phase_imb_deg, f1, f2, f4);

        const uint32_t iq_channel_id = 0;
        const bool iq_tx_rx = dir == TRXDir::Tx;
        const bool iq_rst = false;

        uint32_t hiword = MBOX_OPC_IQ_CORR << 24;
        hiword |= (iq_channel_id & 0x3) << 16;
        hiword |= iq_tx_rx << 21;
        hiword |= iq_rst << 20;

        uint32_t loword = 0;
        uint64_t value = 0;

        memcpy(&loword, &f2, sizeof(uint32_t));
        value = (uint64_t(hiword | (MBOX_IQ_CORR_FTAP1 & 0xFFFF)) << 32) | loword;
        OpStatus status = mailbox->Message(vspa_cpu_id, vspa_mbox_id, value);
        if (status != OpStatus::Success)
            return status;

        memcpy(&loword, &f1, sizeof(uint32_t));
        value = (uint64_t(hiword | (MBOX_IQ_CORR_FTAP2 & 0xFFFF)) << 32) | loword;
        mailbox->Message(vspa_cpu_id, vspa_mbox_id, value);
        if (status != OpStatus::Success)
            return status;

        memcpy(&loword, &f4, sizeof(uint32_t));
        value = (uint64_t(hiword | (MBOX_IQ_CORR_FTAP3 & 0xFFFF)) << 32) | loword;
        return mailbox->Message(vspa_cpu_id, vspa_mbox_id, value);
    }

    OpStatus SetPhaseCorrection(float phase_imb_deg) override { return OpStatus::NotImplemented; }

    OpStatus SetGainCorrection(float phase_imb_deg) override { return OpStatus::NotImplemented; }

  private:
    std::shared_ptr<VSPA_mailbox> mailbox;
    const lime::TRXDir dir;
};

std::shared_ptr<IQuadratureErrorCorrector> VSPA_iqplayer::GetRxQEC()
{
    return std::make_shared<VSPA_QEC>(mailbox, TRXDir::Rx);
}

std::shared_ptr<IQuadratureErrorCorrector> VSPA_iqplayer::GetTxQEC()
{
    return std::make_shared<VSPA_QEC>(mailbox, TRXDir::Tx);
}

bool VSPA_iqplayer::IsFirmwareLoaded() const
{
    constexpr uint32_t VSPA_CCSR_offset = 0x1000000;
    constexpr uint32_t vcpu_busy = (1 << 8);

    const volatile uint8_t* BAR0_addr = reinterpret_cast<const uint8_t*>(port->GetBar(LA9310_WINDOW_BAR0).vaddr);
    const auto VSPA_STATUS_reg = reinterpret_cast<const volatile uint32_t*>(BAR0_addr + VSPA_CCSR_offset + 0x10);
    return (*VSPA_STATUS_reg) & vcpu_busy;
}

OpStatus VSPA_iqplayer::ResetVCPU()
{
    OpStatus status;
    if (IsFirmwareLoaded())
    {
        printf_dbg_log("IQPlayer: Reset VCPU\n");
        const mbox_opc_e command = MBOX_OPC_DONE_SWRESET;
        uint32_t hiword = command << 24;
        uint32_t loword = 0;
        uint64_t value = (uint64_t(hiword) << 32) | loword;
        uint64_t result = 0;
        status = mailbox->Message(vspa_cpu_id, vspa_mbox_id, value, &result);
    }
    else
    {
        printf_dbg_log("IQPlayer: VCPU not running\n");
        status = OpStatus::Success;
    }
    return status;
}

} // namespace lime
