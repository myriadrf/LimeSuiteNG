#include "VSPA_iqplayer.h"

#include "limesuiteng/types.h"
#include "limesuiteng/Logger.h"

#include <assert.h>
#include <cstring>
#include <chrono>
#include <iostream>
#include <thread>

#include "interface/IDCCorrector.h"
#include "interface/IQuadratureErrorCorrector.h"

#include "streaming/samplesConversion.h"

#include "comms/PCIe/LA9310_PCIe.h"
#include "drivers/linux/la9310_limesdr/common_headers/la9310_host_if.h"

#include "cli/limeVSPA/vspa_state.h"

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

namespace lime {

static const uint32_t vspa_cpu_id = 0;
static const uint32_t vspa_mbox_id = 0;
static constexpr uint32_t dmem_proxy_reserved = 4096;

static const double refClk = 30.72e6;

static const std::array<e_rx_channel, 4> rx_api_channel_to_vspa_channel = { VSPA_RX0, VSPA_RX1, VSPA_RO0, VSPA_RO1 };
e_rx_channel VSPA_iqplayer::api_channel_remap(uint32_t index)
{
    return rx_api_channel_to_vspa_channel.at(index);
}

VSPA_iqplayer::VSPA_iqplayer(std::shared_ptr<LA9310_PCIe> port)
    : port(port)
    , mailbox(std::make_shared<VSPA_mailbox>(port))
    , tx_dma_channel_count(1)
    , rx_dma_channel_count(1)
{
    Initialize();
}

OpStatus VSPA_iqplayer::Initialize()
{
    if (!IsFirmwareLoaded())
    {
        lime::error("VSPA firmware not running?");
        return OpStatus::Error;
    }

    auto v_iqflood_ddr = port->GetBar(LA9310_WINDOW_IQFLOOD);
    auto dmem_proxy = v_iqflood_ddr; // by default DMEM_PROXY is now at start of IQFLOOD

    vl_iqflood_ddr_addr = reinterpret_cast<uint8_t*>(v_iqflood_ddr.vaddr);
    iqflood_size = v_iqflood_ddr.size;

    vspa_dmem_proxy_ro = reinterpret_cast<const volatile vspa_state_t*>(dmem_proxy.vaddr);
    port->dmem_sync_to_cpu(vspa_dmem_proxy_ro, sizeof(vspa_state_t));

    mTx.fifo_start_addr = vspa_dmem_proxy_ro->info.tx_config.ddr_base_address - LA9310_IQFLOOD_PHYS_ADDR;
    mTx.fifo_size = vspa_dmem_proxy_ro->info.tx_config.ddr_size;
    mTx.fifo_offset = 0;

    uint8_t* BAR2_addr = reinterpret_cast<uint8_t*>(port->GetBar(LA9310_WINDOW_BAR2).vaddr);
    vspa_dmem_proxy_wo = reinterpret_cast<volatile vspa_state_t*>(BAR2_addr + 0x400000 + vspa_dmem_proxy_ro->info.dmemProxyOffset);

    printf_dbg_log("VSPA_iqplayer: IQFLOOD size: %lu, Rx channels %u\n", v_iqflood_ddr.size, vspa_dmem_proxy_ro->info.rx_num_chan);
    port->dmem_sync_to_device(vspa_dmem_proxy_ro, sizeof(vspa_state_t));

    return OpStatus::Success;
}

OpStatus VSPA_iqplayer::EnableRxChannels(uint32_t channel_mask)
{
    // const std::lock_guard<std::mutex> lock(mx);
    printf_dbg_log("IQPlayer: Enable ADC channels 0x%x\n", channel_mask);
    const uint64_t value = (uint64_t(MBOX_OPC_RX_CHAN_SELECT) << (56)) | channel_mask;
    return mailbox->Message(vspa_cpu_id, vspa_mbox_id, value);
}

OpStatus VSPA_iqplayer::TxEnable(bool enable, bool flow_control_disable)
{
    const mbox_opc_e command = MBOX_OPC_TX_CONTROL;
    const bool ddr_enable = enable;
    const bool test_load_start = false;

    uint32_t loword = 0;
    uint32_t hiword = 0;
    hiword |= command << 24;
    // hiword |= host_flow_control_disable ? 0x00400000 : 0;
    // hiword |= test_load_start ? 0x00200000 : 0;
    hiword |= ddr_enable ? (1 << 0) : 0;
    // hiword |= ddr_rd_dma_mBurst ? 0x00080000 : 0;
    // hiword |= (ddr_rd_dma_ch_nb << 16) & 0x00070000;
    // hiword |= chunkCount4k & 0x0000FFFF;

    uint64_t value = uint64_t(hiword) << 32 | loword;
    printf_dbg_log("IQPlayer: TxControl enable:%i, disable_flow_control:%i bytes_preloaded:%li\n",
        enable,
        flow_control_disable,
        mTx.bytes_produced);
    uint64_t response = 0;
    OpStatus status = mailbox->Message(vspa_cpu_id, vspa_mbox_id, value, &response);
    if (status != OpStatus::Success)
        return status;
    return (response & 0xFF) == 0 ? OpStatus::Success : OpStatus::Error;
}

OpStatus VSPA_iqplayer::GenerateTxTone(bool enabled, int fftBin)
{
    const mbox_opc_e command = MBOX_OPC_SINGLE_TONE_TX;
    const bool start = enabled;

    // stop if it was already active
    OpStatus status = mailbox->Message(vspa_cpu_id, vspa_mbox_id, uint64_t(command) << 56);
    if (status != OpStatus::Success)
        return status;
    printf_dbg_log("IQPlayer: GenerateTxTone %i\n", enabled);

    uint32_t loword = fftBin & 0xFFFF; // generated frequency bin
    uint32_t hiword = 0;
    hiword |= command << 24;
    hiword |= start ? 0x00100000 : 0;

    uint64_t value = uint64_t(hiword) << 32 | loword;
    return mailbox->Message(vspa_cpu_id, vspa_mbox_id, value);
}

OpStatus VSPA_iqplayer::RxEnable(uint8_t channel, bool enable, bool reset_pipeline)
{
    // const std::lock_guard<std::mutex> lock(mx);
    assert(channel < 4);

    VSPA_FIFO_State& rxState = mRx[channel];
    rxState.bytes_consumed = 0;
    rxState.bytes_produced = 0;
    rxState.last_produced = 0;
    rxState.last_consumed = 0;
    vspa_dmem_proxy_wo->data_flow.rx[channel].consumed = 0;

    const mbox_opc_e command = MBOX_OPC_RX_CONTROL;

    uint32_t hiword = uint32_t(command) << 24;
    hiword |= (channel & 0x3) << 20;
    hiword |= enable ? (1 << 0) : 0;
    hiword |= reset_pipeline ? (1 << 1) : 0;
    uint32_t loword = 0;

    uint64_t value = uint64_t(hiword) << 32 | loword;

    printf_dbg_log("IQPlayer: RxControl[%i] 0x%08X_%08X\n", channel, hiword, loword);
    return mailbox->Message(vspa_cpu_id, vspa_mbox_id, value);
}

OpStatus VSPA_iqplayer::SetupResources(uint32_t rxMask, uint32_t txCount)
{
    OpStatus status = OpStatus::Success;
    uint32_t mem_offset = dmem_proxy_reserved;

    // assert((proxyalloc + txalloc + rxalloc) < 4 * 1024 * 1024);
    if (txCount)
    {
        size_t txalloc = 1024 * 512;
        status = SetupTx(mem_offset, txalloc);
        if (status != OpStatus::Success)
            return status;
        mem_offset += txalloc;
    }
    if (rxMask)
    {
        size_t rxalloc = 1024 * 512;
        for (int i = 0; i < 4; ++i)
        {
            if (!(rxMask & (1 << i)))
                continue;

            status = SetupRx(i, mem_offset, rxalloc);
            if (status != OpStatus::Success)
                return status;
            mem_offset += rxalloc;
        }
    }
    return status;
}

OpStatus VSPA_iqplayer::SetupRx(uint32_t channel, uint32_t fifo_start_offset, uint32_t fifo_size)
{
    // const std::lock_guard<std::mutex> lock(mx);
    VSPA_FIFO_State& rxState = mRx[channel];

    if (fifo_size == 0)
        return OpStatus::InvalidValue;

    // TODO: choose 1 or 2 based on system clock frequency
    // When 2 is used VSPA Rx DMA transfer migth get stuck in running state when system clock <30MHz
    // Needs power cycle to recover from that.
    // if (expectedDataRate < 200e6)
    rx_dma_channel_count = 1;
    // else
    //     rx_dma_channel_count = 2;

    rxState.fifo_start_addr = fifo_start_offset;
    rxState.fifo_size = fifo_size;
    rxState.fifo_offset = 0;

    // init flow counters
    rxState.bytes_consumed = 0;
    rxState.bytes_produced = 0;
    rxState.last_produced = 0;
    rxState.last_consumed = 0;
    vspa_dmem_proxy_wo->data_flow.rx[channel].consumed = 0;

    const mbox_opc_e command = MBOX_OPC_RX_HOST_FIFO_CONFIG;
    assert(fifo_size / 4096 < 0x10000);
    assert(fifo_size / 4096 > 0);
    if (fifo_size == 0)
        return OpStatus::InvalidValue;

    const uint16_t chunkCount4k = rxState.fifo_size / 4096;

    uint32_t loword = LA9310_IQFLOOD_PHYS_ADDR + rxState.fifo_start_addr;
    uint32_t hiword = 0;
    hiword |= command << 24;
    hiword |= (channel & 0x3) << 20;
    hiword |= chunkCount4k & 0x0000FFFF;

    uint64_t value = uint64_t(hiword) << 32 | loword;
    printf_dbg_log("IQPlayer: Setup FIFO Rx[%i], fifo_size:%u, fifo_base:%08X\n", channel, fifo_size, loword);
    uint64_t response = 0;
    OpStatus status = mailbox->Message(vspa_cpu_id, vspa_mbox_id, value, &response);
    if (status != OpStatus::Success)
        return status;
    return (response & 0xFF) == 0 ? OpStatus::Success : OpStatus::Error;
}

OpStatus VSPA_iqplayer::SetupTx(uint32_t fifo_start_offset, uint32_t fifo_size)
{
    // const std::lock_guard<std::mutex> lock(mx);
    VSPA_FIFO_State& txState = mTx;

    if (fifo_size == 0)
        return OpStatus::InvalidValue;

    // TODO: choose 1 or 2 based on system clock frequency
    // When 2 is used VSPA Tx DMA transfer migth get stuck in running state when system clock <30MHz
    // Needs power cycle to recover from that.
    // if (expectedTxDataRate < 200e6)
    tx_dma_channel_count = 1;
    // else
    //     tx_dma_channel_count = 2;

    // init fifo pointers
    txState.fifo_start_addr = fifo_start_offset;
    txState.fifo_size = fifo_size;
    txState.fifo_offset = 0;

    // init flow control
    txState.bytes_consumed = 0;
    txState.bytes_produced = 0;

    vspa_dmem_proxy_wo->data_flow.tx.produced = 0;
    vspa_dmem_proxy_wo->data_flow.tx.consumed = 0;

    const mbox_opc_e command = MBOX_OPC_TX_HOST_FIFO_CONFIG;

    assert(fifo_size / 4096 < 0x10000);
    assert(fifo_size / 4096 > 0);
    if (fifo_size == 0)
        return OpStatus::InvalidValue;

    const uint16_t chunkCount4k = fifo_size / 4096;

    uint32_t loword = LA9310_IQFLOOD_PHYS_ADDR + mTx.fifo_start_addr;
    uint32_t hiword = command << 24;
    // hiword |= host_flow_control_disable ? 0x00400000 : 0;
    // hiword |= test_load_start ? 0x00200000 : 0;
    // hiword |= start ? 0x00100000 : 0;
    // hiword |= ddr_rd_dma_mBurst ? 0x00080000 : 0;
    // hiword |= (ddr_rd_dma_ch_nb << 16) & 0x00070000;
    hiword |= chunkCount4k & 0x0000FFFF;

    uint64_t value = uint64_t(hiword) << 32 | loword;
    printf_dbg_log(
        "IQPlayer: Setup FIFO Tx, fifo_size:%u, fifo_base:%08X bytes_preloaded:%li\n", fifo_size, loword, mTx.bytes_produced);
    uint64_t response = 0;
    OpStatus status = mailbox->Message(vspa_cpu_id, vspa_mbox_id, value, &response);
    if (status != OpStatus::Success)
        return status;
    return (response & 0xFF) == 0 ? OpStatus::Success : OpStatus::Error;
}

int32_t VSPA_iqplayer::Receive(uint32_t channel, uint32_t* destination, uint32_t read_size, uint64_t* timestamp)
{
    const auto timeout = std::chrono::milliseconds(1000);
    VSPA_FIFO_State& rxState = mRx[channel];

    port->dmem_sync_to_cpu(&vspa_dmem_proxy_ro->data_flow, sizeof(vspa_flow_control));

    // Check new transfer
    uint32_t dev_produced = vspa_dmem_proxy_ro->data_flow.rx[channel].produced;
    uint32_t produceDiff = dev_produced - rxState.last_produced;

    auto t1 = std::chrono::high_resolution_clock::now();
    auto t2 = t1;
    uint32_t data_size = 0;
    do
    {
        port->dmem_sync_to_cpu(&vspa_dmem_proxy_ro->data_flow, sizeof(vspa_flow_control));
        dev_produced = vspa_dmem_proxy_ro->data_flow.rx[channel].produced;
        produceDiff = dev_produced - rxState.last_produced;
        port->dmem_sync_to_device(&vspa_dmem_proxy_ro->data_flow, sizeof(vspa_flow_control));

        rxState.last_produced = dev_produced;
        rxState.bytes_produced += produceDiff;
        data_size = rxState.bytes_produced - rxState.bytes_consumed;
        if (data_size >= rxState.fifo_size)
        {
            // lime::error("VSPA RX overrun, (data_size=0x%08x app_RX_total_produced_size=0x%08lx app_RX_total_consumed_size=0x%08lx)\n",
            //     data_size,
            //     rxState.bytes_produced,
            //     rxState.bytes_consumed);

            rxState.bytes_consumed = rxState.bytes_produced;
            vspa_dmem_proxy_wo->data_flow.rx[channel].consumed = rxState.bytes_consumed;
            return 0;
        }
        if (data_size == 0)
        {
            auto st = port->wait_for_new_data(timeout.count());
            if (st != OpStatus::Success)
                return 0;
        }
        t2 = std::chrono::high_resolution_clock::now();
    } while (data_size == 0 && (t2 - t1) < timeout);

    if (data_size == 0)
    {
        // printf("No data\n");
        return 0;
    }

    const uint32_t contiguousBytesSize = rxState.fifo_size - rxState.fifo_offset;
    const uint32_t rx_ddr_step = vspa_dmem_proxy_ro->info.rx_config[channel].ddr_step;
    if (rx_ddr_step == 0)
    {
        lime::error("VSPA Rx ddr step = 0\n");
        return 0;
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

    // xfer data
    volatile auto ddr_src = vl_iqflood_ddr_addr + rxState.fifo_start_addr + rxState.fifo_offset;
    port->dmem_sync_to_cpu(ddr_src, data_size);
    memcpy(destination, ddr_src, data_size);
    port->dmem_sync_to_device(ddr_src, data_size);

    // update flow control
    vspa_dmem_proxy_wo->data_flow.rx[channel].consumed = rxState.bytes_consumed;

    rxState.fifo_offset += data_size;
    if (rxState.fifo_offset >= rxState.fifo_size)
        rxState.fifo_offset = 0;

    return data_size;
}

int32_t VSPA_iqplayer::Transmit(const void* src, uint32_t write_size, uint64_t timestamp)
{
    // const std::lock_guard<std::mutex> lock(mx);
    volatile VSPA_FIFO_State& txState = mTx;

    port->dmem_sync_to_cpu(&vspa_dmem_proxy_ro->data_flow, sizeof(vspa_flow_control));

    txState.bytes_consumed = vspa_dmem_proxy_ro->data_flow.tx.consumed;
    const uint64_t fifo_filled_bytes = txState.bytes_produced - txState.bytes_consumed;
    // if (fifo_filled_bytes > txState.fifo_size)
    // {
    //     printf("!!!IQPlayer tx fifo overflow\n");
    // }

    const uint32_t contiguousBytesSize = txState.fifo_size - txState.fifo_offset;
    uint32_t empty_size = txState.fifo_size - fifo_filled_bytes;
    const uint32_t tx_ddr_step = vspa_dmem_proxy_ro->info.tx_config.ddr_step;
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
    port->dmem_sync_to_cpu(ddr_dst, write_size);
    memcpy(ddr_dst, src, write_size);
    port->dmem_sync_to_device(ddr_dst, write_size);

    // ready to send new data
    txState.bytes_produced += write_size;
    vspa_dmem_proxy_wo->data_flow.tx.produced = txState.bytes_produced;
    // printf("ss %i %i\n", txState.bytes_produced, txState.bytes_produced / tx_ddr_step);

    txState.fifo_offset += write_size;
    if (txState.fifo_offset >= txState.fifo_size)
        txState.fifo_offset = 0;

    return write_size;
}

size_t VSPA_iqplayer::TxDataEmplace(void* src, size_t write_size)
{
    volatile VSPA_FIFO_State& txState = mTx;

    port->dmem_sync_to_cpu(&vspa_dmem_proxy_ro->data_flow, sizeof(vspa_flow_control));
    txState.bytes_consumed = vspa_dmem_proxy_ro->data_flow.tx.consumed;
    const uint64_t fifo_filled_bytes = txState.bytes_produced - txState.bytes_consumed;
    // if (fifo_filled_bytes > txState.fifo_size)
    // {
    //     printf("!!!IQPlayer tx fifo overflow\n");
    // }

    const uint32_t contiguousBytesSize = txState.fifo_size - txState.fifo_offset;
    uint32_t empty_size = txState.fifo_size - fifo_filled_bytes;

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
    port->dmem_sync_to_cpu(ddr_dst, write_size);
    memcpy(ddr_dst, src, write_size);
    port->dmem_sync_to_device(ddr_dst, write_size);

    // ready to send new data
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

    vspa_dmem_proxy_wo->data_flow.tx.produced = 0;
    return OpStatus::Success;
    //return mailbox->Message(vspa_cpu_id, vspa_mbox_id, value);
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

class VSPA_DC_Offset : public IDCCorrector
{
  public:
    VSPA_DC_Offset(std::shared_ptr<VSPA_mailbox> mailbox, mbox_opc_e command)
        : mailbox(mailbox)
        , command(command)
    {
    }

    OpStatus Enabled(bool enable) { return enable ? OpStatus::Success : OpStatus::NotSupported; }

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

    lime::Range<float> GetRange() override { return lime::Range<float>(-16384, 16383, 1.0); }

    IDCCorrector::Type GetType() const { return IDCCorrector::Type::Digital; }

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
    OpStatus SetImbalance(float iq_gain_imb, float phase_imb_deg) override
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

    lime::Range<float> GetGainRange() override { return lime::Range(-3.0f, 3.0f, 1.0f / 512); }

    lime::Range<float> GetPhaseRange() override { return lime::Range(-22.5f, 22.5f, 45.0f / 1024); }

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
    // Check RCW Completion bit, without it accessing VSPA registers will fail and reboot host
    const volatile uint8_t* BAR0_addr = reinterpret_cast<const uint8_t*>(port->GetBar(LA9310_WINDOW_BAR0).vaddr);
    static const size_t RCW_COMPLETIONR_OFFSET = 0x104;
    static const size_t RESET_SECTION = 0x1E60000;
    static const size_t RCW_COMPLETION_DONE = 0x1;
    auto rcw_completion_addr = reinterpret_cast<const volatile uint32_t*>(BAR0_addr + RESET_SECTION + RCW_COMPLETIONR_OFFSET);
    uint32_t ulRcwCompletion = *rcw_completion_addr;
    if (!(ulRcwCompletion & RCW_COMPLETION_DONE))
    {
        lime::warning("LA9310 on PCIe reference clock, VSPA not powered yet.\n");
        return false;
    }

    constexpr uint32_t VSPA_CCSR_offset = 0x1000000;
    constexpr uint32_t vcpu_busy = (1 << 8);

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

int VSPA_iqplayer::GetDecimation(uint32_t channel) const
{
    if (!vspa_dmem_proxy_ro)
        return 1;

    return vspa_dmem_proxy_ro->info.rx_config[channel].oversample;
}

OpStatus VSPA_iqplayer::SetDecimation(uint32_t channel, uint32_t decimation)
{
    const mbox_opc_e command = MBOX_OPC_RX_CONFIGURE;
    uint32_t hiword = command << 24;
    hiword |= (channel & 0x3) << 20;
    uint32_t loword = uint32_t(std::log2(decimation)) & 0x3;
    uint64_t value = uint64_t(hiword) << 32 | loword;
    uint64_t response = 1;
    if (mailbox->Message(vspa_cpu_id, vspa_mbox_id, value, &response) != OpStatus::Success)
        return OpStatus::Error;
    return (response & 0xFF) == 0 ? OpStatus::Success : OpStatus::Error;
}

OpStatus VSPA_iqplayer::SetInterpolation(uint32_t interpolation)
{
    const mbox_opc_e command = MBOX_OPC_TX_CONFIGURE;
    uint32_t hiword = command << 24;
    uint32_t loword = uint32_t(std::log2(interpolation)) & 0x3;

    uint64_t value = uint64_t(hiword) << 32 | loword;
    uint64_t response = 1;
    if (mailbox->Message(vspa_cpu_id, vspa_mbox_id, value, &response) != OpStatus::Success)
        return OpStatus::Error;
    return (response & 0xFF) == 0 ? OpStatus::Success : OpStatus::Error;
}

int VSPA_iqplayer::GetInterpolation() const
{
    return vspa_dmem_proxy_ro->info.tx_config.oversample;
}

OpStatus VSPA_iqplayer::PrepareRx()
{
    const mbox_opc_e command = MBOX_OPC_RX_PREPARE;
    uint32_t hiword = command << 24;
    uint64_t value = uint64_t(hiword) << 32;
    uint64_t response = 1;
    if (mailbox->Message(vspa_cpu_id, vspa_mbox_id, value, &response) != OpStatus::Success)
        return OpStatus::Error;
    return (response & 0xFF) == 0 ? OpStatus::Success : OpStatus::Error;
}

} // namespace lime
