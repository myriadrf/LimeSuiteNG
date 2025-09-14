#include "VSPA_iqplayer.h"

#include <assert.h>

#include "comms/PCIe/LA9310_PCIe.h"

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
}

OpStatus VSPA_iqplayer::SelectRxChannel(uint32_t rx_channel_index)
{
    printf_dbg_log("IQPlayer: select ADC channel %i\n", rx_channel_index);
    uint64_t value = (uint64_t(MBOX_OPC_RX_CHAN_SELECT) << (24 + 32)) | rx_channel_index;
    mailbox.Send(vspa_cpu_id, vspa_mbox_id, value);
    return mailbox.Receive(vspa_cpu_id, vspa_mbox_id);
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

} // namespace lime
