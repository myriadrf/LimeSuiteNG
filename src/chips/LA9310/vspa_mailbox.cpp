/* Copyright 2022-2024 NXP
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "vspa_mailbox.h"

#include <assert.h>

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

VSPA_MailBox::VSPA_MailBox(std::shared_ptr<ShivaPCIE_lime> port)
    : port(port)
{
    vspa_addr = reinterpret_cast<uint64_t>(port->GetBar(LA9310_WINDOW_BAR0).vaddr) + 0x1000000;
}

VSPA_MailBox::~VSPA_MailBox()
{
}

inline uint64_t MAILBOX_ADDR(uint64_t base, size_t mbox, size_t direction, size_t core_idx)
{
    return (base + 0x680 + direction * 0x10 + mbox * 8 + 0x4000 * core_idx);
}

int VSPA_MailBox::Send(uint32_t core_idx, uint32_t mbox_id, uint64_t value)
{
    uint64_t addr = MAILBOX_ADDR(vspa_addr, mbox_id, 0, core_idx);
    uint32_t msb = value >> 32;
    uint32_t lsb = value;

    printf("%s: VSPA[%d] addr 0x%lX  val 0x%lX.\n", __func__, core_idx, addr, value);
    iowrite32(msb, addr);
    iowrite32(lsb, (addr + 4));
    return 0;
}

int VSPA_MailBox::Receive(uint32_t core_idx, uint32_t mbox_id, uint64_t* value)
{
    uint64_t addr = vspa_addr + 0x660 + 0x4000 * core_idx;
    uint32_t msb, lsb;

    auto t1 = std::chrono::high_resolution_clock::now();
    auto t2 = std::chrono::high_resolution_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1) < std::chrono::milliseconds(1000))
    {
        if (ioread32(addr) & (1 << (mbox_id /*+ 2*/)))
        {
            addr = MAILBOX_ADDR(vspa_addr, mbox_id, 1, core_idx);
            msb = ioread32(addr);
            lsb = ioread32((addr + 4));
            if (value)
            {
                *value = msb;
                *value <<= 32;
                *value |= lsb;
            }
            t2 = std::chrono::high_resolution_clock::now();
            int duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
            printf("Received from VSPA:%d, MBox:%d, MSB:0x%08x, LSB:0x%08x. t=%ius\n", core_idx, mbox_id, msb, lsb, duration);
            return 0;
        }
        t2 = std::chrono::high_resolution_clock::now();
    }
    printf("%s: VCPU:%d MBox:%d is not responding!!\n", __func__, core_idx, mbox_id);
    return -1;
}

int VSPA_MailBox::SelectRxChannel(uint32_t rx_channel_index)
{
    uint64_t value = (uint64_t(MBOX_OPC_RX_CHAN_SELECT) << (24 + 32)) | rx_channel_index;
    Send(0, 0, value);
    return Receive(0, 0);
}

int VSPA_MailBox::StartRx(uint32_t fifo_size, uint32_t fifo_base_la9310_phys_addr)
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

    Send(0, 0, value);
    return 0;
    // For some reason the stream starts, but there is no response from mail box
    // skip waiting for result, otherwise Rx will overrun
    // return Receive(0, 0);
}

int VSPA_MailBox::StopRx()
{
    uint64_t value = uint64_t(MBOX_OPC_IQ_MOD_RX) << (24 + 32);
    Send(0, 0, value);
    return Receive(0, 0);
}

int VSPA_MailBox::StopTx()
{
    uint64_t value = uint64_t(MBOX_OPC_IQ_MOD_TX) << (24 + 32);
    Send(0, 0, value);
    return Receive(0, 0);
}

void VSPA_MailBox::Clear(uint32_t core_idx, uint32_t mbox_id)
{
    if (mbox_id == 0)
        iowrite32(1 << 14, (vspa_addr + 0x10 + 0x4000 * core_idx));
    else
        iowrite32(1 << 15, (vspa_addr + 0x10 + 0x4000 * core_idx));
}

void VSPA_MailBox::iowrite32(uint32_t value, uint64_t addr)
{
    *reinterpret_cast<volatile uint32_t*>(addr) = value;
}

uint32_t VSPA_MailBox::ioread32(uint64_t addr)
{
    return *reinterpret_cast<volatile uint32_t*>(addr);
}

} // namespace lime
