/* Copyright 2022-2024 NXP
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "vspa_mailbox.h"

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
    printf("done\n");
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
            printf("Received from VSPA:%d, MBox:%d, MSB:0x%08x, LSB:0x%08x.\n", core_idx, mbox_id, msb, lsb);
            return 0;
        }
        t2 = std::chrono::high_resolution_clock::now();
    }
    printf("%s: VCPU:%d MBox:%d is not responding!!\n", __func__, core_idx, mbox_id);
    return -1;
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
    *reinterpret_cast<uint32_t*>(addr) = value;
}

uint32_t VSPA_MailBox::ioread32(uint64_t addr)
{
    return *reinterpret_cast<uint32_t*>(addr);
}

} // namespace lime
