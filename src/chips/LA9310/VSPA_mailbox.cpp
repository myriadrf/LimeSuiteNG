#include "VSPA_mailbox.h"

#include <assert.h>

#include "comms/PCIe/LA9310_PCIe.h"

#if 0 // print mailbox messages
    #define printf_mailbox_log(...) \
        do \
        { \
            printf(__VA_ARGS__); \
        } while (0)
#else
    #define printf_mailbox_log(format, ...)
#endif

namespace lime {

enum VSPA_MBox_direction { Send = 0, Receive = 1 };

static inline void iowrite32(uint32_t value, uint64_t addr)
{
    *reinterpret_cast<volatile uint32_t*>(addr) = value;
}

static inline uint32_t ioread32(uint64_t addr)
{
    return *reinterpret_cast<volatile uint32_t*>(addr);
}

VSPA_mailbox::VSPA_mailbox(std::shared_ptr<LA9310_PCIe> port)
    : port(port)
{
    const uint32_t bar0_vspa_offset = 0x1000000;
    vspa_ccsr_base = reinterpret_cast<uint64_t>(port->GetBar(LA9310_WINDOW_BAR0).vaddr) + bar0_vspa_offset;
}

inline static uint64_t GetMailboxOffset(size_t mbox, VSPA_MBox_direction direction, size_t core_idx)
{
    return (0x680 + direction * 0x10 + mbox * 8 + 0x4000 * core_idx);
}

void VSPA_mailbox::Send(uint32_t core_idx, uint32_t mbox_id, uint64_t value)
{
    const uint64_t addr = vspa_ccsr_base + GetMailboxOffset(mbox_id, VSPA_MBox_direction::Send, core_idx);
    const uint32_t msb = value >> 32;
    const uint32_t lsb = value;

    printf_mailbox_log("Send VSPA[%d] MBox:%d, value: 0x%08X_%08X.\n", core_idx, mbox_id, msb, lsb);
    iowrite32(msb, addr);
    iowrite32(lsb, addr + 4);
}

OpStatus VSPA_mailbox::Receive(uint32_t core_idx, uint32_t mbox_id, uint64_t* value)
{
    const std::chrono::milliseconds timeout{ 1000 };
    const uint64_t status_addr = vspa_ccsr_base + 0x660 + 0x4000 * core_idx;

    auto t1 = std::chrono::high_resolution_clock::now();
    auto t2 = t1;
    do
    {
        if (ioread32(status_addr) & (1 << (mbox_id /*+ 2*/)))
        {
            const uint64_t addr = vspa_ccsr_base + GetMailboxOffset(mbox_id, VSPA_MBox_direction::Receive, core_idx);
            const uint32_t msb = ioread32(addr);
            const uint32_t lsb = ioread32((addr + 4));
            if (value)
            {
                *value = uint64_t(msb) << 32;
                *value |= lsb;
            }
            t2 = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
            printf_mailbox_log(
                "Recv VSPA[%d] MBox:%d, value: 0x%08x_%08x. t=%lius\n", core_idx, mbox_id, msb, lsb, duration.count());

            Clear(core_idx, mbox_id);
            return OpStatus::Success;
        }
        t2 = std::chrono::high_resolution_clock::now();
    } while (std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1) < std::chrono::milliseconds(1000));
    printf("\n\t!Timeout VSPA[%d] MBox:%d is not responding!\n", core_idx, mbox_id);

    return OpStatus::Timeout;
}

void VSPA_mailbox::Clear(uint32_t core_idx, uint32_t mbox_id)
{
    // printf_mailbox_log("Clear VSPA[%d] MBox:%d\n", core_idx, mbox_id);
    if (mbox_id == 0)
        iowrite32(1 << 14, (vspa_ccsr_base + 0x10 + 0x4000 * core_idx));
    else
        iowrite32(1 << 15, (vspa_ccsr_base + 0x10 + 0x4000 * core_idx));
}

} // namespace lime
