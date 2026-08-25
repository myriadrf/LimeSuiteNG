#include "LA9310_FW_Impl.h"

#include <assert.h>
#include <string.h>

#include "m4_commands.h"

#include "comms/PCIe/LA9310_PCIe.h"
#include "chips/LA9310/virq.h"
#include "chips/LA9310/firmware/m4_memorymap.h"
#include "chips/LA9310/firmware/IQStreamer_DMA.h"
#include "drivers/linux/la9310_limesdr/common_headers/la9310_host_if.h"

#include "limesuiteng/Logger.h"

using namespace std;

#define LA9310_EP_HIF_OFFSET 0x1B000

namespace lime {

LA9310_FW_Impl::LA9310_FW_Impl(std::shared_ptr<LA9310_PCIe> pcie)
    : LA9310(pcie)
    , pcie(pcie)
{
    const auto bar1 = pcie->GetBar(LA9310_WINDOW_BAR1);
    hif = reinterpret_cast<volatile struct la9310_hif*>(size_t(bar1.vaddr) + LA9310_EP_HIF_OFFSET);
    assert(hif);
}

OpStatus LA9310_FW_Impl::WaitForResponse()
{
    OpStatus status = pcie->WaitSIRQ(LA9310_VIRQ::HOST_COMMAND_DONE, chrono::milliseconds(500));
    if (status != OpStatus::Success)
    {
        lime::error("LA9310_PCIe: RunControlCommand IRQ timeout\n");
        return status;
    }
    status = pcie->ClearSIRQ((1 << LA9310_VIRQ::HOST_COMMAND_DONE));
    if (status != OpStatus::Success)
    {
        lime::error("LA9310_PCIe: RunControlCommand failed clear IRQ\n");
        return status;
    }

    auto t1 = chrono::high_resolution_clock::now();
    while (hif->sw_cmd_desc.status == LA9310_SW_CMD_STATUS_POSTED || hif->sw_cmd_desc.status == LA9310_SW_CMD_STATUS_IN_PROGRESS)
    {
        auto t2 = chrono::high_resolution_clock::now();
        if (chrono::duration_cast<chrono::milliseconds>(t2 - t1) > chrono::milliseconds(1000))
        {
            lime::error("LA9310 M4 timeout\n");
            return OpStatus::Timeout;
        }
    }

    if (hif->sw_cmd_desc.status == LA9310_SW_CMD_STATUS_DONE)
    {
        int32_t status = hif->sw_cmd_desc.data[0];
        return status == int32_t(OpStatus::Success) ? OpStatus::Success : OpStatus::Error;
    }
    return OpStatus::Error;
}

bool LA9310_FW_Impl::CheckFirmwareAlive()
{
    lime::info("LA9310 CheckFirmwareAlive\n");
    uint32_t pattern = 0x55aa55aa;

    hif->sw_cmd_desc.cmd = LIME_M4_HEARTHBEAT;
    hif->sw_cmd_desc.data[0] = pattern;
    hif->sw_cmd_desc.status = LA9310_SW_CMD_STATUS_POSTED;

    auto t1 = chrono::high_resolution_clock::now();
    while (hif->sw_cmd_desc.status == LA9310_SW_CMD_STATUS_POSTED || hif->sw_cmd_desc.status == LA9310_SW_CMD_STATUS_IN_PROGRESS)
    {
        auto t2 = chrono::high_resolution_clock::now();
        if (chrono::duration_cast<chrono::milliseconds>(t2 - t1) > chrono::milliseconds(1000))
        {
            lime::error("LA9310 CheckFirmwareAlive timeout\n");
            return false;
        }
    }

    if (hif->sw_cmd_desc.status == LA9310_SW_CMD_STATUS_DONE)
        return hif->sw_cmd_desc.data[0] == ~pattern;
    return false;
}

OpStatus LA9310_FW_Impl::LoadFirmware(std::span<const char, std::dynamic_extent> firmware)
{
    return pcie->LoadArmM4Firmware(firmware.data(), firmware.size());
}

OpStatus LA9310_FW_Impl::EnterFirmwareReloadMode()
{
    lime::info("LA9310 EnterFirmwareReloadMode\n");
    hif->sw_cmd_desc.cmd = LIME_M4_BOOTLOADER;
    hif->sw_cmd_desc.status = LA9310_SW_CMD_STATUS_POSTED;

    auto t1 = chrono::high_resolution_clock::now();
    while (hif->sw_cmd_desc.status == LA9310_SW_CMD_STATUS_POSTED || hif->sw_cmd_desc.status == LA9310_SW_CMD_STATUS_IN_PROGRESS)
    {
        auto t2 = chrono::high_resolution_clock::now();
        if (chrono::duration_cast<chrono::milliseconds>(t2 - t1) > chrono::milliseconds(1000))
        {
            lime::error("LA9310 M4 timeout\n");
            return OpStatus::Timeout;
        }
    }

    if (hif->sw_cmd_desc.status == LA9310_SW_CMD_STATUS_DONE)
    {
        return OpStatus::Success;
    }
    return OpStatus::Error;
}

OpStatus LA9310_FW_Impl::SetSystemClock(double clk_hz, uint8_t adc_rate_mask, uint8_t dac_rate_mask)
{
    hif->adc_rate_mask = adc_rate_mask;
    hif->dac_rate_mask = dac_rate_mask;
    lime::info("LA9310 M4 SetSystemClock %f\n", clk_hz);
    hif->sw_cmd_desc.cmd = LIME_M4_SET_SYSTEM_CLOCK_FREQUENCY;
    hif->sw_cmd_desc.data[0] = clk_hz;
    hif->sw_cmd_desc.status = LA9310_SW_CMD_STATUS_POSTED;

    return WaitForResponse();
}

uint32_t LA9310_FW_Impl::GetReferenceClock()
{
    hif->sw_cmd_desc.cmd = LIME_M4_GET_REFERENCE_CLOCK_FREQUENCY;
    hif->sw_cmd_desc.status = LA9310_SW_CMD_STATUS_POSTED;

    auto t1 = chrono::high_resolution_clock::now();
    while (hif->sw_cmd_desc.status == LA9310_SW_CMD_STATUS_POSTED || hif->sw_cmd_desc.status == LA9310_SW_CMD_STATUS_IN_PROGRESS)
    {
        auto t2 = chrono::high_resolution_clock::now();
        if (chrono::duration_cast<chrono::milliseconds>(t2 - t1) > chrono::milliseconds(100))
        {
            lime::error("LA9310_PCIe: GetReferenceClock timeout\n");
            break;
        }
    }

    if (hif->sw_cmd_desc.status == LA9310_SW_CMD_STATUS_DONE)
    {
        const uint32_t frequency = hif->sw_cmd_desc.data[0];
        return frequency;
    }
    else
        return 0;
}

OpStatus LA9310_FW_Impl::SetReferenceClock(uint32_t clk_hz, bool external)
{
    lime::info("LA9310 SetReferenceClock %u ext:%i\n", clk_hz, external);
    hif->sw_cmd_desc.cmd = LIME_M4_SET_REFERENCE_CLOCK_FREQUENCY;
    hif->sw_cmd_desc.data[0] = clk_hz;
    hif->sw_cmd_desc.data[1] = external ? 1 : 0;
    hif->sw_cmd_desc.status = LA9310_SW_CMD_STATUS_POSTED;

    return WaitForResponse();
}

OpStatus LA9310_FW_Impl::ResetHardwareTime()
{
    hif->sw_cmd_desc.cmd = LIME_M4_HARDWARE_COUNTER_RESET;
    hif->sw_cmd_desc.status = LA9310_SW_CMD_STATUS_POSTED;
    return WaitForResponse();
}

uint64_t LA9310_FW_Impl::GetHardwareTime()
{
    hif->sw_cmd_desc.cmd = LIME_M4_HARDWARE_COUNTER_GET;
    hif->sw_cmd_desc.status = LA9310_SW_CMD_STATUS_POSTED;

    auto t1 = chrono::high_resolution_clock::now();
    while (hif->sw_cmd_desc.status == LA9310_SW_CMD_STATUS_POSTED || hif->sw_cmd_desc.status == LA9310_SW_CMD_STATUS_IN_PROGRESS)
    {
        auto t2 = chrono::high_resolution_clock::now();
        if (chrono::duration_cast<chrono::milliseconds>(t2 - t1) > chrono::milliseconds(1000))
        {
            lime::error("LA9310_PCIe: ScheduleTimerEvent timeout\n");
            break;
        }
    }

    if (hif->sw_cmd_desc.status == LA9310_SW_CMD_STATUS_DONE)
    {
        volatile uint64_t* counter64 = reinterpret_cast<volatile uint64_t*>(&hif->sw_cmd_desc.data[0]);
        return *counter64;
    }
    return 0;
}

void* LA9310_FW_Impl::GetHIF(e_m4_mmap type)
{
    hif->sw_cmd_desc.cmd = LIME_M4_GET_FEATURES;
    auto t1 = chrono::high_resolution_clock::now();
    hif->sw_cmd_desc.status = LA9310_SW_CMD_STATUS_POSTED;
    while (hif->sw_cmd_desc.status == LA9310_SW_CMD_STATUS_POSTED || hif->sw_cmd_desc.status == LA9310_SW_CMD_STATUS_IN_PROGRESS)
    {
        auto t2 = chrono::high_resolution_clock::now();
        if (chrono::duration_cast<chrono::milliseconds>(t2 - t1) > chrono::milliseconds(1000))
        {
            lime::error("M4: GetHIF timeout\n");
            break;
        }
    }

    if (hif->sw_cmd_desc.status != LA9310_SW_CMD_STATUS_DONE)
    {
        if (hif->sw_cmd_desc.status == LA9310_SW_CMD_STATUS_POSTED)
        {
            printf("Reset cmd statsu\n");
            hif->sw_cmd_desc.status = LA9310_SW_CMD_STATUS_FREE;
        }
        printf("not done %i\n", hif->sw_cmd_desc.status);
        return nullptr;
    }

    constexpr std::array<std::pair<uint32_t, uint32_t>, 3> valid_addr_ranges{
        { { 0x1F800000, 0x1F81FFFF }, { 0x20000000, 0x2000FFFF }, { 0x20400000, 0x207FFFFF } }
    };
    uint8_t* bars[3] = { reinterpret_cast<uint8_t*>(pcie->GetBar(LA9310_WINDOW_BAR1).vaddr),
        reinterpret_cast<uint8_t*>(pcie->GetBar(LA9310_WINDOW_BAR2).vaddr),
        reinterpret_cast<uint8_t*>(pcie->GetBar(LA9310_WINDOW_BAR2).vaddr) };

    uint32_t table_addr = hif->sw_cmd_desc.data[0];
    int addr_range_index = -1;
    for (size_t j = 0; j < valid_addr_ranges.size(); ++j)
    {
        if (table_addr >= valid_addr_ranges[j].first && table_addr <= valid_addr_ranges[j].second)
        {
            addr_range_index = j;
            break;
        }
    }

    if (addr_range_index < 0)
    {
        printf("Invalid table addr: %08X\n", table_addr);
        return nullptr;
    }

    m4_memory_map_t* feature_table_va =
        reinterpret_cast<m4_memory_map_t*>(bars[addr_range_index] + table_addr - valid_addr_ranges[addr_range_index].first);
    // printf("Table @ 0x%08x\n", table_addr);

    for (int i = 0; i < 32; ++i)
    {
        m4_memory_map_t* row = &feature_table_va[i];
        int addr_range_index = -1;
        for (size_t j = 0; j < valid_addr_ranges.size(); ++j)
        {
            if (row->address >= valid_addr_ranges[j].first && row->address <= valid_addr_ranges[j].second)
            {
                addr_range_index = j;
                break;
            }
        }

        if (addr_range_index < 0)
            break;

        uint8_t* bar = bars[addr_range_index];
        void* feature_va = reinterpret_cast<void*>(bar + row->address - valid_addr_ranges[addr_range_index].first);

        if (row->type == type)
        {
            printf("feature %i ep_pa: 0x%08x\n", row->type, row->address);
            return feature_va;
        }
    }
    return nullptr;
}

} // namespace lime