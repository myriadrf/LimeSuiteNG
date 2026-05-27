#include "M4.h"

#include <assert.h>
#include <string.h>

#include "m4_commands.h"

#include "comms/PCIe/LA9310_PCIe.h"
#include "drivers/linux/la9310_limesdr/common_headers/la9310_host_if.h"

#include "limesuiteng/Logger.h"

using namespace std;

#define LA9310_EP_HIF_OFFSET 0x1B000

namespace lime {

LimeSDR_Micro_M4::LimeSDR_Micro_M4(std::shared_ptr<LA9310_PCIe> pcie)
    : LA9310(pcie)
    , pcie(pcie)
{
    const auto bar1 = pcie->GetBar(LA9310_WINDOW_BAR1);
    hif = reinterpret_cast<volatile struct la9310_hif*>(size_t(bar1.vaddr) + LA9310_EP_HIF_OFFSET);
    assert(hif);
}

OpStatus LimeSDR_Micro_M4::WaitForResponse()
{
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

OpStatus LimeSDR_Micro_M4::ScheduleCommand(uint64_t timepoint, uint32_t cmd, const void* data, uint32_t len)
{
    hif->sw_cmd_desc.cmd = LIME_M4_SCHEDULE_CMD;
    volatile struct scheduled_cmd* command = reinterpret_cast<volatile struct scheduled_cmd*>(&hif->sw_cmd_desc.data[0]);
    command->timepoint = timepoint;
    command->cmd = cmd;
    for (uint32_t i = 0; i < len / sizeof(uint32_t); ++i)
        command->data[i] = static_cast<const uint32_t*>(data)[i];

    hif->sw_cmd_desc.status = LA9310_SW_CMD_STATUS_POSTED;
    // printf("sched T%u @ %016X v: %u\n", timerid, counter, value);

    return WaitForResponse();
}

bool LimeSDR_Micro_M4::CheckFirmwareAlive()
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

OpStatus LimeSDR_Micro_M4::LoadFirmware(std::span<const char, std::dynamic_extent> firmware)
{
    return pcie->LoadArmM4Firmware(firmware.data(), firmware.size());
}

OpStatus LimeSDR_Micro_M4::EnterFirmwareReloadMode()
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

OpStatus LimeSDR_Micro_M4::SetSystemClock(double clk_hz, uint8_t adc_rate_mask, uint8_t dac_rate_mask)
{
    hif->adc_rate_mask = adc_rate_mask;
    hif->dac_rate_mask = dac_rate_mask;
    lime::info("LA9310 M4 SetSystemClock %f\n", clk_hz);
    hif->sw_cmd_desc.cmd = LIME_M4_SET_SYSTEM_CLOCK_FREQUENCY;
    hif->sw_cmd_desc.data[0] = clk_hz;
    hif->sw_cmd_desc.status = LA9310_SW_CMD_STATUS_POSTED;

    return WaitForResponse();
}

uint32_t LimeSDR_Micro_M4::GetReferenceClock()
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

OpStatus LimeSDR_Micro_M4::SetReferenceClock(uint32_t clk_hz, bool external)
{
    lime::info("LA9310 SetReferenceClock %u ext:%i\n", clk_hz, external);
    hif->sw_cmd_desc.cmd = LIME_M4_SET_REFERENCE_CLOCK_FREQUENCY;
    hif->sw_cmd_desc.data[0] = clk_hz;
    hif->sw_cmd_desc.data[1] = external ? 1 : 0;
    hif->sw_cmd_desc.status = LA9310_SW_CMD_STATUS_POSTED;

    return WaitForResponse();
}

OpStatus LimeSDR_Micro_M4::ResetHardwareTime()
{
    hif->sw_cmd_desc.cmd = LIME_M4_HARDWARE_COUNTER_RESET;
    hif->sw_cmd_desc.status = LA9310_SW_CMD_STATUS_POSTED;
    return WaitForResponse();
}

uint64_t LimeSDR_Micro_M4::GetHardwareTime()
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

} // namespace lime