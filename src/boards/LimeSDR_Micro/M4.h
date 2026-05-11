#pragma once

#include "limesuiteng/OpStatus.h"

#include "chips/LA9310/LA9310.h"

#include <chrono>
#include <span>
#include <stdint.h>

struct la9310_hif;

namespace lime {

class LA9310_PCIe;

class LimeSDR_Micro_M4 : public LA9310
{
  public:
    LimeSDR_Micro_M4(std::shared_ptr<LA9310_PCIe> pcie);

    OpStatus ScheduleCommand(uint64_t timepoint, uint32_t cmd, const void* data, uint32_t len);

    bool CheckFirmwareAlive();
    OpStatus LoadFirmware(std::span<const char, std::dynamic_extent> firmware);
    OpStatus EnterFirmwareReloadMode();

    OpStatus SetSystemClock(double clk_hz, uint8_t adc_rate_mask, uint8_t dac_rate_mask);
    uint32_t GetReferenceClock();
    OpStatus SetReferenceClock(uint32_t clk_hz, bool external);

    OpStatus ResetHardwareTime();
    uint64_t GetHardwareTime();

    OpStatus TxEnableImmediate(bool enable);
    OpStatus TxEnableScheduled(int64_t time_point, bool enable, uint32_t data_len, uint32_t data_src_offset);

  private:
    OpStatus WaitForResponse();
    std::shared_ptr<LA9310_PCIe> pcie;
    volatile struct la9310_hif* hif;
};

} // namespace lime