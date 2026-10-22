#pragma once

#include "limesuiteng/OpStatus.h"

#include "chips/LA9310/LA9310.h"

#include "m4_memorymap.h"

#include <chrono>
#include <span>
#include <string>
#include <stdint.h>

struct la9310_hif;
struct gps_status_payload;
struct ppsdo_status_payload;

namespace lime {

class LA9310_PCIe;

class LIME_API LA9310_FW_Impl : public LA9310
{
  public:
    LA9310_FW_Impl(std::shared_ptr<LA9310_PCIe> pcie);

    bool CheckFirmwareAlive();
    OpStatus LoadFirmware(std::span<const char, std::dynamic_extent> firmware);
    OpStatus EnterFirmwareReloadMode();

    OpStatus SetSystemClock(double clk_hz, uint8_t adc_rate_mask, uint8_t dac_rate_mask);
    uint32_t GetReferenceClock();
    OpStatus SetReferenceClock(uint32_t clk_hz, bool external);

    OpStatus ResetHardwareTime();
    uint64_t GetHardwareTime();

    void* GetHIF(uint32_t type);
    void* GetVSPAHIF(uint32_t type);

    OpStatus DigitalLoopback(bool enable);
    OpStatus EnablePPSDO(bool enable);
    OpStatus GetPPSDOStatus(ppsdo_status_payload& status);
    /// @param tolerance_ppb Frequency error tolerated by each measurement window, 0 for firmware defaults.
    OpStatus ConfigurePPSDO(uint32_t tolerance_ppb);

    OpStatus GetGPSStatus(gps_status_payload& status);
    /// Sends a sentence to the GPS module, body without '$' and checksum, e.g. "PMTK161,0".
    OpStatus SendGPSSentence(const std::string& body);

    // private:
    OpStatus WaitForResponse();
    std::shared_ptr<LA9310_PCIe> pcie;
    volatile struct la9310_hif* hif;
};

} // namespace lime