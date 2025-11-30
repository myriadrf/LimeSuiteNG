#pragma once

#include <stdint.h>
#include <string>
#include <memory>
#include "limesuiteng/Timespec.h"

namespace lime {

class LA9310_PCIe;
class PCIe_CSR_Access;

class LIME_API PHYTimerControl
{
  public:
    enum Flags {
        CTE = (1 << 4),
        CAP = (1 << 5),
        CMPE = (1 << 6), // comparator enable
        CIF = (1 << 7), // comparator interrupt
        CAP_EDGE = (1 << 8),
        TVAL = (1 << 31),
    };

    enum TriggerLogic { NoChange = 0, ForceZero = 1, ForceOne = 2, Invert = 3 };

    PHYTimerControl(std::shared_ptr<PCIe_CSR_Access> status_control_csr, const std::string& name);
    void TriggerDirectly(TriggerLogic output);
    void TriggerAtCounter(TriggerLogic output, uint32_t counter);
    uint32_t CaptureCounter();
    uint32_t ReadCounter();

    std::string ToString() const;

  private:
    std::string name;
    std::shared_ptr<PCIe_CSR_Access> TM_PHY_TMR_CnSC;
};

class LIME_API PHYTimer
{
  private:
    double tickRate;
    std::shared_ptr<LA9310_PCIe> port;
    std::shared_ptr<PCIe_CSR_Access> phytimer_ccsr_base;

  public:
    PHYTimer(std::shared_ptr<LA9310_PCIe> port);
    void SetReferenceClock(double reference_clock_hz);
    double GetTickRate() const;

    void SoftReset(bool reset_active);
    void Enable(bool enable);
    void Divisor(uint8_t value);

    PHYTimerControl GetTimerControl(uint8_t id) const;
    void DumpMem();
};

} // namespace lime