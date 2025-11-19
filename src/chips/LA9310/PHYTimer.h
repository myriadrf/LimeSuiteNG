#pragma once

#include <stdint.h>
#include <string>
#include "limesuiteng/Timespec.h"

namespace lime {

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

    PHYTimerControl(uint64_t vaddr_status_control, const std::string& name);
    void TriggerDirectly(TriggerLogic output);
    void TriggerAtCounter(TriggerLogic output, uint32_t counter);
    uint32_t CaptureCounter();
    uint32_t ReadCounter();

    std::string ToString() const;

  private:
    std::string name;
    uint64_t vaddr_status_control;
};

class LIME_API PHYTimer
{
  private:
    double tickRate;
    uint64_t vaddr_base;

  public:
    PHYTimer(uint64_t vaddr_base);
    void SetReferenceClock(double reference_clock_hz);
    double GetTickRate() const;

    void SoftReset(bool reset_active);
    void Enable(bool enable);
    void Divisor(uint8_t value);

    PHYTimerControl GetTimerControl(uint8_t id) const;
    void DumpMem();
};

} // namespace lime