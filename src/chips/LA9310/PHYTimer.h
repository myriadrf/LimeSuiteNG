#pragma once

#include <stdint.h>
#include <string>
#include "limesuiteng/Timespec.h"

namespace lime {

class PHYTimerControl
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

    PHYTimerControl(uint64_t vaddr_status_control);
    void TriggerDirectly(TriggerLogic output);
    void TriggerAtCounter(TriggerLogic output, uint32_t counter);
    uint32_t CaptureCounter();

    std::string ToString() const;

  private:
    uint64_t vaddr_status_control;
};

class PHYTimer
{
  private:
    double tickRate;
    uint64_t vaddr_base;

  public:
    PHYTimer(uint64_t vaddr_base);
    void SetTickRate(double tickRate);

    void SoftReset(bool reset_active);
    void Enable(bool enable);
    void Divisor(uint8_t value);

    PHYTimerControl GetTimerControl(uint8_t id) const;
    void DumpMem();
};

} // namespace lime