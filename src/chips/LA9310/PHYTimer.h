#pragma once

#include <stdint.h>
#include "limesuiteng/Timespec.h"

namespace lime {

class PHYTimer
{
  public:
    double tickRate;
    uint64_t vaddr_base;
    struct Timer {
        bool trigger_output_value{ 0 }; // TVAL
        bool capture_on_falling_edge{ 0 }; // CAP_EDGE
        bool interrupt_flag{ 0 }; // CIF
        bool comparator_enable_value{ 0 }; // CMPE
        bool capture_current_value{ 0 }; // CAP
        bool cross_trigger_enable{ 0 }; // CTE
        uint8_t firmware_trigger_mode{ 0 }; // DIR_TRIG
        uint8_t comparator_trigger_mode{ 0 }; // CMP_TRIG
    };

  public:
    PHYTimer(uint64_t vaddr_base);
    void SetTickRate(double tickRate);

    struct ComparatorStatusAndControl {
        bool trigger_output_value;
        bool capture_on_falling_edge; // CIF
        bool capture_current_value; // CAP
        bool cross_trigger_enable; // CTE
        uint8_t firmware_trigger_mode; // DIR_TRIG
        uint8_t comparator_trigger_mode; // CMP_TRIG
    };

    void SoftReset(bool reset_active);
    void Enable(bool enable);
    void Divisor(uint8_t value);
    void SetTimer(uint8_t id, const PHYTimer::Timer& cfg);
    void SetTimerValue(uint8_t id, uint32_t value);
    PHYTimer::Timer GetTimer(uint8_t id) const;

    uint32_t GetStatusControl(uint8_t id);
    void SetStatusControl(uint8_t id, uint32_t value);

    uint32_t GetValue(uint8_t id);
    void SetValue(uint32_t value, uint8_t id);

    uint32_t GetTriggerState(uint8_t id, bool* wasTriggered);

    void DumpMem();
};

class PPS_Timer
{
  public:
    PPS_Timer(PHYTimer* phy, uint8_t clkId);
    void Begin();
    bool Update(bool capture = false);

    Timespec Now();

    bool GetTriggerState();
    void ScheduleAt(Timespec tm, uint8_t value);

  private:
    Timespec currentTime;
    PHYTimer* phy;
    uint8_t clkId;
    uint32_t nextTrigger;
    std::chrono::time_point<std::chrono::steady_clock> t1;
};

} // namespace lime