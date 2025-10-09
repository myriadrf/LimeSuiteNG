#include "PHYTimer.h"
#include <stdio.h>
namespace lime {

static inline void iowrite32(uint32_t value, uint64_t addr)
{
    *reinterpret_cast<volatile uint32_t*>(addr) = value;
}

static inline uint32_t ioread32(uint64_t addr)
{
    return *reinterpret_cast<volatile uint32_t*>(addr);
}

PHYTimer::PHYTimer(uint64_t vaddr_base)
    : vaddr_base(vaddr_base)
{
}

void PHYTimer::SetTickRate(double rate)
{
    tickRate = rate / 2;
}

void PHYTimer::SoftReset(bool reset_active)
{
    uint32_t regvalue = ioread32(vaddr_base);
    regvalue &= ~(1 << 4);
    regvalue |= (reset_active << 4);
    iowrite32(regvalue, vaddr_base);
}

void PHYTimer::Enable(bool enable)
{
    uint32_t regvalue = ioread32(vaddr_base);
    regvalue &= ~(1 << 0);
    regvalue |= (enable << 0);
    iowrite32(regvalue, vaddr_base);
}

void PHYTimer::Divisor(uint8_t value)
{
    uint32_t regvalue = ioread32(vaddr_base);
    regvalue &= ~(0x3f << 8);
    regvalue |= (int32_t(value) << 8);
    iowrite32(regvalue, vaddr_base);
}

void PHYTimer::SetTimer(uint8_t id, const PHYTimer::Timer& cfg)
{
    uint32_t regvalue = 0; //ioread32(vaddr_base + 4 + id * 8);
    regvalue &= ~(0x1F);

    regvalue |= cfg.capture_on_falling_edge << 8;
    regvalue |= cfg.interrupt_flag << 7;
    regvalue |= cfg.comparator_enable_value << 6;
    regvalue |= cfg.capture_current_value << 5;
    regvalue |= cfg.cross_trigger_enable << 4;
    regvalue |= cfg.firmware_trigger_mode << 2;
    regvalue |= cfg.comparator_trigger_mode;
    iowrite32(regvalue, vaddr_base + 4 + id * 8);
}

PHYTimer::Timer PHYTimer::GetTimer(uint8_t id) const
{
    Timer cfg;
    const uint32_t regvalue = ioread32(vaddr_base + 4 + id * 8);
    cfg.trigger_output_value = regvalue & (1 << 31);
    cfg.capture_on_falling_edge = regvalue & (1 << 8);
    cfg.interrupt_flag = regvalue & (1 << 7);
    cfg.comparator_enable_value = regvalue & (1 << 6);
    cfg.capture_current_value = regvalue & (1 << 5);
    cfg.cross_trigger_enable = regvalue & (1 << 4);
    cfg.firmware_trigger_mode = (regvalue & 0x3) >> 2;
    cfg.comparator_trigger_mode = regvalue & 0x3;
    return cfg;
}

uint32_t PHYTimer::GetValue(uint8_t id)
{
    uint64_t addr = vaddr_base + 4 + id * 8;
    return ioread32(addr + 4);
}

void PHYTimer::SetValue(uint32_t value, uint8_t id)
{
    uint64_t addr = vaddr_base + 4 + id * 8;
    iowrite32(value, addr + 4);
}

uint32_t PHYTimer::GetTriggerState(uint8_t id, bool* trigger)
{
    uint64_t addr = vaddr_base + 4 + id * 8;
    uint32_t regvalue = ioread32(addr);
    uint32_t tempvalue = regvalue & ~(1 << 7); // 0, to not clear CIF
    iowrite32(regvalue & ~(1 << 5), addr);
    iowrite32(regvalue | (1 << 6), addr);
    uint32_t counter = ioread32(addr + 4);
    regvalue = ioread32(addr);
    bool wasTriggered = regvalue & (1 << 7);
    // iowrite32(regvalue & ~(1<<6), addr);
    if (wasTriggered)
    {
        iowrite32(regvalue | (1 << 7), addr); // clear CIF
        if (trigger)
            *trigger = wasTriggered;
    }
    return counter;
}

uint32_t PHYTimer::GetStatusControl(uint8_t id)
{
    return ioread32(vaddr_base + 4 + id * 8);
}
void PHYTimer::SetStatusControl(uint8_t id, uint32_t value)
{
    iowrite32(value, vaddr_base + 4 + id * 8);
}

void PHYTimer::SetTimerValue(uint8_t id, uint32_t value)
{
    iowrite32(value, vaddr_base + 8 + id * 8);
}

void PHYTimer::DumpMem()
{
    uint64_t addr = vaddr_base;
    printf("base: %08X\n", ioread32(addr));
    addr += 4;
    for (int i = 0; i < 12; ++i)
    {
        printf("T%i %08X %08X\n", i, ioread32(addr), ioread32(addr + 4));
        addr += 8;
    }
}

PPS_Timer::PPS_Timer(PHYTimer* phy, uint8_t clkId)
    : phy(phy)
    , clkId(clkId)
    , nextTrigger(0)
{
    nextTrigger = phy->tickRate;
    t1 = std::chrono::steady_clock::now();
}

void PPS_Timer::Begin()
{
    PHYTimer::Timer cfg;
    cfg.capture_on_falling_edge = false;
    cfg.capture_current_value = false;
    cfg.cross_trigger_enable = false;
    cfg.interrupt_flag = false;
    cfg.firmware_trigger_mode = 0;
    cfg.comparator_trigger_mode = 2;
    cfg.comparator_enable_value = false;
    phy->SetTimer(clkId, cfg);
    phy->SetValue(nextTrigger, clkId);
    cfg.comparator_enable_value = true;
    phy->SetTimer(clkId, cfg);
}

bool PPS_Timer::Update(bool capture)
{
    uint32_t regvalue = phy->GetStatusControl(clkId);
    bool wasTriggered = regvalue & (1 << 7);
    regvalue &= 0x11F;
    if (capture)
    {
        phy->SetStatusControl(clkId, regvalue | (1 << 5));
    }
    regvalue = phy->GetStatusControl(clkId);
    uint32_t counter = phy->GetValue(clkId);
    if (wasTriggered)
    {
        auto t2 = std::chrono::steady_clock::now();
        phy->SetStatusControl(clkId, regvalue | (1 << 7)); // clear CIF
        t1 = t2;
    }

    uint64_t lastSeconds = currentTime.GetSeconds();
    if (wasTriggered)
    {
        ++lastSeconds;
    }
    uint32_t counterRemainder = counter % uint32_t(phy->tickRate);
    currentTime = Timespec(lastSeconds, counterRemainder, phy->tickRate);
    return wasTriggered;
}

Timespec PPS_Timer::Now()
{
    return currentTime;
}

bool PPS_Timer::GetTriggerState()
{
    uint32_t regvalue = phy->GetStatusControl(clkId);
    return regvalue & 0x80000000;
}

void PPS_Timer::ScheduleAt(Timespec tm, uint8_t value)
{
    PHYTimer::Timer cfg;
    cfg.capture_on_falling_edge = false;
    cfg.capture_current_value = false;
    cfg.cross_trigger_enable = false;
    cfg.interrupt_flag = true; // clears flag if was set
    cfg.firmware_trigger_mode = value;
    cfg.comparator_trigger_mode = value;
    cfg.comparator_enable_value = false;
    phy->SetTimer(clkId, cfg);
    uint32_t trigValue = tm.GetTicks() & 0xFFFFFFFF;
    phy->SetValue(trigValue, clkId);
    // cfg.comparator_enable_value = true;
    // phy->SetTimer(clkId, cfg);
}

} // namespace lime