#include "PHYTimer.h"
#include <stdio.h>

#include <iostream>
#include <iomanip>
#include <sstream>
#include <unordered_map>

#if 0 // print debug messages
    #define printf_dbg_log(...) \
        do \
        { \
            printf(__VA_ARGS__); \
        } while (0)
#else
    #define printf_dbg_log(format, ...)
#endif

static std::unordered_map<uint8_t, std::string> phytimer_names = {
    { 0, "VSPA_GO[0]" },
    { 1, "(RO ADC 0)" },
    { 2, "(RO ADC 1)" },
    { 3, "(RX ADC 0)" },
    { 4, "(RX ADC 1)" },
    { 5, "(AUX ADC)" },
    { 6, "RSSI" },
    { 11, "(DAC IQ)" },
    { 12, "VSPA_GO[1]" },
    { 13, "PPS_IN" },
    { 14, "PPS_OUT" },
    { 15, "TXRX1" },
    { 16, "TXRX0" },
    { 17, "LNA3_EN" },
    { 18, "LNA2_EN" },
    { 19, "LNA1_EN" },
    { 20, "PA_EN" },
};

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

void PHYTimer::SetReferenceClock(double reference_clock_hz)
{
    tickRate = reference_clock_hz / 2;
}

double PHYTimer::GetTickRate() const
{
    return tickRate;
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

void PHYTimer::DumpMem()
{
    uint64_t addr = vaddr_base;
    uint32_t csr = ioread32(addr);
    std::cerr << "PHYTimer enable:" << bool(csr & 0x1) << " softReset:" << bool(csr & 0x8) << " divisor:" << ((csr >> 8) & 0x3f)
              << "\n";
    for (int i = 0; i < 23; ++i)
    {
        std::cerr << std::left << std::setw(12);
        if (phytimer_names.find(i) != phytimer_names.end())
            std::cerr << phytimer_names.at(i);
        else
            std::cerr << int(i);
        std::cerr << "\t" << GetTimerControl(i).ToString() << std::endl;
    }
}

PHYTimerControl PHYTimer::GetTimerControl(uint8_t id) const
{
    std::stringstream name;
    if (phytimer_names.find(id) != phytimer_names.end())
        name << phytimer_names.at(id);
    else
        name << "T" << int(id);
    return PHYTimerControl(vaddr_base + 4 + id * 8, name.str());
}

PHYTimerControl::PHYTimerControl(uint64_t vaddr_status_control, const std::string& name)
    : name(name)
    , vaddr_status_control(vaddr_status_control)
{
}

void PHYTimerControl::TriggerDirectly(TriggerLogic output)
{
    uint32_t regvalue = ioread32(vaddr_status_control);
    regvalue &= ~0xC; // clear DIR_TRIG;
    regvalue |= CIF;
    regvalue |= CMPE;

    // The value of DIR_TRIG should always be written as 00 when writing to TM_PHY_CnSC while the corresponding
    // comparator is enabled. If the comparator is not known to be disabled and a direct trigger needs to be performed the
    // comparator should be disabled by writing CMPE=1 with DIR_TRIG=00 at least one instruction before writing a non-
    // zero value to DIR_TRIG.

    // Writing 1 to CMPE bit disables the comparator, writing 0 has no effect.
    iowrite32(regvalue, vaddr_status_control); // disable comparator

    regvalue &= ~CMPE;
    regvalue &= ~CIF;
    regvalue |= (output << 2); // DIR_TRIG
    iowrite32(regvalue, vaddr_status_control);
    printf_dbg_log("PHYTimer %s software Trigger %i\n", name.c_str(), bool(ioread32(vaddr_status_control) & TVAL));
}

void PHYTimerControl::TriggerAtCounter(TriggerLogic output, uint32_t counter)
{
    uint32_t regvalue = ioread32(vaddr_status_control);
    regvalue &= ~0xF; // clear DIR_TRIG, CMP_TRIG;
    regvalue |= output; // CMP_TRIG

    regvalue &= ~CAP;
    regvalue |= CIF; // writing 1 clears CIF
    regvalue |= CMPE; // writing 1 disabled comparator
    iowrite32(regvalue, vaddr_status_control);
    regvalue &= ~CMPE; // writing 1 disabled comparator
    iowrite32(regvalue, vaddr_status_control);

    iowrite32(counter, vaddr_status_control + 4); // writing counter enables comparator
    printf_dbg_log("PHYTimer %s schedule comparator TriggerLogic %i @ phy:0x%08X\n", name.c_str(), output, counter);
}

std::string PHYTimerControl::ToString() const
{
    char ctemp[128];
    uint32_t csr = ioread32(vaddr_status_control);
    snprintf(ctemp, sizeof(ctemp), "CSR:%08X CNT:%08X | ", csr, ioread32(vaddr_status_control + 4));
    std::stringstream ss;
    ss << ctemp;
    ss << " Trig:" << (csr & TVAL ? 1 : 0);
    ss << " CIF:" << (csr & CIF ? 1 : 0);
    ss << " CMPE:" << (csr & CMPE ? 1 : 0);
    ss << " CAP:" << (csr & CAP ? 1 : 0);

    return ss.str();
}

uint32_t PHYTimerControl::CaptureCounter()
{
    uint32_t regvalue = ioread32(vaddr_status_control);
    regvalue |= CAP;
    iowrite32(regvalue, vaddr_status_control);
    return ioread32(vaddr_status_control + 4);
}

uint32_t PHYTimerControl::ReadCounter()
{
    return ioread32(vaddr_status_control + 4);
}

} // namespace lime