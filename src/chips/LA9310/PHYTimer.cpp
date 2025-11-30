#include "PHYTimer.h"
#include <stdio.h>

#include <iostream>
#include <iomanip>
#include <sstream>
#include <unordered_map>

#include "comms/PCIe/LA9310_PCIe.h"

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

static constexpr size_t la9310_phytimer_base_addr = 0x1020000;

PHYTimer::PHYTimer(std::shared_ptr<LA9310_PCIe> port)
    : port(port)
    , phytimer_ccsr_base(port->GetCSRAccess(LA9310_WINDOW_BAR0, la9310_phytimer_base_addr))
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
    uint32_t regvalue = phytimer_ccsr_base->ioread32(0);
    regvalue &= ~(1 << 4);
    regvalue |= (reset_active << 4);
    phytimer_ccsr_base->iowrite32(regvalue, 0);
}

void PHYTimer::Enable(bool enable)
{
    uint32_t regvalue = phytimer_ccsr_base->ioread32(0);
    regvalue &= ~(1 << 0);
    regvalue |= (enable << 0);
    phytimer_ccsr_base->iowrite32(regvalue, 0);
}

void PHYTimer::Divisor(uint8_t value)
{
    uint32_t regvalue = phytimer_ccsr_base->ioread32(0);
    regvalue &= ~(0x3f << 8);
    regvalue |= (int32_t(value) << 8);
    phytimer_ccsr_base->iowrite32(regvalue, 0);
}

void PHYTimer::DumpMem()
{
    uint32_t csr = phytimer_ccsr_base->ioread32(0);
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
    return PHYTimerControl(port->GetCSRAccess(LA9310_WINDOW_BAR0, la9310_phytimer_base_addr + 4 + id * 8), name.str());
}

PHYTimerControl::PHYTimerControl(std::shared_ptr<PCIe_CSR_Access> status_control_csr, const std::string& name)
    : name(name)
    , TM_PHY_TMR_CnSC(status_control_csr)
{
}

void PHYTimerControl::TriggerDirectly(TriggerLogic output)
{
    uint32_t regvalue = TM_PHY_TMR_CnSC->ioread32(0);
    regvalue &= ~0xC; // clear DIR_TRIG;
    regvalue |= CIF;
    regvalue |= CMPE;

    // The value of DIR_TRIG should always be written as 00 when writing to TM_PHY_CnSC while the corresponding
    // comparator is enabled. If the comparator is not known to be disabled and a direct trigger needs to be performed the
    // comparator should be disabled by writing CMPE=1 with DIR_TRIG=00 at least one instruction before writing a non-
    // zero value to DIR_TRIG.

    // Writing 1 to CMPE bit disables the comparator, writing 0 has no effect.
    TM_PHY_TMR_CnSC->iowrite32(regvalue, 0); // disable comparator

    regvalue &= ~CMPE;
    regvalue &= ~CIF;
    regvalue |= (output << 2); // DIR_TRIG
    TM_PHY_TMR_CnSC->iowrite32(regvalue, 0);
    printf_dbg_log("PHYTimer %s software Trigger %i\n", name.c_str(), bool(TM_PHY_TMR_CnSC->ioread32(0) & TVAL));
}

void PHYTimerControl::TriggerAtCounter(TriggerLogic output, uint32_t counter)
{
    uint32_t regvalue = TM_PHY_TMR_CnSC->ioread32(0);
    regvalue &= ~0xF; // clear DIR_TRIG, CMP_TRIG;
    regvalue |= output; // CMP_TRIG

    regvalue &= ~CAP;
    regvalue |= CIF; // writing 1 clears CIF
    regvalue |= CMPE; // writing 1 disabled comparator
    TM_PHY_TMR_CnSC->iowrite32(regvalue, 0);
    regvalue &= ~CMPE; // writing 1 disabled comparator
    TM_PHY_TMR_CnSC->iowrite32(regvalue, 0);

    TM_PHY_TMR_CnSC->iowrite32(counter, 4); // writing counter enables comparator
    printf_dbg_log("PHYTimer %s schedule comparator TriggerLogic %i @ phy:0x%08X\n", name.c_str(), output, counter);
}

std::string PHYTimerControl::ToString() const
{
    char ctemp[128];
    uint32_t csr = TM_PHY_TMR_CnSC->ioread32(0);
    snprintf(ctemp, sizeof(ctemp), "CSR:%08X CNT:%08X | ", csr, TM_PHY_TMR_CnSC->ioread32(4));
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
    uint32_t regvalue = TM_PHY_TMR_CnSC->ioread32(0);
    regvalue |= CAP;
    TM_PHY_TMR_CnSC->iowrite32(regvalue, 0);
    return TM_PHY_TMR_CnSC->ioread32(4);
}

uint32_t PHYTimerControl::ReadCounter()
{
    return TM_PHY_TMR_CnSC->ioread32(4);
}

} // namespace lime