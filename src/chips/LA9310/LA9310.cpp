#include "LA9310.h"

#include "limesuiteng/Logger.h"

#include "comms/PCIe/LA9310_PCIe.h"
#include "drivers/linux/la9310_limesdr/common_headers/la9310_host_if.h"

namespace lime {

LA9310::LA9310(std::shared_ptr<LA9310_PCIe> port)
    : phytimer(port)
    , vspa(port)
    , pcie(port)
{
    volatile uint8_t* BAR1_addr = reinterpret_cast<uint8_t*>(pcie->GetBar(LA9310_WINDOW_BAR1).vaddr);
    hif = reinterpret_cast<volatile struct la9310_hif*>(BAR1_addr + LA9310_EP_HIF_OFFSET);
}

LA9310::~LA9310()
{
}

OpStatus LA9310::SetSystemClock(double sysClk_Hz, uint8_t adc_rate_mask, uint8_t dac_rate_mask)
{
    hif->adc_rate_mask = adc_rate_mask;
    hif->dac_rate_mask = dac_rate_mask;

    lime::debug("LA9310 set system clock:%g adc_rates:%1x dac_rate:%1x", sysClk_Hz, adc_rate_mask, dac_rate_mask);
    return pcie->SetReferenceClock(sysClk_Hz, 1000);
}

void LA9310::GetADCDACRates(uint8_t* adc_rate_mask, uint8_t* dac_rate_mask)
{
    if (adc_rate_mask)
        *adc_rate_mask = hif->adc_rate_mask;
    if (dac_rate_mask)
        *adc_rate_mask = hif->adc_rate_mask;
}

bool LA9310::IsM4CoreProgrammed()
{
    volatile uint8_t* BAR0_addr = reinterpret_cast<uint8_t*>(pcie->GetBar(LA9310_WINDOW_BAR0).vaddr);
    volatile uint32_t* SCRATCHRW2 = reinterpret_cast<volatile uint32_t*>(BAR0_addr + 0x1E00000 + 0x204);
    return *SCRATCHRW2 != 0; // firmware booting stage
}

} // namespace lime