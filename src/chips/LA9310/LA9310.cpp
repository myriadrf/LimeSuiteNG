#include "LA9310.h"

#include "limesuiteng/Logger.h"

#include "comms/PCIe/LA9310_PCIe.h"
#include "drivers/linux/la9310_limesdr/common_headers/la9310_host_if.h"

#include "I2C.h"

namespace lime {

LA9310::LA9310(std::shared_ptr<LA9310_PCIe> port)
    : pcie(port)
    , i2c(std::make_shared<LA9310_I2C>(port))
    , phytimer(port)
    , vspa(port)
    , eeprom(i2c, 0x50, 65536, 32)
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
    return pcie->SetSystemClock(sysClk_Hz, 1000);
}

void LA9310::GetADCDACRates(uint8_t* adc_rate_mask, uint8_t* dac_rate_mask)
{
    constexpr uint32_t DCS_BASE_ADDR_offset = 0x1040000;
    volatile uint8_t* DCS = reinterpret_cast<uint8_t*>(pcie->GetBar(LA9310_WINDOW_BAR0).vaddr) + DCS_BASE_ADDR_offset;

    volatile uint32_t* ADC_DAC_CLKCFG = reinterpret_cast<volatile uint32_t*>(DCS + 0x300);
    volatile uint32_t* ADC_DAC_CLKCTRL = reinterpret_cast<volatile uint32_t*>(DCS + 0x310);

    uint32_t dividers = *ADC_DAC_CLKCFG;

    const uint32_t dacDividerBy2 = (dividers >> 16) & 1;
    const uint32_t adcDividerBy2 = ((dividers >> 6) & 0xC) | dividers & 0x3;

    const uint32_t dividersEnabled = *ADC_DAC_CLKCTRL;
    const uint32_t rxDividersEn = dividersEnabled & 0xF;
    const uint32_t txDividerEn = (dividersEnabled >> 16) & 0x1;

    if (adc_rate_mask)
        *adc_rate_mask = rxDividersEn & adcDividerBy2;
    if (dac_rate_mask)
        *dac_rate_mask = txDividerEn & dacDividerBy2;
}

bool LA9310::IsM4CoreProgrammed()
{
    volatile uint8_t* BAR0_addr = reinterpret_cast<uint8_t*>(pcie->GetBar(LA9310_WINDOW_BAR0).vaddr);
    volatile uint32_t* SCRATCHRW2 = reinterpret_cast<volatile uint32_t*>(BAR0_addr + 0x1E00000 + 0x204);
    return *SCRATCHRW2 == LA9310_HOST_START_DRIVER_INIT; // firmware boot handshake complete
}

} // namespace lime