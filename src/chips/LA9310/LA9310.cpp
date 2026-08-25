#include "LA9310.h"

#include "limesuiteng/Logger.h"

#include "comms/PCIe/LA9310_PCIe.h"
#include "drivers/linux/la9310_limesdr/common_headers/la9310_host_if.h"

#include "vspa/VSPA_mailbox.h"

#include "I2C.h"

static const uint32_t vspa_cpu_id = 0;
static const uint32_t vspa_mbox_id = 0;

namespace lime {

static constexpr uint32_t VSPA_CCSR_offset = 0x1000000;

LA9310::LA9310(std::shared_ptr<LA9310_PCIe> port)
    : pcie(port)
    , i2c(std::make_shared<LA9310_I2C>(port))
    , phytimer(port)
    , eeprom(i2c, 0x50, 65536, 32)
    , mailbox(std::make_shared<VSPA_mailbox>(port))
{
    volatile uint8_t* BAR1_addr = reinterpret_cast<uint8_t*>(pcie->GetBar(LA9310_WINDOW_BAR1).vaddr);
    hif = reinterpret_cast<volatile struct la9310_hif*>(BAR1_addr + LA9310_EP_HIF_OFFSET);
}

LA9310::~LA9310()
{
}

void LA9310::GetADCDACRates(uint8_t* adc_rate_mask, uint8_t* dac_rate_mask)
{
    constexpr uint32_t DCS_BASE_ADDR_offset = 0x1040000;
    volatile uint8_t* DCS = reinterpret_cast<uint8_t*>(pcie->GetBar(LA9310_WINDOW_BAR0).vaddr) + DCS_BASE_ADDR_offset;

    volatile uint32_t* ADC_DAC_CLKCFG = reinterpret_cast<volatile uint32_t*>(DCS + 0x300);
    volatile uint32_t* ADC_DAC_CLKCTRL = reinterpret_cast<volatile uint32_t*>(DCS + 0x310);

    uint32_t dividers = *ADC_DAC_CLKCFG;

    const uint32_t dacDividerBy2 = (dividers >> 16) & 1;
    const uint32_t adcDividerBy2 = ((dividers >> 6) & 0xC) | (dividers & 0x3);

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

OpStatus LA9310::LoadVSPAFirmware(std::span<const char> firmware)
{
    OpStatus status = ResetVCPU();
    if (status != OpStatus::Success)
        return status;
    return pcie->LoadVSPAFirmware(firmware.data(), firmware.size());
}

bool LA9310::IsVSPAFirmwareLoaded() const
{
    const bool clockEnabled = IsClockEnabled();
    if (!clockEnabled)
    {
        printf("VSPA no clock\n");
        return false;
    }

    // Check RCW Completion bit, without it accessing VSPA registers will fail and reboot host
    const volatile uint8_t* BAR0_addr = reinterpret_cast<const uint8_t*>(pcie->GetBar(LA9310_WINDOW_BAR0).vaddr);
    auto swversion_addr = reinterpret_cast<const volatile uint32_t*>(BAR0_addr + VSPA_CCSR_offset + 0x4);
    const uint32_t fw_version = *swversion_addr;
    return fw_version != 0x0;
}

std::vector<DMA_Buffer> LA9310::GetUserSpaceDMABuffers()
{
    return pcie->GetUserSpaceDMABuffers();
}

OpStatus LA9310::ResetVCPU()
{
    OpStatus status;
    if (IsVSPAFirmwareLoaded())
    {
        const mbox_opc_e command = MBOX_OPC_DONE_SWRESET;
        uint32_t hiword = command << 24;
        uint32_t loword = 0;
        uint64_t value = (uint64_t(hiword) << 32) | loword;
        uint64_t result = 0;
        status = mailbox->Message(vspa_cpu_id, vspa_mbox_id, value, &result);
    }
    else
    {
        status = OpStatus::Success;
    }
    return status;
}

bool LA9310::IsClockEnabled() const
{
    // Check RCW Completion bit, without it accessing VSPA registers will fail and reboot host
    const volatile uint8_t* BAR0_addr = reinterpret_cast<const uint8_t*>(pcie->GetBar(LA9310_WINDOW_BAR0).vaddr);
    static const size_t RCW_COMPLETIONR_OFFSET = 0x104;
    static const size_t RESET_SECTION = 0x1E60000;
    static const size_t RCW_COMPLETION_DONE = 0x1;
    auto rcw_completion_addr = reinterpret_cast<const volatile uint32_t*>(BAR0_addr + RESET_SECTION + RCW_COMPLETIONR_OFFSET);
    uint32_t ulRcwCompletion = *rcw_completion_addr;
    if (!(ulRcwCompletion & RCW_COMPLETION_DONE))
    {
        return false;
    }
    return true;
}

} // namespace lime