#include "LA9310.h"

#include "limesuiteng/Logger.h"

#include <sys/mman.h>

#include "comms/PCIe/LA9310_PCIe.h"
#include "drivers/linux/la9310_limesdr/common_headers/la9310_host_if.h"

#include "vspa/vspa_loader.h"
#include "vspa/vspa_recovery.h"
#include "vspa/VSPA_mailbox.h"

static const uint32_t vspa_cpu_id = 0;
static const uint32_t vspa_mbox_id = 0;

namespace lime {

static constexpr uint32_t VSPA_CCSR_offset = 0x1000000;

LA9310::LA9310(std::shared_ptr<LA9310_PCIe> port)
    : pcie(port)
    , phytimer(port)
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

namespace {

struct VSPALoaderContext {
    std::shared_ptr<LA9310_PCIe> pcie;
    DMA_Buffer staging;
};

int VSPALoaderSync(void* user, size_t offset, size_t size)
{
    auto* context = static_cast<VSPALoaderContext*>(user);
    return context->pcie->DMA_SyncForDevice(context->staging.subspan(offset, size)) == OpStatus::Success ? 0 : -1;
}

void VSPALoaderLog(void*, vspa_loader_log_level level, const char* message)
{
    if (level == VSPA_LOADER_LOG_ERROR)
        lime::error("%s", message);
    else
        lime::debug("%s", message);
}

OpStatus ToOpStatus(int status)
{
    switch (status)
    {
    case VSPA_LOADER_OK:
        return OpStatus::Success;
    case VSPA_LOADER_INVALID_IMAGE:
        return OpStatus::InvalidValue;
    case VSPA_LOADER_NO_MEMORY:
        return OpStatus::NotSupported;
    case VSPA_LOADER_BUSY:
        return OpStatus::Busy;
    case VSPA_LOADER_TIMEOUT:
        return OpStatus::Timeout;
    case VSPA_LOADER_DMA_ERROR:
        return OpStatus::IOFailure;
    default:
        return OpStatus::Error;
    }
}

} // namespace

OpStatus LA9310::LoadVSPAFirmware(std::span<const char> firmware)
{
    // Check RCW Completion bit, without it accessing VSPA registers will fail and reboot host
    auto t1 = std::chrono::high_resolution_clock::now();
    if (!IsClockEnabled())
    {
        lime::error("VSPA has no clock, can't load firmware");
        return OpStatus::NotConnected;
    }

    // Stage in the last quarter of the first userspace DMA region: LA9310_TRX uses the other quarters (proxy data,
    // Rx and Tx IQ flood), and the core is stopped while it loads anyway.
    std::vector<DMA_Buffer> mappedRegions = pcie->GetUserSpaceDMABuffers();
    struct Unmap {
        const std::vector<DMA_Buffer>& regions;
        ~Unmap()
        {
            for (const DMA_Buffer& region : regions)
                munmap(region.va<void>(), region.size());
        }
    } unmap{ mappedRegions };

    VSPALoaderContext context{ pcie, {} };
    vspa_loader loader{};
    loader.regs =
        reinterpret_cast<volatile vspa_regs*>(static_cast<uint8_t*>(pcie->GetBar(LA9310_WINDOW_BAR0).vaddr) + VSPA_CCSR_offset);
    loader.sync_for_device = VSPALoaderSync;
    loader.log = VSPALoaderLog;
    loader.user = &context;

    // For a core that doesn't stop: its debug block is reached through the M4 firmware's debug proxy
    const mmaped_region bar0 = pcie->GetBar(LA9310_WINDOW_BAR0);
    const mmaped_region bar1 = pcie->GetBar(LA9310_WINDOW_BAR1);
    vspa_recovery recovery{};
    recovery.ccsr = static_cast<volatile uint32_t*>(bar0.vaddr);
    recovery.ccsr_size = bar0.size;
    recovery.tcml = static_cast<volatile uint32_t*>(bar1.vaddr);
    recovery.tcml_size = bar1.size;
    recovery.log = VSPALoaderLog;
    if (recovery.ccsr && recovery.tcml)
        loader.recovery = &recovery;

    if (!mappedRegions.empty())
    {
        const size_t quarter = mappedRegions.front().size() / 4;
        context.staging = mappedRegions.front().subspan(3 * quarter, quarter);
        loader.staging = context.staging.va<uint8_t>();
        loader.staging_pa = static_cast<uint32_t>(context.staging.endpoint_pa());
        loader.staging_size = context.staging.size();
    }

    // Stop the running firmware only when there is something to replace it with
    const uint8_t* image = reinterpret_cast<const uint8_t*>(firmware.data());
    OpStatus status = ToOpStatus(vspa_loader_prepare(&loader, image, firmware.size()));
    if (status == OpStatus::NotSupported)
    {
        // no DMA memory to stage the image in, the driver can still load it
        lime::warning("VSPA firmware: loading through the driver");
        status = ResetVCPU();
        if (status != OpStatus::Success)
            return status;
        return pcie->LoadVSPAFirmware(firmware.data(), firmware.size());
    }
    if (status != OpStatus::Success)
        return status;

    // A non responsive or corrupted firmware is forced into idle by the loader
    status = ResetVCPU();
    if (status != OpStatus::Success)
        lime::warning("VSPA firmware didn't acknowledge the reset request");
    int loadstat = vspa_loader_load(&loader, image, firmware.size());
    auto t2 = std::chrono::high_resolution_clock::now();
    return ToOpStatus(loadstat);
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
        // A firmware that isn't responsive to the reset request is forced into idle by vspa_loader_load(), through
        // the M4 debug proxy (vspa/vspa_recovery.h)
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