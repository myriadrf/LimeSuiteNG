#include "M4_loader.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstring>
#include <thread>

#include "limesuiteng/Logger.h"
#include "comms/PCIe/LA9310_PCIe.h"
#include "drivers/linux/la9310_limesdr/common_headers/la9310_host_if.h"
#include "drivers/linux/la9310_limesdr/common_headers/la9310_pci_def.h"

namespace lime {

namespace {

static_assert(sizeof(M4_BootHeader) == 8 * sizeof(uint32_t), "it's written to the device as is");

constexpr uint32_t BOOT_PREAMBLE = 0xAA55AA55;
constexpr uint32_t TCML_ADDRESS = 0x1F800000;
constexpr size_t MAX_IMAGE_SIZE = 0x20000; // TCML size

constexpr uint32_t BOOT_FLAG_BYPASS_PLUGIN = 1u << 16;
constexpr uint32_t BOOT_FLAG_BYPASS_EDMA = 1u << 0; // Boot code copies the image using memcpy

// CCSR (BAR0) offsets
constexpr uint32_t DCR_offset = 0x1E00000;
constexpr uint32_t SCRATCHRW_offset = DCR_offset + 0x200;
constexpr uint32_t BOOT_STAGE_offset = SCRATCHRW_offset + LA9310_BOOT_HSHAKE_SCRATCH_REG * sizeof(uint32_t);
constexpr uint32_t HIF_SIZE_offset = SCRATCHRW_offset + LA9310_BOOT_HSHAKE_HIF_SIZ_REG * sizeof(uint32_t);
constexpr uint32_t AMBA_ERROR_RESPONSE_offset = PCIE_RHOM_DBI_BASE + AMBA_ERROR_RESPONSE_DEFAULT_OFF;
// TCMU (BAR2) offset of the boot header that the boot code polls for
constexpr uint32_t BOOT_HEADER_offset = 0x0;

constexpr auto BOOT_STAGE_TIMEOUT = std::chrono::milliseconds(250); // usually M4 responds in <40ms

OpStatus InvalidImage(const char* reason)
{
    lime::error("Arm M4 firmware image is invalid: %s", reason);
    return OpStatus::InvalidValue;
}

template<typename T> volatile T* Register(const mmaped_region& bar, size_t offset)
{
    if (!bar.vaddr || offset + sizeof(T) > bar.size)
        return nullptr;
    return reinterpret_cast<volatile T*>(static_cast<uint8_t*>(bar.vaddr) + offset);
}

bool IsInsideImage(std::span<const char> image, uint32_t offset, uint32_t size)
{
    return offset <= image.size() && size <= image.size() - offset;
}

} // namespace

OpStatus ParseM4Firmware(std::span<const char> image, M4_BootHeader& header)
{
    if (image.empty())
        return InvalidImage("empty file");
    if (image.size() > MAX_IMAGE_SIZE)
        return InvalidImage("firmware image larger than TCML");

    uint32_t preamble = 0;
    if (image.size() >= sizeof(preamble))
        std::memcpy(&preamble, image.data(), sizeof(preamble));

    if (preamble == BOOT_PREAMBLE)
    {
        if (image.size() < sizeof(header))
            return InvalidImage("boot header is truncated");
        std::memcpy(&header, image.data(), sizeof(header));
        if (!IsInsideImage(image, header.bl_src_offset, header.bl_size))
            return InvalidImage("firmware source offset is out of bounds");
        if (header.plugin_size && !IsInsideImage(image, header.plugin_offset, header.plugin_size))
            return InvalidImage("plugin source offset is out of bounds");
    }
    else
    {
        header.preamble = BOOT_PREAMBLE;
        header.plugin_size = 0;
        header.plugin_offset = 0;
        header.bl_size = image.size();
        header.bl_src_offset = 0;
        header.bl_dest = TCML_ADDRESS;
        header.bl_entry = TCML_ADDRESS;
        header.reserved = BOOT_FLAG_BYPASS_PLUGIN;
    }

    header.reserved |= BOOT_FLAG_BYPASS_EDMA;
    return OpStatus::Success;
}

M4_loader::M4_loader(std::shared_ptr<LA9310_PCIe> port)
    : port(port)
{
    const mmaped_region ccsr = port->GetBar(LA9310_WINDOW_BAR0);
    const mmaped_region tcml = port->GetBar(LA9310_WINDOW_BAR1);
    const mmaped_region tcmu = port->GetBar(LA9310_WINDOW_BAR2);

    bootStage = Register<uint32_t>(ccsr, BOOT_STAGE_offset);
    hifSize = Register<uint32_t>(ccsr, HIF_SIZE_offset);
    ambaErrorResponse = Register<uint32_t>(ccsr, AMBA_ERROR_RESPONSE_offset);
    hif = Register<la9310_hif>(tcml, LA9310_EP_HIF_OFFSET);
    bootHeader = Register<M4_BootHeader>(tcmu, BOOT_HEADER_offset);
}

OpStatus M4_loader::WaitForBootStage(uint32_t stage)
{
    const auto deadline = std::chrono::steady_clock::now() + BOOT_STAGE_TIMEOUT;
    uint32_t value = *bootStage;
    while (value < stage)
    {
        if (std::chrono::steady_clock::now() >= deadline)
        {
            lime::error("Arm M4 firmware boot timeout, boot stage %u, expected %u", value, stage);
            return OpStatus::Timeout;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        value = *bootStage;
    }
    return OpStatus::Success;
}

OpStatus M4_loader::VerifyHostInterface()
{
    const uint32_t size = *hifSize;
    if (size < sizeof(la9310_hif))
    {
        lime::error("Arm M4 firmware is not compatible, host interface size %u, expected at least %zu", size, sizeof(la9310_hif));
        return OpStatus::Error;
    }

    const uint32_t expected = LA9310_VER_MAKE(LA9310_HIF_MAJOR_VERSION, LA9310_HIF_MINOR_VERSION);
    const uint32_t version = hif->hif_ver;
    if (version != expected)
    {
        lime::error("Arm M4 firmware is not compatible, host interface version 0x%08X, expected 0x%08X", version, expected);
        return OpStatus::Error;
    }
    return OpStatus::Success;
}

OpStatus M4_loader::Load(std::span<const char> image)
{
    if (!bootStage || !hifSize || !ambaErrorResponse || !hif || !bootHeader)
    {
        lime::error("Arm M4 firmware: device memory is not mapped");
        return OpStatus::NotConnected;
    }

    M4_BootHeader header;
    OpStatus status = ParseM4Firmware(image, header);
    if (status != OpStatus::Success)
        return status;

    // The boot code copies the image into TCML, so it's needed only until the firmware has started.
    // Uncached, so the image needs no cache maintenance on its way to the device.
    DMA_Buffer staging;
    status = port->AllocDMABuffer(image.size(), staging, LA9310_DMAPool::Uncached);
    if (status != OpStatus::Success)
    {
        lime::error("Arm M4 firmware: no DMA memory to stage the image in");
        return status;
    }
    struct FreeStaging {
        LA9310_PCIe& port;
        const DMA_Buffer& buffer;
        ~FreeStaging() { port.FreeDMABuffer(buffer); }
    } freeStaging{ *port, staging };

    std::memcpy(staging.va<void>(), image.data(), image.size());
    status = port->DMA_SyncForDevice(staging);
    if (status != OpStatus::Success)
        return status;

    // The boot code reads the offsets relative to the PCIe outbound window
    const uint32_t windowOffset = staging.endpoint_pa() - PCI_OUTBOUND_WINDOW_BASE_ADDR;
    header.bl_src_offset += windowOffset;
    if (header.plugin_size)
        header.plugin_offset += windowOffset;

    // The image has to be in memory before the boot is initiated. The BARs are mapped uncached, so the register
    // accesses below stay in program order.
    std::atomic_thread_fence(std::memory_order_seq_cst);

    // NXP erratum A-008822
    *ambaErrorResponse = AMBA_ERROR_RESPONSE_DEFAULT_VALUE;

    *bootStage = NONE;

    bootHeader->plugin_size = header.plugin_size;
    bootHeader->plugin_offset = header.plugin_offset;
    bootHeader->bl_src_offset = header.bl_src_offset;
    bootHeader->bl_size = header.bl_size;
    bootHeader->bl_dest = header.bl_dest;
    bootHeader->bl_entry = header.bl_entry;
    bootHeader->reserved = header.reserved;
    // The boot code polls for the preamble, so it has to be written last, this initiates the boot
    bootHeader->preamble = header.preamble;

    // Firmware asks for clock configuration. The host has nothing to configure, and lets the firmware continue.
    status = WaitForBootStage(LA9310_HOST_START_CLOCK_CONFIG);
    if (status != OpStatus::Success)
        return status;
    if (*bootStage < LA9310_HOST_COMPLETE_CLOCK_CONFIG)
        *bootStage = LA9310_HOST_COMPLETE_CLOCK_CONFIG;

    status = WaitForBootStage(LA9310_HOST_START_DRIVER_INIT);
    if (status != OpStatus::Success)
        return status;

    return VerifyHostInterface();
}

} // namespace lime
