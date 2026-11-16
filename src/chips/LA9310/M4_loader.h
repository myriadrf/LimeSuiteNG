#ifndef LIME_M4_loader_H
#define LIME_M4_loader_H

#include <cstdint>
#include <memory>
#include <span>

#include "limesuiteng/OpStatus.h"

struct la9310_hif;

namespace lime {

class LA9310_PCIe;

/// @brief The boot header that the LA9310 boot code polls for at the start of TCMU.
struct M4_BootHeader {
    uint32_t preamble;
    uint32_t plugin_size;
    uint32_t plugin_offset;
    uint32_t bl_size;
    uint32_t bl_src_offset;
    uint32_t bl_dest;
    uint32_t bl_entry;
    uint32_t reserved;
};

/// @brief Determines the boot header of an Arm M4 firmware image.
/// An image that starts with the boot preamble provides its own header, otherwise the image is a raw binary that runs from TCML.
/// @param image The firmware image.
/// @param header The boot header, its offsets are relative to the start of the image.
/// @return InvalidValue if the image is malformed.
OpStatus ParseM4Firmware(std::span<const char> image, M4_BootHeader& header);

/// @brief Validates an Arm M4 boot image for the boot EEPROM, which the LA9310 boot ROM reads from offset 0.
/// Unlike ParseM4Firmware(), a raw binary is rejected, because the boot ROM needs the image to provide its own boot header.
/// @param image The boot image.
/// @param maxSize The space available for the image.
/// @param header The boot header, its offsets are relative to the start of the image.
/// @return InvalidValue if the image is malformed or has no boot header, OutOfRange if it is larger than maxSize.
OpStatus ParseM4BootImage(std::span<const char> image, size_t maxSize, M4_BootHeader& header);

/// @brief Boots the Arm M4 core from a firmware image in the host's memory.
/// The core has to be waiting for a boot header, either after a device reset or after entering the firmware reload mode.
class M4_loader
{
  public:
    M4_loader(std::shared_ptr<LA9310_PCIe> port);

    /// @brief Boots the image and runs the handshake with the started firmware.
    OpStatus Load(std::span<const char> image);

  private:
    OpStatus WaitForBootStage(uint32_t stage);
    OpStatus VerifyHostInterface();

    std::shared_ptr<LA9310_PCIe> port;
    // Device registers and memory, accessed through the mapped BARs. Null if the BAR isn't mapped.
    volatile uint32_t* bootStage; // boot handshake scratch register
    volatile uint32_t* hifSize; // host interface size, reported by the firmware
    volatile uint32_t* ambaErrorResponse;
    volatile la9310_hif* hif;
    volatile M4_BootHeader* bootHeader;
};

} // namespace lime

#endif
