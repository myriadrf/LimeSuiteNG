#ifndef LIME_LA9310_PCIe_lime_H
#define LIME_LA9310_PCIe_lime_H

#include "limesuiteng/config.h"
#include "limesuiteng/OpStatus.h"

#include <cstdint>
#include <filesystem>
#include <vector>
#include <string>
#include <array>

#include "comms/PCIe/LimePCIe.h"

#include "drivers/linux/la9310_limesdr/la9310_ioctl.h"

namespace lime {

struct mmaped_region {
    void* vaddr{ 0 };
    size_t size{ 0 };
};

/// @brief Class for communicating with a PCIe device.
class LIME_API LA9310_PCIe : public LimePCIe
{
  public:
    LA9310_PCIe();
    virtual ~LA9310_PCIe();

    /// @brief Opens the specified PCIe device for communications.
    /// @param deviceFilename The filename of the device to open.
    /// @param flags The flags to pass to `open()`.
    /// @return The status of the operation.
    OpStatus Open(const std::filesystem::path& deviceFilename, uint32_t flags) override;

    /// @brief Closes this PCIe device.
    void Close() override;

    // Write/Read for communicating to control end points (SPI, I2C...)

    /// @brief Sends the given data buffer to the control port of the device.
    /// @param buffer The data to send to the device.
    /// @param length The length of data to send to the device.
    /// @param timeout_ms The communications timeout in ms to wait for the operation.
    /// @return The amount of bytes transferred (or -1 on error).
    virtual int WriteControl(const uint8_t* buffer, int length, int timeout_ms = 100);

    /// @brief Reads data from the device's control port.
    /// @param buffer The buffer to read the data to.
    /// @param length The maximum length of the data to read.
    /// @param timeout_ms The timeout to wait for for the operation to succeed.
    /// @return The amount of bytes read (or -1 on error).
    virtual int ReadControl(uint8_t* buffer, int length, int timeout_ms = 100);
    virtual OpStatus RunControlCommand(uint8_t* data, size_t length, int timeout_ms = 100);
    virtual OpStatus RunControlCommand(uint8_t* request, uint8_t* response, size_t length, int timeout_ms = 100);

    mmaped_region GetBar(uint8_t i);

  private:
    std::filesystem::path mFilePath;
    int mFileDescriptor;
    void* hostInterfaceAddr;
    LA9310_IOCTL_memory_layout memoryLayout;

    std::array<mmaped_region, LA9310_WINDOW_COUNT> mapped_ranges;
};

} // namespace lime

#endif // LIME_LA9310_PCIe_lime_H
