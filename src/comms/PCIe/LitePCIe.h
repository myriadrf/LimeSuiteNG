#ifndef LIME_LITEPCIE_H
#define LIME_LITEPCIE_H

#include "limesuiteng/config.h"
#include "limesuiteng/OpStatus.h"

#include <cstdint>
#include <filesystem>
#include <vector>
#include <string>

namespace lime {

class LitePCIeDMA;

/// @brief Class for communicating with a PCIe device.
class LIME_API LitePCIe
{
  public:
    friend LitePCIeDMA;

    /// @brief Gets the devices that match the given pattern.
    /// @param pattern The pattern of a filename to pass to `find`.
    /// @return The list of devices matching the given pattern.
    static std::vector<std::string> GetDevicesWithPattern(const std::string& pattern);

    /// @brief Gets the list of devices available via the PCIe driver.
    /// @return The list of devices available.
    static std::vector<std::string> GetPCIeDeviceList();

    /// @brief The constructor for the class.
    LitePCIe();
    virtual ~LitePCIe();

    /// @brief Opens the specified PCIe device for communications.
    /// @param deviceFilename The filename of the device to open.
    /// @param flags The flags to pass to `open()`.
    /// @return The status of the operation.
    OpStatus Open(const std::filesystem::path& deviceFilename, uint32_t flags);

    /// @brief Closes this PCIe device.
    void Close();

    /// @brief Checks if the communication to the device is open.
    /// @return True of can communicate to the device.
    bool IsOpen() const;

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

    /// @brief Gets the name of the file used to communicate with the device.
    /// @return The name of the file used to communicate with the device.
    const std::filesystem::path& GetPathName() const { return mFilePath; };

    /// @brief Sets the path of the file to use to communicate with a device.
    /// @param filePath The new file to use for communications with a device.
    void SetPathName(const std::filesystem::path& filePath) { mFilePath = filePath; };

  private:
    std::filesystem::path mFilePath;
    int mFileDescriptor;
};

} // namespace lime

#endif // LIME_LITEPCIE_H
