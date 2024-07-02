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
    static std::vector<std::string> GetDevicesWithPattern(const std::string& regex);
    static std::vector<std::string> GetPCIeDeviceList();
    LitePCIe();
    virtual ~LitePCIe();

    OpStatus Open(const std::filesystem::path& deviceFilename, uint32_t flags);
    void Close();
    bool IsOpen() const;

    // Write/Read for communicating to control end points (SPI, I2C...)
    virtual int WriteControl(const uint8_t* buffer, int length, int timeout_ms = 100);
    virtual int ReadControl(uint8_t* buffer, int length, int timeout_ms = 100);

    const std::filesystem::path& GetPathName() const { return mFilePath; };
    void SetPathName(const std::filesystem::path& filePath) { mFilePath = filePath; };

  protected:
    std::filesystem::path mFilePath;
    int mFileDescriptor;
};

} // namespace lime

#endif // LIME_LITEPCIE_H
