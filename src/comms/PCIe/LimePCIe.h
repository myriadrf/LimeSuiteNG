#ifndef LIME_LIMEPCIE_H
#define LIME_LIMEPCIE_H

#include "limesuiteng/config.h"
#include "limesuiteng/OpStatus.h"

#include <cstdint>
#include <filesystem>
#include <vector>
#include <string>

namespace lime {

class LimePCIeDMA;

/// @brief Class for communicating with a PCIe device.
class LIME_API LimePCIe
{
  public:
    friend LimePCIeDMA;
    static std::vector<std::string> GetEndpointsWithPattern(const std::string& deviceAddr, const std::string& regex);
    static std::vector<std::string> GetPCIeDeviceList();
    LimePCIe();
    virtual ~LimePCIe();

    OpStatus Open(const std::filesystem::path& deviceFilename, uint32_t flags);
    void Close();
    bool IsOpen() const;

    // Write/Read for communicating to control end points (SPI, I2C...)
    virtual int WriteControl(const uint8_t* buffer, int length, int timeout_ms = 100);
    virtual int ReadControl(uint8_t* buffer, int length, int timeout_ms = 100);
    virtual OpStatus RunControlCommand(uint8_t* data, size_t length, int timeout_ms = 100);
    virtual OpStatus RunControlCommand(uint8_t* request, uint8_t* response, size_t length, int timeout_ms = 100);

    const std::filesystem::path& GetPathName() const { return mFilePath; };
    void SetPathName(const std::filesystem::path& filePath) { mFilePath = filePath; };

  protected:
    std::filesystem::path mFilePath;
    int mFileDescriptor;
};

} // namespace lime

#endif // LIME_LIMEPCIE_H
