#ifndef LIME_LITEPCIE_H
#define LIME_LITEPCIE_H

#include "limesuiteng/config.h"
#include "limesuiteng/OpStatus.h"

#include <cstdint>
#include <filesystem>
#include <vector>
#include <string>

namespace lime {

/** @brief Class for communicating with a PCIe device. */
class LIME_API LitePCIe
{
  public:
    static std::vector<std::string> GetDevicesWithPattern(const std::string& regex);
    static std::vector<std::string> GetPCIeDeviceList();
    LitePCIe();
    ~LitePCIe();

    OpStatus Open(const std::filesystem::path& deviceFilename, uint32_t flags);
    void Close();
    bool IsOpen();

    // Write/Read for communicating to control end points (SPI, I2C...)
    int WriteControl(const uint8_t* buffer, int length, int timeout_ms = 100);
    int ReadControl(uint8_t* buffer, int length, int timeout_ms = 100);

    const std::filesystem::path& GetPathName() const { return mFilePath; };
    void SetPathName(const std::filesystem::path& filePath) { mFilePath = filePath; };

    void RxDMAEnable(bool enabled, uint32_t bufferSize, uint8_t irqPeriod);
    void TxDMAEnable(bool enabled);

    /** @brief Structure for holding the Direct Memory Access (DMA) information. */
    struct DMAInfo {
        DMAInfo()
            : rxMemory(nullptr)
            , txMemory(nullptr)
            , bufferSize(0)
            , bufferCount(0)
        {
        }
        uint8_t* rxMemory;
        uint8_t* txMemory;
        int bufferSize;
        int bufferCount;
    };
    DMAInfo GetDMAInfo() { return mDMA; }

    /** @brief Structure for holding the current state of the Direct Memory Access (DMA). */
    struct DMAState {
        uint32_t hwIndex;
        uint32_t swIndex;
        uint32_t bufferSize;
        bool enabled;
        bool genIRQ;
    };
    DMAState GetRxDMAState();
    DMAState GetTxDMAState();

    int SetRxDMAState(DMAState s);
    int SetTxDMAState(DMAState s);

    bool WaitRx();
    bool WaitTx();

    void CacheFlush(bool isTx, bool toDevice, uint16_t index);

  protected:
    std::filesystem::path mFilePath;
    DMAInfo mDMA;
    int mFileDescriptor;
};

} // namespace lime

#endif // LIME_LITEPCIE_H
