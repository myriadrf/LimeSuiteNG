#ifndef LIME_LITEPCIE_H
#define LIME_LITEPCIE_H

#include "comms/IDMA.h"
#include "limesuiteng/config.h"
#include "limesuiteng/OpStatus.h"

#include <cstdint>
#include <filesystem>
#include <vector>
#include <string>

namespace lime {

/** @brief Class for communicating with a PCIe device. */
class LIME_API LitePCIe : public IDMA
{
  public:
    static std::vector<std::string> GetDevicesWithPattern(const std::string& regex);
    static std::vector<std::string> GetPCIeDeviceList();
    LitePCIe();
    virtual ~LitePCIe();

    OpStatus Open(const std::filesystem::path& deviceFilename, uint32_t flags);
    void Close();
    bool IsOpen() const override;

    // Write/Read for communicating to control end points (SPI, I2C...)
    virtual int WriteControl(const uint8_t* buffer, int length, int timeout_ms = 100);
    virtual int ReadControl(uint8_t* buffer, int length, int timeout_ms = 100);

    const std::filesystem::path& GetPathName() const { return mFilePath; };
    void SetPathName(const std::filesystem::path& filePath) { mFilePath = filePath; };

    void RxEnable(uint32_t bufferSize, uint8_t irqPeriod) override;
    void TxEnable() override;
    void Disable(TRXDir direction) override;

    /** @brief Structure for holding the Direct Memory Access (DMA) information. */
    struct DMAInfo {
        DMAInfo()
            : rxMemory(nullptr)
            , txMemory(nullptr)
            , bufferSize(0)
            , bufferCount(0)
        {
        }
        std::byte* rxMemory;
        std::byte* txMemory;
        int bufferSize;
        int bufferCount;
    };
    int GetBufferSize() const override;
    int GetBufferCount() const override;
    std::byte* GetMemoryAddress(TRXDir direction) const override;

    DMAState GetState(TRXDir direction) override;
    int SetState(TRXDir direction, DMAState state) override;

    bool Wait(TRXDir direction) override;

    void CacheFlush(TRXDir samplesDirection, DataTransferDirection dataDirection, uint16_t index) override;

  private:
    std::filesystem::path mFilePath;
    DMAInfo mDMA;
    int mFileDescriptor;

    void RxDMAEnable(bool enabled, uint32_t bufferSize, uint8_t irqPeriod);
    void TxDMAEnable(bool enabled);

    DMAState GetRxDMAState();
    DMAState GetTxDMAState();

    int SetRxDMAState(DMAState s);
    int SetTxDMAState(DMAState s);

    bool WaitRx();
    bool WaitTx();
};

} // namespace lime

#endif // LIME_LITEPCIE_H
