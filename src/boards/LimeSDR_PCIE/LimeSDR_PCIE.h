#ifndef LIME_LimeSDR_PCIE_H
#define LIME_LimeSDR_PCIE_H

#include "boards/LMS7002M_SDRDevice.h"

#include <vector>
#include <mutex>
#include <memory>

namespace lime {

class Xillybus;
class ISerialPort;
class ISPI;
class ICSR;
class ADF4002;

static const float LIMESDR_PCIE_DEFAULT_REFERENCE_CLOCK = 30.72e6;

/** @brief Class for managing the LimeSDR XTRX device. */
class LimeSDR_PCIE : public LMS7002M_SDRDevice
{
  public:
    LimeSDR_PCIE() = delete;
    LimeSDR_PCIE(std::shared_ptr<ISPI> spiLMS7002M,
        std::shared_ptr<ISPI> spiFPGA,
        std::shared_ptr<Xillybus> sampleStream,
        std::shared_ptr<ISerialPort> control,
        double refClk = LIMESDR_PCIE_DEFAULT_REFERENCE_CLOCK);
    void SetSubDeviceIndex(uint32_t index);

    OpStatus Configure(const SDRConfig& config, uint8_t socIndex) override;

    OpStatus Init() override;

    OpStatus SetSampleRate(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double sampleRate, uint8_t oversample) override;

    double GetClockFreq(uint8_t clk_id, uint8_t channel) override;
    OpStatus SetClockFreq(uint8_t clk_id, double freq, uint8_t channel) override;

    OpStatus SPI(uint32_t chipSelect, const uint32_t* MOSI, uint32_t* MISO, uint32_t count) override;

    OpStatus CustomParameterWrite(const std::vector<CustomParameterIO>& parameters) override;
    OpStatus CustomParameterRead(std::vector<CustomParameterIO>& parameters) override;

    ICSR* getICSR() override;

    OpStatus UploadMemory(
        eMemoryDevice device, uint8_t moduleIndex, const char* data, size_t length, UploadMemoryCallback callback) override;
    OpStatus MemoryWrite(std::shared_ptr<DataStorage> storage, Region region, const void* data) override;
    OpStatus MemoryRead(std::shared_ptr<DataStorage> storage, Region region, void* data) override;

    virtual OpStatus WriteSerialNumber(uint64_t serialNumber) override;

    OpStatus SetAntenna(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t path) override;

    std::unique_ptr<lime::RFStream> StreamCreate(const StreamConfig& config, uint8_t moduleIndex) override;

  private:
    void LMSSetPath(TRXDir dir, uint8_t chan, uint8_t path);
    OpStatus LMS1_SetSampleRate(double f_Hz, uint8_t rxDecimation, uint8_t txInterpolation);
    static OpStatus LMS1_UpdateFPGAInterface(void* userData);

    enum class ePathLMS1_Tx : uint8_t { NONE, BAND1, BAND2 };

    std::shared_ptr<ISPI> lms7002mPort;
    std::shared_ptr<ISPI> fpgaPort;
    std::shared_ptr<Xillybus> mStreamPort;
    std::shared_ptr<ISerialPort> mSerialPort;
    std::unique_ptr<ADF4002> mADF;

    bool mConfigInProgress;
    uint32_t mSubDeviceIndex;
};

} // namespace lime

#endif // LIME_LimeSDR_PCIE_H
