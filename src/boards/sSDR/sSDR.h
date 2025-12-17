#ifndef sSDR_H
#define sSDR_H

#include "boards/LMS7002M_SDRDevice.h"

#include <vector>
#include <mutex>
#include <memory>

namespace lime {

class LimePCIe;
class ISerialPort;
class ISPI;
class LMS8001;

static const float SSDR_DEFAULT_REFERENCE_CLOCK = 26e6;

/// @brief Class for managing the sSDR device.
class sSDR : public LMS7002M_SDRDevice
{
  public:
    sSDR() = delete;
    sSDR(std::shared_ptr<ISPI> spiLMS7002M,
        std::shared_ptr<ISPI> spiFPGA,
        std::shared_ptr<LimePCIe> sampleStream,
        std::shared_ptr<ISerialPort> control,
        double refClk = SSDR_DEFAULT_REFERENCE_CLOCK);

    OpStatus Configure(const SDRConfig& config, uint8_t socIndex) override;

    OpStatus Init() override;

    OpStatus SetSampleRate(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double sampleRate, uint8_t oversample) override;

    double GetClockFreq(uint8_t clk_id, uint8_t channel) override;
    OpStatus SetClockFreq(uint8_t clk_id, double freq, uint8_t channel) override;

    OpStatus SPI(uint32_t chipSelect, const uint32_t* MOSI, uint32_t* MISO, uint32_t count) override;

    OpStatus CustomParameterWrite(const std::vector<CustomParameterIO>& parameters) override;
    OpStatus CustomParameterRead(std::vector<CustomParameterIO>& parameters) override;

    OpStatus UploadMemory(
        eMemoryDevice device, uint8_t moduleIndex, const char* data, size_t length, UploadMemoryCallback callback) override;
    OpStatus MemoryWrite(std::shared_ptr<DataStorage> storage, Region region, const void* data) override;
    OpStatus MemoryRead(std::shared_ptr<DataStorage> storage, Region region, void* data) override;

    virtual OpStatus WriteSerialNumber(uint64_t serialNumber) override;

    OpStatus SetAntenna(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t path) override;

    std::unique_ptr<lime::RFStream> StreamCreate(const StreamConfig& config, uint8_t moduleIndex) override;

  private:
    void LMSSetPath(TRXDir dir, uint8_t chan, uint8_t path);
    OpStatus LMS7002M_SetSampleRate(double f_Hz, uint8_t rxDecimation, uint8_t txInterpolation);
    static OpStatus UpdateFPGAInterface(void* userData);

    enum class ePathLMS1_Tx : uint8_t { NONE, BAND1, BAND2 };

    std::shared_ptr<ISPI> lms7002mPort;
    std::shared_ptr<ISPI> fpgaPort;
    std::shared_ptr<ISPI> lms8spi;
    std::shared_ptr<LimePCIe> mStreamPort;
    std::shared_ptr<ISerialPort> mSerialPort;

    std::unique_ptr<LMS8001> rfsoc_lms8001;

    bool mConfigInProgress;
    uint32_t mSubDeviceIndex;
};

} // namespace lime

#endif // sSDR_H
