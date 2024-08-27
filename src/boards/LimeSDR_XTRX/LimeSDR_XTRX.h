#ifndef LIME_LIMESDR_XTRX_H
#define LIME_LIMESDR_XTRX_H

#include "LMS7002M_SDRDevice.h"

#include <vector>
#include <mutex>
#include <memory>

namespace lime {

class LimePCIe;
class ISerialPort;
class IComms;
class OEMTestData;

static const float XTRX_DEFAULT_REFERENCE_CLOCK = 26e6;

/** @brief Class for managing the LimeSDR XTRX device. */
class LimeSDR_XTRX : public LMS7002M_SDRDevice
{
  public:
    LimeSDR_XTRX() = delete;
    LimeSDR_XTRX(std::shared_ptr<IComms> spiLMS7002M,
        std::shared_ptr<IComms> spiFPGA,
        std::shared_ptr<LimePCIe> sampleStream,
        std::shared_ptr<ISerialPort> control,
        double refClk = XTRX_DEFAULT_REFERENCE_CLOCK);

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

    virtual OpStatus OEMTest(OEMTestReporter* reporter) override;
    virtual OpStatus WriteSerialNumber(uint64_t serialNumber) override;

    OpStatus SetAntenna(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t path) override;

  private:
    void LMS1SetPath(bool tx, uint8_t chan, uint8_t path);
    OpStatus LMS1_SetSampleRate(double f_Hz, uint8_t rxDecimation, uint8_t txInterpolation);
    static OpStatus LMS1_UpdateFPGAInterface(void* userData);

    enum class ePathLMS1_Tx : uint8_t { NONE, BAND1, BAND2 };

    struct TestData {
        TestData();
        uint32_t vctcxoMinCount{};
        uint32_t vctcxoMaxCount{};
        bool refClkPassed{};
        bool gnssPassed{};
        bool vctcxoPassed{};
        bool lmsChipPassed{};
        struct RFData {
            float frequency;
            float amplitude;
            bool passed;
        };
        RFData lnal[2]{};
        RFData lnaw[2]{};
        RFData lnah[2]{};
    };

    using TestResults = std::vector<OEMTestData>;

    OEMTestData PCIeClockTest(OEMTestReporter& reporter);
    OEMTestData VCTCXOTest(OEMTestReporter& reporter);
    OEMTestData GNSSTest(OEMTestReporter& reporter);
    OEMTestData LMS7002_Test(OEMTestReporter& reporter);

    OEMTestData ConfigureAndMeasure(OEMTestReporter& reporter,
        uint8_t channelIndex,
        double LOFreq,
        const std::string& txAntenna,
        int txGain,
        const std::string& rxAntenna,
        int rxGain,
        double expect_dBFS,
        double allowed_deviation_dBFS);

    OEMTestData RFTest(OEMTestReporter& reporter);

    std::shared_ptr<IComms> lms7002mPort;
    std::shared_ptr<IComms> fpgaPort;
    std::shared_ptr<LimePCIe> mStreamPort;
    std::shared_ptr<ISerialPort> mSerialPort;

    bool mConfigInProgress;
};

} // namespace lime

#endif // LIME_LIMESDR_XTRX_H
