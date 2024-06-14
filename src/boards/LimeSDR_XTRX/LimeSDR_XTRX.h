#ifndef LIME_LIMESDR_XTRX_H
#define LIME_LIMESDR_XTRX_H

#include "LMS7002M_SDRDevice.h"

#include <vector>
#include <mutex>
#include <memory>

namespace lime {

class LitePCIe;
class ISerialPort;
class IComms;

static const float XTRX_DEFAULT_REFERENCE_CLOCK = 26e6;

/** @brief Class for managing the LimeSDR XTRX device. */
class LimeSDR_XTRX : public LMS7002M_SDRDevice
{
  public:
    LimeSDR_XTRX() = delete;
    LimeSDR_XTRX(std::shared_ptr<IComms> spiLMS7002M,
        std::shared_ptr<IComms> spiFPGA,
        std::shared_ptr<LitePCIe> sampleStream,
        std::shared_ptr<ISerialPort> control,
        double refClk = XTRX_DEFAULT_REFERENCE_CLOCK);

    OpStatus Configure(const SDRConfig& config, uint8_t socIndex) override;

    OpStatus Init() override;

    OpStatus SetSampleRate(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double sampleRate, uint8_t oversample) override;

    double GetClockFreq(uint8_t clk_id, uint8_t channel) override;
    OpStatus SetClockFreq(uint8_t clk_id, double freq, uint8_t channel) override;

    OpStatus SPI(uint32_t chipSelect, const uint32_t* MOSI, uint32_t* MISO, uint32_t count) override;

    OpStatus StreamSetup(const StreamConfig& config, uint8_t moduleIndex) override;
    void StreamStop(uint8_t moduleIndex) override;

    OpStatus CustomParameterWrite(const std::vector<CustomParameterIO>& parameters) override;
    OpStatus CustomParameterRead(std::vector<CustomParameterIO>& parameters) override;

    OpStatus UploadMemory(
        eMemoryDevice device, uint8_t moduleIndex, const char* data, size_t length, UploadMemoryCallback callback) override;
    OpStatus MemoryWrite(std::shared_ptr<DataStorage> storage, Region region, const void* data) override;
    OpStatus MemoryRead(std::shared_ptr<DataStorage> storage, Region region, void* data) override;

    virtual OpStatus OEMTest(OEMTestReporter* reporter) override;
    virtual OpStatus WriteSerialNumber(uint64_t serialNumber) override;

    OpStatus SetAntenna(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t path) override;

  protected:
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

    OpStatus ClkTest(OEMTestReporter& reporter, TestData& results);
    OpStatus VCTCXOTest(OEMTestReporter& reporter, TestData& results);
    OpStatus GNSSTest(OEMTestReporter& reporter, TestData& results);
    OpStatus LMS7002_Test(OEMTestReporter& reporter, TestData& results);
    OpStatus RunTestConfig(OEMTestReporter& reporter,
        TestData::RFData* results,
        const std::string& name,
        double LOFreq,
        int gain,
        int rxPath,
        double expectChA_dBFS,
        double expectChB_dBFS);
    OpStatus RFTest(OEMTestReporter& reporter, TestData& results);

  private:
    std::shared_ptr<IComms> lms7002mPort;
    std::shared_ptr<IComms> fpgaPort;
    std::shared_ptr<LitePCIe> mStreamPort;
    std::shared_ptr<ISerialPort> mSerialPort;

    std::mutex mCommsMutex;
    bool mConfigInProgress;
};

} // namespace lime

#endif // LIME_LIMESDR_XTRX_H
