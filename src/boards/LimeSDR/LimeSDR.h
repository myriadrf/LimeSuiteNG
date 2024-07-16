#ifndef LIME_LIMESDR_H
#define LIME_LIMESDR_H

#include "LMS7002M_SDRDevice.h"
#include "protocols/LMS64CProtocol.h"

#include <vector>
#include <memory>

namespace lime {

class IUSB;
class IComms;

/** @brief Class for managing the LimeSDR-USB device. */
class LimeSDR : public LMS7002M_SDRDevice
{
  public:
    LimeSDR(std::shared_ptr<IComms> spiLMS,
        std::shared_ptr<IComms> spiFPGA,
        std::shared_ptr<IUSB> mStreamPort,
        std::shared_ptr<ISerialPort> commsPort);
    ~LimeSDR();

    OpStatus Configure(const SDRConfig& config, uint8_t moduleIndex) override;

    OpStatus Init() override;
    OpStatus Reset() override;

    OpStatus EnableChannel(uint8_t moduleIndex, TRXDir trx, uint8_t channel, bool enable) override;

    double GetClockFreq(uint8_t clk_id, uint8_t channel) override;
    OpStatus SetClockFreq(uint8_t clk_id, double freq, uint8_t channel) override;

    OpStatus SetSampleRate(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double sampleRate, uint8_t oversample) override;

    OpStatus SPI(uint32_t chipSelect, const uint32_t* MOSI, uint32_t* MISO, uint32_t count) override;

    OpStatus GPIODirRead(uint8_t* buffer, const size_t bufLength) override;
    OpStatus GPIORead(uint8_t* buffer, const size_t bufLength) override;
    OpStatus GPIODirWrite(const uint8_t* buffer, const size_t bufLength) override;
    OpStatus GPIOWrite(const uint8_t* buffer, const size_t bufLength) override;

    OpStatus CustomParameterWrite(const std::vector<CustomParameterIO>& parameters) override;
    OpStatus CustomParameterRead(std::vector<CustomParameterIO>& parameters) override;

    OpStatus UploadMemory(
        eMemoryDevice device, uint8_t moduleIndex, const char* data, size_t length, UploadMemoryCallback callback) override;
    OpStatus MemoryWrite(std::shared_ptr<DataStorage> storage, Region region, const void* data) override;
    OpStatus MemoryRead(std::shared_ptr<DataStorage> storage, Region region, void* data) override;

  private:
    SDRDescriptor GetDeviceInfo();
    void ResetUSBFIFO();
    static OpStatus UpdateFPGAInterface(void* userData);

    int GPIFClkTest(OEMTestReporter& reporter);
    int Si5351CTest(OEMTestReporter& reporter);
    int ADF4002Test(OEMTestReporter& reporter);
    int ClockNetworkTest(OEMTestReporter& reporter);
    int VCTCXOTest(OEMTestReporter& reporter);
    int FPGA_EEPROM_Test(OEMTestReporter& reporter);
    int LMS7002_Test(OEMTestReporter& reporter);
    int RunTestConfig(OEMTestReporter& reporter, const std::string& name, double LOFreq, int gain, int rxPath);
    int RFTest(OEMTestReporter& reporter);

    std::shared_ptr<IUSB> mStreamPort;
    std::shared_ptr<ISerialPort> mSerialPort;
    std::shared_ptr<IComms> mlms7002mPort;
    std::shared_ptr<IComms> mfpgaPort;
    bool mConfigInProgress;
};

} // namespace lime

#endif /* LIME_LIMESDR_H */
