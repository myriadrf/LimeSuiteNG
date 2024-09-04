#ifndef LIME_LIMESDR_MMX8_H
#define LIME_LIMESDR_MMX8_H

#include "chips/ADF4002/ADF4002.h"
#include "chips/CDCM6208/CDCM6208.h"
#include "comms/IComms.h"
#include "limesuiteng/SDRDevice.h"
#include "limesuiteng/SDRDescriptor.h"
#include "protocols/LMS64CProtocol.h"

#include <cstdint>
#include <map>
#include <memory>
#include <vector>

namespace lime {

class LimePCIe;
class LimeSDR_XTRX;

/** @brief Class for managing the LimeSDR-MMX8 device and its subdevices. */
class LimeSDR_MMX8 : public SDRDevice
{
  public:
    LimeSDR_MMX8() = delete;
    LimeSDR_MMX8(std::vector<std::shared_ptr<IComms>>& spiLMS7002M,
        std::vector<std::shared_ptr<IComms>>& spiFPGA,
        std::vector<std::shared_ptr<LimePCIe>> trxStreams,
        std::shared_ptr<ISerialPort> control,
        std::shared_ptr<ISPI> adfComms);
    ~LimeSDR_MMX8();

    OpStatus Configure(const SDRConfig& config, uint8_t socIndex) override;
    const SDRDescriptor& GetDescriptor() const override;

    OpStatus Init() override;
    OpStatus Reset() override;
    OpStatus GetGPSLock(GPS_Lock* status) override;

    OpStatus EnableChannel(uint8_t moduleIndex, TRXDir trx, uint8_t channel, bool enable) override;

    double GetFrequency(uint8_t moduleIndex, TRXDir trx, uint8_t channel) override;
    OpStatus SetFrequency(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double frequency) override;

    double GetNCOFrequency(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t index, double& phaseOffset) override;
    OpStatus SetNCOFrequency(
        uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t index, double frequency, double phaseOffset = -1.0) override;

    int GetNCOIndex(uint8_t moduleIndex, TRXDir trx, uint8_t channel) override;
    OpStatus SetNCOIndex(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t index, bool downconv) override;

    double GetNCOOffset(uint8_t moduleIndex, TRXDir trx, uint8_t channel) override;

    double GetSampleRate(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint32_t* rf_samplerate = nullptr) override;
    OpStatus SetSampleRate(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double sampleRate, uint8_t oversample) override;

    double GetLowPassFilter(uint8_t moduleIndex, TRXDir trx, uint8_t channel) override;
    OpStatus SetLowPassFilter(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double lpf) override;

    uint8_t GetAntenna(uint8_t moduleIndex, TRXDir trx, uint8_t channel) override;
    OpStatus SetAntenna(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t path) override;

    OpStatus SetTestSignal(uint8_t moduleIndex,
        TRXDir direction,
        uint8_t channel,
        ChannelConfig::Direction::TestSignal signalConfiguration,
        int16_t dc_i = 0,
        int16_t dc_q = 0) override;
    ChannelConfig::Direction::TestSignal GetTestSignal(uint8_t moduleIndex, TRXDir direction, uint8_t channel) override;

    double GetClockFreq(uint8_t clk_id, uint8_t channel) override;
    OpStatus SetClockFreq(uint8_t clk_id, double freq, uint8_t channel) override;

    OpStatus GetGain(uint8_t moduleIndex, TRXDir direction, uint8_t channel, eGainTypes gain, double& value) override;
    OpStatus SetGain(uint8_t moduleIndex, TRXDir direction, uint8_t channel, eGainTypes gain, double value) override;

    bool GetDCOffsetMode(uint8_t moduleIndex, TRXDir trx, uint8_t channel) override;
    OpStatus SetDCOffsetMode(uint8_t moduleIndex, TRXDir trx, uint8_t channel, bool isAutomatic) override;

    complex64f_t GetDCOffset(uint8_t moduleIndex, TRXDir trx, uint8_t channel) override;
    OpStatus SetDCOffset(uint8_t moduleIndex, TRXDir trx, uint8_t channel, const complex64f_t& offset) override;

    complex64f_t GetIQBalance(uint8_t moduleIndex, TRXDir trx, uint8_t channel) override;
    OpStatus SetIQBalance(uint8_t moduleIndex, TRXDir trx, uint8_t channel, const complex64f_t& balance) override;

    bool GetCGENLocked(uint8_t moduleIndex) override;
    double GetTemperature(uint8_t moduleIndex) override;

    bool GetSXLocked(uint8_t moduleIndex, TRXDir trx) override;

    unsigned int ReadRegister(uint8_t moduleIndex, unsigned int address, bool useFPGA = false) override;
    OpStatus WriteRegister(uint8_t moduleIndex, unsigned int address, unsigned int value, bool useFPGA = false) override;

    OpStatus LoadConfig(uint8_t moduleIndex, const std::string& filename) override;
    OpStatus SaveConfig(uint8_t moduleIndex, const std::string& filename) override;

    uint16_t GetParameter(uint8_t moduleIndex, uint8_t channel, const std::string& parameterKey) override;
    OpStatus SetParameter(uint8_t moduleIndex, uint8_t channel, const std::string& parameterKey, uint16_t value) override;

    uint16_t GetParameter(uint8_t moduleIndex, uint8_t channel, uint16_t address, uint8_t msb, uint8_t lsb) override;
    OpStatus SetParameter(
        uint8_t moduleIndex, uint8_t channel, uint16_t address, uint8_t msb, uint8_t lsb, uint16_t value) override;

    OpStatus Synchronize(bool toChip) override;
    void EnableCache(bool enable) override;

    OpStatus Calibrate(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double bandwidth) override;
    OpStatus ConfigureGFIR(
        uint8_t moduleIndex, TRXDir trx, uint8_t channel, ChannelConfig::Direction::GFIRFilter settings) override;

    std::vector<double> GetGFIRCoefficients(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t gfirID) override;
    OpStatus SetGFIRCoefficients(
        uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t gfirID, std::vector<double> coefficients) override;
    OpStatus SetGFIR(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t gfirID, bool enabled) override;

    uint64_t GetHardwareTimestamp(uint8_t moduleIndex) override;
    OpStatus SetHardwareTimestamp(uint8_t moduleIndex, const uint64_t now) override;

    OpStatus StreamSetup(const StreamConfig& config, uint8_t moduleIndex) override;
    void StreamStart(uint8_t moduleIndex) override;
    void StreamStart(const std::vector<uint8_t>& moduleIndexes) override;
    void StreamStop(uint8_t moduleIndex) override;
    void StreamStop(const std::vector<uint8_t>& moduleIndexes) override;
    void StreamDestroy(uint8_t moduleIndex) override;

    uint32_t StreamRx(uint8_t moduleIndex,
        lime::complex32f_t* const* samples,
        uint32_t count,
        StreamMeta* meta,
        std::chrono::microseconds timeout) override;
    uint32_t StreamRx(uint8_t moduleIndex,
        lime::complex16_t* const* samples,
        uint32_t count,
        StreamMeta* meta,
        std::chrono::microseconds timeout) override;
    uint32_t StreamRx(uint8_t moduleIndex,
        lime::complex12_t* const* samples,
        uint32_t count,
        StreamMeta* meta,
        std::chrono::microseconds timeout) override;
    uint32_t StreamTx(uint8_t moduleIndex,
        const lime::complex32f_t* const* samples,
        uint32_t count,
        const StreamMeta* meta,
        std::chrono::microseconds timeout) override;
    uint32_t StreamTx(uint8_t moduleIndex,
        const lime::complex16_t* const* samples,
        uint32_t count,
        const StreamMeta* meta,
        std::chrono::microseconds timeout) override;
    uint32_t StreamTx(uint8_t moduleIndex,
        const lime::complex12_t* const* samples,
        uint32_t count,
        const StreamMeta* meta,
        std::chrono::microseconds timeout) override;
    void StreamStatus(uint8_t moduleIndex, StreamStats* rx, StreamStats* tx) override;

    OpStatus SPI(uint32_t chipSelect, const uint32_t* MOSI, uint32_t* MISO, uint32_t count) override;

    OpStatus CustomParameterWrite(const std::vector<CustomParameterIO>& parameters) override;
    OpStatus CustomParameterRead(std::vector<CustomParameterIO>& parameters) override;

    void SetMessageLogCallback(LogCallbackType callback) override;

    void* GetInternalChip(uint32_t index) override;

    OpStatus UploadMemory(
        eMemoryDevice device, uint8_t moduleIndex, const char* data, size_t length, UploadMemoryCallback callback) override;
    OpStatus MemoryWrite(std::shared_ptr<DataStorage> storage, Region region, const void* data) override;
    OpStatus MemoryRead(std::shared_ptr<DataStorage> storage, Region region, void* data) override;
    OpStatus UploadTxWaveform(const StreamConfig& config, uint8_t moduleIndex, const void** samples, uint32_t count) override;

  private:
    std::shared_ptr<IComms> mMainFPGAcomms;
    SDRDescriptor mDeviceDescriptor;
    std::vector<std::shared_ptr<LimePCIe>> mTRXStreamPorts;
    std::vector<std::unique_ptr<LimeSDR_XTRX>> mSubDevices;
    std::map<uint32_t, LimeSDR_XTRX*> chipSelectToDevice;
    std::map<uint32_t, LimeSDR_XTRX*> customParameterToDevice;
    std::unique_ptr<lime::ADF4002> mADF;
};

} // namespace lime

#endif // LIME_LIMESDR_MMX8_H
