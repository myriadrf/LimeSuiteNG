#ifndef LIME_LMS7002M_SDRDevice_H
#define LIME_LMS7002M_SDRDevice_H

#include <cstdint>
#include <vector>

#include "limesuiteng/SDRDevice.h"
#include "limesuiteng/SDRDescriptor.h"
#include "limesuiteng/StreamConfig.h"
#include "limesuiteng/LMS7002M.h"

namespace lime {

class TRXLooper;
class FPGA;
struct RFSOCDescriptor;

/** @brief Base class for device with multiple LMS7002M chips and FPGA */
class LIME_API LMS7002M_SDRDevice : public SDRDevice
{
  public:
    LMS7002M_SDRDevice();
    ~LMS7002M_SDRDevice();

    const SDRDescriptor& GetDescriptor() const override;

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

    double GetSampleRate(uint8_t moduleIndex, TRXDir trx, uint8_t channel) override;

    OpStatus SetGain(uint8_t moduleIndex, TRXDir direction, uint8_t channel, eGainTypes gain, double value) override;
    OpStatus GetGain(uint8_t moduleIndex, TRXDir direction, uint8_t channel, eGainTypes gain, double& value) override;

    double GetLowPassFilter(uint8_t moduleIndex, TRXDir trx, uint8_t channel) override;
    OpStatus SetLowPassFilter(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double lpf) override;

    uint8_t GetAntenna(uint8_t moduleIndex, TRXDir trx, uint8_t channel) override;
    OpStatus SetAntenna(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t path) override;

    ChannelConfig::Direction::TestSignal GetTestSignal(uint8_t moduleIndex, TRXDir direction, uint8_t channel) override;
    OpStatus SetTestSignal(uint8_t moduleIndex,
        TRXDir direction,
        uint8_t channel,
        ChannelConfig::Direction::TestSignal signalConfiguration,
        int16_t dc_i = 0,
        int16_t dc_q = 0) override;

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

    OpStatus Calibrate(uint8_t moduleIndex, TRXDir trx, uint8_t channel, double bandwidth) override;
    OpStatus ConfigureGFIR(
        uint8_t moduleIndex, TRXDir trx, uint8_t channel, ChannelConfig::Direction::GFIRFilter settings) override;

    std::vector<double> GetGFIRCoefficients(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t gfirID) override;
    OpStatus SetGFIRCoefficients(
        uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t gfirID, std::vector<double> coefficients) override;
    OpStatus SetGFIR(uint8_t moduleIndex, TRXDir trx, uint8_t channel, uint8_t gfirID, bool enabled) override;

    OpStatus Synchronize(bool toChip) override;
    void EnableCache(bool enable) override;

    uint64_t GetHardwareTimestamp(uint8_t moduleIndex) override;
    OpStatus SetHardwareTimestamp(uint8_t moduleIndex, const uint64_t now) override;

    void StreamStart(uint8_t moduleIndex) override;
    void StreamStop(uint8_t moduleIndex) override;

    uint32_t StreamRx(uint8_t moduleIndex, complex32f_t* const* samples, uint32_t count, StreamMeta* meta) override;
    uint32_t StreamRx(uint8_t moduleIndex, complex16_t* const* samples, uint32_t count, StreamMeta* meta) override;
    uint32_t StreamRx(uint8_t moduleIndex, complex12_t* const* samples, uint32_t count, StreamMeta* meta) override;
    uint32_t StreamTx(uint8_t moduleIndex, const complex32f_t* const* samples, uint32_t count, const StreamMeta* meta) override;
    uint32_t StreamTx(uint8_t moduleIndex, const complex16_t* const* samples, uint32_t count, const StreamMeta* meta) override;
    uint32_t StreamTx(uint8_t moduleIndex, const complex12_t* const* samples, uint32_t count, const StreamMeta* meta) override;
    void StreamStatus(uint8_t moduleIndex, StreamStats* rx, StreamStats* tx) override;

    void SetMessageLogCallback(LogCallbackType callback) override;

    void* GetInternalChip(uint32_t index) override;

    OpStatus UploadMemory(
        eMemoryDevice device, uint8_t moduleIndex, const char* data, size_t length, UploadMemoryCallback callback) override;

    /// @copydoc FPGA::ReadRegister()
    virtual int ReadFPGARegister(uint32_t address);
    /// @copydoc FPGA::WriteRegister()
    virtual OpStatus WriteFPGARegister(uint32_t address, uint32_t value);

  protected:
    static OpStatus UpdateFPGAInterfaceFrequency(LMS7002M& soc, FPGA& fpga, uint8_t chipIndex);
    void SetGainInformationInDescriptor(RFSOCDescriptor& descriptor);

    OpStatus LMS7002LOConfigure(LMS7002M* chip, const SDRConfig& config);
    OpStatus LMS7002ChannelConfigure(LMS7002M* chip, const ChannelConfig& config, uint8_t channelIndex);
    OpStatus LMS7002ChannelCalibration(LMS7002M* chip, const ChannelConfig& config, uint8_t channelIndex);
    OpStatus LMS7002TestSignalConfigure(LMS7002M* chip, const ChannelConfig& config, uint8_t channelIndex);

    static constexpr uint8_t NCOValueCount = 16;

    LogCallbackType mCallback_logMessage;
    std::vector<LMS7002M*> mLMSChips;
    std::vector<TRXLooper*> mStreamers;

    SDRDescriptor mDeviceDescriptor;
    StreamConfig mStreamConfig;
    FPGA* mFPGA{};

  private:
    OpStatus SetGenericRxGain(LMS7002M* device, LMS7002M::Channel channel, double value);
    OpStatus SetGenericTxGain(LMS7002M* device, LMS7002M::Channel channel, double value);

    std::unordered_map<TRXDir, std::unordered_map<uint8_t, double>> lowPassFilterCache;
};

} // namespace lime
#endif
