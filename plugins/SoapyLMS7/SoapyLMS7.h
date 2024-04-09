/**
@file	SoapyLMS7.h
@brief	Header for Soapy SDR + IConnection bindings.
@author Lime Microsystems (www.limemicro.com)
*/

#include <SoapySDR/Device.hpp>

#include "limesuiteng/DeviceRegistry.h"
#include "limesuiteng/SDRDevice.h"
#include "limesuiteng/StreamConfig.h"

#include <array>
#include <chrono>
#include <map>
#include <mutex>
#include <set>
#include <vector>

struct IConnectionStream;

class SoapyLMS7 : public SoapySDR::Device
{
  public:
    SoapyLMS7(const lime::DeviceHandle& handle, const SoapySDR::Kwargs& args);

    ~SoapyLMS7(void);

    /*******************************************************************
     * Identification API
     ******************************************************************/

    std::string getDriverKey(void) const override;

    std::string getHardwareKey(void) const override;

    SoapySDR::Kwargs getHardwareInfo(void) const override;

    /*******************************************************************
     * Channels API
     ******************************************************************/

    size_t getNumChannels(int direction) const override;

    bool getFullDuplex(int direction, size_t channel) const override;

    /*******************************************************************
     * Stream API
     ******************************************************************/
    std::vector<std::string> getStreamFormats(int direction, size_t channel) const override;

    std::string getNativeStreamFormat(int direction, size_t channel, double& fullScale) const override;

    SoapySDR::ArgInfoList getStreamArgsInfo(int direction, size_t channel) const override;

    SoapySDR::Stream* setupStream(int direction,
        const std::string& format,
        const std::vector<size_t>& channels = std::vector<size_t>(),
        const SoapySDR::Kwargs& args = SoapySDR::Kwargs()) override;

    void closeStream(SoapySDR::Stream* stream) override;

    size_t getStreamMTU(SoapySDR::Stream* stream) const override;

    int activateStream(SoapySDR::Stream* stream, int flags = 0, long long timeNs = 0, size_t numElems = 0) override;

    int deactivateStream(SoapySDR::Stream* stream, int flags = 0, long long timeNs = 0) override;

    int readStream(SoapySDR::Stream* stream,
        void* const* buffs,
        size_t numElems,
        int& flags,
        long long& timeNs,
        long timeoutUs = 100000) override;

    int writeStream(SoapySDR::Stream* stream,
        const void* const* buffs,
        size_t numElems,
        int& flags,
        long long timeNs = 0,
        long timeoutUs = 100000) override;

    int readStreamStatus(
        SoapySDR::Stream* stream, size_t& chanMask, int& flags, long long& timeNs, long timeoutUs = 100000) override;

    /*******************************************************************
     * Antenna API
     ******************************************************************/

    std::vector<std::string> listAntennas(int direction, size_t channel) const override;

    void setAntenna(int direction, size_t channel, const std::string& name) override;

    std::string getAntenna(int direction, size_t channel) const override;

    /*******************************************************************
     * Frontend corrections API
     ******************************************************************/

    bool hasDCOffsetMode(int direction, size_t channel) const override;

    void setDCOffsetMode(int direction, size_t channel, const bool automatic) override;

    bool getDCOffsetMode(int direction, size_t channel) const override;

    bool hasDCOffset(int direction, size_t channel) const override;

    void setDCOffset(int direction, size_t channel, const std::complex<double>& offset) override;

    std::complex<double> getDCOffset(int direction, size_t channel) const override;

    bool hasIQBalance(int direction, size_t channel) const override;

    void setIQBalance(int direction, size_t channel, const std::complex<double>& balance) override;

    std::complex<double> getIQBalance(int direction, size_t channel) const override;

    /*******************************************************************
     * Gain API
     ******************************************************************/

    std::vector<std::string> listGains(int direction, size_t channel) const override;

    void setGain(int direction, size_t channel, const double value) override;

    double getGain(int direction, size_t channel) const override;

    void setGain(int direction, size_t channel, const std::string& name, const double value) override;

    double getGain(int direction, size_t channel, const std::string& name) const override;

    SoapySDR::Range getGainRange(int direction, size_t channel) const override;

    SoapySDR::Range getGainRange(int direction, size_t channel, const std::string& name) const override;

    /*******************************************************************
     * Frequency API
     ******************************************************************/

    SoapySDR::ArgInfoList getFrequencyArgsInfo(int direction, size_t channel) const override;

    void setFrequency(
        int direction, size_t channel, const double frequency, const SoapySDR::Kwargs& args = SoapySDR::Kwargs()) override;

    void setFrequency(int direction,
        size_t channel,
        const std::string& name,
        const double frequency,
        const SoapySDR::Kwargs& args = SoapySDR::Kwargs()) override;

    double getFrequency(int direction, size_t channel, const std::string& name) const override;

    double getFrequency(int direction, size_t channel) const override;

    std::vector<std::string> listFrequencies(int direction, size_t channel) const override;

    SoapySDR::RangeList getFrequencyRange(int direction, size_t channel) const override;

    SoapySDR::RangeList getFrequencyRange(int direction, size_t channel, const std::string& name) const override;

    /*******************************************************************
     * Sample Rate API
     ******************************************************************/

    void setSampleRate(int direction, size_t channel, const double rate) override;

    double getSampleRate(int direction, size_t channel) const override;

    SoapySDR::RangeList getSampleRateRange(int direction, size_t channel) const override;

    /*******************************************************************
     * Bandwidth API
     ******************************************************************/

    void setBandwidth(int direction, size_t channel, const double bw) override;

    double getBandwidth(int direction, size_t channel) const override;

    SoapySDR::RangeList getBandwidthRange(int direction, size_t channel) const override;

    /*******************************************************************
     * Clocking API
     ******************************************************************/

    double getMasterClockRate(void) const override;

    /*******************************************************************
     * Time API
     ******************************************************************/

    bool hasHardwareTime(const std::string& what = "") const override;

    long long getHardwareTime(const std::string& what = "") const override;

    void setHardwareTime(long long timeNs, const std::string& what = "") override;

    /*******************************************************************
     * Sensor API
     ******************************************************************/

    std::vector<std::string> listSensors(void) const override;

    SoapySDR::ArgInfo getSensorInfo(const std::string& name) const override;

    std::string readSensor(const std::string& name) const override;

    std::vector<std::string> listSensors(int direction, size_t channel) const override;

    SoapySDR::ArgInfo getSensorInfo(int direction, size_t channel, const std::string& name) const override;

    std::string readSensor(int direction, size_t channel, const std::string& name) const override;

    /*******************************************************************
     * Register API
     ******************************************************************/

    std::vector<std::string> listRegisterInterfaces(void) const override;

    void writeRegister(const std::string& name, const unsigned addr, const unsigned value) override;

    unsigned readRegister(const std::string& name, const unsigned addr) const override;

    void writeRegister(const unsigned addr, const unsigned value) override;

    unsigned readRegister(const unsigned addr) const override;

    /*******************************************************************
     * Settings API
     ******************************************************************/

    SoapySDR::ArgInfoList getSettingInfo(void) const override;

    void writeSetting(const std::string& key, const std::string& value) override;

    SoapySDR::ArgInfoList getSettingInfo(int direction, size_t channel) const override;

    void writeSetting(int direction, size_t channel, const std::string& key, const std::string& value) override;

    std::string readSetting(const std::string& key) const override;

    std::string readSetting(int direction, size_t channel, const std::string& key) const override;

    /*******************************************************************
     * GPIO API
     ******************************************************************/

    std::vector<std::string> listGPIOBanks(void) const override;

    void writeGPIO(const std::string& bank, const unsigned value) override;

    unsigned readGPIO(const std::string& bank) const override;

    void writeGPIODir(const std::string& bank, const unsigned dir) override;

    unsigned readGPIODir(const std::string& bank) const override;

  private:
    struct SettingsCache {
        SettingsCache()
            : calibrationBandwidth(-1)
            , GFIRBandwidth(-1)
            , DCTestAmplitude(0){};
        double calibrationBandwidth;
        double GFIRBandwidth;
        int DCTestAmplitude;
    };

    const std::string _moduleName;
    lime::SDRDevice* sdrDevice;
    std::set<SoapySDR::Stream*> activeStreams;

    lime::StreamConfig streamConfig;

    mutable std::recursive_mutex _accessMutex;

    std::array<double, 2> sampleRate; // sampleRate[direction]
    int oversampling;

    std::map<std::size_t, int> _interps;
    std::map<std::size_t, int> _decims;

    std::array<std::vector<SettingsCache>, 2> settingsCache; // settingsCache[direction]
};
