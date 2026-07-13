// Unit tests for the antenna-by-name C entry points
// (lime_sdrdevice_set_antenna / lime_sdrdevice_get_antenna), which resolve
// antenna names to indexes via RFSOCDescriptor::pathNames.
//
// These exercise pure library logic and require no hardware. A minimal StubSDRDevice
// implements every pure virtual; only the handful relevant to channel addressing and
// antenna resolution carry real behaviour, the rest are trivial stubs.

#include <gtest/gtest.h>

#include "limesuiteng/sdrdevice.h"

#include "limesuiteng/SDRDevice.hpp"
#include "limesuiteng/SDRDescriptor.hpp"
#include "limesuiteng/RFSOCDescriptor.h"
#include "limesuiteng/RFStream.hpp"

using namespace lime;

namespace lime::testing {

// Overriding the deprecated stream virtuals is unavoidable for a concrete subclass;
// silence the resulting -Wdeprecated-declarations noise for the stub only.
#if defined(__GNUC__)
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

class StubSDRDevice : public SDRDevice
{
  public:
    // --- state the tests inspect ---
    SDRDescriptor desc;
    uint8_t currentPath{ 0 }; ///< last path set via the (module,dir,channel,path) virtual

    // --- pure virtuals with real behaviour ---
    const SDRDescriptor& GetDescriptor() const override { return desc; }

    uint8_t GetAntenna(uint8_t, TRXDir, uint8_t) override { return currentPath; }
    OpStatus SetAntenna(uint8_t, TRXDir, uint8_t, uint8_t path) override
    {
        currentPath = path;
        return OpStatus::Success;
    }
    OpStatus SetFrequency(uint8_t, TRXDir, uint8_t, double) override { return OpStatus::Success; }
    OpStatus EnableChannel(uint8_t, TRXDir, uint8_t, bool) override { return OpStatus::Success; }

    // --- remaining pure virtuals: trivial stubs ---
    OpStatus Configure(const SDRConfig&, uint8_t) override { return OpStatus::Success; }
    OpStatus Init() override { return OpStatus::Success; }
    OpStatus Reset() override { return OpStatus::Success; }
    OpStatus GetGPSLock(GPS_Lock*) override { return OpStatus::Success; }
    double GetClockFreq(uint8_t, uint8_t) override { return 0.0; }
    OpStatus SetClockFreq(uint8_t, double, uint8_t) override { return OpStatus::Success; }
    double GetFrequency(uint8_t, TRXDir, uint8_t) override { return 0.0; }
    double GetNCOFrequency(uint8_t, TRXDir, uint8_t, uint8_t, double&) override { return 0.0; }
    OpStatus SetNCOFrequency(uint8_t, TRXDir, uint8_t, uint8_t, double, double) override { return OpStatus::Success; }
    double GetNCOOffset(uint8_t, TRXDir, uint8_t) override { return 0.0; }
    int GetNCOIndex(uint8_t, TRXDir, uint8_t) override { return 0; }
    OpStatus SetNCOIndex(uint8_t, TRXDir, uint8_t, uint8_t, bool) override { return OpStatus::Success; }
    double GetSampleRate(uint8_t, TRXDir, uint8_t, uint32_t*) override { return 0.0; }
    OpStatus SetSampleRate(uint8_t, TRXDir, uint8_t, double, uint8_t) override { return OpStatus::Success; }
    OpStatus GetGain(uint8_t, TRXDir, uint8_t, eGainTypes, double&) override { return OpStatus::Success; }
    OpStatus SetGain(uint8_t, TRXDir, uint8_t, eGainTypes, double) override { return OpStatus::Success; }
    double GetLowPassFilter(uint8_t, TRXDir, uint8_t) override { return 0.0; }
    OpStatus SetLowPassFilter(uint8_t, TRXDir, uint8_t, double) override { return OpStatus::Success; }
    ChannelConfig::Direction::TestSignal GetTestSignal(uint8_t, TRXDir, uint8_t) override { return {}; }
    OpStatus SetTestSignal(uint8_t, TRXDir, uint8_t, ChannelConfig::Direction::TestSignal, int16_t, int16_t) override
    {
        return OpStatus::Success;
    }
    bool GetDCOffsetMode(uint8_t, TRXDir, uint8_t) override { return false; }
    OpStatus SetDCOffsetMode(uint8_t, TRXDir, uint8_t, bool) override { return OpStatus::Success; }
    complex64f_t GetDCOffset(uint8_t, TRXDir, uint8_t) override { return {}; }
    OpStatus SetDCOffset(uint8_t, TRXDir, uint8_t, const complex64f_t&) override { return OpStatus::Success; }
    complex64f_t GetIQBalance(uint8_t, TRXDir, uint8_t) override { return {}; }
    OpStatus SetIQBalance(uint8_t, TRXDir, uint8_t, const complex64f_t&) override { return OpStatus::Success; }
    bool GetCGENLocked(uint8_t) override { return false; }
    double GetTemperature(uint8_t) override { return 0.0; }
    bool GetSXLocked(uint8_t, TRXDir) override { return false; }
    unsigned int ReadRegister(uint8_t, unsigned int, bool) override { return 0; }
    OpStatus WriteRegister(uint8_t, unsigned int, unsigned int, bool) override { return OpStatus::Success; }
    OpStatus LoadConfig(uint8_t, const std::string&) override { return OpStatus::Success; }
    OpStatus SaveConfig(uint8_t, const std::string&) override { return OpStatus::Success; }
    uint16_t GetParameter(uint8_t, uint8_t, const std::string&) override { return 0; }
    OpStatus SetParameter(uint8_t, uint8_t, const std::string&, uint16_t) override { return OpStatus::Success; }
    uint16_t GetParameter(uint8_t, uint8_t, uint16_t, uint8_t, uint8_t) override { return 0; }
    OpStatus SetParameter(uint8_t, uint8_t, uint16_t, uint8_t, uint8_t, uint16_t) override { return OpStatus::Success; }
    OpStatus Calibrate(uint8_t, TRXDir, uint8_t, double) override { return OpStatus::Success; }
    OpStatus ConfigureGFIR(uint8_t, TRXDir, uint8_t, ChannelConfig::Direction::GFIRFilter) override { return OpStatus::Success; }
    std::vector<double> GetGFIRCoefficients(uint8_t, TRXDir, uint8_t, uint8_t) override { return {}; }
    OpStatus SetGFIRCoefficients(uint8_t, TRXDir, uint8_t, uint8_t, std::vector<double>) override { return OpStatus::Success; }
    OpStatus SetGFIR(uint8_t, TRXDir, uint8_t, uint8_t, bool) override { return OpStatus::Success; }
    OpStatus Synchronize(bool) override { return OpStatus::Success; }
    void EnableCache(bool) override {}
    uint64_t GetHardwareTimestamp(uint8_t) override { return 0; }
    OpStatus SetHardwareTimestamp(uint8_t, const uint64_t) override { return OpStatus::Success; }
    OpStatus StreamSetup(const StreamConfig&, uint8_t) override { return OpStatus::Success; }
    void StreamStart(uint8_t) override {}
    void StreamStop(uint8_t) override {}
    void StreamDestroy(uint8_t) override {}
    uint32_t StreamRx(uint8_t, complex32f_t* const*, uint32_t, StreamMeta*, std::chrono::microseconds) override { return 0; }
    uint32_t StreamRx(uint8_t, complex16_t* const*, uint32_t, StreamMeta*, std::chrono::microseconds) override { return 0; }
    uint32_t StreamRx(uint8_t, complex12_t* const*, uint32_t, StreamMeta*, std::chrono::microseconds) override { return 0; }
    uint32_t StreamTx(uint8_t, const complex32f_t* const*, uint32_t, const StreamMeta*, std::chrono::microseconds) override
    {
        return 0;
    }
    uint32_t StreamTx(uint8_t, const complex16_t* const*, uint32_t, const StreamMeta*, std::chrono::microseconds) override
    {
        return 0;
    }
    uint32_t StreamTx(uint8_t, const complex12_t* const*, uint32_t, const StreamMeta*, std::chrono::microseconds) override
    {
        return 0;
    }
    void StreamStatus(uint8_t, StreamStats*, StreamStats*) override {}
    std::unique_ptr<RFStream> StreamCreate(const StreamConfig&, uint8_t) override { return nullptr; }
    void* GetInternalChip(uint32_t) override { return nullptr; }
};

#if defined(__GNUC__)
    #pragma GCC diagnostic pop
#endif

// Builds a descriptor with one RF SoC carrying named Rx/Tx paths.
static SDRDescriptor MakeDescriptorWithPaths()
{
    using namespace std::string_literals;
    RFSOCDescriptor soc;
    soc.name = "TestSoC";
    soc.channelCount = 2;
    soc.pathNames[TRXDir::Rx] = { "None"s, "LNAH"s, "LNAL"s, "LNAW"s };
    soc.pathNames[TRXDir::Tx] = { "None"s, "Band1"s, "Band2"s };

    SDRDescriptor desc;
    desc.rfSOC.push_back(soc);
    return desc;
}

// The C API's device handle is the SDRDevice pointer; tests hand the stub to the
// C entry points exactly the way lime_device_as_sdr() would.
static lime_SDRDevice* AsC(StubSDRDevice& stub)
{
    return reinterpret_cast<lime_SDRDevice*>(static_cast<SDRDevice*>(&stub));
}

// --------------------------------------------------------------------------
// Antenna addressed by name through the C API
// --------------------------------------------------------------------------

TEST(AntennaByName, SetKnownNameResolvesToIndex)
{
    StubSDRDevice stub;
    stub.desc = MakeDescriptorWithPaths();

    EXPECT_EQ(lime_sdrdevice_set_antenna(AsC(stub), 0, lime_TRXDir_Rx, 0, "LNAW"), lime_OpStatus_Success);
    EXPECT_EQ(stub.currentPath, 3); // "LNAW" is index 3 in the Rx path list
}

TEST(AntennaByName, SetUnknownNameReturnsInvalidValue)
{
    StubSDRDevice stub;
    stub.desc = MakeDescriptorWithPaths();

    EXPECT_EQ(lime_sdrdevice_set_antenna(AsC(stub), 0, lime_TRXDir_Rx, 0, "DOES_NOT_EXIST"), lime_OpStatus_InvalidValue);
    EXPECT_EQ(stub.currentPath, 0); // unchanged
}

TEST(AntennaByName, SetOnOutOfRangeModuleReturnsInvalidValue)
{
    StubSDRDevice stub;
    stub.desc = MakeDescriptorWithPaths(); // only module 0 exists

    EXPECT_EQ(lime_sdrdevice_set_antenna(AsC(stub), 5, lime_TRXDir_Rx, 0, "LNAH"), lime_OpStatus_InvalidValue);
}

TEST(AntennaByName, DirectionIsHonoured)
{
    StubSDRDevice stub;
    stub.desc = MakeDescriptorWithPaths();

    // "Band2" exists only on the Tx path (index 2); it must not resolve on Rx.
    EXPECT_EQ(lime_sdrdevice_set_antenna(AsC(stub), 0, lime_TRXDir_Rx, 0, "Band2"), lime_OpStatus_InvalidValue);

    EXPECT_EQ(lime_sdrdevice_set_antenna(AsC(stub), 0, lime_TRXDir_Tx, 0, "Band2"), lime_OpStatus_Success);
    EXPECT_EQ(stub.currentPath, 2);
}

TEST(AntennaByName, GetAntennaRoundTrips)
{
    StubSDRDevice stub;
    stub.desc = MakeDescriptorWithPaths();

    ASSERT_EQ(lime_sdrdevice_set_antenna(AsC(stub), 0, lime_TRXDir_Rx, 0, "LNAL"), lime_OpStatus_Success);
    EXPECT_STREQ(lime_sdrdevice_get_antenna(AsC(stub), 0, lime_TRXDir_Rx, 0), "LNAL");
}

TEST(AntennaByName, GetAntennaOnOutOfRangeModuleReturnsNull)
{
    StubSDRDevice stub;
    stub.desc = MakeDescriptorWithPaths();

    EXPECT_EQ(lime_sdrdevice_get_antenna(AsC(stub), 9, lime_TRXDir_Rx, 0), nullptr);
}

TEST(AntennaByName, GetAntennaWithPathOutOfRangeReturnsNull)
{
    StubSDRDevice stub;
    stub.desc = MakeDescriptorWithPaths();
    stub.currentPath = 42; // beyond the 4 Rx names; simulates an inconsistent index

    EXPECT_EQ(lime_sdrdevice_get_antenna(AsC(stub), 0, lime_TRXDir_Rx, 0), nullptr);
}

} // namespace lime::testing
