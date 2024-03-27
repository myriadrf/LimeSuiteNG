#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "LimeSDR_Mini.h"
#include "limesuite/DeviceRegistry.h"

using namespace lime;
using namespace std::literals::string_literals;

MATCHER_P2(AreSamplesCorrect, samples, maxSampleCount, "Checks if the test pattern gave the correct samples"s)
{
    // Samples can begin at any point in the sequence, so we're finding a common start point first.
    for (std::size_t i = 0; i < maxSampleCount - samples.size(); ++i)
    {
        if (arg[i].i == samples.at(0).i && arg[i].q == samples.at(0).q)
        {
            for (std::size_t j = 1; j < samples.size(); ++j)
            {
                if (arg[i + j].i != samples.at(j).i && arg[i + j].q != samples.at(j).q)
                {
                    return false;
                }
            }

            return true;
        }
    }

    return false;
}

class LimeSDR_Mini_Fixture : public ::testing::Test
{
  protected:
    void SetUp() override
    {
        auto devices = DeviceRegistry::enumerate(DeviceHandle{ deviceHandleHint });

        if (devices.empty())
        {
            GTEST_SKIP() << "LimeSDR-Mini not connected, skipping"s;
        }

        device = DeviceRegistry::makeDevice(devices.at(0));

        ASSERT_NE(device, nullptr);

        device->Init();
    }

    void TearDown() override { DeviceRegistry::freeDevice(device); }

    inline static const std::string deviceHandleHint{ "LimeSDR Mini"s };

    SDRDevice* device = nullptr;
};

TEST_F(LimeSDR_Mini_Fixture, ConnectToDevice)
{
}

TEST_F(LimeSDR_Mini_Fixture, Configure4HalfTestPatternAndReceiveIt)
{
    const int samplesToReceive = 8;

    SDRDevice::SDRConfig deviceConfiguration;
    SDRDevice::ChannelConfig::Direction& directionConfiguration = deviceConfiguration.channel[0].rx;
    directionConfiguration.enabled = true;
    directionConfiguration.centerFrequency = 1.2e9;
    directionConfiguration.sampleRate = 10e6;
    directionConfiguration.oversample = 2;
    directionConfiguration.path = 2;
    directionConfiguration.testSignal.enabled = true;
    directionConfiguration.testSignal.dcMode = false;
    directionConfiguration.testSignal.scale = SDRDevice::ChannelConfig::Direction::TestSignal::Scale::Half;
    directionConfiguration.testSignal.divide = SDRDevice::ChannelConfig::Direction::TestSignal::Divide::Div4;

    deviceConfiguration.channel[0].tx.enabled = true; // needed for v2
    deviceConfiguration.channel[0].tx.centerFrequency = 1.9e9 - 1e6;

    device->Configure(deviceConfiguration, 0);

    SDRDevice::StreamConfig streamConfiguration;
    streamConfiguration.channels[TRXDir::Rx] = { 0 };
    streamConfiguration.format = SDRDevice::StreamConfig::DataFormat::I16;
    streamConfiguration.linkFormat = SDRDevice::StreamConfig::DataFormat::I16;

    device->StreamSetup(streamConfiguration, 0);

    device->StreamStart(0);

    std::array<complex16_t, samplesToReceive> sampleBuffer;
    complex16_t* const data = sampleBuffer.data();

    device->StreamRx(0, &data, samplesToReceive, nullptr);

    device->StreamStop(0);

    std::array<complex16_t, 4> expectedSamples{ {
        { 5792, -14000 },
        { 13984, 5792 },
        { -5808, 13984 },
        { -14000, -5808 },
    } };

    static_assert(expectedSamples.size() * 2 == samplesToReceive);
    EXPECT_THAT(sampleBuffer, AreSamplesCorrect(expectedSamples, samplesToReceive));
}

TEST_F(LimeSDR_Mini_Fixture, Configure4FullTestPatternAndReceiveIt)
{
    const int samplesToReceive = 8;

    SDRDevice::SDRConfig deviceConfiguration;
    SDRDevice::ChannelConfig::Direction& directionConfiguration = deviceConfiguration.channel[0].rx;
    directionConfiguration.enabled = true;
    directionConfiguration.centerFrequency = 1.9e9;
    directionConfiguration.sampleRate = 10e6;
    directionConfiguration.oversample = 2;
    directionConfiguration.path = 2;
    directionConfiguration.testSignal.enabled = true;
    directionConfiguration.testSignal.dcMode = false;
    directionConfiguration.testSignal.scale = SDRDevice::ChannelConfig::Direction::TestSignal::Scale::Full;
    directionConfiguration.testSignal.divide = SDRDevice::ChannelConfig::Direction::TestSignal::Divide::Div4;

    deviceConfiguration.channel[0].tx.enabled = true; // needed for v2
    deviceConfiguration.channel[0].tx.centerFrequency = 1.9e9 - 1e6;

    device->Configure(deviceConfiguration, 0);

    SDRDevice::StreamConfig streamConfiguration;
    streamConfiguration.channels[TRXDir::Rx] = { 0 };
    streamConfiguration.format = SDRDevice::StreamConfig::DataFormat::I16;
    streamConfiguration.linkFormat = SDRDevice::StreamConfig::DataFormat::I16;

    device->StreamSetup(streamConfiguration, 0);

    device->StreamStart(0);

    std::array<complex16_t, samplesToReceive> sampleBuffer;
    complex16_t* const data = sampleBuffer.data();

    device->StreamRx(0, &data, samplesToReceive, nullptr);

    device->StreamStop(0);

    std::array<complex16_t, 4> expectedSamples{ {
        { 11584, -27984 },
        { 27968, 11584 },
        { -11600, 27968 },
        { -27984, -11600 },
    } };

    static_assert(expectedSamples.size() * 2 == samplesToReceive);
    EXPECT_THAT(sampleBuffer, AreSamplesCorrect(expectedSamples, samplesToReceive));
}

TEST_F(LimeSDR_Mini_Fixture, Configure8HalfTestPatternAndReceiveIt)
{
    const int samplesToReceive = 16;

    SDRDevice::SDRConfig deviceConfiguration;
    SDRDevice::ChannelConfig::Direction& directionConfiguration = deviceConfiguration.channel[0].rx;
    directionConfiguration.enabled = true;
    directionConfiguration.centerFrequency = 1.9e9;
    directionConfiguration.sampleRate = 10e6;
    directionConfiguration.oversample = 2;
    directionConfiguration.path = 2;
    directionConfiguration.testSignal.enabled = true;
    directionConfiguration.testSignal.dcMode = false;
    directionConfiguration.testSignal.scale = SDRDevice::ChannelConfig::Direction::TestSignal::Scale::Half;
    directionConfiguration.testSignal.divide = SDRDevice::ChannelConfig::Direction::TestSignal::Divide::Div8;

    deviceConfiguration.channel[0].tx.enabled = true; // needed for v2
    deviceConfiguration.channel[0].tx.centerFrequency = 1.9e9 - 1e6;

    device->Configure(deviceConfiguration, 0);

    SDRDevice::StreamConfig streamConfiguration;
    streamConfiguration.channels[TRXDir::Rx] = { 0 };
    streamConfiguration.format = SDRDevice::StreamConfig::DataFormat::I16;
    streamConfiguration.linkFormat = SDRDevice::StreamConfig::DataFormat::I16;

    device->StreamSetup(streamConfiguration, 0);

    device->StreamStart(0);

    std::array<complex16_t, samplesToReceive> sampleBuffer;
    complex16_t* const data = sampleBuffer.data();

    device->StreamRx(0, &data, samplesToReceive, nullptr);

    device->StreamStop(0);

    std::array<complex16_t, 8> expectedSamples{ {
        { 3120, -15776 },
        { 13360, -8928 },
        { 15744, 3120 },
        { 8912, 13360 },
        { -3136, 15744 },
        { -13376, 8912 },
        { -15776, -3136 },
        { -8928, -13376 },
    } };

    static_assert(expectedSamples.size() * 2 == samplesToReceive);
    EXPECT_THAT(sampleBuffer, AreSamplesCorrect(expectedSamples, samplesToReceive));
}

TEST_F(LimeSDR_Mini_Fixture, Configure8FullTestPatternAndReceiveIt)
{
    const int samplesToReceive = 16;

    SDRDevice::SDRConfig deviceConfiguration;
    SDRDevice::ChannelConfig::Direction& directionConfiguration = deviceConfiguration.channel[0].rx;
    directionConfiguration.enabled = true;
    directionConfiguration.centerFrequency = 1.9e9;
    directionConfiguration.sampleRate = 10e6;
    directionConfiguration.oversample = 2;
    directionConfiguration.path = 2;
    directionConfiguration.testSignal.enabled = true;
    directionConfiguration.testSignal.dcMode = false;
    directionConfiguration.testSignal.scale = SDRDevice::ChannelConfig::Direction::TestSignal::Scale::Full;
    directionConfiguration.testSignal.divide = SDRDevice::ChannelConfig::Direction::TestSignal::Divide::Div8;

    deviceConfiguration.channel[0].tx.enabled = true; // needed for v2
    deviceConfiguration.channel[0].tx.centerFrequency = 1.9e9 - 1e6;

    device->Configure(deviceConfiguration, 0);

    SDRDevice::StreamConfig streamConfiguration;
    streamConfiguration.channels[TRXDir::Rx] = { 0 };
    streamConfiguration.format = SDRDevice::StreamConfig::DataFormat::I16;
    streamConfiguration.linkFormat = SDRDevice::StreamConfig::DataFormat::I16;

    device->StreamSetup(streamConfiguration, 0);

    device->StreamStart(0);

    std::array<complex16_t, samplesToReceive> sampleBuffer;
    complex16_t* const data = sampleBuffer.data();

    device->StreamRx(0, &data, samplesToReceive, nullptr);

    device->StreamStop(0);

    std::array<complex16_t, 8> expectedSamples{ {
        { 6256, -31520 },
        { 26720, -17856 },
        { 31504, 6256 },
        { 17840, 26720 },
        { -6272, 31504 },
        { -26736, 17840 },
        { -31520, -6272 },
        { -17856, -26736 },
    } };

    static_assert(expectedSamples.size() * 2 == samplesToReceive);
    EXPECT_THAT(sampleBuffer, AreSamplesCorrect(expectedSamples, samplesToReceive));
}
