#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "LimeSDR_XTRX.h"
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

class LimeSDR_XTRX_Fixture : public ::testing::Test
{
  protected:
    void SetUp() override
    {
        auto devices = DeviceRegistry::enumerate(DeviceHandle{ deviceHandleHint });

        if (devices.empty())
        {
            GTEST_SKIP() << "LimeSDR-XTRX not connected, skipping"s;
        }

        device = DeviceRegistry::makeDevice(DeviceHandle{ deviceHandleHint });

        ASSERT_NE(device, nullptr);
    }

    void TearDown() override { DeviceRegistry::freeDevice(device); }

    inline static const std::string deviceHandleHint{ "LimeXTRX0"s };

    SDRDevice* device = nullptr;
};

TEST_F(LimeSDR_XTRX_Fixture, ConnectToDevice)
{
}

TEST_F(LimeSDR_XTRX_Fixture, Configure4HalfTestPatternAndReceiveIt)
{
    constexpr int samplesToReceive = 8;

    SDRDevice::SDRConfig deviceConfiguration;
    SDRDevice::ChannelConfig::Direction& directionConfiguration = deviceConfiguration.channel[0].rx;
    directionConfiguration.enabled = true;
    directionConfiguration.centerFrequency = 1.4e9;
    directionConfiguration.sampleRate = 10e6;
    directionConfiguration.oversample = 6;
    directionConfiguration.path = 2;
    directionConfiguration.lpf = 0;
    directionConfiguration.testSignal.enabled = true;
    directionConfiguration.testSignal.dcMode = false;
    directionConfiguration.testSignal.scale = SDRDevice::ChannelConfig::Direction::TestSignal::Scale::Half;
    directionConfiguration.testSignal.divide = SDRDevice::ChannelConfig::Direction::TestSignal::Divide::Div4;

    device->Configure(deviceConfiguration, 0);

    SDRDevice::StreamConfig streamConfiguration;
    streamConfiguration.channels[TRXDir::Rx] = { 0 };
    streamConfiguration.format = SDRDevice::StreamConfig::DataFormat::I16;

    device->StreamSetup(streamConfiguration, 0);

    device->StreamStart(0);

    std::array<complex16_t, samplesToReceive> sampleBuffer;
    sampleBuffer.fill({ 0, 0 });

    complex16_t* const data = sampleBuffer.data();
    device->StreamRx(0, &data, samplesToReceive, nullptr);

    device->StreamStop(0);

    constexpr std::array<complex16_t, 4> expectedSamples{ {
        { 6960, -13040 },
        { 13024, 6960 },
        { -6976, 13024 },
        { -13040, -6976 },
    } };

    static_assert(expectedSamples.size() * 2 == samplesToReceive);
    EXPECT_THAT(sampleBuffer, AreSamplesCorrect(expectedSamples, samplesToReceive));
}

TEST_F(LimeSDR_XTRX_Fixture, Configure4FullTestPatternAndReceiveIt)
{
    constexpr int samplesToReceive = 8;

    SDRDevice::SDRConfig deviceConfiguration;
    SDRDevice::ChannelConfig::Direction& directionConfiguration = deviceConfiguration.channel[0].rx;
    directionConfiguration.enabled = true;
    directionConfiguration.centerFrequency = 1.4e9;
    directionConfiguration.sampleRate = 10e6;
    directionConfiguration.oversample = 6;
    directionConfiguration.path = 2;
    directionConfiguration.lpf = 0;
    directionConfiguration.testSignal.enabled = true;
    directionConfiguration.testSignal.dcMode = false;
    directionConfiguration.testSignal.scale = SDRDevice::ChannelConfig::Direction::TestSignal::Scale::Full;
    directionConfiguration.testSignal.divide = SDRDevice::ChannelConfig::Direction::TestSignal::Divide::Div4;

    device->Configure(deviceConfiguration, 0);

    SDRDevice::StreamConfig streamConfiguration;
    streamConfiguration.channels[TRXDir::Rx] = { 0 };
    streamConfiguration.format = SDRDevice::StreamConfig::DataFormat::I16;

    device->StreamSetup(streamConfiguration, 0);

    device->StreamStart(0);

    std::array<complex16_t, samplesToReceive> sampleBuffer;
    sampleBuffer.fill({ 0, 0 });

    complex16_t* const data = sampleBuffer.data();
    device->StreamRx(0, &data, samplesToReceive, nullptr);

    device->StreamStop(0);

    constexpr std::array<complex16_t, 4> expectedSamples{ {
        { 13584, -25296 },
        { 25280, 13584 },
        { -13600, 25280 },
        { -25296, -13600 },
    } };

    static_assert(expectedSamples.size() * 2 == samplesToReceive);
    EXPECT_THAT(sampleBuffer, AreSamplesCorrect(expectedSamples, samplesToReceive));
}

TEST_F(LimeSDR_XTRX_Fixture, Configure8HalfTestPatternAndReceiveIt)
{
    const int samplesToReceive = 16;

    SDRDevice::SDRConfig deviceConfiguration;
    SDRDevice::ChannelConfig::Direction& directionConfiguration = deviceConfiguration.channel[0].rx;
    directionConfiguration.enabled = true;
    directionConfiguration.centerFrequency = 1.4e9;
    directionConfiguration.sampleRate = 10e6;
    directionConfiguration.oversample = 6;
    directionConfiguration.path = 2;
    directionConfiguration.lpf = 0;
    directionConfiguration.testSignal.enabled = true;
    directionConfiguration.testSignal.dcMode = false;
    directionConfiguration.testSignal.scale = SDRDevice::ChannelConfig::Direction::TestSignal::Scale::Half;
    directionConfiguration.testSignal.divide = SDRDevice::ChannelConfig::Direction::TestSignal::Divide::Div8;

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
        { 3872, -15504 },
        { 13696, -8224 },
        { 15488, 3872 },
        { 8208, 13696 },
        { -3888, 15488 },
        { -13712, 8208 },
        { -15504, -3888 },
        { -8224, -13712 },
    } };

    static_assert(expectedSamples.size() * 2 == samplesToReceive);
    EXPECT_THAT(sampleBuffer, AreSamplesCorrect(expectedSamples, samplesToReceive));
}

TEST_F(LimeSDR_XTRX_Fixture, Configure8FullTestPatternAndReceiveIt)
{
    const int samplesToReceive = 16;

    SDRDevice::SDRConfig deviceConfiguration;
    SDRDevice::ChannelConfig::Direction& directionConfiguration = deviceConfiguration.channel[0].rx;
    directionConfiguration.enabled = true;
    directionConfiguration.centerFrequency = 1.4e9;
    directionConfiguration.sampleRate = 10e6;
    directionConfiguration.oversample = 6;
    directionConfiguration.path = 2;
    directionConfiguration.lpf = 0;
    directionConfiguration.testSignal.enabled = true;
    directionConfiguration.testSignal.dcMode = false;
    directionConfiguration.testSignal.scale = SDRDevice::ChannelConfig::Direction::TestSignal::Scale::Full;
    directionConfiguration.testSignal.divide = SDRDevice::ChannelConfig::Direction::TestSignal::Divide::Div8;

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
        { 7792, -30784 },
        { 27296, -16464 },
        { 30768, 7792 },
        { 16448, 27296 },
        { -7808, 30768 },
        { -27312, 16448 },
        { -30784, -7808 },
        { -16464, -27312 },
    } };

    static_assert(expectedSamples.size() * 2 == samplesToReceive);
    EXPECT_THAT(sampleBuffer, AreSamplesCorrect(expectedSamples, samplesToReceive));
}
