#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "boards/LimeSDR_XTRX/LimeSDR_XTRX.h"
#include "limesuiteng/DeviceRegistry.h"

#include <limits>

using namespace lime;
using namespace std::literals::string_literals;

MATCHER_P2(AreSamplesCorrect, samples, maxSampleCount, "Checks if the test pattern gave the correct samples"s)
{
    // Samples can begin at any point in the sequence, so we're finding a common start point first.
    std::size_t startPoint = std::numeric_limits<std::size_t>::max();

    for (std::size_t i = 0; i < maxSampleCount - samples.size(); ++i)
    {
        if (arg.at(i).i == samples.at(0).i && arg.at(i).q == samples.at(0).q)
        {
            startPoint = i;
            break;
        }
    }

    if (startPoint == std::numeric_limits<std::size_t>::max())
    {
        return false;
    }

    // Starting from 1 because the 0th sample is already checked above.
    for (std::size_t i = 1; i < samples.size(); ++i)
    {
        if (arg.at(startPoint + i).i != samples.at(i).i || arg.at(startPoint + i).q != samples.at(i).q)
        {
            return false;
        }
    }

    return true;
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

    void SetUpDeviceForTestPattern(SDRDevice::ChannelConfig::Direction::TestSignal::Scale scale,
        SDRDevice::ChannelConfig::Direction::TestSignal::Divide divide);

    inline static const std::string deviceHandleHint{ "LimeXTRX0"s };

    SDRDevice* device = nullptr;
};

void LimeSDR_XTRX_Fixture::SetUpDeviceForTestPattern(
    SDRDevice::ChannelConfig::Direction::TestSignal::Scale scale, SDRDevice::ChannelConfig::Direction::TestSignal::Divide divide)
{
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
    directionConfiguration.testSignal.scale = scale;
    directionConfiguration.testSignal.divide = divide;

    device->Configure(deviceConfiguration, 0);

    SDRDevice::StreamConfig streamConfiguration;
    streamConfiguration.channels[TRXDir::Rx] = { 0 };
    streamConfiguration.format = SDRDevice::StreamConfig::DataFormat::I16;

    device->StreamSetup(streamConfiguration, 0);

    device->StreamStart(0);
}

TEST_F(LimeSDR_XTRX_Fixture, ConnectToDevice)
{
}

TEST_F(LimeSDR_XTRX_Fixture, Configure4HalfTestPatternAndReceiveIt)
{
    constexpr int samplesToReceive = 8;

    SetUpDeviceForTestPattern(SDRDevice::ChannelConfig::Direction::TestSignal::Scale::Half,
        SDRDevice::ChannelConfig::Direction::TestSignal::Divide::Div4);

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

    SetUpDeviceForTestPattern(SDRDevice::ChannelConfig::Direction::TestSignal::Scale::Full,
        SDRDevice::ChannelConfig::Direction::TestSignal::Divide::Div4);

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

    SetUpDeviceForTestPattern(SDRDevice::ChannelConfig::Direction::TestSignal::Scale::Half,
        SDRDevice::ChannelConfig::Direction::TestSignal::Divide::Div8);

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

    SetUpDeviceForTestPattern(SDRDevice::ChannelConfig::Direction::TestSignal::Scale::Full,
        SDRDevice::ChannelConfig::Direction::TestSignal::Divide::Div8);

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
