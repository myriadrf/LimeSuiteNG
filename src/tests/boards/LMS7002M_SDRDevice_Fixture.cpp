#include "LMS7002M_SDRDevice_Fixture.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "limesuiteng/DeviceRegistry.h"
#include "limesuiteng/DeviceHandle.h"
#include "limesuiteng/StreamConfig.h"

#include <array>
#include <cmath>
#include <complex>
#include <iostream>

using namespace lime;
using namespace lime::testing;
using namespace std::literals::string_literals;

LMS7002M_SDRDevice_Fixture::LMS7002M_SDRDevice_Fixture(const std::string& deviceHandleHint)
    : deviceHandleHint(deviceHandleHint)
{
}

void LMS7002M_SDRDevice_Fixture::SetUp()
{
    auto devices = DeviceRegistry::enumerate();

    if (devices.empty())
    {
        GTEST_SKIP() << deviceHandleHint + " not connected, skipping"s;
    }

    device = DeviceRegistry::makeDevice(DeviceHandle{ deviceHandleHint });

    if (device == nullptr)
    {
        GTEST_SKIP() << deviceHandleHint + " not connected, skipping"s;
    }
    else
    {
        device->Init();
    }
}

void LMS7002M_SDRDevice_Fixture::TearDown()
{
    DeviceRegistry::freeDevice(device);
}

void LMS7002M_SDRDevice_Fixture::SetUpDeviceForTestPattern(
    ChannelConfig::Direction::TestSignal::Scale scale, ChannelConfig::Direction::TestSignal::Divide divide)
{
    SDRConfig deviceConfiguration;
    ChannelConfig::Direction& directionConfiguration = deviceConfiguration.channel[0].rx;
    directionConfiguration.enabled = true;
    directionConfiguration.centerFrequency = 1.4e9;
    directionConfiguration.sampleRate = 10e6;
    directionConfiguration.path = 2;
    directionConfiguration.oversample = 2;
    directionConfiguration.lpf = 130e6;
    directionConfiguration.testSignal.enabled = true;
    directionConfiguration.testSignal.dcMode = false;
    directionConfiguration.testSignal.scale = scale;
    directionConfiguration.testSignal.divide = divide;

    device->Configure(deviceConfiguration, 0);

    StreamConfig streamConfiguration;
    streamConfiguration.channels[TRXDir::Rx] = { 0 };
    streamConfiguration.format = DataFormat::I16;

    device->StreamSetup(streamConfiguration, 0);

    device->StreamStart(0);
}

void LMS7002M_SDRDevice_Fixture::Configure4HalfTestPatternAndReceiveIt()
{
    constexpr int samplesToReceive = 8192;

    SetUpDeviceForTestPattern(
        ChannelConfig::Direction::TestSignal::Scale::Half, ChannelConfig::Direction::TestSignal::Divide::Div4);

    std::array<complex16_t, samplesToReceive> sampleBuffer;
    sampleBuffer.fill({ 0, 0 });

    complex16_t* const data = sampleBuffer.data();
    const auto actualSamplesReceived{ device->StreamRx(0, &data, samplesToReceive, nullptr) };

    device->StreamStop(0);

    EXPECT_EQ(samplesToReceive, actualSamplesReceived);
    EXPECT_THAT(sampleBuffer, AreSamplesCorrect(ChannelConfig::Direction::TestSignal::Divide::Div4));
}

void LMS7002M_SDRDevice_Fixture::Configure4FullTestPatternAndReceiveIt()
{
    constexpr int samplesToReceive = 8192;

    SetUpDeviceForTestPattern(
        ChannelConfig::Direction::TestSignal::Scale::Full, ChannelConfig::Direction::TestSignal::Divide::Div4);

    std::array<complex16_t, samplesToReceive> sampleBuffer;
    sampleBuffer.fill({ 0, 0 });

    complex16_t* const data = sampleBuffer.data();
    const auto actualSamplesReceived{ device->StreamRx(0, &data, samplesToReceive, nullptr) };

    device->StreamStop(0);

    EXPECT_EQ(samplesToReceive, actualSamplesReceived);
    EXPECT_THAT(sampleBuffer, AreSamplesCorrect(ChannelConfig::Direction::TestSignal::Divide::Div4));
}

void LMS7002M_SDRDevice_Fixture::Configure8HalfTestPatternAndReceiveIt()
{
    constexpr int samplesToReceive = 16384;

    SetUpDeviceForTestPattern(
        ChannelConfig::Direction::TestSignal::Scale::Half, ChannelConfig::Direction::TestSignal::Divide::Div8);

    std::array<complex16_t, samplesToReceive> sampleBuffer;
    sampleBuffer.fill({ 0, 0 });

    complex16_t* const data = sampleBuffer.data();
    const auto actualSamplesReceived{ device->StreamRx(0, &data, samplesToReceive, nullptr) };

    device->StreamStop(0);

    EXPECT_EQ(samplesToReceive, actualSamplesReceived);
    EXPECT_THAT(sampleBuffer, AreSamplesCorrect(ChannelConfig::Direction::TestSignal::Divide::Div8));
}

void LMS7002M_SDRDevice_Fixture::Configure8FullTestPatternAndReceiveIt()
{
    constexpr int samplesToReceive = 16384;

    SetUpDeviceForTestPattern(
        ChannelConfig::Direction::TestSignal::Scale::Full, ChannelConfig::Direction::TestSignal::Divide::Div8);

    std::array<complex16_t, samplesToReceive> sampleBuffer;
    sampleBuffer.fill({ 0, 0 });

    complex16_t* const data = sampleBuffer.data();
    const auto actualSamplesReceived{ device->StreamRx(0, &data, samplesToReceive, nullptr) };

    device->StreamStop(0);

    EXPECT_EQ(samplesToReceive, actualSamplesReceived);
    EXPECT_THAT(sampleBuffer, AreSamplesCorrect(ChannelConfig::Direction::TestSignal::Divide::Div8));
}
