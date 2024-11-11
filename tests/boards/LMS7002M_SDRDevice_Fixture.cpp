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

#include "tests/externalData.h"

using namespace lime;
using namespace lime::testing;
using namespace std::literals::string_literals;

LMS7002M_SDRDevice_Fixture::LMS7002M_SDRDevice_Fixture()
    : deviceHandleHint()
{
}

LMS7002M_SDRDevice_Fixture::LMS7002M_SDRDevice_Fixture(const std::string& deviceHandleHint)
    : deviceHandleHint(deviceHandleHint)
{
}

void LMS7002M_SDRDevice_Fixture::SetUp()
{
    device = DeviceRegistry::makeDevice(std::string{ GetTestDeviceHandleArgument() });
    ASSERT_NE(device, nullptr);
    ASSERT_EQ(device->Init(), OpStatus::Success);
}

void LMS7002M_SDRDevice_Fixture::TearDown()
{
    DeviceRegistry::freeDevice(device);
}

void LMS7002M_SDRDevice_Fixture::SetUpDeviceForRxTestPattern(
    ChannelConfig::Direction::TestSignal::Scale scale, ChannelConfig::Direction::TestSignal::Divide divide)
{
    SDRConfig deviceConfiguration;
    ChannelConfig::Direction& directionConfiguration = deviceConfiguration.channel[0].rx;
    directionConfiguration.enabled = true;
    directionConfiguration.centerFrequency = 1.4e9;
    directionConfiguration.sampleRate = 30.72e6;
    directionConfiguration.path = 2;
    directionConfiguration.oversample = 2;
    directionConfiguration.lpf = 130e6;
    directionConfiguration.testSignal.enabled = true;
    directionConfiguration.testSignal.dcMode = false;
    directionConfiguration.testSignal.scale = scale;
    directionConfiguration.testSignal.divide = divide;

    ASSERT_EQ(device->Configure(deviceConfiguration, moduleIndex), OpStatus::Success);
}

void LMS7002M_SDRDevice_Fixture::SetupStream()
{
    StreamConfig streamConfiguration;
    streamConfiguration.channels[TRXDir::Rx] = { 0 };
    streamConfiguration.format = DataFormat::I16;

    ASSERT_EQ(device->StreamSetup(streamConfiguration, moduleIndex), OpStatus::Success);
    device->StreamStart(moduleIndex);
}

void LMS7002M_SDRDevice_Fixture::DestroySteam()
{
    device->StreamStop(moduleIndex);
    device->StreamDestroy(moduleIndex);
}

TEST_F(LMS7002M_SDRDevice_Fixture, Configure8HalfTestPatternAndReceiveIt)
{
    ASSERT_NO_FATAL_FAILURE(SetUpDeviceForRxTestPattern(
        ChannelConfig::Direction::TestSignal::Scale::Half, ChannelConfig::Direction::TestSignal::Divide::Div8));
    ASSERT_NO_FATAL_FAILURE(SetupStream());
    constexpr int samplesToReceive = 1020 * 16;

    std::array<complex16_t, samplesToReceive> sampleBuffer;
    sampleBuffer.fill({ 0, 0 });

    complex16_t* const data = sampleBuffer.data();
    StreamMeta meta{};
    const auto actualSamplesReceived{ device->StreamRx(0, &data, samplesToReceive, &meta) };
    EXPECT_EQ(meta.timestamp, 0);
    ASSERT_EQ(samplesToReceive, actualSamplesReceived);
    EXPECT_THAT(sampleBuffer, AreSamplesCorrect(ChannelConfig::Direction::TestSignal::Divide::Div8));
}
