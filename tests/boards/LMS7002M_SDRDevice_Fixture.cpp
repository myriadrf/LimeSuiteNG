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
    auto devices = DeviceRegistry::enumerate(DeviceHandle{ deviceHandleHint });

    if (devices.empty())
    {
        GTEST_SKIP() << deviceHandleHint + " not connected, skipping"s;
    }

    device = DeviceRegistry::makeDevice(devices.at(0));

    ASSERT_NE(device, nullptr);
    // std::cout << "Run on device: " << devices.at(0).Serialize() << "\n";

    ASSERT_EQ(device->Init(), OpStatus::Success);
}

void LMS7002M_SDRDevice_Fixture::TearDown()
{
    if (device)
        DestroySteam();
    DeviceRegistry::freeDevice(device);
}

OpStatus LMS7002M_SDRDevice_Fixture::SetUpDeviceForRxTestPattern(
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

    return device->Configure(deviceConfiguration, moduleIndex);
}

OpStatus LMS7002M_SDRDevice_Fixture::SetupStream()
{
    StreamConfig streamConfiguration;
    streamConfiguration.channels[TRXDir::Rx] = { 0 };
    streamConfiguration.format = DataFormat::I16;

    OpStatus status = device->StreamSetup(streamConfiguration, moduleIndex);
    if (status != OpStatus::Success)
        return status;

    device->StreamStart(moduleIndex);
    return status;
}

void LMS7002M_SDRDevice_Fixture::DestroySteam()
{
    device->StreamStop(moduleIndex);
    device->StreamDestroy(moduleIndex);
}

TEST_F(LMS7002M_SDRDevice_Fixture, Configure8HalfTestPatternAndReceiveIt)
{
    ASSERT_EQ(SetUpDeviceForRxTestPattern(
                  ChannelConfig::Direction::TestSignal::Scale::Half, ChannelConfig::Direction::TestSignal::Divide::Div8),
        OpStatus::Success);
    ASSERT_EQ(SetupStream(), OpStatus::Success);
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
