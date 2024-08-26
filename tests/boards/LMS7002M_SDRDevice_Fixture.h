#ifndef LIMESUITENG_LMS7002M_SDRDEVICE_FIXTURE_H
#define LIMESUITENG_LMS7002M_SDRDEVICE_FIXTURE_H

#define _USE_MATH_DEFINES

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "limesuiteng/DeviceRegistry.h"
#include "limesuiteng/SDRDevice.h"

#include <cmath>
#include <complex>
#include <initializer_list>

// Allow for a one or two bit offset (in 12-bit format) in calculation in the samples.
constexpr bool EqualOrOffsetBy(int16_t target, int16_t actual, std::initializer_list<int16_t> deltas)
{
    return target == actual || std::any_of(deltas.begin(), deltas.end(), [target, actual](const int16_t value) {
        return std::abs(target - actual) == value;
    });
}

MATCHER_P(AreSamplesCorrect, divide, "Checks if the test pattern gave the correct samples")
{
    auto nextExpectedSample = arg.at(0);
    const int divideBy = divide == lime::ChannelConfig::Direction::TestSignal::Divide::Div4 ? 4 : 8;

    for (const auto& sample : arg)
    {
        if (!EqualOrOffsetBy(sample.real(), nextExpectedSample.real(), { 16, 32 }) ||
            !EqualOrOffsetBy(sample.imag(), nextExpectedSample.imag(), { 16, 32 }))
        {
            return false;
        }

        std::complex<float> complex{ static_cast<float>(sample.real()), static_cast<float>(sample.imag()) };
        auto angle = std::arg(complex);
        angle += 2 * M_PI / divideBy;

        auto magnitude = std::abs(complex);

        EXPECT_NE(magnitude, 0);
        if (magnitude == 0.0)
            return false;

        complex = std::polar(magnitude, angle);

        // Truncate to a 12-bit number bit-shifted left by 4
        nextExpectedSample.real(static_cast<int16_t>(std::round(complex.real())) & 0xFFF0);
        nextExpectedSample.imag(static_cast<int16_t>(std::round(complex.imag())) & 0xFFF0);
    }

    return true;
}

namespace lime::testing {

class LMS7002M_SDRDevice_Fixture : public ::testing::Test
{
  protected:
    LMS7002M_SDRDevice_Fixture();
    LMS7002M_SDRDevice_Fixture(const std::string& deviceHandleHint);

    void SetUp() override;

    void TearDown() override;

    OpStatus SetUpDeviceForRxTestPattern(
        ChannelConfig::Direction::TestSignal::Scale scale, ChannelConfig::Direction::TestSignal::Divide divide);

    OpStatus SetupStream();
    void DestroySteam();

    void Configure4HalfTestPatternAndReceiveIt();
    void Configure4FullTestPatternAndReceiveIt();
    void Configure8HalfTestPatternAndReceiveIt();
    void Configure8FullTestPatternAndReceiveIt();

  protected:
    std::string deviceHandleHint;
    uint8_t moduleIndex{ 0 };
    bool configValid{ false };

    SDRDevice* device = nullptr;
};

} // namespace lime::testing

#endif // LIMESUITENG_LMS7002M_SDRDEVICE_FIXTURE_H
