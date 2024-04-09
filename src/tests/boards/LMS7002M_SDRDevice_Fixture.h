#define _USE_MATH_DEFINES

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "limesuiteng/DeviceRegistry.h"
#include "limesuiteng/SDRDevice.h"

#include <cmath>
#include <complex>

using namespace std::literals::string_literals;

// Allow for a one or two bit offset (in 12-bit format) in calculation in the samples.
constexpr bool EqualOrOffsetBy(int16_t target, int16_t actual, int16_t allowedExactDelta1, int16_t allowedExactDelta2)
{
    return target == actual || std::abs(target - actual) == allowedExactDelta1 || std::abs(target - actual) == allowedExactDelta2;
}

MATCHER_P(AreSamplesCorrect, divide, "Checks if the test pattern gave the correct samples"s)
{
    auto nextExpectedSample = arg.at(0);
    const int divideBy = divide == lime::ChannelConfig::Direction::TestSignal::Divide::Div4 ? 4 : 8;

    for (const auto& sample : arg)
    {
        if (!EqualOrOffsetBy(sample.real(), nextExpectedSample.real(), 16, 32) ||
            !EqualOrOffsetBy(sample.imag(), nextExpectedSample.imag(), 16, 32))
        {
            return false;
        }

        std::complex<float> complex{ static_cast<float>(sample.real()), static_cast<float>(sample.imag()) };
        auto angle = std::arg(complex);
        angle += 2 * M_PI / divideBy;

        auto magnitude = std::abs(complex);
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
    LMS7002M_SDRDevice_Fixture(const std::string& deviceHandleHint);

    void SetUp() override;

    void TearDown() override;

    void SetUpDeviceForTestPattern(
        ChannelConfig::Direction::TestSignal::Scale scale, ChannelConfig::Direction::TestSignal::Divide divide);

    void Configure4HalfTestPatternAndReceiveIt();
    void Configure4FullTestPatternAndReceiveIt();
    void Configure8HalfTestPatternAndReceiveIt();
    void Configure8FullTestPatternAndReceiveIt();

  private:
    std::string deviceHandleHint;

    SDRDevice* device = nullptr;
};

} // namespace lime::testing