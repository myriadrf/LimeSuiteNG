#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "DSP/FFT/FFT.h"
#include "limesuiteng/complex.h"

using namespace lime;

namespace lime::testing {

// The window coefficients are normalized so that their mean is 1, which makes
// their sum the coefficient count regardless of the window shape.
TEST(FFT, WindowCoefficientsAreAmplitudeCorrected)
{
    const std::vector<FFT::WindowFunctionType> types{
        FFT::WindowFunctionType::BLACKMAN_HARRIS, FFT::WindowFunctionType::HAMMING, FFT::WindowFunctionType::HANNING
    };

    for (auto type : types)
    {
        for (std::size_t count : { 64u, 1024u, 16384u })
        {
            std::vector<float> coeffs;
            FFT::GenerateWindowCoefficients(type, count, coeffs);
            ASSERT_EQ(coeffs.size(), count);

            double sum = 0;
            for (float c : coeffs)
                sum += c;

            // the tolerance only has to absorb float rounding of the coefficients. It stays
            // far below the constant offset (~2.7) that a missing accumulator reset causes,
            // so the largest size is still covered.
            EXPECT_NEAR(sum, static_cast<double>(count), count * 1e-5 + 1e-3)
                << "window type " << static_cast<int>(type) << ", count " << count;
        }
    }
}

// A constant input has all its energy in bin 0. With the normalization applied
// by Calc that bin has to come out as 1.0 for every transform size.
TEST(FFT, DcBinIsNormalizedForLargeTransforms)
{
    for (int fftSize : { 1024, 65536 })
    {
        const std::vector<complex32f_t> samples(fftSize, complex32f_t{ 1.0f, 0.0f });
        const std::vector<float> bins = FFT::Calc(samples, FFT::WindowFunctionType::NONE);

        ASSERT_EQ(static_cast<int>(bins.size()), fftSize);
        EXPECT_TRUE(std::isfinite(bins[0])) << "fftSize " << fftSize;
        EXPECT_NEAR(bins[0], 1.0f, 1e-3) << "fftSize " << fftSize;
    }
}

} // namespace lime::testing
