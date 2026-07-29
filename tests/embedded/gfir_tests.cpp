#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "gfir/lms_gfir.h"

// The filter designer used to hand a singular system to nrerror(), which prints to
// stderr and calls exit(1). Any test below that reaches that path takes the whole
// runner down with it, which is the failure this fixture is here to catch.

TEST(gfir, NonFiniteBandEdgesDoNotTerminateTheProcess)
{
    const int tapCount = 120;
    std::vector<float> coefficients(tapCount, 1.0f);

    const float nan = std::numeric_limits<float>::quiet_NaN();
    GenerateFilter(tapCount, nan, nan, 1.0f, 0.0f, coefficients.data());

    // reaching this line at all is the point of the test, a singular system must be
    // reported back rather than ending the process
    for (int i = 0; i < tapCount; ++i)
        EXPECT_FALSE(std::isnan(coefficients[i])) << "tap " << i;
}

TEST(gfir, InfiniteBandEdgesDoNotTerminateTheProcess)
{
    const int tapCount = 60;
    std::vector<float> coefficients(tapCount, 1.0f);

    const float inf = std::numeric_limits<float>::infinity();
    GenerateFilter(tapCount, inf, inf, 1.0f, 0.0f, coefficients.data());

    for (int i = 0; i < tapCount; ++i)
        EXPECT_FALSE(std::isnan(coefficients[i])) << "tap " << i;
}

TEST(gfir, ValidBandEdgesProduceAFiniteSymmetricResponse)
{
    // the tap counts lms7002m_set_gfir_filter can ask for, L is clamped to 8 there
    for (const int tapCount : { 15, 30, 60, 120 })
    {
        std::vector<float> coefficients(tapCount, std::numeric_limits<float>::quiet_NaN());
        const float w = 0.2f;
        GenerateFilter(tapCount, w, w * 1.1f, 1.0f, 0.0f, coefficients.data());

        float energy = 0;
        for (int i = 0; i < tapCount; ++i)
        {
            ASSERT_TRUE(std::isfinite(coefficients[i])) << "tap count " << tapCount << ", tap " << i;
            energy += std::fabs(coefficients[i]);
        }
        EXPECT_GT(energy, 0.0f) << "tap count " << tapCount << " produced an all zero filter";

        // the designer builds the second half from the first by symmetry
        for (int i = 0; i < tapCount / 2; ++i)
            EXPECT_FLOAT_EQ(coefficients[i], coefficients[tapCount - i - 1]) << "tap count " << tapCount << ", tap " << i;
    }
}
