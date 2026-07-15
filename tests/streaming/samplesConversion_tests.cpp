#include <gtest/gtest.h>

#include "streaming/samplesConversion.h"

#include <vector>

using namespace lime;

// srcCount of 65536 or more used to hang: the loop counter was 16 bit and
// wrapped to 0 before ever reaching the bound (issue #195).
TEST(SamplesConversion, SlowPathConvertHandlesCountsAbove16bitRange)
{
    constexpr uint32_t srcCount = 65537;
    std::vector<complex16_t> src(srcCount);
    std::vector<complex32f_t> dest(srcCount);
    src.front() = complex16_t(-32768, 16384);
    src.back() = complex16_t(16384, -32768);

    slowPath_convert(dest.data(), src.data(), srcCount);

    EXPECT_FLOAT_EQ(dest.front().real(), -1.0f);
    EXPECT_FLOAT_EQ(dest.front().imag(), 0.5f);
    EXPECT_FLOAT_EQ(dest.back().real(), 0.5f);
    EXPECT_FLOAT_EQ(dest.back().imag(), -1.0f);
}
