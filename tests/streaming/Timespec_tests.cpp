#include <gtest/gtest.h>

#include "limesuiteng/Timespec.h"

using namespace lime;

TEST(Timespec, Addition)
{
    Timespec a(2, 0.12345);
    Timespec b(103, 0.57893);
    Timespec c = a + b;
    EXPECT_EQ(c.GetSeconds(), 105);
    EXPECT_FLOAT_EQ(c.GetFracSeconds(), 0.70238);
}

TEST(Timespec, Addition_FractionalOverflow)
{
    Timespec a(1, 0.456);
    Timespec b(2, 0.789);
    Timespec c = a + b;
    EXPECT_EQ(c.GetSeconds(), 4);
    EXPECT_FLOAT_EQ(c.GetFracSeconds(), 0.245);
}

TEST(Timespec, Addition_LargeNumbers)
{
    Timespec a(100000000000, 0.123456789);
    Timespec b(300000000001, 0.234567890);
    Timespec c = a + b;
    EXPECT_EQ(c.GetSeconds(), 400000000001);
    EXPECT_FLOAT_EQ(c.GetFracSeconds(), 0.358024679);
}

TEST(Timespec, Addition_NegativeTime)
{
    Timespec a(1, 0.123456789);
    Timespec b(-2, -0.234567890);
    Timespec c = a + b;
    EXPECT_EQ(c.GetSeconds(), -1);
    EXPECT_FLOAT_EQ(c.GetFracSeconds(), -0.111111101);
}

TEST(Timespec, Subtraction)
{
    Timespec a(4, 0.789);
    Timespec b(1, 0.456);
    Timespec c = a - b;
    EXPECT_EQ(c.GetSeconds(), 3);
    EXPECT_FLOAT_EQ(c.GetFracSeconds(), 0.333);
}

TEST(Timespec, Subtraction_FractionalUnderflow)
{
    Timespec a(4, 0.123);
    Timespec b(1, 0.456);
    Timespec c = a - b;
    EXPECT_EQ(c.GetSeconds(), 2);
    EXPECT_FLOAT_EQ(c.GetFracSeconds(), 0.667);
}

TEST(Timespec, Subtraction_FractionalUnderflow_IntoNegative)
{
    Timespec a(1, 0.123);
    Timespec b(3, 0.456);
    Timespec c = a - b;
    EXPECT_EQ(c.GetSeconds(), -2);
    EXPECT_FLOAT_EQ(c.GetFracSeconds(), -0.333);
}

TEST(Timespec, Subtraction_LargeNumbers)
{
    Timespec a(100000000000, 0.123456789);
    Timespec b(300000000001, 0.234567890);
    Timespec c = a - b;
    EXPECT_EQ(c.GetSeconds(), -200000000001);
    EXPECT_FLOAT_EQ(c.GetFracSeconds(), -0.111111101);
}

TEST(Timespec, Compare_lesser)
{
    Timespec a(1, 0.456);
    Timespec b(4, 0.123);
    EXPECT_TRUE(a < b);

    Timespec c(4, 0.123);
    Timespec d(4, 0.456);
    EXPECT_TRUE(c < d);
}

TEST(Timespec, Compare_greater)
{
    Timespec a(4, 0.123);
    Timespec b(1, 0.456);
    EXPECT_TRUE(a > b);

    Timespec c(4, 0.456);
    Timespec d(4, 0.123);
    EXPECT_TRUE(c > d);
}
