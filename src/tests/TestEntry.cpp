#include "TestEntry.h"
#include <gtest/gtest.h>

using ::testing::InitGoogleTest;

int lime::testing::TestEntry(int argc, char** argv)
{
    InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
