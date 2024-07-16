#include "TestEntry.h"
#include <gtest/gtest.h>

using ::testing::InitGoogleTest;

int lime::tests::TestEntry(int argc, char** argv)
{
    InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
