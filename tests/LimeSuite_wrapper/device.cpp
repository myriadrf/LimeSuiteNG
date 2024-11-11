#include "device.h"

#include <chrono>
#include <thread>

#include "tests/externalData.h"

using namespace std;
using namespace std::literals::string_literals;

namespace lime::testing {

LimeSuiteWrapper_device::LimeSuiteWrapper_device()
{
}

void LimeSuiteWrapper_device::SetUp()
{
    int rez = LMS_Open(&device, lime::testing::GetTestDeviceHandleArgument(), NULL);
    ASSERT_EQ(rez, 0);
    ASSERT_NE(device, nullptr);
    ASSERT_EQ(LMS_Init(device), 0);
}

void LimeSuiteWrapper_device::TearDown()
{
    if (device)
        LMS_Close(device);
}

TEST_F(LimeSuiteWrapper_device, GetChipTemperatureIsInValidRange)
{
    double temp{ 0 };
    ASSERT_EQ(LMS_GetChipTemperature(device, 0, &temp), 0);
    ASSERT_GE(temp, -40);
    ASSERT_LE(temp, 85);
}

} // namespace lime::testing