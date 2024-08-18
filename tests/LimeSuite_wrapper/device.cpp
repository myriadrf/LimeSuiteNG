#include "device.h"

#include <chrono>
#include <thread>

using namespace std;
using namespace std::literals::string_literals;

namespace lime::testing {

LimeSuiteWrapper_device::LimeSuiteWrapper_device()
{
}

void LimeSuiteWrapper_device::SetUp()
{
    int n;
    lms_info_str_t list[16]; //should be large enough to hold all detected devices
    if ((n = LMS_GetDeviceList(list)) <= 0) //NULL can be passed to only get number of devices
        GTEST_SKIP();

    //open the first device
    int rez = LMS_Open(&device, list[0], NULL);
    ASSERT_EQ(rez, 0);

    rez = LMS_Init(device);
    ASSERT_EQ(rez, 0);
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