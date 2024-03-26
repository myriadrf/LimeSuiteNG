#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "LimeSDR_X3.h"
#include "limesuite/DeviceRegistry.h"

using namespace lime;
using namespace std::literals::string_literals;

class LimeSDR_X3_Fixture : public ::testing::Test
{
  protected:
    void SetUp() override
    {
        auto devices = DeviceRegistry::enumerate(DeviceHandle{ deviceHandleHint });

        if (devices.empty())
        {
            GTEST_SKIP() << "LimeSDR-X3 not connected, skipping"s;
        }

        device = DeviceRegistry::makeDevice(DeviceHandle{ deviceHandleHint });

        ASSERT_NE(device, nullptr);
    }

    void TearDown() override { DeviceRegistry::freeDevice(device); }

    inline static const std::string deviceHandleHint{ "LimeX30"s };

    SDRDevice* device = nullptr;
};

TEST_F(LimeSDR_X3_Fixture, ConnectToDevice)
{
}
