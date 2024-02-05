#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "LimeSDR_Mini.h"
#include "limesuite/DeviceRegistry.h"

using namespace lime;
using namespace std::literals::string_literals;

class LimeSDR_Mini_Fixture : public ::testing::Test
{
  protected:
    void SetUp() override
    {
        auto devices = DeviceRegistry::enumerate(DeviceHandle{ deviceHandleHint });

        if (devices.empty())
        {
            GTEST_SKIP() << "LimeSDR-Mini not connected, skipping"s;
        }

        device = DeviceRegistry::makeDevice(devices.at(0));

        ASSERT_NE(device, nullptr);
    }

    void TearDown() override { DeviceRegistry::freeDevice(device); }

    inline static const std::string deviceHandleHint{ "LimeSDR Mini"s };

    SDRDevice* device = nullptr;
};

TEST_F(LimeSDR_Mini_Fixture, ConnectToDevice)
{
}
