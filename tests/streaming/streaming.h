#pragma once

#include <string>

#include "limesuiteng/SDRDevice.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace lime::testing {

class SDRDevice_streaming : public ::testing::Test
{
  protected:
    SDRDevice_streaming();

    void SetUp() override;
    void TearDown() override;

  public:
    SDRDevice* device;
    int channelCount;
    int sampleRate;
    int moduleIndex;
};

} // namespace lime::testing