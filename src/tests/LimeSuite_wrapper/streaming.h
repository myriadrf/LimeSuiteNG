#pragma once

#include <string>

#include "lime/LimeSuite.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace lime::testing {

class LimeSuiteWrapper_streaming : public ::testing::Test
{
  protected:
    LimeSuiteWrapper_streaming();

    void SetUp() override;
    void TearDown() override;

  public:
    lms_device_t* device = nullptr;
    int channelCount;
    int sampleRate;
};

} // namespace lime::testing