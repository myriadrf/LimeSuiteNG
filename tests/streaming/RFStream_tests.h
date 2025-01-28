#pragma once

#include <string>

#include "limesuiteng/SDRDevice.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace lime::testing {

class RFStream_tests : public ::testing::Test
{
  protected:
    RFStream_tests();

    void SetUp() override;
    void TearDown() override;

  public:
    SDRDevice* device;
    std::unique_ptr<RFStream> stream;
    int channelCount;
    int sampleRate;
    int moduleIndex;
};

} // namespace lime::testing