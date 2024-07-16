#pragma once

#include <string>

#include "lime/LimeSuite.h"

#include <iostream>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace lime::tests {

struct RxStreamParams {
    double sampleRate;
    uint32_t oversample;
    size_t channelCount;

    friend std::ostream& operator<<(std::ostream& os, const RxStreamParams& obj)
    {
        os << "FS" << static_cast<uint32_t>(obj.sampleRate) << "_OVR" << obj.oversample << "CHC" << obj.channelCount;
        return os;
    }
};

class LimeSuiteWrapper_streaming : public ::testing::TestWithParam<RxStreamParams>
{
  protected:
    LimeSuiteWrapper_streaming();

    void SetUp() override;
    void TearDown() override;

    int SetupSampleRate();
    void SetupRxStreams(std::vector<lms_stream_t>& channels);
    void ReceiveSamplesVerifySampleRate(std::vector<lms_stream_t>& channels);

  public:
    lms_device_t* device;
};

} // namespace lime::tests
