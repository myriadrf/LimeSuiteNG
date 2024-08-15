#pragma once

#include <string>

#include "lime/LimeSuite.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace lime::testing {

class LimeSuiteWrapper_device : public ::testing::Test
{
  protected:
    LimeSuiteWrapper_device();

    void SetUp() override;
    void TearDown() override;

  public:
    lms_device_t* device = nullptr;
};

} // namespace lime::testing