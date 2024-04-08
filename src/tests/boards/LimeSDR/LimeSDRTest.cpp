#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "tests/boards/LMS7002M_SDRDevice_Fixture.h"

#include <string>

using namespace lime::testing;
using namespace std::literals::string_literals;

class LimeSDR_Fixture : public LMS7002M_SDRDevice_Fixture
{
  public:
    LimeSDR_Fixture()
        : LMS7002M_SDRDevice_Fixture(deviceHandleHint)
    {
    }

  protected:
    inline static const std::string deviceHandleHint{ "LimeSDR-USB"s };
};

TEST_F(LimeSDR_Fixture, ConnectToDevice)
{
}

TEST_F(LimeSDR_Fixture, Configure4HalfTestPatternAndReceiveIt)
{
    Configure4HalfTestPatternAndReceiveIt();
}

TEST_F(LimeSDR_Fixture, Configure4FullTestPatternAndReceiveIt)
{
    Configure4FullTestPatternAndReceiveIt();
}

TEST_F(LimeSDR_Fixture, Configure8HalfTestPatternAndReceiveIt)
{
    Configure8HalfTestPatternAndReceiveIt();
}

TEST_F(LimeSDR_Fixture, Configure8FullTestPatternAndReceiveIt)
{
    Configure8FullTestPatternAndReceiveIt();
}
