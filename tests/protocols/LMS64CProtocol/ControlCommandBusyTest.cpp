#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <chrono>

#include "comms/ISerialPort.h"
#include "protocols/LMS64CProtocol.h"

using namespace lime;
using ::testing::_;
using ::testing::Invoke;
using ::testing::Return;

namespace {

class MockSerialPort : public ISerialPort
{
  public:
    MOCK_METHOD(int, Write, (const uint8_t*, std::size_t, int), (override));
    MOCK_METHOD(int, Read, (uint8_t*, std::size_t, int), (override));
    MOCK_METHOD(OpStatus, RunControlCommand, (uint8_t*, size_t, int), (override));
    MOCK_METHOD(OpStatus, RunControlCommand, (uint8_t*, uint8_t*, size_t, int), (override));
};

} // namespace

// The retry loop used to spin forever when the transport kept reporting
// Busy, hanging the host with no error (issue #203).
TEST(LMS64CProtocol, ControlCommandStopsRetryingBusyAfterTimeout)
{
    MockSerialPort port;
    EXPECT_CALL(port, RunControlCommand(_, _, _, _)).WillRepeatedly(Return(OpStatus::Busy));

    LMS64CProtocol::FirmwareInfo info{};
    const auto start = std::chrono::steady_clock::now();
    const OpStatus status = LMS64CProtocol::GetFirmwareInfo(port, info, 0);
    const auto elapsed = std::chrono::steady_clock::now() - start;

    EXPECT_EQ(status, OpStatus::Busy);
    EXPECT_LT(elapsed, std::chrono::seconds(5));
}

TEST(LMS64CProtocol, ControlCommandRetriesBusyThenSucceeds)
{
    MockSerialPort port;
    EXPECT_CALL(port, RunControlCommand(_, _, _, _))
        .WillOnce(Return(OpStatus::Busy))
        .WillOnce(Invoke([](uint8_t*, uint8_t* response, size_t, int) {
            reinterpret_cast<LMS64CPacket*>(response)->status = LMS64CProtocol::CommandStatus::Completed;
            return OpStatus::Success;
        }));

    LMS64CProtocol::FirmwareInfo info{};
    EXPECT_EQ(LMS64CProtocol::GetFirmwareInfo(port, info, 0), OpStatus::Success);
}
