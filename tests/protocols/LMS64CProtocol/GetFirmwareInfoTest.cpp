#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "tests/protocols/SerialPortMock.h"
#include "protocols/LMS64CProtocol.h"

using namespace lime;
using namespace lime::testing;
using ::testing::_;
using ::testing::AllOf;
using ::testing::DoAll;
using ::testing::Return;
using ::testing::ReturnArg;
using ::testing::SetArrayArgument;

static constexpr std::size_t PACKET_SIZE = sizeof(LMS64CPacket);

// NOLINTNEXTLINE(cppcoreguidelines-avoid-const-or-ref-data-members)
MATCHER_P(IsCommandCorrect, command, "Checks if the packet has the correct command"sv)
{
    auto packet = reinterpret_cast<const LMS64CPacket*>(arg);

    return packet->cmd == command;
}

// NOLINTNEXTLINE(cppcoreguidelines-avoid-const-or-ref-data-members)
MATCHER_P(IsSubdeviceCorrect, subDevice, "Checks if the packet has the correct subdevice"sv)
{
    auto packet = reinterpret_cast<const LMS64CPacket*>(arg);

    return packet->subDevice == subDevice;
}

TEST(LMS64CProtocol, GetFirmwareInfoGetsInfo)
{
    SerialPortMock mockPort{};
    LMS64CPacket packet{};
    packet.status = LMS64CProtocol::CommandStatus::Completed;

    LMS64CProtocol::FirmwareInfo info{};

    uint32_t subdevice = 1U;

    ON_CALL(mockPort, Read(_, PACKET_SIZE, _))
        .WillByDefault(DoAll(
            SetArrayArgument<0>(reinterpret_cast<uint8_t*>(&packet), reinterpret_cast<uint8_t*>(&packet + 1)), ReturnArg<1>()));

    EXPECT_CALL(
        mockPort, Write(AllOf(IsCommandCorrect(LMS64CProtocol::Command::GET_INFO), IsSubdeviceCorrect(subdevice)), PACKET_SIZE, _))
        .Times(1);
    EXPECT_CALL(mockPort, Read(_, PACKET_SIZE, _)).Times(1);

    OpStatus returnValue = LMS64CProtocol::GetFirmwareInfo(mockPort, info, subdevice);

    EXPECT_EQ(returnValue, OpStatus::Success);
}

TEST(LMS64CProtocol, GetFirmwareInfoNotFullyWritten)
{
    SerialPortMock mockPort{};
    LMS64CProtocol::FirmwareInfo info{};

    uint32_t subdevice = 1U;

    ON_CALL(mockPort, Write(_, PACKET_SIZE, _)).WillByDefault(Return(0));

    EXPECT_CALL(
        mockPort, Write(AllOf(IsCommandCorrect(LMS64CProtocol::Command::GET_INFO), IsSubdeviceCorrect(subdevice)), PACKET_SIZE, _))
        .Times(1);
    EXPECT_CALL(mockPort, Read(_, PACKET_SIZE, _)).Times(0);

    OpStatus returnValue = LMS64CProtocol::GetFirmwareInfo(mockPort, info, subdevice);

    EXPECT_EQ(returnValue, OpStatus::IOFailure);
}

TEST(LMS64CProtocol, GetFirmwareInfoNotFullyRead)
{
    SerialPortMock mockPort{};
    LMS64CProtocol::FirmwareInfo info{};

    uint32_t subdevice = 1U;

    ON_CALL(mockPort, Read(_, PACKET_SIZE, _)).WillByDefault(Return(0));

    EXPECT_CALL(
        mockPort, Write(AllOf(IsCommandCorrect(LMS64CProtocol::Command::GET_INFO), IsSubdeviceCorrect(subdevice)), PACKET_SIZE, _))
        .Times(1);
    EXPECT_CALL(mockPort, Read(_, PACKET_SIZE, _)).Times(1);

    OpStatus returnValue = LMS64CProtocol::GetFirmwareInfo(mockPort, info, subdevice);

    EXPECT_EQ(returnValue, OpStatus::IOFailure);
}

TEST(LMS64CProtocol, GetFirmwareInfoWrongStatus)
{
    SerialPortMock mockPort{};
    LMS64CPacket packet{};
    packet.status = LMS64CProtocol::CommandStatus::ResourceDenied;

    LMS64CProtocol::FirmwareInfo info{};

    uint32_t subdevice = 1U;

    ON_CALL(mockPort, Read(_, PACKET_SIZE, _))
        .WillByDefault(DoAll(
            SetArrayArgument<0>(reinterpret_cast<uint8_t*>(&packet), reinterpret_cast<uint8_t*>(&packet + 1)), ReturnArg<1>()));

    EXPECT_CALL(
        mockPort, Write(AllOf(IsCommandCorrect(LMS64CProtocol::Command::GET_INFO), IsSubdeviceCorrect(subdevice)), PACKET_SIZE, _))
        .Times(1);
    EXPECT_CALL(mockPort, Read(_, PACKET_SIZE, _)).Times(1);

    OpStatus returnValue = LMS64CProtocol::GetFirmwareInfo(mockPort, info, subdevice);

    EXPECT_EQ(returnValue, OpStatus::IOFailure);
}
