#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "tests/protocols/SerialPortMock.h"
#include "protocols/LMS64CProtocol.h"
#include "limesuiteng/SDRDevice.h"

using namespace lime;
using namespace lime::testing;
using namespace std::literals::string_literals;
using ::testing::_;
using ::testing::AllOf;
using ::testing::DoAll;
using ::testing::Return;
using ::testing::ReturnArg;
using ::testing::Sequence;
using ::testing::SetArrayArgument;

static constexpr std::size_t PACKET_SIZE = sizeof(LMS64CPacket);

// NOLINTNEXTLINE(cppcoreguidelines-avoid-const-or-ref-data-members)
MATCHER_P(IsCommandCorrect, command, "Checks if the packet has the correct command"sv)
{
    auto packet = reinterpret_cast<const LMS64CPacket*>(arg);

    return packet->cmd == command;
}

// NOLINTNEXTLINE(cppcoreguidelines-avoid-const-or-ref-data-members)
MATCHER_P(IsBlockCountCorrect, blockCount, "Checks if the packet has the correct block count"sv)
{
    auto packet = reinterpret_cast<const LMS64CPacket*>(arg);

    return packet->blockCount == blockCount;
}

// NOLINTNEXTLINE(cppcoreguidelines-avoid-const-or-ref-data-members)
MATCHER_P(IsSubdeviceCorrect, subDevice, "Checks if the packet has the correct subdevice"sv)
{
    auto packet = reinterpret_cast<const LMS64CPacket*>(arg);

    return packet->subDevice == subDevice;
}

TEST(LMS64CProtocol, CustomParameterReadTestEmptyDoesNothing)
{
    SerialPortMock mockPort{};

    uint32_t subdevice = 1U;

    EXPECT_CALL(mockPort, Write(_, PACKET_SIZE, _)).Times(0);
    EXPECT_CALL(mockPort, Read(_, PACKET_SIZE, _)).Times(0);

    std::vector<CustomParameterIO> parameters;
    OpStatus returnValue = LMS64CProtocol::CustomParameterRead(mockPort, parameters, subdevice);

    EXPECT_EQ(returnValue, OpStatus::Success);
}

TEST(LMS64CProtocol, CustomParameterReadTestOneParameter)
{
    SerialPortMock mockPort{};
    LMS64CPacket packet{};
    packet.status = LMS64CProtocol::CommandStatus::Completed;
    packet.blockCount = 1;
    packet.payload[1] = 0x00; // RAW and powerOf10 = 0

    uint16_t value = 5U;
    packet.payload[2] = value >> 8;
    packet.payload[3] = value & 0xFF;

    uint32_t subdevice = 1U;

    ON_CALL(mockPort, Read(_, PACKET_SIZE, _))
        .WillByDefault(DoAll(
            SetArrayArgument<0>(reinterpret_cast<uint8_t*>(&packet), reinterpret_cast<uint8_t*>(&packet + 1)), ReturnArg<1>()));

    EXPECT_CALL(mockPort,
        Write(
            AllOf(IsCommandCorrect(LMS64CProtocol::Command::ANALOG_VAL_RD), IsBlockCountCorrect(1), IsSubdeviceCorrect(subdevice)),
            PACKET_SIZE,
            _))
        .Times(1);
    EXPECT_CALL(mockPort, Read(_, PACKET_SIZE, _)).Times(1);

    std::vector<CustomParameterIO> parameters{ { 16, 0, ""s } };
    OpStatus returnValue = LMS64CProtocol::CustomParameterRead(mockPort, parameters, subdevice);

    EXPECT_EQ(returnValue, OpStatus::Success);
    EXPECT_EQ(parameters[0].value, static_cast<uint16_t>(value));
    EXPECT_EQ(parameters[0].units, ""s);
}

TEST(LMS64CProtocol, CustomParameterReadTestSixteenParameters)
{
    SerialPortMock mockPort{};
    LMS64CPacket packet1{};
    packet1.status = LMS64CProtocol::CommandStatus::Completed;

    uint32_t subdevice = 1U;

    Sequence writeSequence;

    std::vector<CustomParameterIO> parameters(16);

    for (int i = 0; i < 16; i++)
    {
        parameters[i] = { i, 0, ""s };
    }

    ON_CALL(mockPort, Read(_, PACKET_SIZE, _))
        .WillByDefault(DoAll(
            SetArrayArgument<0>(reinterpret_cast<uint8_t*>(&packet1), reinterpret_cast<uint8_t*>(&packet1 + 1)), ReturnArg<1>()));

    EXPECT_CALL(mockPort,
        Write(
            AllOf(IsCommandCorrect(LMS64CProtocol::Command::ANALOG_VAL_RD), IsBlockCountCorrect(14), IsSubdeviceCorrect(subdevice)),
            PACKET_SIZE,
            _))
        .Times(1)
        .InSequence(writeSequence);
    EXPECT_CALL(mockPort,
        Write(
            AllOf(IsCommandCorrect(LMS64CProtocol::Command::ANALOG_VAL_RD), IsBlockCountCorrect(2), IsSubdeviceCorrect(subdevice)),
            PACKET_SIZE,
            _))
        .Times(1)
        .InSequence(writeSequence);

    EXPECT_CALL(mockPort, Read(_, PACKET_SIZE, _)).Times(2);

    OpStatus returnValue = LMS64CProtocol::CustomParameterRead(mockPort, parameters, subdevice);

    EXPECT_EQ(returnValue, OpStatus::Success);
}

TEST(LMS64CProtocol, CustomParameterReadCorrectPrefix)
{
    SerialPortMock mockPort{};
    LMS64CPacket packet{};
    packet.status = LMS64CProtocol::CommandStatus::Completed;
    packet.blockCount = 1;
    packet.payload[1] = 0x21; // CURRENT and powerOf10 = 1

    uint16_t value = 2U;
    packet.payload[2] = value >> 8;
    packet.payload[3] = value & 0xFF;

    uint32_t subdevice = 1U;

    ON_CALL(mockPort, Read(_, PACKET_SIZE, _))
        .WillByDefault(DoAll(
            SetArrayArgument<0>(reinterpret_cast<uint8_t*>(&packet), reinterpret_cast<uint8_t*>(&packet + 1)), ReturnArg<1>()));

    EXPECT_CALL(mockPort,
        Write(
            AllOf(IsCommandCorrect(LMS64CProtocol::Command::ANALOG_VAL_RD), IsBlockCountCorrect(1), IsSubdeviceCorrect(subdevice)),
            PACKET_SIZE,
            _))
        .Times(1);
    EXPECT_CALL(mockPort, Read(_, PACKET_SIZE, _)).Times(1);

    std::vector<CustomParameterIO> parameters{ { 16, 0, ""s } };
    OpStatus returnValue = LMS64CProtocol::CustomParameterRead(mockPort, parameters, subdevice);

    EXPECT_EQ(returnValue, OpStatus::Success);
    EXPECT_EQ(parameters[0].value, value);
    EXPECT_EQ(parameters[0].units, "kA"s);
}

TEST(LMS64CProtocol, CustomParameterReadTemperatureCorrection)
{
    SerialPortMock mockPort{};
    LMS64CPacket packet{};
    packet.status = LMS64CProtocol::CommandStatus::Completed;
    packet.blockCount = 1;
    packet.payload[1] = 0x5F; // TEMPERATURE and powerOf10 = -1

    uint16_t value = 0U;
    packet.payload[2] = value >> 8;
    packet.payload[3] = value & 0xFF;

    uint32_t subdevice = 1U;

    ON_CALL(mockPort, Read(_, PACKET_SIZE, _))
        .WillByDefault(DoAll(
            SetArrayArgument<0>(reinterpret_cast<uint8_t*>(&packet), reinterpret_cast<uint8_t*>(&packet + 1)), ReturnArg<1>()));

    EXPECT_CALL(mockPort,
        Write(
            AllOf(IsCommandCorrect(LMS64CProtocol::Command::ANALOG_VAL_RD), IsBlockCountCorrect(1), IsSubdeviceCorrect(subdevice)),
            PACKET_SIZE,
            _))
        .Times(1);
    EXPECT_CALL(mockPort, Read(_, PACKET_SIZE, _)).Times(1);

    std::vector<CustomParameterIO> parameters{ { 16, 0, ""s } };
    OpStatus returnValue = LMS64CProtocol::CustomParameterRead(mockPort, parameters, subdevice);

    EXPECT_EQ(returnValue, OpStatus::Success);
    EXPECT_EQ(parameters[0].value, value / 10);
    EXPECT_EQ(parameters[0].units, "mC"s);
}

TEST(LMS64CProtocol, CustomParameterReadUnknownUnits)
{
    SerialPortMock mockPort{};
    LMS64CPacket packet{};
    packet.status = LMS64CProtocol::CommandStatus::Completed;
    packet.blockCount = 1;
    packet.payload[1] = 0xFE; // UNKNOWN and powerOf10 = -2

    uint16_t value = 8U;
    packet.payload[2] = value >> 8;
    packet.payload[3] = value & 0xFF;

    uint32_t subdevice = 1U;

    ON_CALL(mockPort, Read(_, PACKET_SIZE, _))
        .WillByDefault(DoAll(
            SetArrayArgument<0>(reinterpret_cast<uint8_t*>(&packet), reinterpret_cast<uint8_t*>(&packet + 1)), ReturnArg<1>()));

    EXPECT_CALL(mockPort,
        Write(
            AllOf(IsCommandCorrect(LMS64CProtocol::Command::ANALOG_VAL_RD), IsBlockCountCorrect(1), IsSubdeviceCorrect(subdevice)),
            PACKET_SIZE,
            _))
        .Times(1);
    EXPECT_CALL(mockPort, Read(_, PACKET_SIZE, _)).Times(1);

    std::vector<CustomParameterIO> parameters{ { 16, 0, ""s } };
    OpStatus returnValue = LMS64CProtocol::CustomParameterRead(mockPort, parameters, subdevice);

    EXPECT_EQ(returnValue, OpStatus::Success);
    EXPECT_EQ(parameters[0].value, value);
    EXPECT_EQ(parameters[0].units, "u unknown"s);
}

TEST(LMS64CProtocol, CustomParameterReadNotFullyWritten)
{
    SerialPortMock mockPort{};

    uint32_t subdevice = 1U;

    ON_CALL(mockPort, Write(_, PACKET_SIZE, _)).WillByDefault(Return(0));

    EXPECT_CALL(mockPort,
        Write(
            AllOf(IsCommandCorrect(LMS64CProtocol::Command::ANALOG_VAL_RD), IsBlockCountCorrect(1), IsSubdeviceCorrect(subdevice)),
            PACKET_SIZE,
            _))
        .Times(1);
    EXPECT_CALL(mockPort, Read(_, PACKET_SIZE, _)).Times(0);

    std::vector<CustomParameterIO> parameters{ { 16, 0, ""s } };

    EXPECT_THROW(LMS64CProtocol::CustomParameterRead(mockPort, parameters, subdevice);, std::runtime_error);
}

TEST(LMS64CProtocol, CustomParameterReadNotFullyRead)
{
    SerialPortMock mockPort{};

    uint32_t subdevice = 1U;

    ON_CALL(mockPort, Read(_, PACKET_SIZE, _)).WillByDefault(Return(0));

    EXPECT_CALL(mockPort,
        Write(
            AllOf(IsCommandCorrect(LMS64CProtocol::Command::ANALOG_VAL_RD), IsBlockCountCorrect(1), IsSubdeviceCorrect(subdevice)),
            PACKET_SIZE,
            _))
        .Times(1);
    EXPECT_CALL(mockPort, Read(_, PACKET_SIZE, _)).Times(1);

    std::vector<CustomParameterIO> parameters{ { 16, 0, ""s } };

    EXPECT_THROW(LMS64CProtocol::CustomParameterRead(mockPort, parameters, subdevice);, std::runtime_error);
}

TEST(LMS64CProtocol, CustomParameterReadWrongStatus)
{
    SerialPortMock mockPort{};
    LMS64CPacket packet{};
    packet.status = LMS64CProtocol::CommandStatus::TooManyBlocks;

    uint32_t subdevice = 1U;

    ON_CALL(mockPort, Read(_, PACKET_SIZE, _))
        .WillByDefault(DoAll(
            SetArrayArgument<0>(reinterpret_cast<uint8_t*>(&packet), reinterpret_cast<uint8_t*>(&packet + 1)), ReturnArg<1>()));

    EXPECT_CALL(mockPort,
        Write(
            AllOf(IsCommandCorrect(LMS64CProtocol::Command::ANALOG_VAL_RD), IsBlockCountCorrect(1), IsSubdeviceCorrect(subdevice)),
            PACKET_SIZE,
            _))
        .Times(1);
    EXPECT_CALL(mockPort, Read(_, PACKET_SIZE, _)).Times(1);

    std::vector<CustomParameterIO> parameters{ { 16, 0, ""s } };

    EXPECT_THROW(LMS64CProtocol::CustomParameterRead(mockPort, parameters, subdevice);, std::runtime_error);
}
