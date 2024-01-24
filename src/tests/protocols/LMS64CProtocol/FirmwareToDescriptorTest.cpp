#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "LMS64CProtocol.h"

using namespace lime;
using namespace std::literals::string_literals;

TEST(LMS64CProtocol, FirmwareToDescriptorSetsInfo)
{
    LMS64CProtocol::FirmwareInfo firmwareInfo{ 1, 3, 2, 4, 5, 6 };
    SDRDevice::Descriptor descriptor;

    LMS64CProtocol::FirmwareToDescriptor(firmwareInfo, descriptor);

    EXPECT_EQ(descriptor.name, "EVB6"s);
    EXPECT_EQ(descriptor.expansionName, "Myriad1"s);
    EXPECT_EQ(descriptor.firmwareVersion, "2"s);
    EXPECT_EQ(descriptor.hardwareVersion, "4"s);
    EXPECT_EQ(descriptor.protocolVersion, "5"s);
    EXPECT_EQ(descriptor.serialNumber, 6);
}

TEST(LMS64CProtocol, FirmwareToDescriptorUnknownsLow)
{
    LMS64CProtocol::FirmwareInfo firmwareInfo{ 0, 0, 0, 0, 0, 0 };
    SDRDevice::Descriptor descriptor;

    LMS64CProtocol::FirmwareToDescriptor(firmwareInfo, descriptor);

    EXPECT_EQ(descriptor.name, "UNKNOWN"s);
    EXPECT_EQ(descriptor.expansionName, "UNKNOWN"s);
    EXPECT_EQ(descriptor.firmwareVersion, "0"s);
    EXPECT_EQ(descriptor.hardwareVersion, "0"s);
    EXPECT_EQ(descriptor.protocolVersion, "0"s);
    EXPECT_EQ(descriptor.serialNumber, 0);
}

TEST(LMS64CProtocol, FirmwareToDescriptorUnknownsHigh)
{
    LMS64CProtocol::FirmwareInfo firmwareInfo{ 2000000, 2000001, 2000002, 2000003, 2000004, 2000005 };
    SDRDevice::Descriptor descriptor;

    LMS64CProtocol::FirmwareToDescriptor(firmwareInfo, descriptor);

    EXPECT_EQ(descriptor.name, "Unknown (0x1E8480)"s);
    EXPECT_EQ(descriptor.expansionName, "Unknown (0x1E8481)"s);
    EXPECT_EQ(descriptor.firmwareVersion, "2000002"s);
    EXPECT_EQ(descriptor.hardwareVersion, "2000003"s);
    EXPECT_EQ(descriptor.protocolVersion, "2000004"s);
    EXPECT_EQ(descriptor.serialNumber, 2000005);
}
