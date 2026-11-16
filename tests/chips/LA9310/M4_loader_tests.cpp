#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <vector>

#include "chips/LA9310/M4_loader.h"

using namespace lime;

namespace {

constexpr uint32_t PREAMBLE = 0xAA55AA55;
constexpr uint32_t TCML = 0x1F800000;
constexpr uint32_t BYPASS_PLUGIN = 1u << 16;
constexpr uint32_t BYPASS_EDMA = 1u << 0;

// Same as the images produced by the firmware build
M4_BootHeader ValidHeader(uint32_t imageSize)
{
    return M4_BootHeader{ PREAMBLE, 0, 0, imageSize, 0, TCML, TCML + 0x201, BYPASS_PLUGIN };
}

std::vector<char> ImageWithHeader(const M4_BootHeader& header, size_t size)
{
    std::vector<char> image(size);
    std::memcpy(image.data(), &header, sizeof(header));
    return image;
}

} // namespace

TEST(M4_loader, RawImageRunsFromTCML)
{
    const std::vector<char> image(1024, 0x11);

    M4_BootHeader header;
    ASSERT_EQ(ParseM4Firmware(image, header), OpStatus::Success);

    EXPECT_EQ(header.preamble, PREAMBLE);
    EXPECT_EQ(header.plugin_size, 0u);
    EXPECT_EQ(header.plugin_offset, 0u);
    EXPECT_EQ(header.bl_size, 1024u);
    EXPECT_EQ(header.bl_src_offset, 0u);
    EXPECT_EQ(header.bl_dest, TCML);
    EXPECT_EQ(header.bl_entry, TCML);
    EXPECT_EQ(header.reserved, BYPASS_PLUGIN | BYPASS_EDMA);
}

TEST(M4_loader, RawImageShorterThanPreamble)
{
    const std::vector<char> image(2, 0x11);

    M4_BootHeader header;
    ASSERT_EQ(ParseM4Firmware(image, header), OpStatus::Success);
    EXPECT_EQ(header.bl_size, 2u);
}

TEST(M4_loader, ImageProvidesItsOwnHeader)
{
    const std::vector<char> image = ImageWithHeader(ValidHeader(1024), 1024);

    M4_BootHeader header;
    ASSERT_EQ(ParseM4Firmware(image, header), OpStatus::Success);

    EXPECT_EQ(header.bl_size, 1024u);
    EXPECT_EQ(header.bl_src_offset, 0u);
    EXPECT_EQ(header.bl_dest, TCML);
    EXPECT_EQ(header.bl_entry, TCML + 0x201);
}

TEST(M4_loader, BootCodeIsAlwaysToldToBypassEDMA)
{
    M4_BootHeader source = ValidHeader(1024);
    source.reserved = 0;
    const std::vector<char> image = ImageWithHeader(source, 1024);

    M4_BootHeader header;
    ASSERT_EQ(ParseM4Firmware(image, header), OpStatus::Success);
    EXPECT_EQ(header.reserved, BYPASS_EDMA);
}

TEST(M4_loader, RejectsEmptyImage)
{
    M4_BootHeader header;
    EXPECT_EQ(ParseM4Firmware({}, header), OpStatus::InvalidValue);
}

TEST(M4_loader, AcceptsImageThatFillsTCML)
{
    const std::vector<char> image(0x20000, 0x11);

    M4_BootHeader header;
    EXPECT_EQ(ParseM4Firmware(image, header), OpStatus::Success);
}

TEST(M4_loader, RejectsImageLargerThanTCML)
{
    const std::vector<char> image(0x20001, 0x11);

    M4_BootHeader header;
    EXPECT_EQ(ParseM4Firmware(image, header), OpStatus::InvalidValue);
}

TEST(M4_loader, RejectsTruncatedHeader)
{
    std::vector<char> image(sizeof(M4_BootHeader) - 1);
    std::memcpy(image.data(), &PREAMBLE, sizeof(PREAMBLE));

    M4_BootHeader header;
    EXPECT_EQ(ParseM4Firmware(image, header), OpStatus::InvalidValue);
}

TEST(M4_loader, RejectsFirmwareLargerThanImage)
{
    const std::vector<char> image = ImageWithHeader(ValidHeader(1025), 1024);

    M4_BootHeader header;
    EXPECT_EQ(ParseM4Firmware(image, header), OpStatus::InvalidValue);
}

TEST(M4_loader, RejectsFirmwareOffsetThatOverflows)
{
    M4_BootHeader source = ValidHeader(1024);
    source.bl_src_offset = 0xFFFFFFFF;
    const std::vector<char> image = ImageWithHeader(source, 1024);

    M4_BootHeader header;
    EXPECT_EQ(ParseM4Firmware(image, header), OpStatus::InvalidValue);
}

TEST(M4_loader, RejectsPluginOutOfBounds)
{
    M4_BootHeader source = ValidHeader(1024);
    source.plugin_size = 16;
    source.plugin_offset = 1020;
    const std::vector<char> image = ImageWithHeader(source, 1024);

    M4_BootHeader header;
    EXPECT_EQ(ParseM4Firmware(image, header), OpStatus::InvalidValue);
}

TEST(M4_loader, BootImageWithHeaderIsAccepted)
{
    const std::vector<char> image = ImageWithHeader(ValidHeader(1024), 1024);

    M4_BootHeader header;
    ASSERT_EQ(ParseM4BootImage(image, 1024, header), OpStatus::Success);
    EXPECT_EQ(header.bl_size, 1024u);
    EXPECT_EQ(header.bl_entry, TCML + 0x201);
}

TEST(M4_loader, BootImageRejectsRawBinary)
{
    const std::vector<char> image(1024, 0x11);

    M4_BootHeader header;
    EXPECT_EQ(ParseM4BootImage(image, 65532, header), OpStatus::InvalidValue);
}

TEST(M4_loader, BootImageRejectsImageShorterThanPreamble)
{
    const std::vector<char> image(2, 0x55);

    M4_BootHeader header;
    EXPECT_EQ(ParseM4BootImage(image, 65532, header), OpStatus::InvalidValue);
}

TEST(M4_loader, BootImageRejectsEmptyImage)
{
    M4_BootHeader header;
    EXPECT_EQ(ParseM4BootImage({}, 65532, header), OpStatus::InvalidValue);
}

TEST(M4_loader, BootImageRejectsImageLargerThanMemory)
{
    const std::vector<char> image = ImageWithHeader(ValidHeader(1024), 1025);

    M4_BootHeader header;
    EXPECT_EQ(ParseM4BootImage(image, 1024, header), OpStatus::OutOfRange);
}

TEST(M4_loader, BootImageRejectsFirmwareOutOfBounds)
{
    const std::vector<char> image = ImageWithHeader(ValidHeader(1025), 1024);

    M4_BootHeader header;
    EXPECT_EQ(ParseM4BootImage(image, 65532, header), OpStatus::InvalidValue);
}

TEST(M4_loader, BootImageRejectsEmptyFirmware)
{
    const std::vector<char> image = ImageWithHeader(ValidHeader(0), 1024);

    M4_BootHeader header;
    EXPECT_EQ(ParseM4BootImage(image, 65532, header), OpStatus::InvalidValue);
}
