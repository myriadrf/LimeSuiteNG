#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "chips/LA9310/vspa/vspa_loader.h"

namespace {

constexpr uint32_t DMEM = 0u << 8;
constexpr uint32_t PRAM = 2u << 8;
constexpr uint32_t NOBITS = 8;
constexpr uint32_t PROGBITS = 1;

struct TestSection {
    std::string name;
    uint32_t address;
    std::string contents;
    uint32_t type = PROGBITS;
};

template<typename T> void Put(std::vector<char>& image, size_t offset, T value)
{
    std::memcpy(image.data() + offset, &value, sizeof(T));
}

// Lays out: ELF header, program headers, section contents, section names, section headers
std::vector<char> BuildImage(const std::vector<TestSection>& sections, const std::vector<std::pair<uint32_t, uint32_t>>& segments)
{
    constexpr size_t headerSize = 52;
    constexpr size_t phSize = 32;
    constexpr size_t shSize = 40;

    std::vector<char> image(headerSize + segments.size() * phSize);
    std::vector<uint32_t> offsets;
    for (const TestSection& section : sections)
    {
        offsets.push_back(image.size());
        image.insert(image.end(), section.contents.begin(), section.contents.end());
    }

    const uint32_t namesOffset = image.size();
    std::string names(1, '\0');
    std::vector<uint32_t> nameOffsets;
    for (const TestSection& section : sections)
    {
        nameOffsets.push_back(names.size());
        names += section.name;
        names += '\0';
    }
    const uint32_t shstrtabName = names.size();
    names += ".shstrtab";
    names += '\0';
    image.insert(image.end(), names.begin(), names.end());

    const uint32_t shoff = image.size();
    const uint16_t shnum = sections.size() + 1;
    image.resize(image.size() + shnum * shSize);

    std::memcpy(image.data(),
        "\x7F"
        "ELF\x01\x01\x01",
        7);
    Put<uint16_t>(image, 16, 2);
    Put<uint16_t>(image, 18, 0x40C8);
    Put<uint32_t>(image, 28, headerSize);
    Put<uint32_t>(image, 32, shoff);
    Put<uint16_t>(image, 40, headerSize);
    Put<uint16_t>(image, 42, phSize);
    Put<uint16_t>(image, 44, segments.size());
    Put<uint16_t>(image, 46, shSize);
    Put<uint16_t>(image, 48, shnum);
    Put<uint16_t>(image, 50, shnum - 1);

    for (size_t i = 0; i < segments.size(); ++i)
    {
        const size_t ph = headerSize + i * phSize;
        Put<uint32_t>(image, ph + 8, segments[i].first);
        Put<uint32_t>(image, ph + 12, segments[i].second);
    }

    for (size_t i = 0; i < sections.size(); ++i)
    {
        const size_t sh = shoff + i * shSize;
        Put<uint32_t>(image, sh + 0, nameOffsets[i]);
        Put<uint32_t>(image, sh + 4, sections[i].type);
        Put<uint32_t>(image, sh + 12, sections[i].address);
        Put<uint32_t>(image, sh + 16, offsets[i]);
        Put<uint32_t>(image, sh + 20, sections[i].contents.size());
    }
    const size_t strtabHeader = shoff + sections.size() * shSize;
    Put<uint32_t>(image, strtabHeader + 0, shstrtabName);
    Put<uint32_t>(image, strtabHeader + 4, 3);
    Put<uint32_t>(image, strtabHeader + 16, namesOffset);
    Put<uint32_t>(image, strtabHeader + 20, names.size());

    return image;
}

std::string ToString(const vspa_eld_section& section)
{
    return std::string(reinterpret_cast<const char*>(section.data), section.size);
}

enum class Result { Success, InvalidValue };

// Collects all the loadable sections, the way the loader iterates over them
Result ParseVSPAFirmware(const std::vector<char>& image, std::vector<vspa_eld_section>& sections)
{
    sections.clear();
    vspa_eld_parser parser;
    int status = vspa_eld_open(&parser, image.data(), image.size());
    vspa_eld_section section;
    while (status == VSPA_LOADER_OK && (status = vspa_eld_next(&parser, &section)) == 1)
    {
        sections.push_back(section);
        status = VSPA_LOADER_OK;
    }
    if (status != VSPA_LOADER_OK || sections.empty())
        return Result::InvalidValue;
    return Result::Success;
}

} // namespace

TEST(VSPA_loader, ReturnsLoadableSectionsWithTheirDestination)
{
    const auto image = BuildImage(
        {
            { ".vdram", 0x0, "data" },
            { ".vpram", 0x0, "program!" },
            { ".idram", 0x4000, "ippu" },
        },
        { { 0x0, 0x0 }, { 0x4000, 0x4000 } });

    std::vector<vspa_eld_section> sections;
    ASSERT_EQ(ParseVSPAFirmware(image, sections), Result::Success);
    ASSERT_EQ(sections.size(), 3u);

    EXPECT_EQ(ToString(sections[0]), "data");
    EXPECT_EQ(sections[0].xfr_ctrl, DMEM);
    EXPECT_EQ(ToString(sections[1]), "program!");
    EXPECT_EQ(sections[1].xfr_ctrl, PRAM);
    EXPECT_EQ(ToString(sections[2]), "ippu");
    EXPECT_EQ(sections[2].address, 0x4000u);
    EXPECT_EQ(sections[2].xfr_ctrl, DMEM);
}

TEST(VSPA_loader, SkipsSectionsThatDontGetLoaded)
{
    const auto image = BuildImage(
        {
            { ".vpram", 0x0, "program!" },
            { ".dmem_cmdbuf", 0x0, "skip" },
            { ".ocram_vspa", 0x0, "skip" },
            { ".symtab", 0x0, "skip" },
            { ".CAL_ovl_ddr", 0x0, "skip" },
            { ".ipram", 0x0, "" },
            { ".idram", 0x0, "nobits!!", NOBITS },
        },
        { { 0x0, 0x0 } });

    std::vector<vspa_eld_section> sections;
    ASSERT_EQ(ParseVSPAFirmware(image, sections), Result::Success);
    ASSERT_EQ(sections.size(), 1u);
    EXPECT_EQ(ToString(sections[0]), "program!");
}

TEST(VSPA_loader, LoadsPramOverlaySection)
{
    const auto image = BuildImage({ { ".vpram_overlay", 0x100, "overlay!" } }, { { 0x100, 0x100 } });

    std::vector<vspa_eld_section> sections;
    ASSERT_EQ(ParseVSPAFirmware(image, sections), Result::Success);
    ASSERT_EQ(sections.size(), 1u);
    EXPECT_EQ(sections[0].xfr_ctrl, PRAM);
}

TEST(VSPA_loader, ClearsMemoryTypeBitsFromLoadAddress)
{
    const auto image = BuildImage({ { ".vpram", 0x100, "program!" } }, { { 0x100, 0x05000100 } });

    std::vector<vspa_eld_section> sections;
    ASSERT_EQ(ParseVSPAFirmware(image, sections), Result::Success);
    ASSERT_EQ(sections.size(), 1u);
    EXPECT_EQ(sections[0].address, 0x100u);
}

TEST(VSPA_loader, RejectsSectionWithoutLoadAddress)
{
    const auto image = BuildImage({ { ".vpram", 0x100, "program!" } }, { { 0x200, 0x200 } });

    std::vector<vspa_eld_section> sections;
    EXPECT_EQ(ParseVSPAFirmware(image, sections), Result::InvalidValue);
}

TEST(VSPA_loader, RejectsImageWithNothingToLoad)
{
    const auto image = BuildImage({ { ".symtab", 0x0, "skip" } }, { { 0x0, 0x0 } });

    std::vector<vspa_eld_section> sections;
    EXPECT_EQ(ParseVSPAFirmware(image, sections), Result::InvalidValue);
}

TEST(VSPA_loader, RejectsFilesThatAreNotVSPAImages)
{
    std::vector<vspa_eld_section> sections;
    EXPECT_EQ(ParseVSPAFirmware(std::vector<char>{}, sections), Result::InvalidValue);

    const auto valid = BuildImage({ { ".vpram", 0x0, "program!" } }, { { 0x0, 0x0 } });

    auto badMagic = valid;
    badMagic[0] = 'X';
    EXPECT_EQ(ParseVSPAFirmware(badMagic, sections), Result::InvalidValue);

    auto wrongMachine = valid;
    Put<uint16_t>(wrongMachine, 18, 0x28);
    EXPECT_EQ(ParseVSPAFirmware(wrongMachine, sections), Result::InvalidValue);
}

TEST(VSPA_loader, RejectsTruncatedImages)
{
    const auto valid = BuildImage({ { ".vpram", 0x0, "program!" } }, { { 0x0, 0x0 } });

    std::vector<vspa_eld_section> sections;
    for (size_t size = 0; size < valid.size(); ++size)
    {
        const std::vector<char> truncated(valid.begin(), valid.begin() + size);
        EXPECT_EQ(ParseVSPAFirmware(truncated, sections), Result::InvalidValue) << "size " << size;
    }
}

TEST(VSPA_loader, RejectsSectionContentsOutOfBounds)
{
    auto image = BuildImage({ { ".vpram", 0x0, "program!" } }, { { 0x0, 0x0 } });
    uint32_t shoff;
    std::memcpy(&shoff, image.data() + 32, sizeof(shoff));
    Put<uint32_t>(image, shoff + 20, 0xFFFFFFF0); // first section's size

    std::vector<vspa_eld_section> sections;
    EXPECT_EQ(ParseVSPAFirmware(image, sections), Result::InvalidValue);
}
