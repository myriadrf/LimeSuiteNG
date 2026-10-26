#include "OEMData.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <random>

using namespace std;

namespace lime {

// Field offsets in the image
static constexpr std::size_t magicOffset = 0x00;
static constexpr std::size_t formatVersionOffset = 0x04;
static constexpr std::size_t headerLengthOffset = 0x05;
static constexpr std::size_t dataLengthOffset = 0x06;
static constexpr std::size_t crcOffset = 0x08;
static constexpr std::size_t boardIdOffset = 0x0C;
static constexpr std::size_t boardVersionOffset = 0x10;
static constexpr std::size_t boardRevisionOffset = 0x11;
static constexpr std::size_t uuidOffset = 0x12;
static constexpr std::size_t flagsOffset = 0x30;

static void WriteBigEndian(uint8_t* dest, uint64_t value, std::size_t byteCount)
{
    for (std::size_t i = 0; i < byteCount; ++i)
        dest[i] = value >> (8 * (byteCount - 1 - i));
}

static uint64_t ReadBigEndian(const uint8_t* src, std::size_t byteCount)
{
    uint64_t value = 0;
    for (std::size_t i = 0; i < byteCount; ++i)
        value = (value << 8) | src[i];
    return value;
}

uint32_t CRC32(std::span<const uint8_t> data)
{
    uint32_t crc = 0xFFFFFFFF;
    for (uint8_t byte : data)
    {
        crc ^= byte;
        for (int bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
    }
    return ~crc;
}

OEMData::Image OEMData::Encode() const
{
    Image image{}; // reserved bytes are 0x00
    WriteBigEndian(&image[magicOffset], magic, 4);
    image[formatVersionOffset] = formatVersion;
    image[headerLengthOffset] = headerLength;
    WriteBigEndian(&image[dataLengthOffset], dataLength, 2);
    WriteBigEndian(&image[boardIdOffset], boardId, 4);
    image[boardVersionOffset] = boardVersion;
    image[boardRevisionOffset] = boardRevision;
    std::copy(uuid.begin(), uuid.end(), &image[uuidOffset]);
    std::copy(configuration_flags.begin(), configuration_flags.end(), &image[flagsOffset]);

    const uint32_t crc = CRC32(std::span<const uint8_t>(&image[headerLength], dataLength));
    WriteBigEndian(&image[crcOffset], crc, 4);
    return image;
}

OpStatus OEMData::Decode(std::span<const uint8_t, imageSize> image, OEMData& data)
{
    if (ReadBigEndian(&image[magicOffset], 4) != magic)
        return OpStatus::InvalidValue;
    if (image[formatVersionOffset] != formatVersion)
        return OpStatus::InvalidValue;
    const uint16_t length = ReadBigEndian(&image[dataLengthOffset], 2);
    if (image[headerLengthOffset] != headerLength || length != dataLength)
        return OpStatus::InvalidValue;
    if (ReadBigEndian(&image[crcOffset], 4) != CRC32(image.subspan(headerLength, length)))
        return OpStatus::InvalidValue;

    data.boardId = ReadBigEndian(&image[boardIdOffset], 4);
    data.boardVersion = image[boardVersionOffset];
    data.boardRevision = image[boardRevisionOffset];
    std::copy_n(&image[uuidOffset], data.uuid.size(), data.uuid.begin());
    std::copy_n(&image[flagsOffset], data.configuration_flags.size(), data.configuration_flags.begin());
    return OpStatus::Success;
}

OEMData::UUID OEMData::GenerateUUIDv7()
{
    UUID uuid;
    std::random_device rd;
    std::uniform_int_distribution<int> byteDistribution(0, 255);
    for (auto& byte : uuid)
        byte = byteDistribution(rd);

    const uint64_t unixMs = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
    WriteBigEndian(&uuid[0], unixMs, 6);
    uuid[6] = 0x70 | (uuid[6] & 0x0F); // version 7
    uuid[8] = 0x80 | (uuid[8] & 0x3F); // variant 0b10
    return uuid;
}

std::string OEMData::ToString(const UUID& uuid)
{
    char str[37];
    char* out = str;
    for (std::size_t i = 0; i < uuid.size(); ++i)
    {
        if (i == 4 || i == 6 || i == 8 || i == 10)
            *out++ = '-';
        out += std::snprintf(out, 3, "%02x", uuid[i]);
    }
    return std::string(str);
}

} // namespace lime
