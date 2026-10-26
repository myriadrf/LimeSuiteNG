#ifndef LIME_OEMDATA_H
#define LIME_OEMDATA_H

#include "limesuiteng/config.h"
#include "limesuiteng/OpStatus.h"
#include "protocols/LMSBoards.h"

#include <array>
#include <cstdint>
#include <span>
#include <string>

namespace lime {

/// @brief Board identification and configuration data programmed during manufacturing (format version 1)
struct LIME_API OEMData {
    static constexpr std::size_t imageSize = 128;
    static constexpr uint32_t magic = 0x4C494D45; // "LIME"
    static constexpr uint8_t formatVersion = 1;
    static constexpr uint8_t headerLength = 12;
    static constexpr uint16_t dataLength = 68;

    using UUID = std::array<uint8_t, 16>;
    using Image = std::array<uint8_t, imageSize>;

    uint32_t boardId{ LMS_DEV_UNKNOWN }; ///< eLMS_DEV
    uint8_t boardVersion{ 0 };
    uint8_t boardRevision{ 0 };
    UUID uuid{}; ///< UUID version 7, big-endian
    std::array<uint8_t, 32> configuration_flags; ///< 256-bit big-endian bit field, fitted components configuration

    OEMData() { configuration_flags.fill(0xFF); }

    /// @brief Builds the complete 128-byte image, including CRC and reserved bytes.
    Image Encode() const;
    /// @brief Validates and parses an image.
    /// @return OpStatus::Success if the image is valid, OpStatus::InvalidValue otherwise.
    static OpStatus Decode(std::span<const uint8_t, imageSize> image, OEMData& data);

    /// @brief Generates a new UUID version 7 (RFC 9562) from the current time and random bits.
    static UUID GenerateUUIDv7();
    static std::string ToString(const UUID& uuid);
};

/// @brief IEEE 802.3 CRC-32
LIME_API uint32_t CRC32(std::span<const uint8_t> data);

} // namespace lime

#endif // LIME_OEMDATA_H
