#pragma once

#include <array>

#include "AddressValuePair.h"

namespace lime::lms7002m {

/// Addresses and masks of readonly bits
constexpr std::array<AddressValuePair, 13> registersReadOnlyMasks{ {
    { 0x002F, 0x0000 },
    { 0x008C, 0x0FFF },
    { 0x00A8, 0x007F },
    { 0x00A9, 0x0000 },
    { 0x00AA, 0x0000 },
    { 0x00AB, 0x0000 },
    { 0x00AC, 0x0000 },
    { 0x0123, 0x003F },
    { 0x0209, 0x0000 },
    { 0x020A, 0x0000 },
    { 0x020B, 0x0000 },
    { 0x040E, 0x0000 },
    { 0x040F, 0x0000 },
} };

} // namespace lime::lms7002m
