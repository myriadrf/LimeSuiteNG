#pragma once

#include <cstdint>

namespace lime::lms7002m {

struct AddressValuePair {
    std::uint16_t address;
    std::uint16_t value;
};

} // namespace lime::lms7002m
