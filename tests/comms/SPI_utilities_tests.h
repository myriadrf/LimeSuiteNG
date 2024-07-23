#pragma once

#include <gtest/gtest.h>

#include "comms/ISPI.h"

namespace lime::testing {

// Emulates LMS7002M registers memory write/read
class SPI_emulation : public lime::ISPI
{
  public:
    OpStatus SPI(const uint32_t* MOSI, uint32_t* MISO, uint32_t count) override;
    OpStatus SPI(uint32_t spiBusAddress, const uint32_t* MOSI, uint32_t* MISO, uint32_t count) override;

    std::unordered_map<uint16_t, uint16_t> registers;
};

} // namespace lime::testing
