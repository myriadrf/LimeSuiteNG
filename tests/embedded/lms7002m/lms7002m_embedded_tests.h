#pragma once

#include <gtest/gtest.h>

#include "lms7002m/csr.h"

#include <cstdint>
#include <unordered_map>

struct lms7002m_context;

namespace lime::testing {

// Emulates LMS7002M registers memory write/read
class LMS7002M_SPI_STUB
{
  private:
    // Module addresses needs to be sorted in ascending order
    const std::unordered_map<uint16_t, uint16_t> readOnlyRegisterMasks{
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
    };

  public:
    LMS7002M_SPI_STUB()
    {
        registers[0][0x0020] = 0xFFFF; // by default MAC is 3
    };

    void Set(uint8_t ch, const lms7002m_csr& csr, uint16_t value)
    {
        uint16_t regValue = registers[ch][csr.address];
        uint16_t mask = (~(~0u << (csr.msb - csr.lsb + 1))) << (csr.lsb);
        registers[ch][csr.address] = (regValue & (~mask)) | ((value << csr.lsb) & mask); //clear bits
    }

    uint16_t Get(uint8_t ch, const lms7002m_csr& csr)
    {
        uint16_t regValue = registers[ch][csr.address];
        return (regValue & (~(~0u << (csr.msb + 1)))) >> csr.lsb;
    }

    static int spi16_transact(const uint32_t* mosi, uint32_t* miso, uint32_t count, void* userData)
    {
        LMS7002M_SPI_STUB* self = reinterpret_cast<LMS7002M_SPI_STUB*>(userData);
        for (uint32_t i = 0; i < count; ++i)
        {
            if (mosi[i] & (1 << 31))
            {
                uint16_t addr = mosi[i] >> 16;
                addr &= 0x7FFF; // clear write bit for now
                uint16_t value = mosi[i] & 0xFFFF;

                // prevent writing to read only bits
                auto iter = self->readOnlyRegisterMasks.find(addr);
                if (iter != self->readOnlyRegisterMasks.end())
                    value &= iter->second;

                uint8_t mac = self->registers[0].at(0x0020) & 0x3;
                if (mac & 0x1 || addr < 0x0100)
                    self->registers[0][addr] = value;
                if (mac & 0x2 && addr >= 0x0100)
                    self->registers[1][addr] = value;
                ++self->writeCount;
            }
            else
            {
                uint16_t addr = mosi[i] >> 16;
                uint16_t value = 0;

                uint8_t mac = self->registers[0].at(0x0020) & 0x3;

                if (addr >= 0x0100)
                {
                    if (mac == 0x3)
                        throw std::runtime_error("Reading from both channels simultaneously is undefined");

                    if (mac != 0)
                        value = self->registers[mac - 1][addr];
                }
                else
                    value = self->registers[0][addr];

                if (miso)
                    miso[i] = value;
                ++self->readCount;
            }
        }
        return 0;
    }

    std::unordered_map<uint16_t, uint16_t> registers[2];
    int32_t writeCount{ 0 };
    int32_t readCount{ 0 };
};

} // namespace lime::testing
