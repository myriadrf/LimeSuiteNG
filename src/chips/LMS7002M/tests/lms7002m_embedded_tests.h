#pragma once

#include <gtest/gtest.h>

#include <cstdint>
#include <unordered_map>

struct lms7002m_context;

namespace lime::testing {

// Emulates LMS7002M registers memory write/read
class LMS7002M_SPI_STUB
{
  public:
    LMS7002M_SPI_STUB()
    {
        registers[0][0x0020] = 0xFFFF; // by default MAC is 3
    };

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

                uint8_t mac = self->registers[0].at(0x0020) & 0x3;
                if (mac & 0x1)
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

                if (mac == 0x3)
                    throw std::runtime_error("Reading from both channels simultaneously is undefined");

                value = self->registers[mac & 0x2 ? 1 : 0][addr];
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

class lms7002m_embedded : public ::testing::Test
{
  protected:
    lms7002m_embedded();

    void SetUp() override;
    void TearDown() override;

    LMS7002M_SPI_STUB spi_stub;

  public:
    lms7002m_context* chip;
};

} // namespace lime::testing