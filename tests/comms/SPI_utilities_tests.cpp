#include "SPI_utilities_tests.h"

#include "comms/SPI_utilities.h"

#include "limesuiteng/Register.h"

using namespace lime;

namespace lime::testing {

OpStatus SPI_emulation::SPI(const uint32_t* MOSI, uint32_t* MISO, uint32_t count)
{
    return SPI(0, MOSI, MISO, count);
}

OpStatus SPI_emulation::SPI(uint32_t spiBusAddress, const uint32_t* MOSI, uint32_t* MISO, uint32_t count)
{
    if (!MOSI)
        return OpStatus::Error;

    for (size_t i = 0; i < count; ++i)
    {
        if (MOSI[i] & (1 << 31))
        {
            const uint16_t addr = (MOSI[i] >> 16) & 0x7FFF;
            const uint16_t value = MOSI[i] & 0xFFFF;
            registers[addr] = value;
        }
        else
        {
            if (!MISO)
                return OpStatus::Error;

            const uint16_t addr = MOSI[i] & 0x7FFF;
            MISO[i] = addr << 16 | registers[addr];
        }
    }
    return OpStatus::Success;
}

TEST(SPI_utilities, WriteSPI)
{
    SPI_emulation emul;
    constexpr uint16_t address = 0x1234;
    constexpr uint16_t wrValue = 0xABCD;
    OpStatus status = WriteSPI(&emul, address, wrValue);
    ASSERT_EQ(status, OpStatus::Success);

    ASSERT_EQ(emul.registers.size(), 1);
    ASSERT_EQ(emul.registers.at(address), wrValue);
}

TEST(SPI_utilities, ReadSPI)
{
    SPI_emulation emul;
    constexpr uint16_t address = 0x1234;
    constexpr uint16_t value = 0xABCD;
    emul.registers[address] = value;
    OpStatus status{ OpStatus::Error };
    const uint16_t rdValue = ReadSPI(&emul, address, &status);
    ASSERT_EQ(status, OpStatus::Success);
    ASSERT_EQ(rdValue, value);
}

TEST(SPI_utilities, ModifyRegister)
{
    SPI_emulation emul;
    constexpr uint16_t address = 0x1234;
    emul.registers[address] = 0xABCD;

    Register reg{ address, 11, 4 };

    OpStatus status = ModifyRegister(&emul, reg, 0xEF);
    ASSERT_EQ(status, OpStatus::Success);
    ASSERT_EQ(emul.registers.at(address), 0xAEFD);
}

TEST(SPI_utilities, ReadRegister)
{
    SPI_emulation emul;
    constexpr uint16_t address = 0x1234;
    emul.registers[address] = 0xABCD;

    Register reg{ address, 11, 4 };

    OpStatus status{ OpStatus::Error };
    uint16_t value = ReadRegister(&emul, reg, &status);
    ASSERT_EQ(status, OpStatus::Success);
    ASSERT_EQ(value, 0xBC);
}

} // namespace lime::testing