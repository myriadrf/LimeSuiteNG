#include "EEPROM_I2C.h"

#include <cassert>
#include <stdio.h>

namespace lime {

EEPROM_I2C::EEPROM_I2C(std::shared_ptr<II2C> port, uint16_t i2c_addr, uint32_t memory_size, uint32_t page_size)
    : port(port)
    , i2c_addr(i2c_addr)
    , memory_size(memory_size)
    , page_size(page_size)
{
    assert(memory_size > 0);
    assert(page_size > 0);
}

OpStatus EEPROM_I2C::Write(uint32_t mem_addr, const void* data, size_t length)
{
    if (mem_addr + length > memory_size)
        return OpStatus::OutOfRange;

    const uint8_t* src = reinterpret_cast<const uint8_t*>(data);
    // align to pages
    size_t bytesToEndOfPage = page_size - (mem_addr % page_size);
    uint32_t toWrite = std::min(length, bytesToEndOfPage);
    if (toWrite > length)
        toWrite = length;
    OpStatus status = port->I2CWrite(i2c_addr, mem_addr, 2, src, toWrite);
    if (status != OpStatus::Success)
        return status;

    length -= toWrite;
    src += toWrite;
    mem_addr += toWrite;

    while (length > 0)
    {
        const uint32_t toWrite = length > page_size ? page_size : length;
        OpStatus status = port->I2CWrite(i2c_addr, mem_addr, 2, src, toWrite);
        if (status != OpStatus::Success)
            return status;

        length -= toWrite;
        src += toWrite;
        mem_addr += toWrite;
    }
    return OpStatus::Success;
}

OpStatus EEPROM_I2C::Read(uint32_t mem_addr, void* data, size_t length)
{
    if (mem_addr + length > memory_size)
        return OpStatus::OutOfRange;

    uint8_t* dest = reinterpret_cast<uint8_t*>(data);
    // align to pages
    size_t bytesToEndOfPage = page_size - (mem_addr % page_size);
    uint32_t toRead = std::min(length, bytesToEndOfPage);
    if (toRead > length)
        toRead = length;
    OpStatus status = port->I2CRead(i2c_addr, mem_addr, 2, dest, toRead);
    if (status != OpStatus::Success)
        return status;

    length -= toRead;
    dest += toRead;
    mem_addr += toRead;

    while (length > 0)
    {
        const uint32_t toRead = length > page_size ? page_size : length;
        OpStatus status = port->I2CWrite(i2c_addr, mem_addr, 2, dest, toRead);
        if (status != OpStatus::Success)
            return status;

        length -= toRead;
        dest += toRead;
        mem_addr += toRead;
    }
    return OpStatus::Success;
}

} // namespace lime
