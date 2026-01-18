#ifndef LIME_EEPROM_I2C_H
#define LIME_EEPROM_I2C_H

#include "limesuiteng/OpStatus.h"

#include "comms/II2C.h"

#include <cstdint>
#include <memory>

namespace lime {

class EEPROM_I2C
{
  public:
    EEPROM_I2C() = delete;
    EEPROM_I2C(std::shared_ptr<II2C> port, uint16_t i2c_addr, uint32_t memory_size, uint32_t page_size);

    OpStatus Write(uint32_t addr, const void* data, size_t length);
    OpStatus Read(uint32_t addr, void* data, size_t length);

  private:
    std::shared_ptr<II2C> port;
    uint16_t i2c_addr;
    uint32_t memory_size;
    uint32_t page_size;
};

} // namespace lime

#endif