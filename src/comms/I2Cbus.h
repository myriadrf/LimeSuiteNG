#ifndef LIME_I2C_BUS_H
#define LIME_I2C_BUS_H

#include "limesuiteng/config.h"
#include "limesuiteng/OpStatus.h"

#include <cstdint>

namespace lime {

class I2C_bus
{
  public:
    virtual ~I2C_bus() {}

    virtual OpStatus Write(uint32_t soc_address, uint16_t register_offset, const uint8_t* data, uint32_t length) = 0;
    virtual OpStatus Read(uint32_t soc_address, uint16_t register_offset, uint8_t* dest, uint32_t length) = 0;
};

} // namespace lime

#endif // LIME_I2C_BUS_H
