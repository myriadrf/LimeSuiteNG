#ifndef LIME_II2C_H
#define LIME_II2C_H

#include "limesuiteng/config.h"
#include "limesuiteng/OpStatus.h"

#include <cstdint>

namespace lime {

/** @brief An interface for Inter-Integrated Circuit communications */
class LIME_API II2C
{
  public:
    /** @brief Destroys the interfaced object. */
    virtual ~II2C() {}
    /**
      @brief Write to an available Inter-Integrated Circuit slave.
      @param address Inter-Integrated Circuit slave address.
      @param data Output buffer.
      @param length Output data length.
      @return The operation status.
     */
    virtual OpStatus I2CWrite(int address, const uint8_t* data, uint32_t length) = 0;

    /**
      @brief Read from an available Inter-Integrated Circuit slave.

      Some implementations can combine a write + read transaction.
      If the device contains multiple I2C masters,
      the address bits can encode which master.
      @param address The address of the slave.
      @param [out] dest Buffer to store read data from the slave.
      @param length Number of bytes to read.
      @return The operation status.
     */
    virtual OpStatus I2CRead(int address, uint8_t* dest, uint32_t length) = 0;
};

} // namespace lime

#endif // LIME_II2C_H
