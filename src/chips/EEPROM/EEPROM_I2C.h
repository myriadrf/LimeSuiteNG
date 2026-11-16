#ifndef LIME_EEPROM_I2C_H
#define LIME_EEPROM_I2C_H

#include "limesuiteng/OpStatus.h"

#include "comms/II2C.h"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>

namespace lime {

class EEPROM_I2C
{
  public:
    EEPROM_I2C() = delete;
    /**
      @param port I2C bus to use.
      @param i2c_addr Memory device address.
      @param memory_size Memory size in bytes.
      @param page_size Maximum bytes per single I2C transaction.
     */
    EEPROM_I2C(std::shared_ptr<II2C> port, uint16_t i2c_addr, uint32_t memory_size, uint32_t page_size);

    /// @brief Called before each I2C transaction of a transfer, starting with bytesDone = 0 before anything is transferred,
    /// and once more with bytesDone = bytesTotal when the transfer completes.
    /// @param bytesDone The number of bytes transferred so far.
    /// @param bytesTotal The total number of bytes in the transfer.
    /// @return true to abort the transfer. Ignored for the completion call.
    typedef std::function<bool(size_t bytesDone, size_t bytesTotal)> ProgressCallback;

    /// @return Aborted if the callback requested it, the memory before addr+bytesDone is already written (nothing if bytesDone was 0).
    OpStatus Write(uint32_t addr, const void* data, size_t length, const ProgressCallback& callback = nullptr);
    /// @return Aborted if the callback requested it.
    OpStatus Read(uint32_t addr, void* data, size_t length, const ProgressCallback& callback = nullptr);

  private:
    std::shared_ptr<II2C> port;
    uint16_t i2c_addr;
    uint32_t memory_size;
    uint32_t page_size;
};

} // namespace lime

#endif
