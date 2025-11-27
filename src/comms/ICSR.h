#ifndef LIME_ICSR_H
#define LIME_ICSR_H

#include "limesuiteng/config.h"
#include "limesuiteng/OpStatus.h"

#include <cstdint>

namespace lime {

/** @brief An interface for reading and writing Configuration Space Registers.
*/
class LIME_API ICSR
{
  public:
    virtual ~ICSR() {}
    /**
      @brief Default path for writing/reading CSRegisters.
      @param data_wr Buffer with register address and register data blocks for writing CSRegisters.
      @param data_rd Buffer for storing read data from CSRegisters. Pass nullptr when wrinting only to CSRegisters.
      @param count Number of CSRegisters to write/read. Not the size of data_wr buffer!
      @returns The operation status.
     */
    virtual OpStatus Transact(const uint64_t* data_wr, uint64_t* data_rd, uint32_t count) = 0;
};

} // namespace lime

#endif // LIME_ICSR_H
