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
      @brief Configuration Space Register write function.
      @param address Register address.
      @param data_rd New register value.
      @returns The operation status.
     */
    virtual OpStatus ioWrite64(uint64_t address, uint64_t value) = 0;

    /**
      @brief Configuration Space Register read function.
      @param address Register address.
      @param status Read operation status.
      @returns Value of CSRegister.
     */
    virtual uint64_t ioRead64(uint64_t address, OpStatus* status = nullptr) = 0;
};

} // namespace lime

#endif // LIME_ICSR_H
