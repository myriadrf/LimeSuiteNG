#ifndef LIME_ISPI_H
#define LIME_ISPI_H

#include "limesuiteng/config.h"
#include "limesuiteng/OpStatus.h"

#include <cstdint>

namespace lime {

/** @brief An interface for Serial Peripheral Interface communications */
class LIME_API ISPI
{
  public:
    /** @brief Destroys the interfaced object. */
    virtual ~ISPI() {}
    /**
      @brief Default path for writing/reading registers.
      @param MOSI Main Out Sub In (data output from main).
      @param MISO Main In Sub Out (data output from sub).
      @param count Input/output data count.
      @returns The operation status.
     */
    virtual OpStatus SPI(const uint32_t* MOSI, uint32_t* MISO, uint32_t count) = 0;

    /**
      @brief Writing/reading registers for specific slave.
      @param chipSelect Which chips to communicate with.
      @param MOSI Main Out Sub In (data output from main).
      @param MISO Main In Sub Out (data output from sub).
      @param count Input/output data count.
      @returns The operation status.
     */
    virtual OpStatus SPI(uint32_t chipSelect, const uint32_t* MOSI, uint32_t* MISO, uint32_t count) = 0;
};

} // namespace lime

#endif // LIME_ISPI_H
