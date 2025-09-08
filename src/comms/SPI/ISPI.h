#ifndef LIME_ISPI_H
#define LIME_ISPI_H

#include "limesuiteng/config.h"
#include "limesuiteng/OpStatus.h"

#include <cstdint>

namespace lime {

/** @brief An interface for Serial Peripheral Interface communications
*/
class LIME_API ISPI
{
  public:
    virtual ~ISPI() {}
    /**
      @brief Default path for writing/reading registers.
      @param MOSI Master Output Slave input
      @param MISO Master Input Slave output
      @param count Input/output data count.
      @returns The operation status.
     */
    virtual OpStatus Transact(const uint32_t* MOSI, uint32_t* MISO, uint32_t count) = 0;
};

} // namespace lime

#endif // LIME_ISPI_H
