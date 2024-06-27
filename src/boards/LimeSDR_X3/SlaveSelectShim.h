#ifndef LIME_SLAVESELECTSHIM_H
#define LIME_SLAVESELECTSHIM_H

#include "comms/IComms.h"
#include "comms/ISPI.h"
#include "limesuiteng/config.h"

#include <cstdint>
#include <memory>

namespace lime {

/** @brief Communications helper to divert data to specific device. */
class LIME_API SlaveSelectShim : public ISPI
{
  public:
    /**
    @brief Construct a new Slave Select Shim object
    @param comms The communications interface to use.
    @param slaveId The ID of the slave for this shim.
   */
    SlaveSelectShim(std::shared_ptr<IComms> comms, uint32_t slaveId);
    OpStatus SPI(const uint32_t* MOSI, uint32_t* MISO, uint32_t count) override;
    OpStatus SPI(uint32_t spiBusAddress, const uint32_t* MOSI, uint32_t* MISO, uint32_t count) override;

    /**
      @brief Send the reset device command to the device under this shim.
      @return The status of the operation.
     */
    OpStatus ResetDevice();

  private:
    std::shared_ptr<IComms> port;
    uint32_t slaveId;
};

} // namespace lime

#endif // LIME_SLAVESELECTSHIM_H
