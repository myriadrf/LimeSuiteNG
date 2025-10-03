#ifndef LIME_LMS64C_SPI_BUS_H
#define LIME_LMS64C_SPI_BUS_H

#include <cstdint>
#include <memory>

#include "comms/SPI/ISPI.h"
#include "protocols/LMS64CProtocol.h"

namespace lime {

class ISerialPort;

/** @brief Communications helper to divert data to specific device. */
class LMS64C_SPI_Bus : public ISPI
{
  public:
    LMS64C_SPI(std::shared_ptr<ISerialPort> port, uint32_t busIndex);
    OpStatus Transact(const uint32_t* MOSI, uint32_t* MISO, uint32_t count) override;

  private:
    std::shared_ptr<ISerialPort> port;
    const uint32_t busIndex;
    const uint32_t peripheralId;
};

} // namespace lime

#endif // LIME_LMS64C_SPI_BUS_H
