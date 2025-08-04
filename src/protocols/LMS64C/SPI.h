#ifndef LIME_LMS64C_SPI_H
#define LIME_LMS64C_SPI_H

#include <cstdint>
#include <memory>

#include "comms/SPI/ISPI.h"
#include "protocols/LMS64CProtocol.h"

namespace lime {

class ISerialPort;

/** @brief Communications helper to divert data to specific device. */
class LMS64C_SPI : public ISPI
{
  public:
    LMS64C_SPI(std::shared_ptr<ISerialPort> port,
        LMS64CProtocol::Command write_command,
        LMS64CProtocol::Command read_command,
        uint32_t subdeviceIndex,
        uint32_t peripheralId);
    OpStatus Transact(const uint32_t* MOSI, uint32_t* MISO, uint32_t count) override;

  private:
    std::shared_ptr<ISerialPort> port;
    const LMS64CProtocol::Command write_command;
    const LMS64CProtocol::Command read_command;
    const uint32_t subdeviceIndex;
    const uint32_t peripheralId;
};

} // namespace lime

#endif // LIME_LMS64C_SPI_H
