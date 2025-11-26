#ifndef LIME_LMS64C_CSR_H
#define LIME_LMS64C_CSR_H

#include <cstdint>
#include <memory>

#include "comms/ICSR.h"
#include "protocols/LMS64CProtocol.h"

namespace lime
{

class ISerialPort;

/** @brief Communication helper to divert data read/writes to Configuration Space Registers. */
class LMS64C_CSR : public ICSR
{
  public:
    LMS64C_CSR(std::shared_ptr<ISerialPort> port,
        LMS64CProtocol::Command write_command,
        LMS64CProtocol::Command read_command,
        uint32_t subdeviceIndex,
        uint32_t peripheralId);
    OpStatus Transact(const uint64_t* data_wr, uint64_t* data_rd, uint32_t count) override;

  private:
    std::shared_ptr<ISerialPort> port;
    const LMS64CProtocol::Command write_command;
    const LMS64CProtocol::Command read_command;
    const uint32_t subdeviceIndex;
    const uint32_t peripheralId;
};


} // namespace lime


#endif