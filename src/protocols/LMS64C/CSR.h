#ifndef LIME_LMS64C_CSR_H
#define LIME_LMS64C_CSR_H

#include <cstdint>
#include <memory>

#include "limesuiteng/ICSR.h"
#include "protocols/LMS64CProtocol.h"

namespace lime {

class ISerialPort;

/** @brief Communication helper to divert data writes/reads to/from Configuration Space Registers. */
class LMS64C_CSR : public ICSR
{
  public:
    LMS64C_CSR(std::shared_ptr<ISerialPort> port);

    OpStatus ioWrite64(uint64_t address, uint64_t value) override;

    uint64_t ioRead64(uint64_t address, OpStatus* status = nullptr) override;

  private:
    std::shared_ptr<ISerialPort> port;
};

} // namespace lime

#endif