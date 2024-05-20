#ifndef PCIE_CSR_PIPE_H
#define PCIE_CSR_PIPE_H

#include "comms/IComms.h"
#include "ISerialPort.h"
#include "LimeLitePCIe.h"

#include <memory>

namespace lime {

/** @brief An abstract class for interfacing with Control/Status registers (CSR) of a PCIe device. */
class PCIE_CSR_Pipe : public ISerialPort
{
  public:
    /**
    @brief Constructs a new PCIE_CSR_Pipe object.
    
    @param port The LitePCIe port to use with this pipe.
   */
    explicit PCIE_CSR_Pipe(std::shared_ptr<LimeLitePCIe> port);

    int Write(const uint8_t* data, std::size_t length, int timeout_ms) override;
    int Read(uint8_t* data, std::size_t length, int timeout_ms) override;
    OpStatus RunControlCommand(uint8_t* data, size_t length, int timeout_ms) override;
    OpStatus RunControlCommand(uint8_t* request, uint8_t* response, size_t length, int timeout_ms) override;

  protected:
    std::shared_ptr<LimeLitePCIe> port;
};

} // namespace lime

#endif // PCIE_CSR_PIPE_H
