#ifndef PCIE_CSR_PIPE_H
#define PCIE_CSR_PIPE_H

#include "comms/IComms.h"
#include "ISerialPort.h"
#include "LitePCIe.h"

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
    explicit PCIE_CSR_Pipe(std::shared_ptr<LitePCIe> port);

    int Write(const uint8_t* data, std::size_t length, int timeout_ms) override;
    int Read(uint8_t* data, std::size_t length, int timeout_ms) override;

  protected:
    std::shared_ptr<LitePCIe> port;
};

} // namespace lime

#endif // PCIE_CSR_PIPE_H
