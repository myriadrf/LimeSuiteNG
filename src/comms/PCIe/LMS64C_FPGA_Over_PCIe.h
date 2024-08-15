#ifndef LIME_LMS64C_FPGA_OVER_PCIE_H
#define LIME_LMS64C_FPGA_OVER_PCIE_H

#include "comms/IComms.h"
#include "comms/PCIe/LimePCIe.h"
#include "comms/PCIe/PCIE_CSR_Pipe.h"

#include <cstdint>
#include <memory>

namespace lime {

/** @brief A class for communicating with a device's FPGA over a PCIe interface. */
class LMS64C_FPGA_Over_PCIe : public IComms
{
  public:
    /**
      @brief Constructs a new LMS64C_FPGA_Over_PCIe object.
      @param dataPort The PCIe data bus to use.
     */
    LMS64C_FPGA_Over_PCIe(std::shared_ptr<LimePCIe> dataPort);

    OpStatus SPI(const uint32_t* MOSI, uint32_t* MISO, uint32_t count) override;
    OpStatus SPI(uint32_t spiBusAddress, const uint32_t* MOSI, uint32_t* MISO, uint32_t count) override;

    OpStatus CustomParameterWrite(const std::vector<CustomParameterIO>& parameters) override;
    OpStatus CustomParameterRead(std::vector<CustomParameterIO>& parameters) override;

    OpStatus ProgramWrite(const char* data, size_t length, int prog_mode, int target, ProgressCallback callback = nullptr) override;

    OpStatus MemoryWrite(uint32_t address, const void* data, uint32_t dataLength) override;
    OpStatus MemoryRead(uint32_t address, void* data, uint32_t dataLength) override;

  private:
    PCIE_CSR_Pipe pipe;
};

} // namespace lime

#endif // LIME_LMS64C_FPGA_OVER_PCIE_H
