#ifndef LIME_LMS64C_I2C_H
#define LIME_LMS64C_I2C_H

#include <cstdint>
#include <memory>

#include "comms/II2C.h"
#include "comms/PCIe/PCIE_CSR_Pipe.h"
namespace lime {

class LA9310_PCIe;

/** @brief Communications helper to divert data to specific device. */
class LA9310_I2C : public II2C
{
  public:
    LA9310_I2C(std::shared_ptr<LA9310_PCIe> port);

    OpStatus I2CWrite(uint16_t address, uint32_t offset, uint8_t offset_len, const uint8_t* data, uint32_t data_length) override;
    OpStatus I2CRead(uint16_t address, uint32_t offset, uint8_t offset_len, uint8_t* data, uint32_t data_length) override;

  private:
    PCIE_CSR_Pipe serialPort;
};

} // namespace lime

#endif // LIME_LMS64C_I2C_H
