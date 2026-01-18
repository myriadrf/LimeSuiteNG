#include "I2C.h"

#include "protocols/LMS64CProtocol.h"
#include "comms/PCIe/LA9310_PCIe.h"
#include "comms/PCIe/PCIE_CSR_Pipe.h"

#include <memory>

namespace lime {

LA9310_I2C::LA9310_I2C(std::shared_ptr<LA9310_PCIe> port)
    : serialPort(port)
{
}

OpStatus LA9310_I2C::I2CWrite(uint16_t address, uint32_t offset, uint8_t offset_len, const uint8_t* data, uint32_t data_length)
{
    return LMS64CProtocol::I2C_Write(serialPort, address, offset, offset_len, data, data_length);
}

OpStatus LA9310_I2C::I2CRead(uint16_t address, uint32_t offset, uint8_t offset_len, uint8_t* data, uint32_t data_length)
{
    return LMS64CProtocol::I2C_Read(serialPort, address, offset, offset_len, data, data_length);
}

} // namespace lime
