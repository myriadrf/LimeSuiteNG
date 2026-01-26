#include "CSR.h"

namespace lime {

LMS64C_CSR::LMS64C_CSR(std::shared_ptr<ISerialPort> port)
    : port(port)
{
}

OpStatus LMS64C_CSR::ioWrite64(uint64_t address, uint64_t value)
{
    return LMS64CProtocol::CSRegIoWrite(*port, address, value);
}

uint64_t LMS64C_CSR::ioRead64(uint64_t address, OpStatus* status)
{
    return LMS64CProtocol::CSRegIoRead(*port, address, status);
}

} // namespace lime