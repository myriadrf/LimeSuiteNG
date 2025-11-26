#include "CSR.h"

namespace lime {

LMS64C_CSR::LMS64C_CSR(std::shared_ptr<ISerialPort> port,
    LMS64CProtocol::Command write_command,
    LMS64CProtocol::Command read_command,
    uint32_t subdeviceIndex,
    uint32_t peripheralId)
    : port(port)
    , write_command(write_command)
    , read_command(read_command)
    , subdeviceIndex(subdeviceIndex)
    , peripheralId(peripheralId)
{
}

OpStatus LMS64C_CSR::Transact(const uint64_t* data_wr, uint64_t* data_rd, uint32_t count)
{
   // subdeviceIndex - index of daughter board
   // peripheralId - within the board
   return LMS64CProtocol::CSRegisterTransaction(*port, peripheralId, write_command, data_wr, read_command, data_rd, count, subdeviceIndex);
}

} // namespace lime