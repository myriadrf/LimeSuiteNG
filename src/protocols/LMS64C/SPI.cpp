#include "SPI.h"

namespace lime {

LMS64C_SPI::LMS64C_SPI(std::shared_ptr<ISerialPort> port,
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

OpStatus LMS64C_SPI::Transact(const uint32_t* MOSI, uint32_t* MISO, uint32_t count)
{
    if (write_command == LMS64CProtocol::Command::PERIPHSPI_TRNSF && read_command == LMS64CProtocol::Command::PERIPHSPI_TRNSF)
    {
        // subdeviceIndex - SPI bus index
        // peripheralId - chip select on the bus
        return LMS64CProtocol::SPI_generic(*port, subdeviceIndex, peripheralId, MOSI, MISO, count);
    }
    else
    {
        // subdeviceIndex - index of daughter board
        // peripheralId - within the board
        return LMS64CProtocol::SPI16(*port, peripheralId, write_command, MOSI, read_command, MISO, count, subdeviceIndex);
    }
}

} // namespace lime
