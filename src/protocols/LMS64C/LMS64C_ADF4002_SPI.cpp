#include "LMS64C_ADF4002_SPI.h"
#include "protocols/LMS64CProtocol.h"

using namespace lime;

LMS64C_ADF4002_SPI::LMS64C_ADF4002_SPI(std::shared_ptr<ISerialPort> serialPort, uint32_t subdeviceIndex)
    : mSerialPort(serialPort)
    , mSubdeviceIndex(subdeviceIndex)
{
}

OpStatus LMS64C_ADF4002_SPI::Transact(const uint32_t* MOSI, uint32_t* MISO, uint32_t count)
{
    return LMS64CProtocol::ADF4002_SPI(*(mSerialPort.get()), MOSI, count, mSubdeviceIndex);
}
