#include "LMS64C_ADF_Over_USB.h"
#include "protocols/LMS64CProtocol.h"

using namespace lime;

LMS64C_ADF_Over_USB::LMS64C_ADF_Over_USB(std::shared_ptr<ISerialPort> serialPort, uint32_t subdeviceIndex)
    : mSerialPort(serialPort)
    , mSubdeviceIndex(subdeviceIndex)
{
}

OpStatus LMS64C_ADF_Over_USB::SPI(const uint32_t* MOSI, uint32_t* MISO, uint32_t count)
{
    return LMS64CProtocol::ADF4002_SPI(*(mSerialPort.get()), MOSI, count, mSubdeviceIndex);
}

OpStatus LMS64C_ADF_Over_USB::SPI(uint32_t spiBusAddress, const uint32_t* MOSI, uint32_t* MISO, uint32_t count)
{
    return LMS64CProtocol::ADF4002_SPI(*(mSerialPort.get()), MOSI, count, mSubdeviceIndex);
}
