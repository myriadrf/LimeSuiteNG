#include "LMS64C_LMS7002M_Over_PCIe.h"
#include "protocols/LMS64CProtocol.h"

using namespace lime;

LMS64C_LMS7002M_Over_PCIe::LMS64C_LMS7002M_Over_PCIe(std::shared_ptr<LitePCIe> dataPort)
    : pipe(dataPort)
{
}

OpStatus LMS64C_LMS7002M_Over_PCIe::SPI(const uint32_t* MOSI, uint32_t* MISO, uint32_t count)
{
    return SPI(0, MOSI, MISO, count);
}

OpStatus LMS64C_LMS7002M_Over_PCIe::SPI(uint32_t spiBusAddress, const uint32_t* MOSI, uint32_t* MISO, uint32_t count)
{
    return LMS64CProtocol::LMS7002M_SPI(pipe, spiBusAddress, MOSI, MISO, count);
}

OpStatus LMS64C_LMS7002M_Over_PCIe::ResetDevice(int chipSelect)
{
    return OpStatus::NotImplemented;
}

OpStatus LMS64C_LMS7002M_Over_PCIe::GPIOWrite(const uint8_t* buffer, const size_t bufLength)
{
    return OpStatus::NotImplemented;
}

OpStatus LMS64C_LMS7002M_Over_PCIe::GPIORead(uint8_t* buffer, const size_t bufLength)
{
    return OpStatus::NotImplemented;
}

OpStatus LMS64C_LMS7002M_Over_PCIe::GPIODirWrite(const uint8_t* buffer, const size_t bufLength)
{
    return OpStatus::NotImplemented;
}

OpStatus LMS64C_LMS7002M_Over_PCIe::GPIODirRead(uint8_t* buffer, const size_t bufLength)
{
    return OpStatus::NotImplemented;
}

OpStatus LMS64C_LMS7002M_Over_PCIe::CustomParameterWrite(const std::vector<CustomParameterIO>& parameters)
{
    return OpStatus::NotImplemented;
}

OpStatus LMS64C_LMS7002M_Over_PCIe::CustomParameterRead(std::vector<CustomParameterIO>& parameters)
{
    return OpStatus::NotImplemented;
}

OpStatus LMS64C_LMS7002M_Over_PCIe::ProgramWrite(
    const char* data, size_t length, int prog_mode, int target, ProgressCallback callback)
{
    return OpStatus::NotImplemented;
}

OpStatus LMS64C_LMS7002M_Over_PCIe::MemoryWrite(uint32_t address, const void* data, uint32_t dataLength)
{
    return OpStatus::NotImplemented;
}

OpStatus LMS64C_LMS7002M_Over_PCIe::MemoryRead(uint32_t address, void* data, uint32_t dataLength)
{
    return OpStatus::NotImplemented;
}
