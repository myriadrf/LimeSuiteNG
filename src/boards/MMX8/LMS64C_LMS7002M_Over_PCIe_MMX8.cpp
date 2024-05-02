#include "LMS64C_LMS7002M_Over_PCIe_MMX8.h"
#include "protocols/LMS64CProtocol.h"

using namespace lime;

LMS64C_LMS7002M_Over_PCIe_MMX8::LMS64C_LMS7002M_Over_PCIe_MMX8(std::shared_ptr<LitePCIe> dataPort, uint32_t subdeviceIndex)
    : pipe(dataPort)
    , subdeviceIndex(subdeviceIndex)
{
}

OpStatus LMS64C_LMS7002M_Over_PCIe_MMX8::SPI(const uint32_t* MOSI, uint32_t* MISO, uint32_t count)
{
    return SPI(0, MOSI, MISO, count);
}

OpStatus LMS64C_LMS7002M_Over_PCIe_MMX8::SPI(uint32_t spiBusAddress, const uint32_t* MOSI, uint32_t* MISO, uint32_t count)
{
    return LMS64CProtocol::LMS7002M_SPI(pipe, spiBusAddress, MOSI, MISO, count, subdeviceIndex);
}

OpStatus LMS64C_LMS7002M_Over_PCIe_MMX8::ResetDevice(int chipSelect)
{
    return LMS64CProtocol::DeviceReset(pipe, chipSelect, subdeviceIndex);
}

OpStatus LMS64C_LMS7002M_Over_PCIe_MMX8::GPIOWrite(const uint8_t* buffer, const size_t bufLength)
{
    return OpStatus::NotImplemented;
}

OpStatus LMS64C_LMS7002M_Over_PCIe_MMX8::GPIORead(uint8_t* buffer, const size_t bufLength)
{
    return OpStatus::NotImplemented;
}

OpStatus LMS64C_LMS7002M_Over_PCIe_MMX8::GPIODirWrite(const uint8_t* buffer, const size_t bufLength)
{
    return OpStatus::NotImplemented;
}

OpStatus LMS64C_LMS7002M_Over_PCIe_MMX8::GPIODirRead(uint8_t* buffer, const size_t bufLength)
{
    return OpStatus::NotImplemented;
}

OpStatus LMS64C_LMS7002M_Over_PCIe_MMX8::CustomParameterWrite(const std::vector<CustomParameterIO>& parameters)
{
    return OpStatus::NotImplemented;
}

OpStatus LMS64C_LMS7002M_Over_PCIe_MMX8::CustomParameterRead(std::vector<CustomParameterIO>& parameters)
{
    return OpStatus::NotImplemented;
}

OpStatus LMS64C_LMS7002M_Over_PCIe_MMX8::ProgramWrite(
    const char* data, size_t length, int prog_mode, int target, ProgressCallback callback)
{
    return OpStatus::NotImplemented;
}

OpStatus LMS64C_LMS7002M_Over_PCIe_MMX8::MemoryWrite(uint32_t address, const void* data, uint32_t dataLength)
{
    return OpStatus::NotImplemented;
}

OpStatus LMS64C_LMS7002M_Over_PCIe_MMX8::MemoryRead(uint32_t address, void* data, uint32_t dataLength)
{
    return OpStatus::NotImplemented;
}
