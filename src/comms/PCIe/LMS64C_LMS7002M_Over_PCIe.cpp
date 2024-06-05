#include "LMS64C_LMS7002M_Over_PCIe.h"
#include "protocols/LMS64CProtocol.h"

using namespace lime;

LMS64C_LMS7002M_Over_PCIe::LMS64C_LMS7002M_Over_PCIe(std::shared_ptr<LimePCIe> dataPort)
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
