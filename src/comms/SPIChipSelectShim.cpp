#include "SPIChipSelectShim.h"
#include "limesuiteng/SDRDevice.h"

namespace lime {

SPIChipSelectShim::SPIChipSelectShim(SDRDevice* sdrdev, uint32_t chipSelect)
    : device(sdrdev)
    , defaultChipSelect(chipSelect)
{
}

OpStatus SPIChipSelectShim::SPI(const uint32_t* MOSI, uint32_t* MISO, uint32_t count)
{
    return device->SPI(defaultChipSelect, MOSI, MISO, count);
}

OpStatus SPIChipSelectShim::SPI(uint32_t chipSelect, const uint32_t* MOSI, uint32_t* MISO, uint32_t count)
{
    return device->SPI(chipSelect, MOSI, MISO, count);
}

} // namespace lime
