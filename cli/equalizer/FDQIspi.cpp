#include "FDQIspi.h"

using namespace lime;

FDQI_SPI::FDQI_SPI(SDRDevice* sdrdev, int32_t chipSelect)
    : sdrdev(sdrdev)
    , defaultChipSelect(chipSelect)
{
}

FDQI_SPI::~FDQI_SPI()
{
}

OpStatus FDQI_SPI::SPI(const uint32_t* MOSI, uint32_t* MISO, uint32_t count)
{
    return sdrdev->SPI(defaultChipSelect, MOSI, MISO, count);
}

OpStatus FDQI_SPI::SPI(uint32_t chipSelect, const uint32_t* MOSI, uint32_t* MISO, uint32_t count)
{
    return sdrdev->SPI(chipSelect, MOSI, MISO, count);
}
