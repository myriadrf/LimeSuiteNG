#ifndef LIME_SPI_UTILITIES_H
#define LIME_SPI_UTILITIES_H

#include "ISPI.h"
#include "limesuiteng/OpStatus.h"

namespace lime {

class SDRDevice;

/// @brief Wrapper class to redirect default SPI operations to specific chip
class SPIChipSelectShim : public lime::ISPI
{
  public:
    SPIChipSelectShim(lime::SDRDevice* sdrdev, uint32_t chipSelect);

    lime::OpStatus SPI(const uint32_t* MOSI, uint32_t* MISO, uint32_t count) override;
    lime::OpStatus SPI(uint32_t chipSelect, const uint32_t* MOSI, uint32_t* MISO, uint32_t count) override;

  private:
    lime::SDRDevice* device;
    uint32_t defaultChipSelect;
};

} // namespace lime

#endif // LIME_SPI_UTILITIES_H