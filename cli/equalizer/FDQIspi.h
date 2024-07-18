#pragma once

#include "limesuiteng/Register.h"
#include "limesuiteng/SDRDevice.h"
#include "comms/ISPI.h"

class FDQI_SPI : public lime::ISPI
{
  public:
    FDQI_SPI(lime::SDRDevice* sdrdev, int32_t chipSelect);
    virtual ~FDQI_SPI();

    lime::OpStatus SPI(const uint32_t* MOSI, uint32_t* MISO, uint32_t count) override;
    lime::OpStatus SPI(uint32_t chipSelect, const uint32_t* MOSI, uint32_t* MISO, uint32_t count);

  private:
    lime::SDRDevice* sdrdev;
    int32_t defaultChipSelect;
};
