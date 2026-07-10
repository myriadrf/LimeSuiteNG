/* C wrapper for the SPI subinterface, over lime::SDRDevice::SPI. */
#include "limesuiteng/spi_c.h"
#include "private.h"

using lime::SDRDevice;

extern "C" {

lime_OpStatus lime_spi_transact(lime_SPI* spi, uint32_t bus_address, const uint32_t* mosi, uint32_t* miso, uint32_t count)
{
    if (spi == nullptr || (mosi == nullptr && miso == nullptr) || count == 0)
        return lime_OpStatus_InvalidValue;
    SDRDevice* dev = reinterpret_cast<SDRDevice*>(spi);
    return static_cast<lime_OpStatus>(dev->SPI(bus_address, mosi, miso, count));
}

} /* extern "C" */
