#include "CommsToDevice.h"

namespace lime {
/**
  @brief Constructs the Serial Peripheral Interface communication layer.
  @param sdr The device to communicate with.
  @param spiDefaultSlave The default slave ID to use.
 */
SPIToSDR::SPIToSDR(lime::SDRDevice& sdr, uint32_t spiDefaultSlave)
    : device(sdr)
    , mSPIDefaultSlaveId(spiDefaultSlave)
{
}

OpStatus SPIToSDR::SPI(const uint32_t* MOSI, uint32_t* MISO, uint32_t count)
{
    return device.SPI(mSPIDefaultSlaveId, MOSI, MISO, count);
}

OpStatus SPIToSDR::SPI(uint32_t spiBusAddress, const uint32_t* MOSI, uint32_t* MISO, uint32_t count)
{
    return device.SPI(spiBusAddress, MOSI, MISO, count);
}

/**
  @brief Constructs the Inter-Integrated Circuit communication layer.
  @param sdr The device to communicate with.
 */
I2CToSDR::I2CToSDR(lime::SDRDevice& sdr)
    : device(sdr)
{
}

OpStatus I2CToSDR::I2CWrite(uint16_t address, uint32_t offset, uint8_t offset_len, const uint8_t* data, uint32_t data_length)
{
    return device.I2CWrite(0, address, offset, offset_len, data, data_length);
}

OpStatus I2CToSDR::I2CRead(uint16_t address, uint32_t offset, uint8_t offset_len, uint8_t* data, uint32_t data_length)
{
    return device.I2CRead(0, address, offset, offset_len, data, data_length);
}

} // namespace lime