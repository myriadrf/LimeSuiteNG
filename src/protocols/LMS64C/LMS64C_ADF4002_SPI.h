#ifndef LIME_LMS64C_ADF4002_H
#define LIME_LMS64C_ADF4002_H

#include "comms/SPI/ISPI.h"

#include <cstdint>
#include <memory>

namespace lime {

class ISerialPort;

/** @brief A class for communicating with LimeSDR's subdevice's ADF4002 chips. */
class LMS64C_ADF4002_SPI : public ISPI
{
  public:
    /**
      @param serialPort The serial port the ADF4002 is connected to
      @param subdeviceIndex The subdevice index of the ADF4002 on serialPort
     */
    LMS64C_ADF4002_SPI(std::shared_ptr<ISerialPort> serialPort, uint32_t subdeviceIndex);

    OpStatus Transact(const uint32_t* MOSI, uint32_t* MISO, uint32_t count) override;

  private:
    std::shared_ptr<ISerialPort> mSerialPort;
    uint32_t mSubdeviceIndex;
};

} // namespace lime

#endif // LIME_LMS64C_ADF4002_H
