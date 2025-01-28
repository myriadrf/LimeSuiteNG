#ifndef LIME_LMS64C_ADF_OVER_USB_LIMESDR_H
#define LIME_LMS64C_ADF_OVER_USB_LIMESDR_H

#include "comms/IComms.h"
#include "comms/ISerialPort.h"

#include <cstdint>
#include <memory>

namespace lime {

/** @brief A class for communicating with LimeSDR's subdevice's ADF4002 chips. */
class LMS64C_ADF_Over_USB : public ISPI
{
  public:
    /**
      @brief Constructs a new LMS64C_ADF_Over_USB object
      @param serialPort The serial port the ADF4002 is connected to
      @param subdeviceIndex The subdevice index of the ADF4002 on serialPort
     */
    LMS64C_ADF_Over_USB(std::shared_ptr<ISerialPort> serialPort, uint32_t subdeviceIndex);

    OpStatus SPI(const uint32_t* MOSI, uint32_t* MISO, uint32_t count) override;
    OpStatus SPI(uint32_t spiBusAddress, const uint32_t* MOSI, uint32_t* MISO, uint32_t count) override;

  private:
    std::shared_ptr<ISerialPort> mSerialPort;
    uint32_t mSubdeviceIndex;
};

} // namespace lime

#endif // LIME_LMS64C_ADF_OVER_USB_LIMESDR_H
