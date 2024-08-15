#ifndef LIME_USB_CSR_PIPE_H
#define LIME_USB_CSR_PIPE_H

#include "ISerialPort.h"

namespace lime {

/** @brief An abstract class for interfacing with Control/Status registers (CSR) of a USB device. */
class USB_CSR_Pipe : public ISerialPort
{
  public:
    int Write(const uint8_t* data, std::size_t length, int timeout_ms) override = 0;
    int Read(uint8_t* data, std::size_t length, int timeout_ms) override = 0;
    OpStatus RunControlCommand(uint8_t* data, size_t length, int timeout_ms = 100) override = 0;
    OpStatus RunControlCommand(uint8_t* request, uint8_t* response, size_t length, int timeout_ms = 100) override = 0;
};

} // namespace lime

#endif // LIME_USB_CSR_PIPE_H
