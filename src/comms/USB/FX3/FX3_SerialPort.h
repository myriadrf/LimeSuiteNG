#ifndef LIME_FX3_SERIALPORT_H
#define LIME_FX3_SERIALPORT_H

#include "comms/ISerialPort.h"

#include <cstdint>

namespace lime {

class FX3;

/** @brief Class for interfacing with Control/Status registers (CSR) of LimeSDR-USB. */
class FX3_SerialPort : public ISerialPort
{
  public:
    /**
      @brief Constructs a new FX3_SerialPort object
      @param port The FX3 communications port to use.
     */
    explicit FX3_SerialPort(FX3& port);

    int Write(const uint8_t* data, std::size_t length, int timeout_ms) override;
    int Read(uint8_t* data, std::size_t length, int timeout_ms) override;
    OpStatus RunControlCommand(uint8_t* data, size_t length, int timeout_ms) override;
    OpStatus RunControlCommand(uint8_t* request, uint8_t* response, size_t length, int timeout_ms) override;

  private:
    FX3& port;
};

} // namespace lime

#endif // LIME_FX3_SERIALPORT_H
