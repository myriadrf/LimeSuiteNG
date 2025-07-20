#ifndef LIME_FTDI_SerialPort_H
#define LIME_FTDI_SerialPort_H

#include "comms/ISerialPort.h"

namespace lime {

class FT601;

/** @brief Class for interfacing with Control/Status registers (CSR) of LimeSDR-Mini. */
class FTDI_SerialPort : public ISerialPort
{
  public:
    /**
      @brief Constructs a new FTDI_SerialPort object
      @param port The FT601 communications port to use.
     */
    explicit FTDI_SerialPort(FT601& port);

    int Write(const uint8_t* data, std::size_t length, int timeout_ms) override;
    int Read(uint8_t* data, std::size_t length, int timeout_ms) override;
    OpStatus RunControlCommand(uint8_t* data, size_t length, int timeout_ms) override;
    OpStatus RunControlCommand(uint8_t* request, uint8_t* response, size_t length, int timeout_ms) override;

  private:
    FT601& port;
};

} // namespace lime

#endif // LIME_FTDI_SerialPort_H
