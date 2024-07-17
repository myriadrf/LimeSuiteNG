#ifndef LIME_ISERIALPORT_H
#define LIME_ISERIALPORT_H

#include <cstddef>
#include <cstdint>
#include "limesuiteng/OpStatus.h"

namespace lime {

/** @brief Interface for directly reading or writing data to/from the device. */
class ISerialPort
{
  public:
    virtual ~ISerialPort(){};

    /**
    @brief Writes the specified data into whatever device is implementing this interface.
    
    @param data The data to write to the device.
    @param length The length of the data.
    @param timeout_ms The timeout (in ms) to wait until the transfer times out.
    @return The amount of bytes written.
   */
    virtual int Write(const uint8_t* data, std::size_t length, int timeout_ms) = 0;

    /**
      @brief Reads some data from the device.
      
      @param data The buffer in which to store the read data.
      @param length The length of the data to store.
      @param timeout_ms The timeout (in ms) to wait until the transfer times out.
      @return The amount of bytes read.
     */
    virtual int Read(uint8_t* data, std::size_t length, int timeout_ms) = 0;

    virtual OpStatus RunControlCommand(uint8_t* data, size_t length, int timeout_ms = 100) = 0;
    virtual OpStatus RunControlCommand(uint8_t* request, uint8_t* response, size_t length, int timeout_ms = 100) = 0;
};

} // namespace lime

#endif // LIME_ISERIALPORT_H
