#pragma once

#include "ISPI.h"
#include "limesuiteng/config.h"
#include "limesuiteng/OpStatus.h"

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace lime {

/// @brief The structure for writing and reading custom parameters
struct CustomParameterIO;

/** @brief An interface for general device communications */
class IComms : public ISPI
{
  public:
    /** @brief Destroys the interfaced object. */
    virtual ~IComms(){};

    /**
      @brief Writes general-purpose input/output (GPIO) values to device.
      @param buffer For source of GPIO values. Least significant bit first, each bit sets GPIO state.
      @param bufLength The length of @p buffer.
      @return The operation success state.
     */
    virtual OpStatus GPIOWrite(const uint8_t* buffer, const size_t bufLength) { return OpStatus::NotImplemented; };

    /**
      @brief Reads general-purpose input/output (GPIO) values from device
      @param[out] buffer Destination for GPIO values. Least significant bit first, each bit represents GPIO state.
      @param bufLength The length of @p buffer.
      @return The operation success state.
     */
    virtual OpStatus GPIORead(uint8_t* buffer, const size_t bufLength) { return OpStatus::NotImplemented; };

    /**
      @brief Write general-purpose input/output (GPIO) direction control values to device.
      @param buffer A buffer of data with GPIO direction configuration (0 - input, 1 - output).
      @param bufLength The length of @p buffer.
      @return The operation success state.
     */
    virtual OpStatus GPIODirWrite(const uint8_t* buffer, const size_t bufLength) { return OpStatus::NotImplemented; };

    /**
      @brief Read general-purpose input/output (GPIO) direction control configuration from device.
      @param[out] buffer A buffer of data with GPIO direction configuration (0 - input, 1 - output).
      @param bufLength The length of @p buffer.
      @return The operation success state.
     */
    virtual OpStatus GPIODirRead(uint8_t* buffer, const size_t bufLength) { return OpStatus::NotImplemented; };

    /**
      @brief Writes a given list of custom parameters to the device.
      @param parameters The list of parameters to write.
      @return The operation success state.
     */
    virtual OpStatus CustomParameterWrite(const std::vector<CustomParameterIO>& parameters) { return OpStatus::NotImplemented; };

    /**
      @brief Reads a given list of custom parameters from the device.
      @param[inout] parameters The list of parameters to read. The read values will end up in this vector.
      @return The operation success state.
     */
    virtual OpStatus CustomParameterRead(std::vector<CustomParameterIO>& parameters) { return OpStatus::NotImplemented; };

    /**
      @brief The function to call when writing a program into the device using @ref lime::IComms::ProgramWrite().
      @param bytesSent The amount of bytes already sent to the device.
      @param bytesTotal The total amount of bytes to send.
      @param progressMsg A C-string describing the current status of the data transfer.
      @return Whether to stop writing the data or not (true - stop, false - continue).
     */
    typedef std::function<bool(std::size_t bsent, std::size_t btotal, const std::string&)> ProgressCallback;

    /**
      @brief Writes the given program into the device.
      @param data The program to write.
      @param length The length of the program.
      @param prog_mode The mode with which to write to the device.
      @param target The device to write into.
      @param callback The progress callback to call when writing data.
      @return The operation success state.
     */
    virtual OpStatus ProgramWrite(const char* data, size_t length, int prog_mode, int target, ProgressCallback callback = nullptr)
    {
        return OpStatus::NotImplemented;
    }

    /**
      @brief Resets the selected device.
      @param chipSelect Which chip to reset.
      @return Whether the operation succeeded or not.
     */
    virtual OpStatus ResetDevice(int chipSelect) { return OpStatus::NotImplemented; };

    /**
      @brief Writes given values into a given memory address in EEPROM memory.
      @param address The address in the EEPROM memory to write into.
      @param data The data to write into the memory.
      @param dataLength The length of the data to write.
      @return The operation success state.
     */
    virtual OpStatus MemoryWrite(uint32_t address, const void* data, uint32_t dataLength) { return OpStatus::NotImplemented; };

    /**
      @brief Reads memory from a given memory address in EEPROM memory.
      @param address The address from which to start reading the memory.
      @param[out] data The destination buffer.
      @param dataLength The length of the destination buffer (the amount of bytes to read).
      @return The operation success state.
     */
    virtual OpStatus MemoryRead(uint32_t address, void* data, uint32_t dataLength) { return OpStatus::NotImplemented; };
};

} // namespace lime
