#ifndef LIMESUITENG_TYPES_H
#define LIMESUITENG_TYPES_H

/**
* @file limesuiteng/types.h
* @author Lime Microsystems
* @brief File with API types definitions.
*/

#include "limesuiteng/config.h"
#include <cstdint>
#include <string>

namespace lime {

/// @brief The direction of the transmission
enum class TRXDir : bool 
{ 
    Rx, ///< Receiver direction.
    Tx  ///< Transmiter direction.
};

/// @brief Enumerator describing the data formats.
enum class DataFormat : uint8_t {
    I16, ///< 16-bit integers.
    I12, ///< 12-bit integers. Stored as int16_t, but the expected range is [-2048;2047]
    F32, ///< 32-bit floating-point.
};

/// @brief Available gain types on the devices.
enum class eGainTypes : uint8_t {
    LNA,                ///< Receiver Low Noise Amplifier.
    LoopbackLNA,        ///< Receiver with loopback buffer Low Noise Amplifier.
    PGA,                ///< Receiver Programmable Gain Amplifier.
    TIA,                ///< Receiver Trans Impedance Amplifier.
    PAD,                ///< Transmitter Programmable Amplifier Driver.
    LoopbackPAD,        ///< Transmitter loopback Programmable Amplifier Driver.
    IAMP,               ///< TBB (Transmitter baseband) frontend amplifier.
    PA,                 ///< On-board Power Amplifier.
    UNKNOWN,            ///< Not supported by API.
    GENERIC = UNKNOWN,  ///< Not supported by API.
};

/// @brief Structure describing the range possible.
template<class T> struct Range {
    /// @brief Constructs the range structure,
    /// @param min The minimum value of the range (default 0.0)
    /// @param max The maximum value of the range (default 0.0)
    /// @param step The step of the range (default 0.0 - no step)
    constexpr Range(T min = 0, T max = 0, T step = 1)
        : min(min)
        , max(max)
        , step(step){};
    T min; ///< The minimum value of the range
    T max; ///< The maximum value of the range
    T step; ///< The step of the range (or 0.0 for any step)
};

/// @brief Structure for storing the information of a memory region.
struct Region {
    int32_t address; ///< Starting address of the memory region
    int32_t size; ///< The size of the memory region
};

enum class eMemoryDevice : uint8_t { FPGA_RAM = 0, FPGA_FLASH, EEPROM, GATEWARE_GOLD_IMAGE, GATEWARE_USER_IMAGE, COUNT };

/// @brief The structure for writing and reading custom parameters
struct CustomParameterIO {
    int32_t id; ///< The ID of the parameter
    double value; ///< The value of the parameter.
    std::string units; ///< The units of the parameter.
};

} // namespace lime

#endif
