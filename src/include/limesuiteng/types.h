#ifndef LIMESUITENG_TYPES_H
#define LIMESUITENG_TYPES_H

#include "limesuiteng/config.h"
#include <cstdint>
#include <string>

namespace lime {

/// @brief The direction of the transmission
enum class TRXDir : bool { Rx, Tx };

/// @brief Enumerator describing the data formats.
enum class DataFormat : uint8_t {
    I16, ///< 16-bit integers.
    I12, ///< 12-bit integers. Stored as int16_t, but the expected range is [-2048;2047]
    F32, ///< 32-bit floating-point.
};

/// @brief Available gain types on the devices.
enum class eGainTypes : uint8_t {
    LNA, ///< Low Noise Amplifier
    LoopbackLNA,
    PGA, ///< Programmable Gain Amplifier
    TIA, ///< Trans Impedance Amplifier
    PAD,
    LoopbackPAD,
    IAMP,
    PA, ///< On-board Power Amplifier
    UNKNOWN,
    GENERIC = UNKNOWN,
};

/// @brief Structure describing the range possible.
struct Range {
    /// @brief Constructs the range structure,
    /// @param min The minimum value of the range (default 0.0)
    /// @param max The maximum value of the range (default 0.0)
    /// @param step The step of the range (default 0.0 - no step)
    constexpr Range(double min = 0.0, double max = 0.0, double step = 0.0)
        : min(min)
        , max(max)
        , step(step){};
    double min; ///< The minimum value of the range
    double max; ///< The maximum value of the range
    double step; ///< The step of the range (or 0.0 for any step)
};

enum class eMemoryRegion : uint8_t { VCTCXO_DAC, COUNT };

/// @brief Structure for storing the information of a memory region.
struct Region {
    int32_t address; ///< Starting address of the memory region
    int32_t size; ///< The size of the memory region
};

enum class eMemoryDevice : uint8_t { FPGA_RAM = 0, FPGA_FLASH, EEPROM, COUNT };

/// @brief The structure for writing and reading custom parameters
struct CustomParameterIO {
    int32_t id; ///< The ID of the parameter
    double value; ///< The value of the parameter.
    std::string units; ///< The units of the parameter.
};

} // namespace lime

#endif
