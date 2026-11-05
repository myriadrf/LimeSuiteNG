#ifndef LIMESUITENG_ISENSOR_H
#define LIMESUITENG_ISENSOR_H

#include "limesuiteng/OpStatus.h"

#include <cstdint>
#include <string>

namespace lime {

/// @brief A read-only measurement or status value provided by a chip (e.g. temperature, PLL lock).
class ISensor
{
  public:
    /// @brief The units the sensor value is expressed in.
    enum class Units : uint8_t {
        None, ///< Dimensionless value.
        Boolean, ///< 0 - false, 1 - true (e.g. PLL lock status).
        Celsius, ///< Degrees Celsius.
        Volt, ///< Volts.
        Ampere, ///< Amperes.
        Watt, ///< Watts.
        Hertz, ///< Hertz.
        dB, ///< Decibels.
        dBm, ///< Decibels relative to 1 mW.
    };

    virtual ~ISensor() = default;

    /// @return The name of the sensor (e.g. "Temperature").
    virtual const std::string& GetName() const = 0;
    /// @return The units of the value returned by GetValue().
    virtual Units GetUnits() const = 0;
    /// @brief Reads the current sensor value.
    /// @param value Output: the reading, in GetUnits() units. Left unchanged if the read fails.
    /// @return The status of the operation.
    virtual OpStatus GetValue(double& value) = 0;
};

} // namespace lime

#endif
