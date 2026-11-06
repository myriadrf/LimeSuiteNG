#ifndef LIMESUITENG_IGAINSTAGE_H
#define LIMESUITENG_IGAINSTAGE_H

#include "limesuiteng/OpStatus.h"
#include "limesuiteng/types.h"

#include <cstdint>
#include <string>

namespace lime {

/// @brief A single adjustable amplifier or attenuator in the signal path of an RF SoC channel.
class IGainStage
{
  public:
    /// @brief The units the gain value is expressed in.
    enum class Units : uint8_t {
        Raw, ///< Hardware specific register code, without a defined relation to the actual gain.
        Linear, ///< Linear gain factor.
        dB, ///< Decibels.
    };

    virtual ~IGainStage() = default;

    /// @return The name of the gain stage (e.g. "LNA").
    virtual const std::string& GetName() const = 0;
    /// @return The units of the values accepted by SetValue() and returned by GetValue() and GetRange().
    virtual Units GetUnits() const = 0;
    /// @return The range of values the gain stage can be set to. A step of 0 means the valid values are not evenly spaced,
    /// and the stage uses the value it can get closest to.
    virtual Range<double> GetRange() const = 0;

    /// @brief Sets the gain.
    /// @param value The gain, in GetUnits() units.
    virtual OpStatus SetValue(double value) = 0;
    /// @brief Reads the current gain.
    /// @param value Output: the gain, in GetUnits() units. Left unchanged if the read fails.
    /// @return The status of the operation.
    virtual OpStatus GetValue(double& value) = 0;
};

} // namespace lime

#endif
