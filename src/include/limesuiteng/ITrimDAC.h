#ifndef LIMESUITENG_ITRIMDAC_H
#define LIMESUITENG_ITRIMDAC_H

#include "limesuiteng/OpStatus.h"
#include "limesuiteng/types.h"

#include <cstdint>
#include <string>
#include <string_view>

namespace lime {

/// @brief Trim DAC that tunes the frequency of an oscillator, e.g. the board's reference VCTCXO.
///
/// The DAC has a volatile value, which is what it outputs now, and a non-volatile value, which the
/// device stores and loads into the DAC at power up. Changing one does not change the other.
class ITrimDAC
{
  public:
    static constexpr std::string_view DeviceTreeClass{ "TrimDAC" };

    virtual ~ITrimDAC() = default;

    /// @return The name of the oscillator the DAC tunes (e.g. "VCTCXO").
    virtual const std::string& GetName() const = 0;

    /// @return The range of the DAC values.
    virtual Range<uint32_t> GetRange() const = 0;

    /// @brief Reads the current (volatile) DAC value.
    virtual OpStatus GetValue(uint32_t& value) = 0;
    /// @brief Sets the DAC output, without storing the value.
    /// @return OpStatus::OutOfRange if the value is outside of GetRange().
    virtual OpStatus SetValue(uint32_t value) = 0;

    /// @brief Reads the value stored in the non-volatile memory, the one loaded at power up.
    /// @return OpStatus::NotSupported if the device does not store the value.
    virtual OpStatus GetNonVolatileValue(uint32_t& value) = 0;
    /// @brief Stores the value to the non-volatile memory and verifies it. Does not change the current DAC output.
    /// @return OpStatus::NotSupported if the device does not store the value, OpStatus::OutOfRange if the
    /// value is outside of GetRange() or does not fit the storage.
    virtual OpStatus SetNonVolatileValue(uint32_t value) = 0;
};

} // namespace lime

#endif
