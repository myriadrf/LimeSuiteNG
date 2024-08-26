/**
  @file ADCUnits.h
  @author Lime Microsystems
  @brief Enumerations of ADC sensor units
*/
#pragma once

#include <array>
#include <string_view>

namespace lime {

/// @brief The available units to store the information as.
enum eADC_UNITS { RAW, VOLTAGE, CURRENT, RESISTANCE, POWER, TEMPERATURE, ADC_UNITS_COUNT };

static constexpr std::array<const std::string_view, ADC_UNITS_COUNT> ADC_UNITS_TEXT{ "", "V", "A", "Ohm", "W", "C" };
static constexpr std::string_view UNKNOWN{ " unknown" };

static constexpr const std::string_view adcUnits2string(const unsigned units)
{
    if (units < ADC_UNITS_COUNT)
    {
        return ADC_UNITS_TEXT.at(units);
    }

    return UNKNOWN;
}

} // namespace lime
