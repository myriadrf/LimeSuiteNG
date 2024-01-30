/**
  @file ADCUnits.h
  @author Lime Microsystems
  @brief Enumerations of ADC sensor units
*/
#pragma once

#include <array>
#include <string_view>

using namespace std::literals::string_view_literals;

namespace lime {

enum eADC_UNITS { RAW, VOLTAGE, CURRENT, RESISTANCE, POWER, TEMPERATURE, ADC_UNITS_COUNT };

static constexpr std::array<const std::string_view, ADC_UNITS_COUNT> ADC_UNITS_TEXT{ ""sv, "V"sv, "A"sv, "Ohm"sv, "W"sv, "C"sv };
static constexpr std::string_view UNKNOWN{ " unknown"sv };

static constexpr const std::string_view adcUnits2string(const unsigned units)
{
    if (units < ADC_UNITS_COUNT)
    {
        return ADC_UNITS_TEXT.at(units);
    }

    return UNKNOWN;
}

} // namespace lime
