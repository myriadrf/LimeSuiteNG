#ifndef WIDGET_GAIN_STAGE_FORMAT_H
#define WIDGET_GAIN_STAGE_FORMAT_H

#include "limesuiteng/IGainStage.h"

#include <cmath>

#include <wx/string.h>

/// @brief Helpers shared by the lime::IGainStage GUI panels.
namespace GainStageFormat {

inline wxString UnitsToString(lime::IGainStage::Units units)
{
    switch (units)
    {
    case lime::IGainStage::Units::dB:
        return wxT("dB");
    case lime::IGainStage::Units::Linear:
        return wxT("x");
    case lime::IGainStage::Units::Raw:
        return wxT("(raw)");
    }
    return wxEmptyString;
}

/// @return The spin control increment: the range step, or a guess for stages without evenly spaced values.
inline double GetIncrement(const lime::Range<double>& range)
{
    if (range.step > 0)
        return range.step;
    return (range.max - range.min) <= 10 ? 0.1 : 1;
}

/// @return The number of decimal digits needed to show the increment.
inline unsigned GetDigits(double increment)
{
    unsigned digits = 0;
    while (digits < 6 && std::fabs(increment - std::round(increment)) > 1e-9)
    {
        increment *= 10;
        ++digits;
    }
    return digits;
}

} // namespace GainStageFormat

#endif
