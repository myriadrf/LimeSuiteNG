#ifndef LIMESUITENG_IGFIR_H
#define LIMESUITENG_IGFIR_H

#include "limesuiteng/OpStatus.h"
#include "limesuiteng/types.h"

#include <cstddef>
#include <span>
#include <vector>

namespace lime {

/// @brief General purpose digital FIR filter of a single RF SoC channel.
///
/// Coefficients are normalized to the [-1; 1] range.
class IGFIR
{
  public:
    virtual ~IGFIR() = default;

    /// @return The direction of the signal path this filter is in.
    virtual TRXDir GetDirection() const = 0;

    /// @brief Enables the filter, or bypasses it when disabled.
    virtual OpStatus Enable(bool enable) = 0;
    virtual bool IsEnabled() = 0;

    /// @return The number of coefficients that can be currently set, depends on the current sample rate.
    virtual std::size_t GetMaxCoefficientCount() = 0;
    /// @brief Writes the coefficients. If fewer than the maximum count are given, the rest are set to 0.
    virtual OpStatus SetCoefficients(std::span<const double> coefficients) = 0;
    /// @brief Reads back the coefficients.
    /// @param coefficients Output, resized to the number of coefficients the filter holds.
    virtual OpStatus GetCoefficients(std::vector<double>& coefficients) = 0;
};

} // namespace lime

#endif
