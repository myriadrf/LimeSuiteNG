#pragma once

#include "limesuiteng/complex.h"
#include "limesuiteng/config.h"
#include <vector>

namespace lime {

/// @brief Class for calculating Fast Fourier Transforms.
class FFT
{
  public:
    /// @brief Enumeration for selecting the window coefficient function to use
    enum class WindowFunctionType { NONE = 0, BLACKMAN_HARRIS, HAMMING, HANNING };

    /// @brief Generates the coefficients for a given window function
    /// @param type The type of the window function to generate the coefficients for.
    /// @param coefCount The amount of coefficients to generate.
    /// @param coefs The buffer to which to store the coefficients.
    LIME_API static void GenerateWindowCoefficients(WindowFunctionType type, uint32_t coefCount, std::vector<float>& coefs);

    /// @brief Calculates the FFT bins from the provided samples.
    /// @param samples The samples to calculate from.
    /// @param window The window function to apply to the samples before calculating the bins.
    /// @return The array of computed bins, the same width as the amount of samples.
    static std::vector<float> Calc(const std::vector<complex32f_t>& samples, WindowFunctionType window = WindowFunctionType::NONE);

    /// @brief Convert the amplitude in the bins to a Decibels relative to full scale.
    /// @param bins The bins to convert.
    static void ConvertToDBFS(std::vector<float>& bins);
};

} // namespace lime