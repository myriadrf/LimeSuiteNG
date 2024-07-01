#pragma once

#include "limesuiteng/complex.h"
#include "limesuiteng/config.h"
#include <vector>

namespace lime {

class FFT
{
  public:
    enum class WindowFunctionType { NONE = 0, BLACKMAN_HARRIS, HAMMING, HANNING };
    LIME_API static void GenerateWindowCoefficients(WindowFunctionType type, uint32_t coefCount, std::vector<float>& coefs);
    static std::vector<float> Calc(const std::vector<complex32f_t>& samples, WindowFunctionType window = WindowFunctionType::NONE);
    static void ConvertToDBFS(std::vector<float>& bins);
};

} // namespace lime