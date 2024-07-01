#include "FFT.h"

#include <assert.h>
#include <chrono>
#include "../external/kissFFT/kiss_fft.h"

namespace lime {

std::vector<float> FFT::Calc(const std::vector<complex32f_t>& samples, WindowFunctionType window)
{
    const int fftSize = samples.size();
    std::vector<float> coefs;
    GenerateWindowCoefficients(window, fftSize, coefs);

    std::vector<kiss_fft_cpx> fftIn(fftSize);
    std::vector<kiss_fft_cpx> fftOut(fftSize);
    std::vector<float> bins(fftSize);
    kiss_fft_cfg plan = kiss_fft_alloc(fftSize, 0, 0, 0);

    for (int i = 0; i < fftSize; ++i)
    {
        fftIn[i].r = samples[i].real() * coefs[i];
        fftIn[i].i = samples[i].imag() * coefs[i];
    }
    kiss_fft(plan, fftIn.data(), fftOut.data());
    for (int i = 0; i < fftSize; ++i)
    {
        bins[i] = (fftOut[i].r * fftOut[i].r + fftOut[i].i * fftOut[i].i) / (fftSize * fftSize);
    }
    kiss_fft_free(plan);
    return bins;
}

void FFT::ConvertToDBFS(std::vector<float>& bins)
{
    for (float& amplitude : bins)
        amplitude = amplitude > 0 ? 10 * log10(amplitude) : -150;
}

void FFT::GenerateWindowCoefficients(WindowFunctionType func, uint32_t N /*coef count*/, std::vector<float>& windowFcoefs)
{
    float amplitudeCorrection = 1;
    windowFcoefs.resize(N);
    float a0 = 0.35875;
    float a1 = 0.48829;
    float a2 = 0.14128;
    float a3 = 0.01168;
    float PI = 3.14159265359;
    switch (func)
    {
    case WindowFunctionType::BLACKMAN_HARRIS:
        for (uint32_t i = 0; i < N; ++i)
            windowFcoefs[i] =
                a0 - a1 * cos((2 * PI * i) / (N - 1)) + a2 * cos((4 * PI * i) / (N - 1)) - a3 * cos((6 * PI * i) / (N - 1));
        break;
    case WindowFunctionType::HAMMING:
        amplitudeCorrection = 0;
        a0 = 0.54;
        for (uint32_t i = 0; i < N; ++i)
            windowFcoefs[i] = a0 - (1 - a0) * cos((2 * PI * i) / (N));
        break;
    case WindowFunctionType::HANNING:
        amplitudeCorrection = 0;
        for (uint32_t i = 0; i < N; ++i)
            windowFcoefs[i] = 0.5 * (1 - cos((2 * PI * i) / (N)));
        break;
    case WindowFunctionType::NONE:
    default:
        for (uint32_t i = 0; i < N; ++i)
            windowFcoefs[i] = 1;
        return;
    }
    for (uint32_t i = 0; i < N; ++i)
        amplitudeCorrection += windowFcoefs[i];
    amplitudeCorrection = 1.0 / (amplitudeCorrection / N);
    for (uint32_t i = 0; i < N; ++i)
        windowFcoefs[i] *= amplitudeCorrection;
}

} // namespace lime
