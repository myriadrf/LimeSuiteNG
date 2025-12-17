#pragma once

#include <complex>

namespace lime {

/// Calculates the cross-correlation of two complex signals
template<class T> std::vector<double> CrossCorrelation(const std::vector<T>& samples, const std::vector<T>& reference)
{
    std::vector<double> correlation(samples.size());

    for (size_t lag = 0; lag < samples.size(); ++lag)
    {
        std::complex<double> result{ 0.0, 0.0 };

        const size_t windowLength = std::min(samples.size() - lag, reference.size());
        for (size_t i = 0; i < windowLength; ++i)
        {
            result +=
                std::complex<double>(reference[i].real() * samples[lag + i].real() + reference[i].imag() * samples[lag + i].imag(),
                    reference[i].imag() * samples[lag + i].real() - reference[i].real() * samples[lag + i].imag());
        }
        correlation[lag] = abs(result);
    }
    return correlation;
}

} // namespace lime
