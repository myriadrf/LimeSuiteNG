#pragma once

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <vector>

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

/// Result of CrossCorrelationPeak()
struct CrossCorrelationPeakResult {
    size_t lag{ 0 }; ///< Lag (index into samples) with the highest correlation
    double magnitude{ 0.0 }; ///< Correlation magnitude at that lag, same scale as CrossCorrelation()
};

namespace detail {
/// Correlation magnitude at a single lag, same definition as CrossCorrelation().
/// Accumulates in plain doubles so it stays reasonably fast in unoptimized builds.
template<class T>
double CrossCorrelationAtLag(const T* samples, size_t samplesCount, const T* reference, size_t referenceCount, size_t lag)
{
    const size_t windowLength = std::min(samplesCount - lag, referenceCount);
    const T* s = samples + lag;
    double re = 0;
    double im = 0;
    for (size_t i = 0; i < windowLength; ++i)
    {
        const double rr = reference[i].real();
        const double ri = reference[i].imag();
        const double sr = s[i].real();
        const double si = s[i].imag();
        re += rr * sr + ri * si;
        im += ri * sr - rr * si;
    }
    return std::sqrt(re * re + im * im);
}
} // namespace detail

/// @brief Finds the lag of the maximum cross-correlation without computing the full correlation.
///
/// Multi-pass search: the first pass evaluates every @p coarseStep lag over the whole samples range.
/// The best @p candidatesCount local peaks (at least one step apart) are kept, and each following pass
/// re-evaluates only the neighbourhood (+-previous step) of those candidates with a step
/// @p stepDivider times smaller, until the step reaches 1.
///
/// @p coarseStep must be smaller than the correlation main lobe width (roughly 1/bandwidth of the
/// reference signal, in samples), otherwise the peak can fall between coarse points and be missed.
/// With coarseStep = 1 the result is identical to taking the maximum of CrossCorrelation().
///
/// @param samples The signal to search in.
/// @param reference The signal to search for.
/// @param coarseStep Lag step of the first pass.
/// @param candidatesCount How many peaks to keep between passes (guards against picking a sidelobe or noise).
/// @param stepDivider Factor the step is reduced by on each pass (>= 2).
/// @return Lag and magnitude of the maximum correlation.
template<class T>
CrossCorrelationPeakResult CrossCorrelationPeak(const std::vector<T>& samples,
    const std::vector<T>& reference,
    size_t coarseStep = 16,
    size_t candidatesCount = 8,
    size_t stepDivider = 4)
{
    CrossCorrelationPeakResult best;
    if (samples.empty() || reference.empty())
        return best;

    coarseStep = std::max<size_t>(coarseStep, 1);
    candidatesCount = std::max<size_t>(candidatesCount, 1);
    stepDivider = std::max<size_t>(stepDivider, 2);

    struct Point {
        size_t lag;
        double magnitude;
    };

    const auto evaluate = [&](size_t lag) -> Point {
        return { lag, detail::CrossCorrelationAtLag(samples.data(), samples.size(), reference.data(), reference.size(), lag) };
    };

    // Picks up to candidatesCount strongest points, rejecting points closer than minDistance to an already chosen one,
    // so that the candidates don't all end up on the same correlation lobe.
    const auto selectCandidates = [&](std::vector<Point>& points, size_t minDistance) {
        std::sort(points.begin(), points.end(), [](const Point& a, const Point& b) { return a.magnitude > b.magnitude; });
        std::vector<Point> selected;
        for (const Point& p : points)
        {
            const bool tooClose = std::any_of(selected.begin(), selected.end(), [&](const Point& s) {
                return (p.lag > s.lag ? p.lag - s.lag : s.lag - p.lag) < minDistance;
            });
            if (tooClose)
                continue;
            selected.push_back(p);
            if (selected.size() >= candidatesCount)
                break;
        }
        return selected;
    };

    // First pass: whole range with the coarse step
    size_t step = std::min(coarseStep, samples.size());
    std::vector<Point> points;
    points.reserve(samples.size() / step + 1);
    for (size_t lag = 0; lag < samples.size(); lag += step)
        points.push_back(evaluate(lag));

    std::vector<Point> candidates = selectCandidates(points, step * 2);

    // Refinement passes: search around each candidate with a smaller step
    while (step > 1)
    {
        const size_t previousStep = step;
        step = std::max<size_t>(step / stepDivider, 1);

        points.clear();
        for (const Point& c : candidates)
        {
            points.push_back(c);
            const size_t from = c.lag > previousStep ? c.lag - previousStep : 0;
            const size_t to = std::min(c.lag + previousStep, samples.size() - 1);
            for (size_t lag = from; lag <= to; lag += step)
            {
                if (lag == c.lag)
                    continue;
                // neighbourhoods of different candidates can overlap, skip already evaluated lags
                const bool known = std::any_of(points.begin(), points.end(), [lag](const Point& p) { return p.lag == lag; });
                if (!known)
                    points.push_back(evaluate(lag));
            }
        }
        candidates = selectCandidates(points, step * 2);
    }

    best.lag = candidates.front().lag;
    best.magnitude = candidates.front().magnitude;
    return best;
}

} // namespace lime
