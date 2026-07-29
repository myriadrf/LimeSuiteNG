#include "AvgRmsCounter.h"

#include <cmath>

using namespace lime;

AvgRmsCounter::AvgRmsCounter()
    : counter(0)
    , avgAccumulator(0)
    , rmsAccumulator(0)
    , min(1e16)
    , max(-1e16)
{
}

void AvgRmsCounter::Add(double value)
{
    avgAccumulator += value;
    rmsAccumulator += value * value;
    ++counter;

    if (value < min)
    {
        min = value;
    }

    if (value > max)
    {
        max = value;
    }
}

void AvgRmsCounter::GetResult(double& avg, double& rms)
{
    if (counter == 0)
    {
        return;
    }

    avg = avgAccumulator / counter;
    rms = std::sqrt(rmsAccumulator / counter);

    avgAccumulator = 0;
    rmsAccumulator = 0;
    counter = 0;
}

double AvgRmsCounter::Min()
{
    auto temp = min;
    min = 1e16;
    return temp;
}

double AvgRmsCounter::Max()
{
    auto temp = max;
    // seed below every possible sample, the Tx timestamp advance this counts is
    // negative during an underrun and +1e-16 would be reported as the maximum
    max = -1e16;
    return temp;
}
