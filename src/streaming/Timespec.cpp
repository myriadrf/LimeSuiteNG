#include "limesuiteng/Timespec.h"

#include <math.h>

namespace lime {

void Timespec::normalize(Timespec& ts)
{
    ts.seconds += int64_t(ts.fracSeconds);
    ts.fracSeconds = fmod(ts.fracSeconds, 1.0);
    if (ts.seconds < 0)
    {
        if (ts.fracSeconds > 0)
        {
            ++ts.seconds;
            ts.fracSeconds -= 1.0;
        }
    }
    else if (ts.seconds > 0)
    {
        if (ts.fracSeconds < 0)
        {
            --ts.seconds;
            ts.fracSeconds += 1.0;
        }
    }
}

Timespec::Timespec()
    : seconds(0)
    , fracSeconds(0)
    , ticksPerSecond(1000000000.0)
    , hasTickRate(false)
{
}

// Timespec::Timespec(double realSeconds)
//     : seconds(realSeconds)
//     , fracSeconds(realSeconds - seconds)
//     , ticksPerSecond(1000000000.0)
//     , hasTickRate(false)
// {
// }

Timespec::Timespec(int64_t int_seconds, double frac_seconds)
    : seconds(int_seconds)
    , fracSeconds(frac_seconds)
    , ticksPerSecond(1000000000.0)
    , hasTickRate(false)
{
    if (fracSeconds > 1e9)
    {
        seconds += fracSeconds / 1e9;
        fracSeconds = fmod(fracSeconds, 1.0);
    }
}

Timespec::Timespec(int64_t seconds, uint64_t ticks, double ticks_per_second)
    : seconds(seconds)
    , fracSeconds(double(ticks) / ticks_per_second)
    , ticksPerSecond(ticks_per_second)
    , hasTickRate(true)
{
}

Timespec::Timespec(int64_t ticks)
    : seconds(0)
    , fracSeconds(0)
    , ticksPerSecond(0)
    , hasTickRate(false)
{
}

void Timespec::AddTicks(uint64_t tick_increment)
{
    if (!hasTickRate)
    {
        seconds += tick_increment;
        return;
    }

    double secToAdd = tick_increment / ticksPerSecond;
    secToAdd += fracSeconds;

    fracSeconds = fmod(secToAdd, 1.0);
    seconds += int(secToAdd);
}

int64_t Timespec::GetSeconds() const
{
    return seconds;
}

double Timespec::GetFracSeconds() const
{
    return fracSeconds;
}

uint64_t Timespec::GetTicks() const
{
    if (!hasTickRate)
        return seconds;

    if (seconds < 0)
        return 0;
    return seconds * ticksPerSecond + fracSeconds * ticksPerSecond;
}

double Timespec::GetRealSeconds() const
{
    double secs = seconds;
    secs += fracSeconds;
    return secs;
}

void Timespec::SetTickRate(double tps)
{
    ticksPerSecond = tps;
    hasTickRate = true;
}

bool operator==(const Timespec& lhs, const Timespec& rhs)
{
    bool fracSecondsMatch = fabs(lhs.fracSeconds - rhs.fracSeconds) < 1.0e-9;
    return (lhs.seconds == rhs.seconds) && fracSecondsMatch;
}

bool operator!=(const Timespec& lhs, const Timespec& rhs)
{
    return !(lhs == rhs);
}

bool operator<(const Timespec& lhs, const Timespec& rhs)
{
    if (lhs.seconds < rhs.seconds)
        return true;
    else if (lhs.seconds == rhs.seconds)
        return lhs.fracSeconds < rhs.fracSeconds;
    else
        return false;
}

bool operator>(const Timespec& lhs, const Timespec& rhs)
{
    if (lhs.seconds > rhs.seconds)
        return true;
    else if (lhs.seconds == rhs.seconds)
        return lhs.fracSeconds > rhs.fracSeconds;
    else
        return false;
}

Timespec operator+(Timespec lhs, const Timespec& rhs)
{
    lhs.seconds += rhs.seconds;
    lhs.fracSeconds += rhs.fracSeconds;
    Timespec::normalize(lhs);
    return lhs;
}

Timespec operator-(Timespec lhs, const Timespec& rhs)
{
    lhs.seconds -= rhs.seconds;
    lhs.fracSeconds -= rhs.fracSeconds;
    Timespec::normalize(lhs);
    return lhs;
}

Timespec LIME_API abs(const Timespec& ts)
{
    Timespec a(::abs(ts.GetSeconds()), fabs(ts.GetFracSeconds()));
    return a;
}

} // namespace lime
