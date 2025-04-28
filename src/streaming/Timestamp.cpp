#include "limesuiteng/Timestamp.h"

namespace lime {

Timestamp::Timestamp()
    : seconds(0)
    , fracSeconds(0)
    , ticksPerSecond(1000000000.0)
    , hasTickRate(false)
{
}

Timestamp::Timestamp(double sec)
    : seconds(sec)
    , fracSeconds(sec - seconds)
    , ticksPerSecond(1000000000.0)
    , hasTickRate(false)
{
}

Timestamp::Timestamp(int64_t int_seconds, double frac_seconds)
    : seconds(int_seconds)
    , fracSeconds(frac_seconds)
    , ticksPerSecond(1000)
    , hasTickRate(false)
{
    if (fracSeconds > 1e9)
    {
        seconds += fracSeconds / 1e9;
        fracSeconds = fmod(fracSeconds, 1.0);
    }
}

Timestamp::Timestamp(int64_t seconds, uint64_t ticks, double tickRate)
    : seconds(seconds)
    , fracSeconds(double(ticks) / tickRate)
    , ticksPerSecond(tickRate)
    , hasTickRate(true)
{
}

struct timespec Timestamp::GetTimespec() const
{
    struct timespec ts;
    ts.tv_sec = int64_t(seconds);
    ts.tv_nsec = 1e9 * fracSeconds;
    return ts;
}

void Timestamp::AddTicks(uint64_t tick_increment)
{
    double secToAdd = tick_increment / ticksPerSecond;
    secToAdd += fracSeconds;

    fracSeconds = fmod(secToAdd, 1.0);
    seconds += int(secToAdd);
}

void Timestamp::AddSeconds(int64_t increment)
{
    seconds += increment;
}

int64_t Timestamp::GetSeconds() const
{
    return seconds;
}

double Timestamp::GetFracSeconds() const
{
    return fracSeconds;
}

uint64_t Timestamp::GetTicks() const
{
    return seconds * ticksPerSecond + fracSeconds * ticksPerSecond;
}

void Timestamp::Reset()
{
    seconds = 0;
    fracSeconds = 0;
    ticksPerSecond = 1000000000.0;
    hasTickRate = false;
}

double Timestamp::GetRealSeconds() const
{
    double secs = seconds;
    secs += fracSeconds;
    return secs;
}

void Timestamp::SetTickRate(double tps)
{
    ticksPerSecond = tps;
}

bool operator==(const Timestamp& lhs, const Timestamp& rhs)
{
    bool fracSecondsMatch = std::fabs(lhs.fracSeconds - rhs.fracSeconds) < 1.0e-9;
    return (lhs.seconds == rhs.seconds) && fracSecondsMatch;
}

bool operator!=(const Timestamp& lhs, const Timestamp& rhs)
{
    return !(lhs == rhs);
}

Timestamp operator+(Timestamp lhs, const Timestamp& rhs)
{
    lhs.seconds += rhs.seconds;
    lhs.fracSeconds += rhs.fracSeconds;
    if (lhs.fracSeconds >= 1e9)
    {
        ++lhs.seconds;
        lhs.fracSeconds -= 1e9;
    }
    return lhs;
}

Timestamp operator-(Timestamp lhs, const Timestamp& rhs)
{
    lhs.seconds -= rhs.seconds;
    lhs.fracSeconds -= rhs.fracSeconds;
    if (lhs.fracSeconds < 0)
    {
        --lhs.seconds;
        lhs.fracSeconds += 1e9;
    }
    return lhs;
}

} // namespace lime
