#ifndef LIME_TIMESTAMP_H
#define LIME_TIMESTAMP_H

#include <chrono>
#include <math.h>
#include <time.h>

#include "limesuiteng/config.h"

namespace lime {

class LIME_API Timestamp
{
  public:
    Timestamp();
    Timestamp(double seconds);
    Timestamp(int64_t int_seconds, double frac_seconds);
    Timestamp(int64_t seconds, uint64_t ticks, double tickRate);

    struct timespec GetTimespec() const;
    void AddTicks(uint64_t ticks);
    uint64_t GetTicks() const;

    void AddSeconds(int64_t increment);
    int64_t GetSeconds() const;
    double GetFracSeconds() const;

    void Reset();

    double GetRealSeconds() const;
    void SetTickRate(double ticksPerSecond);

    friend bool operator==(const Timestamp& lhs, const Timestamp& rhs);
    friend bool operator!=(const Timestamp& lhs, const Timestamp& rhs);
    friend Timestamp operator+(Timestamp lhs, const Timestamp& rhs);
    friend Timestamp operator-(Timestamp lhs, const Timestamp& rhs);

  private:
    int64_t seconds;
    double fracSeconds;
    double ticksPerSecond;
    bool hasTickRate;
};

bool LIME_API operator==(const Timestamp& lhs, const Timestamp& rhs);
bool LIME_API operator!=(const Timestamp& lhs, const Timestamp& rhs);
Timestamp LIME_API operator+(Timestamp lhs, const Timestamp& rhs);
Timestamp LIME_API operator-(Timestamp lhs, const Timestamp& rhs);

} // namespace lime

#endif
