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
    Timestamp(double int_seconds, double frac_seconds);
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

  private:
    int64_t seconds;
    double fracSeconds;
    double ticksPerSecond;
    bool hasTickRate;
};

} // namespace lime

#endif
