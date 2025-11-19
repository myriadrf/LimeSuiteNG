#ifndef LIME_Timespec_H
#define LIME_Timespec_H

#include <chrono>
#include <math.h>
#include <time.h>

#include "limesuiteng/config.h"

namespace lime {

class LIME_API Timespec
{
  public:
    Timespec();
    //Timespec(double realSeconds);
    Timespec(int64_t int_seconds, double frac_seconds);
    Timespec(int64_t seconds, int64_t ticks, double ticks_per_second);
    explicit Timespec(int64_t ticks);

    void AddTicks(int64_t ticks);
    uint64_t GetTicks() const;

    int64_t GetSeconds() const;
    double GetFracSeconds() const;

    double GetRealSeconds() const;
    void SetTickRate(double ticksPerSecond);
    double GetTickRate() const;

    friend bool LIME_API operator==(const Timespec& lhs, const Timespec& rhs);
    friend bool LIME_API operator!=(const Timespec& lhs, const Timespec& rhs);
    friend bool LIME_API operator<(const Timespec& lhs, const Timespec& rhs);
    friend bool LIME_API operator>(const Timespec& lhs, const Timespec& rhs);
    friend Timespec LIME_API operator+(Timespec lhs, const Timespec& rhs);
    friend Timespec LIME_API operator-(Timespec lhs, const Timespec& rhs);

  private:
    static void normalize(Timespec& ts);
    int64_t seconds;
    double fracSeconds;
    double ticksPerSecond;
    bool hasTickRate;
};

bool LIME_API operator==(const Timespec& lhs, const Timespec& rhs);
bool LIME_API operator!=(const Timespec& lhs, const Timespec& rhs);
bool LIME_API operator<(const Timespec& lhs, const Timespec& rhs);
bool LIME_API operator>(const Timespec& lhs, const Timespec& rhs);

Timespec LIME_API operator+(Timespec lhs, const Timespec& rhs);
Timespec LIME_API operator-(Timespec lhs, const Timespec& rhs);

Timespec LIME_API abs(const Timespec& ts);

} // namespace lime

#endif
