#ifndef LIME_Timespec_H
#define LIME_Timespec_H

/**
 * @file limesuiteng/Timespec.h
 * @author Lime Microsystems
 * @brief Defines Timespec storage class for operating with SDR device stream time stamps
 */

#include <chrono>
#include <math.h>
#include <time.h>

#include "limesuiteng/config.h"

namespace lime {

/// @brief Timestamp storage class
class LIME_API Timespec
{
  public:
    Timespec(); ///< Create a timestamp from real-valued seconds.
    //Timespec(double realSeconds);

    /**
     * @brief Create a timestamp from whole and fractional seconds.
     * @param int_seconds The whole part of timestamp seconds.
     * @param frac_seconds The fractional part of timestamp seconds. This parameter also accepts system ticks as argumment. System ticks will be converted to seconds.
     */
    Timespec(int64_t int_seconds, double frac_seconds);

    /**
     * @brief Create a timestamp from whole and fractional seconds, where fractional seconds are represented by system tick count.
     * @param seconds The whole part of timestamp seconds.
     * @param ticks Fractional part of timestamp seconds in system ticks.
     * @param ticks_per_second Number of ticks that compose a second.
     */
    Timespec(int64_t seconds, int64_t ticks, double ticks_per_second);

    /**
     * @brief Create a timestamp from system ticks.
     * @param ticks System ticks.
     */
    explicit Timespec(int64_t ticks);

    void AddTicks(
        int64_t ticks); ///< Adds ticks to the timestamp if `hasTickRate` is set to true, else adds seconds to seconds counter.
    uint64_t
    GetTicks() const; ///< Returns number of ticks in timestamp if `hasTickRate` is set to true, else returns seconds counter value.

    int64_t GetSeconds() const; ///< Returns only the whole part of timestamp seconds.
    double GetFracSeconds() const; ///< Returns only the fractional part of timestamp seconds.

    double GetRealSeconds() const; ///< Returns full timestamp seconds count with a fractional part.
    void SetTickRate(double ticksPerSecond); ///< Sets the new number of ticks per second.
    double GetTickRate() const; ///< Returns the number of ticks per second.

    friend bool LIME_API operator==(const Timespec& lhs, const Timespec& rhs);
    friend bool LIME_API operator!=(const Timespec& lhs, const Timespec& rhs);
    friend bool LIME_API operator<(const Timespec& lhs, const Timespec& rhs);
    friend bool LIME_API operator>(const Timespec& lhs, const Timespec& rhs);
    friend Timespec LIME_API operator+(Timespec lhs, const Timespec& rhs);
    friend Timespec LIME_API operator-(Timespec lhs, const Timespec& rhs);

  private:
    /// @brief Adjusts seconds and fractional seconds values based on fraction subtraction and addition rules.
    /// @param ts Timestamp object to adjust.
    static void normalize(Timespec& ts);

    int64_t seconds; ///< Seconds counter.
    double fracSeconds; ///< Fractional seconds counter.
    double ticksPerSecond; ///< Number of ticks that compose a second.
    bool hasTickRate; ///< Tick rate set up status.
};

/// @brief Compares two timestamps to check if the seconds and fractional seconds counts are equal.
bool LIME_API operator==(const Timespec& lhs, const Timespec& rhs);

/// @brief Compares two timestamps to check if the seconds and fractional seconds counts are not equal.
bool LIME_API operator!=(const Timespec& lhs, const Timespec& rhs);

/// @brief Compares two timestamps to check if rhs timestamp has higher count of seconds.
bool LIME_API operator<(const Timespec& lhs, const Timespec& rhs);

/// @brief Compares two timestamps to check if lhs timestamp has higher count of seconds.
bool LIME_API operator>(const Timespec& lhs, const Timespec& rhs);

/// @brief Adds second and fractional second counter values of two timestamps.
Timespec LIME_API operator+(Timespec lhs, const Timespec& rhs);

/// @brief Subtracts second and fractional second counter values of two timestamps.
Timespec LIME_API operator-(Timespec lhs, const Timespec& rhs);

/// @brief Returns timestamp with absolute second and fractional second values.
Timespec LIME_API abs(const Timespec& ts);

} // namespace lime

#endif
