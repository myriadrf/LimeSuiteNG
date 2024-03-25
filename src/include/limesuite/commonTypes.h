#pragma once

#if __cplusplus < 201402L
    #define CPP14constexpr
#else
    #define CPP14constexpr constexpr
#endif

namespace lime {

/// @brief The direction of the transmission
enum class TRXDir : bool { Rx, Tx };

const char* ToCString(TRXDir dir);

/// @brief Structure describing the range possible.
struct Range {
    /// @brief Constructs the range structure,
    /// @param min The minimum value of the range (default 0.0)
    /// @param max The maximum value of the range (default 0.0)
    /// @param step The step of the range (default 0.0 - no step)
    constexpr Range(double min = 0.0, double max = 0.0, double step = 0.0)
        : min(min)
        , max(max)
        , step(step){};
    double min; ///< The minimum value of the range
    double max; ///< The maximum value of the range
    double step; ///< The step of the range (or 0.0 for any step)
};

/**
  @brief Class for counting the difference since last time the value was set.
  @tparam T The type of the value to hold.
 */
template<class T> class DeltaVariable
{
  public:
    /**
    @brief Construct a new DeltaVariable object.
    @param init The initial value of the variable.
   */
    constexpr DeltaVariable(T init)
        : mValue(init)
        , mLastValue(0){};

    /**
      @brief Sets the value of the delta variable.
      @param val The value to set the delta variable to.
     */
    CPP14constexpr void set(T val) { mValue = val; }

    /**
      @brief Adds the given value to the delta variable
      @param val The value to add to the delta variable.
     */
    CPP14constexpr void add(T val) { mValue += val; }

    /**
      @brief Gets the difference between the value at last reset and current value.
      @return The delta between the last time the delta variable was reset and the current value.
     */
    constexpr T delta() const { return mValue - mLastValue; }

    /**
      @brief Gets the current value of the delta variable.
      @return The current stored value of the delta variable.
     */
    constexpr T value() const { return mValue; }

    /** @brief Resets the delta variable to start measuring from its current value. */
    CPP14constexpr void checkpoint() { mLastValue = mValue; }

  private:
    T mValue;
    T mLastValue;
};

} // namespace lime
