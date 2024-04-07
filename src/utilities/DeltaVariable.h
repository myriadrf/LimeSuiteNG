#pragma once

namespace lime {

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

}; // namespace lime
