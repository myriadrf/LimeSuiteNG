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
    DeltaVariable(T init)
        : mValue(init)
        , mLastValue(0){};

    /**
      @brief Sets the value of the delta variable.
      @param val The value to set the delta variable to.
     */
    void set(T val) { mValue = val; }

    /**
      @brief Adds the given value to the delta variable
      @param val The value to add to the delta variable.
     */
    void add(T val) { mValue += val; }

    /**
      @brief Gets the difference between the value at last reset and current value.
      @return The delta between the last time the delta variable was reset and the current value.
     */
    T delta() const { return mValue - mLastValue; }

    /**
      @brief Gets the current value of the delta variable.
      @return The current stored value of the delta variable.
     */
    T value() const { return mValue; }

    /** @brief Resets the delta variable to start measuring from its current value. */
    void checkpoint() { mLastValue = mValue; }

  private:
    T mValue;
    T mLastValue;
};

} // namespace lime
