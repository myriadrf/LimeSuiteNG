#pragma once

#include <vector>
#include <mutex>

namespace lime {

// @brief Stack container for multithreaded use
template<class T> class MTStack
{
  public:
    MTStack(size_t count) { mStack.reserve(count); }

    bool push(const T& element)
    {
        std::unique_lock lk{ mMutex };
        if (mStack.size() >= mStack.capacity())
            return false;

        mStack.emplace_back(element);
        return true;
    }

    bool pop(T* element)
    {
        std::unique_lock lk{ mMutex };
        if (mStack.empty())
            return false;

        *element = mStack.back();
        mStack.pop_back();
        return true;
    }
    size_t capacity() const { return mStack.capacity(); }

  private:
    std::vector<T> mStack;
    std::mutex mMutex;
};

} // namespace lime