#pragma once

#include <string.h>
#include <algorithm>
#include <vector>
#include <mutex>

/// @brief A buffer for manipulating many elements at a time in a queue-like fashion.
/// @tparam T The type of object to hold.
template<class T> class RingBuffer
{
  public:
    /// @brief Constructs the ring buffer with the given capacity.
    /// @param count The maximum capacity of the buffer.
    RingBuffer(std::size_t count)
        : headIndex(0)
        , tailIndex(0)
        , size(0)
        , capacity(count)
    {
        buffer.resize(count);
    }

    /// @brief Removes the items from the queue and puts them into the given destination.
    /// @param dest The buffer to store the items in.
    /// @param count The maximum amount of items to retrieve.
    /// @return The amount of items actually retrieved.
    std::size_t Consume(T* dest, std::size_t count)
    {
        std::lock_guard<std::mutex> lck(mMutex);
        std::size_t consumed = 0;
        count = std::min(count, size);

        if (headIndex + count >= capacity)
        {
            std::size_t toCopy = capacity - headIndex;
            memcpy(dest, &buffer[headIndex], toCopy * sizeof(T));
            dest += toCopy;
            headIndex = 0;
            size -= toCopy;
            count -= toCopy;
            consumed += toCopy;
        }
        memcpy(dest, &buffer[headIndex], count * sizeof(T));
        headIndex += count;
        size -= count;
        consumed += count;
        return consumed;
    }

    /// @brief Adds items into the buffer.
    /// @param src The array of items to add to the buffer.
    /// @param count The amount of items to add to the buffer.
    /// @return The amount of items actually added to the buffer.
    std::size_t Produce(const T* src, std::size_t count)
    {
        std::lock_guard<std::mutex> lck(mMutex);
        count = std::min(count, capacity - size);
        std::size_t produced = 0;

        if (tailIndex + count >= capacity)
        {
            std::size_t toCopy = capacity - tailIndex;
            memcpy(&buffer[tailIndex], src, toCopy * sizeof(T));
            src += toCopy;
            count -= toCopy;
            size += toCopy;
            produced += toCopy;
            tailIndex = 0;
        }
        memcpy(&buffer[tailIndex], src, count * sizeof(T));
        tailIndex += count;
        size += count;
        produced += count;
        return produced;
    }

    /// @brief Gets the amount of items in the buffer.
    /// @return The amount of items currently in the buffer.
    std::size_t Size() const { return size; }

    /// @brief Gets the total possible capacity of the buffer.
    /// @return The total possible capacity of the buffer.
    std::size_t Capacity() const { return capacity; }

  private:
    std::mutex mMutex;
    std::vector<T> buffer;
    std::size_t headIndex;
    std::size_t tailIndex;
    std::size_t size;
    std::size_t capacity;
};
