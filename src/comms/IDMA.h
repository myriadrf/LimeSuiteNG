#ifndef LIME_IDMA_H
#define LIME_IDMA_H

#include <cassert>
#include <cstdint>
#include "limesuiteng/types.h"
#include "limesuiteng/OpStatus.h"
#include <vector>

namespace lime {

enum class DataTransferDirection : bool { DeviceToHost, HostToDevice };

class IDMA
{
  public:
    struct State {
        uint64_t transfersCompleted;
    };

    virtual ~IDMA(){};

    /**
      @brief Enables or disabled DMA memory access.
      @param enabled Enable DMA transactions.
      @return The operation success state.
     */
    virtual OpStatus Enable(bool enabled) = 0;

    /**
      @brief Enables or disabled DMA transactions in continuous mode.
      @param enabled Enable DMA transactions.
      @param maxTransferSize Size of each memory transaction.
      @param irqPeriod Number of transfers after which to trigger interrupt request
      @return The operation success state.

      In continuous mode, the device automatically keeps looping over available buffers
      and transfering data without explicit requests.
     */
    virtual OpStatus EnableContinuous(bool enabled, uint32_t maxTransferSize, uint8_t irqPeriod) = 0;

    /**
     * @brief Returns how many transactions have been completed, can overflow
     */
    virtual State GetCounters() = 0;

    /**
     * @brief Returns how many transactions have been completed, can overflow
     */
    virtual OpStatus SubmitRequest(uint64_t index, uint32_t bytesCount, DataTransferDirection dir, bool irq) = 0;

    /**
     * @brief Blocks until data is available.
     * @return OpStatus::Success if data has been transferred.
     */
    virtual OpStatus Wait() = 0;

    /**
     * @brief Transfer ownership and flush CUP cache of memory buffer.
     * @param index which DMA buffer to modify.
     * @param dir Ownership transfer direction.
     * @return The operation success state.
     */
    virtual void BufferOwnership(uint16_t index, DataTransferDirection dir) = 0;

    struct Buffer {
        uint8_t* buffer;
        size_t size;
    };

    /**
     * @brief Returns a list of DMA buffers
     */
    virtual std::vector<Buffer> GetBuffers() const = 0;
};

} // namespace lime

#endif // LIME_IDMA_H
