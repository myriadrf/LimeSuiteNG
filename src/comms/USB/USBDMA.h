#ifndef LIME_USBDMA_H
#define LIME_USBDMA_H

#include "comms/IDMA.h"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace lime {

class USBGeneric;

class USBDMA : public IDMA
{
  public:
    USBDMA(std::shared_ptr<USBGeneric> port, uint8_t rxEndpoint, uint8_t txEndpoint);

    void RxEnable(uint32_t bufferSize, uint8_t irqPeriod) override;
    void TxEnable() override;
    void Disable(TRXDir direction) override;

    bool IsOpen() const override;

    int GetBufferSize() const override;
    int GetBufferCount() const override;
    std::byte* GetMemoryAddress(TRXDir direction) const override;
    DMAState GetState(TRXDir direction) override;
    int SetState(TRXDir direction, DMAState state) override;

    bool Wait(TRXDir direction) override;

    void CacheFlush(TRXDir samplesDirection, DataTransferDirection dataDirection, uint16_t index) override;

  private:
    std::shared_ptr<USBGeneric> port;

    struct DirectionState {
        DirectionState(uint8_t endpoint, std::byte* buffer);
        ~DirectionState();

        uint8_t endpoint;
        std::byte* buffer;
        DMAState state;
        std::vector<int> contextHandles;
        std::atomic<bool> isRunning;

        std::thread sendThread;
        std::mutex mutex;
        std::condition_variable cv;
    };

    DirectionState rx;
    DirectionState tx;

    DirectionState& GetDirectionState(TRXDir direction);
    const DirectionState& GetDirectionState(TRXDir direction) const;

    int GetContextHandle(TRXDir direction);
    int GetContextHandleFromIndex(TRXDir direction, uint16_t index);
    void SetContextHandle(TRXDir direction, int handle);
    std::size_t GetTransferArrayIndexFromState(TRXDir direction);
    std::size_t GetTransferArrayIndex(uint16_t index);
    std::byte* GetIndexAddress(TRXDir direction, uint16_t index);
    uint8_t GetEndpointAddress(TRXDir direction);

    std::mutex& GetStateMutex(TRXDir direction);
    std::condition_variable& GetStateCV(TRXDir direction);

    void RxStartTransferThread();
    void TxStartTransferThread();
};

} // namespace lime

#endif // LIME_USBDMA_H
