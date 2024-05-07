#ifndef LIME_USBDMA_H
#define LIME_USBDMA_H

#include "comms/IDMA.h"
#include "USBGeneric.h"

#include <array>
#include <cstdint>
#include <memory>

namespace lime {

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
        std::array<int, USBGeneric::GetBufferCount()> contextHandles;
    };

    DirectionState rx;
    DirectionState tx;

    DirectionState& GetDirectionState(TRXDir direction);
    const DirectionState& GetDirectionState(TRXDir direction) const;

    void IncreaseHardwareIndex(TRXDir direction);
    void HandleIncreasedSoftwareIndex(TRXDir direction, uint32_t oldIndex);
    int GetContextHandle(TRXDir direction);
    void SetContextHandle(TRXDir direction, int handle);
    std::size_t GetTransferArrayIndexFromState(TRXDir direction);
    std::byte* GetIndexAddress(TRXDir direction, uint16_t index);
    uint8_t GetEndpointAddress(TRXDir direction);
    uint32_t GetStateSoftwareIndex(TRXDir direction);
};

} // namespace lime

#endif // LIME_USBDMA_H
