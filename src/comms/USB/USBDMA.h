#ifndef LIME_USBDMA_H
#define LIME_USBDMA_H

#include "comms/IDMA.h"

#include <cstdint>
#include <memory>
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
    uint8_t* const GetMemoryAddress(TRXDir direction) const override;
    DMAState GetState(TRXDir direction) override;
    int SetState(TRXDir direction, DMAState state) override;

    bool Wait(TRXDir direction) override;

    void CacheFlush(TRXDir samplesDirection, DataTransferDirection dataDirection, uint16_t index) override;

  private:
    std::shared_ptr<USBGeneric> port;

    struct DirectionState {
        DirectionState(uint8_t endpoint, uint8_t* const buffer);
        ~DirectionState();

        uint8_t endpoint;
        uint8_t* const buffer;
        DMAState state;
        std::vector<int> contextHandles;
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
    uint8_t* GetIndexAddress(TRXDir direction, uint16_t index);
    uint8_t GetEndpointAddress(TRXDir direction);

    int SetStateReceive(DMAState state);
    int SetStateTransmit(DMAState state);
};

} // namespace lime

#endif // LIME_USBDMA_H
