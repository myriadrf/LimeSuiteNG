#ifndef LIME_IDMA_H
#define LIME_IDMA_H

#include <cstdint>
#include "limesuiteng/types.h"

namespace lime {

enum class DataTransferDirection : bool { DeviceToHost, HostToDevice };

class IDMA
{
  public:
    struct DMAState {
        uint32_t hardwareIndex;
        uint32_t softwareIndex;
        uint32_t bufferSize;
        bool isEnabled;
        bool generateIRQ; ///< Generate an Interrupt Request (IRQ)
    };

    virtual ~IDMA() {}

    virtual void RxEnable(uint32_t bufferSize, uint8_t irqPeriod) = 0;
    virtual void TxEnable() = 0;
    virtual void Disable(TRXDir direction) = 0;

    virtual bool IsOpen() const = 0;

    virtual int GetBufferSize() const = 0;
    virtual int GetBufferCount() const = 0;
    virtual std::byte* const GetMemoryAddress(TRXDir direction) const = 0;
    virtual DMAState GetState(TRXDir direction) = 0;
    virtual int SetState(TRXDir direction, DMAState state) = 0;

    virtual bool Wait(TRXDir direction) = 0;

    virtual void CacheFlush(TRXDir samplesDirection, DataTransferDirection dataDirection, uint16_t index) = 0;
};

} // namespace lime

#endif // LIME_IDMA_H
