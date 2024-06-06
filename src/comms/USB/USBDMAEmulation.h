#ifndef LIME_USBDMAEMULATION_H
#define LIME_USBDMAEMULATION_H

#include "comms/IDMA.h"

#include <cstdint>
#include <memory>
#include <vector>
#include <queue>

namespace lime {

class IUSB;

class USBDMAEmulation : public IDMA
{
  public:
    USBDMAEmulation(std::shared_ptr<IUSB> port, uint8_t endpoint, DataTransferDirection dir);
    virtual ~USBDMAEmulation();

    OpStatus Enable(bool enabled) override;
    OpStatus EnableContinuous(bool enabled, uint32_t maxTransferSize, uint8_t irqPeriod) override;

    State GetCounters() override;
    OpStatus SubmitRequest(uint64_t index, uint32_t bytesCount, DataTransferDirection dir, bool irq) override;

    OpStatus Wait() override;
    void BufferOwnership(uint16_t index, DataTransferDirection dir) override;

    std::vector<IDMA::Buffer> GetBuffers() const override;

  private:
    struct AsyncXfer {
        void* xfer;
        std::vector<uint8_t> buffer;
        uint32_t requestedSize;
        uint32_t size;
        uint8_t id;
    };

    std::queue<AsyncXfer*> transfers;
    std::queue<AsyncXfer*> pendingXfers;
    std::vector<Buffer> mappings;

    void AbortAllTransfers();
    void UpdateProducerStates();

    std::shared_ptr<IUSB> port;
    State counters;
    uint16_t lastRequestIndex;
    uint8_t endpoint;
    DataTransferDirection dir;
    bool continuous;
};

} // namespace lime

#endif // LIME_USBDMAEMULATION_H
