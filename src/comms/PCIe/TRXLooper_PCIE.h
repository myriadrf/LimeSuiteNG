#ifndef TRXLooper_PCIE_H
#define TRXLooper_PCIE_H

#include "comms/IDMA.h"
#include "limesuiteng/SDRDevice.h"
#include "MemoryPool.h"
#include "TRXLooper.h"

#include <memory>
#include <vector>

namespace lime {

/** @brief Class responsible for receiving and transmitting continuous sample data from a PCIe device */
class TRXLooper_PCIE : public TRXLooper
{
  public:
    TRXLooper_PCIE(std::shared_ptr<IDMA> comms, FPGA* f, LMS7002M* chip, uint8_t moduleIndex);
    virtual ~TRXLooper_PCIE();
    virtual OpStatus Setup(const StreamConfig& config) override;
    virtual void Start() override;

    static OpStatus UploadTxWaveform(FPGA* fpga,
        std::shared_ptr<IDMA> port,
        const StreamConfig& config,
        uint8_t moduleIndex,
        const void** samples,
        uint32_t count);

    /** @brief The transfer arguments for the PCIe transfer. */
    struct TransferArgs {
        std::shared_ptr<IDMA> port;
        std::vector<std::byte*> buffers;
        int32_t bufferSize;
        int16_t packetSize;
        uint8_t packetsToBatch;
        int32_t samplesInPacket;
        int64_t cnt;
        int64_t sw;
        int64_t hw;
    };

  protected:
    virtual int RxSetup() override;
    virtual void ReceivePacketsLoop() override;
    virtual void RxTeardown() override;

    virtual int TxSetup() override;
    virtual void TransmitPacketsLoop() override;
    virtual void TxTeardown() override;

    TransferArgs mRxArgs;
    TransferArgs mTxArgs;
};

} // namespace lime

#endif // TRXLooper_PCIE_H
