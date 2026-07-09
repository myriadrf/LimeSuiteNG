#ifndef TRXLooper_H
#define TRXLooper_H

#include <chrono>
#include <vector>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include "limesuiteng/SDRDevice.h"
#include "limesuiteng/StreamConfig.h"
#include "limesuiteng/complex.h"
#include "limesuiteng/RFStream.h"
#include "PacketsFIFO.h"
#include "StreamPacket.h"
#include "MTStack.h"

namespace lime {

class FPGA;
class IDMA;
class LMS7002M;

/** @brief Class responsible for receiving and transmitting continuous sample data */
class TRXLooper : public RFStream
{
  public:
    TRXLooper(std::shared_ptr<IDMA> rx, std::shared_ptr<IDMA> tx, FPGA* f, LMS7002M* chip, uint8_t moduleIndex);
    virtual ~TRXLooper();

    uint64_t GetHardwareTimestamp() const override;
    OpStatus SetHardwareTimestamp(const uint64_t now);
    OpStatus Setup(const lime::StreamConfig& cfg) override;
    const StreamConfig& GetConfig() const override;
    OpStatus Start() override;
    OpStatus StageStart() override;
    void Stop() override;
    void Teardown() override;

    /// @brief Gets whether the stream is currently running or not.
    /// @return The current status of the stream (true if running).
    constexpr inline bool IsStreamRunning() const { return mStreamEnabled; }
    uint32_t StreamRx(
        lime::complex32f_t* const* samples, uint32_t count, StreamMeta* meta, std::chrono::microseconds timeout) override;
    uint32_t StreamRx(
        lime::complex16_t* const* samples, uint32_t count, StreamMeta* meta, std::chrono::microseconds timeout) override;
    uint32_t StreamRx(
        lime::complex12_t* const* samples, uint32_t count, StreamMeta* meta, std::chrono::microseconds timeout) override;
    uint32_t StreamTx(const lime::complex32f_t* const* samples,
        uint32_t count,
        const StreamMeta* meta,
        std::chrono::microseconds timeout) override;
    uint32_t StreamTx(const lime::complex16_t* const* samples,
        uint32_t count,
        const StreamMeta* meta,
        std::chrono::microseconds timeout) override;
    uint32_t StreamTx(const lime::complex12_t* const* samples,
        uint32_t count,
        const StreamMeta* meta,
        std::chrono::microseconds timeout) override;

    uint32_t Receive(
        lime::complex32f_t* const* samples, uint32_t count, StreamRxMeta* meta, std::chrono::microseconds timeout) override;
    uint32_t Receive(
        lime::complex16_t* const* samples, uint32_t count, StreamRxMeta* meta, std::chrono::microseconds timeout) override;
    uint32_t Receive(
        lime::complex12_t* const* samples, uint32_t count, StreamRxMeta* meta, std::chrono::microseconds timeout) override;

    uint32_t Transmit(const lime::complex32f_t* const* samples,
        uint32_t count,
        const StreamTxMeta* meta,
        std::chrono::microseconds timeout) override;
    uint32_t Transmit(const lime::complex16_t* const* samples,
        uint32_t count,
        const StreamTxMeta* meta,
        std::chrono::microseconds timeout) override;
    uint32_t Transmit(const lime::complex12_t* const* samples,
        uint32_t count,
        const StreamTxMeta* meta,
        std::chrono::microseconds timeout) override;

    /// @brief Sets the callback to use for message logging.
    /// @param callback The new callback to use.
    void SetMessageLogCallback(SDRDevice::LogCallbackType callback) { mCallback_logMessage = callback; }
    void StreamStatus(StreamStats* rx, StreamStats* tx) override;

    static OpStatus UploadTxWaveform(FPGA* fpga,
        std::shared_ptr<IDMA> port,
        const StreamConfig& config,
        uint8_t moduleIndex,
        const void** samples,
        uint32_t count);

    /** @brief The transfer arguments. */
    struct TransferArgs {
        std::shared_ptr<IDMA> dma; ///< The DMA interface to use.
        std::vector<uint8_t*> buffers; ///< The memory buffers to use.
        uint32_t bufferSize; ///< The size of a single buffer.
        uint32_t samplesInPacket; ///< The amount of samples in a single packet.
        uint32_t packetSize; ///< The size of a single packet.
        uint32_t packetsToBatch; ///< The amount of packets to batch in a single data transfer operation.
    };

  private:
    OpStatus RxSetup();
    void RxWorkLoop();
    void ReceivePacketsLoop();
    void RxTeardown();

    OpStatus TxSetup();
    void TxWorkLoop();
    void TransmitPacketsLoop();
    void TxTeardown();

    uint64_t mTimestampOffset;
    lime::StreamConfig mConfig;

    TransferArgs mRxArgs;
    TransferArgs mTxArgs;

    FPGA* fpga;
    LMS7002M* lms;
    uint8_t chipId;

    SDRDevice::LogCallbackType mCallback_logMessage;
    std::condition_variable streamActive;
    std::mutex streamMutex;
    bool mStreamEnabled;
    bool omitRxPackets;

    struct Stream {
        enum class ReadyStage : uint8_t { Disabled = 0, WorkerReady = 1, Active = 2 };

        std::unique_ptr<MTStack<StreamPacket*>> packetsPool;
        std::unique_ptr<PacketsFIFO<StreamPacket*>> fifo;
        StreamPacket* stagingPacket;
        StreamStats stats;
        std::thread thread;
        std::atomic<uint64_t> lastTimestamp;
        std::atomic<bool> terminate;
        std::atomic<bool> terminateWorker;
        std::atomic<ReadyStage> stage;
        std::mutex mutex;
        std::condition_variable cv;
        // how many packets to batch in data transaction
        // lower count will give better latency, but can cause problems with really high data rates
        uint32_t samplesInPkt;
        uint32_t packetsToBatch;

        Stream()
            : packetsPool(nullptr)
            , fifo(nullptr)
            , stagingPacket(nullptr)
            , terminateWorker(false)
            , stage(ReadyStage::Disabled)
        {
        }

        ~Stream() { DeleteMemoryPool(); }

        void DeleteMemoryPool()
        {
            do
            {
                if (stagingPacket)
                    delete stagingPacket;
                stagingPacket = nullptr;
                if (packetsPool && !packetsPool->pop(&stagingPacket))
                    break;
            } while (stagingPacket);
        }
    };

    Stream mRx;
    Stream mTx;

    std::mutex startTimeMutex;
    std::condition_variable startTimeIsSet;
    int64_t startUnixTime;
    bool startUnixTimeSet;
    int ticksPerSample = 1;

    template<class T>
    uint32_t StreamMetaToStreamTxMeta(
        const T* const* samples, uint32_t count, const StreamMeta* meta, std::chrono::microseconds timeout);
    template<class T>
    uint32_t StreamRxTemplate(T* const* dest, uint32_t count, StreamRxMeta* meta, std::chrono::microseconds timeout);
    template<class T>
    uint32_t StreamTxTemplate(const T* const* samples, uint32_t count, const StreamTxMeta* meta, std::chrono::microseconds timeout);
};

} // namespace lime

#endif /* TRXLooper_H */
