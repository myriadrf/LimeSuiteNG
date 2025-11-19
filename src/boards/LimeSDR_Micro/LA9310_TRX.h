#ifndef LIME_LA9310_TRX_H
#define LIME_LA9310_TRX_H

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
#include "streaming/PacketsFIFO.h"
#include "streaming/StreamPacket.h"

#include "TransmitControl.h"
#include "chips/LA9310/LA9310.h"

namespace lime {

/** @brief Class responsible for receiving and transmitting continuous sample data */
class LA9310_TRX : public RFStream
{
  public:
    LA9310_TRX(std::shared_ptr<LA9310> la9310);
    virtual ~LA9310_TRX();

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

    uint32_t Receive(lime::complex32f_t* const* samples, uint32_t count, StreamRxMeta* meta) override;
    uint32_t Receive(lime::complex16_t* const* samples, uint32_t count, StreamRxMeta* meta) override;
    uint32_t Receive(lime::complex12_t* const* samples, uint32_t count, StreamRxMeta* meta) override;

    uint32_t Transmit(const lime::complex32f_t* const* samples, uint32_t count, const StreamTxMeta* meta) override;
    uint32_t Transmit(const lime::complex16_t* const* samples, uint32_t count, const StreamTxMeta* meta) override;
    uint32_t Transmit(const lime::complex12_t* const* samples, uint32_t count, const StreamTxMeta* meta) override;

    /// @brief Sets the callback to use for message logging.
    /// @param callback The new callback to use.
    void SetMessageLogCallback(SDRDevice::LogCallbackType callback) { mCallback_logMessage = callback; }
    void StreamStatus(StreamStats* rx, StreamStats* tx) override;

    /** @brief The transfer arguments. */
    struct TransferArgs {
        std::vector<uint8_t*> buffers; ///< The memory buffers to use.
        int32_t bufferSize; ///< The size of a single buffer.
        int16_t packetSize; ///< The size of a single packet.
        uint8_t packetsToBatch; ///< The amount of packets to batch in a single data transfer operation.
        int32_t samplesInPacket; ///< The amount of samples in a single packet.
    };

  private:
    std::shared_ptr<LA9310> la9310;

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

    SDRDevice::LogCallbackType mCallback_logMessage;
    std::condition_variable streamActive;
    std::mutex streamMutex;
    bool mStreamEnabled;

    struct Stream {
        enum class ReadyStage : uint8_t { Disabled = 0, WorkerReady = 1, Active = 2 };

        std::unique_ptr<PacketsFIFO<StreamPacket*>> packetsPool;
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
        uint16_t samplesInPkt;
        uint8_t packetsToBatch;

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
                if (packetsPool && !packetsPool->pop(&stagingPacket, false, std::chrono::microseconds(0)))
                    break;
            } while (stagingPacket);
        }
    };

    std::vector<uint32_t> rxbuffer;
    Stream mRx;
    Stream mTx;

    int map_physical_regions();
    int get_modem_info(int modem_id);

    template<class T>
    uint32_t StreamMetaToStreamTxMeta(
        const T* const* samples, uint32_t count, const StreamMeta* meta, std::chrono::microseconds timeout);
    template<class T>
    uint32_t StreamRxTemplate(T* const* dest, uint32_t count, StreamRxMeta* meta, std::chrono::microseconds timeout);
    template<class T>
    uint32_t StreamTxTemplate(const T* const* samples, uint32_t count, const StreamTxMeta* meta, std::chrono::microseconds timeout);

    TransmitControl tx_tdd_switcher;
};

} // namespace lime

#endif /* LIME_LA9310_TRX_H */
