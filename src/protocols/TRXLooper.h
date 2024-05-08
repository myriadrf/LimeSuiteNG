#ifndef TRXLooper_H
#define TRXLooper_H

#include <vector>
#include <atomic>
#include <thread>
#include "limesuiteng/SDRDevice.h"
#include "limesuiteng/StreamConfig.h"
#include "limesuiteng/complex.h"
#include "PacketsFIFO.h"
#include "MemoryPool.h"
#include "SamplesPacket.h"

namespace lime {

class FPGA;
class IDMA;
class LMS7002M;

/** @brief Class responsible for receiving and transmitting continuous sample data */
class TRXLooper
{
  public:
    TRXLooper(std::shared_ptr<IDMA> comms, FPGA* f, LMS7002M* chip, uint8_t moduleIndex);
    ~TRXLooper();

    uint64_t GetHardwareTimestamp() const;
    OpStatus SetHardwareTimestamp(const uint64_t now);
    OpStatus Setup(const lime::StreamConfig& cfg);
    void Start();
    void Stop();

    /// @brief Gets whether the stream is currently running or not.
    /// @return The current status of the stream (true if running).
    constexpr bool IsStreamRunning() const { return mStreamEnabled; }

    /// @brief Gets the current configuration of the stream.
    /// @return The current configuration of the stream.
    constexpr const lime::StreamConfig& GetConfig() const { return mConfig; }

    uint32_t StreamRx(lime::complex32f_t* const* samples, uint32_t count, StreamMeta* meta);
    uint32_t StreamRx(lime::complex16_t* const* samples, uint32_t count, StreamMeta* meta);
    uint32_t StreamRx(lime::complex12_t* const* samples, uint32_t count, StreamMeta* meta);
    uint32_t StreamTx(const lime::complex32f_t* const* samples, uint32_t count, const StreamMeta* meta);
    uint32_t StreamTx(const lime::complex16_t* const* samples, uint32_t count, const StreamMeta* meta);
    uint32_t StreamTx(const lime::complex12_t* const* samples, uint32_t count, const StreamMeta* meta);

    /// @brief Sets the callback to use for message logging.
    /// @param callback The new callback to use.
    void SetMessageLogCallback(SDRDevice::LogCallbackType callback) { mCallback_logMessage = callback; }

    StreamStats GetStats(TRXDir tx) const;

    /// @brief The type of a sample packet.
    typedef SamplesPacket<2> SamplesPacketType;

    static OpStatus UploadTxWaveform(FPGA* fpga,
        std::shared_ptr<IDMA> port,
        const StreamConfig& config,
        uint8_t moduleIndex,
        const void** samples,
        uint32_t count);

    /** @brief The transfer arguments. */
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
    int RxSetup();
    void ReceivePacketsLoop();
    void RxTeardown();

    int TxSetup();
    void TransmitPacketsLoop();
    void TxTeardown();

    uint64_t mTimestampOffset;
    lime::StreamConfig mConfig;

    TransferArgs mRxArgs;
    TransferArgs mTxArgs;

    FPGA* fpga;
    LMS7002M* lms;
    uint8_t chipId;

    std::chrono::time_point<std::chrono::steady_clock> streamClockStart;

    SDRDevice::LogCallbackType mCallback_logMessage;
    std::condition_variable streamActive;
    std::mutex streamMutex;
    bool mStreamEnabled;

    struct Stream {
        MemoryPool* memPool;
        StreamStats stats;
        PacketsFIFO<SamplesPacketType*>* fifo;
        SamplesPacketType* stagingPacket;
        std::thread thread;
        std::atomic<uint64_t> lastTimestamp;
        std::atomic<bool> terminate;
        // how many packets to batch in data transaction
        // lower count will give better latency, but can cause problems with really high data rates
        uint16_t samplesInPkt;
        uint8_t packetsToBatch;

        Stream()
            : memPool(nullptr)
            , fifo(nullptr)
            , stagingPacket(nullptr)
        {
        }

        ~Stream()
        {
            if (fifo != nullptr)
            {
                delete fifo;
            }

            DeleteMemoryPool();
        }

        void DeleteMemoryPool()
        {
            if (memPool == nullptr)
            {
                return;
            }

            if (stagingPacket != nullptr)
            {
                memPool->Free(stagingPacket);
                stagingPacket = nullptr;
            }

            delete memPool;
            memPool = nullptr;
        }
    };

    Stream mRx;
    Stream mTx;

  private:
    template<class T> uint32_t StreamRxTemplate(T* const* dest, uint32_t count, StreamMeta* meta);
    template<class T> uint32_t StreamTxTemplate(const T* const* samples, uint32_t count, const StreamMeta* meta);
};

} // namespace lime

#endif /* TRXLooper_H */
