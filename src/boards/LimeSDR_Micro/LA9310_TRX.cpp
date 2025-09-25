#include "LA9310_TRX.h"

#include "streaming/AvgRmsCounter.h"
#include "comms/IDMA.h"
#include "FPGA/FPGA_common.h"
#include "limesuiteng/LMS7002M.h"
#include "limesuiteng/StreamMeta.h"
#include "limesuiteng/Logger.h"
#include "chips/LMS7002M/LMS7002MCSR_Data.h"
#include "protocols/LMSBoards.h"
#include "threadHelper.h"
#include "utilities/DeltaVariable.h"
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include "streaming/BufferInterleaving.h"

#include <algorithm>
#include <cassert>
#include <ciso646>
#include <complex>
#include <queue>
#include <cinttypes>

using namespace std::literals::string_literals;

namespace lime {

using namespace std;
using namespace std::chrono;

static constexpr bool showStats{ true };

static constexpr int statsPeriod_ms{ 1000 };

static constexpr int64_t ts_to_us(int64_t fs, int64_t ts)
{
    int64_t n = (ts / fs);
    int64_t r = (ts % fs);
    return n * 1000000 + ((r * 1000000) / fs);
}

template<class T> static uint32_t indexListToMask(const std::vector<T>& indexes)
{
    uint32_t mask = 0;
    for (T bitIndex : indexes)
        mask |= 1 << bitIndex;
    return mask;
}

int LA9310_TRX::map_physical_regions()
{

    return 0;
}

/// @brief Constructs a new LA9310_TRX object.
/// @param rx The DMA communications interface to receive the data from.
/// @param tx The DMA communications interface to send the data to.
/// @param f The FPGA to use in this stream.
/// @param chip The LMS7002M chip to use in this stream.
/// @param moduleIndex The ID of the chip to use.
LA9310_TRX::LA9310_TRX(std::shared_ptr<LA9310_PCIe> port)
    : port(port)
    , vspa(port)
    , mCallback_logMessage(nullptr)
    , mStreamEnabled(false)
{
    mTimestampOffset = 0;
}

LA9310_TRX::~LA9310_TRX()
{
    Stop();
    Teardown();
}

/// @brief Gets the current timestamp of the hardware.
/// @return The current timestamp of the hardware.
uint64_t LA9310_TRX::GetHardwareTimestamp() const
{
    return mRx.lastTimestamp.load(std::memory_order_relaxed) + mTimestampOffset;
}

/// @brief Sets the hardware timestamp.
/// @param now The current timestamp to set.
/// @return The status of the operation.
OpStatus LA9310_TRX::SetHardwareTimestamp(const uint64_t now)
{
    mTimestampOffset = now - mRx.lastTimestamp.load(std::memory_order_relaxed);
    return OpStatus::Success;
}

/// @brief Sets up the stream of this looper.
/// @param cfg The configuration settings to set up the stream with.
/// @return The status of the operation.
OpStatus LA9310_TRX::Setup(const StreamConfig& cfg)
{
    if (mStreamEnabled)
        return ReportError(OpStatus::Busy, "Samples streaming already running"s);

    // if (cfg.linkFormat != DataFormat::I16)
    //     return ReportError(OpStatus::InvalidValue, "Unsupported stream link format"s);

    mTx.packetsToBatch = 6;
    mRx.packetsToBatch = 6;

    bool needTx = cfg.channels.at(TRXDir::Tx).size() > 0;
    bool needRx = cfg.channels.at(TRXDir::Rx).size() > 0;

    mConfig = cfg;
    mConfig.linkFormat = DataFormat::I16; // always force I16 link

    if (!needTx && !needRx)
        return OpStatus::Success;

    OpStatus status = OpStatus::Success;

    RxTeardown();
    if (needRx)
        status = RxSetup();

    if (status != OpStatus::Success)
        return status;

    TxTeardown();
    if (needTx)
        status = TxSetup();

    if (status != OpStatus::Success)
        return status;

    return vspa.Setup(cfg.channels.at(TRXDir::Rx).size(), cfg.channels.at(TRXDir::Tx).size());
}

const StreamConfig& LA9310_TRX::GetConfig() const
{
    return mConfig;
}

/// @brief Starts the stream of this looper.
OpStatus LA9310_TRX::Start()
{
    if (mStreamEnabled)
        return OpStatus::Success;

    mRx.terminate.store(false, std::memory_order_relaxed);
    mTx.terminate.store(false, std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> lock(streamMutex);
        mStreamEnabled = true;
        streamActive.notify_all();
    }

    OpStatus status = OpStatus::InvalidValue;
    if (mConfig.channels.at(TRXDir::Tx).size() > 0)
    {
        status = vspa.StartTx();
        if (status != OpStatus::Success)
            return status;
    }
    if (mConfig.channels.at(TRXDir::Rx).size() > 0)
    {
        status = vspa.StartRx();
        if (status != OpStatus::Success)
            return status;
    }
    return status;
}

OpStatus LA9310_TRX::StageStart()
{
    return OpStatus::NotImplemented;
}

/// @brief Stops the stream and cleans up all the memory.
void LA9310_TRX::Stop()
{
    if (!mStreamEnabled)
        return;
    lime::debug("LA9310_TRX::Stop()");
    mStreamEnabled = false;

    // wait for loop ends

    if (mRx.stage.load(std::memory_order_relaxed) == Stream::ReadyStage::Active)
    {
        mRx.terminate.store(true, std::memory_order_relaxed);
        lime::debug("LA9310_TRX: wait for Rx loop end.");
        {
            std::unique_lock lck{ mRx.mutex };
            while (mRx.stage.load(std::memory_order_relaxed) == Stream::ReadyStage::Active)
                mRx.cv.wait(lck);
        }

        if (mCallback_logMessage)
        {
            char msg[256];
            std::snprintf(msg, sizeof(msg), "Rx%i stop: packetsIn: %" PRIi64, 0, mRx.stats.packets);
            mCallback_logMessage(LogLevel::Verbose, msg);
        }
    }
    vspa.StopRx();

    // wait for loop ends
    if (mTx.stage.load(std::memory_order_relaxed) == Stream::ReadyStage::Active)
    {
        mTx.terminate.store(true, std::memory_order_relaxed);
        lime::debug("LA9310_TRX: wait for Tx loop end."s);
        {
            std::unique_lock lck{ mTx.mutex };
            while (mTx.stage.load(std::memory_order_relaxed) == Stream::ReadyStage::Active)
                mTx.cv.wait(lck);
        }
    }
    vspa.StopTx();

    if (mRx.stagingPacket != nullptr)
    {
        mRx.packetsPool->push(mRx.stagingPacket, true);
        mRx.stagingPacket = nullptr;
    }
    if (mRx.fifo)
    {
        while (mRx.fifo->pop(&mRx.stagingPacket, false))
            mRx.packetsPool->push(mRx.stagingPacket, true);
        mRx.fifo->clear();
        mRx.stagingPacket = nullptr;
    }
    if (mTx.stagingPacket != nullptr)
    {
        mTx.packetsPool->push(mTx.stagingPacket, true);
        mTx.stagingPacket = nullptr;
    }
    if (mTx.fifo)
    {
        while (mTx.fifo->pop(&mTx.stagingPacket, false))
            mTx.packetsPool->push(mTx.stagingPacket, true);
        mTx.fifo->clear();
        mTx.stagingPacket = nullptr;
    }

    mRx.lastTimestamp.store(0, std::memory_order_relaxed);
}

/// @brief Stops all the running streams and clears up the memory.
void LA9310_TRX::Teardown()
{
    RxTeardown();
    TxTeardown();
}

OpStatus LA9310_TRX::RxSetup()
{
    vspa.StopRx();
    // init tx channel
    int la9310chan = mConfig.channels.at(TRXDir::Rx).front() == 0 ? 3 : 1;
    vspa.SelectRxChannel(la9310chan);

    mRx.fifo = std::make_unique<PacketsFIFO<StreamPacket*>>(512);
    mRx.terminate.store(false, std::memory_order_relaxed);

    mRx.lastTimestamp.store(0, std::memory_order_relaxed);
    const int chCount = std::max(mConfig.channels.at(lime::TRXDir::Rx).size(), mConfig.channels.at(lime::TRXDir::Tx).size());
    const int sampleSize = 4; // sizeof IQ pair

    constexpr std::size_t headerSize{ 0 };

    uint32_t packetSize = 4096;
    mRx.samplesInPkt = (packetSize - headerSize) / (sampleSize * chCount);

    if (mConfig.extraConfig.rx.packetsInBatch != 0)
        mRx.packetsToBatch = mConfig.extraConfig.rx.packetsInBatch;

    mRx.packetsToBatch = 4;
    char msg[256];
    std::snprintf(msg,
        sizeof(msg),
        "%s Rx%i Setup: rxSamplesInPkt:%i rxPacketsInBatch:%i, DMA_ReadSize:%i, link:%s, FS:%f\n",
        "la9310",
        0,
        mRx.samplesInPkt,
        mRx.packetsToBatch,
        mRx.packetsToBatch * packetSize,
        (mConfig.linkFormat == DataFormat::I12 ? "I12" : "I16"),
        mConfig.hintSampleRate);
    if (showStats)
        printf("%s", msg);
    if (mCallback_logMessage)
        mCallback_logMessage(LogLevel::Verbose, msg);

    mRxArgs.bufferSize = 4096;
    mRxArgs.packetSize = packetSize;
    mRxArgs.packetsToBatch = mRx.packetsToBatch;
    mRxArgs.samplesInPacket = mRx.samplesInPkt;

    mRx.packetsPool = std::make_unique<PacketsFIFO<StreamPacket*>>(1024);
    const uint32_t userSampleSize = mConfig.format == DataFormat::F32 ? sizeof(lime::complex32f_t) : sizeof(lime::complex16_t);
    for (uint32_t i = 0; i < mRx.packetsPool->max_size(); ++i)
        mRx.packetsPool->push(new StreamPacket(1024 * 32, chCount, userSampleSize));

    // Don't just use REALTIME scheduling, or at least be cautious with it.
    // if the thread blocks for too long, Linux can trigger RT throttling
    // which can cause unexpected data packet losses and timing issues.
    // Also need to set policy to default here, because if host process is running
    // with REALTIME policy, these threads would inherit it and exhibit mentioned
    // issues.
    const auto schedulingPolicy = ThreadPolicy::REALTIME;
    mRx.terminate.store(false, std::memory_order_relaxed);
    mRx.terminateWorker.store(false, std::memory_order_relaxed);

    auto RxLoopFunction = std::bind(&LA9310_TRX::RxWorkLoop, this);
    mRx.thread = std::thread(RxLoopFunction);
    SetOSThreadPriority(ThreadPriority::HIGHEST, schedulingPolicy, &mRx.thread);
#ifdef __linux__
    char threadName[16]; // limited to 16 chars, including null byte.
    snprintf(threadName, sizeof(threadName), "lime:Rx%i", 0);
    pthread_setname_np(mRx.thread.native_handle(), threadName);
#endif

    // wait for Rx thread to be ready
    lime::debug("RxSetup wait for Rx worker thread."s);
    {
        std::unique_lock lck{ mRx.mutex };
        while (mRx.stage.load(std::memory_order_relaxed) < Stream::ReadyStage::WorkerReady)
            mRx.cv.wait(lck);
    }

    return OpStatus::Success;
}

void LA9310_TRX::RxWorkLoop()
{
    lime::debug("Rx worker thread ready.");
    // signal that thread is ready for work
    {
        std::unique_lock lck{ mRx.mutex };

        // signal that thread is ready for work
        mRx.stage.store(Stream::ReadyStage::WorkerReady, std::memory_order_relaxed);
        mRx.cv.notify_all();
    }

    while (!mRx.terminateWorker.load(std::memory_order_relaxed))
    {
        // thread ready for work, just wait for stream enable
        {
            std::unique_lock lk{ streamMutex };
            while (!mStreamEnabled && !mRx.terminateWorker.load(std::memory_order_relaxed))
                streamActive.wait_for(lk, std::chrono::milliseconds(100));
        }
        if (!mStreamEnabled)
            continue;

        mRx.stage.store(Stream::ReadyStage::Active, std::memory_order_relaxed);
        ReceivePacketsLoop();

        std::unique_lock lck{ mRx.mutex };
        mRx.stage.store(Stream::ReadyStage::WorkerReady, std::memory_order_relaxed);
        mRx.cv.notify_all();
    }
    mRx.stage.store(Stream::ReadyStage::Disabled, std::memory_order_relaxed);
    lime::debug("Rx worker thread shutdown.");
}

/** @brief Function dedicated for receiving data samples from board */
void LA9310_TRX::ReceivePacketsLoop()
{
    lime::debug("Rx receive loop start.");

    DataConversion conversion{};
    conversion.srcFormat = mConfig.linkFormat;
    conversion.destFormat = mConfig.format;
    conversion.channelCount = std::max(mConfig.channels.at(lime::TRXDir::Tx).size(), mConfig.channels.at(lime::TRXDir::Rx).size());

    // const int32_t readSize = mRxArgs.packetSize * mRxArgs.packetsToBatch;
    // const int32_t packetSize = mRxArgs.packetSize;

    StreamStats& stats = mRx.stats;
    auto& fifo = mRx.fifo;

    DeltaVariable<int32_t> overrun(0);
    DeltaVariable<int32_t> loss(0);

    auto t1{ std::chrono::steady_clock::now() };
    auto t2 = t1;

    int32_t Bps = 0;
    StreamPacket* outputPkt = nullptr;

    assert(mRx.stagingPacket == nullptr); // should be clean start
    assert(fifo->empty());

    std::vector<uint32_t> buffer(1024 * 1024 * 4);

    while (mRx.terminate.load(std::memory_order_relaxed) == false)
    {
        // prepare next transmit
        // std::this_thread::sleep_for(std::chrono::milliseconds(1));
        uint32_t size_received = vspa.Receive(0, buffer.data(), 65536);
        // size_received = iq_player_receive_data(RxChanID, buffer.data(), buffer.size());
        // printf("Got bytes: %li\n", size_received);
        Bps += size_received;

        // print stats
        t2 = std::chrono::steady_clock::now();
        const auto timePeriod{ std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() };
        if (timePeriod >= statsPeriod_ms)
        {
            t1 = t2;
            double dataRateBps = 1000.0 * Bps / timePeriod;
            stats.dataRate_Bps = dataRateBps;
            char msg[512];
            std::snprintf(msg,
                sizeof(msg) - 1,
                "%s Rx%i: %3.3f MB/s | TS:%" PRIu64 " pkt:%" PRIi64 " o:%i(%+i) l:%i(%+i) swFIFO:%" PRIuPTR,
                "la9310",
                0,
                stats.dataRate_Bps / 1e6,
                stats.timestamp,
                stats.packets,
                overrun.value(),
                overrun.delta(),
                loss.value(),
                loss.delta(),
                fifo->size());
            if (showStats)
                printf("%s\n", msg);
            if (mCallback_logMessage)
            {
                bool showAsWarning = overrun.delta() || loss.delta();
                LogLevel level = showAsWarning ? LogLevel::Warning : LogLevel::Debug;
                mCallback_logMessage(level, msg);
            }
            overrun.checkpoint();
            loss.checkpoint();
            Bps = 0;
        }

        if (size_received == 0)
        {
            // std::this_thread::yield();
            continue;
        }

        if (outputPkt == nullptr)
        {
            if (!mRx.packetsPool->pop(&outputPkt, false) || outputPkt == nullptr)
            {
                lime::warning("Rx%i: packets fifo full.", 0);
                continue;
            }
            outputPkt->Reset();
        }

        int samplesProduced =
            Deinterleave(outputPkt->samples.back(), reinterpret_cast<uint8_t*>(buffer.data()), size_received, conversion);
        outputPkt->samples.SetSize(outputPkt->samples.size() + samplesProduced);

        if (fifo->push(outputPkt, false))
        {
            outputPkt = nullptr;
        }
        else
        {
            ++stats.overrun;
            overrun.add(1);
            outputPkt->Reset();
        }

        // one callback for the entire batch
        // if (reportProblems && mConfig.statusCallback)
        //     mConfig.statusCallback(false, &stats, mConfig.userData);
        // std::this_thread::yield();
    }
    lime::debug("Rx receive loop end.");
}

void LA9310_TRX::RxTeardown()
{
    if (mRx.stage.load(std::memory_order_relaxed) != Stream::ReadyStage::Disabled)
    {
        lime::debug("RxTeardown wait for Rx worker shutdown.");
        mRx.terminateWorker.store(true, std::memory_order_relaxed);
        {
            std::unique_lock lck{ streamMutex };
            mRx.terminateWorker.store(true, std::memory_order_relaxed);
            streamActive.notify_all();
        }

        mRx.terminate.store(true, std::memory_order_relaxed);
        try
        {
            if (mRx.thread.joinable())
                mRx.thread.join();
        } catch (...)
        {
            lime::error("Failed to join LA9310_TRX Rx thread"s);
        }
    }

    if (mRx.stagingPacket)
    {
        delete mRx.stagingPacket;
        mRx.stagingPacket = nullptr;
    }

    if (mRx.packetsPool)
    {
        while (mRx.packetsPool->pop(&mRx.stagingPacket, false))
            delete mRx.stagingPacket;
        mRx.stagingPacket = nullptr;
    }

    delete mRx.fifo.release();
    delete mRx.packetsPool.release();
}

template<class T>
uint32_t LA9310_TRX::StreamRxTemplate(T* const* dest, uint32_t count, StreamRxMeta* meta, chrono::microseconds timeout)
{
    bool timestampSet = false;
    uint32_t samplesProduced = 0;
    const bool useChannelB = mConfig.channels.at(TRXDir::Rx).size() > 1;

    bool firstIteration = true;

    assert(dest);
    assert(dest[0]);
    if (useChannelB)
        assert(dest[1]);

    auto start = chrono::high_resolution_clock::now();
    while (samplesProduced < count)
    {
        if (!mRx.stagingPacket && !mRx.fifo->pop(&mRx.stagingPacket, firstIteration, timeout))
        {
            lime::error("No samples or timeout"s);
            return samplesProduced;
        }

        if (!timestampSet && meta)
        {
            meta->timestamp = mRx.stagingPacket->meta.timestamp;
            meta->hasTimestamp = true;
            timestampSet = true;
        }

        uint32_t expectedCount = count - samplesProduced;
        const uint32_t samplesToCopy = std::min(expectedCount, mRx.stagingPacket->samples.size());

        T* const* src = reinterpret_cast<T* const*>(mRx.stagingPacket->samples.front());

        std::memcpy(&dest[0][samplesProduced], src[0], samplesToCopy * sizeof(T));

        if (useChannelB)
        {
            assert(dest[1]);
            std::memcpy(&dest[1][samplesProduced], src[1], samplesToCopy * sizeof(T));
        }

        mRx.stagingPacket->samples.pop(samplesToCopy);
        mRx.stagingPacket->meta.timestamp.AddTicks(samplesToCopy);

        samplesProduced += samplesToCopy;

        if (mRx.stagingPacket->samples.empty())
        {
            mRx.packetsPool->push(mRx.stagingPacket);
            mRx.stagingPacket = nullptr;
        }

        auto duration = chrono::duration_cast<chrono::microseconds>(chrono::high_resolution_clock::now() - start);
        if (duration > timeout)
            return samplesProduced;
    }

    return samplesProduced;
}

/// @brief Receives samples from this specific stream.
/// @param samples The buffer to put the received samples in.
/// @param count The amount of samples to receive.
/// @param meta The metadata of the packets of the stream.
/// @return The amount of samples received.
uint32_t LA9310_TRX::StreamRx(
    lime::complex32f_t* const* samples, uint32_t count, StreamMeta* meta, std::chrono::microseconds timeout)
{
    StreamRxMeta rxmeta;
    uint32_t samplesRead = StreamRxTemplate(samples, count, &rxmeta, timeout);
    if (meta)
        meta->timestamp = rxmeta.timestamp.GetTicks();
    return samplesRead;
}

/// @copydoc LA9310_TRX::StreamRx()
uint32_t LA9310_TRX::StreamRx(
    lime::complex16_t* const* samples, uint32_t count, StreamMeta* meta, std::chrono::microseconds timeout)
{
    StreamRxMeta rxmeta;
    uint32_t samplesRead = StreamRxTemplate(samples, count, &rxmeta, timeout);
    if (meta)
        meta->timestamp = rxmeta.timestamp.GetTicks();
    return samplesRead;
}

/// @copydoc LA9310_TRX::StreamRx()
uint32_t LA9310_TRX::StreamRx(
    lime::complex12_t* const* samples, uint32_t count, StreamMeta* meta, std::chrono::microseconds timeout)
{
    StreamRxMeta rxmeta;
    uint32_t samplesRead = StreamRxTemplate(samples, count, &rxmeta, timeout);
    if (meta)
        meta->timestamp = rxmeta.timestamp.GetTicks();
    return samplesRead;
}

OpStatus LA9310_TRX::TxSetup()
{
    vspa.StopTx();
    const int chipId = 0;
    mTx.fifo = std::make_unique<PacketsFIFO<StreamPacket*>>(512);
    mTx.terminate.store(false, std::memory_order_relaxed);

    mTx.lastTimestamp.store(0, std::memory_order_relaxed);
    const int chCount = 1;

    uint32_t packetSize = 4096;
    mTx.samplesInPkt = 1024;

    // if (mConfig.extraConfig.tx.samplesInPacket != 0)
    // {
    //     mTx.samplesInPkt = mConfig.extraConfig.tx.samplesInPacket;
    //     lime::debug("Tx samples override %i", mTx.samplesInPkt);
    // }

    mTx.packetsToBatch = 16; // Tx packets can be flushed early without filling whole batch
    // aim batch size to desired data output period, ~100us should be good enough
    if (mConfig.hintSampleRate > 0)
        mTx.packetsToBatch = std::floor((0.0001 * mConfig.hintSampleRate) / mTx.samplesInPkt);

    if (mConfig.extraConfig.tx.packetsInBatch != 0)
        mTx.packetsToBatch = mConfig.extraConfig.tx.packetsInBatch;

    // const auto dmaChunks{ mTxArgs.dma->GetBuffers() };
    const auto dmaBufferSize = 65536; //dmaChunks.front().size;

    mTx.packetsToBatch = std::clamp<uint8_t>(mTx.packetsToBatch, 1, dmaBufferSize / packetSize);

    mTxArgs.bufferSize = dmaBufferSize;
    mTxArgs.packetSize = packetSize;
    mTxArgs.packetsToBatch = mTx.packetsToBatch;
    mTxArgs.samplesInPacket = mTx.samplesInPkt;

    {
        float bufferTimeDuration;
        if (mConfig.hintSampleRate)
            bufferTimeDuration = mTx.samplesInPkt * mTx.packetsToBatch / mConfig.hintSampleRate;
        else
            bufferTimeDuration = 0;
        char msg[256];
        std::snprintf(msg,
            sizeof(msg),
            "Tx%i Setup: samplesInTxPkt:%i maxTxPktInBatch:%i, batchSizeInTime:%gus",
            chipId,
            mTx.samplesInPkt,
            mTx.packetsToBatch,
            bufferTimeDuration * 1e6);
        if (showStats)
            printf("%s\n", msg);
        if (mCallback_logMessage)
            mCallback_logMessage(LogLevel::Verbose, msg);
    }

    mTx.packetsPool = std::make_unique<PacketsFIFO<StreamPacket*>>(1024);
    const uint32_t userSampleSize = mConfig.format == DataFormat::F32 ? sizeof(lime::complex32f_t) : sizeof(lime::complex16_t);
    for (uint32_t i = 0; i < mTx.packetsPool->max_size(); ++i)
        mTx.packetsPool->push(new StreamPacket(mTx.packetsToBatch * mTx.samplesInPkt, chCount, userSampleSize));

    mTx.terminate.store(false, std::memory_order_relaxed);
    mTx.terminateWorker.store(false, std::memory_order_relaxed);
    auto TxLoopFunction = std::bind(&LA9310_TRX::TxWorkLoop, this);

    const auto schedulingPolicy = ThreadPolicy::REALTIME;
    mTx.thread = std::thread(TxLoopFunction);
    SetOSThreadPriority(ThreadPriority::HIGHEST, schedulingPolicy, &mTx.thread);
#ifdef __linux__
    char threadName[16]; // limited to 16 chars, including null byte.
    snprintf(threadName, sizeof(threadName), "lime:Tx%i", chipId);
    pthread_setname_np(mTx.thread.native_handle(), threadName);
#endif

    // Initialize DMA
    // mTxArgs.dma->Enable(true);

    lime::debug("TxSetup wait for Tx worker.");
    // wait for Tx thread to be ready
    {
        std::unique_lock lck{ mTx.mutex };
        while (mTx.stage.load(std::memory_order_relaxed) < Stream::ReadyStage::WorkerReady)
            mTx.cv.wait(lck);
    }
    return OpStatus::Success;
}

void LA9310_TRX::TxWorkLoop()
{
    lime::debug("Tx worker thread ready.");
    // signal that thread is ready for work
    {
        std::unique_lock lck{ mTx.mutex };
        mTx.stage.store(Stream::ReadyStage::WorkerReady, std::memory_order_relaxed);
        mTx.cv.notify_all();
    }

    while (!mTx.terminateWorker.load(std::memory_order_relaxed))
    {
        // thread ready for work, just wait for stream enable
        {
            std::unique_lock lk{ streamMutex };
            while (!mStreamEnabled && !mTx.terminateWorker.load(std::memory_order_relaxed))
                streamActive.wait_for(lk, std::chrono::milliseconds(100));
        }
        if (!mStreamEnabled)
            continue;

        mTx.stage.store(Stream::ReadyStage::Active, std::memory_order_relaxed);
        TransmitPacketsLoop();
        {
            std::unique_lock lk{ mTx.mutex };
            mTx.stage.store(Stream::ReadyStage::WorkerReady, std::memory_order_relaxed);
            mTx.cv.notify_all();
        }
    }
    mTx.stage.store(Stream::ReadyStage::Disabled, std::memory_order_relaxed);
    lime::debug("Tx worker thread shutdown.");
}

void LA9310_TRX::TransmitPacketsLoop()
{
    const int chipId = 0;
    lime::debug("Tx transmit loop start.");
    StreamStats& stats = mTx.stats;

    auto& fifo = mTx.fifo;
    int64_t totalBytesSent = 0; //for data rate calculation

    StreamPacket* srcPkt = nullptr;

    auto t1{ std::chrono::steady_clock::now() };
    auto t2 = t1;

    DeltaVariable<int32_t> underrun(0);
    DeltaVariable<int32_t> loss(0);

    bool outputReady = false;
    uint8_t dmaBuffer[65536];
    uint32_t dmaFilled = 0;
    uint32_t samplesFilled = 0;
    uint32_t packetsCounter = 0;

    uint32_t bytesRemaining = 0;

    while (mTx.terminate.load(std::memory_order_relaxed) == false)
    {
        t2 = std::chrono::steady_clock::now();
        const auto timePeriod{ std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() };
        if (timePeriod >= statsPeriod_ms || mTx.terminate.load(std::memory_order_relaxed))
        {
            t1 = t2;
            double dataRate = 1000.0 * totalBytesSent / timePeriod;
            mTx.stats.dataRate_Bps = dataRate;
            loss.set(stats.loss);
            if (showStats || mCallback_logMessage)
            {
                char msg[512];
                std::snprintf(msg,
                    sizeof(msg) - 1,
                    "%s Tx%i: %3.3f MB/s | TS:%s pkt:%" PRIi64 " u:%i(%+i) l:%i(%+i) f:%" PRIuPTR,
                    "la9310", // mTxArgs.dma->GetName().c_str(),
                    chipId,
                    dataRate / 1000000.0,
                    "noTS",
                    stats.packets,
                    underrun.value(),
                    underrun.delta(),
                    loss.value(),
                    loss.delta(),
                    fifo->size());
                if (showStats)
                    lime::info("%s", msg);
                if (mCallback_logMessage)
                {
                    bool showAsWarning = underrun.delta() || loss.delta();
                    LogLevel level = showAsWarning ? LogLevel::Warning : LogLevel::Debug;
                    mCallback_logMessage(level, msg);
                }
            }
            loss.checkpoint();
            underrun.checkpoint();
            totalBytesSent = 0;
        }

        bool reportProblems = false;
        // collect and transform samples data to output buffer
        while (!outputReady && !mTx.terminate.load(std::memory_order_relaxed))
        {
            if (!srcPkt)
            {
                if (!fifo->pop(&srcPkt, true, chrono::microseconds(100000)))
                {
                    std::this_thread::yield();
                    break;
                }
            }
            uint32_t bytesForFrame = 4;
            uint32_t samplesToConsume = std::min(mTxArgs.samplesInPacket - samplesFilled, srcPkt->samples.size());

            DataConversion conversion;
            conversion.srcFormat = mConfig.format; //DataFormat::F32;
            conversion.destFormat = DataFormat::I16;
            conversion.channelCount = 1;

            int samplesDataSize = Interleave(
                reinterpret_cast<uint8_t*>(&dmaBuffer[dmaFilled]), srcPkt->samples.front(), samplesToConsume, conversion);
            srcPkt->samples.pop(samplesToConsume);
            srcPkt->meta.timestamp.AddTicks(samplesToConsume);
            dmaFilled += samplesDataSize;
            samplesFilled += samplesDataSize / bytesForFrame;

            // samplesFilled = payloadSize / bytesForFrame;
            bool isPacketFull = samplesFilled == mTxArgs.samplesInPacket;
            bool doFlush = srcPkt->meta.flush && srcPkt->samples.size() == 0;

            if (isPacketFull || doFlush)
            {
                samplesFilled = 0;
                // const int producedDataSize = sizeof(StreamHeader) + tempPacket.GetPayloadSize();
                // memcpy(outputTail, &tempPacket, producedDataSize);
                ++packetsCounter;
            }

            doFlush |= packetsCounter == mTxArgs.packetsToBatch;

            if (srcPkt->samples.empty())
            {
                mTx.packetsPool->push(srcPkt, true);
                srcPkt = nullptr;
            }

            if (doFlush)
            {
                outputReady = true;
                break;
            }
            bytesRemaining = dmaFilled;
        }

        // one callback for the entire batch
        if (reportProblems && mConfig.statusCallback)
            mConfig.statusCallback(true, &stats, mConfig.userData);

        if (!outputReady)
            continue;

        while (bytesRemaining > 0)
        {
            int32_t sent = vspa.Transmit(&dmaBuffer[dmaFilled - bytesRemaining], bytesRemaining);

            if (sent == 0)
            {
                ++stats.overrun;
                // std::this_thread::yield();
                break;
            }
            bytesRemaining -= sent;
            totalBytesSent += sent;
        }

        if (bytesRemaining == 0)
        {
            stats.packets += packetsCounter;
            outputReady = false;
            dmaFilled = 0;
            packetsCounter = 0;
        }
    }
    lime::debug("Tx transmit loop end.");
}

void LA9310_TRX::TxTeardown()
{
    if (mTx.stage.load(std::memory_order_relaxed) != Stream::ReadyStage::Disabled)
    {
        lime::debug("TxTeardown wait for Tx worker shutdown.");
        mTx.terminateWorker.store(true, std::memory_order_relaxed);
        {
            std::unique_lock lck{ streamMutex };
            mTx.terminateWorker.store(true, std::memory_order_relaxed);
            streamActive.notify_all();
        }

        mTx.terminate.store(true, std::memory_order_relaxed);
        try
        {
            if (mTx.thread.joinable())
                mTx.thread.join();
        } catch (...)
        {
            lime::error("Failed to join LA9310_TRX Tx thread"s);
        }
    }

    if (mTx.stagingPacket)
    {
        delete mTx.stagingPacket;
        mTx.stagingPacket = nullptr;
    }
    if (mTx.packetsPool)
    {
        while (mTx.packetsPool->pop(&mTx.stagingPacket, false))
            delete mTx.stagingPacket;
        mTx.stagingPacket = nullptr;
    }

    delete mTx.fifo.release();
    delete mTx.packetsPool.release();
}

template<class T>
uint32_t LA9310_TRX::StreamMetaToStreamTxMeta(
    const T* const* samples, uint32_t count, const StreamMeta* meta, std::chrono::microseconds timeout)
{
    StreamTxMeta txmeta;
    txmeta.hasTimestamp = meta ? meta->waitForTimestamp : false;
    txmeta.flags = meta ? (meta->flushPartialPacket ? StreamTxMeta::EndOfBurst : 0) : 0;
    if (txmeta.hasTimestamp)
    {
        if (mConfig.timestampType == TimestampType::SAMPLE_TICKS)
            txmeta.timestamp = Timespec(meta->timestamp / mConfig.hintSampleRate);
        else
            txmeta.timestamp = Timespec(meta->timestamp >> 32, (meta->timestamp & 0xFFFFFFFF) / 1e9);
    }
    return StreamTxTemplate(samples, count, &txmeta, timeout);
}

template<class T>
uint32_t LA9310_TRX::StreamTxTemplate(
    const T* const* samples, uint32_t count, const StreamTxMeta* meta, chrono::microseconds timeout)
{
    const bool useChannelB = mConfig.channels.at(lime::TRXDir::Tx).size() > 1;
    const bool useTimestamp = meta ? (meta->hasTimestamp) : false;
    const bool flush = meta ? (meta->flags & StreamTxMeta::EndOfBurst) : false;

    Timespec ts = meta->timestamp;
    ts.SetTickRate(mConfig.hintSampleRate);

    uint32_t samplesRemaining = count;

    bool timeGap = true; // expectedTS != lime::Timespec(meta->timestamp);

    if (mTx.stagingPacket && timeGap)
    {
        if (!mTx.fifo->push(mTx.stagingPacket, true, timeout))
            return 0;

        mTx.stagingPacket = nullptr;
    }

    assert(samples);
    assert(samples[0]);
    if (useChannelB)
        assert(samples[1]);
    const T* src[2] = { samples[0], useChannelB ? samples[1] : nullptr };
    while (samplesRemaining > 0)
    {
        if (!mTx.stagingPacket)
        {
            mTx.packetsPool->pop(&mTx.stagingPacket, true);
            if (!mTx.stagingPacket)
                break;

            mTx.stagingPacket->Reset();
            mTx.stagingPacket->meta.timestamp = ts;
            mTx.stagingPacket->meta.useTimestamp = useTimestamp;
        }

        int consumed = mTx.stagingPacket->samples.push(src, samplesRemaining);
        src[0] += consumed;
        if (useChannelB)
            src[1] += consumed;

        samplesRemaining -= consumed;
        ts.AddTicks(consumed);

        bool pushPacket = mTx.stagingPacket->samples.isFull();

        if (samplesRemaining == 0 && flush)
        {
            mTx.stagingPacket->meta.flush = flush;
            pushPacket = true;
        }

        if (pushPacket)
        {
            if (!mTx.fifo->push(mTx.stagingPacket, true, chrono::microseconds(1000000)))
                break;

            mTx.stagingPacket = nullptr;
        }
    }

    return count - samplesRemaining;
}

/// @brief Transmits packets from from this specific stream.
/// @param samples The buffer of the samples to transmit.
/// @param count The amount of samples to transmit.
/// @param meta The metadata of the packets of the stream.
/// @return The amount of samples transmitted.
uint32_t LA9310_TRX::StreamTx(
    const lime::complex32f_t* const* samples, uint32_t count, const StreamMeta* meta, std::chrono::microseconds timeout)
{
    return StreamMetaToStreamTxMeta(samples, count, meta, timeout);
}

/// @copydoc LA9310_TRX::StreamTx()
uint32_t LA9310_TRX::StreamTx(
    const lime::complex16_t* const* samples, uint32_t count, const StreamMeta* meta, std::chrono::microseconds timeout)
{
    return StreamMetaToStreamTxMeta(samples, count, meta, timeout);
}

/// @copydoc LA9310_TRX::StreamTx()
uint32_t LA9310_TRX::StreamTx(
    const lime::complex12_t* const* samples, uint32_t count, const StreamMeta* meta, std::chrono::microseconds timeout)
{
    return StreamMetaToStreamTxMeta(samples, count, meta, timeout);
}

/// @brief Gets Rx/Tx data transfer statistics.
/// @param rxStats Pointer to rx statistics structure, (Optional, can be NULL)
/// @param txStats Pointer to rx statistics structure, (Optional, can be NULL)
void LA9310_TRX::StreamStatus(StreamStats* rxStats, StreamStats* txStats)
{
    if (txStats)
    {
        *txStats = mTx.stats;
        if (mTx.fifo)
            txStats->FIFO = { mTx.fifo->max_size(), mTx.fifo->size() };
        else
            txStats->FIFO = { 1, 0 };
    }
    if (rxStats)
    {
        *rxStats = mRx.stats;
        if (mRx.fifo)
            rxStats->FIFO = { mRx.fifo->max_size(), mRx.fifo->size() };
        else
            rxStats->FIFO = { 1, 0 };
    }
}

uint32_t LA9310_TRX::Receive(lime::complex32f_t* const* samples, uint32_t count, StreamRxMeta* meta)
{
    return StreamRxTemplate<complex32f_t>(samples, count, meta, chrono::microseconds(1000000));
}

uint32_t LA9310_TRX::Receive(lime::complex16_t* const* samples, uint32_t count, StreamRxMeta* meta)
{
    return StreamRxTemplate<complex16_t>(samples, count, meta, chrono::microseconds(1000000));
}

uint32_t LA9310_TRX::Receive(lime::complex12_t* const* samples, uint32_t count, StreamRxMeta* meta)
{
    return StreamRxTemplate<complex12_t>(samples, count, meta, chrono::microseconds(1000000));
}

uint32_t LA9310_TRX::Transmit(const lime::complex32f_t* const* samples, uint32_t count, const StreamTxMeta* meta)
{
    return StreamTxTemplate(samples, count, meta, chrono::microseconds(100000));
}

uint32_t LA9310_TRX::Transmit(const lime::complex16_t* const* samples, uint32_t count, const StreamTxMeta* meta)
{
    return StreamTxTemplate(samples, count, meta, chrono::microseconds(100000));
}

uint32_t LA9310_TRX::Transmit(const lime::complex12_t* const* samples, uint32_t count, const StreamTxMeta* meta)
{
    return StreamTxTemplate(samples, count, meta, chrono::microseconds(100000));
}

} // namespace lime
