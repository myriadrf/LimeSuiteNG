#include "TRXLooper.h"

#include "AvgRmsCounter.h"
#include "comms/IDMA.h"
#include "FPGA/FPGA_common.h"
#include "limesuiteng/LMS7002M.h"
#include "limesuiteng/Logger.h"
#include "chips/LMS7002M/LMS7002MCSR_Data.h"
#include "LMSBoards.h"
#include "threadHelper.h"
#include "TxBufferManager.h"
#include "utilities/DeltaVariable.h"

#include <algorithm>
#include <cassert>
#include <ciso646>
#include <complex>
#include <queue>

using namespace std::literals::string_literals;

namespace lime {
using namespace LMS7002MCSR_Data;
using namespace std::chrono;

static constexpr uint16_t defaultSamplesInPkt = 256;

static constexpr bool showStats{ false };
static constexpr int statsPeriod_ms{ 1000 }; // at 122.88 MHz MIMO, fpga tx pkt counter overflows every 272ms

static int ReadySlots(uint32_t writer, uint32_t reader, uint32_t ringSize)
{
    assert(writer < ringSize);
    assert(reader < ringSize);
    if (writer >= reader)
        return writer - reader;
    else
        return ringSize - reader + writer;
}

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

/// @brief Constructs a new TRXLooper object.
/// @param rx The DMA communications interface to receive the data from.
/// @param tx The DMA communications interface to send the data to.
/// @param f The FPGA to use in this stream.
/// @param chip The LMS7002M chip to use in this stream.
/// @param moduleIndex The ID of the chip to use.
TRXLooper::TRXLooper(std::shared_ptr<IDMA> rx, std::shared_ptr<IDMA> tx, FPGA* f, LMS7002M* chip, uint8_t moduleIndex)
    : fpga(f)
    , lms(chip)
    , chipId(moduleIndex)
    , mCallback_logMessage(nullptr)
    , mStreamEnabled(false)
{
    mRxArgs.dma = rx;
    mTxArgs.dma = tx;

    assert(fpga);
    mTimestampOffset = 0;
}

TRXLooper::~TRXLooper()
{
    Stop();
    Teardown();
}

/// @brief Gets the current timestamp of the hardware.
/// @return The current timestamp of the hardware.
uint64_t TRXLooper::GetHardwareTimestamp() const
{
    return mRx.lastTimestamp.load(std::memory_order_relaxed) + mTimestampOffset;
}

/// @brief Sets the hardware timestamp.
/// @param now The current timestamp to set.
/// @return The status of the operation.
OpStatus TRXLooper::SetHardwareTimestamp(const uint64_t now)
{
    mTimestampOffset = now - mRx.lastTimestamp.load(std::memory_order_relaxed);
    return OpStatus::Success;
}

/// @brief Sets up the stream of this looper.
/// @param cfg The configuration settings to set up the stream with.
/// @return The status of the operation.
OpStatus TRXLooper::Setup(const StreamConfig& cfg)
{
    if (mStreamEnabled)
        return ReportError(OpStatus::Busy, "Samples streaming already running"s);

    // if (cfg.channels.at(lime::TRXDir::Rx).size() > 0 && !mRxArgs.port->IsOpen())
    //     return ReportError(OpStatus::IOFailure, "Rx data port not open"s);
    // if (cfg.channels.at(lime::TRXDir::Tx).size() > 0 && !mTxArgs.port->IsOpen())
    //     return ReportError(OpStatus::IOFailure, "Tx data port not open"s);

    float combinedSampleRate =
        std::max(cfg.channels.at(lime::TRXDir::Tx).size(), cfg.channels.at(lime::TRXDir::Rx).size()) * cfg.hintSampleRate;
    int batchSize = 7; // should be good enough for most cases
    // for high data rates e.g 16bit ADC/DAC 2x2 MIMO @ 122.88Msps = ~1973 MB/s
    // need to batch as many packets as possible into transfer buffers
    if (combinedSampleRate != 0)
    {
        batchSize = combinedSampleRate / 61.44e6;
        batchSize = std::clamp(batchSize, 1, 4);
    }

    if ((cfg.linkFormat != DataFormat::I12) && (cfg.linkFormat != DataFormat::I16))
        return ReportError(OpStatus::InvalidValue, "Unsupported stream link format"s);

    mTx.packetsToBatch = 6;
    mRx.packetsToBatch = 6;

    OpStatus status = fpga->SelectModule(chipId);
    if (status != OpStatus::Success)
        return status;
    fpga->StopStreaming();
    fpga->StopWaveformPlayback();
    fpga->ResetPacketCounters(chipId);
    fpga->ResetTimestamp();

    mConfig = cfg;
    bool needTx = cfg.channels.at(TRXDir::Tx).size() > 0;
    bool needRx = cfg.channels.at(TRXDir::Rx).size() > 0 || needTx; // always need Rx to know current timestamps, cfg.rxCount > 0;

    uint16_t channelEnables = 0;
    channelEnables |= indexListToMask(cfg.channels.at(TRXDir::Rx));
    if (channelEnables & ~0x3)
        return ReportError(OpStatus::InvalidValue, "Invalid Rx channel, only [0,1] channels supported"s);
    channelEnables |= indexListToMask(cfg.channels.at(TRXDir::Tx));
    if (channelEnables & ~0x3)
        return ReportError(OpStatus::InvalidValue, "Invalid Tx channel, only [0,1] channels supported"s);

    mConfig = cfg;

    if (!needTx && !needRx)
        return OpStatus::Success;

    const bool use_trxiqpulse = lms->Get_SPI_Reg_bits(LMS7002MCSR::LML1_TRXIQPULSE);
    const bool sisoddr_on = lms->Get_SPI_Reg_bits(LMS7002MCSR::LML1_SISODDR);
    status = fpga->ConfigureSamplesStream(channelEnables, cfg.linkFormat, sisoddr_on, use_trxiqpulse);
    if (status != OpStatus::Success)
        return status;

    // XTRX has RF switches control bits where the GPS_PPS control should be.
    bool hasGPSPPS = fpga->ReadRegister(0x0000) != LMS_DEV_LIMESDR_XTRX;
    if (hasGPSPPS)
    {
        constexpr uint16_t waitGPS_PPS = 1 << 2;
        int interface_ctrl_000A = fpga->ReadRegister(0x000A);
        interface_ctrl_000A &= ~waitGPS_PPS; // disable by default
        if (cfg.extraConfig.waitPPS)
        {
            interface_ctrl_000A |= waitGPS_PPS;
        }
        fpga->WriteRegister(0x000A, interface_ctrl_000A);
    }

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

    return OpStatus::Success;
}

/// @brief Starts the stream of this looper.
OpStatus TRXLooper::Start()
{
    OpStatus status = fpga->SelectModule(chipId);
    if (status != OpStatus::Success)
        return status;

    if (mRx.stagingPacket)
    {
        mRx.memPool->Free(mRx.stagingPacket);
        mRx.stagingPacket = nullptr;
    }
    if (mRx.fifo)
        mRx.fifo->clear();
    if (mTx.fifo)
        mTx.fifo->clear();

    // Rx start
    {
        const int32_t readSize = mRxArgs.packetSize * mRxArgs.packetsToBatch;
        constexpr uint8_t irqPeriod{ 4 };
        // Rx DMA has to be enabled before the stream enable, otherwise some data
        // might be lost in the time frame between stream enable and then dma enable.
        mRxArgs.dma->EnableContinuous(true, readSize, irqPeriod);
    }

    fpga->StartStreaming();
    {
        std::lock_guard<std::mutex> lock(streamMutex);
        mStreamEnabled = true;
        streamActive.notify_all();
    }
    return OpStatus::Success;
}

/// @brief Stops the stream and cleans up all the memory.
void TRXLooper::Stop()
{
    if (!mStreamEnabled)
        return;
    lime::debug("TRXLooper::Stop()");
    mStreamEnabled = false;
    fpga->StopStreaming();

    // wait for loop ends
    if (mRx.stage.load(std::memory_order_relaxed) == Stream::ReadyStage::Active)
    {
        mRx.terminate.store(true, std::memory_order_relaxed);
        mRxArgs.dma->Enable(false);
        lime::debug("TRXLooper: wait for Rx loop end.");
        {
            std::unique_lock lck{ mRx.mutex };
            while (mRx.stage.load(std::memory_order_relaxed) == Stream::ReadyStage::Active)
                mRx.cv.wait(lck);
        }

        if (mCallback_logMessage)
        {
            char msg[256];
            std::snprintf(msg, sizeof(msg), "Rx%i stop: packetsIn: %li", chipId, mRx.stats.packets);
            mCallback_logMessage(LogLevel::Verbose, msg);
        }
    }

    // wait for loop ends
    if (mTx.stage.load(std::memory_order_relaxed) == Stream::ReadyStage::Active)
    {
        mTx.terminate.store(true, std::memory_order_relaxed);
        mTxArgs.dma->Enable(false);
        lime::debug("TRXLooper: wait for Tx loop end."s);
        {
            std::unique_lock lck{ mTx.mutex };
            while (mTx.stage.load(std::memory_order_relaxed) == Stream::ReadyStage::Active)
                mTx.cv.wait(lck);
        }

        uint32_t fpgaTxPktIngressCount;
        uint32_t fpgaTxPktDropCounter;
        fpga->ReadTxPacketCounters(chipId, &fpgaTxPktIngressCount, &fpgaTxPktDropCounter);
        if (mCallback_logMessage)
        {
            char msg[512];
            std::snprintf(msg,
                sizeof(msg),
                "Tx%i stop: host sent packets: %li (0x%08lX), FPGA packet ingresed: %i (0x%08X), diff: %li, Tx packet dropped: %i",
                chipId,
                mTx.stats.packets,
                mTx.stats.packets,
                fpgaTxPktIngressCount,
                fpgaTxPktIngressCount,
                (mTx.stats.packets & 0xFFFFFFFF) - fpgaTxPktIngressCount,
                fpgaTxPktDropCounter);
            mCallback_logMessage(LogLevel::Verbose, msg);
        }
    }
}

/// @brief Stops all the running streams and clears up the memory.
void TRXLooper::Teardown()
{
    RxTeardown();
    TxTeardown();
}

OpStatus TRXLooper::RxSetup()
{
    OpStatus status = mRxArgs.dma->Initialize();
    if (status != OpStatus::Success)
        return status;

    mRx.samplesInPkt = defaultSamplesInPkt;
    mRx.fifo = std::make_unique<PacketsFIFO<SamplesPacketType*>>(512);
    mRx.terminate.store(false, std::memory_order_relaxed);

    mRx.lastTimestamp.store(0, std::memory_order_relaxed);
    const bool usePoll = mConfig.extraConfig.usePoll;
    const int chCount = std::max(mConfig.channels.at(lime::TRXDir::Rx).size(), mConfig.channels.at(lime::TRXDir::Tx).size());
    const int sampleSize = (mConfig.linkFormat == DataFormat::I16 ? 4 : 3); // sizeof IQ pair

    constexpr std::size_t headerSize{ sizeof(StreamHeader) };

    const int requestSamplesInPkt = 4080 / sampleSize / chCount;

    const int payloadSize = requestSamplesInPkt * sampleSize * chCount;
    const int samplesInPkt = payloadSize / (sampleSize * chCount);

    uint32_t packetSize = payloadSize + headerSize;
    packetSize = fpga->SetUpVariableRxSize(packetSize, payloadSize, sampleSize, chipId);

    mRx.packetsToBatch = 4; // TODO: adjust according to sampling rate to guarantee low latency
    if (mConfig.extraConfig.rx.packetsInBatch != 0)
        mRx.packetsToBatch = mConfig.extraConfig.rx.packetsInBatch;

    // const auto dmaBufferSize{ mRxArgs.port->GetBufferSize() };

    const auto dmaChunks{ mRxArgs.dma->GetBuffers() };
    const auto dmaBufferSize = dmaChunks.front().size;

    if (mConfig.hintSampleRate == 0)
    {
        uint8_t samplerateChannel = 0;
        // if no Rx channels are configured for streaming use channel 0 as reference sample rate
        if (!mConfig.channels.at(TRXDir::Rx).empty())
            samplerateChannel = mConfig.channels.at(TRXDir::Rx).at(0);

        mConfig.hintSampleRate =
            lms->GetSampleRate(TRXDir::Rx, samplerateChannel == 0 ? LMS7002M::Channel::ChA : LMS7002M::Channel::ChB);
    }

    // aim batch size to desired data output period, ~100us should be good enough
    if (mConfig.hintSampleRate > 0)
        mRx.packetsToBatch = std::floor((0.0001 * mConfig.hintSampleRate) / samplesInPkt);

    mRx.packetsToBatch = std::clamp<uint8_t>(mRx.packetsToBatch, 1, dmaBufferSize / packetSize);

    if (mCallback_logMessage)
    {
        float bufferTimeDuration;
        if (mConfig.hintSampleRate)
            bufferTimeDuration = samplesInPkt * mRx.packetsToBatch / mConfig.hintSampleRate;
        else
            bufferTimeDuration = 0;
        char msg[256];
        std::snprintf(msg,
            sizeof(msg),
            "Rx%i Setup: usePoll:%i rxSamplesInPkt:%i rxPacketsInBatch:%i, DMA_ReadSize:%i, link:%s, batchSizeInTime:%gus FS:%f\n",
            chipId,
            usePoll ? 1 : 0,
            samplesInPkt,
            mRx.packetsToBatch,
            mRx.packetsToBatch * packetSize,
            (mConfig.linkFormat == DataFormat::I12 ? "I12" : "I16"),
            bufferTimeDuration * 1e6,
            mConfig.hintSampleRate);
        mCallback_logMessage(LogLevel::Verbose, msg);
    }

    std::vector<uint8_t*> dmaBuffers(dmaChunks.size());
    for (uint32_t i = 0; i < dmaChunks.size(); ++i)
    {
        dmaBuffers[i] = dmaChunks[i].buffer;
    }

    mRxArgs.buffers = std::move(dmaBuffers);
    mRxArgs.bufferSize = dmaBufferSize;
    mRxArgs.packetSize = packetSize;
    mRxArgs.packetsToBatch = mRx.packetsToBatch;
    mRxArgs.samplesInPacket = samplesInPkt;

    const std::string name = "MemPool_Rx"s + std::to_string(chipId);
    const int upperAllocationLimit =
        sizeof(complex32f_t) * mRx.packetsToBatch * samplesInPkt * chCount + SamplesPacketType::headerSize;
    mRx.memPool = std::make_unique<MemoryPool>(1024, upperAllocationLimit, 8, name);

    // Don't just use REALTIME scheduling, or at least be cautious with it.
    // if the thread blocks for too long, Linux can trigger RT throttling
    // which can cause unexpected data packet losses and timing issues.
    // Also need to set policy to default here, because if host process is running
    // with REALTIME policy, these threads would inherit it and exhibit mentioned
    // issues.
    const auto schedulingPolicy = ThreadPolicy::REALTIME;
    mRx.terminate.store(false, std::memory_order_relaxed);
    mRx.terminateWorker.store(false, std::memory_order_relaxed);

    auto RxLoopFunction = std::bind(&TRXLooper::RxWorkLoop, this);
    mRx.thread = std::thread(RxLoopFunction);
    SetOSThreadPriority(ThreadPriority::HIGHEST, schedulingPolicy, &mRx.thread);
#ifdef __linux__
    char threadName[16]; // limited to 16 chars, including null byte.
    snprintf(threadName, sizeof(threadName), "lime:Rx%i", chipId);
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

struct DMATransactionCounter {
    uint64_t requests{ 0 };
    uint64_t completed{ 0 };
};

void TRXLooper::RxWorkLoop()
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
void TRXLooper::ReceivePacketsLoop()
{
    lime::debug("Rx receive loop start.");
    constexpr int headerSize{ sizeof(StreamHeader) };

    DataConversion conversion{};
    conversion.srcFormat = mConfig.linkFormat;
    conversion.destFormat = mConfig.format;
    conversion.channelCount = std::max(mConfig.channels.at(lime::TRXDir::Tx).size(), mConfig.channels.at(lime::TRXDir::Rx).size());

    const int32_t bufferCount = mRxArgs.buffers.size();
    const int32_t readSize = mRxArgs.packetSize * mRxArgs.packetsToBatch;
    const int32_t packetSize = mRxArgs.packetSize;
    const int32_t samplesInPkt = mRxArgs.samplesInPacket;
    const std::vector<uint8_t*>& dmaBuffers{ mRxArgs.buffers };
    StreamStats& stats = mRx.stats;
    auto& fifo = mRx.fifo;

    const uint8_t outputSampleSize = mConfig.format == DataFormat::F32 ? sizeof(complex32f_t) : sizeof(complex16_t);
    const int32_t outputPktSize = SamplesPacketType::headerSize + mRxArgs.packetsToBatch * samplesInPkt * outputSampleSize;

    DeltaVariable<int32_t> overrun(0);
    DeltaVariable<int32_t> loss(0);

    constexpr uint8_t irqPeriod{ 4 };

    auto t1{ std::chrono::steady_clock::now() };
    auto t2 = t1;

    int32_t Bps = 0;
    int64_t expectedTS = 0;
    SamplesPacketType* outputPkt = nullptr;

    uint32_t lastHwIndex{ 0 };
    DMATransactionCounter counters;

    assert(mRx.stagingPacket == nullptr); // should be clean start
    assert(fifo->empty());

    while (mRx.terminate.load(std::memory_order_relaxed) == false)
    {
        IDMA::State dma{ mRxArgs.dma->GetCounters() };
        int64_t counterDiff = ReadySlots(dma.transfersCompleted, lastHwIndex, 65536);
        lastHwIndex = dma.transfersCompleted;
        counters.completed += counterDiff;

        if (counterDiff > 0)
        {
            const int bytesTransferred = counterDiff * readSize;
            Bps += bytesTransferred;
            stats.bytesTransferred += bytesTransferred;
        }

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
                "%s Rx%i: %3.3f MB/s | TS:%li pkt:%li o:%i(%+i) l:%i(%+i) dma:%lu/%lu(+%li) swFIFO:%li",
                mRxArgs.dma->GetName().c_str(),
                chipId,
                stats.dataRate_Bps / 1e6,
                stats.timestamp,
                stats.packets,
                overrun.value(),
                overrun.delta(),
                loss.value(),
                loss.delta(),
                counters.requests,
                counters.completed,
                counters.completed - counters.requests,
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

        if (counters.completed - counters.requests == 0)
        {
            if (mConfig.extraConfig.usePoll)
                mRxArgs.dma->Wait();
            else
                std::this_thread::yield();
            continue;
        }

        if (outputPkt == nullptr)
        {
            outputPkt = SamplesPacketType::ConstructSamplesPacket(
                mRx.memPool->Allocate(outputPktSize), samplesInPkt * mRxArgs.packetsToBatch, outputSampleSize);
            if (outputPkt == nullptr)
            {
                lime::warning("Rx%i: packets fifo full.", chipId);
                continue;
            }
        }

        const uint64_t currentBufferIndex{ counters.requests % bufferCount };
        mRxArgs.dma->BufferOwnership(currentBufferIndex, DataTransferDirection::DeviceToHost);
        const uint8_t* buffer{ dmaBuffers.at(currentBufferIndex) };

        const FPGA_RxDataPacket* pkt = reinterpret_cast<const FPGA_RxDataPacket*>(buffer);
        outputPkt->timestamp = pkt->counter;

        bool reportProblems = false;
        const int srcPktCount = mRxArgs.packetsToBatch;
        for (int i = 0; i < srcPktCount; ++i)
        {
            pkt = reinterpret_cast<const FPGA_RxDataPacket*>(&buffer[packetSize * i]);
            if (pkt->counter - expectedTS != 0)
            {
                // printf("Loss: pkt:%li exp: %li, got: %li, diff: %li\n",
                //     stats.packets + i,
                //     expectedTS,
                //     pkt->counter,
                //     pkt->counter - expectedTS);
                ++stats.loss;
                loss.add(1);
                // reportProblems = true;
            }
            if (pkt->txWasDropped())
            {
                ++stats.late;
                // reportProblems = true; // don't spam if tx continuously has late packets
            }

            const int payloadSize{ packetSize - headerSize };
            const int samplesProduced = Deinterleave(outputPkt->back(), pkt->data, payloadSize, conversion);
            outputPkt->SetSize(outputPkt->size() + samplesProduced);
            expectedTS = pkt->counter + samplesProduced;
        }
        stats.packets += srcPktCount;
        stats.timestamp = expectedTS;
        mRx.lastTimestamp.store(expectedTS, std::memory_order_relaxed);

        if (mConfig.extraConfig.negateQ)
        {
            switch (mConfig.format)
            {
            case DataFormat::I16:
                outputPkt->Scale<complex16_t>(1, -1, mConfig.channels.at(lime::TRXDir::Rx).size());
                break;
            case DataFormat::F32:
                outputPkt->Scale<complex32f_t>(1, -1, mConfig.channels.at(lime::TRXDir::Rx).size());
                break;
            default:
                break;
            }
        }

        if (fifo->push(outputPkt, false))
        {
            outputPkt = nullptr;
        }
        else
        {
            ++stats.overrun;
            outputPkt->Reset();
        }

        mRxArgs.dma->BufferOwnership(currentBufferIndex, DataTransferDirection::HostToDevice);
        bool requestIRQ = (counters.requests % irqPeriod) == 0;
        ++counters.requests;
        mRxArgs.dma->SubmitRequest(currentBufferIndex, readSize, DataTransferDirection::DeviceToHost, requestIRQ);

        // one callback for the entire batch
        if (reportProblems && mConfig.statusCallback)
            mConfig.statusCallback(false, &stats, mConfig.userData);
        std::this_thread::yield();
    }
    lime::debug("Rx receive loop end.");
}

void TRXLooper::RxTeardown()
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
            lime::error("Failed to join TRXLooper Rx thread"s);
        }
    }

    if (mRx.stagingPacket)
    {
        mRx.memPool->Free(mRx.stagingPacket);
        mRx.stagingPacket = nullptr;
    }

    delete mRx.fifo.release();
    delete mRx.memPool.release();
}

template<class T> uint32_t TRXLooper::StreamRxTemplate(T* const* dest, uint32_t count, StreamMeta* meta)
{
    bool timestampSet = false;
    uint32_t samplesProduced = 0;
    const bool useChannelB = mConfig.channels.at(TRXDir::Rx).size() > 1;

    bool firstIteration = true;

    assert(dest);
    assert(dest[0]);
    if (useChannelB)
        assert(dest[1]);

    //auto start = high_resolution_clock::now();
    while (samplesProduced < count)
    {
        if (!mRx.stagingPacket && !mRx.fifo->pop(&mRx.stagingPacket, firstIteration, 2000))
        {
            lime::error("No samples or timeout"s);
            return samplesProduced;
        }

        if (!timestampSet && meta)
        {
            meta->timestamp = mRx.stagingPacket->timestamp;
            timestampSet = true;
        }

        uint32_t expectedCount = count - samplesProduced;
        const uint32_t samplesToCopy = std::min(expectedCount, mRx.stagingPacket->size());

        T* const* src = reinterpret_cast<T* const*>(mRx.stagingPacket->front());

        std::memcpy(&dest[0][samplesProduced], src[0], samplesToCopy * sizeof(T));

        if (useChannelB)
        {
            assert(dest[1]);
            std::memcpy(&dest[1][samplesProduced], src[1], samplesToCopy * sizeof(T));
        }

        mRx.stagingPacket->pop(samplesToCopy);
        samplesProduced += samplesToCopy;

        if (mRx.stagingPacket->empty())
        {
            mRx.memPool->Free(mRx.stagingPacket);
            mRx.stagingPacket = nullptr;
        }

        // int duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count();
        // if(duration > 300) // TODO: timeout duration in meta
        //     return samplesProduced;
    }

    return samplesProduced;
}

/// @brief Receives samples from this specific stream.
/// @param samples The buffer to put the received samples in.
/// @param count The amount of samples to receive.
/// @param meta The metadata of the packets of the stream.
/// @return The amount of samples received.
uint32_t TRXLooper::StreamRx(complex32f_t* const* samples, uint32_t count, StreamMeta* meta)
{
    return StreamRxTemplate<complex32f_t>(samples, count, meta);
}

/// @copydoc TRXLooper::StreamRx()
uint32_t TRXLooper::StreamRx(complex16_t* const* samples, uint32_t count, StreamMeta* meta)
{
    return StreamRxTemplate<complex16_t>(samples, count, meta);
}

/// @copydoc TRXLooper::StreamRx()
uint32_t TRXLooper::StreamRx(lime::complex12_t* const* samples, uint32_t count, StreamMeta* meta)
{
    return StreamRxTemplate<complex12_t>(samples, count, meta);
}

OpStatus TRXLooper::TxSetup()
{
    OpStatus status = mTxArgs.dma->Initialize();
    if (status != OpStatus::Success)
        return status;

    mTx.samplesInPkt = defaultSamplesInPkt;
    mTx.fifo = std::make_unique<PacketsFIFO<SamplesPacketType*>>(512);
    mTx.terminate.store(false, std::memory_order_relaxed);

    mTx.lastTimestamp.store(0, std::memory_order_relaxed);
    const int chCount = std::max(mConfig.channels.at(lime::TRXDir::Rx).size(), mConfig.channels.at(lime::TRXDir::Tx).size());
    const int sampleSize = (mConfig.linkFormat == DataFormat::I16 ? 4 : 3); // sizeof IQ pair

    int samplesInPkt = 256; //(mConfig.linkFormat == DataFormat::I16 ? 1020 : 1360) / chCount;
    const int packetSize = sizeof(StreamHeader) + samplesInPkt * sampleSize * chCount;

    if (mConfig.extraConfig.tx.samplesInPacket != 0)
    {
        samplesInPkt = mConfig.extraConfig.tx.samplesInPacket;
        lime::debug("Tx samples override %i", samplesInPkt);
    }

    mTx.samplesInPkt = samplesInPkt;
    mTx.packetsToBatch = 32; // Tx packets can be flushed early without filling whole batch
    if (mConfig.extraConfig.tx.packetsInBatch != 0)
    {
        mTx.packetsToBatch = mConfig.extraConfig.tx.packetsInBatch;
    }

    const auto dmaChunks{ mTxArgs.dma->GetBuffers() };
    const auto dmaBufferSize = dmaChunks.front().size;

    mTx.packetsToBatch = std::clamp<uint8_t>(mTx.packetsToBatch, 1, dmaBufferSize / packetSize);

    std::vector<uint8_t*> dmaBuffers(dmaChunks.size());
    for (uint32_t i = 0; i < dmaChunks.size(); ++i)
    {
        dmaBuffers[i] = dmaChunks[i].buffer;
    }

    mTxArgs.buffers = std::move(dmaBuffers);
    mTxArgs.bufferSize = dmaBufferSize;
    mTxArgs.packetSize = packetSize;
    mTxArgs.packetsToBatch = mTx.packetsToBatch;
    mTxArgs.samplesInPacket = samplesInPkt;

    if (mCallback_logMessage)
    {
        float bufferTimeDuration;
        if (mConfig.hintSampleRate)
            bufferTimeDuration = samplesInPkt * mTx.packetsToBatch / mConfig.hintSampleRate;
        else
            bufferTimeDuration = 0;
        char msg[256];
        std::snprintf(msg,
            sizeof(msg),
            "Tx%i Setup: samplesInTxPkt:%i maxTxPktInBatch:%i, batchSizeInTime:%gus",
            chipId,
            samplesInPkt,
            mTx.packetsToBatch,
            bufferTimeDuration * 1e6);
        mCallback_logMessage(LogLevel::Verbose, msg);
    }

    const std::string name = "MemPool_Tx"s + std::to_string(chipId);
    const int upperAllocationLimit =
        sizeof(complex32f_t) * mTx.packetsToBatch * samplesInPkt * chCount + SamplesPacketType::headerSize;
    mTx.memPool = std::make_unique<MemoryPool>(1024, upperAllocationLimit, 4096, name);

    mTx.terminate.store(false, std::memory_order_relaxed);
    mTx.terminateWorker.store(false, std::memory_order_relaxed);
    auto TxLoopFunction = std::bind(&TRXLooper::TxWorkLoop, this);

    const auto schedulingPolicy = ThreadPolicy::REALTIME;
    mTx.thread = std::thread(TxLoopFunction);
    SetOSThreadPriority(ThreadPriority::HIGHEST, schedulingPolicy, &mTx.thread);
#ifdef __linux__
    char threadName[16]; // limited to 16 chars, including null byte.
    snprintf(threadName, sizeof(threadName), "lime:Tx%i", chipId);
    pthread_setname_np(mTx.thread.native_handle(), threadName);
#endif

    // Initialize DMA
    mTxArgs.dma->Enable(true);

    lime::debug("TxSetup wait for Tx worker.");
    // wait for Tx thread to be ready
    {
        std::unique_lock lck{ mTx.mutex };
        while (mTx.stage.load(std::memory_order_relaxed) < Stream::ReadyStage::WorkerReady)
            mTx.cv.wait(lck);
    }
    return OpStatus::Success;
}

void TRXLooper::TxWorkLoop()
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

void TRXLooper::TransmitPacketsLoop()
{
    lime::debug("Tx transmit loop start.");
    const bool isRxActive = true; // rx is always activated to provide timestamps // mConfig.channels.at(lime::TRXDir::Rx).size() > 0;
    const bool mimo = std::max(mConfig.channels.at(lime::TRXDir::Tx).size(), mConfig.channels.at(lime::TRXDir::Rx).size()) > 1;
    const bool compressed = mConfig.linkFormat == DataFormat::I12;
    constexpr int irqPeriod{ 4 }; // Interrupt request period

    const uint32_t bufferCount = mTxArgs.buffers.size();
    const std::vector<uint8_t*>& dmaBuffers{ mTxArgs.buffers };

    StreamStats& stats = mTx.stats;

    auto& fifo = mTx.fifo;

    int64_t totalBytesSent = 0; //for data rate calculation
    int64_t lastTS = 0;

    struct PendingWrite {
        uint32_t id;
        uint8_t* data;
        uint32_t size;
    };
    std::queue<PendingWrite> pendingWrites;

    uint32_t stagingBufferIndex = 0;
    SamplesPacketType* srcPkt = nullptr;

    TxBufferManager<SamplesPacketType> output(mimo, compressed, mTxArgs.samplesInPacket, mTxArgs.packetsToBatch, mConfig.format);

    mTxArgs.dma->BufferOwnership(0, DataTransferDirection::DeviceToHost);
    output.Reset(dmaBuffers[0], mTxArgs.bufferSize);

    bool outputReady = false;

    AvgRmsCounter txTSAdvance;

    auto t1{ std::chrono::steady_clock::now() };
    auto t2 = t1;

    DeltaVariable<int32_t> underrun(0);
    DeltaVariable<int32_t> loss(0);

    uint64_t lastHwIndex = 0;
    DMATransactionCounter counters;

    while (mTx.terminate.load(std::memory_order_relaxed) == false)
    {
        IDMA::State dma{ mTxArgs.dma->GetCounters() };
        int64_t counterDiff = ReadySlots(dma.transfersCompleted, lastHwIndex, 65536);
        lastHwIndex = dma.transfersCompleted;
        counters.completed += counterDiff;

        // process pending transactions
        while (!pendingWrites.empty() && counterDiff > 0)
        {
            PendingWrite& dataBlock = pendingWrites.front();
            totalBytesSent += dataBlock.size;
            stats.bytesTransferred += dataBlock.size;
            pendingWrites.pop();
            --counterDiff;
        }

        t2 = std::chrono::steady_clock::now();
        const auto timePeriod{ std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() };
        if (timePeriod >= statsPeriod_ms || mTx.terminate.load(std::memory_order_relaxed))
        {
            t1 = t2;
            double dataRate = 1000.0 * totalBytesSent / timePeriod;
            mTx.stats.dataRate_Bps = dataRate;

            double avgTxAdvance = 0, rmsTxAdvance = 0;
            txTSAdvance.GetResult(avgTxAdvance, rmsTxAdvance);
            loss.set(stats.loss);
            if (showStats || mCallback_logMessage)
            {
                char msg[512];
                std::snprintf(msg,
                    sizeof(msg) - 1,
                    "%s Tx%i: %3.3f MB/s | TS:%li pkt:%li u:%i(%+i) l:%i(%+i) dma:%lu/%lu(%+li) tsAdvance:%+.0f/%+.0f/%+.0f%s, "
                    "f:%li",
                    mTxArgs.dma->GetName().c_str(),
                    chipId,
                    dataRate / 1000000.0,
                    lastTS,
                    stats.packets,
                    underrun.value(),
                    underrun.delta(),
                    loss.value(),
                    loss.delta(),
                    counters.completed,
                    counters.requests,
                    counters.requests - counters.completed,
                    txTSAdvance.Min(),
                    avgTxAdvance,
                    txTSAdvance.Max(),
                    (mConfig.hintSampleRate ? "us" : ""),
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

        // collect and transform samples data to output buffer
        while (!outputReady && output.hasSpace() && !mTx.terminate.load(std::memory_order_relaxed))
        {
            if (!srcPkt)
            {
                if (!fifo->pop(&srcPkt, true, 100))
                {
                    std::this_thread::yield();
                    break;
                }
                if (mConfig.extraConfig.negateQ)
                {
                    switch (mConfig.format)
                    {
                    case DataFormat::I16:
                        srcPkt->Scale<complex16_t>(1, -1, mConfig.channels.at(lime::TRXDir::Tx).size());
                        break;
                    case DataFormat::F32:
                        srcPkt->Scale<complex32f_t>(1, -1, mConfig.channels.at(lime::TRXDir::Tx).size());
                        break;
                    default:
                        break;
                    }
                }
            }

            // drop old packets before forming, Rx is needed to get current timestamp
            if (srcPkt->useTimestamp && isRxActive)
            {
                int64_t rxNow = mRx.lastTimestamp.load(std::memory_order_relaxed);
                const int64_t txAdvance = srcPkt->timestamp - rxNow;
                if (mConfig.hintSampleRate)
                {
                    int64_t timeAdvance = ts_to_us(mConfig.hintSampleRate, txAdvance);
                    txTSAdvance.Add(timeAdvance);
                }
                else
                    txTSAdvance.Add(txAdvance);
                if (txAdvance <= 0)
                {
                    underrun.add(1);
                    ++stats.underrun;
                    mTx.memPool->Free(srcPkt);
                    srcPkt = nullptr;
                    continue;
                }
            }

            const bool doFlush = output.consume(srcPkt);

            if (srcPkt->empty())
            {
                mTx.memPool->Free(srcPkt);
                srcPkt = nullptr;
            }
            if (doFlush)
            {
                stats.packets += output.packetCount();
                outputReady = true;
                mTxArgs.dma->BufferOwnership(stagingBufferIndex, DataTransferDirection::HostToDevice);
                break;
            }
        }

        bool canSend = pendingWrites.size() < bufferCount - 1;
        if (!canSend)
        {
            if (mConfig.extraConfig.usePoll)
                canSend = mTxArgs.dma->Wait() == OpStatus::Success;
            else
                std::this_thread::yield();
            continue;
        }

        if (!outputReady)
            continue;

        StreamHeader* pkt = reinterpret_cast<StreamHeader*>(output.data());
        lastTS = pkt->counter;
        if (isRxActive) // Rx is needed for current timestamp
        {
            int64_t rxNow = mRx.lastTimestamp.load(std::memory_order_relaxed);
            const int64_t txAdvance = pkt->counter - rxNow;
            if (mConfig.hintSampleRate)
            {
                int64_t timeAdvance = ts_to_us(mConfig.hintSampleRate, txAdvance);
                txTSAdvance.Add(timeAdvance);
            }
            else
                txTSAdvance.Add(txAdvance);
            if (txAdvance <= 0)
            {
                underrun.add(1);
                ++stats.underrun;
                // TODO: first packet in the buffer is already late, could just skip this
                // buffer transmission, but packets at the end of buffer might just still
                // make it in time.
                // outputReady = false;
                // output.Reset(dmaBuffers[stagingBufferIndex % bufferCount], mTxArgs.bufferSize);
                // continue;
            }
        }

        PendingWrite wrInfo{ stagingBufferIndex, output.data(), output.size() };
        bool requestIRQ = (wrInfo.id % irqPeriod) == 0;
        // DMA memory is write only, to read from the buffer will trigger Bus errors
        const OpStatus status{ mTxArgs.dma->SubmitRequest(
            stagingBufferIndex, wrInfo.size, DataTransferDirection::HostToDevice, requestIRQ) };
        if (status != OpStatus::Success)
        {
            lime::error("Failed to submit dma write");
            ++stats.overrun;
            mTxArgs.dma->Wait();
            continue;
        }

        pendingWrites.push(wrInfo);
        stagingBufferIndex = (stagingBufferIndex + 1) % bufferCount;
        mTxArgs.dma->BufferOwnership(stagingBufferIndex, DataTransferDirection::DeviceToHost);
        ++counters.requests;

        outputReady = false;
        stats.timestamp = lastTS;
        output.Reset(dmaBuffers[stagingBufferIndex], mTxArgs.bufferSize);
    }
    lime::debug("Tx transmit loop end.");
}

void TRXLooper::TxTeardown()
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
            lime::error("Failed to join TRXLooper Tx thread"s);
        }
    }

    if (mTx.stagingPacket)
    {
        mTx.memPool->Free(mTx.stagingPacket);
        mTx.stagingPacket = nullptr;
    }

    delete mTx.fifo.release();
    delete mTx.memPool.release();
}

template<class T> uint32_t TRXLooper::StreamTxTemplate(const T* const* samples, uint32_t count, const StreamMeta* meta)
{
    const bool useChannelB = mConfig.channels.at(lime::TRXDir::Tx).size() > 1;
    const bool useTimestamp = meta ? meta->waitForTimestamp : false;
    const bool flush = meta && meta->flushPartialPacket;
    int64_t ts = meta ? meta->timestamp : 0;

    uint32_t samplesRemaining = count;

    const int samplesInPkt = mTx.samplesInPkt;
    const int packetsToBatch = mTx.packetsToBatch;
    const int32_t outputPktSize = SamplesPacketType::headerSize + packetsToBatch * samplesInPkt * sizeof(T);

    if (mTx.stagingPacket && mTx.stagingPacket->timestamp + mTx.stagingPacket->size() != meta->timestamp)
    {
        if (!mTx.fifo->push(mTx.stagingPacket))
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
            mTx.stagingPacket = SamplesPacketType::ConstructSamplesPacket(
                mTx.memPool->Allocate(outputPktSize), samplesInPkt * packetsToBatch, sizeof(T));

            if (!mTx.stagingPacket)
                break;

            mTx.stagingPacket->Reset();
            mTx.stagingPacket->timestamp = ts;
            mTx.stagingPacket->useTimestamp = useTimestamp;
        }

        int consumed = mTx.stagingPacket->push(src, samplesRemaining);
        src[0] += consumed;
        if (useChannelB)
            src[1] += consumed;

        samplesRemaining -= consumed;
        ts += consumed;

        if (mTx.stagingPacket->isFull() || flush)
        {
            if (samplesRemaining == 0)
                mTx.stagingPacket->flush = flush;

            if (!mTx.fifo->push(mTx.stagingPacket))
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
uint32_t TRXLooper::StreamTx(const lime::complex32f_t* const* samples, uint32_t count, const StreamMeta* meta)
{
    return StreamTxTemplate(samples, count, meta);
}

/// @copydoc TRXLooper::StreamTx()
uint32_t TRXLooper::StreamTx(const lime::complex16_t* const* samples, uint32_t count, const StreamMeta* meta)
{
    return StreamTxTemplate(samples, count, meta);
}

/// @copydoc TRXLooper::StreamTx()
uint32_t TRXLooper::StreamTx(const lime::complex12_t* const* samples, uint32_t count, const StreamMeta* meta)
{
    return StreamTxTemplate(samples, count, meta);
}

/// @brief Gets statistics from a specified transfer direction.
/// @param dir The direction of which to get the statistics.
/// @return The statistics of the transfers.
StreamStats TRXLooper::GetStats(TRXDir dir) const
{
    StreamStats stats;

    if (dir == TRXDir::Tx)
    {
        stats = mTx.stats;
        if (mTx.fifo)
            stats.FIFO = { mTx.fifo->max_size(), mTx.fifo->size() };
        else
            stats.FIFO = { 1, 0 };
    }
    else
    {
        stats = mRx.stats;
        if (mRx.fifo)
            stats.FIFO = { mRx.fifo->max_size(), mRx.fifo->size() };
        else
            stats.FIFO = { 1, 0 };
    }

    return stats;
}

/// @copydoc SDRDevice::UploadTxWaveform()
/// @param fpga The FPGA device to use.
/// @param dma The data channel to use.
OpStatus TRXLooper::UploadTxWaveform(FPGA* fpga,
    std::shared_ptr<IDMA> dma,
    const lime::StreamConfig& config,
    uint8_t moduleIndex,
    const void** samples,
    uint32_t count)
{
    const int samplesInPkt = 256;
    const bool useChannelB = config.channels.at(lime::TRXDir::Tx).size() > 1;
    const bool mimo = config.channels.at(lime::TRXDir::Tx).size() == 2;
    const bool compressed = config.linkFormat == DataFormat::I12;
    fpga->SelectModule(moduleIndex);
    fpga->WriteRegister(0x000C, mimo ? 0x3 : 0x1); //channels 0,1
    if (config.linkFormat == DataFormat::I16)
        fpga->WriteRegister(0x000E, 0x0); //16bit samples
    else
        fpga->WriteRegister(0x000E, 0x2); //12bit samples

    fpga->WriteRegister(0x000D, 0x4); // WFM_LOAD

    DataConversion conversion;
    conversion.srcFormat = config.format;
    conversion.destFormat = compressed ? DataFormat::I12 : DataFormat::I16;
    conversion.channelCount = mimo ? 2 : 1;

    const auto dmaChunks{ dma->GetBuffers() };

    std::vector<uint8_t*> dmaBuffers(dmaChunks.size());
    for (uint32_t i = 0; i < dmaChunks.size(); ++i)
        dmaBuffers[i] = dmaChunks[i].buffer;

    dma->Enable(true);

    uint32_t samplesRemaining = count;
    uint8_t stagingBufferIndex = 0;

    const uint8_t* src[2] = { static_cast<const uint8_t*>(samples[0]),
        static_cast<const uint8_t*>(useChannelB ? samples[1] : nullptr) };
    const int sampleSize = config.format == DataFormat::F32 ? sizeof(lime::complex32f_t) : sizeof(lime::complex16_t);

    while (samplesRemaining > 0)
    {
        // IDMA::State state{ dma->GetCounters() };
        dma->Wait(); // block until there is a free DMA buffer

        int samplesToSend = samplesRemaining > samplesInPkt ? samplesInPkt : samplesRemaining;
        int samplesDataSize = 0;

        dma->BufferOwnership(stagingBufferIndex, DataTransferDirection::DeviceToHost);

        FPGA_TxDataPacket* pkt = reinterpret_cast<FPGA_TxDataPacket*>(dmaBuffers[stagingBufferIndex]);
        pkt->counter = 0;
        pkt->reserved[0] = 0;

        samplesDataSize = Interleave(pkt->data, reinterpret_cast<const void**>(src), samplesToSend, conversion);

        src[0] += samplesToSend * sampleSize;
        if (useChannelB)
            src[1] += samplesToSend * sampleSize;

        int payloadSize = (samplesDataSize / 4) * 4;
        if (samplesDataSize % 4 != 0)
            lime::warning("Packet samples count not multiple of 4"s);
        pkt->reserved[2] = (payloadSize >> 8) & 0xFF; //WFM loading
        pkt->reserved[1] = payloadSize & 0xFF; //WFM loading
        pkt->reserved[0] = 0x1 << 5; //WFM loading

        dma->BufferOwnership(stagingBufferIndex, DataTransferDirection::HostToDevice);

        size_t transferSize = 16 + payloadSize;
        assert(transferSize <= dmaChunks.front().size);
        bool requestIRQ = (stagingBufferIndex % 4) == 0;

        // DMA memory is write only, to read from the buffer will trigger Bus errors
        const OpStatus status{ dma->SubmitRequest(
            stagingBufferIndex, transferSize, DataTransferDirection::HostToDevice, requestIRQ) };
        if (status != OpStatus::Success)
        {
            dma->Enable(false);
            return ReportError(OpStatus::IOFailure, "Failed to submit dma write (%i) %s", errno, strerror(errno));
        }

        samplesRemaining -= samplesToSend;
        stagingBufferIndex = (stagingBufferIndex + 1) % dmaBuffers.size();
    }

    // Give some time to load samples to FPGA
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    dma->Enable(false);

    fpga->StopWaveformPlayback();
    if (samplesRemaining != 0)
        return ReportError(OpStatus::Error, "Failed to upload waveform"s);

    return OpStatus::Success;
}

} // namespace lime
