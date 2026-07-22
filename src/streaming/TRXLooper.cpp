#include "TRXLooper.h"

#include "AvgRmsCounter.h"
#include "comms/IDMA.h"
#include "FPGA/FPGA_common.h"
#include "limesuiteng/LMS7002M.h"
#include "limesuiteng/Logger.h"
#include "limesuiteng/StreamMeta.h"
#include "chips/LMS7002M/LMS7002MCSR_Data.h"
#include "protocols/LMSBoards.h"
#include "threadHelper.h"
#include "utilities/DeltaVariable.h"
#include "streaming/DataPacket.h"

#include <algorithm>
#include <cassert>
#include <ciso646>
#include <complex>
#include <queue>
#include <fstream>
#include <iostream>
#include <cinttypes>
#include <numeric>

using namespace std::literals::string_literals;

namespace lime {
using namespace LMS7002MCSR_Data;
using namespace std;
using namespace std::chrono;

static constexpr uint16_t defaultSamplesInPkt = 1360;

static constexpr bool showStats{ false };
static constexpr int statsPeriod_ms{ 1000 }; // at 122.88 MHz MIMO, fpga tx pkt counter overflows every 272ms

static struct tm ReadUTC(FPGA* fpga, uint16_t base)
{
    uint32_t addr[3] = { base, base + 1u, base + 2u };
    uint32_t reg[3];
    fpga->ReadRegisters(addr, reg, 3);
    int sec = reg[0] & 0x3F;
    int min = (reg[0] >> 6) & 0x3F;
    int h = reg[1] & 0x1F;
    int d = (reg[1] >> 5) & 0x1F;
    int m = (reg[1] >> 10) & 0xF;
    int y = (reg[2] >> 0) & 0xFFF;

    struct tm tm;
    memset(&tm, 0, sizeof(struct tm));
    tm.tm_sec = sec;
    tm.tm_min = min;
    tm.tm_hour = h;
    tm.tm_mday = d;
    tm.tm_mon = m - 1; // months since January
    tm.tm_year = y - 1900; // years since 1900
    tm.tm_isdst = -1;
    return tm;
}

static int64_t UTC_to_UnixTime(const struct tm& calendarTime)
{
    struct tm datetime;
    memcpy(&datetime, &calendarTime, sizeof(struct tm));
#ifndef __unix__
    time_t unixtime = _mkgmtime(&datetime);
#else
    time_t unixtime = timegm(&datetime);
#endif
    return unixtime;
}

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

static void NegateQChannel(StreamPacket* srcPkt, DataFormat format)
{
    switch (format)
    {
    case DataFormat::I12:
        srcPkt->samples.Scale<lime::complex12_t>(1, -1);
        break;
    case DataFormat::I16:
        srcPkt->samples.Scale<lime::complex16_t>(1, -1);
        break;
    case DataFormat::F32:
        srcPkt->samples.Scale<lime::complex32f_t>(1, -1);
        break;
    default:
        break;
    }
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
    , omitRxPackets(false)
    , startUnixTimeSet(false)
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

    bool needTx = cfg.channels.at(TRXDir::Tx).size() > 0;
    bool needRx = cfg.channels.at(TRXDir::Rx).size() > 0 || needTx; // always need Rx to know current timestamps, cfg.rxCount > 0;
    omitRxPackets = cfg.channels.at(TRXDir::Rx).size() == 0;

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
    uint16_t devId = fpga->ReadRegister(0x0000);
    bool hasGPSPPS = devId != LMS_DEV_LIMESDR_XTRX && devId != LMS_DEV_EXTERNAL_SSDR;
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

    if (mConfig.timestampType == TimestampType::SAMPLE_TICKS)
    {
        fpga->WriteRegister(0x0280, 0); // samples counting
        ticksPerSample = 1;
    }
    else
    {
        fpga->WriteRegister(0x0280, 1); // PPS, and clock ticks
        uint16_t port1sisoddr = lms->Get_SPI_Reg_bits(LMS7002MCSR::LML1_SISODDR);
        // uint16_t port2sisoddr = lms->Get_SPI_Reg_bits(LMS7002MCSR::LML2_SISODDR);
        ticksPerSample = port1sisoddr ? 1 : 2; // 1 or 2 depending on chip settings
    }

    if (mConfig.extraConfig.waitPPS)
    {
        fpga->WriteRegister(0x0281, 1); // rx start with next PPS
        fpga->WriteRegister(0x0282, 1); // tx delay
    }
    else
    {
        fpga->WriteRegister(0x0281, 0); // rx no delay
        fpga->WriteRegister(0x0282, 0); // tx delay
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

const StreamConfig& TRXLooper::GetConfig() const
{
    return mConfig;
}

/// @brief Starts the stream of this looper.
OpStatus TRXLooper::Start()
{
    if (mStreamEnabled)
        return OpStatus::Success;

    OpStatus status = fpga->SelectModule(chipId);
    if (status != OpStatus::Success)
        return status;

    // Rx start
    {
        mRx.lastTimestamp.store(0, std::memory_order_relaxed);
        const int32_t readSize = mRxArgs.packetSize * mRxArgs.packetsToBatch;
        constexpr uint8_t irqPeriod{ 4 };
        // Rx DMA has to be enabled before the stream enable, otherwise some data
        // might be lost in the time frame between stream enable and then dma enable.
        mRxArgs.dma->EnableContinuous(true, readSize, irqPeriod);
    }
    mRx.terminate.store(false, std::memory_order_relaxed);
    mTx.terminate.store(false, std::memory_order_relaxed);

    fpga->StartStreaming();
    {
        std::lock_guard<std::mutex> lock(streamMutex);
        mStreamEnabled = true;
        streamActive.notify_all();
    }
    return OpStatus::Success;
}

OpStatus TRXLooper::StageStart()
{
    return OpStatus::NotImplemented;
}

/// @brief Stops the stream and cleans up all the memory.
void TRXLooper::Stop()
{
    if (!mStreamEnabled)
        return;
    lime::debug("TRXLooper::Stop()");
    mStreamEnabled = false;

    // wait for loop ends
    if (mRx.stage.load(std::memory_order_relaxed) != Stream::ReadyStage::Disabled)
    {
        mRx.terminate.store(true, std::memory_order_relaxed);
        lime::debug("TRXLooper: wait for Rx loop end.");
        {
            std::unique_lock lck{ mRx.mutex };
            while (mRx.stage.load(std::memory_order_relaxed) == Stream::ReadyStage::Active)
                mRx.cv.wait(lck);
        }
        mRxArgs.dma->Enable(false);

        if (mCallback_logMessage)
        {
            char msg[256];
            std::snprintf(msg, sizeof(msg), "Rx%i stop: packetsIn: %" PRIi64, chipId, mRx.stats.packets);
            mCallback_logMessage(LogLevel::Verbose, msg);
        }
    }

    // wait for loop ends
    if (mTx.stage.load(std::memory_order_relaxed) != Stream::ReadyStage::Disabled)
    {
        mTx.terminate.store(true, std::memory_order_relaxed);
        lime::debug("TRXLooper: wait for Tx loop end."s);
        {
            std::unique_lock lck{ mTx.mutex };
            while (mTx.stage.load(std::memory_order_relaxed) == Stream::ReadyStage::Active)
                mTx.cv.wait(lck);
        }
        mTxArgs.dma->Enable(false);

        uint32_t fpgaTxPktIngressCount;
        uint32_t fpgaTxPktDropCounter;
        fpga->ReadTxPacketCounters(chipId, &fpgaTxPktIngressCount, &fpgaTxPktDropCounter);
        if (mCallback_logMessage)
        {
            char msg[512];
            std::snprintf(msg,
                sizeof(msg),
                "Tx%i stop: host sent packets: %" PRIi64 " (0x%08" PRIX64 "), FPGA packet ingresed: %i (0x%08X), diff: %" PRIi64
                ", Tx packet dropped: %i",
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

    // Disable FPGA streaming only after data transfer threads finish work.
    // Becase stream disable halts DMA, and threads could get stuck waiting for interrupt
    // of the next data batch.
    fpga->StopStreaming();

    if (mRx.stagingPacket != nullptr)
    {
        mRx.packetsPool->push(mRx.stagingPacket);
        mRx.stagingPacket = nullptr;
    }
    if (mRx.fifo)
    {
        while (mRx.fifo->pop(&mRx.stagingPacket, false))
            mRx.packetsPool->push(mRx.stagingPacket);
        mRx.fifo->clear();
        mRx.stagingPacket = nullptr;
    }
    if (mTx.stagingPacket != nullptr)
    {
        mTx.packetsPool->push(mTx.stagingPacket);
        mTx.stagingPacket = nullptr;
    }
    if (mTx.fifo)
    {
        while (mTx.fifo->pop(&mTx.stagingPacket, false))
            mTx.packetsPool->push(mTx.stagingPacket);
        mTx.fifo->clear();
        mTx.stagingPacket = nullptr;
    }

    mRx.lastTimestamp.store(0, std::memory_order_relaxed);
    fpga->ResetPacketCounters(chipId);
    fpga->ResetTimestamp();
    startUnixTimeSet = false;
    startUnixTime = 0;
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

    mRx.terminate.store(false, std::memory_order_relaxed);

    mRx.lastTimestamp.store(0, std::memory_order_relaxed);
    const bool usePoll = mConfig.extraConfig.usePoll;
    const int chCount = std::max(mConfig.channels.at(lime::TRXDir::Rx).size(), mConfig.channels.at(lime::TRXDir::Tx).size());
    assert(chCount > 0);
    const int sampleSize = (mConfig.linkFormat == DataFormat::I16 ? 4 : 3); // sizeof IQ pair

    constexpr std::size_t headerSize{ sizeof(StreamHeader) };

    const GatewareFeatures gw = fpga->GetFeatures();
    uint32_t packetSize = 4096;
    if (gw.hasConfigurableStreamPacketSize)
    {
        int requestSamplesInPkt = 256 / chCount;
        if (mConfig.extraConfig.rx.samplesInPacket > 0)
        {
            requestSamplesInPkt = mConfig.extraConfig.rx.samplesInPacket;
        }
        packetSize = fpga->SetUpVariableRxSize(requestSamplesInPkt, sampleSize, chCount, chipId);
        mRx.samplesInPkt = (packetSize - headerSize) / (sampleSize * chCount);
    }
    else
    {
        mRx.samplesInPkt = (packetSize - headerSize) / (sampleSize * chCount);
    }

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
        mRx.packetsToBatch = std::floor((0.0001 * mConfig.hintSampleRate) / mRx.samplesInPkt);

    if (mConfig.extraConfig.rx.packetsInBatch != 0)
        mRx.packetsToBatch = mConfig.extraConfig.rx.packetsInBatch;

    mRx.packetsToBatch = std::clamp<uint32_t>(mRx.packetsToBatch, 1u, dmaBufferSize / packetSize);

    float bufferTimeDuration;
    if (mConfig.hintSampleRate)
        bufferTimeDuration = mRx.samplesInPkt * mRx.packetsToBatch / mConfig.hintSampleRate;
    else
        bufferTimeDuration = 0;

    std::vector<uint8_t*> dmaBuffers(dmaChunks.size());
    for (uint32_t i = 0; i < dmaChunks.size(); ++i)
    {
        dmaBuffers[i] = dmaChunks[i].buffer;
    }

    mRxArgs.buffers = std::move(dmaBuffers);
    mRxArgs.bufferSize = dmaBufferSize;
    mRxArgs.packetSize = packetSize;
    mRxArgs.packetsToBatch = mRx.packetsToBatch;
    mRxArgs.samplesInPacket = mRx.samplesInPkt;

    assert(mRxArgs.bufferSize > 0);
    assert(mRxArgs.packetSize > 0);
    assert(mRxArgs.packetsToBatch > 0);
    assert(mRxArgs.samplesInPacket > 0);

    const uint32_t rxPacketSamples = mRx.samplesInPkt * mRx.packetsToBatch;
    int packetsInFIFO = 0.25 * mConfig.hintSampleRate / rxPacketSamples; // buffer 0.25 second of data
    // honor the requested FIFO size (in samples), the pool needs 2 extra packets for staging
    if (mConfig.bufferSize > 0)
        packetsInFIFO = (mConfig.bufferSize + rxPacketSamples - 1) / rxPacketSamples + 2;
    mRx.packetsPool = std::make_unique<MTStack<StreamPacket*>>(packetsInFIFO);
    const uint32_t userSampleSize = mConfig.format == DataFormat::F32 ? sizeof(lime::complex32f_t) : sizeof(lime::complex16_t);
    for (uint32_t i = 0; i < mRx.packetsPool->capacity(); ++i)
        mRx.packetsPool->push(new StreamPacket(mRx.samplesInPkt * mRx.packetsToBatch, chCount, userSampleSize));
    mRx.fifo = std::make_unique<PacketsFIFO<StreamPacket*>>(mRx.packetsPool->capacity() - 2);

    char msg[256];
    std::snprintf(msg,
        sizeof(msg),
        "%s Rx%i Setup: usePoll:%i rxSamplesInPkt:%i rxPacketsInBatch:%i, DMA_ReadSize:%i, link:%s, batchSizeInTime:%gus FS:%f, "
        "FIFO=%i*%i\n",
        mRxArgs.dma->GetName().c_str(),
        chipId,
        usePoll ? 1 : 0,
        mRx.samplesInPkt,
        mRx.packetsToBatch,
        mRx.packetsToBatch * packetSize,
        (mConfig.linkFormat == DataFormat::I12 ? "I12" : "I16"),
        bufferTimeDuration * 1e6,
        mConfig.hintSampleRate,
        packetsInFIFO,
        mRx.samplesInPkt * mRx.packetsToBatch);
    if (showStats)
        printf("%s", msg);
    if (mCallback_logMessage)
        mCallback_logMessage(LogLevel::Verbose, msg);

    // REALTIME scheduling is safe only when the worker blocks on DMA interrupts
    // (usePoll). In the busy wait mode the thread never sleeps, and a spinning
    // realtime thread can exceed its kernel scheduling slot, triggering RT
    // throttling, packet loss and timing issues. The policy is set explicitly
    // in both cases so the workers do not inherit REALTIME from the host process.
    const auto schedulingPolicy = mConfig.extraConfig.usePoll ? ThreadPolicy::REALTIME : ThreadPolicy::DEFAULT;
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

    return status;
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

inline static Timespec ExtractPacketTimestamp(
    const StreamConfig& config, const FPGA_RxDataPacket* fpgapacket, int clockTicksPerSample)
{
    switch (config.timestampType)
    {
    case TimestampType::SAMPLE_TICKS:
        return Timespec(0, fpgapacket->counter, config.hintSampleRate);
        break;
    case TimestampType::REALTIME_SECONDS:
    case TimestampType::UNIX_EPOCH: {
        uint32_t clockCount = fpgapacket->counter & 0xFFFFFFFF;
        uint32_t PPScount = (fpgapacket->counter >> 32) & 0xFFFFFFFF;
        uint64_t ticks = clockCount; // depending on interface configuration there might be 2 or 1 tick per sample
        uint64_t seconds = PPScount;
        return Timespec(seconds, ticks, clockTicksPerSample * config.hintSampleRate);
    }
    break;
    }
    return Timespec();
}

inline static int32_t ExtractPacketSamples(const DataConversion& conversion,
    const TRXLooper::TransferArgs& args,
    StreamPacket* userPkt,
    const FPGA_RxDataPacket* fpgapacket)
{
    assert(userPkt);
    assert(userPkt->samples.isFull() == false);

    const size_t payloadSize{ args.packetSize - sizeof(StreamHeader) };
    const int samplesProduced = Deinterleave(userPkt->samples.back(), fpgapacket->data, payloadSize, conversion);
    userPkt->samples.SetSize(userPkt->samples.size() + samplesProduced);
    return samplesProduced;
}

static std::string timespec_to_utc_string(struct timespec* ts)
{
    const int nanosecond_offset = 21;
    char buf[64];
    struct tm tm;
#ifdef __unix__
    gmtime_r(&ts->tv_sec, &tm);
#else
    gmtime_s(&tm, &ts->tv_sec);
#endif
    strftime(buf, nanosecond_offset, "%Y-%m-%dT%H:%M:%S.", &tm);
    sprintf(buf + nanosecond_offset - 1, "%09luZ", ts->tv_nsec);
    return std::string(buf);
}

static std::string TimestampToString(Timespec timestamp, TimestampType type)
{
    char timestampstr[256];
    switch (type)
    {
    case TimestampType::SAMPLE_TICKS:
        snprintf(timestampstr, sizeof(timestampstr), "%" PRIu64, timestamp.GetTicks());
        break;
    case TimestampType::REALTIME_SECONDS:
        snprintf(timestampstr, sizeof(timestampstr), "%.9fs", timestamp.GetRealSeconds());
        break;
    case TimestampType::UNIX_EPOCH: {
        struct timespec ts;
        ts.tv_sec = timestamp.GetSeconds();
        ts.tv_nsec = timestamp.GetFracSeconds() * 1e9;
        std::string utctimestamp = timespec_to_utc_string(&ts);
        snprintf(timestampstr, sizeof(timestampstr), "%s", utctimestamp.c_str());
        break;
    }
    }
    return std::string(timestampstr);
}

/** @brief Function dedicated for receiving data samples from board */
void TRXLooper::ReceivePacketsLoop()
{
    lime::debug("Rx receive loop start.");

    const int32_t bufferCount = mRxArgs.buffers.size();
    const int32_t readSize = mRxArgs.packetSize * mRxArgs.packetsToBatch;
    const int32_t packetSize = mRxArgs.packetSize;
    const std::vector<uint8_t*>& dmaBuffers{ mRxArgs.buffers };
    StreamStats& stats = mRx.stats;
    auto& fifo = mRx.fifo;

    DataConversion conversion;
    conversion.srcFormat = mConfig.linkFormat;
    conversion.destFormat = mConfig.format;
    conversion.channelCount = std::max(mConfig.channels.at(lime::TRXDir::Tx).size(), mConfig.channels.at(lime::TRXDir::Rx).size());

    DeltaVariable<int32_t> overrun(0);
    DeltaVariable<int32_t> loss(0);

    constexpr uint8_t irqPeriod{ 4 };

    auto t1{ std::chrono::steady_clock::now() };
    auto t2 = t1;

    int32_t Bps = 0;
    StreamPacket* userPkt = nullptr;

    uint32_t lastHwIndex{ 0 };
    DMATransactionCounter counters;

    assert(mRx.stagingPacket == nullptr); // should be clean start
    assert(fifo->empty());

    bool getStartTime = mConfig.timestampType != TimestampType::SAMPLE_TICKS;
    startUnixTime = 0;

    Timespec lastPacketTS;
    Timespec expectedTimestamp;
    Timespec fpgaFrontEndDelay;
    expectedTimestamp.SetTickRate(ticksPerSample * mConfig.hintSampleRate);

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
                "%s Rx%i: %3.3f MB/s | TS:%s pkt:%" PRIi64 " o:%i(%+i) l:%i(%+i) dma:%" PRIu64 "/%" PRIu64 "(+%" PRIu64
                ") swFIFO:%" PRIuPTR,
                mRxArgs.dma->GetName().c_str(),
                chipId,
                stats.dataRate_Bps / 1e6,
                TimestampToString(lastPacketTS, mConfig.timestampType).c_str(),
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

        const uint64_t currentBufferIndex{ counters.requests % bufferCount };
        mRxArgs.dma->BufferOwnership(currentBufferIndex, DataTransferDirection::DeviceToHost);
        const uint8_t* buffer{ dmaBuffers.at(currentBufferIndex) };

        const FPGA_RxDataPacket* firstPkt = reinterpret_cast<const FPGA_RxDataPacket*>(buffer);

        if (getStartTime)
        {
            std::lock_guard<std::mutex> lock(startTimeMutex);
            startUnixTime = UTC_to_UnixTime(ReadUTC(fpga, 0x0283));
            ++startUnixTime; // last GNSS message is from previous PPS, so add 1 second
            startUnixTimeSet = true;
            // stream starts with the PPS signal, but there is delay until the samples are put into packet by the hardware
            // rewind timstamps by the amount of first packet clockCounter
            if (mConfig.extraConfig.waitPPS)
            {
                fpgaFrontEndDelay = ExtractPacketTimestamp(mConfig, firstPkt, ticksPerSample);
                expectedTimestamp = expectedTimestamp + fpgaFrontEndDelay;
            }
            getStartTime = false;
            t1 = std::chrono::steady_clock::now();
        }
        startTimeIsSet.notify_all();

        bool reportProblems = false;
        if (userPkt == nullptr)
        {
            if (!mRx.packetsPool->pop(&userPkt))
            {
                ++stats.overrun;
                overrun.add(1);
                reportProblems = true;
                continue;
            }
            userPkt->Reset();
            userPkt->meta.timestamp = ExtractPacketTimestamp(mConfig, firstPkt, ticksPerSample);
            userPkt->meta.useTimestamp = true;
            userPkt->meta.flush = false;
        }

        const int srcPktCount = mRxArgs.packetsToBatch;
        for (int i = 0; i < srcPktCount; ++i)
        {
            const FPGA_RxDataPacket* hardwarePkt = reinterpret_cast<const FPGA_RxDataPacket*>(&buffer[packetSize * i]);
            Timespec hwts = ExtractPacketTimestamp(mConfig, hardwarePkt, ticksPerSample);

            // clock counter can drift, and gets reset with PPS, creating small discontinuity in expected and received counter
            Timespec diff = (hwts - expectedTimestamp);
            const Timespec samplePeriod(0, 1.0 / mConfig.hintSampleRate);
            if (abs(diff) > samplePeriod)
            {
                int64_t fpgaTicks = hardwarePkt->counter & 0xFFFFFFFF;
                if (mConfig.timestampType != TimestampType::SAMPLE_TICKS)
                {
                    lime::debug("Expected PPS=%li, CLK=%li, got FPGA PPS=%li, CLK=%li, diff=%lins",
                        expectedTimestamp.GetSeconds(),
                        static_cast<int64_t>(expectedTimestamp.GetFracSeconds() * ticksPerSample * mConfig.hintSampleRate),
                        hardwarePkt->counter >> 32,
                        fpgaTicks,
                        static_cast<int64_t>((diff.GetSeconds() + diff.GetFracSeconds()) * 1e9));
                }
                lime::debug("Loss: pkt:%li exp: %016lx, got: %016lx, diff: %+li, timeDiff:%+lins",
                    stats.packets + i,
                    expectedTimestamp.GetTicks(),
                    hwts.GetTicks(),
                    hwts.GetTicks() - expectedTimestamp.GetTicks(),
                    static_cast<int64_t>((diff.GetSeconds() + diff.GetFracSeconds()) * 1e9));
                ++stats.loss;
                loss.add(1);
                reportProblems = true;
            }
            if (hardwarePkt->txWasDropped())
            {
                ++mTx.stats.loss;
            }
            expectedTimestamp = hwts;
            expectedTimestamp.AddTicks(mRxArgs.samplesInPacket * ticksPerSample);

            lastPacketTS = hwts;
            if (mConfig.timestampType == TimestampType::UNIX_EPOCH)
                lastPacketTS = lastPacketTS + Timespec(startUnixTime);

            if (omitRxPackets)
                continue;

            ExtractPacketSamples(conversion, mRxArgs, userPkt, hardwarePkt);

            if (mConfig.extraConfig.negateQ)
                NegateQChannel(userPkt, mConfig.format);

            if (mConfig.timestampType == TimestampType::UNIX_EPOCH)
            {
                userPkt->meta.timestamp = userPkt->meta.timestamp + Timespec(startUnixTime);
                userPkt->meta.timestamp = userPkt->meta.timestamp - fpgaFrontEndDelay;
            }
        }

        assert(userPkt);
        if (fifo->push(userPkt, false))
            userPkt = nullptr;
        else
        {
            ++stats.overrun;
            overrun.add(1);
            userPkt->Reset();
            reportProblems = true;
        }

        stats.packets += srcPktCount;
        stats.timestamp = expectedTimestamp.GetTicks();
        mRx.lastTimestamp.store(expectedTimestamp.GetTicks(), std::memory_order_relaxed);

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
        delete mRx.stagingPacket;
        mRx.stagingPacket = nullptr;
    }

    if (mRx.packetsPool)
    {
        while (mRx.packetsPool->pop(&mRx.stagingPacket))
            delete mRx.stagingPacket;
        mRx.stagingPacket = nullptr;
    }

    delete mRx.fifo.release();
    delete mRx.packetsPool.release();
}

template<class T>
uint32_t TRXLooper::StreamRxTemplate(T* const* dest, uint32_t count, StreamRxMeta* meta, chrono::microseconds timeout)
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
        mRx.stagingPacket->meta.timestamp.AddTicks(samplesToCopy * ticksPerSample);

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
uint32_t TRXLooper::StreamRx(
    lime::complex32f_t* const* samples, uint32_t count, StreamMeta* meta, std::chrono::microseconds timeout)
{
    StreamRxMeta rxmeta;
    uint32_t samplesRead = StreamRxTemplate(samples, count, &rxmeta, timeout);
    if (meta)
        meta->timestamp = rxmeta.timestamp.GetTicks();
    return samplesRead;
}

/// @copydoc TRXLooper::StreamRx()
uint32_t TRXLooper::StreamRx(lime::complex16_t* const* samples, uint32_t count, StreamMeta* meta, std::chrono::microseconds timeout)
{
    StreamRxMeta rxmeta;
    uint32_t samplesRead = StreamRxTemplate(samples, count, &rxmeta, timeout);
    if (meta)
        meta->timestamp = rxmeta.timestamp.GetTicks();
    return samplesRead;
}

/// @copydoc TRXLooper::StreamRx()
uint32_t TRXLooper::StreamRx(lime::complex12_t* const* samples, uint32_t count, StreamMeta* meta, std::chrono::microseconds timeout)
{
    StreamRxMeta rxmeta;
    uint32_t samplesRead = StreamRxTemplate(samples, count, &rxmeta, timeout);
    if (meta)
        meta->timestamp = rxmeta.timestamp.GetTicks();
    return samplesRead;
}

uint32_t TRXLooper::Receive(lime::complex32f_t* const* samples, uint32_t count, StreamRxMeta* meta)
{
    return StreamRxTemplate<complex32f_t>(samples, count, meta, chrono::microseconds(1000000));
}

uint32_t TRXLooper::Receive(lime::complex16_t* const* samples, uint32_t count, StreamRxMeta* meta)
{
    return StreamRxTemplate<complex16_t>(samples, count, meta, chrono::microseconds(1000000));
}

uint32_t TRXLooper::Receive(lime::complex12_t* const* samples, uint32_t count, StreamRxMeta* meta)
{
    return StreamRxTemplate<complex12_t>(samples, count, meta, chrono::microseconds(1000000));
}

OpStatus TRXLooper::TxSetup()
{
    OpStatus status = mTxArgs.dma->Initialize();
    if (status != OpStatus::Success)
        return status;

    mTx.samplesInPkt = defaultSamplesInPkt;
    mTx.terminate.store(false, std::memory_order_relaxed);

    mTx.lastTimestamp.store(0, std::memory_order_relaxed);
    const int chCount = std::max(mConfig.channels.at(lime::TRXDir::Rx).size(), mConfig.channels.at(lime::TRXDir::Tx).size());
    assert(chCount > 0);
    const int sampleSize = (mConfig.linkFormat == DataFormat::I16 ? 4 : 3); // sizeof IQ pair

    const GatewareFeatures gw = fpga->GetFeatures();
    uint32_t packetSize;
    if (gw.hasConfigurableStreamPacketSize)
    {
        mTx.samplesInPkt = 256 / chCount;
        if (mConfig.extraConfig.tx.samplesInPacket != 0)
        {
            mTx.samplesInPkt = mConfig.extraConfig.tx.samplesInPacket;
            lime::debug("Tx samples override %i", mTx.samplesInPkt);
        }
        packetSize = GetPacketSizeForBusSize(mTx.samplesInPkt, sampleSize, chCount, 32);
        const int headerSize = sizeof(StreamHeader);
        mTx.samplesInPkt = (packetSize - headerSize) / (sampleSize * chCount);
    }
    else
    {
        // FT601 USB encounters random BUS and IOMMU errors if transmitting not in 4096 byte chunks
        mTx.samplesInPkt = 4080 / sampleSize / chCount;
        packetSize = 4096;
    }

    mTx.packetsToBatch = 16; // Tx packets can be flushed early without filling whole batch
    // aim batch size to desired data output period, ~100us should be good enough
    if (mConfig.hintSampleRate > 0)
        mTx.packetsToBatch = std::floor((0.0005 * mConfig.hintSampleRate) / mTx.samplesInPkt);

    if (mConfig.extraConfig.tx.packetsInBatch != 0)
    {
        mTx.packetsToBatch = mConfig.extraConfig.tx.packetsInBatch;
    }

    const auto dmaChunks{ mTxArgs.dma->GetBuffers() };
    const auto dmaBufferSize = dmaChunks.front().size;

    mTx.packetsToBatch = std::clamp<uint32_t>(mTx.packetsToBatch, 1u, dmaBufferSize / packetSize);

    std::vector<uint8_t*> dmaBuffers(dmaChunks.size());
    for (uint32_t i = 0; i < dmaChunks.size(); ++i)
    {
        dmaBuffers[i] = dmaChunks[i].buffer;
    }

    mTxArgs.buffers = std::move(dmaBuffers);
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

    const uint32_t txPacketSamples = mTx.packetsToBatch * mTx.samplesInPkt;
    int packetsInFIFO = 0.25 * mConfig.hintSampleRate / txPacketSamples; // buffer 0.25 second of data
    // honor the requested FIFO size (in samples), the pool needs 2 extra packets for staging
    if (mConfig.bufferSize > 0)
        packetsInFIFO = (mConfig.bufferSize + txPacketSamples - 1) / txPacketSamples + 2;
    mTx.packetsPool = std::make_unique<MTStack<StreamPacket*>>(packetsInFIFO);
    const uint32_t userSampleSize = mConfig.format == DataFormat::F32 ? sizeof(lime::complex32f_t) : sizeof(lime::complex16_t);
    for (uint32_t i = 0; i < mTx.packetsPool->capacity(); ++i)
        mTx.packetsPool->push(new StreamPacket(mTx.packetsToBatch * mTx.samplesInPkt, chCount, userSampleSize));
    mTx.fifo = std::make_unique<PacketsFIFO<StreamPacket*>>(mTx.packetsPool->capacity() - 2);

    mTx.terminate.store(false, std::memory_order_relaxed);
    mTx.terminateWorker.store(false, std::memory_order_relaxed);
    auto TxLoopFunction = std::bind(&TRXLooper::TxWorkLoop, this);

    // see RxSetup for the reasoning behind the policy selection
    const auto schedulingPolicy = mConfig.extraConfig.usePoll ? ThreadPolicy::REALTIME : ThreadPolicy::DEFAULT;
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

static void TxPacketPadding(FPGA_TxDataPacket& packet, DataFormat linkFormat, uint8_t channelCount)
{
    // Tx data transfers have to be multiple of the bus size
    constexpr uint16_t busWidthBytes = 32;
    assert(busWidthBytes > 0);
    const int frameSize = (linkFormat == DataFormat::I12 ? 3 : 4) * channelCount;
    const int headerSize = sizeof(StreamHeader);
    const int maxPacketSize = 4096;

    uint16_t payloadSize = packet.GetPayloadSize();
    uint32_t samplesCount = payloadSize / frameSize;
    uint32_t packetSize = headerSize + payloadSize;

    while (packetSize % busWidthBytes)
    {
        packetSize += frameSize;
        ++samplesCount;

        if (packetSize >= maxPacketSize)
            break;
    }
    if (packetSize % busWidthBytes)
    {
        printf("Failed to pad Tx packet\n");
    }

    uint16_t paddingSize = packetSize - headerSize - packet.GetPayloadSize();
    if (paddingSize > 0)
    {
        std::memset(&packet.data[payloadSize], 0, paddingSize); // pad with zeroes
        packet.SetPayloadSize(payloadSize + paddingSize);
    }
}

void TRXLooper::TransmitPacketsLoop()
{
    lime::debug("Tx transmit loop start.");
    const bool isRxActive =
        true; // rx is always activated to provide timestamps // mConfig.channels.at(lime::TRXDir::Rx).size() > 0;
    const bool mimo = std::max(mConfig.channels.at(lime::TRXDir::Tx).size(), mConfig.channels.at(lime::TRXDir::Rx).size()) > 1;
    const bool compressed = mConfig.linkFormat == DataFormat::I12;
    constexpr int irqPeriod{ 4 }; // Interrupt request period

    const uint32_t bufferCount = mTxArgs.buffers.size();
    const std::vector<uint8_t*>& dmaBuffers{ mTxArgs.buffers };

    StreamStats& stats = mTx.stats;

    auto& fifo = mTx.fifo;

    int64_t totalBytesSent = 0; //for data rate calculation
    Timespec lastTS;

    struct PendingWrite {
        uint32_t id;
        uint8_t* data;
        uint32_t size;
    };
    std::queue<PendingWrite> pendingWrites;

    uint32_t stagingBufferIndex = 0;
    StreamPacket* srcPkt = nullptr;

    mTxArgs.dma->BufferOwnership(0, DataTransferDirection::DeviceToHost);

    bool outputReady = false;

    AvgRmsCounter txTSAdvance;

    auto t1{ std::chrono::steady_clock::now() };
    auto t2 = t1;

    DeltaVariable<int32_t> underrun(0);
    DeltaVariable<int32_t> loss(0);

    uint64_t lastHwIndex = 0;
    DMATransactionCounter counters;

    FPGA_TxDataPacket tempPacket;

    uint8_t* outputTail = dmaBuffers[0];
    uint64_t packetsCounter = 0;

    if (mConfig.timestampType == TimestampType::UNIX_EPOCH)
    {
        std::unique_lock lk{ startTimeMutex };
        while (!startUnixTimeSet && !mTx.terminateWorker.load(std::memory_order_relaxed))
        {
            startTimeIsSet.wait_for(lk, std::chrono::milliseconds(100));
        }
    }

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
                    "%s Tx%i: %3.3f MB/s | TS:%s pkt:%" PRIi64 " u:%i(%+i) l:%i(%+i) dma:%" PRIu64 "/%" PRIu64 "(%+" PRIi64
                    ") tsAdvance:%+.0f/%+.0f/%+.0f%s, f:%" PRIuPTR,
                    mTxArgs.dma->GetName().c_str(),
                    chipId,
                    dataRate / 1000000.0,
                    TimestampToString(lastTS, mConfig.timestampType).c_str(),
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
                if (mConfig.extraConfig.negateQ)
                    NegateQChannel(srcPkt, mConfig.format);

                lastTS = srcPkt->meta.timestamp;
                if (mConfig.timestampType == TimestampType::UNIX_EPOCH)
                {
                    srcPkt->meta.timestamp = srcPkt->meta.timestamp - Timespec(startUnixTime);
                    if (srcPkt->meta.useTimestamp && srcPkt->meta.timestamp.GetSeconds() < 0) // Drop packets that are in the past
                    {
                        reportProblems = true;
                        ++stats.underrun;
                        srcPkt->Reset();
                        mTx.packetsPool->push(srcPkt);
                        srcPkt = nullptr;
                        break;
                    }
                }
            }

            uint32_t payloadOffset = tempPacket.GetPayloadSize();
            uint8_t* payload = &tempPacket.data[payloadOffset];

            if (payloadOffset == 0)
            {
                tempPacket.ignoreTimestamp(!srcPkt->meta.useTimestamp);
                if (mConfig.timestampType == TimestampType::SAMPLE_TICKS)
                    tempPacket.counter = srcPkt->meta.timestamp.GetTicks();
                else
                {
                    tempPacket.counter = srcPkt->meta.timestamp.GetSeconds() << 32;
                    tempPacket.counter |=
                        uint64_t(srcPkt->meta.timestamp.GetFracSeconds() * mConfig.hintSampleRate * ticksPerSample);
                }
            }

            uint32_t bytesForFrame = (compressed ? 3 : 4) * (mimo ? 2 : 1);

            uint32_t samplesFilled = payloadOffset / bytesForFrame;
            uint32_t samplesToConsume = std::min(mTxArgs.samplesInPacket - samplesFilled, srcPkt->samples.size());

            DataConversion conversion;
            conversion.srcFormat = mConfig.format; //DataFormat::F32;
            conversion.destFormat = compressed ? DataFormat::I12 : DataFormat::I16;
            conversion.channelCount = mimo ? 2 : 1;

            int samplesDataSize = Interleave(payload, srcPkt->samples.front(), samplesToConsume, conversion);
            srcPkt->samples.pop(samplesToConsume);
            srcPkt->meta.timestamp.AddTicks(samplesToConsume * ticksPerSample);
            payloadOffset += samplesDataSize;
            tempPacket.SetPayloadSize(payloadOffset);
            assert(payloadOffset > 0);

            samplesFilled = payloadOffset / bytesForFrame;
            bool isPacketFull = samplesFilled == mTxArgs.samplesInPacket;
            bool doFlush = srcPkt->meta.flush && srcPkt->samples.size() == 0;
            if (isPacketFull || doFlush)
            {
                ++packetsCounter;

                TxPacketPadding(tempPacket, mConfig.linkFormat, conversion.channelCount);

                const int producedDataSize = sizeof(StreamHeader) + tempPacket.GetPayloadSize();
                memcpy(outputTail, &tempPacket, producedDataSize);
                outputTail += producedDataSize;
                tempPacket.ClearHeader();
            }

            doFlush |= packetsCounter == mTxArgs.packetsToBatch;

            if (srcPkt->samples.empty())
            {
                mTx.packetsPool->push(srcPkt);
                srcPkt = nullptr;
            }

            if (doFlush)
            {
                stats.packets += packetsCounter;
                packetsCounter = 0;
                outputReady = true;
                break;
            }
        }

        // one callback for the entire batch
        if (reportProblems && mConfig.statusCallback)
            mConfig.statusCallback(true, &stats, mConfig.userData);

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

        FPGA_TxDataPacket* pkt = reinterpret_cast<FPGA_TxDataPacket*>(dmaBuffers[stagingBufferIndex]);
        if (!pkt->getIgnoreTimestamp() && isRxActive) // Rx is needed for current timestamp
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
                reportProblems = true;
                underrun.add(1);
                ++stats.underrun;
            }
        }

        if (reportProblems && mConfig.statusCallback)
            mConfig.statusCallback(true, &stats, mConfig.userData);

        uint32_t bytesToSend = outputTail - dmaBuffers[stagingBufferIndex];
        PendingWrite wrInfo{ stagingBufferIndex, dmaBuffers[stagingBufferIndex], bytesToSend };
        bool requestIRQ = (wrInfo.id % irqPeriod) == 0;
        // DMA memory is write only, to read from the buffer will trigger Bus errors
        mTxArgs.dma->BufferOwnership(stagingBufferIndex, DataTransferDirection::HostToDevice);
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
        stats.timestamp = lastTS.GetTicks();
        outputTail = dmaBuffers[stagingBufferIndex];
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
        delete mTx.stagingPacket;
        mTx.stagingPacket = nullptr;
    }
    if (mTx.packetsPool)
    {
        while (mTx.packetsPool->pop(&mTx.stagingPacket))
            delete mTx.stagingPacket;
        mTx.stagingPacket = nullptr;
    }

    delete mTx.fifo.release();
    delete mTx.packetsPool.release();
}

template<class T>
uint32_t TRXLooper::StreamMetaToStreamTxMeta(
    const T* const* samples, uint32_t count, const StreamMeta* meta, std::chrono::microseconds timeout)
{
    StreamTxMeta txmeta;
    txmeta.hasTimestamp = meta ? meta->waitForTimestamp : false;
    txmeta.flags = meta ? (meta->flushPartialPacket ? StreamTxMeta::EndOfBurst : 0) : 0;
    if (txmeta.hasTimestamp)
    {
        if (mConfig.timestampType == TimestampType::SAMPLE_TICKS)
            // Preserve sample-tick timestamp semantics when converting generic StreamMeta.
            txmeta.timestamp = Timespec(0, meta->timestamp, mConfig.hintSampleRate);
        else
            txmeta.timestamp = Timespec(meta->timestamp >> 32, (meta->timestamp & 0xFFFFFFFF) / 1e9);
    }
    return StreamTxTemplate(samples, count, &txmeta, timeout);
}

template<class T>
uint32_t TRXLooper::StreamTxTemplate(
    const T* const* samples, uint32_t count, const StreamTxMeta* meta, chrono::microseconds timeout)
{
    const bool useChannelB = mConfig.channels.at(lime::TRXDir::Tx).size() > 1;
    const bool useTimestamp = meta ? (meta->hasTimestamp) : false;
    const bool flush = meta ? (meta->flags & StreamTxMeta::EndOfBurst) : false;

    Timespec ts = meta->timestamp;
    ts.SetTickRate(ticksPerSample * mConfig.hintSampleRate);

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
            mTx.packetsPool->pop(&mTx.stagingPacket);
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
        ts.AddTicks(consumed * ticksPerSample);

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
uint32_t TRXLooper::StreamTx(
    const lime::complex32f_t* const* samples, uint32_t count, const StreamMeta* meta, std::chrono::microseconds timeout)
{
    return StreamMetaToStreamTxMeta(samples, count, meta, timeout);
}

/// @copydoc TRXLooper::StreamTx()
uint32_t TRXLooper::StreamTx(
    const lime::complex16_t* const* samples, uint32_t count, const StreamMeta* meta, std::chrono::microseconds timeout)
{
    return StreamMetaToStreamTxMeta(samples, count, meta, timeout);
}

/// @copydoc TRXLooper::StreamTx()
uint32_t TRXLooper::StreamTx(
    const lime::complex12_t* const* samples, uint32_t count, const StreamMeta* meta, std::chrono::microseconds timeout)
{
    return StreamMetaToStreamTxMeta(samples, count, meta, timeout);
}

uint32_t TRXLooper::Transmit(const lime::complex32f_t* const* samples, uint32_t count, const StreamTxMeta* meta)
{
    return StreamTxTemplate(samples, count, meta, chrono::microseconds(100000));
}

uint32_t TRXLooper::Transmit(const lime::complex16_t* const* samples, uint32_t count, const StreamTxMeta* meta)
{
    return StreamTxTemplate(samples, count, meta, chrono::microseconds(100000));
}

uint32_t TRXLooper::Transmit(const lime::complex12_t* const* samples, uint32_t count, const StreamTxMeta* meta)
{
    return StreamTxTemplate(samples, count, meta, chrono::microseconds(100000));
}

/// @brief Gets Rx/Tx data transfer statistics.
/// @param rxStats Pointer to rx statistics structure, (Optional, can be NULL)
/// @param txStats Pointer to rx statistics structure, (Optional, can be NULL)
void TRXLooper::StreamStatus(StreamStats* rxStats, StreamStats* txStats)
{
    if (txStats)
    {
        *txStats = mTx.stats;
        // FIFO holds sample packets, report the counts in samples
        if (mTx.fifo)
        {
            const std::size_t txPacketSamples = mTx.packetsToBatch * mTx.samplesInPkt;
            txStats->FIFO = { mTx.fifo->max_size() * txPacketSamples, mTx.fifo->size() * txPacketSamples };
        }
        else
            txStats->FIFO = { 1, 0 };
    }
    if (rxStats)
    {
        *rxStats = mRx.stats;
        if (mRx.fifo)
        {
            const std::size_t rxPacketSamples = mRx.packetsToBatch * mRx.samplesInPkt;
            rxStats->FIFO = { mRx.fifo->max_size() * rxPacketSamples, mRx.fifo->size() * rxPacketSamples };
        }
        else
            rxStats->FIFO = { 1, 0 };
    }
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
