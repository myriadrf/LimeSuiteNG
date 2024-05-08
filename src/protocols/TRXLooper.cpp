#include "TRXLooper.h"

#include "AvgRmsCounter.h"
#include "comms/IDMA.h"
#include "FPGA_common.h"
#include "limesuiteng/LMS7002M.h"
#include "limesuiteng/Logger.h"
#include "lms7002m/LMS7002MCSR_Data.h"
#include "LMSBoards.h"
#include "threadHelper.h"
#include "TxBufferManager.h"
#include "utilities/DeltaVariable.h"

#include <cassert>
#include <ciso646>
#include <complex>
#include <queue>

using namespace std::literals::string_literals;

// needed for the hacky workarounds
// TODO: delete
#include "LitePCIe.h"
#include "USBDMA.h"

namespace lime {
using namespace LMS7002MCSR_Data;

using namespace std::chrono;

static constexpr uint16_t defaultSamplesInPkt = 256;

static constexpr bool showStats{ false };
static constexpr int statsPeriod_ms{ 1000 }; // at 122.88 MHz MIMO, fpga tx pkt counter overflows every 272ms

static constexpr int64_t ts_to_us(int64_t fs, int64_t ts)
{
    int64_t n = (ts / fs);
    int64_t r = (ts % fs);

    return n * 1000000 + ((r * 1000000) / fs);
}

/// @brief Constructs a new TRXLooper object.
/// @param comms The DMA communications interface to use.
/// @param f The FPGA to use in this stream.
/// @param chip The LMS7002M chip to use in this stream.
/// @param moduleIndex The ID of the chip to use.
TRXLooper::TRXLooper(std::shared_ptr<IDMA> comms, FPGA* f, LMS7002M* chip, uint8_t moduleIndex)
    : mCallback_logMessage(nullptr)
    , mStreamEnabled(false)
{
    mRx.packetsToBatch = 1;
    mTx.packetsToBatch = 1;
    mTx.samplesInPkt = defaultSamplesInPkt;
    mRx.samplesInPkt = defaultSamplesInPkt;
    const int fifoLen = 512;
    mRx.fifo = new PacketsFIFO<SamplesPacketType*>(fifoLen);
    mTx.fifo = new PacketsFIFO<SamplesPacketType*>(fifoLen);

    mRxArgs.port = comms;
    mTxArgs.port = comms;

    lms = chip, fpga = f;
    chipId = moduleIndex;
    mTimestampOffset = 0;
    mRx.lastTimestamp.store(0, std::memory_order_relaxed);
    mRx.terminate.store(false, std::memory_order_relaxed);
    mTx.terminate.store(false, std::memory_order_relaxed);
}

TRXLooper::~TRXLooper()
{
    Stop();
    mRx.terminate.store(true, std::memory_order_relaxed);
    mTx.terminate.store(true, std::memory_order_relaxed);

    // Thread joining code has to be as high as possible,
    // so that every variable and virtual function is still properly accessible when the thread is still executing.
    // Otherwise, it can cause crashes when the destructor is being called before the thread is fully stopped and
    // it is still trying to access variables, or, even worse, virtual functions of the class.
    if (mTx.thread.joinable())
    {
        mTx.thread.join();
    }

    if (mRx.thread.joinable())
    {
        mRx.thread.join();
    }
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
    if (cfg.channels.at(lime::TRXDir::Rx).size() > 0 && !mRxArgs.port->IsOpen())
        return ReportError(OpStatus::IOFailure, "Rx data port not open"s);
    if (cfg.channels.at(lime::TRXDir::Tx).size() > 0 && !mTxArgs.port->IsOpen())
        return ReportError(OpStatus::IOFailure, "Tx data port not open"s);

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

    mTx.packetsToBatch = 6;
    mRx.packetsToBatch = 6;
    //lime::debug("Batch size %i", batchSize);

    fpga->WriteRegister(0xFFFF, 1 << chipId);
    fpga->StopStreaming();
    uint16_t startAddress = 0x7FE1 + (chipId * 5);
    // reset Tx received/dropped packets counters
    uint32_t addrs[] = { startAddress, startAddress, startAddress };
    uint32_t values[] = { 0, 3, 0 };
    fpga->WriteRegisters(addrs, values, 3);

    mConfig = cfg;
    if (cfg.channels.at(lime::TRXDir::Rx).size() > 0)
        RxSetup();
    if (cfg.channels.at(lime::TRXDir::Tx).size() > 0)
        TxSetup();

    if (mRx.thread.joinable() || mTx.thread.joinable())
        return ReportError(OpStatus::Busy, "Samples streaming already running"s);

    bool needTx = cfg.channels.at(TRXDir::Tx).size() > 0;
    bool needRx = cfg.channels.at(TRXDir::Rx).size() > 0; // always need Rx to know current timestamps, cfg.rxCount > 0;
    //bool needMIMO = cfg.rxCount > 1 || cfg.txCount > 1; // TODO: what if using only B channel, does it need MIMO configuration?
    uint8_t channelEnables = 0;

    for (std::size_t i = 0; i < cfg.channels.at(TRXDir::Rx).size(); ++i)
    {
        if (cfg.channels.at(TRXDir::Rx).at(i) > 1)
        {
            return ReportError(OpStatus::InvalidValue, "Invalid Rx channel, only [0,1] channels supported"s);
        }
        else
        {
            channelEnables |= (1 << cfg.channels.at(TRXDir::Rx).at(i));
        }
    }

    for (std::size_t i = 0; i < cfg.channels.at(TRXDir::Tx).size(); ++i)
    {
        if (cfg.channels.at(TRXDir::Tx).at(i) > 1)
        {
            return ReportError(OpStatus::InvalidValue, "Invalid Tx channel, only [0,1] channels supported"s);
        }
        else
        {
            channelEnables |= (1 << cfg.channels.at(TRXDir::Tx).at(i)); // << 8;
        }
    }

    if ((cfg.linkFormat != DataFormat::I12) && (cfg.linkFormat != DataFormat::I16))
    {
        return ReportError(OpStatus::InvalidValue, "Unsupported stream link format"s);
    }

    mConfig = cfg;

    //configure FPGA on first start, or disable FPGA when not streaming
    if (!needTx && !needRx)
        return OpStatus::Success;

    assert(fpga);
    fpga->WriteRegister(0xFFFF, 1 << chipId);
    fpga->StopStreaming();
    fpga->WriteRegister(0xD, 0); //stop WFM
    mRx.lastTimestamp.store(0, std::memory_order_relaxed);

    // const uint16_t MIMO_EN = needMIMO << 8;
    // const uint16_t TRIQ_PULSE = lms->Get_SPI_Reg_bits(LMS7002MCSR::LML1_TRXIQPULSE) << 7; // 0-OFF, 1-ON
    // const uint16_t DDR_EN = lms->Get_SPI_Reg_bits(LMS7002MCSR::LML1_SISODDR) << 6; // 0-SDR, 1-DDR
    // const uint16_t MODE = 0 << 5; // 0-TRXIQ, 1-JESD207 (not impelemented)
    // const uint16_t smpl_width =
    //     cfg.linkFormat == DataFormat::I12 ? 2 : 0;
    // printf("TRIQ:%i, DDR_EN:%i, MIMO_EN:%i\n", TRIQ_PULSE, DDR_EN, MIMO_EN);
    // const uint16_t reg8 = MIMO_EN | TRIQ_PULSE | DDR_EN | MODE | smpl_width;

    uint16_t mode = 0x0100;
    if (lms->Get_SPI_Reg_bits(LMS7002MCSR::LML1_SISODDR))
        mode = 0x0040;
    else if (lms->Get_SPI_Reg_bits(LMS7002MCSR::LML1_TRXIQPULSE))
        mode = 0x0180;

    const uint16_t smpl_width = cfg.linkFormat == DataFormat::I12 ? 2 : 0;
    fpga->WriteRegister(0x0008, mode | smpl_width);
    fpga->WriteRegister(0x0007, channelEnables);
    fpga->ResetTimestamp();

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

    // Don't just use REALTIME scheduling, or at least be cautious with it.
    // if the thread blocks for too long, Linux can trigger RT throttling
    // which can cause unexpected data packet losses and timing issues.
    // Also need to set policy to default here, because if host process is running
    // with REALTIME policy, these threads would inherit it and exhibit mentioned
    // issues.
    const auto schedulingPolicy = ThreadPolicy::PREEMPTIVE;
    if (needRx)
    {
        mRx.terminate.store(false, std::memory_order_relaxed);
        auto RxLoopFunction = std::bind(&TRXLooper::ReceivePacketsLoop, this);
        mRx.thread = std::thread(RxLoopFunction);
        SetOSThreadPriority(ThreadPriority::HIGHEST, schedulingPolicy, &mRx.thread);
#ifdef __linux__
        pthread_setname_np(mRx.thread.native_handle(), "lime:RxLoop");

        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(2, &cpuset);
        //int rc = pthread_setaffinity_np(mRx.thread.native_handle(), sizeof(cpu_set_t), &cpuset);
        // if (rc != 0) {
        //   printf("Error calling pthread_setaffinity_np: %i\n", rc);
        // }
#endif
    }
    if (needTx)
    {
        mTx.terminate.store(false, std::memory_order_relaxed);
        auto TxLoopFunction = std::bind(&TRXLooper::TransmitPacketsLoop, this);
        mTx.thread = std::thread(TxLoopFunction);
        SetOSThreadPriority(ThreadPriority::HIGHEST, schedulingPolicy, &mTx.thread);
#ifdef __linux__
        pthread_setname_np(mTx.thread.native_handle(), "lime:TxLoop");

        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(3, &cpuset);
        //int rc = pthread_setaffinity_np(mTx.thread.native_handle(), sizeof(cpu_set_t), &cpuset);
        // if (rc != 0) {
        //   printf("Error calling pthread_setaffinity_np: %i\n", rc);
        // }
#endif
    }

    // if (cfg.alignPhase)
    //     TODO: AlignRxRF(true);
    //enable FPGA streaming
    return OpStatus::Success;
}

/// @brief Starts the stream of this looper.
void TRXLooper::Start()
{
    fpga->WriteRegister(0xFFFF, 1 << chipId);

    mRx.fifo->clear();
    mTx.fifo->clear();

    fpga->StartStreaming();

    {
        std::lock_guard<std::mutex> lock(streamMutex);
        mStreamEnabled = true;
        streamActive.notify_all();
    }

    streamClockStart = steady_clock::now();
    //int64_t startPoint = std::chrono::time_point_cast<std::chrono::microseconds>(pcStreamStart).time_since_epoch().count();
    //printf("Stream%i start %lius\n", chipId, startPoint);
    // if (!mConfig.alignPhase)
    //     lms->ResetLogicRegisters();
}

/// @brief Stops the stream and cleans up all the memory.
void TRXLooper::Stop()
{
    if (!mStreamEnabled)
    {
        return;
    }

    mTx.terminate.store(true, std::memory_order_relaxed);
    mRx.terminate.store(true, std::memory_order_relaxed);
    try
    {
        if (mRx.thread.joinable())
            mRx.thread.join();

        if (mTx.thread.joinable())
            mTx.thread.join();
    } catch (...)
    {
        lime::error("Failed to join TRXLooper threads"s);
    }
    fpga->StopStreaming();

    RxTeardown();
    TxTeardown();

    mRx.DeleteMemoryPool();
    mTx.DeleteMemoryPool();

    mStreamEnabled = false;
}

int TRXLooper::RxSetup()
{
    mRx.lastTimestamp.store(0, std::memory_order_relaxed);
    const bool usePoll = mConfig.extraConfig.usePoll;
    const int chCount = std::max(mConfig.channels.at(lime::TRXDir::Rx).size(), mConfig.channels.at(lime::TRXDir::Tx).size());
    const int sampleSize = (mConfig.linkFormat == DataFormat::I16 ? 4 : 3); // sizeof IQ pair

    constexpr std::size_t headerSize{ sizeof(StreamHeader) };

    int requestSamplesInPkt = 4080 / sampleSize / chCount;

    int payloadSize = requestSamplesInPkt * sampleSize * chCount;
    const int samplesInPkt = payloadSize / (sampleSize * chCount);

    uint32_t packetSize = payloadSize + headerSize;

    // TODO: fix workaround
    // only if PCIe device
    if (dynamic_cast<LitePCIe*>(mRxArgs.port.get()) != nullptr)
    {
        // iqSamplesCount must be N*16, or N*8 depending on device BUS width
        const uint32_t iqSamplesCount = (payloadSize / (sampleSize * 2)) & ~0xF; //magic number needed for fpga's FSMs
        payloadSize = iqSamplesCount * sampleSize * 2;
        packetSize = payloadSize + headerSize;

        // Request fpga to provide Rx packets with desired payloadSize
        // Two writes are needed
        fpga->WriteRegister(0xFFFF, 1 << chipId);
        uint32_t requestAddr[] = { 0x0019, 0x000E };
        uint32_t requestData[] = { packetSize, iqSamplesCount };
        fpga->WriteRegisters(requestAddr, requestData, 2);

        if (mConfig.extraConfig.rx.packetsInBatch != 0)
        {
            mRx.packetsToBatch = mConfig.extraConfig.rx.packetsInBatch;
        }
    }

    const auto dmaBufferSize{ mRxArgs.port->GetBufferSize() };

    mRx.packetsToBatch = std::clamp<uint8_t>(mRx.packetsToBatch, 1, dmaBufferSize / packetSize);

    if (mCallback_logMessage)
    {
        char msg[256];
        std::snprintf(msg,
            sizeof(msg),
            "usePoll:%i rxSamplesInPkt:%i rxPacketsInBatch:%i, DMA_ReadSize:%i\n",
            usePoll ? 1 : 0,
            samplesInPkt,
            mRx.packetsToBatch,
            mRx.packetsToBatch * packetSize);
        mCallback_logMessage(LogLevel::Debug, msg);
    }

    std::vector<std::byte*> dmaBuffers(mRxArgs.port->GetBufferCount());
    for (uint32_t i = 0; i < dmaBuffers.size(); ++i)
    {
        dmaBuffers[i] = mRxArgs.port->GetMemoryAddress(TRXDir::Rx) + dmaBufferSize * i;
    }

    mRxArgs.buffers = std::move(dmaBuffers);
    mRxArgs.bufferSize = dmaBufferSize;
    mRxArgs.packetSize = packetSize;
    mRxArgs.packetsToBatch = mRx.packetsToBatch;
    mRxArgs.samplesInPacket = samplesInPkt;

    const std::string name = "MemPool_Rx"s + std::to_string(chipId);
    const int upperAllocationLimit =
        sizeof(complex32f_t) * mRx.packetsToBatch * samplesInPkt * chCount + SamplesPacketType::headerSize;
    mRx.memPool = new MemoryPool(1024, upperAllocationLimit, 8, name);

    // TODO: fix
    // very hacky but does the job somewhy
    if (dynamic_cast<LitePCIe*>(mRxArgs.port.get()) != nullptr)
    {
        // only do this now if it's a PCIe device
        constexpr uint8_t irqPeriod{ 4 };
        const uint32_t readSize = mRxArgs.packetSize * mRxArgs.packetsToBatch;
        mRxArgs.port->RxEnable(readSize, irqPeriod);
    }

    return 0;
}

/** @brief Function dedicated for receiving data samples from board
    @param stream a pointer to an active receiver stream
*/
void TRXLooper::ReceivePacketsLoop()
{
    constexpr int headerSize{ sizeof(StreamHeader) };

    DataConversion conversion;
    conversion.srcFormat = mConfig.linkFormat;
    conversion.destFormat = mConfig.format;
    conversion.channelCount = std::max(mConfig.channels.at(lime::TRXDir::Tx).size(), mConfig.channels.at(lime::TRXDir::Rx).size());

    const int32_t bufferCount = mRxArgs.buffers.size();
    const int32_t readSize = mRxArgs.packetSize * mRxArgs.packetsToBatch;
    const int32_t packetSize = mRxArgs.packetSize;
    const int32_t samplesInPkt = mRxArgs.samplesInPacket;
    const std::vector<std::byte*>& dmaBuffers{ mRxArgs.buffers };
    StreamStats& stats = mRx.stats;
    auto fifo = mRx.fifo;

    const uint8_t outputSampleSize = mConfig.format == DataFormat::F32 ? sizeof(complex32f_t) : sizeof(complex16_t);
    const int32_t outputPktSize = SamplesPacketType::headerSize + mRxArgs.packetsToBatch * samplesInPkt * outputSampleSize;

    DeltaVariable<int32_t> overrun(0);
    DeltaVariable<int32_t> loss(0);

    mRxArgs.cnt = 0;
    mRxArgs.sw = 0;
    mRxArgs.hw = 0;

    // thread ready for work, just wait for stream enable
    {
        std::unique_lock<std::mutex> lk(streamMutex);
        while (!mStreamEnabled && !mRx.terminate.load(std::memory_order_relaxed))
            streamActive.wait_for(lk, std::chrono::milliseconds(100));
        lk.unlock();
    }

    auto t1{ std::chrono::steady_clock::now() };
    auto t2 = t1;

    // TODO: fix
    // very hacky but does the job somewhy
    if (dynamic_cast<USBDMA*>(mRxArgs.port.get()) != nullptr)
    {
        // only run this here if it's a USB device
        constexpr uint8_t irqPeriod{ 4 };
        mRxArgs.port->RxEnable(readSize, irqPeriod);
    }

    int32_t Bps = 0;

    uint32_t lastHwIndex{ 0 };
    int64_t expectedTS = 0;
    SamplesPacketType* outputPkt = nullptr;
    while (mRx.terminate.load(std::memory_order_relaxed) == false)
    {
        IDMA::DMAState dma{ mRxArgs.port->GetState(TRXDir::Rx) };
        if (dma.hardwareIndex != lastHwIndex)
        {
            const uint8_t buffersTransferred{ static_cast<uint8_t>(dma.hardwareIndex - lastHwIndex) };
            const int bytesTransferred = buffersTransferred * readSize;
            assert(bytesTransferred > 0);
            Bps += bytesTransferred;
            stats.bytesTransferred += bytesTransferred;
            lastHwIndex = dma.hardwareIndex;
        }

        // print stats
        t2 = std::chrono::steady_clock::now();
        const auto timePeriod{ std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() };
        if (timePeriod >= statsPeriod_ms)
        {
            t1 = t2;
            double dataRateBps = 1000.0 * Bps / timePeriod;
            stats.dataRate_Bps = dataRateBps;
            mRx.stats.dataRate_Bps = dataRateBps;

            char msg[512];
            std::snprintf(msg,
                sizeof(msg) - 1,
                "Rx: %3.3f MB/s | TS:%li pkt:%li o:%i(%+i) l:%i(%+i) dma:%u/%u(%u) swFIFO:%li",
                stats.dataRate_Bps / 1e6,
                stats.timestamp,
                stats.packets,
                overrun.value(),
                overrun.delta(),
                loss.value(),
                loss.delta(),
                dma.softwareIndex,
                dma.hardwareIndex,
                dma.hardwareIndex - dma.softwareIndex,
                mRx.fifo->size());
            if (showStats)
                lime::info("%s", msg);
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

        const uint32_t buffersAvailable{ dma.hardwareIndex - dma.softwareIndex };

        // process received data
        bool reportProblems = false;

        if (buffersAvailable == 0)
        {
            if (mConfig.extraConfig.usePoll && !mRxArgs.port->Wait(TRXDir::Rx))
            {
                std::this_thread::yield();
            }

            continue;
        }

        const uint32_t currentBufferIndex{ dma.softwareIndex % bufferCount };
        mRxArgs.port->CacheFlush(TRXDir::Rx, DataTransferDirection::DeviceToHost, currentBufferIndex);
        const std::byte* buffer{ dmaBuffers.at(currentBufferIndex) };
        const FPGA_RxDataPacket* pkt = reinterpret_cast<const FPGA_RxDataPacket*>(buffer);

        if (outputPkt == nullptr)
        {
            outputPkt = SamplesPacketType::ConstructSamplesPacket(
                mRx.memPool->Allocate(outputPktSize), samplesInPkt * mRxArgs.packetsToBatch, outputSampleSize);
        }

        assert(outputPkt != nullptr);

        outputPkt->timestamp = pkt->counter;

        const int srcPktCount = mRxArgs.packetsToBatch;
        for (int i = 0; i < srcPktCount; ++i)
        {
            pkt = reinterpret_cast<const FPGA_RxDataPacket*>(&buffer[packetSize * i]);
            if (pkt->counter - expectedTS != 0)
            {
                //lime::info("Loss: pkt:%i exp: %li, got: %li, diff: %li", stats.packets+i, expectedTS, pkt->counter, pkt->counter-expectedTS);
                ++stats.loss;
                loss.add(1);
            }
            if (pkt->txWasDropped())
                ++mTx.stats.loss;

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

        mRxArgs.port->CacheFlush(TRXDir::Rx, DataTransferDirection::HostToDevice, currentBufferIndex);

        ++dma.softwareIndex;
        mRxArgs.port->SetState(TRXDir::Rx, dma);

        mRxArgs.sw = dma.softwareIndex;
        mRxArgs.hw = dma.hardwareIndex;
        ++mRxArgs.cnt;
        // one callback for the entire batch
        if (reportProblems && mConfig.statusCallback)
            mConfig.statusCallback(false, &stats, mConfig.userData);
        std::this_thread::yield();
    }

    if (mCallback_logMessage)
    {
        char msg[256];
        std::snprintf(msg, sizeof(msg), "Rx%i: packetsIn: %li", chipId, stats.packets);
        mCallback_logMessage(LogLevel::Debug, msg);
    }
}

void TRXLooper::RxTeardown()
{
    mRxArgs.port->Disable(TRXDir::Rx);
}

template<class T> uint32_t TRXLooper::StreamRxTemplate(T* const* dest, uint32_t count, StreamMeta* meta)
{
    bool timestampSet = false;
    uint32_t samplesProduced = 0;
    const bool useChannelB = mConfig.channels.at(TRXDir::Rx).size() > 1;

    bool firstIteration = true;

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

/// @brief Reveives samples from this specific stream.
/// @param samples The buffer to put the received samples in.
/// @param count The amount of samples to reveive.
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

int TRXLooper::TxSetup()
{
    mTx.lastTimestamp.store(0, std::memory_order_relaxed);
    const int chCount = std::max(mConfig.channels.at(lime::TRXDir::Rx).size(), mConfig.channels.at(lime::TRXDir::Tx).size());
    const int sampleSize = (mConfig.linkFormat == DataFormat::I16 ? 4 : 3); // sizeof IQ pair

    int samplesInPkt = 256; //(mConfig.linkFormat == DataFormat::I16 ? 1020 : 1360) / chCount;
    const int packetSize = sizeof(StreamHeader) + samplesInPkt * sampleSize * chCount;

    if (mConfig.extraConfig.tx.samplesInPacket != 0)
    {
        samplesInPkt = mConfig.extraConfig.tx.samplesInPacket;
        lime::debug("Tx samples overide %i", samplesInPkt);
    }

    mTx.samplesInPkt = samplesInPkt;

    if (mConfig.extraConfig.tx.packetsInBatch != 0)
    {
        mTx.packetsToBatch = mConfig.extraConfig.tx.packetsInBatch;
    }

    const auto dmaBufferSize{ mTxArgs.port->GetBufferSize() };
    mTx.packetsToBatch = std::clamp<uint8_t>(mTx.packetsToBatch, 1, dmaBufferSize / packetSize);

    std::vector<std::byte*> dmaBuffers(mTxArgs.port->GetBufferCount());
    for (uint32_t i = 0; i < dmaBuffers.size(); ++i)
    {
        dmaBuffers[i] = mTxArgs.port->GetMemoryAddress(TRXDir::Tx) + dmaBufferSize * i;
    }

    mTxArgs.buffers = std::move(dmaBuffers);
    mTxArgs.bufferSize = dmaBufferSize;
    mTxArgs.packetSize = packetSize;
    mTxArgs.packetsToBatch = mTx.packetsToBatch;
    mTxArgs.samplesInPacket = samplesInPkt;

    float bufferTimeDuration = samplesInPkt * mTx.packetsToBatch / mConfig.hintSampleRate;
    if (mCallback_logMessage)
    {
        char msg[256];
        std::snprintf(msg,
            sizeof(msg),
            "Stream%i samplesInTxPkt:%i maxTxPktInBatch:%i, batchSizeInTime:%gus",
            chipId,
            samplesInPkt,
            mTx.packetsToBatch,
            bufferTimeDuration * 1e6);
        mCallback_logMessage(LogLevel::Debug, msg);
    }

    const std::string name = "MemPool_Tx"s + std::to_string(chipId);
    const int upperAllocationLimit =
        65536; //sizeof(complex32f_t) * mTx.packetsToBatch * samplesInPkt * chCount + SamplesPacketType::headerSize;
    mTx.memPool = new MemoryPool(1024, upperAllocationLimit, 4096, name);
    return 0;
}

void TRXLooper::TransmitPacketsLoop()
{
    const bool mimo = std::max(mConfig.channels.at(lime::TRXDir::Tx).size(), mConfig.channels.at(lime::TRXDir::Rx).size()) > 1;
    const bool compressed = mConfig.linkFormat == DataFormat::I12;
    constexpr int irqPeriod{ 4 }; // Interrupt request period

    const int32_t bufferCount = mTxArgs.buffers.size();
    const uint32_t overflowLimit = bufferCount - irqPeriod;
    const std::vector<std::byte*>& dmaBuffers{ mTxArgs.buffers };

    StreamStats& stats = mTx.stats;

    auto fifo = mTx.fifo;

    int64_t totalBytesSent = 0; //for data rate calculation
    int64_t lastTS = 0;

    struct PendingWrite {
        uint32_t id;
        std::byte* data;
        int32_t offset;
        int32_t size;
    };
    std::queue<PendingWrite> pendingWrites;

    uint32_t stagingBufferIndex = 0;
    SamplesPacketType* srcPkt = nullptr;

    TxBufferManager<SamplesPacketType> output(mimo, compressed, mTxArgs.samplesInPacket, mTxArgs.packetsToBatch, mConfig.format);

    mTxArgs.port->CacheFlush(TRXDir::Tx, DataTransferDirection::DeviceToHost, 0);
    output.Reset(dmaBuffers[0], mTxArgs.bufferSize);

    bool outputReady = false;

    AvgRmsCounter txTSAdvance;
    AvgRmsCounter transferSize;

    // Initialize DMA
    mTxArgs.port->TxEnable();
    // thread ready for work, just wait for stream enable
    {
        std::unique_lock<std::mutex> lk(streamMutex);
        while (!mStreamEnabled && !mTx.terminate.load(std::memory_order_relaxed))
            streamActive.wait_for(lk, std::chrono::milliseconds(100));
        lk.unlock();
    }

    auto t1{ std::chrono::steady_clock::now() };
    auto t2 = t1;

    DeltaVariable<int32_t> underrun(0);
    DeltaVariable<int32_t> loss(0);

    while (mTx.terminate.load(std::memory_order_relaxed) == false)
    {
        // check if DMA transfers have completed
        IDMA::DMAState state{ mTxArgs.port->GetState(TRXDir::Tx) };

        // process pending transactions
        while (!pendingWrites.empty() && !mTx.terminate.load(std::memory_order_relaxed))
        {
            PendingWrite& dataBlock = pendingWrites.front();
            if (dataBlock.id - state.hardwareIndex <= 255)
            {
                break;
            }

            totalBytesSent += dataBlock.size;
            pendingWrites.pop();
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
            if (srcPkt->useTimestamp && mConfig.channels.at(lime::TRXDir::Rx).size() > 0)
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
                    break;
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
                mTxArgs.port->CacheFlush(TRXDir::Tx, DataTransferDirection::HostToDevice, stagingBufferIndex % bufferCount);
                break;
            }
        }
        const uint32_t pendingBuffers{ stagingBufferIndex - state.hardwareIndex };
        bool canSend = pendingBuffers < overflowLimit;
        if (!canSend)
        {
            if (mConfig.extraConfig.usePoll)
            {
                canSend = mTxArgs.port->Wait(TRXDir::Tx);
            }
            else
            {
                std::this_thread::yield();
            }
        }
        // send output buffer if possible
        if (outputReady && canSend)
        {
            PendingWrite wrInfo;
            wrInfo.id = stagingBufferIndex;
            wrInfo.offset = 0;
            wrInfo.size = output.size();
            wrInfo.data = output.data();
            // write or schedule write
            state.softwareIndex = stagingBufferIndex;
            state.bufferSize = wrInfo.size;
            state.generateIRQ = (wrInfo.id % irqPeriod) == 0;

            StreamHeader* pkt = reinterpret_cast<StreamHeader*>(output.data());
            lastTS = pkt->counter;
            if (mConfig.channels.at(lime::TRXDir::Rx).size() > 0) // Rx is needed for current timestamp
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

            // DMA memory is write only, to read from the buffer will trigger Bus errors
            const int ret{ mTxArgs.port->SetState(TRXDir::Tx, state) };
            if (ret)
            {
                if (errno == EINVAL)
                {
                    lime::error("Failed to submit dma write (%i) %s", errno, strerror(errno));
                    return;
                }
                mTxArgs.port->Wait(TRXDir::Tx);
                ++stats.overrun;
            }
            else
            {
                //lime::debug("Sent sw: %li hw: %li, diff: %li", stagingBufferIndex, reader.hw_count, stagingBufferIndex-reader.hw_count);
                outputReady = false;
                pendingWrites.push(wrInfo);
                transferSize.Add(wrInfo.size);
                ++stagingBufferIndex;
                stagingBufferIndex &= 0xFFFF;
                stats.timestamp = lastTS;
                stats.bytesTransferred += wrInfo.size;
                mTxArgs.port->CacheFlush(TRXDir::Tx, DataTransferDirection::DeviceToHost, stagingBufferIndex % bufferCount);
                output.Reset(dmaBuffers[stagingBufferIndex % bufferCount], mTxArgs.bufferSize);
            }
        }

        t2 = std::chrono::steady_clock::now();
        const auto timePeriod{ std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() };
        if (timePeriod >= statsPeriod_ms || mTx.terminate.load(std::memory_order_relaxed))
        {
            t1 = t2;

            state = mTxArgs.port->GetState(TRXDir::Tx);
            double dataRate = 1000.0 * totalBytesSent / timePeriod;
            double avgTransferSize = 0, rmsTransferSize = 0;
            transferSize.GetResult(avgTransferSize, rmsTransferSize);

            double avgTxAdvance = 0, rmsTxAdvance = 0;
            txTSAdvance.GetResult(avgTxAdvance, rmsTxAdvance);
            loss.set(stats.loss);
            if (showStats || mCallback_logMessage)
            {
                char msg[512];
                std::snprintf(msg,
                    sizeof(msg) - 1,
                    "Tx: %3.3f MB/s | TS:%li pkt:%li o:%i shw:%u/%u(%+i) u:%i(%+i) l:%i(%+i) tsAdvance:%+.0f/%+.0f/%+.0f%s, "
                    "f:%li",
                    dataRate / 1000000.0,
                    lastTS,
                    stats.packets,
                    stats.overrun,
                    state.softwareIndex,
                    state.hardwareIndex,
                    state.softwareIndex - state.hardwareIndex,
                    underrun.value(),
                    underrun.delta(),
                    loss.value(),
                    loss.delta(),
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
            mTx.stats.dataRate_Bps = dataRate;
        }
    }
}

void TRXLooper::TxTeardown()
{
    mTxArgs.port->Disable(TRXDir::Tx);

    uint16_t addr = 0x7FE1 + chipId * 5;
    //fpga->WriteRegister(addr, 0);
    uint32_t addrs[] = { addr + 1u, addr + 2u, addr + 3u, addr + 4u };
    uint32_t values[4];
    fpga->ReadRegisters(addrs, values, 4);
    const uint32_t fpgaTxPktIngressCount = (values[0] << 16) | values[1];
    const uint32_t fpgaTxPktDropCounter = (values[2] << 16) | values[3];
    if (mCallback_logMessage)
    {
        char msg[256];
        std::snprintf(msg,
            sizeof(msg),
            "Tx Loop totals: packets sent: %li (0x%08lX) , FPGA packet counter: %i (0x%08X), diff: %li, FPGA tx drops: %i",
            mTx.stats.packets,
            mTx.stats.packets,
            fpgaTxPktIngressCount,
            fpgaTxPktIngressCount,
            (mTx.stats.packets & 0xFFFFFFFF) - fpgaTxPktIngressCount,
            fpgaTxPktDropCounter);
        mCallback_logMessage(LogLevel::Debug, msg);
    }
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
        stats.FIFO = { mTx.fifo->max_size(), mTx.fifo->size() };
    }
    else
    {
        stats = mRx.stats;
        stats.FIFO = { mRx.fifo->max_size(), mRx.fifo->size() };
    }

    return stats;
}

/// @copydoc SDRDevice::UploadTxWaveform()
/// @param fpga The FPGA device to use.
/// @param port The PCIe communications port to use.
OpStatus TRXLooper::UploadTxWaveform(FPGA* fpga,
    std::shared_ptr<IDMA> port,
    const lime::StreamConfig& config,
    uint8_t moduleIndex,
    const void** samples,
    uint32_t count)
{
    const int samplesInPkt = 256;
    const bool useChannelB = config.channels.at(lime::TRXDir::Tx).size() > 1;
    const bool mimo = config.channels.at(lime::TRXDir::Tx).size() == 2;
    const bool compressed = config.linkFormat == DataFormat::I12;
    fpga->WriteRegister(0xFFFF, 1 << moduleIndex);
    fpga->WriteRegister(0x000C, mimo ? 0x3 : 0x1); //channels 0,1
    if (config.linkFormat == DataFormat::I16)
        fpga->WriteRegister(0x000E, 0x0); //16bit samples
    else
        fpga->WriteRegister(0x000E, 0x2); //12bit samples

    fpga->WriteRegister(0x000D, 0x4); // WFM_LOAD

    std::vector<std::byte*> dmaBuffers(port->GetBufferCount());
    for (uint32_t i = 0; i < dmaBuffers.size(); ++i)
    {
        dmaBuffers[i] = port->GetMemoryAddress(TRXDir::Tx) + port->GetBufferSize() * i;
    }

    port->TxEnable();

    uint32_t samplesRemaining = count;
    uint8_t dmaIndex = 0;

    while (samplesRemaining > 0)
    {
        IDMA::DMAState state{ port->GetState(TRXDir::Tx) };
        port->Wait(TRXDir::Tx); // block until there is a free DMA buffer

        int samplesToSend = samplesRemaining > samplesInPkt ? samplesInPkt : samplesRemaining;
        int samplesDataSize = 0;

        port->CacheFlush(TRXDir::Tx, DataTransferDirection::DeviceToHost, dmaIndex);
        FPGA_TxDataPacket* pkt = reinterpret_cast<FPGA_TxDataPacket*>(dmaBuffers[dmaIndex]);
        pkt->counter = 0;
        pkt->reserved[0] = 0;

        if (config.format == DataFormat::I16)
        {
            const lime::complex16_t* src[2] = { &static_cast<const lime::complex16_t*>(samples[0])[count - samplesRemaining],
                useChannelB ? &static_cast<const lime::complex16_t*>(samples[1])[count - samplesRemaining] : nullptr };
            samplesDataSize = FPGA::Samples2FPGAPacketPayload(src, samplesToSend, mimo, compressed, pkt->data);
        }
        else if (config.format == DataFormat::F32)
        {
            const lime::complex32f_t* src[2] = { &static_cast<const lime::complex32f_t*>(samples[0])[count - samplesRemaining],
                useChannelB ? &static_cast<const lime::complex32f_t*>(samples[1])[count - samplesRemaining] : nullptr };
            samplesDataSize = FPGA::Samples2FPGAPacketPayloadFloat(src, samplesToSend, mimo, compressed, pkt->data);
        }

        int payloadSize = (samplesDataSize / 4) * 4;
        if (samplesDataSize % 4 != 0)
            lime::warning("Packet samples count not multiple of 4"s);
        pkt->reserved[2] = (payloadSize >> 8) & 0xFF; //WFM loading
        pkt->reserved[1] = payloadSize & 0xFF; //WFM loading
        pkt->reserved[0] = 0x1 << 5; //WFM loading

        port->CacheFlush(TRXDir::Tx, DataTransferDirection::HostToDevice, dmaIndex);

        state.softwareIndex = dmaIndex;
        state.bufferSize = 16 + payloadSize;
        state.generateIRQ = (dmaIndex % 4) == 0;

        // DMA memory is write only, to read from the buffer will trigger Bus errors
        const int ret{ port->SetState(TRXDir::Tx, state) };
        if (ret && errno == EINVAL)
        {
            port->Disable(TRXDir::Tx);
            return ReportError(OpStatus::IOFailure, "Failed to submit dma write (%i) %s", errno, strerror(errno));
        }

        samplesRemaining -= samplesToSend;
        dmaIndex = (dmaIndex + 1) % port->GetBufferCount();
    }

    /*Give some time to load samples to FPGA*/
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    port->Disable(TRXDir::Tx);

    fpga->WriteRegister(0x000D, 0); // WFM_LOAD off
    if (samplesRemaining != 0)
        return ReportError(OpStatus::Error, "Failed to upload waveform"s);

    return OpStatus::Success;
}

} // namespace lime
