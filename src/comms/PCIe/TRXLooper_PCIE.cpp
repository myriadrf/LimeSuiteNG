#include "TRXLooper_PCIE.h"

#include "AvgRmsCounter.h"
#include "BufferInterleaving.h"
#include "comms/IDMA.h"
#include "DataPacket.h"
#include "FPGA_common.h"
#include "limesuiteng/types.h"
#include "limesuiteng/complex.h"
#include "limesuiteng/LMS7002M.h"
#include "limesuiteng/SDRDevice.h"
#include "limesuiteng/Logger.h"
#include "MemoryPool.h"
#include "TxBufferManager.h"
#include "utilities/DeltaVariable.h"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <memory>
#include <queue>
#include <thread>
#include <vector>

// needed for the hacky workarounds
// TODO: delete
#include "LitePCIe.h"
#include "USBDMA.h"

static constexpr bool showStats{ false };
static constexpr int statsPeriod_ms{ 1000 }; // at 122.88 MHz MIMO, fpga tx pkt counter overflows every 272ms

namespace lime {

static constexpr int64_t ts_to_us(int64_t fs, int64_t ts)
{
    int64_t n = (ts / fs);
    int64_t r = (ts % fs);

    return n * 1000000 + ((r * 1000000) / fs);
}

/// @brief Constructs a new TRXLooper_PCIE object.
/// @param comms The DMA communications interface to use.
/// @param f The FPGA to use in this stream.
/// @param chip The LMS7002M chip to use in this stream.
/// @param moduleIndex The ID of the chip to use.
TRXLooper_PCIE::TRXLooper_PCIE(std::shared_ptr<IDMA> comms, FPGA* f, LMS7002M* chip, uint8_t moduleIndex)
    : TRXLooper(f, chip, moduleIndex)
{
    mRx.packetsToBatch = 1;
    mTx.packetsToBatch = 1;
    mRxArgs.port = comms;
    mTxArgs.port = comms;
}

TRXLooper_PCIE::~TRXLooper_PCIE()
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

OpStatus TRXLooper_PCIE::Setup(const StreamConfig& config)
{
    if (config.channels.at(lime::TRXDir::Rx).size() > 0 && !mRxArgs.port->IsOpen())
        return ReportError(OpStatus::IOFailure, "Rx data port not open"s);
    if (config.channels.at(lime::TRXDir::Tx).size() > 0 && !mTxArgs.port->IsOpen())
        return ReportError(OpStatus::IOFailure, "Tx data port not open"s);

    float combinedSampleRate =
        std::max(config.channels.at(lime::TRXDir::Tx).size(), config.channels.at(lime::TRXDir::Rx).size()) * config.hintSampleRate;
    int batchSize = 7; // should be good enough for most cases
    // for high data rates e.g 16bit ADC/DAC 2x2 MIMO @ 122.88Msps = ~1973 MB/s
    // need to batch as many packets as possible into transfer buffers
    if (combinedSampleRate != 0)
    {
        batchSize = combinedSampleRate / 61.44e6;
        batchSize = std::clamp(batchSize, 1, 4);
    }

    if (config.hintSampleRate)
        mRx.packetsToBatch = 2 * config.hintSampleRate / 30.72e6;
    else
        mRx.packetsToBatch = 4;
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

    mConfig = config;
    if (config.channels.at(lime::TRXDir::Rx).size() > 0)
        RxSetup();
    if (config.channels.at(lime::TRXDir::Tx).size() > 0)
        TxSetup();

    return TRXLooper::Setup(config);
}

void TRXLooper_PCIE::Start()
{
    fpga->WriteRegister(0xFFFF, 1 << chipId);
    TRXLooper::Start();
}

int TRXLooper_PCIE::TxSetup()
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

void TRXLooper_PCIE::TransmitPacketsLoop()
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
    int packetsSent = 0;
    int totalPacketSent = 0;
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
                packetsSent += output.packetCount();
                totalPacketSent += output.packetCount();
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
            packetsSent = 0;
            totalBytesSent = 0;
            mTx.stats.dataRate_Bps = dataRate;
        }
    }
}

void TRXLooper_PCIE::TxTeardown()
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

int TRXLooper_PCIE::RxSetup()
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
void TRXLooper_PCIE::ReceivePacketsLoop()
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

void TRXLooper_PCIE::RxTeardown()
{
    mRxArgs.port->Disable(TRXDir::Rx);
}

/// @copydoc SDRDevice::UploadTxWaveform()
/// @param fpga The FPGA device to use.
/// @param port The PCIe communications port to use.
OpStatus TRXLooper_PCIE::UploadTxWaveform(FPGA* fpga,
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
