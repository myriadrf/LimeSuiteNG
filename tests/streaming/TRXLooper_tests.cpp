#include <gtest/gtest.h>

#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "FPGA/FPGA_common.h"
#include "comms/IDMA.h"
#include "comms/SPI/ISPI.h"
#include "limesuiteng/LMS7002M.h"
#include "limesuiteng/OpStatus.h"
#include "limesuiteng/StreamConfig.h"
#include "streaming/TRXLooper.h"

using namespace lime;

namespace {

// Answers all SPI traffic with zeroes, enough for TRXLooper::Setup to pass
class SPIStub : public ISPI
{
  public:
    OpStatus Transact(const uint32_t* MOSI, uint32_t* MISO, uint32_t count) override
    {
        if (MISO != nullptr)
            std::memset(MISO, 0, count * sizeof(uint32_t));
        return OpStatus::Success;
    }
};

// Provides fake DMA buffers, all operations succeed without a device
class DMAStub : public IDMA
{
  public:
    DMAStub()
        : memory(bufferCount * bufferSize)
    {
    }
    OpStatus Initialize() override { return OpStatus::Success; }
    OpStatus Enable(bool enabled) override { return OpStatus::Success; }
    OpStatus EnableContinuous(bool enabled, uint32_t maxTransferSize, uint8_t irqPeriod) override { return OpStatus::Success; }
    State GetCounters() override { return { 0 }; }
    OpStatus SubmitRequest(uint64_t index, uint32_t bytesCount, DataTransferDirection dir, bool irq) override
    {
        return OpStatus::Success;
    }
    OpStatus Wait() override { return OpStatus::Success; }
    void BufferOwnership(uint16_t index, DataTransferDirection dir) override {}
    std::vector<Buffer> GetBuffers() const override
    {
        std::vector<Buffer> buffers;
        for (uint32_t i = 0; i < bufferCount; ++i)
            buffers.push_back({ const_cast<uint8_t*>(memory.data()) + i * bufferSize, bufferSize });
        return buffers;
    }
    std::string GetName() const override { return "DMAStub"; }

  private:
    static constexpr uint32_t bufferCount = 8;
    static constexpr uint32_t bufferSize = 8192;
    std::vector<uint8_t> memory;
};

StreamConfig MakeConfig(TRXDir dir, uint32_t requestedFIFOSize)
{
    StreamConfig cfg;
    cfg.channels.at(dir).push_back(0);
    cfg.format = DataFormat::I16;
    cfg.linkFormat = DataFormat::I16;
    cfg.hintSampleRate = 10e6;
    cfg.bufferSize = requestedFIFOSize;
    return cfg;
}

} // namespace

namespace lime::testing {

TEST(TRXLooperFIFO, TxFIFOSizeHonorsRequest)
{
    auto spi = std::make_shared<SPIStub>();
    FPGA fpga(spi, spi);
    LMS7002M lms(spi);
    TRXLooper looper(std::make_shared<DMAStub>(), std::make_shared<DMAStub>(), &fpga, &lms, 0);

    constexpr uint32_t requestedSamples = 129600;
    ASSERT_EQ(looper.Setup(MakeConfig(TRXDir::Tx, requestedSamples)), OpStatus::Success);

    StreamStats stats{};
    looper.StreamStatus(nullptr, &stats);
    EXPECT_GE(stats.FIFO.totalCount, requestedSamples);
    // the size can be rounded up to whole packets, but not more
    EXPECT_LE(stats.FIFO.totalCount, requestedSamples + 8192);
}

TEST(TRXLooperFIFO, RxFIFOSizeHonorsRequest)
{
    auto spi = std::make_shared<SPIStub>();
    FPGA fpga(spi, spi);
    LMS7002M lms(spi);
    TRXLooper looper(std::make_shared<DMAStub>(), std::make_shared<DMAStub>(), &fpga, &lms, 0);

    constexpr uint32_t requestedSamples = 129600;
    ASSERT_EQ(looper.Setup(MakeConfig(TRXDir::Rx, requestedSamples)), OpStatus::Success);

    StreamStats stats{};
    looper.StreamStatus(&stats, nullptr);
    EXPECT_GE(stats.FIFO.totalCount, requestedSamples);
    EXPECT_LE(stats.FIFO.totalCount, requestedSamples + 8192);
}

TEST(TRXLooperFIFO, TxAcceptsMoreThan65535SamplesInOneCall)
{
    auto spi = std::make_shared<SPIStub>();
    FPGA fpga(spi, spi);
    LMS7002M lms(spi);
    TRXLooper looper(std::make_shared<DMAStub>(), std::make_shared<DMAStub>(), &fpga, &lms, 0);

    // reported in issue 125: transmits above 65535 samples per call failed
    constexpr uint32_t samplesCount = 500000;
    StreamConfig cfg = MakeConfig(TRXDir::Tx, samplesCount + 8192);
    cfg.linkFormat = DataFormat::I12; // the default legacy API link format
    ASSERT_EQ(looper.Setup(cfg), OpStatus::Success);

    std::vector<complex16_t> samples(samplesCount);
    const complex16_t* buffers[1] = { samples.data() };
    StreamMeta meta{};
    meta.flushPartialPacket = true;

    const uint32_t accepted = looper.StreamTx(buffers, samplesCount, &meta, std::chrono::microseconds(2000000));
    EXPECT_EQ(accepted, samplesCount);
}

} // namespace lime::testing
