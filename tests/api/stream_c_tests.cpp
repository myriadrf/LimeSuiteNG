// Unit tests for the C streaming entry points (lime_stream_create / start /
// stop / recv / send / destroy), driven through stub SDRDevice and RFStream
// implementations. Pure library logic, no hardware (issue #187).

#include <gtest/gtest.h>

#include "limesuiteng/rfstream.h"
#include "limesuiteng/sdrdevice.h"

#include "limesuiteng/SDRDevice.hpp"
#include "limesuiteng/SDRDescriptor.hpp"
#include "limesuiteng/RFStream.hpp"
#include "limesuiteng/StreamConfig.h"
#include "limesuiteng/StreamMeta.h"

#include <cstring>
#include <memory>
#include <vector>

using namespace lime;

namespace {

// Records how the C wrapper drives the RFStream interface.
struct StreamCalls {
    bool started = false;
    bool stopped = false;
    char recvFormat = 0; // f = F32, s = I16, t = I12
    char sendFormat = 0;
    uint32_t count = 0;
    StreamTxMeta txMeta{};
};

// Overriding the deprecated stream virtuals is unavoidable for a concrete subclass;
// silence the resulting -Wdeprecated-declarations noise for the stubs only.
#if defined(__GNUC__)
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

class StubRFStream : public RFStream
{
  public:
    StubRFStream(StreamCalls* calls, const StreamConfig& cfg)
        : calls(calls)
        , config(cfg)
    {
    }

    uint64_t GetHardwareTimestamp() const override { return 0; }
    OpStatus Setup(const StreamConfig&) override { return OpStatus::Success; }
    const StreamConfig& GetConfig() const override { return config; }
    OpStatus Start() override
    {
        calls->started = true;
        return OpStatus::Success;
    }
    OpStatus StageStart() override { return OpStatus::Success; }
    void Stop() override { calls->stopped = true; }
    void Teardown() override {}

    uint32_t StreamRx(complex32f_t* const*, uint32_t, StreamMeta*, std::chrono::microseconds) override { return 0; }
    uint32_t StreamRx(complex16_t* const*, uint32_t, StreamMeta*, std::chrono::microseconds) override { return 0; }
    uint32_t StreamRx(complex12_t* const*, uint32_t, StreamMeta*, std::chrono::microseconds) override { return 0; }
    uint32_t StreamTx(const complex32f_t* const*, uint32_t, const StreamMeta*, std::chrono::microseconds) override { return 0; }
    uint32_t StreamTx(const complex16_t* const*, uint32_t, const StreamMeta*, std::chrono::microseconds) override { return 0; }
    uint32_t StreamTx(const complex12_t* const*, uint32_t, const StreamMeta*, std::chrono::microseconds) override { return 0; }
    void StreamStatus(StreamStats*, StreamStats*) override {}

    uint32_t Receive(complex32f_t* const*, uint32_t count, StreamRxMeta* meta, std::chrono::microseconds) override
    {
        return RecordRecv('f', count, meta);
    }
    uint32_t Receive(complex16_t* const*, uint32_t count, StreamRxMeta* meta, std::chrono::microseconds) override
    {
        return RecordRecv('s', count, meta);
    }
    uint32_t Receive(complex12_t* const*, uint32_t count, StreamRxMeta* meta, std::chrono::microseconds) override
    {
        return RecordRecv('t', count, meta);
    }
    uint32_t Transmit(const complex32f_t* const*, uint32_t count, const StreamTxMeta* meta, std::chrono::microseconds) override
    {
        return RecordSend('f', count, meta);
    }
    uint32_t Transmit(const complex16_t* const*, uint32_t count, const StreamTxMeta* meta, std::chrono::microseconds) override
    {
        return RecordSend('s', count, meta);
    }
    uint32_t Transmit(const complex12_t* const*, uint32_t count, const StreamTxMeta* meta, std::chrono::microseconds) override
    {
        return RecordSend('t', count, meta);
    }

  private:
    uint32_t RecordRecv(char format, uint32_t count, StreamRxMeta* meta)
    {
        calls->recvFormat = format;
        calls->count = count;
        if (meta != nullptr)
        {
            meta->timestamp = Timespec(static_cast<int64_t>(1234));
            meta->hasTimestamp = true;
        }
        return count;
    }

    uint32_t RecordSend(char format, uint32_t count, const StreamTxMeta* meta)
    {
        calls->sendFormat = format;
        calls->count = count;
        if (meta != nullptr)
            calls->txMeta = *meta;
        return count;
    }

    StreamCalls* calls;
    StreamConfig config;
};

// A minimal SDRDevice: StreamCreate captures the configuration the C wrapper
// built and serves the stub stream, every other pure virtual is trivial.
class StubStreamDevice : public SDRDevice
{
  public:
    StreamCalls calls;
    StreamConfig lastConfig;
    uint8_t lastModuleIndex = 255;
    bool refuseCreate = false;

    std::unique_ptr<RFStream> StreamCreate(const StreamConfig& config, uint8_t moduleIndex) override
    {
        if (refuseCreate)
            return nullptr;
        lastConfig = config;
        lastModuleIndex = moduleIndex;
        return std::make_unique<StubRFStream>(&calls, config);
    }

    // --- remaining pure virtuals: trivial stubs ---
    const SDRDescriptor& GetDescriptor() const override { return desc; }
    uint8_t GetAntenna(uint8_t, TRXDir, uint8_t) override { return 0; }
    OpStatus SetAntenna(uint8_t, TRXDir, uint8_t, uint8_t) override { return OpStatus::Success; }
    OpStatus SetFrequency(uint8_t, TRXDir, uint8_t, double) override { return OpStatus::Success; }
    OpStatus EnableChannel(uint8_t, TRXDir, uint8_t, bool) override { return OpStatus::Success; }
    OpStatus Configure(const SDRConfig&, uint8_t) override { return OpStatus::Success; }
    OpStatus Init() override { return OpStatus::Success; }
    OpStatus Reset() override { return OpStatus::Success; }
    OpStatus GetGPSLock(GPS_Lock*) override { return OpStatus::Success; }
    double GetClockFreq(uint8_t, uint8_t) override { return 0.0; }
    OpStatus SetClockFreq(uint8_t, double, uint8_t) override { return OpStatus::Success; }
    double GetFrequency(uint8_t, TRXDir, uint8_t) override { return 0.0; }
    double GetNCOFrequency(uint8_t, TRXDir, uint8_t, uint8_t, double&) override { return 0.0; }
    OpStatus SetNCOFrequency(uint8_t, TRXDir, uint8_t, uint8_t, double, double) override { return OpStatus::Success; }
    double GetNCOOffset(uint8_t, TRXDir, uint8_t) override { return 0.0; }
    int GetNCOIndex(uint8_t, TRXDir, uint8_t) override { return 0; }
    OpStatus SetNCOIndex(uint8_t, TRXDir, uint8_t, uint8_t, bool) override { return OpStatus::Success; }
    double GetSampleRate(uint8_t, TRXDir, uint8_t, uint32_t*) override { return 0.0; }
    OpStatus SetSampleRate(uint8_t, TRXDir, uint8_t, double, uint8_t) override { return OpStatus::Success; }
    OpStatus GetGain(uint8_t, TRXDir, uint8_t, eGainTypes, double&) override { return OpStatus::Success; }
    OpStatus SetGain(uint8_t, TRXDir, uint8_t, eGainTypes, double) override { return OpStatus::Success; }
    double GetLowPassFilter(uint8_t, TRXDir, uint8_t) override { return 0.0; }
    OpStatus SetLowPassFilter(uint8_t, TRXDir, uint8_t, double) override { return OpStatus::Success; }
    ChannelConfig::Direction::TestSignal GetTestSignal(uint8_t, TRXDir, uint8_t) override { return {}; }
    OpStatus SetTestSignal(uint8_t, TRXDir, uint8_t, ChannelConfig::Direction::TestSignal, int16_t, int16_t) override
    {
        return OpStatus::Success;
    }
    bool GetDCOffsetMode(uint8_t, TRXDir, uint8_t) override { return false; }
    OpStatus SetDCOffsetMode(uint8_t, TRXDir, uint8_t, bool) override { return OpStatus::Success; }
    complex64f_t GetDCOffset(uint8_t, TRXDir, uint8_t) override { return {}; }
    OpStatus SetDCOffset(uint8_t, TRXDir, uint8_t, const complex64f_t&) override { return OpStatus::Success; }
    complex64f_t GetIQBalance(uint8_t, TRXDir, uint8_t) override { return {}; }
    OpStatus SetIQBalance(uint8_t, TRXDir, uint8_t, const complex64f_t&) override { return OpStatus::Success; }
    bool GetCGENLocked(uint8_t) override { return false; }
    double GetTemperature(uint8_t) override { return 0.0; }
    bool GetSXLocked(uint8_t, TRXDir) override { return false; }
    unsigned int ReadRegister(uint8_t, unsigned int, bool) override { return 0; }
    OpStatus WriteRegister(uint8_t, unsigned int, unsigned int, bool) override { return OpStatus::Success; }
    OpStatus LoadConfig(uint8_t, const std::string&) override { return OpStatus::Success; }
    OpStatus SaveConfig(uint8_t, const std::string&) override { return OpStatus::Success; }
    uint16_t GetParameter(uint8_t, uint8_t, const std::string&) override { return 0; }
    OpStatus SetParameter(uint8_t, uint8_t, const std::string&, uint16_t) override { return OpStatus::Success; }
    uint16_t GetParameter(uint8_t, uint8_t, uint16_t, uint8_t, uint8_t) override { return 0; }
    OpStatus SetParameter(uint8_t, uint8_t, uint16_t, uint8_t, uint8_t, uint16_t) override { return OpStatus::Success; }
    OpStatus Calibrate(uint8_t, TRXDir, uint8_t, double) override { return OpStatus::Success; }
    OpStatus ConfigureGFIR(uint8_t, TRXDir, uint8_t, ChannelConfig::Direction::GFIRFilter) override { return OpStatus::Success; }
    std::vector<double> GetGFIRCoefficients(uint8_t, TRXDir, uint8_t, uint8_t) override { return {}; }
    OpStatus SetGFIRCoefficients(uint8_t, TRXDir, uint8_t, uint8_t, std::vector<double>) override { return OpStatus::Success; }
    OpStatus SetGFIR(uint8_t, TRXDir, uint8_t, uint8_t, bool) override { return OpStatus::Success; }
    OpStatus Synchronize(bool) override { return OpStatus::Success; }
    void EnableCache(bool) override {}
    uint64_t GetHardwareTimestamp(uint8_t) override { return 0; }
    OpStatus SetHardwareTimestamp(uint8_t, const uint64_t) override { return OpStatus::Success; }
    OpStatus StreamSetup(const StreamConfig&, uint8_t) override { return OpStatus::Success; }
    void StreamStart(uint8_t) override {}
    void StreamStop(uint8_t) override {}
    void StreamDestroy(uint8_t) override {}
    uint32_t StreamRx(uint8_t, complex32f_t* const*, uint32_t, StreamMeta*, std::chrono::microseconds) override { return 0; }
    uint32_t StreamRx(uint8_t, complex16_t* const*, uint32_t, StreamMeta*, std::chrono::microseconds) override { return 0; }
    uint32_t StreamRx(uint8_t, complex12_t* const*, uint32_t, StreamMeta*, std::chrono::microseconds) override { return 0; }
    uint32_t StreamTx(uint8_t, const complex32f_t* const*, uint32_t, const StreamMeta*, std::chrono::microseconds) override
    {
        return 0;
    }
    uint32_t StreamTx(uint8_t, const complex16_t* const*, uint32_t, const StreamMeta*, std::chrono::microseconds) override
    {
        return 0;
    }
    uint32_t StreamTx(uint8_t, const complex12_t* const*, uint32_t, const StreamMeta*, std::chrono::microseconds) override
    {
        return 0;
    }
    void StreamStatus(uint8_t, StreamStats*, StreamStats*) override {}
    void* GetInternalChip(uint32_t) override { return nullptr; }

  private:
    SDRDescriptor desc;
};

#if defined(__GNUC__)
    #pragma GCC diagnostic pop
#endif

// The C API's device handle is the SDRDevice pointer; tests hand the stub to the
// C entry points exactly the way lime_device_as_sdr() would.
static lime_SDRDevice* AsC(StubStreamDevice& stub)
{
    return reinterpret_cast<lime_SDRDevice*>(static_cast<SDRDevice*>(&stub));
}

static lime_StreamConfig ValidConfig(const uint32_t* rx, size_t rxCount)
{
    lime_StreamConfig cfg;
    std::memset(&cfg, 0, sizeof(cfg));
    cfg.struct_size = sizeof(lime_StreamConfig);
    cfg.rx_channels = rx;
    cfg.rx_count = rxCount;
    cfg.format = lime_DataFormat_F32;
    cfg.link_format = lime_DataFormat_I16;
    return cfg;
}

} // namespace

TEST(StreamCAPI, CreateRejectsInvalidArguments)
{
    StubStreamDevice stub;
    const uint32_t channel = 0;
    lime_StreamConfig cfg = ValidConfig(&channel, 1);

    EXPECT_EQ(lime_stream_create(nullptr, &cfg), nullptr);
    EXPECT_EQ(lime_stream_create(AsC(stub), nullptr), nullptr);

    cfg.struct_size = 4;
    EXPECT_EQ(lime_stream_create(AsC(stub), &cfg), nullptr);

    cfg = ValidConfig(nullptr, 1); // channel count without a channel array
    EXPECT_EQ(lime_stream_create(AsC(stub), &cfg), nullptr);
}

TEST(StreamCAPI, CreateMapsConfigurationToDevice)
{
    StubStreamDevice stub;
    const uint32_t rx[2] = { 0, 1 };
    const uint32_t tx[1] = { 1 };
    lime_StreamConfig cfg = ValidConfig(rx, 2);
    cfg.tx_channels = tx;
    cfg.tx_count = 1;
    cfg.module_index = 3;
    cfg.hint_sample_rate_hz = 10e6;

    lime_Stream* s = lime_stream_create(AsC(stub), &cfg);
    ASSERT_NE(s, nullptr);
    EXPECT_EQ(stub.lastModuleIndex, 3);
    EXPECT_EQ(stub.lastConfig.channels.at(TRXDir::Rx), (std::vector<uint8_t>{ 0, 1 }));
    EXPECT_EQ(stub.lastConfig.channels.at(TRXDir::Tx), (std::vector<uint8_t>{ 1 }));
    EXPECT_EQ(stub.lastConfig.format, DataFormat::F32);
    EXPECT_EQ(stub.lastConfig.linkFormat, DataFormat::I16);
    EXPECT_DOUBLE_EQ(stub.lastConfig.hintSampleRate, 10e6);
    lime_stream_destroy(s);
}

TEST(StreamCAPI, CreateReturnsNullWhenDeviceRefuses)
{
    StubStreamDevice stub;
    stub.refuseCreate = true;
    const uint32_t channel = 0;
    lime_StreamConfig cfg = ValidConfig(&channel, 1);
    EXPECT_EQ(lime_stream_create(AsC(stub), &cfg), nullptr);
}

TEST(StreamCAPI, StartStopAndDestroyForward)
{
    StubStreamDevice stub;
    const uint32_t channel = 0;
    lime_StreamConfig cfg = ValidConfig(&channel, 1);
    lime_Stream* s = lime_stream_create(AsC(stub), &cfg);
    ASSERT_NE(s, nullptr);

    EXPECT_EQ(lime_stream_start(s), lime_OpStatus_Success);
    EXPECT_TRUE(stub.calls.started);
    lime_stream_stop(s);
    EXPECT_TRUE(stub.calls.stopped);
    lime_stream_destroy(s);

    EXPECT_EQ(lime_stream_start(nullptr), lime_OpStatus_InvalidValue);
    lime_stream_stop(nullptr); // NULL is allowed and ignored
    lime_stream_destroy(nullptr);
}

TEST(StreamCAPI, RecvDispatchesFormatAndFillsMeta)
{
    StubStreamDevice stub;
    const uint32_t channel = 0;
    lime_StreamConfig cfg = ValidConfig(&channel, 1);
    lime_Stream* s = lime_stream_create(AsC(stub), &cfg);
    ASSERT_NE(s, nullptr);

    lime::complex32f_t buffer[16]{};
    void* dst[1] = { buffer };
    lime_StreamRxMeta meta;
    std::memset(&meta, 0, sizeof(meta));

    EXPECT_EQ(lime_stream_recv(s, dst, 16, &meta, 100), 16);
    EXPECT_EQ(stub.calls.recvFormat, 'f');
    EXPECT_EQ(stub.calls.count, 16u);
    EXPECT_TRUE(meta.has_timestamp);
    EXPECT_EQ(meta.timestamp, 1234u);

    EXPECT_EQ(lime_stream_recv(nullptr, dst, 16, nullptr, 100), lime_OpStatus_InvalidValue);
    EXPECT_EQ(lime_stream_recv(s, nullptr, 16, nullptr, 100), lime_OpStatus_InvalidValue);
    lime_stream_destroy(s);
}

TEST(StreamCAPI, RecvDispatchesIntegerFormat)
{
    StubStreamDevice stub;
    const uint32_t channel = 0;
    lime_StreamConfig cfg = ValidConfig(&channel, 1);
    cfg.format = lime_DataFormat_I16;
    lime_Stream* s = lime_stream_create(AsC(stub), &cfg);
    ASSERT_NE(s, nullptr);

    lime::complex16_t buffer[8]{};
    void* dst[1] = { buffer };
    EXPECT_EQ(lime_stream_recv(s, dst, 8, nullptr, 100), 8);
    EXPECT_EQ(stub.calls.recvFormat, 's');
    lime_stream_destroy(s);
}

TEST(StreamCAPI, SendTranslatesMetadata)
{
    StubStreamDevice stub;
    const uint32_t channel = 0;
    lime_StreamConfig cfg = ValidConfig(&channel, 1);
    lime_Stream* s = lime_stream_create(AsC(stub), &cfg);
    ASSERT_NE(s, nullptr);

    const lime::complex32f_t buffer[8]{};
    const void* src[1] = { buffer };
    lime_StreamTxMeta meta;
    std::memset(&meta, 0, sizeof(meta));
    meta.timestamp = 777;
    meta.has_timestamp = true;
    meta.flush = true;

    EXPECT_EQ(lime_stream_send(s, src, 8, &meta, 100), 8);
    EXPECT_EQ(stub.calls.sendFormat, 'f');
    EXPECT_EQ(stub.calls.count, 8u);
    EXPECT_TRUE(stub.calls.txMeta.hasTimestamp);
    EXPECT_EQ(stub.calls.txMeta.timestamp.GetTicks(), 777);
    EXPECT_TRUE(stub.calls.txMeta.flags & StreamTxMeta::EndOfBurst);

    // NULL metadata means no timestamp and no flush
    EXPECT_EQ(lime_stream_send(s, src, 8, nullptr, 100), 8);
    EXPECT_FALSE(stub.calls.txMeta.hasTimestamp);
    EXPECT_FALSE(stub.calls.txMeta.flags & StreamTxMeta::EndOfBurst);

    EXPECT_EQ(lime_stream_send(nullptr, src, 8, nullptr, 100), lime_OpStatus_InvalidValue);
    EXPECT_EQ(lime_stream_send(s, nullptr, 8, nullptr, 100), lime_OpStatus_InvalidValue);
    lime_stream_destroy(s);
}
