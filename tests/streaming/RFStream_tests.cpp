#include "RFStream_tests.h"

#include <chrono>
#include <thread>

#include "limesuiteng/limesuiteng.hpp"
#include "limesuiteng/RFStream.h"

#include "tests/externalData.h"

using namespace lime;

using namespace std;
using namespace std::literals::string_literals;

namespace lime::testing {

RFStream_tests::RFStream_tests()
    : device(nullptr)
    , channelCount(1)
    , sampleRate(10e6)
    , moduleIndex(0)
{
}

void RFStream_tests::SetUp()
{
    device = DeviceRegistry::makeDevice(std::string{ GetTestDeviceHandleArgument() });
    ASSERT_NE(device, nullptr);

    ASSERT_EQ(device->Init(), OpStatus::Success);

    uint64_t frequencyLO = 1e9;
    uint8_t moduleIndex = 0;

    // RF parameters
    SDRConfig config;
    config.channel[0].rx.enabled = true;
    config.channel[0].rx.centerFrequency = frequencyLO;
    config.channel[0].rx.sampleRate = sampleRate;
    config.channel[0].rx.oversample = 2;
    config.channel[0].rx.lpf = 0;
    config.channel[0].rx.path = 2; // TODO: replace with string names
    config.channel[0].rx.calibrate = false;
    config.channel[0].rx.testSignal.enabled = false;

    config.channel[0].tx.enabled = false;
    config.channel[0].tx.sampleRate = sampleRate;
    config.channel[0].tx.oversample = 2;
    config.channel[0].tx.path = 2; // TODO: replace with string names
    config.channel[0].tx.centerFrequency = frequencyLO - 1e6;
    config.channel[0].tx.testSignal.enabled = false;

    ASSERT_EQ(device->Configure(config, moduleIndex), OpStatus::Success);
}

void RFStream_tests::TearDown()
{
    if (stream)
        stream.reset();
    DeviceRegistry::freeDevice(device);
}

TEST_F(RFStream_tests, SetSampleRateIsAccurate)
{
    StreamConfig streamCfg;
    streamCfg.channels[TRXDir::Rx] = { 0 };
    streamCfg.format = DataFormat::I16;
    streamCfg.linkFormat = DataFormat::I12;

    const int samplesBatchSize = 5000;
    complex16_t samplesA[samplesBatchSize];
    complex16_t samplesB[samplesBatchSize];

    complex16_t* rxSamples[2] = { samplesA, samplesB };

    stream = std::move(device->StreamCreate(streamCfg, moduleIndex));
    ASSERT_TRUE(stream);
    // ASSERT_EQ(device->StreamSetup(stream, moduleIndex), OpStatus::Success);
    stream->Start();

    auto t1 = chrono::high_resolution_clock::now();

    int samplesReceived = 0;
    int samplesRemaining = sampleRate;
    while (samplesRemaining > 0)
    {
        int toRead = samplesRemaining < samplesBatchSize ? samplesRemaining : samplesBatchSize;
        int samplesGot = 0;

        StreamMeta rxMeta{};
        samplesGot = stream->StreamRx(rxSamples, toRead, &rxMeta);

        ASSERT_EQ(samplesGot, toRead);

        if (samplesGot != toRead)
            break;
        samplesReceived += samplesGot;
        samplesRemaining -= toRead;
    }
    auto t2 = chrono::high_resolution_clock::now();
    ASSERT_EQ(samplesRemaining, 0);
    ASSERT_EQ(samplesReceived, sampleRate);
    const auto duration{ chrono::duration_cast<chrono::milliseconds>(t2 - t1) };
    bool timeCorrect = chrono::milliseconds(980) < duration && duration < chrono::milliseconds(1020);
    ASSERT_TRUE(timeCorrect);

    //Stop streaming
    stream->Stop();
    // device->StreamDestroy(moduleIndex);
}

TEST_F(RFStream_tests, RepeatedStartStopWorks)
{
    StreamConfig streamCfg;
    streamCfg.channels[TRXDir::Rx] = { 0 };
    streamCfg.format = DataFormat::I16;
    streamCfg.linkFormat = DataFormat::I12;

    const int samplesBatchSize = 5000;
    complex16_t samplesA[samplesBatchSize];
    complex16_t samplesB[samplesBatchSize];

    complex16_t* rxSamples[2] = { samplesA, samplesB };

    stream = std::move(device->StreamCreate(streamCfg, moduleIndex));
    ASSERT_TRUE(stream);
    // ASSERT_EQ(stream->Setup(stream, moduleIndex), OpStatus::Success);
    stream->Start();

    int samplesReceived = 0;
    int samplesRemaining = samplesBatchSize;
    uint64_t lastTimestamp = 0;
    bool firstRead = true;
    while (samplesRemaining > 0)
    {
        int toRead = samplesRemaining < samplesBatchSize ? samplesRemaining : samplesBatchSize;
        int samplesGot = 0;

        StreamMeta rxMeta{};
        samplesGot = stream->StreamRx(rxSamples, toRead, &rxMeta);
        if (firstRead)
        {
            EXPECT_EQ(rxMeta.timestamp, 0);
            firstRead = false;
        }

        ASSERT_EQ(samplesGot, toRead);

        if (samplesGot != toRead)
            break;
        samplesReceived += samplesGot;
        samplesRemaining -= toRead;
        EXPECT_GE(rxMeta.timestamp, lastTimestamp);
        lastTimestamp = rxMeta.timestamp;
    }

    stream->Stop();

    stream->Start();

    samplesReceived = 0;
    samplesRemaining = samplesBatchSize;
    lastTimestamp = 0;
    firstRead = true;
    while (samplesRemaining > 0)
    {
        int toRead = samplesRemaining < samplesBatchSize ? samplesRemaining : samplesBatchSize;
        int samplesGot = 0;

        StreamMeta rxMeta{};
        samplesGot = stream->StreamRx(rxSamples, toRead, &rxMeta);
        if (firstRead)
        {
            EXPECT_EQ(rxMeta.timestamp, 0);
            firstRead = false;
        }

        ASSERT_EQ(samplesGot, toRead);

        if (samplesGot != toRead)
            break;
        samplesReceived += samplesGot;
        samplesRemaining -= toRead;
        EXPECT_GE(rxMeta.timestamp, lastTimestamp);
        lastTimestamp = rxMeta.timestamp;
    }
    ASSERT_EQ(samplesRemaining, 0);
    ASSERT_EQ(samplesReceived, samplesBatchSize);

    //Stop streaming
    stream->Stop();
    // device->StreamDestroy(moduleIndex);
}

TEST_F(RFStream_tests, RepeatedSetupDestroyWorks)
{
    StreamConfig streamCfg;
    streamCfg.channels[TRXDir::Rx] = { 0 };
    streamCfg.format = DataFormat::I16;
    streamCfg.linkFormat = DataFormat::I12;

    const int samplesBatchSize = 5000;
    complex16_t samplesA[samplesBatchSize];
    complex16_t samplesB[samplesBatchSize];

    complex16_t* rxSamples[2] = { samplesA, samplesB };

    stream = std::move(device->StreamCreate(streamCfg, moduleIndex));
    ASSERT_TRUE(stream);
    // ASSERT_EQ(device->StreamSetup(stream, moduleIndex), OpStatus::Success);
    stream->Start();
    stream->Stop();
    stream.reset();
    // device->StreamDestroy(moduleIndex);

    stream = std::move(device->StreamCreate(streamCfg, moduleIndex));
    ASSERT_TRUE(stream);
    // ASSERT_EQ(device->StreamSetup(stream, moduleIndex), OpStatus::Success);
    stream->Start();

    auto t1 = chrono::high_resolution_clock::now();

    int samplesReceived = 0;
    int samplesRemaining = sampleRate;
    while (samplesRemaining > 0)
    {
        int toRead = samplesRemaining < samplesBatchSize ? samplesRemaining : samplesBatchSize;
        int samplesGot = 0;

        StreamMeta rxMeta{};
        samplesGot = stream->StreamRx(rxSamples, toRead, &rxMeta);

        ASSERT_EQ(samplesGot, toRead);

        if (samplesGot != toRead)
            break;
        samplesReceived += samplesGot;
        samplesRemaining -= toRead;
    }
    auto t2 = chrono::high_resolution_clock::now();
    ASSERT_EQ(samplesRemaining, 0);
    ASSERT_EQ(samplesReceived, sampleRate);
    const auto duration{ chrono::duration_cast<chrono::milliseconds>(t2 - t1) };
    bool timeCorrect = chrono::milliseconds(980) < duration && duration < chrono::milliseconds(1020);
    ASSERT_TRUE(timeCorrect);

    //Stop streaming
    stream->Stop();
    // device->StreamDestroy(moduleIndex);
}

TEST_F(RFStream_tests, StreamStatusCallbackCalledIfRxOverrun)
{
    StreamConfig streamCfg;
    streamCfg.channels[TRXDir::Rx] = { 0 };
    streamCfg.format = DataFormat::I16;
    streamCfg.linkFormat = DataFormat::I12;

    long overrun_count = 0;
    auto lambda_callback = [](bool isTx, const StreamStats* stats, void* userData) -> bool {
        long* overrun = reinterpret_cast<long*>(userData);
        *overrun = stats->overrun;
        return true;
    };

    streamCfg.userData = &overrun_count;
    streamCfg.statusCallback = lambda_callback;

    stream = std::move(device->StreamCreate(streamCfg, moduleIndex));
    ASSERT_TRUE(stream);

    stream->Start();

    // should be lone enough to allow filling up internal buffers
    std::this_thread::sleep_for(std::chrono::milliseconds(150));

    ASSERT_GT(overrun_count, 0);
    stream->Stop();
    stream.reset();
}

TEST_F(RFStream_tests, StreamStatusCallbackCalledIfTxUnderrun)
{
    StreamConfig streamCfg;
    streamCfg.channels[TRXDir::Tx] = { 0 };
    streamCfg.format = DataFormat::I16;
    streamCfg.linkFormat = DataFormat::I12;

    long underrun_count = 0;
    auto lambda_callback = [](bool isTx, const StreamStats* stats, void* userData) -> bool {
        if (!isTx)
            return false;
        long* underrun = reinterpret_cast<long*>(userData);
        *underrun = stats->underrun;
        return true;
    };

    streamCfg.userData = &underrun_count;
    streamCfg.statusCallback = lambda_callback;

    complex16_t samples[1024];
    complex16_t* txSamples[2] = { samples, nullptr };

    stream = std::move(device->StreamCreate(streamCfg, moduleIndex));
    ASSERT_TRUE(stream);

    stream->Start();

    // should be lone enough to allow filling up internal buffers
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    StreamMeta txMeta{};
    txMeta.waitForTimestamp = true;
    txMeta.timestamp = 10;
    txMeta.flushPartialPacket = true;

    int sent = stream->StreamTx(txSamples, 1024, &txMeta);
    ASSERT_GT(sent, 0);

    // give some time to process the queued packets
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    ASSERT_GT(underrun_count, 0);
    stream->Stop();
    stream.reset();
}

} // namespace lime::testing
