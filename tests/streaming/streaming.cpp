#include "streaming.h"

#include <chrono>
#include <thread>

#include "limesuiteng/limesuiteng.hpp"

#include "tests/externalData.h"

using namespace lime;

using namespace std;
using namespace std::literals::string_literals;

namespace lime::testing {

SDRDevice_streaming::SDRDevice_streaming()
    : device(nullptr)
    , channelCount(1)
    , sampleRate(10e6)
    , moduleIndex(0)
{
}

void SDRDevice_streaming::SetUp()
{
    device = lime::testing::GetTestDevice();
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

void SDRDevice_streaming::TearDown()
{
}

TEST_F(SDRDevice_streaming, SetSampleRateIsAccurate)
{
    StreamConfig stream;
    stream.channels[TRXDir::Rx] = { 0 };
    stream.format = DataFormat::I16;
    stream.linkFormat = DataFormat::I12;

    const int samplesBatchSize = 5000;
    complex16_t samplesA[samplesBatchSize];
    complex16_t samplesB[samplesBatchSize];

    complex16_t* rxSamples[2] = { samplesA, samplesB };

    ASSERT_EQ(device->StreamSetup(stream, moduleIndex), OpStatus::Success);
    device->StreamStart(moduleIndex);

    auto t1 = chrono::high_resolution_clock::now();

    int samplesReceived = 0;
    int samplesRemaining = sampleRate;
    while (samplesRemaining > 0)
    {
        int toRead = samplesRemaining < samplesBatchSize ? samplesRemaining : samplesBatchSize;
        int samplesGot = 0;

        StreamMeta rxMeta{};
        samplesGot = device->StreamRx(moduleIndex, rxSamples, toRead, &rxMeta);

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
    device->StreamStop(moduleIndex);
    device->StreamDestroy(moduleIndex);
}

TEST_F(SDRDevice_streaming, RepeatedStartStopWorks)
{
    StreamConfig stream;
    stream.channels[TRXDir::Rx] = { 0 };
    stream.format = DataFormat::I16;
    stream.linkFormat = DataFormat::I12;

    const int samplesBatchSize = 5000;
    complex16_t samplesA[samplesBatchSize];
    complex16_t samplesB[samplesBatchSize];

    complex16_t* rxSamples[2] = { samplesA, samplesB };

    ASSERT_EQ(device->StreamSetup(stream, moduleIndex), OpStatus::Success);
    device->StreamStart(moduleIndex);

    int samplesReceived = 0;
    int samplesRemaining = samplesBatchSize;
    uint64_t lastTimestamp = 0;
    bool firstRead = true;
    while (samplesRemaining > 0)
    {
        int toRead = samplesRemaining < samplesBatchSize ? samplesRemaining : samplesBatchSize;
        int samplesGot = 0;

        StreamMeta rxMeta{};
        samplesGot = device->StreamRx(moduleIndex, rxSamples, toRead, &rxMeta);
        if (firstRead)
        {
            EXPECT_TRUE(rxMeta.timestamp == 0);
            firstRead = false;
        }

        ASSERT_EQ(samplesGot, toRead);

        if (samplesGot != toRead)
            break;
        samplesReceived += samplesGot;
        samplesRemaining -= toRead;
        EXPECT_TRUE(rxMeta.timestamp >= lastTimestamp);
        lastTimestamp = rxMeta.timestamp;
    }

    device->StreamStop(moduleIndex);

    device->StreamStart(moduleIndex);

    samplesReceived = 0;
    samplesRemaining = samplesBatchSize;
    lastTimestamp = 0;
    firstRead = true;
    while (samplesRemaining > 0)
    {
        int toRead = samplesRemaining < samplesBatchSize ? samplesRemaining : samplesBatchSize;
        int samplesGot = 0;

        StreamMeta rxMeta{};
        samplesGot = device->StreamRx(moduleIndex, rxSamples, toRead, &rxMeta);
        if (firstRead)
        {
            EXPECT_TRUE(rxMeta.timestamp == 0);
            firstRead = false;
        }

        ASSERT_EQ(samplesGot, toRead);

        if (samplesGot != toRead)
            break;
        samplesReceived += samplesGot;
        samplesRemaining -= toRead;
        EXPECT_TRUE(rxMeta.timestamp >= lastTimestamp);
        lastTimestamp = rxMeta.timestamp;
    }
    ASSERT_EQ(samplesRemaining, 0);
    ASSERT_EQ(samplesReceived, samplesBatchSize);

    //Stop streaming
    device->StreamStop(moduleIndex);
    device->StreamDestroy(moduleIndex);
}

TEST_F(SDRDevice_streaming, RepeatedSetupDestroyWorks)
{
    StreamConfig stream;
    stream.channels[TRXDir::Rx] = { 0 };
    stream.format = DataFormat::I16;
    stream.linkFormat = DataFormat::I12;

    const int samplesBatchSize = 5000;
    complex16_t samplesA[samplesBatchSize];
    complex16_t samplesB[samplesBatchSize];

    complex16_t* rxSamples[2] = { samplesA, samplesB };

    ASSERT_EQ(device->StreamSetup(stream, moduleIndex), OpStatus::Success);
    device->StreamStart(moduleIndex);
    device->StreamStop(moduleIndex);
    device->StreamDestroy(moduleIndex);

    ASSERT_EQ(device->StreamSetup(stream, moduleIndex), OpStatus::Success);
    device->StreamStart(moduleIndex);

    auto t1 = chrono::high_resolution_clock::now();

    int samplesReceived = 0;
    int samplesRemaining = sampleRate;
    while (samplesRemaining > 0)
    {
        int toRead = samplesRemaining < samplesBatchSize ? samplesRemaining : samplesBatchSize;
        int samplesGot = 0;

        StreamMeta rxMeta{};
        samplesGot = device->StreamRx(moduleIndex, rxSamples, toRead, &rxMeta);

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
    device->StreamStop(moduleIndex);
    device->StreamDestroy(moduleIndex);
}

} // namespace lime::testing
