#include "streaming.h"

#include <chrono>
#include <thread>

using namespace std;
using namespace std::literals::string_literals;

namespace lime::testing {

LimeSuiteWrapper_streaming::LimeSuiteWrapper_streaming()
{
    channelCount = 1;
    sampleRate = 10e6;
}

void LimeSuiteWrapper_streaming::SetUp()
{
    int n;
    lms_info_str_t list[16]; //should be large enough to hold all detected devices
    if ((n = LMS_GetDeviceList(list)) <= 0) //NULL can be passed to only get number of devices
        GTEST_SKIP() << "device not connected, skipping"s;

    //open the first device
    int rez = LMS_Open(&device, list[0], NULL);
    ASSERT_EQ(rez, 0);

    rez = LMS_Init(device);
    ASSERT_EQ(rez, 0);

    int oversample = 4;

    for (int i = 0; i < channelCount; ++i)
    {
        ASSERT_EQ(LMS_EnableChannel(device, LMS_CH_RX, i, true), 0);
    }
    ASSERT_EQ(LMS_SetSampleRate(device, sampleRate, oversample), 0);
    float_type host_Hz, rf_Hz;
    ASSERT_EQ(LMS_GetSampleRate(device, LMS_CH_RX, 0, &host_Hz, &rf_Hz), 0);
    ASSERT_EQ(host_Hz, sampleRate);
    ASSERT_EQ(rf_Hz, sampleRate * oversample);
}

void LimeSuiteWrapper_streaming::TearDown()
{
    LMS_Close(device);
}

TEST_F(LimeSuiteWrapper_streaming, SetSampleRateIsAccurate)
{
    std::vector<lms_stream_t> channels;
    channels.reserve(channelCount);

    for (int i = 0; i < channelCount; ++i)
    {
        lms_stream_t streamId;
        streamId.channel = 0;
        streamId.fifoSize = 1024 * 1024;
        streamId.throughputVsLatency = 1.0;
        streamId.isTx = false;
        streamId.dataFmt = lms_stream_t::LMS_FMT_I12;
        ASSERT_EQ(LMS_SetupStream(device, &streamId), 0);
        channels.push_back(streamId);
    }

    const int samplesBatchSize = 5000;
    int16_t buffer[samplesBatchSize * 2]; //buffer to hold complex values (2*samples))

    //Start streaming
    for (auto& handle : channels)
        LMS_StartStream(&handle);

    auto t1 = chrono::high_resolution_clock::now();

    int samplesReceived = 0;
    int samplesRemaining = sampleRate;
    while (samplesRemaining > 0)
    {
        int toRead = samplesRemaining < samplesBatchSize ? samplesRemaining : samplesBatchSize;
        int samplesGot = 0;
        for (auto& handle : channels)
        {
            samplesGot = LMS_RecvStream(&handle, buffer, toRead, NULL, 1000);
            ASSERT_EQ(samplesGot, toRead);
            if (samplesGot <= 0)
                break;
        }
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
    for (auto& handle : channels)
        LMS_StopStream(&handle);
    for (auto& handle : channels)
        LMS_DestroyStream(device, &handle);
}

TEST_F(LimeSuiteWrapper_streaming, RepeatedStartStopDontChangeSampleRate)
{
    std::vector<lms_stream_t> channels;
    channels.reserve(channelCount);

    for (int i = 0; i < channelCount; ++i)
    {
        lms_stream_t streamId;
        streamId.channel = 0;
        streamId.fifoSize = 1024 * 1024;
        streamId.throughputVsLatency = 1.0;
        streamId.isTx = false;
        streamId.dataFmt = lms_stream_t::LMS_FMT_I12;
        ASSERT_EQ(LMS_SetupStream(device, &streamId), 0);
        channels.push_back(streamId);
    }

    const int samplesBatchSize = 5000;
    int16_t buffer[samplesBatchSize * 2]; //buffer to hold complex values (2*samples))

    for (auto& handle : channels)
        LMS_StartStream(&handle);

    for (auto& handle : channels)
        LMS_StopStream(&handle);

    for (auto& handle : channels)
        LMS_StartStream(&handle);

    auto t1 = chrono::high_resolution_clock::now();

    int samplesReceived = 0;
    int samplesRemaining = sampleRate;
    while (samplesRemaining > 0)
    {
        int toRead = samplesRemaining < samplesBatchSize ? samplesRemaining : samplesBatchSize;
        int samplesGot = 0;
        for (auto& handle : channels)
        {
            samplesGot = LMS_RecvStream(&handle, buffer, toRead, NULL, 1000);
            ASSERT_EQ(samplesGot, toRead);
            if (samplesGot <= 0)
                break;
        }
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
    for (auto& handle : channels)
        LMS_StopStream(&handle);
    for (auto& handle : channels)
        LMS_DestroyStream(device, &handle);
}

TEST_F(LimeSuiteWrapper_streaming, RepeatedSetupDestroyDontChangeSampleRate)
{
    std::vector<lms_stream_t> channels;
    channels.reserve(channelCount);

    for (int i = 0; i < channelCount; ++i)
    {
        lms_stream_t streamId;
        streamId.channel = 0;
        streamId.fifoSize = 1024 * 1024;
        streamId.throughputVsLatency = 1.0;
        streamId.isTx = false;
        streamId.dataFmt = lms_stream_t::LMS_FMT_I12;
        ASSERT_EQ(LMS_SetupStream(device, &streamId), 0);
        channels.push_back(streamId);
    }

    const int samplesBatchSize = 5000;
    int16_t buffer[samplesBatchSize * 2]; //buffer to hold complex values (2*samples))

    for (auto& handle : channels)
        LMS_StartStream(&handle);

    for (auto& handle : channels)
        LMS_StopStream(&handle);

    for (auto& handle : channels)
        LMS_DestroyStream(device, &handle);

    channels.clear();
    for (int i = 0; i < channelCount; ++i)
    {
        lms_stream_t streamId;
        streamId.channel = 0;
        streamId.fifoSize = 1024 * 1024;
        streamId.throughputVsLatency = 1.0;
        streamId.isTx = false;
        streamId.dataFmt = lms_stream_t::LMS_FMT_I12;
        ASSERT_EQ(LMS_SetupStream(device, &streamId), 0);
        channels.push_back(streamId);
    }

    for (auto& handle : channels)
        LMS_StartStream(&handle);

    auto t1 = chrono::high_resolution_clock::now();

    int samplesReceived = 0;
    int samplesRemaining = sampleRate;
    while (samplesRemaining > 0)
    {
        int toRead = samplesRemaining < samplesBatchSize ? samplesRemaining : samplesBatchSize;
        int samplesGot = 0;
        for (auto& handle : channels)
        {
            samplesGot = LMS_RecvStream(&handle, buffer, toRead, NULL, 1000);
            ASSERT_EQ(samplesGot, toRead);
            if (samplesGot <= 0)
                break;
        }
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
    for (auto& handle : channels)
        LMS_StopStream(&handle);
    for (auto& handle : channels)
        LMS_DestroyStream(device, &handle);
}

} // namespace lime::testing