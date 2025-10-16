#include "limesuiteng/StreamConfig.h"
#include "streaming/StreamComposite.h"
#include "limesuiteng/SDRDescriptor.h"
#include "limesuiteng/StreamMeta.h"
#include "limesuiteng/OpStatus.h"
#include <iostream>
#include <chrono>
#include <signal.h>
#include <thread>
#include <condition_variable>
#include <mutex>
#include <filesystem>
#include "args.hxx"

#include "../common.h"
#include "../src/DSP/FFT/FFT.h"
#include "../src/DSP/math/math.h"
#include "../src/utilities/WorkerThread.h"

#define _USE_MATH_DEFINES
#include <math.h>

#define USE_GNU_PLOT 1
#ifdef USE_GNU_PLOT
    #include "gnuPlotPipe.h"
#endif

using namespace lime;
using namespace std;

std::mutex globalGnuPlotMutex; // Seems multiple plot pipes can't be used concurrently
bool showPlots = false;

static LogLevel logVerbosity = LogLevel::Error;
static LogLevel strToLogLevel(const std::string_view str)
{
    if ("debug"sv == str)
        return LogLevel::Debug;
    else if ("verbose"sv == str)
        return LogLevel::Verbose;
    else if ("error"sv == str)
        return LogLevel::Error;
    else if ("warning"sv == str)
        return LogLevel::Warning;
    else if ("info"sv == str)
        return LogLevel::Info;
    return LogLevel::Error;
}

static void LogCallback(LogLevel lvl, const std::string& msg)
{
    if (lvl > logVerbosity)
        return;
    cerr << msg << endl;
}

static std::vector<int> ParseIntArray(args::NargsValueFlag<int>& flag)
{
    std::vector<int> numbers;
    for (const auto& number : args::get(flag))
        numbers.push_back(number);
    return numbers;
}

static lime::complex32f_t Chirp(double w1, double w2, double amplitude, double chirpDuration, double time)
{
    double phase = w1 * time + (w2 - w1) * time * time / (2 * chirpDuration);
    return lime::complex32f_t(amplitude * cos(phase), amplitude * sin(phase));
}

static std::vector<lime::complex32f_t> GenerateChirp(double duration, double sampleRate, double w1, double w2)
{
    std::vector<lime::complex32f_t> values;
    double step = 1.0 / sampleRate;
    double angularF = w1 * 2 * M_PI * sampleRate / 2;
    double angularF2 = w2 * 2 * M_PI * sampleRate / 2;
    double amplitude = 0.5;
    for (double t = 0; t < duration; t += step)
    {
        auto v = Chirp(angularF, angularF2, amplitude, duration, t);
        values.push_back(v);
    }
    return values;
}

static void PlotSamples(const complex32f_t* samples, size_t count)
{
    if (!showPlots)
        return;

    GNUPlotPipe plot;
    plot.writef("set yrange[%f:%f]\n", -1.0, 1.0);
    plot.write("plot '-' with lines, '-' with lines\n");
    uint32_t i = 0;
    for (; i < count; ++i)
        plot.writef("%i %f\n", i, samples[i].real());
    plot.write("e\n");
    i = 0;
    for (; i < count; ++i)
        plot.writef("%i %f\n", i, samples[i].imag());
    plot.write("e\n");
    plot.flush();
}

// static void Plotline(std::vector<double>& samples)
// {
//     if (!showPlots)
//         return;

//     GNUPlotPipe plot;
//     //plot.writef("set yrange[%f:%f]\n", -1.0, 1.0);
//     plot.write("plot '-' with lines, '-' with lines\n");
//     uint32_t i = 0;
//     for (const auto& s : samples)
//         plot.writef("%i %f\n", i++, s);
//     plot.write("e\n");
//     plot.flush();
// }

template<class T> size_t GetMaxElementIndex(const std::vector<T>& values)
{
    size_t maxIndex = 0;
    for (size_t i = 0; i < values.size(); ++i)
    {
        if (values[i] > values[maxIndex])
            maxIndex = i;
    }
    return maxIndex;
}

class ReceiverThread : public WorkerThread
{
  public:
    ReceiverThread(lime::RFStream* stream, size_t rxSize, size_t samplesToSkip)
        : stream(stream)
        , rxSize(rxSize)
        , samplesToSkip(samplesToSkip)
    {
        const int channelCount = 1;
        for (int i = 0; i < channelCount; ++i)
        {
            rxChannel[i].resize(rxSize);
            rxBuffers[i] = rxChannel[i].data();
        }
    }

    bool Work() override
    {
        StreamRxMeta rxMeta{};
        constexpr auto timeout = std::chrono::microseconds(1000000);

        // skip all samples until the expected time
        while (samplesToSkip > 0)
        {
            const size_t toRead = samplesToSkip > rxSize ? rxSize : samplesToSkip;
            rxMeta.timestamp = Timespec(0l);
            const uint32_t samplesRead = stream->Receive(rxBuffers, toRead, &rxMeta);
            if (samplesRead != toRead)
                return false;
            samplesToSkip -= samplesRead;
        }

        stream->Receive(rxBuffers, rxSize, &rxMeta);
        return false;
    }

  public:
    lime::RFStream* stream;
    size_t rxSize;
    std::vector<complex32f_t> rxChannel[16];
    complex32f_t* rxBuffers[16];
    size_t samplesToSkip;
};

class TransmitterThread : public WorkerThread
{
  public:
    TransmitterThread(lime::RFStream* stream, const std::vector<complex32f_t>& chirp, size_t chirpStart)
        : stream(stream)
        , chirp(chirp)
        , chirpStart(chirpStart)
    {
        nulldata.resize(512 * 64);
        const int channelCount = 1;
        for (int i = 0; i < channelCount; ++i)
            txSamples.push_back(chirp.data());
    }

    bool Work() override
    {
        constexpr auto timeout = std::chrono::microseconds(1000000);

        const int channelCount = 1;
        std::vector<const complex32f_t*> nullSamples;
        constexpr float coef = 0.1;
        constexpr complex32f_t pattern[4] = {
            complex32f_t(0, 1.0 * coef), complex32f_t(1.0 * coef, 0), complex32f_t(0, -1.0 * coef), complex32f_t(-1.0 * coef, 0)
        };
        for (size_t i = 0; i < nulldata.size(); ++i)
        {
            //nulldata[i] = complex32f_t(0, 0);
            nulldata[i] = pattern[i % 4];
        }
        for (int i = 0; i < channelCount; ++i)
            nullSamples.push_back(nulldata.data());

        // stream zeroes if timestamps synchronization not available
        // int64_t txSize = chirpStart;
        // while (txSize > 0)
        // {
        //     StreamTxMeta txMeta{};
        //     txMeta.hasTimestamp = true;
        //     txMeta.timestamp = Timespec(int64_t(chirpStart - txSize));
        //     // printf("TTS: %li, src: %li\n", txMeta.timestamp.GetTicks(), int64_t(chirpStart - txSize));

        //     int dummyDataSize = 4 * 512;
        //     const size_t toSend = dummyDataSize > txSize ? txSize : dummyDataSize;
        //     txSize -= toSend;
        //     txMeta.flags = 0; //StreamTxMeta::EndOfBurst;
        //     uint32_t samplesSent = stream->Transmit(nullSamples.data(), toSend, &txMeta);
        //     if (samplesSent != toSend)
        //         return false;
        // }

        {
            StreamTxMeta txMeta{};
            txMeta.hasTimestamp = true;
            txMeta.timestamp = Timespec(int64_t(chirpStart));
            txMeta.flags = StreamTxMeta::EndOfBurst;
            const size_t toSend = chirp.size();
            stream->Transmit(txSamples.data(), toSend, &txMeta);
        }

        // {
        //     StreamTxMeta txMeta{};
        //     txMeta.hasTimestamp = true;
        //     txMeta.timestamp = Timespec(int64_t(2*chirpStart));
        //     txMeta.flags = StreamTxMeta::EndOfBurst;

        //     const size_t toSend = chirp.size();
        //     stream->Transmit(txSamples.data(), toSend, &txMeta);
        // }
        return false;
    }

  private:
    lime::RFStream* stream;
    std::vector<const complex32f_t*> txSamples;
    std::vector<complex32f_t> nulldata;
    const std::vector<complex32f_t>& chirp;
    int64_t chirpStart;
};

OpStatus MeasureChannelDelays(RFStream* rxComposite,
    RFStream* txComposite,
    const std::vector<complex32f_t>& chirp,
    uint8_t channelCount,
    double sampleRate,
    uint8_t transmitChannel,
    std::vector<int>& sampleOffset)
{
    sampleOffset.clear();
    OpStatus status = OpStatus::Success;

    const int64_t chirpStart = sampleRate / 1000; // 1ms
    const uint32_t transmitSamplesCount = chirp.size();
    const int64_t receiveSamplesCount = chirpStart + transmitSamplesCount + (0.5 * sampleRate / 1000);

    ReceiverThread rx(rxComposite, receiveSamplesCount, 0);
    TransmitterThread tx(txComposite, chirp, chirpStart);

    tx.Start();
    rx.Start();

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    rxComposite->Start();
    txComposite->Start();

    rx.Wait();
    tx.Wait();

    rxComposite->Stop();
    txComposite->Stop();

    const int skipRxSamples = 0; //std::max(0l, chirpStart - transmitSamplesCount / 4); // approximate point of the expected data

    for (int c = 0; c < channelCount; ++c)
    {
        const int rxWindowSize = receiveSamplesCount - skipRxSamples;
        const complex32f_t* rxWindow = rx.rxBuffers[c] + skipRxSamples;
        PlotSamples(rxWindow, rxWindowSize);

        std::vector<complex32f_t> inputs(rxWindowSize);
        memcpy(inputs.data(), rxWindow, sizeof(complex32f_t) * rxWindowSize);
        auto correlation = CrossCorrelation(inputs, chirp);
        int ci = GetMaxElementIndex(correlation);
        const int signalExpectedAtTimestamp = chirpStart;
        const int signalFoundAtTimestamp = skipRxSamples + ci;
        std::cerr << "Ch0 chirp @ " << signalFoundAtTimestamp
                  << ", time offset: " << int64_t(1e6 * (signalFoundAtTimestamp - signalExpectedAtTimestamp) / sampleRate) << "us\n"
                  << std::endl;
        sampleOffset.push_back(signalFoundAtTimestamp - signalExpectedAtTimestamp);
    }
    return status;
}

int main(int argc, char** argv)
{
    // clang-format off
    args::ArgumentParser                parser("frontendDelayTest - measure front end delay timing", "");
    args::HelpFlag                      helpFlag(parser, "help", "This help", {'h', "help"});

    args::ValueFlag<std::string>        deviceFlag(parser, "name", "Specifies which device to use", {'d', "device"});
    args::NargsValueFlag<int>           chipFlag(parser, "index", "Specify chip index, or index list for aggregation [0,1...]", {'c', "chip"}, args::Nargs{1, static_cast<size_t>(-1)}); // Arg count range [1, size_t::maxValue]

    args::ValueFlag<int64_t>            timeFlag(parser, "ms", "Time duration in milliseconds to receive", {"time"}, 0, args::Options{});

    args::ValueFlag<std::string>        logFlag(parser, "", "Log verbosity: info, warning, error, verbose, debug", {'l', "log"}, "error", args::Options{});
    args::ImplicitValueFlag<int>        mimoFlag(parser, "channel count", "use multiple channels", {"mimo"}, 1, args::Options{});
    args::Flag plotsFlag(parser, "plot", "Draw information plots", { 'p', "plot" });
    // clang-format on

    try
    {
        parser.ParseCLI(argc, argv);
    } catch (args::Help&)
    {
        cout << parser << endl;
        return EXIT_SUCCESS;
    } catch (const std::exception& e)
    {
        cerr << e.what() << endl;
        return EXIT_FAILURE;
    }
    showPlots = plotsFlag;

    const std::string devName = args::get(deviceFlag);
    const int channelCount = mimoFlag ? args::get(mimoFlag) : 1;

    std::vector<int> chipIndexes = ParseIntArray(chipFlag);

    logVerbosity = strToLogLevel(args::get(logFlag));
    int chipIndex = 0;

    auto handles = DeviceRegistry::enumerate();
    if (handles.size() == 0)
    {
        cerr << "No devices found"sv << endl;
        return EXIT_FAILURE;
    }

    SDRDevice* device = cli::ConnectToFilteredOrDefaultDevice(devName);
    if (!device)
        return EXIT_FAILURE;

    device->SetMessageLogCallback(LogCallback);
    lime::registerLogHandler(LogCallback);

    // if chip index is not specified and device has only one, use it by default
    if (chipIndexes.empty() && device->GetDescriptor().rfSOC.size() == 1)
        chipIndexes.push_back(0);

    float sampleRate = device->GetSampleRate(chipIndex, TRXDir::Rx, 0);
    if (sampleRate <= 0)
        sampleRate = 1e6; // sample rate read-back not available, assign default value

    int chirp_len = 1360 / 2;
    double fs = 1e6;
    double chirpTime = chirp_len / fs;
    auto chirp = GenerateChirp(chirpTime, fs, 0.005, 0.04);

    //PlotSamples(chirpSamples);
    // std::vector<float> window;
    // FFT::GenerateWindowCoefficients(FFT::WindowFunctionType::HANNING, chirp_len + 1, window);
    // for (size_t i=0; i<chirp.size(); ++i)
    // {
    //     chirp[i].real(chirp[i].real() * window[i]);
    //     chirp[i].imag(chirp[i].imag() * window[i]);
    // }
    //PlotSamples(chirp);

    StreamConfig stream;
    stream.channels.at(TRXDir::Rx).push_back(0);
    stream.channels.at(TRXDir::Tx).push_back(0);

    stream.format = DataFormat::F32;
    stream.linkFormat = DataFormat::I12;
    stream.hintSampleRate = device->GetSampleRate(0, TRXDir::Rx, 0);

    auto trx = device->StreamCreate(stream, 0);

    std::vector<int> sampleOffsets;
    OpStatus ret = MeasureChannelDelays(trx.get(), trx.get(), chirp, channelCount, sampleRate, 0, sampleOffsets);
    if (ret != OpStatus::Success)
        printf("Error\n");

    printf("Tx%i ", 0);
    for (const auto& v : sampleOffsets)
        printf("\t %4i", v);
    printf("\n");

    DeviceRegistry::freeDevice(device);
    return 0;
}
