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
    double amplitude = 0.9;
    for (double t = 0; t <= duration; t += step)
    {
        auto v = Chirp(angularF, angularF2, amplitude, duration, t);
        values.push_back(v);
    }
    return values;
}

static void PlotSamples(const complex32f_t* samples, size_t count, int xoffset = 0)
{
    if (!showPlots)
        return;

    GNUPlotPipe plot;
    plot.writef("set terminal x11\n");
    plot.writef("set yrange[%f:%f]\n", -1.0, 1.0);
    plot.write("plot '-' with lines, '-' with lines\n");
    uint32_t i = 0;
    for (; i < count; ++i)
        plot.writef("%i %f\n", i + xoffset, samples[i].real());
    plot.write("e\n");
    i = 0;
    for (; i < count; ++i)
        plot.writef("%i %f\n", i + xoffset, samples[i].imag());
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
        printf("Samples to skip: %i\n", samplesToSkip);
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
        printf("Samples to recv: %i\n", rxSize);
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

        std::vector<complex32f_t> modsamples(chirp.size());
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

        int64_t burst_start = chirpStart;
        bool useTimestamp = true;
        {
            StreamTxMeta txMeta{};
            txMeta.hasTimestamp = useTimestamp;
            txMeta.timestamp = Timespec(int64_t(burst_start));
            txMeta.flags = StreamTxMeta::StartOfBurst | StreamTxMeta::EndOfBurst;
            printf("burst start @ %i\n", txMeta.timestamp.GetTicks());
            const size_t toSend = chirp.size();
            for (size_t i = 0; i < chirp.size(); ++i)
                modsamples[i] = chirp[i];
            modsamples[0] = complex32f_t(0.8, -0.8);
            // modsamples[0] = complex32f_t(1.0/4, 0);
            // modsamples[0] = complex32f_t(1.0/2, 0);
            // modsamples[0] = complex32f_t(1.0, 0);
            // modsamples[chirp.size()-4] = complex32f_t(0, -1.0);
            // modsamples[chirp.size()-3] = complex32f_t(0, -1.0);
            modsamples[chirp.size() - 2] = complex32f_t(0.8, -0.8);
            modsamples[chirp.size() - 1] = complex32f_t(-0.8, 0.8);
            txSamples[0] = modsamples.data();
            stream->Transmit(txSamples.data(), toSend, &txMeta);
            burst_start += toSend;
        }

        //this_thread::sleep_for(chrono::milliseconds(1));
        {
            StreamTxMeta txMeta{};
            txMeta.hasTimestamp = useTimestamp;
            burst_start += 16384;
            txMeta.timestamp = Timespec(int64_t(burst_start));
            printf("burst start @ %i\n", txMeta.timestamp.GetTicks());
            txMeta.flags = StreamTxMeta::StartOfBurst | StreamTxMeta::EndOfBurst;

            const size_t toSend = chirp.size();
            for (size_t i = 0; i < chirp.size(); ++i)
                modsamples[i] = chirp[i] * 0.6f;
            modsamples[0] = complex32f_t(0.6, -0.6);
            modsamples[chirp.size() - 2] = complex32f_t(0.6, -0.6);
            modsamples[chirp.size() - 1] = complex32f_t(-0.6, 0.6);
            txSamples[0] = modsamples.data();
            stream->Transmit(txSamples.data(), toSend, &txMeta);
            burst_start += toSend;
        }

        {
            StreamTxMeta txMeta{};
            txMeta.hasTimestamp = useTimestamp;
            burst_start += 16384;
            txMeta.timestamp = Timespec(int64_t(burst_start));
            printf("burst start @ %i\n", txMeta.timestamp.GetTicks());
            txMeta.flags = StreamTxMeta::StartOfBurst | StreamTxMeta::EndOfBurst;

            const size_t toSend = chirp.size();
            for (size_t i = 0; i < chirp.size(); ++i)
                modsamples[i] = chirp[i] * 0.3f;
            modsamples[0] = complex32f_t(0.4, -0.4);
            modsamples[chirp.size() - 2] = complex32f_t(0.4, -0.4);
            modsamples[chirp.size() - 1] = complex32f_t(-0.4, 0.4);
            txSamples[0] = modsamples.data();
            stream->Transmit(txSamples.data(), toSend, &txMeta);
            burst_start += toSend;
        }
        {
            StreamTxMeta txMeta{};
            txMeta.hasTimestamp = useTimestamp;
            burst_start += 16384;
            txMeta.timestamp = Timespec(int64_t(burst_start));
            txMeta.flags = StreamTxMeta::StartOfBurst | StreamTxMeta::EndOfBurst;

            const size_t toSend = chirp.size();
            for (size_t i = 0; i < chirp.size(); ++i)
                modsamples[i] = chirp[i] * 0.3f;
            modsamples[0] = complex32f_t(0.4, -0.4);
            modsamples[chirp.size() - 2] = complex32f_t(0.4, -0.4);
            modsamples[chirp.size() - 1] = complex32f_t(-0.4, 0.4);
            txSamples[0] = modsamples.data();
            stream->Transmit(txSamples.data(), toSend, &txMeta);
            burst_start += toSend;
        }
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
    const int64_t receiveSamplesCount = chirpStart + transmitSamplesCount + (2.2 * sampleRate / 1000);

    const int RxSamplesToSkip = 0;
    ReceiverThread rx(rxComposite, receiveSamplesCount, RxSamplesToSkip);
    TransmitterThread tx(txComposite, chirp, chirpStart);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    rxComposite->Start();
    txComposite->Start();
    tx.Start();
    rx.Start();

    rx.Wait();
    tx.Wait();

    rxComposite->Stop();
    txComposite->Stop();

    for (int c = 0; c < channelCount; ++c)
    {
        // Draw whole receive buffer
        // PlotSamples(rx.rxBuffers[c], 20000);
        PlotSamples(rx.rxBuffers[c], receiveSamplesCount);

        const int skipRxSamples =
            std::max(int64_t(0), chirpStart - transmitSamplesCount / 8); // approximate point of the expected data
        const int rxWindowSize = receiveSamplesCount - skipRxSamples;
        const complex32f_t* rxWindow = rx.rxBuffers[c] + skipRxSamples;

        PlotSamples(rxWindow, chirp.size() * 2.2, skipRxSamples);

        std::vector<complex32f_t> inputs(rxWindowSize);
        memcpy(inputs.data(), rxWindow, sizeof(complex32f_t) * rxWindowSize);
        auto correlation = CrossCorrelation(inputs, chirp);
        int ci = GetMaxElementIndex(correlation);
        const int signalExpectedAtTimestamp = chirpStart;
        std::cerr << "Chirp expected @ " << chirpStart << std::endl;
        const int signalFoundAtTimestamp = skipRxSamples + ci;
        std::cerr << "Tx chirp sent @ " << chirpStart << " length: " << chirp.size() << std::endl;
        std::cerr << "Rx Ch0 chirp found @ " << signalFoundAtTimestamp
                  << ", samples diff:" << signalFoundAtTimestamp - signalExpectedAtTimestamp
                  << ", time diff: " << int64_t(1e6 * (signalFoundAtTimestamp - signalExpectedAtTimestamp) / sampleRate) << "us "
                  << ", signal correlation: " << correlation[ci] << std::endl;
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

    args::ValueFlag<int64_t>            txonoffset(parser, "samples", "TxOn offset", {"txon"}, 0, args::Options{});
    args::ValueFlag<int64_t>            txoffoffset(parser, "samples", "TxOff offset", {"txoff"}, 0, args::Options{});

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

    int chirp_len = 512 * 32 + 13; //1024 * 16 + 137; //110096;//16 * 1024; //1360 / 2;
    double fs = 1e6;
    double chirpTime = chirp_len / fs;
    auto chirp = GenerateChirp(chirpTime, fs, 0.005, 0.04);
    chirp.resize(chirp_len);

    double c = 0;
    for (size_t i = 0; i < chirp.size(); ++i)
    {
        chirp[i] = complex32f_t(chirp[i].real() * c, chirp[i].imag() * c);
        if (i < chirp.size() / 2)
            c += 2.0 / chirp.size();
        else
            c -= 2.0 / chirp.size();
    }

    // chirp.resize(chirp_len);
    // int s=0;
    // for (int i=0; i<chirp.size()/2; ++i)
    //     chirp[s++] = complex32f_t(i*1.0 / chirp.size(), -i*1.0 / chirp.size());
    // for (int i=chirp.size()/2; s < chirp.size(); --i)
    //     chirp[s++] = complex32f_t(i*1.0 / chirp.size(), -i*1.0 / chirp.size());

    // PlotSamples(chirp.data(), chirp.size());

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
    stream.extraConfig.txonoffset = args::get(txonoffset);
    stream.extraConfig.txoffoffset = args::get(txoffoffset);
    printf("TDD switch offsets on:%li off:%li\n", stream.extraConfig.txonoffset, stream.extraConfig.txoffoffset);

    auto trx = device->StreamCreate(stream, 0);

    std::vector<int> sampleOffsets;
    OpStatus ret = MeasureChannelDelays(trx.get(), trx.get(), chirp, channelCount, sampleRate, 0, sampleOffsets);
    if (ret != OpStatus::Success)
        printf("Error\n");

    DeviceRegistry::freeDevice(device);
    return 0;
}
