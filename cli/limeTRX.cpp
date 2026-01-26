#include "common.h"

#include "limesuiteng/SDRDescriptor.h"

#include "limesuiteng/StreamConfig.h"
#include "limesuiteng/Timespec.h"
#include "limesuiteng/StreamMeta.h"
#include "streaming/StreamComposite.h"
#include <iostream>
#include <chrono>
#include <cmath>
#include <signal.h>
#include <thread>
#include <condition_variable>
#include <mutex>
#include <filesystem>
#include "args.hxx"

#include "DSP/FFT/FFT.h"
#include <stdio.h>
#include <fcntl.h>
#include <time.h>

#include "nlohmann/json.hpp"

// #define USE_GNU_PLOT 1
#ifdef USE_GNU_PLOT
    #include "gnuPlotPipe.h"
#endif

using namespace lime;
using namespace lime::cli;
using namespace std;

using json = nlohmann::json;

struct CaptureChunk {
    lime::Timespec timestamp;
    uint64_t sample_start;
    uint64_t samples_count;
};

std::mutex globalGnuPlotMutex; // Seems multiple plot pipes can't be used concurrently

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

/*
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

static struct timespec utc_string_to_timespec(const std::string& str)
{
    uint32_t year;
    uint32_t month;
    uint32_t day;
    uint32_t hour;
    uint32_t minute;
    uint32_t second;
    uint64_t nanoSecond;

    sscanf(str.c_str(), "%4u-%2u-%2uT%2u:%2u:%2u.%luZ", &year, &month, &day, &hour, &minute, &second, &nanoSecond);

    struct tm tm;
    memset(&tm, 0, sizeof(struct tm));
    tm.tm_sec = second;
    tm.tm_min = minute;
    tm.tm_hour = hour;
    tm.tm_mday = day;
    tm.tm_mon = month - 1; // months since January
    tm.tm_year = year - 1900; // years since 1900
    tm.tm_isdst = -1;

    struct timespec ts;
    ts.tv_sec = UTC_to_UnixTime(tm);
    ts.tv_nsec = nanoSecond;
    return ts;
}
*/

std::atomic<bool> stopProgram(false);
void intHandler(int dummy)
{
    //std::cerr << "Stopping\n"sv;
    stopProgram.store(true);
}

#ifdef USE_GNU_PLOT
/** @brief The fast Fourier transform diagram plotter */
class FFTPlotter
{
  public:
    /**
    @brief Construct a new FFTPlotter object.
    @param sampleRate The sample rate of the transform.
    @param fftSize The amount of samples per transform.
    @param persistent Whether the plot is persistent or not.
   */
    FFTPlotter(float sampleRate, int fftSize, bool persistent)
        : plot(persistent)
        , sampleRate(sampleRate)
        , doWork(false)
    {
        bins.resize(fftSize);
        plot.writef("set xrange[%f:%f]\n set yrange[%i:%i]\n", -sampleRate / 2, sampleRate / 2, -120, 0);
        plot.flush();
    }

    /** @brief Stop the plotter and destroy the FFTPlotter object. */
    ~FFTPlotter() { Stop(); }

    /** @brief Start the plotter. */
    void Start()
    {
        doWork = true;
        plotThread = std::thread(&FFTPlotter::PlotLoop, this);
    }

    /** @brief Stop the plotter. */
    void Stop()
    {
        doWork = false;
        if (plotThread.joinable())
        {
            plotDataReady.notify_one();
            plotThread.join();
        }
    }

    /**
      @brief Set the plot data.
      @param data The new data to plot.
     */
    void SubmitData(const vector<float>& data)
    {
        {
            std::unique_lock<std::mutex> lk(plotLock);
            bins = data;
        }
        plotDataReady.notify_one();
    }

  private:
    /** @brief The plot drawing loop. */
    void PlotLoop()
    {
        std::unique_lock<std::mutex> lk(plotLock);
        while (doWork)
        {
            if (plotDataReady.wait_for(lk, std::chrono::milliseconds(2000)) == std::cv_status::timeout)
            {
                cerr << "plot timeout" << endl;
                continue;
            }
            if (!doWork)
                break;

            std::unique_lock<std::mutex> glk(globalGnuPlotMutex);
            plot.write("plot '-' with lines\n");
            const int fftSize = bins.size();
            for (int j = fftSize / 2; j < fftSize; ++j)
                plot.writef("%f %f\n", sampleRate * (j - fftSize) / fftSize, bins[j]);
            for (int j = 0; j < fftSize / 2; ++j)
                plot.writef("%f %f\n", sampleRate * j / fftSize, bins[j]);
            plot.write("e\n");
            plot.flush();
        }
    }

    GNUPlotPipe plot; ///< The GNU Plot object
    std::vector<float> bins; ///< The data storage
    std::condition_variable plotDataReady; ///< Whether the plot data is ready or not
    std::mutex plotLock; ///< The plot lock
    std::thread plotThread; ///< The plotter thread
    float sampleRate; ///< The sample rate of the data
    bool doWork; ///< Whether to continue plotting or not
};

/** @brief The constellation diagram plotter */
class ConstellationPlotter
{
  public:
    /**
    @brief Construct a new Constellation Plotter object.
    @param range The half side-length of the square to render.
    @param persistent Whether the plot is persistent or not.
   */
    ConstellationPlotter(int range, bool persistent)
        : plot(persistent)
        , doWork(false)
    {
        plot.writef("set size square\n set xrange[%i:%i]\n set yrange[%i:%i]\n", -range, range, -range, range);
        plot.flush();
    }

    /** @brief Stop the thread and destroy the Constellation Plotter object. */
    ~ConstellationPlotter() { Stop(); }

    /** @brief Start the plotter loop. */
    void Start()
    {
        doWork = true;
        plotThread = std::thread(&ConstellationPlotter::PlotLoop, this);
    }

    /** @brief Stop the plotter loop. */
    void Stop()
    {
        doWork = false;
        if (plotThread.joinable())
        {
            plotDataReady.notify_one();
            plotThread.join();
        }
    }

    /**
      @brief Submit data to the plotter.
      @param data The data to submit to the plotter.
     */
    void SubmitData(const vector<complex16_t>& data)
    {
        {
            std::unique_lock<std::mutex> lk(plotLock);
            samples = data;
        }
        plotDataReady.notify_one();
    }

  private:
    /** @brief The plot drawing loop. */
    void PlotLoop()
    {
        std::unique_lock<std::mutex> lk(plotLock);
        while (doWork)
        {
            if (plotDataReady.wait_for(lk, std::chrono::milliseconds(2000)) == std::cv_status::timeout)
            {
                cerr << "plot timeout" << endl;
                continue;
            }
            if (!doWork)
                break;

            std::unique_lock<std::mutex> glk(globalGnuPlotMutex);
            plot.write("plot '-' with points\n");
            const int samplesCount = samples.size();
            for (int j = 0; j < samplesCount; ++j)
                plot.writef("%i %i\n", samples[j].real(), samples[j].imag());
            plot.write("e\n");
            plot.flush();
        }
    }

    GNUPlotPipe plot; ///< The GNU Plot object
    std::vector<complex16_t> samples; ///< The stored samples
    std::condition_variable plotDataReady; ///< Whether the plot data is ready or not
    std::mutex plotLock; ///< The plot lock
    std::thread plotThread; ///< The plotter thread
    bool doWork; ///< Whether to continue plotting or not
};
#endif

struct TransmitLoopArgs {
    RFStream* stream;
    std::ifstream* fileStream;
    int channelCount;
    std::atomic<bool>* terminate;
    bool loop;
};

static void TransmitLoop(TransmitLoopArgs* args)
{
    std::ifstream* fin = args->fileStream;
    assert(fin);

    fin->seekg(0, fin->end);
    std::streamsize fileSize = fin->tellg();
    fin->seekg(0, fin->beg);

    const int64_t txSamplesCountTotal = fileSize / sizeof(complex16_t) / args->channelCount;

    cerr << "Tx file size : "sv << fileSize << " bytes."sv << endl;

    const int samplesBatchSize = 4 * 256 * 16;
    complex16_t* txSamples[16];
    std::vector<complex16_t> channelSamples[16];
    for (int i = 0; i < args->channelCount; ++i)
    {
        channelSamples[i].resize(samplesBatchSize);
        txSamples[i] = channelSamples[i].data();
    }

    StreamTxMeta txMeta{};
    txMeta.flags = 0;
    txMeta.timestamp = Timespec(int64_t(0));
    txMeta.hasTimestamp = false; // transmit immediately

    std::vector<complex16_t> interleavedBuffer;
    if (args->channelCount > 1)
        interleavedBuffer.resize(samplesBatchSize * args->channelCount);

    do
    {
        std::streamsize samplesRemaining = txSamplesCountTotal;
        txMeta.flags = 0;
        while (samplesRemaining > 0 && args->terminate->load(std::memory_order_relaxed) == false)
        {
            int samplesToSend = samplesRemaining > samplesBatchSize ? samplesBatchSize : samplesRemaining;
            if (args->channelCount == 1)
            {
                fin->read(reinterpret_cast<char*>(txSamples[0]), samplesToSend * sizeof(complex16_t));
            }
            else
            {
                fin->read(
                    reinterpret_cast<char*>(interleavedBuffer.data()), args->channelCount * samplesToSend * sizeof(complex16_t));
                int srcIndex = 0;
                for (int s = 0; s < samplesToSend; ++s)
                {
                    for (int c = 0; c < args->channelCount; ++c)
                        txSamples[c][s] = interleavedBuffer[srcIndex++];
                }
            }
            if (samplesRemaining < samplesBatchSize)
                txMeta.flags = StreamTxMeta::EndOfBurst;
            int32_t samplesSent = args->stream->Transmit(txSamples, samplesToSend, &txMeta);

            txMeta.timestamp.AddTicks(samplesSent);
            samplesRemaining -= samplesSent;
        }
    } while (args->loop && args->terminate->load(std::memory_order_relaxed) == false);
}

int main(int argc, char** argv)
{
    // clang-format off
    args::ArgumentParser                parser("limeTRX - Realtime streaming of RF samples", "");
    args::HelpFlag                      helpFlag(parser, "help", "This help", {'h', "help"});

    args::ValueFlag<std::string>        deviceFlag(parser, "name", "Specifies which device to use", {'d', "device"});
    args::NargsValueFlag<int>           chipFlag(parser, "index", "Specify chip index, or index list for aggregation [0,1...]", {'c', "chip"}, args::Nargs{1, static_cast<size_t>(-1)}); // Arg count range [1, size_t::maxValue]
    args::ValueFlag<std::string>        inputFlag(parser, "file path", "Waveform file for samples transmitting", {'i', "input"});
    args::ValueFlag<std::string>        outputFlag(parser, "file path", "Waveform file for received samples", {'o', "output"}, "", args::Options{});
    args::Flag                          looptxFlag(parser, "", "Loop tx samples transmission", {"looptx"});
    args::ValueFlag<int64_t>            samplesCountFlag(parser, "sample count", "Number of samples to receive", {'s', "samplesCount"}, 0, args::Options{});
    args::ValueFlag<int64_t>            timeFlag(parser, "ms", "Time duration in milliseconds to receive", {"time"}, 0, args::Options{});
    args::Flag                          fftFlag(parser, "", "Display Rx FFT plot", {"fft"});
    args::ValueFlag<std::string>        logFlag(parser, "", "Log verbosity: info, warning, error, verbose, debug", {'l', "log"}, "error", args::Options{});
    args::ImplicitValueFlag<int>        mimoFlag(parser, "channel count", "use multiple channels", {"mimo"}, 2, args::Options{});
    args::ImplicitValueFlag<int64_t>    repeaterFlag(parser, "delaySamples", "retransmit received samples with a delay", {"repeater"}, 0, args::Options{});
    args::ValueFlag<std::string>        linkFormatFlag(parser, "I16, I12", "Data transfer format. Default: I12", {"linkFormat"}, "I12", args::Options{});
    args::Flag                          syncPPSFlag(parser, "", "start sampling on next PPS", {"syncPPS"});
    args::ValueFlag<int>                rxSamplesInPacketFlag(parser, "packets", "number of samples in Rx packet", {"rxSamplesInPacket"}, 0, args::Options{});
    args::ValueFlag<int>                txSamplesInPacketFlag(parser, "packets", "number of samples in Tx packet", {"txSamplesInPacket"}, 0, args::Options{});
    args::ValueFlag<int>                rxPacketsInBatchFlag(parser, "packets", "number of Rx packets in data transfer", {"rxPacketsInBatch"}, 0, args::Options{});
    args::ValueFlag<int>                txPacketsInBatchFlag(parser, "packets", "number of Tx packets in data transfer", {"txPacketsInBatch"}, 0, args::Options{});
    args::ValueFlag<int>                fftSizeFlag(parser, "samplesCount", "FFT size in samples", {"fftSize"}, 16384, args::Options{});
    args::ValueFlag<std::string>        timestampFlag(parser, "", "Timestamp type: ticks, seconds, unix", {'t', "timestamp"}, "ticks", args::Options{});
#ifdef USE_GNU_PLOT
    args::Flag                          constellationFlag(parser, "", "Display IQ constellation plot", {"constellation"});
#endif
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

    const std::string devName = args::get(deviceFlag);
    const std::string rxFilename = args::get(outputFlag);
    const std::string txFilename = args::get(inputFlag);
    const bool rx = true; // outputFlag || repeaterFlag || (!repeaterFlag && !inputFlag);
    const bool tx = inputFlag || repeaterFlag;
    const bool showFFT = fftFlag;
#ifdef USE_GNU_PLOT
    const bool showConstellation = constellationFlag;
#endif
    const uint64_t samplesToCollect = args::get(samplesCountFlag);
    const int64_t workTime = args::get(timeFlag);
    const int channelCount = mimoFlag ? args::get(mimoFlag) : 1;
    const bool repeater = repeaterFlag;
    const int64_t repeaterDelay = args::get(repeaterFlag);
    const bool syncPPS = syncPPSFlag;
    const int rxSamplesInPacket = args::get(rxSamplesInPacketFlag);
    const int txSamplesInPacket = args::get(txSamplesInPacketFlag);
    const int rxPacketsInBatch = args::get(rxPacketsInBatchFlag);
    const int txPacketsInBatch = args::get(txPacketsInBatchFlag);

    std::vector<int> chipIndexes = ParseIntArray(chipFlag);

    logVerbosity = strToLogLevel(args::get(logFlag));

    DataFormat linkFormat = DataFormat::I12;
    if (linkFormatFlag)
    {
        std::string val = args::get(linkFormatFlag);
        if (val == "I16")
            linkFormat = DataFormat::I16;
        else if (val == "I12")
            linkFormat = DataFormat::I12;
        else
        {
            cerr << "Invalid linkFormat "sv << val << std::endl;
            return EXIT_FAILURE;
        }
    }

    std::ifstream txFile;
    if (tx && !txFilename.empty())
    {
        txFile.open(txFilename, std::ifstream::in | std::ifstream::binary);
        if (!txFile.is_open())
        {
            cerr << "Failed to open file: "sv << txFilename << endl;
            return -1;
        }
    }

    auto handles = DeviceRegistry::enumerate();
    if (handles.size() == 0)
    {
        cerr << "No devices found"sv << endl;
        return EXIT_FAILURE;
    }

    SDRDevice* device = ConnectToFilteredOrDefaultDevice(devName);
    if (!device)
        return EXIT_FAILURE;

    device->SetMessageLogCallback(lime::cli::LogCallback);
    lime::registerLogHandler(lime::cli::LogCallback);

    // if chip index is not specified and device has only one, use it by default
    if (chipIndexes.empty() && device->GetDescriptor().rfSOC.size() == 1)
        chipIndexes.push_back(0);

    int rx_require = rx ? channelCount : 0;
    int tx_require = tx ? channelCount : 0;

    // Samples data streaming configuration
    StreamConfig streamCfg;
    streamCfg.format = DataFormat::I16;
    streamCfg.linkFormat = linkFormat;

    {
        streamCfg.timestampType = TimestampType::SAMPLE_TICKS;
        const std::string str = args::get(timestampFlag);
        if ("ticks"sv == str)
            streamCfg.timestampType = TimestampType::SAMPLE_TICKS;
        else if ("seconds"sv == str)
            streamCfg.timestampType = TimestampType::REALTIME_SECONDS;
        else if ("unix"sv == str)
            streamCfg.timestampType = TimestampType::UNIX_EPOCH;
    }

    if (syncPPS || rxSamplesInPacket || rxPacketsInBatch || txSamplesInPacket || txPacketsInBatch)
    {
        streamCfg.extraConfig.waitPPS = syncPPS;
        streamCfg.extraConfig.rx.samplesInPacket = rxSamplesInPacket;
        streamCfg.extraConfig.tx.samplesInPacket = txSamplesInPacket;
        streamCfg.extraConfig.rx.packetsInBatch = rxPacketsInBatch;
        streamCfg.extraConfig.tx.packetsInBatch = txPacketsInBatch;
    }

    double sampleRate = device->GetSampleRate(chipIndexes.front(), TRXDir::Rx, 0);
    streamCfg.hintSampleRate = sampleRate;

    std::unique_ptr<StreamComposite> stream = std::make_unique<StreamComposite>();
    for (size_t index : chipIndexes)
    {
        if (rx_require == 0 && tx_require == 0)
            break;

        streamCfg.channels.at(TRXDir::Rx).clear();
        streamCfg.channels.at(TRXDir::Tx).clear();

        const int deviceChannelCount = device->GetDescriptor().rfSOC[index].channelCount;
        for (int j = 0; j < deviceChannelCount; ++j)
        {
            if (rx_require > 0)
            {
                streamCfg.channels.at(TRXDir::Rx).push_back(j);
                --rx_require;
            }
            if (tx_require > 0)
            {
                streamCfg.channels.at(TRXDir::Tx).push_back(j);
                --tx_require;
            }
            if (rx_require == 0 && tx_require == 0)
                break;
        }

        stream->Add(device->StreamCreate(streamCfg, index));
    }

    rx_require = rx ? channelCount : 0;
    tx_require = tx ? channelCount : 0;
    streamCfg.channels.at(TRXDir::Rx).clear();
    for (int i = 0; rx && i < rx_require; ++i)
        streamCfg.channels.at(TRXDir::Rx).push_back(i);

    streamCfg.channels.at(TRXDir::Tx).clear();
    for (int i = 0; tx && i < tx_require; ++i)
        streamCfg.channels.at(TRXDir::Tx).push_back(i);

    OpStatus status = stream->Setup(streamCfg);
    if (status != OpStatus::Success)
    {
        cerr << "Failed to setup streams" << endl;
        stream.reset();
        DeviceRegistry::freeDevice(device);
        return EXIT_FAILURE;
    }

    signal(SIGINT, intHandler);

    const int fftSize = args::get(fftSizeFlag);
    std::vector<complex16_t> rxData[16];
    for (int i = 0; i < 16; ++i)
        rxData[i].resize(fftSize);

    lime::FFT fft(channelCount, fftSize);
    std::vector<float> fftBins(fftSize);
    fftBins[0] = 0;
    std::vector<complex32f_t> samples(fftSize);

    std::ofstream rxFile;
    if (!rxFilename.empty())
    {
        std::cout << "Rx data to file: "sv << rxFilename << std::endl;
        rxFile.open(rxFilename + ".sigmf-data", std::ofstream::out | std::ofstream::binary);
    }

    float peakAmplitude = 0;
    float peakFrequency = 0;
    if (sampleRate <= 0)
        sampleRate = 1; // sample rate read-back not available, assign default value
    float frequencyLO = 0;

#ifdef USE_GNU_PLOT
    bool persistPlotWindows = false;
    int range = 32768;
    ConstellationPlotter constellationPlot(range, persistPlotWindows);
    FFTPlotter fftPlot(sampleRate, fftSize, persistPlotWindows);
#endif

    int tickRatio = streamCfg.timestampType == TimestampType::SAMPLE_TICKS ? 1 : 2;

    StreamRxMeta rxMeta{};
    StreamTxMeta txMeta{};
    txMeta.flags = StreamTxMeta::EndOfBurst;

    lime::Timespec ts(0, sampleRate / 100, tickRatio * sampleRate);
    txMeta.timestamp = ts; //sampleRate / 100; // send tx samples 10ms after start

#ifdef USE_GNU_PLOT
    if (showFFT)
        fftPlot.Start();
    if (showConstellation)
        constellationPlot.Start();
#endif

    json rxmetadataJSON = { { "global",
                                { { "core:datatype", "ci16_le" },
                                    { "core:num_channels", channelCount },
                                    { "core:sample_rate", sampleRate },
                                    { "core:version", "0.0.1" },
                                    { "core:recorder", "limeTRX" } } },
        { "captures", json::array() },
        { "annotations", json::array() } };

    std::vector<CaptureChunk> rxcaptures;
    rxcaptures.reserve(1024 * 1024);

    stream->StageStart();
    status = stream->Start();
    if (status != OpStatus::Success)
    {
        std::cerr << "Failed to start streaming" << std::endl;
        DeviceRegistry::freeDevice(device);
        return EXIT_FAILURE;
    }

    auto startTime = std::chrono::high_resolution_clock::now();
    auto t1 = startTime - std::chrono::seconds(2); // rewind t1 to do update on first loop
    auto t2 = t1;

    std::thread txThread;
    TransmitLoopArgs txArgs;
    txArgs.fileStream = &txFile;
    txArgs.channelCount = channelCount;
    txArgs.stream = stream.get();
    txArgs.terminate = &stopProgram;
    txArgs.loop = looptxFlag;

    uint64_t totalSamplesReceived = 0;

    if (tx && !repeater)
        txThread = std::thread(TransmitLoop, &txArgs);

    Timespec rxExpectedTs(-1, 0);

    bool getActualStreamStartTime = syncPPS;

    std::vector<complex16_t> interleavingBuffer;
    if (!rxFilename.empty())
    {
        interleavingBuffer.resize(fftSize * channelCount);
    }

    while (stopProgram.load() == false)
    {
        if (!rx)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        if (samplesToCollect != 0 && totalSamplesReceived > samplesToCollect)
            break;

        complex16_t* rxSamples[16];
        for (int i = 0; i < 16; ++i)
            rxSamples[i] = rxData[i].data();
        uint32_t samplesRead = stream->Receive(rxSamples, fftSize, &rxMeta);
        if (getActualStreamStartTime)
        {
            // stream start can wait varying amount of time for the PPS, set start time once the first data has been produced
            startTime = std::chrono::high_resolution_clock::now();
            getActualStreamStartTime = false;
        }

        if (workTime != 0 && (std::chrono::high_resolution_clock::now() - startTime) > std::chrono::milliseconds(workTime))
            break;

        if (samplesRead == 0)
            continue;

        if (tx && repeater)
        {
            txMeta.timestamp = rxMeta.timestamp;
            Timespec txts = rxMeta.timestamp;
            txts.AddTicks(tickRatio * repeaterDelay);
            txts.AddTicks(tickRatio * samplesRead);
            txMeta.timestamp = txts;
            txMeta.hasTimestamp = true;
            txMeta.flags = 0;
            stream->Transmit(rxSamples, samplesRead, &txMeta);
        }

        // process samples
        if (!rxFilename.empty())
        {
            double diff = (rxExpectedTs - rxMeta.timestamp).GetRealSeconds();
            if (std::fabs(diff) > 1.0 / sampleRate)
            {
                rxcaptures.push_back({ rxMeta.timestamp, totalSamplesReceived, samplesRead });
            }

            if (channelCount == 1)
            {
                rxFile.write(reinterpret_cast<char*>(rxSamples[0]), samplesRead * sizeof(lime::complex16_t));
            }
            else
            {
                int destIndex = 0;
                for (uint32_t s = 0; s < samplesRead; ++s)
                {
                    for (int c = 0; c < channelCount; ++c)
                        interleavingBuffer[destIndex++] = rxSamples[c][s];
                }
                rxFile.write(reinterpret_cast<char*>(interleavingBuffer.data()), destIndex * sizeof(lime::complex16_t));
            }
        }
        rxExpectedTs = rxMeta.timestamp + Timespec(samplesRead / sampleRate);
        totalSamplesReceived += samplesRead;

        t2 = std::chrono::high_resolution_clock::now();
        const bool doUpdate = t2 - t1 > std::chrono::milliseconds(500);
        if (doUpdate)
            t1 = t2;

        if (doUpdate)
        {
            if (showFFT)
            {
                for (int i = 0; i < fftSize; ++i)
                {
                    samples[i] = complex32f_t(rxSamples[0][i].real() / 32768.0, rxSamples[0][i].imag() / 32768.0);
                }
                fftBins = lime::FFT::Calc(samples);
                lime::FFT::ConvertToDBFS(fftBins);

                for (int i = 0; i < fftSize; ++i)
                {
                    // exclude DC from amplitude comparison, the 0 bin
                    if (fftBins[i] > peakAmplitude && i > 0)
                    {
                        peakAmplitude = fftBins[i];
                        peakFrequency = i * sampleRate / fftSize;
                    }
                }
                if (peakFrequency > sampleRate / 2)
                    peakFrequency = peakFrequency - sampleRate;

                std::cerr << "Samples received: " << totalSamplesReceived << " Peak amplitude " << std::fixed
                          << std::setprecision(2) << peakAmplitude << " dBFS @ " << std::setprecision(3)
                          << (frequencyLO + peakFrequency) / 1e6 << " MHz" << endl;
                peakAmplitude = -1000;
#ifdef USE_GNU_PLOT
                fftPlot.SubmitData(fftBins);
#endif
            }
            else
            {
                // std::cerr << "Samples received: " << totalSamplesReceived << endl;
            }
        }

#ifdef USE_GNU_PLOT
        if (showConstellation && doUpdate)
            constellationPlot.SubmitData(rxData[0]);
#endif
    }
    stopProgram.store(true);

    if (tx && !repeater)
        txThread.join();

#ifdef USE_GNU_PLOT
    // some sleep for GNU plot data to flush, otherwise sometimes cout spams  gnuplot "invalid command"
    this_thread::sleep_for(std::chrono::milliseconds(500));
#endif
    stream->Stop();
    stream.reset();
    DeviceRegistry::freeDevice(device);

    rxFile.close();

    if (!rxFilename.empty())
    {
        ofstream rxMetaFile;
        rxMetaFile.open(rxFilename + ".sigmf-meta", std::ofstream::out);
        auto& captures = rxmetadataJSON["captures"];
        for (auto& chunk : rxcaptures)
        {
            struct timespec ts;
            ts.tv_sec = chunk.timestamp.GetSeconds();
            ts.tv_nsec = chunk.timestamp.GetFracSeconds() * 1e9;
            if (streamCfg.timestampType == TimestampType::UNIX_EPOCH)
            {
                // add datetime only if using such timestamps
                captures.push_back(
                    json{ { "core:sample_start", chunk.sample_start }, { "core:datetime", timespec_to_utc_string(&ts) } });
            }
            else
            {
                captures.push_back(json{ { "core:sample_start", chunk.sample_start } });
            }
        }
        rxMetaFile << rxmetadataJSON.dump();
        rxMetaFile.close();
    }

    return 0;
}
