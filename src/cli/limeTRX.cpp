#include "cli/common.h"
#include "limesuiteng/StreamConfig.h"
#include "limesuiteng/StreamComposite.h"
#include <iostream>
#include <chrono>
#include <cmath>
#include <signal.h>
#include <thread>
#include "kissFFT/kiss_fft.h"
#include <condition_variable>
#include <mutex>
#include <getopt.h>
#include <filesystem>
#include "args/args.hxx"

// #define USE_GNU_PLOT 1
#ifdef USE_GNU_PLOT
    #include "gnuPlotPipe.h"
#endif

using namespace lime;
using namespace std;

std::mutex globalGnuPlotMutex; // Seems multiple plot pipes can't be used concurrently

bool stopProgram(false);
void intHandler(int dummy)
{
    //std::cerr << "Stopping\n"sv;
    stopProgram = true;
}

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
    @param range The half sidelength of the square to render.
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
                plot.writef("%i %i\n", samples[j].i, samples[j].q);
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

static std::vector<int> ParseIntArray(args::NargsValueFlag<int>& flag)
{
    std::vector<int> numbers;
    for (const auto& number : args::get(flag))
        numbers.push_back(number);
    return numbers;
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
    const bool rx = true; // Need to always read data to get timestamps - BY DESIGN
    const bool tx = inputFlag || repeaterFlag;
    const bool showFFT = fftFlag;
#ifdef USE_GNU_PLOT
    const bool showConstelation = constellationFlag;
#endif
    const bool loopTx = looptxFlag;
    const int64_t samplesToCollect = args::get(samplesCountFlag);
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

    StreamComposite* composite = nullptr;
    logVerbosity = strToLogLevel(args::get(logFlag));
    int chipIndex = 0;
    bool useComposite = false;

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
            cerr << "Invalid linkFormat "sv << optarg << std::endl;
            return EXIT_FAILURE;
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

    device->SetMessageLogCallback(LogCallback);
    lime::registerLogHandler(LogCallback);

    // if chip index is not specified and device has only one, use it by default
    if (chipIndexes.empty() && device->GetDescriptor().rfSOC.size() == 1)
        chipIndexes.push_back(0);

    try
    {
        // Samples data streaming configuration
        StreamConfig stream;
        for (int i = 0; rx && i < channelCount; ++i)
        {
            stream.channels.at(TRXDir::Rx).push_back(i);
        }

        for (int i = 0; tx && i < channelCount; ++i)
        {
            stream.channels.at(TRXDir::Tx).push_back(i);
        }

        stream.format = DataFormat::I16;
        stream.linkFormat = linkFormat;

        if (syncPPS || rxSamplesInPacket || rxPacketsInBatch || txSamplesInPacket || txPacketsInBatch)
        {
            stream.extraConfig.waitPPS = syncPPS;
            stream.extraConfig.rx.samplesInPacket = rxSamplesInPacket;
            stream.extraConfig.tx.samplesInPacket = txSamplesInPacket;
            stream.extraConfig.rx.packetsInBatch = rxPacketsInBatch;
            stream.extraConfig.tx.packetsInBatch = txPacketsInBatch;
        }

        useComposite = chipIndexes.size() > 1;
        if (useComposite)
        {
            std::vector<StreamAggregate> aggregates(chipIndexes.size());
            for (size_t i = 0; i < chipIndexes.size(); ++i)
            {
                aggregates[i].device = device;
                aggregates[i].streamIndex = chipIndexes[i];
                int deviceChannelCount = device->GetDescriptor().rfSOC[chipIndexes[i]].channelCount;
                for (int j = 0; j < deviceChannelCount; ++j)
                    aggregates[i].channels.push_back(j);
            }
            composite = new StreamComposite(std::move(aggregates));
            composite->StreamSetup(stream);
        }
        else
        {
            chipIndex = chipIndexes.empty() ? 0 : chipIndexes[0];
            device->StreamSetup(stream, chipIndex);
        }
    } catch (std::runtime_error& e)
    {
        std::cout << "Failed to configure settings: "sv << e.what() << std::endl;
        return -1;
    } catch (std::logic_error& e)
    {
        std::cout << "Failed to configure settings: "sv << e.what() << std::endl;
        return -1;
    }

    signal(SIGINT, intHandler);

    const int fftSize = 16384;
    std::vector<complex16_t> rxData[16];
    for (int i = 0; i < 16; ++i)
        rxData[i].resize(fftSize);

    std::vector<complex16_t> txData;
    int64_t txSent = 0;
    if (tx && !txFilename.empty())
    {
        std::ifstream inputFile;
        inputFile.open(txFilename, std::ifstream::in | std::ifstream::binary);
        if (!inputFile)
        {
            cerr << "Failed to open file: "sv << txFilename << endl;
            return -1;
        }
        inputFile.seekg(0, std::ios_base::end);
        auto cnt = inputFile.tellg();
        inputFile.seekg(0, std::ios_base::beg);
        cerr << "File size : "sv << cnt << " bytes."sv << endl;
        txData.resize(cnt / sizeof(complex16_t));
        inputFile.read(reinterpret_cast<char*>(txData.data()), cnt);
        inputFile.close();
    }

    int64_t totalSamplesReceived = 0;

    std::vector<float> fftBins(fftSize);
    kiss_fft_cfg m_fftCalcPlan = kiss_fft_alloc(fftSize, 0, nullptr, nullptr);
    kiss_fft_cpx m_fftCalcIn[fftSize];
    kiss_fft_cpx m_fftCalcOut[fftSize];
    fftBins[0] = 0;

    std::ofstream rxFile;
    if (!rxFilename.empty())
    {
        std::cout << "Rx data to file: "sv << rxFilename << std::endl;
        rxFile.open(rxFilename, std::ofstream::out | std::ofstream::binary);
    }

    float peakAmplitude = 0;
    float peakFrequency = 0;
    float sampleRate = device->GetSampleRate(chipIndex, TRXDir::Rx, 0);
    if (sampleRate <= 0)
        sampleRate = 1; // sample rate readback not available, assign default value
    float frequencyLO = 0;

#ifdef USE_GNU_PLOT
    bool persistPlotWindows = false;
    int range = 32768;
    ConstellationPlotter constellationplot(range, persistPlotWindows);
    FFTPlotter fftplot(sampleRate, fftSize, persistPlotWindows);
#endif

    StreamMeta rxMeta{};
    StreamMeta txMeta{};
    txMeta.waitForTimestamp = true;
    txMeta.timestamp = sampleRate / 100; // send tx samples 10ms after start

#ifdef USE_GNU_PLOT
    if (showFFT)
        fftplot.Start();
    if (showConstelation)
        constellationplot.Start();
#endif
    if (useComposite)
        composite->StreamStart();
    else
        device->StreamStart(chipIndex);

    auto startTime = std::chrono::high_resolution_clock::now();
    auto t1 = startTime - std::chrono::seconds(2); // rewind t1 to do update on first loop
    auto t2 = t1;

    while (!stopProgram)
    {
        if (workTime != 0 && (std::chrono::high_resolution_clock::now() - startTime) > std::chrono::milliseconds(workTime))
            break;
        if (samplesToCollect != 0 && totalSamplesReceived > samplesToCollect)
            break;

        int toSend = txData.size() - txSent > fftSize ? fftSize : txData.size() - txSent;
        if (tx && !repeater)
        {
            if (loopTx && toSend == 0)
            {
                txSent = 0;
                toSend = txData.size() - txSent > fftSize ? fftSize : txData.size() - txSent;
            }
            if (toSend > 0)
            {
                const complex16_t* txSamples[16];
                for (int i = 0; i < 16; ++i)
                    txSamples[i] = &txData[txSent];
                uint32_t samplesSent = useComposite ? composite->StreamTx(txSamples, toSend, &txMeta)
                                                    : device->StreamTx(chipIndex, txSamples, toSend, &txMeta);
                if (samplesSent > 0)
                {
                    txSent += samplesSent;
                    txMeta.timestamp += samplesSent;
                }
            }
        }

        complex16_t* rxSamples[16];
        for (int i = 0; i < 16; ++i)
            rxSamples[i] = rxData[i].data();
        uint32_t samplesRead = useComposite ? composite->StreamRx(rxSamples, fftSize, &rxMeta)
                                            : device->StreamRx(chipIndex, rxSamples, fftSize, &rxMeta);
        if (samplesRead == 0)
            continue;

        if (tx && repeater)
        {
            txMeta.timestamp = rxMeta.timestamp + samplesRead + repeaterDelay;
            txMeta.waitForTimestamp = true;
            txMeta.flushPartialPacket = true;
            if (useComposite)
                composite->StreamTx(rxSamples, samplesRead, &txMeta);
            else
                device->StreamTx(chipIndex, rxSamples, samplesRead, &txMeta);
        }

        // process samples
        totalSamplesReceived += samplesRead;
        if (!rxFilename.empty())
        {
            rxFile.write(reinterpret_cast<char*>(rxSamples[0]), samplesRead * sizeof(lime::complex16_t));
        }

        t2 = std::chrono::high_resolution_clock::now();
        const bool doUpdate = t2 - t1 > std::chrono::milliseconds(500);
        if (doUpdate)
            t1 = t2;

        if (doUpdate)
        {
            if (showFFT)
            {
                for (unsigned i = 0; i < fftSize; ++i)
                {
                    m_fftCalcIn[i].r = rxSamples[0][i].real() / 32768.0;
                    m_fftCalcIn[i].i = rxSamples[0][i].imag() / 32768.0;
                }
                kiss_fft(
                    m_fftCalcPlan, reinterpret_cast<kiss_fft_cpx*>(&m_fftCalcIn), reinterpret_cast<kiss_fft_cpx*>(&m_fftCalcOut));
                for (unsigned int i = 0; i < fftSize; ++i)
                {
                    float amplitude =
                        ((m_fftCalcOut[i].r * m_fftCalcOut[i].r + m_fftCalcOut[i].i * m_fftCalcOut[i].i) / (fftSize * fftSize));

                    float output = amplitude > 0 ? 10 * log10(amplitude) : -150;
                    fftBins[i] = output;
                    // exlude DC from amplitude comparison, the 0 bin
                    if (output > peakAmplitude && i > 0)
                    {
                        peakAmplitude = output;
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
                fftplot.SubmitData(fftBins);
#endif
            }
            else
            {
                std::cerr << "Samples received: " << totalSamplesReceived << endl;
            }
        }

#ifdef USE_GNU_PLOT
        if (showConstelation && doUpdate)
            constellationplot.SubmitData(rxData[0]);
#endif
    }
#ifdef USE_GNU_PLOT
    // some sleep for GNU plot data to flush, otherwise sometimes cout spams  gnuplot "invalid command"
    this_thread::sleep_for(std::chrono::milliseconds(500));
#endif
    if (useComposite)
        composite->StreamStop();
    else
        device->StreamStop(chipIndex);

    if (composite)
        delete composite;
    DeviceRegistry::freeDevice(device);

    rxFile.close();
    return 0;
}
