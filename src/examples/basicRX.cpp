/**
    @file   basicRX.cpp
    @author Lime Microsystems (www.limemicro.com)
    @brief  minimal RX example
 */
#include "limesuiteng/limesuiteng.hpp"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <string_view>
#include <cmath>
#include <signal.h>
#include "kiss_fft.h"
#include "args.hxx"
#include "common.h"
#ifdef USE_GNU_PLOT
    #include "gnuPlotPipe.h"
#endif

using namespace lime;
using namespace std::literals::string_view_literals;

double frequencyLO = 1.9e9;
float sampleRate = 10e6;
static uint8_t chipIndex = 0; // device might have several RF chips

bool stopProgram(false);
void intHandler(int dummy)
{
    std::cout << "Stopping\n"sv;
    stopProgram = true;
}

static LogLevel logVerbosity = LogLevel::Verbose;
static void LogCallback(LogLevel lvl, const std::string& msg)
{
    if (lvl > logVerbosity)
        return;
    std::cout << msg << std::endl;
}

int main(int argc, char** argv)
{
    // clang-format off
    args::ArgumentParser            parser("basicRX - minimal RX example", "");
    args::ValueFlag<std::string>    deviceFlag(parser, "device", "Specifies which device to use", {'d', "device"}, "");

    args::Group                     rxGroup(parser, "Receiver"); // NOLINT(cppcoreguidelines-slicing)
    args::ValueFlag<double>         rxloFlag(parser, "rxlo", "Receiver center frequency in Hz", {"rxlo"});
    args::ValueFlag<std::string>    rxpathFlag(parser, "antenna name", "Receiver antenna path", {"rxpath"}, "");
    // clang-format on

    try
    {
        parser.ParseCLI(argc, argv);
    } catch (const args::Help&)
    {
        std::cout << parser;
        return EXIT_SUCCESS;
    } catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    lime::registerLogHandler(LogCallback);
    float peakAmplitude = -1000, peakFrequency = 0;

    const std::string devName = args::get(deviceFlag);
    SDRDevice* device = ConnectToFilteredOrDefaultDevice(devName);
    if (!device)
    {
        std::cout << "Failed to connect to device"sv << std::endl;
        return -1;
    }
    std::cout << "Connected to device: " << device->GetDescriptor().name << std::endl;

    device->SetMessageLogCallback(LogCallback);
    device->Init();

    const auto& chipDescriptor = device->GetDescriptor().rfSOC[chipIndex];

    // Default RX path when none is provided via the command lime.
    //
    // TODO: Choose antenna which makes sense for the device.
    // For example, rxPath=2 is LNAL_NC for LimeSDR Mini, which will result in
    // effectively no signal in the received samples.
    int rxPath = 2;

    const std::string rxAntennaName = args::get(rxpathFlag);
    if (!rxAntennaName.empty())
    {
        rxPath = AntennaNameToIndex(chipDescriptor.pathNames.at(TRXDir::Rx), rxAntennaName);
        if (rxPath < 0)
        {
            DeviceRegistry::freeDevice(device);
            return EXIT_FAILURE;
        }
    }

    std::cout << "Using antenna "sv << chipDescriptor.pathNames.at(TRXDir::Rx).at(rxPath) << std::endl;

    if (rxloFlag)
    {
        frequencyLO = args::get(rxloFlag);
    }
    std::cout << "Receiver center frequency: " << std::fixed << std::setprecision(3) << frequencyLO / 1e6 << " MHz" << std::endl;

    // RF parameters
    SDRConfig config;
    config.channel[0].rx.enabled = true;
    config.channel[0].rx.centerFrequency = frequencyLO;
    config.channel[0].rx.sampleRate = sampleRate;
    config.channel[0].rx.oversample = 2;
    config.channel[0].rx.lpf = 0;
    config.channel[0].rx.path = rxPath;
    config.channel[0].rx.calibrate = false;
    config.channel[0].rx.testSignal.enabled = false;

    config.channel[0].tx.enabled = false;
    config.channel[0].tx.sampleRate = sampleRate;
    config.channel[0].tx.oversample = 2;
    config.channel[0].tx.path = 0;
    config.channel[0].tx.centerFrequency = frequencyLO - 1e6;
    config.channel[0].tx.testSignal.enabled = false;

    std::cout << "Configuring device ...\n"sv;
    try
    {
        auto t1 = std::chrono::high_resolution_clock::now();
        device->Configure(config, chipIndex);
        auto t2 = std::chrono::high_resolution_clock::now();
        std::cout << "SDR configured in "sv << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << "ms\n"sv;

        // Samples data streaming configuration
        StreamConfig stream;
        stream.channels[TRXDir::Rx] = { 0 };
        stream.format = DataFormat::F32;
        stream.linkFormat = DataFormat::I16;

        device->StreamSetup(stream, chipIndex);
        device->StreamStart(chipIndex);

    } catch (std::runtime_error& e)
    {
        std::cout << "Failed to configure settings: "sv << e.what() << std::endl;
        return -1;
    } catch (std::logic_error& e)
    {
        std::cout << "Failed to configure settings: "sv << e.what() << std::endl;
        return -1;
    }

    std::cout << "Stream started ...\n"sv;
    signal(SIGINT, intHandler);

    const unsigned int fftSize = 16384;
    complex32f_t** rxSamples = new complex32f_t*[2]; // allocate two channels for simplicity
    for (int i = 0; i < 2; ++i)
        rxSamples[i] = new complex32f_t[fftSize];

#ifdef USE_GNU_PLOT
    GNUPlotPipe gp;
    gp.write("set size square\n set xrange[-1:1]\n set yrange[-1:1]\n");
#endif

    auto startTime = std::chrono::high_resolution_clock::now();
    auto t1 = startTime;
    auto t2 = t1;

    uint64_t totalSamplesReceived = 0;

    std::vector<float> fftBins(fftSize);
    kiss_fft_cfg m_fftCalcPlan = kiss_fft_alloc(fftSize, 0, nullptr, nullptr);
    kiss_fft_cpx m_fftCalcIn[fftSize];
    kiss_fft_cpx m_fftCalcOut[fftSize];

    StreamMeta rxMeta{};
    while (std::chrono::high_resolution_clock::now() - startTime < std::chrono::seconds(10) && !stopProgram)
    {
        uint32_t samplesRead = device->StreamRx(chipIndex, rxSamples, fftSize, &rxMeta);
        if (samplesRead == 0)
            continue;

        // process samples
        totalSamplesReceived += samplesRead;
        for (unsigned i = 0; i < fftSize; ++i)
        {
            m_fftCalcIn[i].r = rxSamples[0][i].real();
            m_fftCalcIn[i].i = rxSamples[0][i].imag();
        }
        kiss_fft(m_fftCalcPlan, reinterpret_cast<kiss_fft_cpx*>(&m_fftCalcIn), reinterpret_cast<kiss_fft_cpx*>(&m_fftCalcOut));
        for (unsigned int i = 1; i < fftSize; ++i)
        {
            float output =
                10 * log10(((m_fftCalcOut[i].r * m_fftCalcOut[i].r + m_fftCalcOut[i].i * m_fftCalcOut[i].i) / (fftSize * fftSize)));
            fftBins[i] = output;
            if (output > peakAmplitude)
            {
                peakAmplitude = output;
                peakFrequency = i * sampleRate / fftSize;
            }
        }
        if (peakFrequency > sampleRate / 2)
            peakFrequency = peakFrequency - sampleRate;
        t2 = std::chrono::high_resolution_clock::now();
        if (t2 - t1 > std::chrono::seconds(1))
        {
            t1 = t2;
            std::cout << "Samples received: " << totalSamplesReceived << ", Peak amplitude: " << std::fixed << std::setprecision(2)
                      << peakAmplitude << "dBFS @ " << std::setprecision(3) << (frequencyLO + peakFrequency) / 1e6 << std::endl;
#ifdef USE_GNU_PLOT
            gp.write("plot '-' with points\n");
            for (uint32_t j = 0; j < samplesRead; ++j)
                gp.writef("%f %f\n", rxSamples[0][j].real(), rxSamples[0][j].imag());
            gp.write("e\n");
            gp.flush();
#endif
            peakAmplitude = -1000;
        }
    }
    DeviceRegistry::freeDevice(device);

    for (int i = 0; i < 2; ++i)
        delete[] rxSamples[i];
    delete[] rxSamples;
    free(m_fftCalcPlan);
    return 0;
}
