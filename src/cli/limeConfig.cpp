#include "cli/common.h"

#include "limesuiteng/LMS7002M.h"
#include "limesuiteng/Logger.h"
#include <cassert>
#include <cstring>
#include <getopt.h>
#include <string_view>
#include "args/args.hxx"

using namespace std;
using namespace lime;
using namespace std::literals::string_literals;
using namespace std::literals::string_view_literals;

static std::vector<int> ParseIntArray(args::NargsValueFlag<int>& flag)
{
    std::vector<int> numbers;
    for (const auto& number : args::get(flag))
        numbers.push_back(number);
    return numbers;
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
    std::cout << msg << std::endl;
}

static int AntennaNameToIndex(const std::vector<std::string>& antennaNames, const std::string& name)
{
    if (name.empty())
        return -1;

    for (size_t j = 0; j < antennaNames.size(); ++j)
    {
        if (antennaNames[j] == name)
            return j;
    }
    std::cerr << "Antenna ("sv << name << " not found. Available:"sv << std::endl;
    for (const auto& iter : antennaNames)
        std::cerr << "\t\""sv << iter << "\""sv << std::endl;
    return -1;
}

int main(int argc, char** argv)
{
    // clang-format off
    args::ArgumentParser            parser("limeConfig - SDR parameters configuring", "");
    args::HelpFlag                  help(parser, "help", "This help", {'h', "help"});
    args::ValueFlag<std::string>    deviceFlag(parser, "device", "Specifies which device to use", {'d', "device"}, "");
    args::NargsValueFlag<int>       chipFlag(parser, "chip", "Selects destination chip/chips", {'c', "chips"}, {1, static_cast<size_t>(-1)});
    args::ValueFlag<std::string>    logFlag(parser, "log level", "Log verbosity: info, warning, error, verbose, debug", {'l', "log"}, "error");
    args::Flag                      initializeFlag(parser, "", "Reset and initialize entire device", {'i', "initialize"});

    args::ValueFlag<double>         refclkFlag(parser, "reference clock", "Reference clock in Hz", {"refclk"});
    args::ValueFlag<double>         samplerateFlag(parser, "sample rate", "Sampling rate in Hz", {"samplerate"});

    args::Group                     rxGroup(parser, "Receiver");
    args::ValueFlag<bool>           rxenFlag(parser, "rx enable", "Enable receiver [0, 1]", {"rxen"});
    args::ValueFlag<double>         rxloFlag(parser, "rxlo", "Receiver center frequency in Hz", {"rxlo"});
    args::ValueFlag<std::string>    rxpathFlag(parser, "antenna name", "Receiver antenna path", {"rxpath"}, "");
    args::ValueFlag<double>         rxlpfFlag(parser, "Hz", "Receiver low pass filter bandwidth in Hz", {"rxlpf"});
    args::ValueFlag<uint8_t>        rxoversampleFlag(parser, "", "Receiver decimation 1,2,4,8...", {"rxoversample"});
    args::ValueFlag<bool>           rxtestsignalFlag(parser, "", "Enables receiver test signal if available", {"rxtestsignal"});

    args::Group                     txGroup(parser, "Transmitter");
    args::ValueFlag<bool>           txenFlag(parser, "tx enable", "Enable transmitter", {"txen"});
    args::ValueFlag<double>         txloFlag(parser, "txlo", "Transmitter center frequency in Hz", {"txlo"});
    args::ValueFlag<std::string>    txpathFlag(parser, "antenna name", "Transmitter antenna path", {"txpath"}, "");
    args::ValueFlag<double>         txlpfFlag(parser, "Hz", "Transmitter low pass filter bandwidth in Hz", {"txlpf"});
    args::ValueFlag<uint8_t>        txoversampleFlag(parser, "", "Transmitter interpolation 1,2,4,8...", {"txoversample"});
    args::ValueFlag<bool>           txtestsignalFlag(parser, "", "Enables transmitter test signal if available", {"txtestsignal"});

    args::ValueFlag<std::string>    iniFlag(parser, "", "Path to LMS7002M .ini configuration file to use as a base", {"ini"}, "");
    // clang-format on

    try
    {
        parser.ParseCLI(argc, argv);
    } catch (const args::Help&)
    {
        cout << parser;
        return EXIT_SUCCESS;
    } catch (const std::exception& e)
    {
        cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    if (argc == 1)
    {
        std::cout << parser;
        return EXIT_SUCCESS;
    }

    const std::string devName = args::get(deviceFlag);
    const bool initializeBoard = initializeFlag;
    const std::string iniFilename = args::get(iniFlag);
    const std::string rxAntennaName = args::get(rxpathFlag);
    ;
    const std::string txAntennaName = args::get(txpathFlag);

    logVerbosity = strToLogLevel(args::get(logFlag));
    std::vector<int> chipIndexes = ParseIntArray(chipFlag);

    SDRConfig config;
    config.channel[0].rx.oversample = 2;
    config.channel[0].tx.oversample = 2;

    if (samplerateFlag)
    {
        double sampleRate = args::get(samplerateFlag);
        config.channel[0].rx.sampleRate = sampleRate;
        config.channel[0].tx.sampleRate = sampleRate;
    }
    // clang-format off
    if (refclkFlag)         config.referenceClockFreq = args::get(refclkFlag);
    if (rxenFlag)           config.channel[0].rx.enabled = args::get(rxenFlag);
    if (rxloFlag)           config.channel[0].rx.centerFrequency = args::get(rxloFlag);
    if (rxlpfFlag)          config.channel[0].rx.lpf = args::get(rxlpfFlag);
    if (rxoversampleFlag)   config.channel[0].rx.oversample = args::get(rxoversampleFlag);
    if (rxtestsignalFlag)   config.channel[0].rx.testSignal.enabled = args::get(rxtestsignalFlag);

    if (txenFlag)           config.channel[0].tx.enabled = args::get(txenFlag);
    if (txloFlag)           config.channel[0].tx.centerFrequency = args::get(txloFlag);
    if (txlpfFlag)          config.channel[0].tx.lpf = args::get(txlpfFlag);
    if (txoversampleFlag)   config.channel[0].tx.oversample = args::get(txoversampleFlag);
    if (txtestsignalFlag)   config.channel[0].tx.testSignal.enabled = args::get(txtestsignalFlag);
    // clang-format on

    auto handles = DeviceRegistry::enumerate();
    if (handles.size() == 0)
    {
        cerr << "No devices found"sv << endl;
        return EXIT_FAILURE;
    }

    SDRDevice* device = ConnectToFilteredOrDefaultDevice(devName);
    if (!device)
        return EXIT_FAILURE;

    // if chip index is not specified and device has only one, use it by default
    if (chipIndexes.empty() && device->GetDescriptor().rfSOC.size() == 1)
        chipIndexes.push_back(0);

    device->SetMessageLogCallback(LogCallback);

    try
    {
        if (initializeBoard)
            device->Init();

        std::string configFilepath;
        if (!iniFilename.empty())
        {
            std::string configFilepath = ""s;
            config.skipDefaults = true;
            std::string_view cwd{ argv[0] };
            const size_t slash0Pos = cwd.find_last_of("/\\"sv);
            if (slash0Pos != std::string_view::npos)
            {
                cwd = cwd.substr(0, slash0Pos - 1);
            }

            if (iniFilename[0] != '/') // is not global path
                configFilepath = std::string{ cwd } + "/"s;

            configFilepath += iniFilename;
        }

        for (int moduleId : chipIndexes)
        {
            LMS7002M* chip = static_cast<LMS7002M*>(device->GetInternalChip(moduleId));
            if (!chip)
            {
                DeviceRegistry::freeDevice(device);
                cerr << "Failed to get internal chip: "sv << moduleId << endl;
                return EXIT_FAILURE;
            }
            if (!configFilepath.empty() && chip->LoadConfig(configFilepath) != OpStatus::Success)
            {
                cerr << "Error loading file: "sv << configFilepath << endl;
                return EXIT_FAILURE;
            }

            const auto& chipDescriptor = device->GetDescriptor().rfSOC[moduleId];

            if (!rxAntennaName.empty())
            {
                int rxPathIndex = AntennaNameToIndex(chipDescriptor.pathNames.at(TRXDir::Rx), rxAntennaName);
                if (rxPathIndex < 0)
                    return EXIT_FAILURE;
                config.channel[0].rx.path = rxPathIndex;
            }
            if (!txAntennaName.empty())
            {
                int txPathIndex = AntennaNameToIndex(chipDescriptor.pathNames.at(TRXDir::Tx), txAntennaName);
                if (txPathIndex < 0)
                    return EXIT_FAILURE;
                config.channel[0].tx.path = txPathIndex;
            }

            // set all channels to identical configuration
            for (int i = 0; i < chipDescriptor.channelCount; ++i)
                config.channel[i] = config.channel[0];

            if (device->Configure(config, moduleId) != OpStatus::Success)
            {
                cerr << "Failed to configure device (chip"sv << moduleId << ")"sv << std::endl;
                return EXIT_FAILURE;
            }
        }
    } catch (std::runtime_error& e)
    {
        DeviceRegistry::freeDevice(device);
        cerr << "Config failed: "sv << e.what() << endl;
        return EXIT_FAILURE;
    } catch (std::logic_error& e)
    {
        DeviceRegistry::freeDevice(device);
        cerr << "Config failed: "sv << e.what() << endl;
        return EXIT_FAILURE;
    }

    DeviceRegistry::freeDevice(device);

    return EXIT_SUCCESS;
}
