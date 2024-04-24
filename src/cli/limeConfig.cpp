#include "cli/common.h"

#include "limesuiteng/LMS7002M.h"
#include "limesuiteng/Logger.h"
#include <cassert>
#include <cstring>
#include <getopt.h>
#include <string_view>

using namespace std;
using namespace lime;
using namespace std::literals::string_literals;
using namespace std::literals::string_view_literals;

static std::vector<int> ParseIntArray(const std::string& str)
{
    std::vector<int> numbers;
    size_t parsed = 0;
    while (parsed < str.length())
    {
        try
        {
            int nr = stoi(&str[parsed]);
            numbers.push_back(nr);
            size_t next = str.find_first_of(',', parsed);
            if (next == string::npos)
                return numbers;
            else
                parsed = next + 1;
        } catch (...)
        {
            return numbers;
        }
    }
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

static int printHelp(void)
{
    cerr << "limeConfig [options]" << endl;
    cerr << "    -h, --help\t\t\t This help" << endl;
    cerr << "    -d, --device <name>\t\t Specifies which device to use" << endl;
    cerr << "    -c, --chip <name>\t\t Selects destination chip" << endl;
    cerr << "    -l, --log\t\t Log verbosity: info, warning, error, verbose, debug" << endl;
    cerr << "    -i, --initialize\t\t Reset and initialize entire device" << endl;

    cerr << "    --refclk\t\t Reference clock in Hz" << endl;
    cerr << "    --samplerate\t Sampling rate in Hz" << endl;
    cerr << "    --rxen=[0,1]\t Enable receiver" << endl;
    cerr << "    --rxlo\t\t Receiver center frequency in Hz" << endl;
    cerr << "    --rxpath\t\t Receiver antenna path" << endl;
    cerr << "    --rxlpf\t\t Receiver low pass filter bandwidth in Hz" << endl;
    cerr << "    --rxoversample\t Receiver decimation 1,2,4,8..." << endl;
    cerr << "    --rxtestsignal=[0,1]\t Enables receiver test signal if available" << endl;

    cerr << "    --txen=[0,1]\t\t Enable transmitter" << endl;
    cerr << "    --txlo\t\t Transmitter center frequency in Hz" << endl;
    cerr << "    --txpath\t\t Transmitter antenna path" << endl;
    cerr << "    --txlpf\t\t Transmitter low pass filter bandwidth in Hz" << endl;
    cerr << "    --txoversample\t Transmitter interpolation 1,2,4,8..." << endl;
    cerr << "    --txtestsignal=[0,1]\t Enables transmitter test signal if available" << endl;

    cerr << "    --ini\t Path to LMS7002M .ini configuration file to use as a base" << endl;

    return EXIT_SUCCESS;
}

enum Args {
    HELP = 'h',
    DEVICE = 'd',
    CHIP = 'c',
    LOG = 'l',
    INIT = 'i',

    REFCLK = 200,
    SAMPLERATE,
    RXEN,
    RXLO,
    RXPATH,
    RXLPF,
    RXOVERSAMPLE,
    RXTESTSIGNAL,
    TXEN,
    TXLO,
    TXPATH,
    TXLPF,
    TXOVERSAMPLE,
    TXTESTSIGNAL,
    INIFILE
};

static int AntennaNameToIndex(const std::vector<std::string>& antennaNames, const std::string& name)
{
    if (name.empty())
        return -1;

    bool match = false;
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
    std::string_view devName{ ""sv };
    bool initializeBoard = false;
    std::string iniFilename;
    std::string rxAntennaName;
    std::string txAntennaName;
    std::vector<int> chipIndexes;

    SDRConfig config;
    config.channel[0].rx.oversample = 2;
    config.channel[0].tx.oversample = 2;

    static struct option long_options[] = { { "help", no_argument, 0, Args::HELP },
        { "device", required_argument, 0, Args::DEVICE },
        { "chip", required_argument, 0, Args::CHIP },
        { "log", required_argument, 0, Args::LOG },
        { "initialize", no_argument, 0, Args::INIT },
        { "refclk", required_argument, 0, Args::REFCLK },
        { "samplerate", required_argument, 0, Args::SAMPLERATE },
        { "rxen", required_argument, 0, Args::RXEN },
        { "rxlo", required_argument, 0, Args::RXLO },
        { "rxpath", required_argument, 0, Args::RXPATH },
        { "rxlpf", required_argument, 0, Args::RXLPF },
        { "rxoversample", required_argument, 0, Args::RXOVERSAMPLE },
        { "rxtestsignal", required_argument, 0, Args::RXTESTSIGNAL },
        { "txen", required_argument, 0, Args::TXEN },
        { "txlo", required_argument, 0, Args::TXLO },
        { "txpath", required_argument, 0, Args::TXPATH },
        { "txlpf", required_argument, 0, Args::TXLPF },
        { "txoversample", required_argument, 0, Args::TXOVERSAMPLE },
        { "txtestsignal", required_argument, 0, Args::TXTESTSIGNAL },
        { "ini", required_argument, 0, Args::INIFILE },
        { 0, 0, 0, 0 } };

    int long_index = 0;
    int option = 0;
    while ((option = getopt_long(argc, argv, "", long_options, &long_index)) != -1)
    {
        switch (option)
        {
        case Args::HELP:
            return printHelp();
        case Args::DEVICE:
            if (optarg != NULL)
                devName = std::string(optarg);
            break;
        case Args::CHIP:
            if (optarg != NULL)
                chipIndexes = ParseIntArray(optarg);

            if (chipIndexes.empty())
            {
                cerr << "Invalid chip index"sv << endl;
                return -1;
            }
            else
                cerr << "Chip count "sv << chipIndexes.size() << endl;
            break;
        case Args::LOG:
            if (optarg != NULL)
            {
                logVerbosity = strToLogLevel(optarg);
            }
            break;
        case Args::INIT:
            initializeBoard = true;
            break;
        case REFCLK:
            config.referenceClockFreq = stof(optarg);
            break;
        case SAMPLERATE:
            config.channel[0].rx.sampleRate = stof(optarg);
            config.channel[0].tx.sampleRate = config.channel[0].rx.sampleRate;
            break;
        case RXEN:
            config.channel[0].rx.enabled = stoi(optarg) != 0;
            break;
        case RXLO:
            config.channel[0].rx.centerFrequency = stof(optarg);
            break;
        case RXPATH:
            if (optarg != NULL)
                rxAntennaName = std::string(optarg);
            break;
        case RXLPF:
            config.channel[0].rx.lpf = stof(optarg);
            break;
        case RXOVERSAMPLE:
            config.channel[0].rx.oversample = stoi(optarg);
            break;
        case RXTESTSIGNAL:
            config.channel[0].rx.testSignal.enabled = stoi(optarg) != 0;
            break;
        case TXEN:
            config.channel[0].tx.enabled = stoi(optarg) != 0;
            break;
        case TXLO:
            config.channel[0].tx.centerFrequency = stof(optarg);
            break;
        case TXPATH:
            if (optarg != NULL)
                txAntennaName = std::string(optarg);
            break;
        case TXLPF:
            config.channel[0].tx.lpf = stof(optarg);
            break;
        case TXOVERSAMPLE:
            config.channel[0].tx.oversample = stoi(optarg);
            break;
        case TXTESTSIGNAL:
            config.channel[0].tx.testSignal.enabled = stoi(optarg) != 0;
            break;
        case INIFILE:
            if (optarg != NULL)
                iniFilename = std::string(optarg);
            break;
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
