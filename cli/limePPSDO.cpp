#include "common.h"

#include "limesuiteng/PPSDO.h"
#include "limesuiteng/SDRDescriptor.h"
#include "DeviceTreeNode.h"
#include "limesuiteng/ToString.h"

#include <array>
#include <atomic>
#include <cinttypes>
#include <csignal>
#include <cstdio>
#include "args.hxx"

struct MonitorResults {
    uint32_t dumpCount{ 0 };
    lime::PPSDO::Status status;
};

using namespace std;
using namespace lime;
using namespace lime::cli;

static std::atomic<bool> cleanUp(false);
static void keyBoardInt(int param)
{
    cleanUp = true;
}

/// Finds the first PPSDO module in the device tree.
static PPSDO* FindPPSDO(const std::shared_ptr<DeviceTreeNode>& node)
{
    if (!node)
        return nullptr;
    if (node->moduleClass == "PPSDO"sv)
        return static_cast<PPSDO*>(node->module);
    for (const auto& child : node->children)
    {
        PPSDO* ppsdo = FindPPSDO(child);
        if (ppsdo)
            return ppsdo;
    }
    return nullptr;
}

static const array<string, 4> accuracyLevelList = { "Disabled/Lowest", "1s Tune", "2s Tune", "3s Tune (Highest)" };

static string stateToStr(PPSDO::TuneState state)
{
    switch (state)
    {
    case PPSDO::TuneState::Idle:
        return "Idle"s;
    case PPSDO::TuneState::CoarseTune:
        return "Coarse tune"s;
    case PPSDO::TuneState::FineTune:
        return "Fine tune"s;
    default:
        return "Unknown"s;
    }
}

static string accToStr(uint8_t accuracy)
{
    if (accuracy < 4)
        return accuracyLevelList[accuracy];

    return string("Unknown("s + std::to_string(accuracy) + ")"s);
}

// ###############################################
// #### limePPSDO helper function definitions ####
// ###############################################

static int printHeader()
{
    string header = "|    Dump    | Enabled |   1s Error   |  10s Error  |  100s Error  |  DAC Value  |    State    |      "
                    "Accuracy      | TPulse |";
    cout << setw(header.size()) << setfill('-') << "-" << setfill(' ') << endl;
    cout << header << endl;
    cout << setw(header.size()) << setfill('-') << "-" << setfill(' ') << endl;
    return header.size();
}

static string formatToFit(MonitorResults* results, int length)
{
    string formatedMessage(length, ' ');
    const PPSDO::Status& status = results->status;
    const char* format =
        "  %10" PRIu32 "    %5s    %11" PRIi32 "    %11" PRIi32 "   %11" PRIi32 "    0x%08" PRIX32 "    %11s   %18s   %5s";
    snprintf(formatedMessage.data(),
        formatedMessage.size(),
        format,
        results->dumpCount,
        status.enabled ? "true" : "false",
        status.error1s,
        status.error10s,
        status.error100s,
        status.dacValue,
        stateToStr(status.state).c_str(),
        accToStr(status.accuracy).c_str(),
        status.ppsActive ? "true" : "false");

    return formatedMessage;
}

static OpStatus runMonitoring(PPSDO* pDriver, uint32_t numDumps, std::chrono::milliseconds delay, int banner_interval)
{
    OpStatus status = OpStatus::Success;
    MonitorResults results;
    cout << "Monitoring PPSDO regulation loop (press Ctrl+C to stop):\n";
    uint32_t dumpCount = 0;
    bool continueLoop = false;
    int headerLength = 0;
    if (numDumps == 0)
        headerLength = printHeader();

    do
    {
        status = pDriver->GetStatus(results.status);
        if (status != OpStatus::Success)
        {
            lime::error("ERROR: Monitoring mode failed to read PPSDO status with error: "s + ToString(status));
            return status;
        }

        if ((dumpCount % banner_interval == 0 && dumpCount != numDumps))
            headerLength = printHeader();

        results.dumpCount = ++dumpCount;
        cout << formatToFit(&results, headerLength) << endl;

        std::this_thread::sleep_for(delay);
        continueLoop = (dumpCount < numDumps || numDumps == 0);

    } while (continueLoop && !cleanUp);

    cout << "Monitoring finished\n";
    return status;
}

static void dumpRegisters(PPSDO* pDriver, int numberOfDumps, double delay)
{
    cout << "Note: Raw register dump not available via named CSRs, use --check for full status\n";
}

static OpStatus resetPPSDO(PPSDO* pDriver, std::chrono::milliseconds rstDelay)
{
    OpStatus status = OpStatus::Success;
    cout << "Resetting PPSDO...\n";
    status = pDriver->Enable(false);
    if (status != OpStatus::Success)
    {
        lime::error("ERROR: Failed to reset PPSDO with error: "s + ToString(status));
        return status;
    }

    std::this_thread::sleep_for(rstDelay);
    status = pDriver->Enable(true);
    if (status != OpStatus::Success)
    {
        lime::error("ERROR: Failed to reset PPSDO with error: "s + ToString(status));
        return status;
    }
    cout << "PPSDO reset complete (re-enabled)\n";
    return status;
}

static OpStatus enablePPSDO(PPSDO* pDriver, double clk, double ppm)
{
    PPSDO::Config config;
    config.clockFrequency = clk * 1e6;
    config.tolerancePPM = ppm;
    lime::info(
        "INFO: Selected frequency freq = "s + to_string(config.clockFrequency) + " Hz, tolerance "s + to_string(ppm) + " ppm;"s);

    OpStatus status = pDriver->Configure(config);
    if (status != OpStatus::Success)
    {
        lime::error("ERROR: PPSDO configuration failed with error: "s + ToString(status));
        return status;
    }

    status = pDriver->Enable(true);
    if (status != OpStatus::Success)
    {
        lime::error("ERROR: PPSDO enable failed with error: "s + ToString(status));
        return status;
    }

    cout << "PPSDO enabled: " << clk << " MHz, " << ppm << " ppm tolerance.\n";
    return status;
}

static OpStatus disablePPSDO(PPSDO* pDriver)
{
    OpStatus status = pDriver->Enable(false);
    if (status != OpStatus::Success)
    {
        lime::error("ERROR: PPSDO disable failed with error: "s + ToString(status));
        return status;
    }

    cout << "PPSDO disabled\n";
    return status;
}

int main(int argc, char** argv)
{
    signal(SIGINT, keyBoardInt);
    // clang-format off
   args::ArgumentParser                    parser("limePPSDO - PPS Disciplined Oscilator control and monitoring. Currently supported PPS sources - GPS", "");
   args::HelpFlag                          help(parser, "help", "This help", {'h', "help"});

   args::Group                             commands(parser, "COMMANDS", args::Group::Validators::AtLeastOne); // NOLINT(cppcoreguidelines-slicing)
   args::Flag                              check(commands, "check", "Run monitoring mode", {"check"} );
   args::Flag                              dump(commands, "dump", "Dump registers", {"dump"});
   args::Flag                              reset(commands, "reset", "Reset PPSDO", {"reset"});
   args::Flag                              enable(commands, "enable", "Configure and enable PPSDO", {"enable"});
   args::Flag                              disable(commands, "disable", "Disable PPSDO", {"disable"});

   args::Group                             arguments(parser, "ARGUMENTS", args::Group::Validators::DontCare, args::Options::Global); // NOLINT(cppcoreguidelines-slicing)
   args::ValueFlag<std::string>            logFlag(arguments, "", "Enable additional device, API and limePPSDO app log output. Log verbosity: info, warning, error, verbose, debug. Log level \'info\' prints intermediate calculations. Log level \'debug\' prints detailed CSR register R/W operations.", {'l', "log"}, "error");
   args::ValueFlag<std::string>            deviceFlag(arguments, "name", "Specifies which device to use", {"device"}, "");
   args::ValueFlag<uint32_t>               num(arguments, "iter", "Number of iterations (for --check: 0 for infinite; for --dump: default 1 if not specified)", {'n', "num"}, 0);
   args::ValueFlag<double>                 delay(arguments, "time", "Delay between iterations (seconds, for --check and --dump)", {'d', "delay"}, 1.0);
   args::ValueFlag<int>                    banner(arguments, "interval", "Banner repeat interval (for --check)", {'b', "banner"}, 10);
   args::ValueFlag<double>                 reset_delay(arguments, "time", "Delay after disable before re-enable (seconds, for --reset)", {'r', "reset-delay"}, 2.0);
   args::ValueFlag<double>                 clk_freq(arguments, "MHz", "Clock frequency in MHz (for --enable, not needed by devices that know the clock they measure, e.g. LimeSDR Micro)", {'c', "clk-freq"}, 0);
   args::ValueFlag<double>                 ppm(arguments, "ppm", "Tolerance in ppm (for --enable, 0 selects the device default if it has one)", {'p', "ppm"}, 0);
   // clang-format on

   try
   {
       parser.ParseCLI(argc, argv);

       if (check)
       {
           int mbanner = args::get(banner);
           if (mbanner == 0)
               throw args::UsageError("ERROR: Check command --banner flag cannot be set to 0!");
       }

    } catch (args::Help&)
    {
        cout << parser << endl;
        return EXIT_SUCCESS;
    } catch (args::ValidationError& e)
    {
        std::cout << "ERROR: Select atleast one COMMAND from the list!" << endl;
        cout << parser << endl;
        return EXIT_SUCCESS;
    } catch (args::UsageError& e)
    {
        std::cout << e.what() << endl;
        return EXIT_SUCCESS;
    } catch (const std::exception& e)
    {
        cerr << parser << endl;
        cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    std::string devName = args::get(deviceFlag);

    auto handles = DeviceRegistry::enumerate();
    if (handles.size() == 0)
    {
        cerr << "No devices found"sv << endl;
        return EXIT_FAILURE;
    }

    SDRDevice* device = ConnectToFilteredOrDefaultDevice(devName);
    if (!device)
    {
        cerr << "ERROR: Failed to connect to SDR device!\n";
        return EXIT_FAILURE;
    }

    PPSDO* driver = FindPPSDO(device->GetDescriptor().socTree);
    if (!driver)
    {
        cerr << "ERROR: Selected SDR device does not support PPSDO!\n";
        DeviceRegistry::freeDevice(device);
        return EXIT_FAILURE;
    }

    logVerbosity = strToLogLevel(args::get(logFlag));
    device->SetMessageLogCallback(lime::cli::LogCallback);
    lime::registerLogHandler(lime::cli::LogCallback);
    registerLogHandler(lime::cli::CStyleLogCallback);

    OpStatus runStatus = OpStatus::Success;
    try
    {
        if (reset)
        {
            int resetDelay = static_cast<int>(args::get(reset_delay) * 1000.0);
            runStatus = resetPPSDO(driver, std::chrono::milliseconds(resetDelay));
        }

        if (enable && runStatus == OpStatus::Success)
        {
            runStatus = enablePPSDO(driver, args::get(clk_freq), args::get(ppm));
        }

        if (check && runStatus == OpStatus::Success)
        {
            int convDelay = static_cast<int>(args::get(delay) * 1000.0);
            runStatus = runMonitoring(driver, args::get(num), std::chrono::milliseconds(convDelay), args::get(banner));
        }

        if (dump && runStatus == OpStatus::Success)
        {
            dumpRegisters(driver, args::get(num), args::get(delay));
        }

        if (disable && runStatus == OpStatus::Success)
        {
            runStatus = disablePPSDO(driver);
        }

    } catch (const std::exception& e)
    {
        DeviceRegistry::freeDevice(device);
        std::cerr << e.what() << '\n';
        return EXIT_FAILURE;
    }

    if (cleanUp)
        cerr << "Keyboard interrupt Ctrl+C detected! Cleaning up ...\n";

    DeviceRegistry::freeDevice(device);
    return EXIT_SUCCESS;
}