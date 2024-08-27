#include "common.h"
#include "limesuiteng/OpStatus.h"
#include "limesuiteng/VersionInfo.h"
#include "OEMTesting.h"
#include <assert.h>
#include <cstring>
#include <sstream>
#include <ctime>
#include <string_view>
#include <filesystem>
#include "utilities/toString.h"
#include "args.hxx"

#include "rang.hpp" // terminal colors

using namespace std;
using namespace lime;
using namespace std::literals::string_literals;
using namespace std::literals::string_view_literals;

static LogLevel logVerbosity = LogLevel::Error;
static LogLevel strToLogLevel(const std::string_view str)
{
    if ("debug"sv == str)
        return LogLevel::Debug;
    else if ("verbose"sv == str)
        return LogLevel::Verbose;
    else if ("info"sv == str)
        return LogLevel::Info;
    else if ("warning"sv == str)
        return LogLevel::Warning;
    else if ("error"sv == str)
        return LogLevel::Error;
    else if ("critical"sv == str)
        return LogLevel::Critical;
    return LogLevel::Error;
}

static void LogCallback(LogLevel lvl, const std::string& msg)
{
    if (lvl > logVerbosity)
        return;
    cerr << msg << endl;
}

class PrintOEMTestReporter : public OEMTestReporter
{
  public:
    PrintOEMTestReporter(uint64_t serialNumber, const std::filesystem::path& reportFilename)
    {
        if (!reportFilename.empty())
            fileOutput.open(reportFilename, std::fstream::out | std::fstream::app);
    };
    ~PrintOEMTestReporter() override
    {
        if (fileOutput.is_open())
        {
            int fileSize = fileOutput.tellg();
            if (fileSize == 0) // if file empty add column headers
            {
                for (const auto& h : headers)
                    fileOutput << h << ",";
                fileOutput << std::endl;
            }
            for (const auto& v : values)
                fileOutput << v << ",";
            fileOutput << std::endl;
        }
        fileOutput.close();
    }
    void OnStart(OEMTestData& test, const std::string& testName = std::string()) override
    {
        std::cerr << Indent() << "=== " << test.title << " ===" << std::endl;
        ++indentLevel;
    }
    void OnStepUpdate(OEMTestData& test, const std::string& text = std::string()) override
    {
        std::cerr << Indent() << text << std::endl;
    }
    void OnSuccess(OEMTestData& test) override
    {
        --indentLevel;
        std::cerr << Indent() << "=== " << rang::fg::green << test.title << " - PASSED" << rang::fg::reset << " ===" << std::endl;
    }
    void OnFail(OEMTestData& test, const std::string& reasonText = std::string()) override
    {
        assert(test.status != OpStatus::Success);
        --indentLevel;
        std::cerr << Indent() << "=== " << rang::fg::red << test.title << " - FAILED";

        if (!reasonText.empty())
            std::cerr << " (" << reasonText << ")";
        std::cerr << rang::fg::reset << " ===" << std::endl;
    }
    void ReportColumn(const std::string& header, const std::string& value) override
    {
        headers.push_back(header);
        values.push_back(value);
    }

  private:
    std::string Indent()
    {
        std::stringstream ss;
        for (int i = 0; i < indentLevel; ++i)
            ss << "  ";
        return ss.str();
    }
    int indentLevel{ 0 };
    std::fstream fileOutput;
    std::vector<std::string> headers;
    std::vector<std::string> values;
};

int main(int argc, char** argv)
{
    // clang-format off
    args::ArgumentParser            parser("limeOEM - utility for device testing and custom device specific operations", "");
    args::HelpFlag                  help(parser, "help", "This help", {'h', "help"});
    args::ValueFlag<std::string>    deviceFlag(parser, "device", "Specifies which device to use", {'d', "device"}, "");
    args::ValueFlag<std::string>    logFlag(parser, "level", "Log verbosity levels: error, warning, info, verbose, debug", {'l', "log"}, "error");

    args::ValueFlag<std::string>    reportFileFlag(parser, "", "File to append test results", {'o', "output"}, "");
    args::ValueFlag<uint64_t>       serialNumberFlag(parser, "decimal", "One time programmable serial number to be written to device", {"write-serial-number"}, 0);
    args::Flag                      runTestsFlag(parser, "", "Run tests to check device functionality", {"test"});
    args::Flag                      showVersion(parser, "", "Print software version", {"version"});
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

    if (showVersion)
    {
        cerr << GetLibraryVersion() << endl;
        if (argc == 2)
            return EXIT_SUCCESS;
    }

    logVerbosity = strToLogLevel(args::get(logFlag));
    const std::string devName = args::get(deviceFlag);
    const std::string reportFilename = args::get(reportFileFlag);

    SDRDevice* device = ConnectToFilteredOrDefaultDevice(devName);
    if (!device)
        return EXIT_FAILURE;

    device->SetMessageLogCallback(LogCallback);
    lime::registerLogHandler(LogCallback);
    OpStatus result = OpStatus::Success;

    if (serialNumberFlag)
    {
        uint64_t serialNumberArg = args::get(serialNumberFlag);
        OpStatus status = device->WriteSerialNumber(serialNumberArg);
        if (status != OpStatus::Success)
        {
            cerr << "Failed to write serial number." << endl;
            DeviceRegistry::freeDevice(device);
            return EXIT_FAILURE;
        }
    }

    if (runTestsFlag)
    {
        uint64_t serialNumber = device->GetDescriptor().serialNumber;
        stringstream ss;
        ss << "Board serial number: " << serialNumber << " (0x" << std::setw(2) << std::setfill('0');
        for (int i = sizeof(serialNumber) - 1; i >= 0; --i)
            ss << hex << ((serialNumber >> 8 * i) & 0xFF);
        ss << ")" << endl;
        cerr << ss.rdbuf();

        PrintOEMTestReporter reporter(serialNumber, reportFilename);
        reporter.ReportColumn("S/N", std::to_string(serialNumber));
        result = device->OEMTest(&reporter);

        if (result == OpStatus::Success)
            cerr << rang::fg::green << "OEM TEST PASSED";
        else
            cerr << rang::fg::red << "OEM TEST FAILED";
        cerr << rang::fg::reset << endl;
    }

    DeviceRegistry::freeDevice(device);
    return result != OpStatus::Success ? EXIT_FAILURE : EXIT_SUCCESS;
}
