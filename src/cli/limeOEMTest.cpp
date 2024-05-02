#include "cli/common.h"
#include "limesuiteng/OpStatus.h"
#include "OEMTesting.h"
#include <assert.h>
#include <cstring>
#include <sstream>
#include <ctime>
#include <getopt.h>
#include <string_view>
#include <filesystem>
#include "utilities/toString.h"

using namespace std;
using namespace lime;
using namespace std::literals::string_literals;
using namespace std::literals::string_view_literals;

static LogLevel logVerbosity = LogLevel::Error;
static LogLevel strToLogLevel(const std::string& str)
{
    if (str == "debug"s)
        return LogLevel::Debug;
    else if (str == "verbose"s)
        return LogLevel::Verbose;
    else if (str == "error"s)
        return LogLevel::Error;
    else if (str == "warning"s)
        return LogLevel::Warning;
    else if (str == "info"s)
        return LogLevel::Info;
    return LogLevel::Error;
}
static void LogCallback(LogLevel lvl, const std::string& msg)
{
    if (lvl > logVerbosity)
        return;
    cerr << msg << endl;
}

static int printHelp(void)
{
    cerr << "limeOEMTest [options]" << endl;
    cerr << "    -h, --help\t\t\t This help" << endl;
    cerr << "    -d, --device <name>\t\t\t Specifies which device to use" << endl;
    cerr << "    -l, --log\t\t Log verbosity: info, warning, error, verbose, debug" << endl;
    cerr << "    -o, --output\t\t File to append test results" << endl;

    return EXIT_SUCCESS;
}

class PrintOEMTestReporter : public OEMTestReporter
{
  public:
    PrintOEMTestReporter(uint64_t serialNumber, const std::filesystem::path& reportFilename)
        : indentLevel(0)
    {
        if (!reportFilename.empty())
            fileOutput.open(reportFilename, std::fstream::out | std::fstream::app);
    };
    ~PrintOEMTestReporter()
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
    virtual void OnStart(OEMTestData& test, const std::string& testName = std::string()) override
    {
        std::cerr << Indent() << "=== " << test.name << " ===" << std::endl;
        ++indentLevel;
    }
    virtual void OnStepUpdate(OEMTestData& test, const std::string& text = std::string()) override
    {
        std::cerr << Indent() << text << std::endl;
    }
    virtual void OnSuccess(OEMTestData& test) override
    {
        --indentLevel;
        std::cerr << Indent() << "=== " << test.name << " - PASSED"
                  << " ===" << std::endl
                  << std::endl;
    }
    virtual void OnFail(OEMTestData& test, const std::string& reasonText = std::string()) override
    {
        assert(!test.passed);
        --indentLevel;
        std::cerr << Indent() << "=== " << test.name << " - FAILED";

        if (!reasonText.empty())
            std::cerr << " (" << reasonText << ")";
        std::cerr << " ===" << std::endl << std::endl;
    }
    virtual void ReportColumn(const std::string& header, const std::string& value)
    {
        headers.push_back(header);
        values.push_back(value);
    }

  protected:
    std::string Indent()
    {
        std::stringstream ss;
        for (int i = 0; i < indentLevel; ++i)
            ss << "  ";
        return ss.str();
    }
    int indentLevel;
    std::fstream fileOutput;
    std::vector<std::string> headers;
    std::vector<std::string> values;
};

int main(int argc, char** argv)
{
    std::string_view devName{ ""sv };
    std::filesystem::path reportFilename{ ""sv };

    static struct option long_options[] = { { "help", no_argument, 0, 'h' },
        { "device", required_argument, 0, 'd' },
        { "log", required_argument, 0, 'l' },
        { "output", required_argument, 0, 'o' },
        { 0, 0, 0, 0 } };

    int long_index = 0;
    int option = 0;
    while ((option = getopt_long_only(argc, argv, "", long_options, &long_index)) != -1)
    {
        switch (option)
        {
        case 'h':
            return printHelp();
        case 'd':
            if (optarg)
                devName = std::string(optarg);
            break;
        case 'l':
            if (optarg)
                logVerbosity = strToLogLevel(std::string(optarg));
            break;
        case 'o':
            if (optarg)
                reportFilename = std::string(optarg);
            break;
        }
    }

    SDRDevice* device = ConnectToFilteredOrDefaultDevice(devName);
    if (!device)
    {
        cerr << "Device connection failed" << endl;
        return EXIT_FAILURE;
    }

    device->SetMessageLogCallback(LogCallback);

    uint64_t serialNumber = device->GetDescriptor().serialNumber;

    PrintOEMTestReporter reporter(serialNumber, reportFilename);
    reporter.ReportColumn("S/N", std::to_string(serialNumber));
    OpStatus result = device->OEMTest(&reporter);

    if (result == OpStatus::Success)
        cerr << "OEM TEST PASSED" << endl;
    else
        cerr << "OEM TEST FAILED" << endl;

    DeviceRegistry::freeDevice(device);
    return result != OpStatus::Success ? EXIT_FAILURE : EXIT_SUCCESS;
}
