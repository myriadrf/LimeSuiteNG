#include "common.h"

#include "limesuiteng/SDRDescriptor.h"
#include "limesuiteng/OpStatus.h"
#include "limesuiteng/VersionInfo.h"
#include "limesuiteng/ToString.h"

#include <assert.h>
#include <cstring>
#include <sstream>
#include <ctime>
#include <array>
#include <span>
#include <string_view>
#include <filesystem>
#include "args.hxx"

#include "OEMTesting.h"
#include "boards/OEMData.h"
#include "protocols/LMSBoards.h"

using namespace std;
using namespace lime;
using namespace lime::cli;
using namespace std::literals::string_literals;
using namespace std::literals::string_view_literals;

static bool interactiveMode = false;
static void WaitForUserInput()
{
    std::cerr << "Press ENTER to continue" << std::endl;
    cin.ignore();
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
        std::cerr << Indent() << "=== " << test.name << " ===" << std::endl;
        ++indentLevel;
    }
    void OnStepUpdate(OEMTestData& test, const std::string& text = std::string()) override
    {
        std::cerr << Indent() << text << std::endl;
    }

    void WaitForUserAction(OEMTestData& test, const std::string& text = std::string())
    {
        std::cerr << "--- " << text << " ---" << std::endl;
        WaitForUserInput();
    };

    std::string UserInputValue(OEMTestData& test)
    {
        std::string str;
        std::cerr << "Input value: ";
        std::cin >> str;
        return str;
    };

    void OnSuccess(OEMTestData& test) override
    {
        --indentLevel;
        std::cerr << Indent() << "=== " << test.name << " - PASSED" << " ===" << std::endl;
        if (interactiveMode)
            WaitForUserInput();
    }
    void OnFail(OEMTestData& test, const std::string& reasonText = std::string()) override
    {
        assert(!test.passed);
        --indentLevel;
        std::cerr << Indent() << "=== " << test.name << " - FAILED";

        if (!reasonText.empty())
            std::cerr << " (" << reasonText << ")";
        std::cerr << " ===" << std::endl;
        if (interactiveMode)
            WaitForUserInput();
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

static std::string ToHexString(std::span<const uint8_t> bytes)
{
    std::stringstream ss;
    ss << std::hex << std::setfill('0');
    for (uint8_t byte : bytes)
        ss << std::setw(2) << static_cast<int>(byte);
    return ss.str();
}

/// @brief Parses a hexadecimal number (optional 0x prefix) into a big-endian bit field, zero-extending it.
static bool ParseConfigurationFlags(std::string_view str, std::array<uint8_t, 32>& flags)
{
    if (str.starts_with("0x"sv) || str.starts_with("0X"sv))
        str.remove_prefix(2);
    if (str.empty() || str.size() > flags.size() * 2)
        return false;

    flags.fill(0);
    for (std::size_t i = 0; i < str.size(); ++i)
    {
        const char c = str[str.size() - 1 - i];
        int nibble;
        if (c >= '0' && c <= '9')
            nibble = c - '0';
        else if (c >= 'a' && c <= 'f')
            nibble = c - 'a' + 10;
        else if (c >= 'A' && c <= 'F')
            nibble = c - 'A' + 10;
        else
            return false;
        flags[flags.size() - 1 - i / 2] |= nibble << (4 * (i % 2));
    }
    return true;
}

/// @brief Finds the board ID matching the device name, LMS_DEV_UNKNOWN if there is none.
static eLMS_DEV BoardIdFromName(const std::string& name)
{
    for (int id = LMS_DEV_UNKNOWN + 1; id < LMS_DEV_COUNT; ++id)
    {
        if (GetDeviceName(static_cast<eLMS_DEV>(id)) == name)
            return static_cast<eLMS_DEV>(id);
    }
    return LMS_DEV_UNKNOWN;
}

static void PrintOEMData(const OEMData& data)
{
    cerr << "Board ID: " << data.boardId << " (" << GetDeviceName(static_cast<eLMS_DEV>(data.boardId)) << ")" << endl;
    cerr << "Board version: " << static_cast<int>(data.boardVersion) << endl;
    cerr << "Board revision: " << static_cast<int>(data.boardRevision) << endl;
    cerr << "UUID: " << OEMData::ToString(data.uuid) << endl;
    cerr << "Configuration flags: 0x" << ToHexString(data.configuration_flags) << endl;
}

struct OEMDataArgs {
    bool read;
    bool write;
    bool lock;
    bool overwrite;
    int boardId;
    int boardVersion;
    int boardRevision;
    std::string configurationFlags;
};

static OpStatus ProcessOEMData(SDRDevice* device, const OEMDataArgs& args)
{
    OEMData existing;
    const OpStatus readStatus = device->ReadOEMData(existing);
    if (readStatus == OpStatus::NotImplemented || readStatus == OpStatus::NotSupported)
    {
        cerr << "OEM data is not supported on this device." << endl;
        return readStatus;
    }
    const bool programmed = readStatus == OpStatus::Success;
    if (args.read)
    {
        if (programmed)
            PrintOEMData(existing);
        else
            cerr << "OEM data is not programmed or invalid (" << lime::ToString(readStatus) << ")" << endl;
    }

    if (!args.write)
        return OpStatus::Success;

    if (args.boardVersion < 0 || args.boardVersion > 255 || args.boardRevision < 0 || args.boardRevision > 255)
    {
        cerr << "Writing OEM data requires --board-version and --board-revision (0-255)." << endl;
        return OpStatus::InvalidValue;
    }
    if (programmed && !args.overwrite)
    {
        cerr << "OEM data is already programmed, use --overwrite to replace it." << endl;
        return OpStatus::Error;
    }

    OEMData data;
    if (args.boardId >= 0)
        data.boardId = args.boardId;
    else if (programmed)
        data.boardId = existing.boardId;
    else
        data.boardId = BoardIdFromName(device->GetDescriptor().name);
    if (data.boardId == LMS_DEV_UNKNOWN)
    {
        cerr << "Unable to determine the board ID, specify it with --board-id." << endl;
        return OpStatus::InvalidValue;
    }

    if (!args.configurationFlags.empty())
    {
        if (!ParseConfigurationFlags(args.configurationFlags, data.configuration_flags))
        {
            cerr << "Invalid --configuration-flags, expected a hexadecimal value of up to 256 bits." << endl;
            return OpStatus::InvalidValue;
        }
    }
    else if (programmed)
        data.configuration_flags = existing.configuration_flags;

    data.boardVersion = args.boardVersion;
    data.boardRevision = args.boardRevision;
    data.uuid = OEMData::GenerateUUIDv7();

    OpStatus status = device->WriteOEMData(data, args.lock);
    if (status != OpStatus::Success)
    {
        cerr << "Failed to write OEM data (" << lime::ToString(status) << ")" << endl;
        return status;
    }
    cerr << "OEM data written" << (args.lock ? " and locked" : "") << ":" << endl;
    PrintOEMData(data);
    return OpStatus::Success;
}

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
    args::Flag                      interactive(parser, "", "Wait for user input after each test", {"interactive"});
    args::Group                     oemGroup(parser, "OEM data (board identification and configuration)");
    args::Flag                      readOEMFlag(oemGroup, "", "Read and print OEM data", {"read-oem-data"});
    args::Flag                      writeOEMFlag(oemGroup, "", "Write OEM data with a newly generated UUID", {"write-oem-data"});
    args::ValueFlag<int>            boardIdFlag(oemGroup, "", "Board ID (default: from existing OEM data or the device type)", {"board-id"}, -1);
    args::ValueFlag<int>            boardVersionFlag(oemGroup, "", "Board hardware version", {"board-version"}, -1);
    args::ValueFlag<int>            boardRevisionFlag(oemGroup, "", "Board PCB revision", {"board-revision"}, -1);
    args::ValueFlag<std::string>    configFlagsFlag(oemGroup, "hex", "256-bit configuration flags as a hexadecimal number (default: from existing OEM data, or all set)", {"configuration-flags"}, "");
    args::Flag                      overwriteOEMFlag(oemGroup, "", "Allow replacing already programmed OEM data", {"overwrite"});
    args::Flag                      lockOEMFlag(oemGroup, "", "Permanently write protect the OEM data after writing. IRREVERSIBLE", {"lock"});
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
    interactiveMode = interactive;

    logVerbosity = strToLogLevel(args::get(logFlag));
    const std::string devName = args::get(deviceFlag);
    const std::string reportFilename = args::get(reportFileFlag);

    SDRDevice* device = ConnectToFilteredOrDefaultDevice(devName);
    if (!device)
        return EXIT_FAILURE;

    device->SetMessageLogCallback(lime::cli::LogCallback);
    lime::registerLogHandler(lime::cli::LogCallback);
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

    if (readOEMFlag || writeOEMFlag)
    {
        const OEMDataArgs oemArgs{ readOEMFlag,
            writeOEMFlag,
            lockOEMFlag,
            overwriteOEMFlag,
            args::get(boardIdFlag),
            args::get(boardVersionFlag),
            args::get(boardRevisionFlag),
            args::get(configFlagsFlag) };
        if (ProcessOEMData(device, oemArgs) != OpStatus::Success)
        {
            DeviceRegistry::freeDevice(device);
            return EXIT_FAILURE;
        }
    }

    if (runTestsFlag)
    {
        uint64_t serialNumber = device->GetDescriptor().serialNumber;
        cerr << "Board serial number: " << serialNumber << " (";
        stringstream ss;
        ss << std::hex << std::setw(2) << std::setfill('0');
        for (size_t i = 0; i < sizeof(serialNumber); ++i)
            ss << "0x" << ((serialNumber >> 8 * i) & 0xFF) << " ";
        cerr << ss.str() << ")" << endl;

        PrintOEMTestReporter reporter(serialNumber, reportFilename);
        reporter.ReportColumn("S/N", std::to_string(serialNumber));
        result = device->OEMTest(&reporter);

        if (result == OpStatus::Success)
            cerr << "OEM TEST PASSED" << endl;
        else
            cerr << "OEM TEST FAILED (" << lime::ToString(result) << ")" << endl;
    }

    DeviceRegistry::freeDevice(device);
    return result != OpStatus::Success ? EXIT_FAILURE : EXIT_SUCCESS;
}
