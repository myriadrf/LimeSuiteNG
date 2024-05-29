#include "cli/common.h"

#include <cassert>
#include <cstring>
#include <filesystem>
#include "args/args.hxx"

using namespace std;
using namespace lime;

static int32_t FindChipSelectByName(SDRDevice* device, const std::string_view chipName)
{
    if (!device)
        return -1;
    const auto chipMap = device->GetDescriptor().spiSlaveIds;
    if (chipName.size() == 0)
    {
        cerr << "specify SPI chip select, -c, --chip :"sv << endl;
        for (const auto& nameIds : chipMap)
            cerr << "\t"sv << nameIds.first << endl;
        return -1;
    }

    auto iter = chipMap.find(std::string{ chipName });
    if (iter == chipMap.end())
    {
        cerr << "Device does not contain target chip ("sv << chipName << "). Available list:"sv << endl;
        for (const auto& nameIds : chipMap)
            cerr << "\t"sv << nameIds.first << endl;
        return -1;
    }
    return iter->second;
}

static void PrintMISO(std::ostream& stream, const std::vector<uint32_t>& miso)
{
    stream << std::hex << std::setfill('0');
    for (uint32_t value : miso)
        stream << std::setw(8) << value << std::endl;
}

static uint32_t hex2int(const std::string_view hexstr)
{
    uint32_t value = 0;
    sscanf(hexstr.data(), "%X", &value);
    return value;
}

static int parseWriteInput(std::string_view hexstr, std::vector<uint32_t>& mosi)
{
    static const std::string_view delimiters = " \n,"sv;
    mosi.clear();

    const uint32_t spiWriteBit = 1 << 31;
    int tokenCount = 0;

    std::size_t position = 0;
    while (position != std::string_view::npos)
    {
        position = hexstr.find_first_of(delimiters);
        std::string_view token = hexstr.substr(0, position);
        int tokenLength = token.size();
        if (tokenLength <= 8 && tokenLength > 4) // write instruction
        {
            uint32_t value = hex2int(token);
            mosi.push_back(spiWriteBit | value);
        }
        else if (tokenLength != 0)
        {
            std::cerr << "Invalid input value: "sv << token << std::endl;
        }
        ++tokenCount;
        hexstr = hexstr.substr(position + 1);
    }
    return tokenCount;
}

static int parseReadInput(std::string_view hexstr, std::vector<uint32_t>& mosi)
{
    static const std::string_view delimiters = " \n,"sv;
    mosi.clear();

    int tokenCount = 0;

    std::size_t position = 0;
    while (position != std::string_view::npos)
    {
        position = hexstr.find_first_of(delimiters);
        std::string_view token = hexstr.substr(0, position);
        int tokenLength = token.size();
        if (tokenLength <= 4 && tokenLength > 0) // read instruction
        {
            uint32_t value = hex2int(token);
            mosi.push_back(value);
        }
        else if (tokenLength != 0)
        {
            std::cerr << "Invalid input value: "sv << token << std::endl;
        }
        ++tokenCount;
        hexstr = hexstr.substr(position + 1);
    }
    return tokenCount;
}

static std::string ReadFile(const std::string& fileName)
{
    std::vector<char> buffer;
    std::ifstream inputFile(fileName);
    if (!inputFile.is_open())
    {
        cerr << "Failed to open file: "sv << fileName << endl;
        exit(EXIT_FAILURE);
    }
    inputFile.seekg(0, std::ios::end);
    long fileSize = inputFile.tellg();
    inputFile.seekg(0, std::ios::beg);

    buffer.resize(fileSize);
    inputFile.read(&buffer[0], fileSize);
    inputFile.close();
    buffer[fileSize] = 0;
    return buffer.data();
}

int main(int argc, char** argv)
{
    // clang-format off
    args::ArgumentParser                    parser("limeSPI - Control status registers I/O", "");
    args::HelpFlag                          help(parser, "help", "This help", {'h', "help"});

    args::Group                             commands(parser, "commands");
    args::Command                           read(commands, "read", "Reading operation");
    args::Command                           write(commands, "write", "Do writing operation");

    args::Group                             arguments(parser, "arguments", args::Group::Validators::DontCare, args::Options::Global);
    args::ValueFlag<std::string>            deviceFlag(arguments, "name", "Specifies which device to use", {'d', "device"}, "");
    args::ValueFlag<std::string>            chipFlag(arguments, "name", "Selects destination chip", {'c', "chip"}, "");
    args::Group                             writeGroup(arguments, "Data options");
    args::ValueFlag<std::string>            fileFlag(arguments, "file", "File", {'f', "file"});
    args::ValueFlag<std::string>            streamFlag(arguments, "stream", "Stream", {'s', "stream"});
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
        cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    if (fileFlag == streamFlag)
    {
        cerr << "Either -s or -f must be provided ONCE" << endl;
        return EXIT_FAILURE;
    }

    const std::string devName = args::get(deviceFlag);
    const std::string chipName = args::get(chipFlag);
    const std::string hexInput = streamFlag ? args::get(streamFlag) : ReadFile(args::get(fileFlag));

    if (hexInput.empty())
    {
        cerr << "No input provided"sv << endl;
        return EXIT_FAILURE;
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

    int32_t chipSelect = FindChipSelectByName(device, chipName);
    if (chipSelect < 0)
    {
        DeviceRegistry::freeDevice(device);
        return EXIT_FAILURE;
    }

    std::vector<uint32_t> mosi;
    if (write)
        parseWriteInput(hexInput, mosi);
    else if (read)
        parseReadInput(hexInput, mosi);

    std::vector<uint32_t> miso(mosi.size());

    try
    {
        device->SPI(chipSelect, mosi.data(), miso.data(), mosi.size());
    } catch (std::runtime_error& e)
    {
        DeviceRegistry::freeDevice(device);
        cerr << "SPI failed: "sv << e.what() << endl;
        return EXIT_FAILURE;
    }

    // fill in register addresses for convenience and reusage as write input
    for (size_t i = 0; i < miso.size(); ++i)
        miso[i] |= mosi[i] << 16;

    if (read)
        PrintMISO(cout, miso);

    DeviceRegistry::freeDevice(device);
    return EXIT_SUCCESS;
}
