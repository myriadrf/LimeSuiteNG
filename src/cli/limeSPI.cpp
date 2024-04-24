#include "cli/common.h"

#include <cassert>
#include <cstring>
#include <getopt.h>
#include <filesystem>

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

static int printHelp(void)
{
    cerr << "limeSPI [options]" << endl;
    cerr << "    -h, --help\t\t\t This help" << endl;
    cerr << "    -d, --device <name>\t\t\t Specifies which device to use" << endl;
    cerr << "    -c, --chip <name>\t\t Selects destination chip" << endl;
    cerr << "    -r, --read \"data or filepath\"\t\t space/newline/comma delimited 16bit hexadecimal addresses for reading" << endl;
    cerr << "    -w, --write \"data or filepath\"\t\t space/newline/comma delimited 32bit hexadecimal values for writing" << endl;
    cerr << "    -f, --file\t\t Use --read/--write argument as filename" << endl;

    return EXIT_SUCCESS;
}

int main(int argc, char** argv)
{
    std::string_view devName{ ""sv };
    std::string_view chipName{ ""sv };
    std::string_view hexInput{ ""sv };
    bool hexInputIsFilename = false;
    bool isWrite = false;
    bool isRead = false;

    static struct option long_options[] = { { "help", no_argument, 0, 'h' },
        { "device", required_argument, 0, 'd' },
        { "chip", required_argument, 0, 'c' },
        { "read", required_argument, 0, 'r' },
        { "write", required_argument, 0, 'w' },
        { "file", optional_argument, 0, 'f' },
        { 0, 0, 0, 0 } };

    int long_index = 0;
    int option = 0;
    while ((option = getopt_long(argc, argv, "", long_options, &long_index)) != -1)
    {
        switch (option)
        {
        case 'h':
            return printHelp();
        case 'd':
            if (optarg != NULL)
                devName = std::string(optarg);
            break;
        case 'c':
            if (optarg != NULL)
                chipName = optarg;
            break;
        case 'r':
            if (optarg != NULL)
            {
                isRead = true;
                hexInput = optarg;
            }
            break;
        case 'w':
            if (optarg != NULL)
            {
                isWrite = true;
                hexInput = optarg;
            }
            break;
        case 'f':
            hexInputIsFilename = true;
            break;
        }
    }

    if (isRead && isWrite)
    {
        cerr << "use --read, OR --write, can't do both at the same time"sv << endl;
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

    std::vector<char> buffer;
    if (hexInputIsFilename)
    {
        //read file
        const std::filesystem::path filename(hexInput);
        std::ifstream inputFile(filename);
        if (!inputFile.is_open())
        {
            cerr << "Failed to open file: "sv << filename << endl;
            return EXIT_FAILURE;
        }
        inputFile.seekg(0, std::ios::end);
        long fileSize = inputFile.tellg();
        inputFile.seekg(0, std::ios::beg);

        buffer.resize(fileSize + 1); // +1 to add null termination
        inputFile.read(&buffer[0], fileSize);
        buffer[fileSize] = 0;
        hexInput = buffer.data();
    }

    if (hexInput.empty())
    {
        DeviceRegistry::freeDevice(device);
        cerr << "No input provided"sv << endl;
        return EXIT_FAILURE;
    }

    std::vector<uint32_t> mosi;
    if (isWrite)
        parseWriteInput(hexInput, mosi);
    else if (isRead)
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

    if (isRead)
        PrintMISO(cout, miso);

    DeviceRegistry::freeDevice(device);
    return EXIT_SUCCESS;
}
