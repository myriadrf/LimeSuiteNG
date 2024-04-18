#include "cli/common.h"
#include <getopt.h>
#include <filesystem>

using namespace std;
using namespace lime;

SDRDevice* device = nullptr;
bool terminateProgress(false);

void inthandler(int sig)
{
    if (terminateProgress == true)
    {
        cerr << "Force exiting"sv << endl;
        exit(-1);
    }
    terminateProgress = true;
}

static void PrintMemoryDevices(SDRDescriptor descriptor)
{
    for (const auto& memoryDevice : descriptor.memoryDevices)
    {
        cerr << '\t' << memoryDevice.first << endl;
    }
}

static const std::shared_ptr<DataStorage> FindMemoryDeviceByName(SDRDevice* device, const std::string_view targetName)
{
    if (!device)
    {
        return std::make_shared<DataStorage>(nullptr, eMemoryDevice::COUNT);
    }

    const auto descriptor = device->GetDescriptor();
    const auto memoryDevices = descriptor.memoryDevices;
    if (targetName.size() == 0)
    {
        // If name is not specified, but only single choice is available, use that.
        if (memoryDevices.size() == 1)
        {
            return memoryDevices.begin()->second;
        }

        cerr << "specify memory device target, -t, --target :"sv << endl;
        PrintMemoryDevices(descriptor);

        return std::make_shared<DataStorage>(nullptr, eMemoryDevice::COUNT);
    }

    try
    {
        return memoryDevices.at(std::string{ targetName });
    } catch (const std::out_of_range& e)
    {
        std::cerr << e.what() << '\n';
    }

    cerr << "Device does not contain target device ("sv << targetName << "). Available list:"sv << endl;
    PrintMemoryDevices(descriptor);

    return std::make_shared<DataStorage>(nullptr, eMemoryDevice::COUNT);
}

static auto lastProgressUpdate = std::chrono::steady_clock::now();
bool progressCallBack(std::size_t bsent, std::size_t btotal, const std::string& statusMessage)
{
    float percentage = 100.0 * bsent / btotal;
    const bool hasCompleted = bsent == btotal;
    auto now = std::chrono::steady_clock::now();
    // no need to spam the text with each callback
    if (now - lastProgressUpdate > std::chrono::milliseconds(10) || hasCompleted)
    {
        lastProgressUpdate = now;
        std::cout << "[" << std::fixed << std::setprecision(0) << percentage << "] bytes written " << bsent << "/" << btotal << "\r"
                  << std::flush;
    }
    if (hasCompleted)
        cout << endl;
    if (terminateProgress)
    {
        printf("\nAborting programing will corrupt firmware and will need external programmer to fix it. Are you sure? [y/n]: ");
        fflush(stdout);
        std::string answer;
        while (1)
        {
            std::getline(std::cin, answer);
            if (answer[0] == 'y')
            {
                cout << "\naborting..."sv << endl;
                return true;
            }
            else if (answer[0] == 'n')
            {
                terminateProgress = false;
                cout << "\ncontinuing..."sv << endl;
                return false;
            }
            else
                cout << "Invalid option("sv << answer << "), [y/n]: "sv;
        }
    }
    return false;
}

static int printHelp(void)
{
    cerr << "Usage: LimeFLASH [OPTIONS] [FILE]" << endl;
    cerr << "  OPTIONS:" << endl;
    cerr << "    -h, --help\t\t\t This help" << endl;
    cerr << "    -d, --device <name>\t\t\t Specifies which device to use" << endl;
    cerr << "    -t, --target <TARGET>\t <TARGET> programming. \"-\" to list targets" << endl;
    cerr << endl;
    cerr << "  FILE: input file path." << endl;

    return EXIT_SUCCESS;
}

int main(int argc, char** argv)
{
    std::filesystem::path filePath{ ""sv };
    std::string_view devName{ ""sv };
    std::string_view targetName{ ""sv };
    signal(SIGINT, inthandler);

    static struct option long_options[] = { { "help", no_argument, 0, 'h' },
        { "device", required_argument, 0, 'd' },
        { "target", required_argument, 0, 't' },
        { 0, 0, 0, 0 } };

    int long_index = 0;
    int option = 0;
    std::string target;

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
        case 't':
            if (optarg != NULL)
            {
                targetName = optarg;
            }
            break;
        }
    }

    if (optind < argc)
        filePath = argv[optind];
    else
    {
        cerr << "File path not specified."sv << endl;
        printHelp();
        return -1;
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

    auto memorySelect = FindMemoryDeviceByName(device, targetName);
    if (memorySelect->ownerDevice == nullptr)
    {
        DeviceRegistry::freeDevice(device);
        return EXIT_FAILURE;
    }

    std::vector<char> data;
    std::ifstream inputFile;
    inputFile.open(filePath, std::ifstream::in | std::ifstream::binary);
    if (!inputFile)
    {
        DeviceRegistry::freeDevice(device);
        cerr << "Failed to open file: "sv << filePath << endl;
        return EXIT_FAILURE;
    }
    inputFile.seekg(0, std::ios_base::end);
    auto cnt = inputFile.tellg();
    inputFile.seekg(0, std::ios_base::beg);
    cerr << "File size : "sv << cnt << " bytes."sv << endl;
    data.resize(cnt);
    inputFile.read(data.data(), cnt);
    inputFile.close();

    if (memorySelect->ownerDevice->UploadMemory(memorySelect->memoryDeviceType, 0, data.data(), data.size(), progressCallBack) !=
        OpStatus::Success)
    {
        DeviceRegistry::freeDevice(device);
        cout << "Device programming failed."sv << endl;
        return EXIT_FAILURE;
    }
    if (terminateProgress)
        cerr << "Programming aborted."sv << endl;
    else
        cerr << "Programming completed."sv << endl;

    DeviceRegistry::freeDevice(device);
    return EXIT_SUCCESS;
}
