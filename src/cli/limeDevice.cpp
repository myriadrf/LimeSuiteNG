#include "cli/common.h"
#include <getopt.h>

using namespace std;
using namespace lime;
using namespace std::literals::string_literals;
using namespace std::literals::string_view_literals;

static int printHelp(void)
{
    cerr << "limeDevice [options]"sv << endl;
    cerr << "    -h, --help\t This help"sv << endl;
    cerr << "    -f, --full\t Force print detailed device(s) info"sv << endl;
    cerr << "    --write-serial-number\t Serial number to be written to device" << endl;
    return EXIT_SUCCESS;
}

static const std::string_view GPSLockToString(const SDRDevice::GPS_Lock::LockStatus& status)
{
    switch (status)
    {
    case SDRDevice::GPS_Lock::LockStatus::NotAvailable:
        return "Not available"sv;
    case SDRDevice::GPS_Lock::LockStatus::Has2D:
        return "2D"sv;
    case SDRDevice::GPS_Lock::LockStatus::Has3D:
        return "3D"sv;
    default:
        return "Undefined"sv;
    }
}

void PrintDeviceDetails(SDRDevice* device)
{
    auto d = device->GetDescriptor();
    cout << '\t' << "Expansion name\t\t: " << d.expansionName << endl;
    cout << '\t' << "Firmware version\t: " << d.firmwareVersion << endl;
    cout << '\t' << "Gateware version\t: " << d.gatewareVersion << endl;
    cout << '\t' << "Gateware revision\t: " << d.gatewareRevision << endl;
    cout << '\t' << "Gateware target board\t: " << d.gatewareTargetBoard << endl;
    cout << '\t' << "Hardware version\t: " << d.hardwareVersion << endl;
    cout << '\t' << "Protocol version\t: " << d.protocolVersion << endl;
    cout << '\t' << "Serial number\t\t: " << d.serialNumber << endl;
    cout << "\t"
         << "SPI slave devices\t:" << endl;
    for (const auto& nameIds : d.spiSlaveIds)
        cout << "\t\t\t\t  " << nameIds.first << endl;
    cout << "\t"
         << "Memory devices\t\t:" << endl;
    for (const auto& mem : d.memoryDevices)
        cout << "\t\t\t\t  " << mem.first << endl;
    cout << "\t"
         << "GPS Lock:" << endl;
    SDRDevice::GPS_Lock gpsStatus;
    device->GetGPSLock(&gpsStatus);
    cout << "\t\t"
         << "GPS - " << GPSLockToString(gpsStatus.gps) << endl;
    cout << "\t\t"
         << "Glonass - " << GPSLockToString(gpsStatus.glonass) << endl;
    cout << "\t\t"
         << "Galileo - " << GPSLockToString(gpsStatus.galileo) << endl;
    cout << "\t\t"
         << "Beidou - " << GPSLockToString(gpsStatus.beidou) << endl;
}

enum Args { DEVICE = 'd', Help = 'h', Full = 'f', WriteSerial = 200 };

int main(int argc, char* argv[])
{
    static struct option long_options[] = { { "help", no_argument, 0, Args::Help },
        { "full", no_argument, 0, Args::Full },
        { "device", required_argument, 0, Args::DEVICE },
        { "write-serial-number", required_argument, 0, Args::WriteSerial },
        { 0, 0, 0, 0 } };

    bool full(false);
    int long_index = 0;
    int option = 0;
    uint64_t serialNumberArg = 0;
    std::string devName;

    while ((option = getopt_long(argc, argv, "", long_options, &long_index)) != -1)
    {
        switch (option)
        {
        case Args::DEVICE:
            if (optarg)
                devName = std::string(optarg);
            break;
        case Args::Help:
            return printHelp();
        case Args::Full:
            full = true;
            break;
        case Args::WriteSerial:
            if (optarg)
                serialNumberArg = std::stoll(std::string(optarg));
            break;
        }
    }

    std::vector<DeviceHandle> handles = DeviceRegistry::enumerate();
    if (handles.size() == 0)
    {
        cerr << "No devices found"sv << endl;
        return -1;
    }
    cerr << "Found "sv << handles.size() << " device(s) :"sv << endl;
    for (uint32_t i = 0; i < handles.size(); i++)
    {
        cout << i << ": "sv << handles[i].Serialize() << endl;
        if (full)
        {
            SDRDevice* device = DeviceRegistry::makeDevice(handles.at(i));
            if (!device)
            {
                cerr << "\tFailed to connect"sv << endl;
                continue;
            }
            PrintDeviceDetails(device);
            DeviceRegistry::freeDevice(device);
        }
    }

    if (serialNumberArg)
    {
        SDRDevice* device = ConnectToFilteredOrDefaultDevice(devName);
        if (!device)
            return EXIT_FAILURE;

        OpStatus status = device->WriteSerialNumber(serialNumberArg);
        if (status != OpStatus::Success)
        {
            cerr << "Failed to write serial number." << endl;
            DeviceRegistry::freeDevice(device);
            return EXIT_FAILURE;
        }
        DeviceRegistry::freeDevice(device);
    }

    return EXIT_SUCCESS;
}
