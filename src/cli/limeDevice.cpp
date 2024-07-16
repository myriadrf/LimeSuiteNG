#include "cli/common.h"
#include "args.hxx"

using namespace std;
using namespace lime;
using namespace std::literals::string_literals;
using namespace std::literals::string_view_literals;

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
    SDRDevice::GPS_Lock gpsStatus{};
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

int main(int argc, char* argv[])
{
    args::ArgumentParser parser("limeDevice - Device operations", "");
    args::HelpFlag help(parser, "help", "Display this help menu", { 'h', "help" });
    args::Flag fullFlag(parser, "full", "Force print detailed device(s) info", { 'f', "full" });

    try
    {
        parser.ParseCLI(argc, argv);
    } catch (args::Help&)
    {
        cout << parser << endl;
        return EXIT_SUCCESS;
    } catch (const std::exception& e)
    {
        cerr << e.what() << endl;
        return EXIT_FAILURE;
    }

    auto handles = DeviceRegistry::enumerate();
    if (handles.size() == 0)
    {
        cerr << "No devices found"sv << endl;
        return -1;
    }
    cerr << "Found "sv << handles.size() << " device(s) :"sv << endl;
    for (uint32_t i = 0; i < handles.size(); i++)
    {
        cout << i << ": "sv << handles[i].Serialize() << endl;
        if (fullFlag)
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
    return EXIT_SUCCESS;
}
