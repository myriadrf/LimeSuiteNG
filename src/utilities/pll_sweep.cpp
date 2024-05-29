#include "boards/LMS7002M_SDRDevice.h"
#include "FPGA_common/FPGA_common.h"
#include "limesuiteng/DeviceHandle.h"
#include "limesuiteng/DeviceRegistry.h"
#include "limesuiteng/LMS7002M.h"
#include "limesuiteng/Logger.h"
#include "limesuiteng/SDRDevice.h"
#include "limesuiteng/SDRDescriptor.h"

#include <iostream>
#include <iomanip>
#include <getopt.h>
#include <string>
#include <algorithm>
#include <cctype>
#include <fstream>
#include <chrono>
#include <cmath>

using namespace std;
using namespace lime;

auto t1 = chrono::high_resolution_clock::now();
auto t2 = chrono::high_resolution_clock::now();

bool ConfigureCGEN(SDRDevice* device, double freqMHz);

int printHelp(void);

lime::LogLevel log_level = lime::LogLevel::Info;

void log_func(const lime::LogLevel level, const std::string& message)
{
    if (level <= log_level)
        std::cout << message << std::endl;
}

int main(int argc, char** argv)
{
    double beginFreq = 50e6;
    double endFreq = 500e6;
    double stepFreq = 1e6;
    string configFilename = "";
    bool init = false;
    int deviceIndex = -1;

    int c;
    while (1)
    {
        static struct option long_options[] = { { "beginFreq", required_argument, 0, 'b' },
            { "stepFreq", required_argument, 0, 's' },
            { "endFreq", required_argument, 0, 'e' },
            { "log", required_argument, 0, 'l' },
            { "config", required_argument, 0, 'c' },
            { "device", required_argument, 0, 'd' },
            { "help", no_argument, 0, 'h' },
            { 0, 0, 0, 0 } };
        /* getopt_long stores the option index here. */
        int option_index = 0;
        c = getopt_long(argc, argv, "b:s:e:l:c::d:h", long_options, &option_index);

        if (c == -1) //no parameters given
            break;
        switch (c)
        {
        case 'b': {
            stringstream ss;
            ss << optarg;
            ss >> beginFreq;
            break;
        }
        case 's': {
            stringstream ss;
            ss << optarg;
            ss >> stepFreq;
            break;
        }
        case 'e': {
            stringstream ss;
            ss << optarg;
            ss >> endFreq;
            break;
        }
        case 'l': {
            stringstream ss;
            ss << optarg;
            int tempInt = 0;
            ss >> tempInt;
            log_level = static_cast<LogLevel>(tempInt);
            break;
        }
        case 'c': {
            if (optarg != NULL)
            {
                stringstream ss;
                ss << optarg;
                ss >> configFilename;
            }
            else
                init = true;
            break;
        }
        case 'd': {
            stringstream ss;
            ss << optarg;
            ss >> deviceIndex;
            break;
        }
        case 'h':
            printHelp();
            return 0;
        case '?':
            /* getopt_long already printed an error message. */
            break;
        default:
            std::cout << "Invalid option" << std::endl;
            abort();
        }
    }

    cout << "beginFreq = " << beginFreq / 1e6 << " MHz" << endl;
    cout << "stepFreq  = " << stepFreq / 1e6 << " MHz" << endl;
    cout << "endFreq   = " << endFreq / 1e6 << " MHz" << endl;

    SDRDevice* device;
    std::vector<DeviceHandle> handles = DeviceRegistry::enumerate();
    if (handles.size() == 0)
    {
        cout << "No devices found" << endl;
        return -1;
    }
    if (handles.size() == 1) //open the only available device
        device = DeviceRegistry::makeDevice(handles.at(0));
    else //display device selection
    {
        if (deviceIndex < 0)
        {
            cout << "Device list:" << endl;
            for (size_t i = 0; i < handles.size(); i++)
                cout << setw(2) << i << ". " << handles[i].name << endl;
            cout << "Select device index (0-" << handles.size() - 1 << "): ";
            int selection = 0;
            cin >> selection;
            selection = selection % handles.size();
            device = DeviceRegistry::makeDevice(handles.at(selection));
        }
        else
            device = DeviceRegistry::makeDevice(handles.at(deviceIndex));
    }
    if (device == nullptr)
    {
        cout << "Failed to connected to device" << endl;
        return -1;
    }
    auto& info = device->GetDescriptor();
    cout << "\nConnected to: " << info.name << " FW: " << info.firmwareVersion << " HW: " << info.hardwareVersion << endl;

    if (configFilename.length() > 0)
    {
        if (device->LoadConfig(0, configFilename) != OpStatus::Success)
        {
            return -1;
        }
    }
    else if (init)
    {
        device->Init();
    }

    lime::registerLogHandler(log_func);
    int errors = 0;
    for (double freq = beginFreq; freq < endFreq; freq += stepFreq)
        if (ConfigureCGEN(device, freq) == false)
            errors++;
    cout << "Errors: " << errors << endl;
    delete device;
    return 0;
}

bool ConfigureCGEN(SDRDevice* device, double freqMHz)
{
    std::cout << "Set CGEN " << freqMHz / 1e6 << " MHz: ";
    LMS7002M* lms = static_cast<LMS7002M*>(device->GetInternalChip(0));
    lms->Modify_SPI_Reg_bits(LMS7002MCSR::MAC, 1, true);
    int interp = lms->Get_SPI_Reg_bits(LMS7002MCSR::HBI_OVR_TXTSP);
    int decim = lms->Get_SPI_Reg_bits(LMS7002MCSR::HBD_OVR_RXTSP);
    if (lms->SetInterfaceFrequency(freqMHz, interp, decim) != OpStatus::Success)
    {
        std::cout << "LMS VCO Fail" << endl;
        return false;
    }

    auto fpgaTxPLL = lms->GetReferenceClk_TSP(TRXDir::Tx);
    if (interp != 7)
        fpgaTxPLL /= std::pow(2.0, interp);
    auto fpgaRxPLL = lms->GetReferenceClk_TSP(TRXDir::Rx);
    if (decim != 7)
        fpgaRxPLL /= std::pow(2.0, decim);
    auto chipInd = lms->GetActiveChannelIndex() / 2;
    auto fpga = static_cast<LMS7002M_SDRDevice*>(device)->GetFPGA();
    if (fpga)
    {
        OpStatus status = fpga->SetInterfaceFreq(fpgaTxPLL, fpgaRxPLL, chipInd);
        if (status != OpStatus::Success)
        {
            std::cout << "FPGA Fail" << endl;
            return false;
        }
    }
    std::cout << "OK" << endl;
    return true;
}

/***********************************************************************
 * print help
 **********************************************************************/
int printHelp(void)
{
    std::cout << "Usage pll_sweep [options]" << std::endl;
    std::cout << "available options:" << std::endl;
    std::cout << "    --help \t\t Print this help message" << std::endl;
    std::cout << "    --beginFreq \t sweep start frequency" << std::endl;
    std::cout << "    --endFreq \t\t sweep end frequency" << std::endl;
    std::cout << "    --stepFreq \t\t sweep frequency step" << std::endl;
    std::cout << "    --log \t\t log level" << std::endl;
    std::cout << "    --config \t\t load config" << std::endl;
    std::cout << "    --device \t\t device index" << std::endl;
    std::cout << std::endl;
    return EXIT_SUCCESS;
}
