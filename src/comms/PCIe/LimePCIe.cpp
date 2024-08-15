#include "comms/PCIe/LimePCIe.h"

#include <iostream>
#include <cerrno>
#include <string.h>
#include <thread>
#include <filesystem>
#include "limesuiteng/Logger.h"
#include "LMS64CProtocol.h"

#ifdef __unix__
    #include <unistd.h>
    #include <fcntl.h>
    #include <poll.h>
    #include <sys/mman.h>
    #include <sys/ioctl.h>
    #include "linux-kernel-module/limepcie.h"
#endif

using namespace std;
using namespace lime;
using namespace std::literals::string_literals;

std::vector<std::string> LimePCIe::GetEndpointsWithPattern(const std::string& devicePath, const std::string& regex)
{
    std::vector<std::string> devices;
    FILE* lsPipe;

    std::string cmd = "ls -1 "s + devicePath + "/"s + regex;
    lsPipe = popen(cmd.c_str(), "r");
    char tempBuffer[512];
    while (fscanf(lsPipe, "%s", tempBuffer) == 1)
        devices.push_back(tempBuffer);
    pclose(lsPipe);
    return devices;
}

std::vector<std::string> LimePCIe::GetPCIeDeviceList()
{
    std::vector<std::string> devices;
    FILE* lsPipe;
    lsPipe = popen("ls -1 -- /sys/class/limepcie 2> /dev/null", "r");
    char tempBuffer[512];
    while (fscanf(lsPipe, "%s", tempBuffer) == 1)
    {
        // Kernel code fakes directories by replacing '/' char with '!'
        // open() can't open that
        // Replace '!' with '/' so we could open device
        std::string parsedDevicePath{ tempBuffer };
        for (auto& c : parsedDevicePath)
        {
            if (c == '!')
                c = '/';
        }
        devices.push_back(parsedDevicePath);
    }
    pclose(lsPipe);
    return devices;
}

LimePCIe::LimePCIe()
    : mFilePath()
    , mFileDescriptor(-1)
{
}

LimePCIe::~LimePCIe()
{
    Close();
}

OpStatus LimePCIe::RunControlCommand(uint8_t* request, uint8_t* response, size_t length, int timeout_ms)
{
#ifndef ENOIOCTLCMD
    constexpr int ENOIOCTLCMD = 515; // not a standard Posix error code, but exists in linux kernel headers
#endif
    limepcie_control_packet pkt;
    pkt.timeout_ms = timeout_ms;
    pkt.length = length;

    memcpy(pkt.request, request, length);

    int ret = ioctl(mFileDescriptor, LIMEPCIE_IOCTL_RUN_CONTROL_COMMAND, &pkt);

    switch (ret)
    {
    case 0:
        break;
    case ENOTTY:
    case -ENOTTY:
    case ENOIOCTLCMD:
    case -ENOIOCTLCMD: {
        // driver does not support ioctl, fallback to write/read
        ret = WriteControl(request, length, timeout_ms);
        if (static_cast<size_t>(ret) != length)
            return OpStatus::IOFailure;

        ret = ReadControl(response, length, timeout_ms);
        if (static_cast<size_t>(ret) != length)
            return OpStatus::IOFailure;
        return OpStatus::Success;
    }
    case EBUSY:
    case -EBUSY:
        return OpStatus::Busy;
    case ETIMEDOUT:
    case -ETIMEDOUT:
        lime::error("control command timeout");
        return OpStatus::Timeout;
    default:
        lime::error("Unable to send control packet");
        return OpStatus::IOFailure;
    }

    if (pkt.length != length)
        return OpStatus::IOFailure;

    memcpy(response, pkt.response, length);

    return OpStatus::Success;
}

OpStatus LimePCIe::RunControlCommand(uint8_t* data, size_t length, int timeout_ms)
{
    return RunControlCommand(data, data, length, timeout_ms);
}

OpStatus LimePCIe::Open(const std::filesystem::path& deviceFilename, uint32_t flags)
{
    mFilePath = deviceFilename;
    // use O_RDWR for now, because MMAP PROT_WRITE implies PROT_READ and will fail if file is opened write only
    flags &= ~O_WRONLY;
    flags |= O_RDWR;
    mFileDescriptor = open(mFilePath.c_str(), flags);
    if (mFileDescriptor < 0)
    {
        lime::error("LimePCIe: Failed to open (%s), errno(%i) %s", mFilePath.c_str(), errno, strerror(errno));
        // TODO: convert errno to OpStatus
        return OpStatus::FileNotFound;
    }
    return OpStatus::Success;
}

bool LimePCIe::IsOpen() const
{
    return mFileDescriptor >= 0;
}

void LimePCIe::Close()
{
    if (mFileDescriptor >= 0)
        close(mFileDescriptor);
    mFileDescriptor = -1;
}

int LimePCIe::WriteControl(const uint8_t* buffer, const int length, int timeout_ms)
{
    return write(mFileDescriptor, buffer, length);
}

int LimePCIe::ReadControl(uint8_t* buffer, const int length, int timeout_ms)
{
    memset(buffer, 0, length);
    uint32_t status = 0;
    auto t1 = chrono::high_resolution_clock::now();
    do
    { //wait for status byte to change
        int ret = read(mFileDescriptor, &status, sizeof(status));
        if (ret < 0)
        {
            if (errno != EAGAIN)
                break;
        }
        if ((status & 0xFF00) != 0)
            break;
        std::this_thread::sleep_for(std::chrono::microseconds(10));
    } while (std::chrono::duration_cast<std::chrono::milliseconds>(chrono::high_resolution_clock::now() - t1).count() < timeout_ms);

    if ((status & 0xFF00) == 0)
        ReportError(OpStatus::Timeout, "CMD %02X Read timeout", status & 0xFF);
    return read(mFileDescriptor, buffer, length);
}
