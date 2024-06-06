#include "LitePCIe.h"

#include <iostream>
#include <errno.h>
#include <string.h>
#include <thread>
#include <filesystem>
#include "limesuiteng/Logger.h"

#ifdef __unix__
    #include <unistd.h>
    #include <fcntl.h>
    #include <poll.h>
    #include <sys/mman.h>
    #include <sys/ioctl.h>
    #include "linux-kernel-module/litepcie.h"
#endif

using namespace std;
using namespace lime;
using namespace std::literals::string_literals;

std::vector<std::string> LitePCIe::GetDevicesWithPattern(const std::string& regex)
{
    std::vector<std::string> devices;
    FILE* lsPipe;
    std::string cmd = "find /dev -maxdepth 1 -readable -name "s + regex;
    lsPipe = popen(cmd.c_str(), "r");
    char tempBuffer[512];
    while (fscanf(lsPipe, "%s", tempBuffer) == 1)
        devices.push_back(tempBuffer);
    pclose(lsPipe);
    return devices;
}

std::vector<std::string> LitePCIe::GetPCIeDeviceList()
{
    std::vector<std::string> devices;
    FILE* lsPipe;
    lsPipe = popen("ls -1 -- /sys/class/litepcie", "r");
    char tempBuffer[512];
    while (fscanf(lsPipe, "%s", tempBuffer) == 1)
        devices.push_back(tempBuffer);
    pclose(lsPipe);
    return devices;
}

LitePCIe::LitePCIe()
    : mFilePath()
    , mFileDescriptor(-1)
{
}

LitePCIe::~LitePCIe()
{
    Close();
}

OpStatus LitePCIe::Open(const std::filesystem::path& deviceFilename, uint32_t flags)
{
    mFilePath = deviceFilename;
    // use O_RDWR for now, because MMAP PROT_WRITE imples PROT_READ and will fail if file is opened write only
    flags &= ~O_WRONLY;
    flags |= O_RDWR;
    mFileDescriptor = open(mFilePath.c_str(), flags);
    if (mFileDescriptor < 0)
    {
        lime::error("LitePCIe: Failed to open (%s), errno(%i) %s", mFilePath.c_str(), errno, strerror(errno));
        // TODO: convert errno to OpStatus
        return OpStatus::FileNotFound;
    }
    return OpStatus::Success;
}

bool LitePCIe::IsOpen() const
{
    return mFileDescriptor >= 0;
}

void LitePCIe::Close()
{
    if (mFileDescriptor >= 0)
        close(mFileDescriptor);
    mFileDescriptor = -1;
}

int LitePCIe::WriteControl(const uint8_t* buffer, const int length, int timeout_ms)
{
    return write(mFileDescriptor, buffer, length);
}

int LitePCIe::ReadControl(uint8_t* buffer, const int length, int timeout_ms)
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
