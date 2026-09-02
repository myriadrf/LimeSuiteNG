#include "comms/PCIe/xillybus/Xillybus.h"

#include "limesuiteng/Logger.h"

#include <iostream>
#include <chrono>
#include <cerrno>
#include <cstring>
#include <thread>
#include "protocols/LMS64CProtocol.h"

#ifdef __unix__
    #include <unistd.h>
    #include <fcntl.h>
    #include <poll.h>
    #include <sys/mman.h>
    #include <sys/ioctl.h>
#endif

using namespace std;
using namespace lime;
using namespace std::literals::string_literals;

std::vector<std::string> Xillybus::GetEndpointsWithPattern(const std::string& devicePath, const std::string& regex)
{
    std::vector<std::string> devices;
    FILE* lsPipe;

    std::string cmd = "ls -1 "s + devicePath + "/"s + regex;
    lsPipe = popen(cmd.c_str(), "r");
    if (lsPipe == nullptr)
        return devices;
    char tempBuffer[512];
    while (fscanf(lsPipe, "%511s", tempBuffer) == 1)
        devices.push_back(tempBuffer);
    pclose(lsPipe);
    return devices;
}

std::vector<std::string> Xillybus::GetPCIeDeviceList()
{
    std::vector<std::string> devices;
    FILE* lsPipe;
    lsPipe = popen("ls -1 -- /sys/class/Xillybus 2> /dev/null", "r");
    if (lsPipe == nullptr)
        return devices;
    char tempBuffer[512];
    while (fscanf(lsPipe, "%511s", tempBuffer) == 1)
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

Xillybus::Xillybus()
    : fd_write(-1)
    , fd_read(-1)
    , nonblocking(true)
{
}

Xillybus::~Xillybus()
{
    Close();
}

OpStatus Xillybus::Open(const std::filesystem::path& write_path, const std::filesystem::path& read_path, bool nonblock)
{
    writer_path = write_path;
    reader_path = read_path;
    nonblocking = nonblock;
    std::this_thread::sleep_for(chrono::microseconds(
        200)); //avoids random bad packet(s) that was occurring when performing lots of consecutive transfers ~1000+
    auto t1 = chrono::high_resolution_clock::now();
    auto t2 = t1;

    uint32_t flags = O_NOCTTY;
    if (nonblock)
        flags |= O_NONBLOCK;

    while (chrono::duration_cast<chrono::milliseconds>(t2 - t1) < chrono::milliseconds(1000))
    {
        if ((fd_write = open(write_path.c_str(), O_WRONLY | flags)) != -1)
            break;
        std::this_thread::sleep_for(chrono::milliseconds(1));
        t2 = chrono::high_resolution_clock::now();
    }

    t1 = chrono::high_resolution_clock::now();
    t2 = t1;
    while (chrono::duration_cast<chrono::milliseconds>(t2 - t1) < chrono::milliseconds(1000))
    {
        if ((fd_read = open(read_path.c_str(), O_RDONLY | flags)) != -1)
            break;
        std::this_thread::sleep_for(chrono::milliseconds(1));
        t2 = chrono::high_resolution_clock::now();
    }

    if (fd_write == -1 || fd_read == -1)
    {
        // lime::error("Xillybus: Failed to open (%s), errno(%i) %s", mFilePath.c_str(), errno, strerror(errno));
        return OpStatus::FileNotFound;
    }
    return OpStatus::Success;
}

OpStatus Xillybus::Reset()
{
    if (fd_write > 0 && fd_read > 0)
    {
        Close();
        return Open(writer_path, reader_path, nonblocking);
    }
    return OpStatus::Error;
}

bool Xillybus::IsOpen() const
{
    return (fd_write != -1 && fd_read != -1);
}

void Xillybus::Close()
{
    if (fd_write)
    {
        close(fd_write);
        fd_write = -1;
    }
    if (fd_read)
    {
        close(fd_read);
        fd_read = -1;
    }
}

int Xillybus::Write(const uint8_t* data, std::size_t length, int timeout_ms)
{
#ifndef __unix__
    if (hWrite == INVALID_HANDLE_VALUE)
        return -1;
#endif
    size_t totalBytesWritten = 0;
    int bytesToWrite = length;
    auto t1 = chrono::high_resolution_clock::now();

    do
    {
#ifndef __unix__
        DWORD bytesSent = 0;
        OVERLAPPED vOverlapped;
        memset(&vOverlapped, 0, sizeof(OVERLAPPED));
        vOverlapped.hEvent = CreateEvent(NULL, false, false, NULL);
        WriteFile(fd_write, data + totalBytesWritten, bytesToWrite, &bytesSent, &vOverlapped);
        if (::GetLastError() != ERROR_IO_PENDING)
        {
            CloseHandle(vOverlapped.hEvent);
            return totalBytesWritten;
        }
        std::this_thread::yield();
        DWORD dwRet = WaitForSingleObject(vOverlapped.hEvent, 500);
        if (dwRet == WAIT_OBJECT_0)
        {
            if (GetOverlappedResult(hWrite, &vOverlapped, &bytesSent, FALSE) == FALSE)
            {
                bytesSent = 0;
            }
        }
        else
        {
            CancelIo(hWrite);
            bytesSent = 0;
        }
        CloseHandle(vOverlapped.hEvent);
#else
        int bytesSent;
        if ((bytesSent = write(fd_write, data + totalBytesWritten, bytesToWrite)) < 0)
        {
            if (errno == EINTR || errno == EAGAIN)
                continue;
            return totalBytesWritten;
        }
#endif
        totalBytesWritten += bytesSent;
        if (totalBytesWritten < length)
            bytesToWrite -= bytesSent;
        else
            break;

    } while (std::chrono::duration_cast<std::chrono::milliseconds>(chrono::high_resolution_clock::now() - t1).count() < timeout_ms);
#ifdef __unix__
    //Flush data to FPGA
    while (write(fd_write, NULL, 0) < 0)
    {
        if (errno == EINTR)
            continue;
        break;
    }
#else
    if (totalBytesWritten != length)
    {
        CloseHandle(fd_write);
        hWrite =
            CreateFileA(writeCtrlPort.c_str(), GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED, 0);
        if (hWrite == INVALID_HANDLE_VALUE)
        {
            CloseHandle(hRead);
            hWrite = INVALID_HANDLE_VALUE;
        }
    }
#endif
    return totalBytesWritten;
}

int Xillybus::Read(uint8_t* data, std::size_t length, int timeout_ms)
{
#ifndef __unix__
    if (hRead == INVALID_HANDLE_VALUE)
        return -1;
#endif
    size_t totalBytesReaded = 0;
    int bytesToRead = length;
    auto t1 = chrono::high_resolution_clock::now();

    do
    {
#ifndef __unix__
        DWORD bytesReceived = 0;
        OVERLAPPED vOverlapped;
        memset(&vOverlapped, 0, sizeof(OVERLAPPED));
        vOverlapped.hEvent = CreateEvent(NULL, false, false, NULL);
        ReadFile(fd_read, data + totalBytesReaded, bytesToRead, &bytesReceived, &vOverlapped);
        if (::GetLastError() != ERROR_IO_PENDING)
        {
            CloseHandle(vOverlapped.hEvent);
            return totalBytesReaded;
        }
        std::this_thread::yield();
        DWORD dwRet = WaitForSingleObject(vOverlapped.hEvent, 1000);
        if (dwRet == WAIT_OBJECT_0)
        {
            if (GetOverlappedResult(hRead, &vOverlapped, &bytesReceived, TRUE) == FALSE)
            {
                bytesReceived = 0;
            }
        }
        else
        {
            CancelIo(hRead);
            bytesReceived = 0;
        }
        CloseHandle(vOverlapped.hEvent);
#else
        int bytesReceived;
        if ((bytesReceived = read(fd_read, data + totalBytesReaded, bytesToRead)) < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            else if (errno == EAGAIN)
            {
                continue;
            }

            return totalBytesReaded;
        }
#endif
        totalBytesReaded += bytesReceived;
        if (totalBytesReaded < length)
            bytesToRead -= bytesReceived;
        else
            break;

    } while (std::chrono::duration_cast<std::chrono::milliseconds>(chrono::high_resolution_clock::now() - t1).count() < timeout_ms);

#ifndef __unix__
    if (totalBytesReaded != length)
    {
        CloseHandle(hRead);
        hRead =
            CreateFileA(readCtrlPort.c_str(), GENERIC_READ, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED, 0);
        if (hRead == INVALID_HANDLE_VALUE)
        {
            CloseHandle(hRead);
            hRead = INVALID_HANDLE_VALUE;
        }
    }
#endif
    return totalBytesReaded;
}

int Xillybus::WriteControl(const uint8_t* buffer, const int length, int timeout_ms)
{
    return Write(buffer, length, timeout_ms);
}

int Xillybus::ReadControl(uint8_t* buffer, const int length, int timeout_ms)
{
    return Read(buffer, length, timeout_ms);
}

OpStatus Xillybus::RunControlCommand(uint8_t* request, uint8_t* response, size_t length, int timeout_ms)
{
    int ret = WriteControl(request, length, timeout_ms);
    if (static_cast<size_t>(ret) != length)
        return OpStatus::IOFailure;

    ret = ReadControl(response, length, timeout_ms);
    if (static_cast<size_t>(ret) != length)
        return OpStatus::IOFailure;

    return OpStatus::Success;
}

OpStatus Xillybus::RunControlCommand(uint8_t* data, size_t length, int timeout_ms)
{
    return RunControlCommand(data, data, length, timeout_ms);
}