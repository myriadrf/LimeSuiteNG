#include "FT601.h"

#include <cassert>
#include <cstring>

using namespace std::literals::string_literals;

namespace lime {

#ifndef __unix__
struct FTDIAsyncContext {
    OVERLAPPED inOvLap;
    uint8_t endPointAddr;
};
#endif

static const int STREAM_BULK_WRITE_ADDRESS = 0x03;
static const int STREAM_BULK_READ_ADDRESS = 0x83;

static const int CONTROL_BULK_WRITE_ADDRESS = 0x02;
static const int CONTROL_BULK_READ_ADDRESS = 0x82;

std::vector<USBDescriptor> FT601::enumerateDevices(const std::set<VendorProductId>& ids)
{
#ifdef __unix__
    return libusb_impl.enumerateDevices(ids);
#else
    std::vector<USBDescriptor> devDescriptors;

    static DWORD numDevs = 0;
    FT_STATUS ftStatus = FT_CreateDeviceInfoList(&numDevs);
    if (FT_FAILED(ftStatus) || numDevs <= 0)
        return devDescriptors;

    DWORD Flags = 0;
    char SerialNumber[512] = { 0 };
    char Description[512] = { 0 };
    for (DWORD i = 0; i < numDevs; i++)
    {
        DWORD deviceId = 0;
        ftStatus = FT_GetDeviceInfoDetail(i, &Flags, nullptr, &deviceId, nullptr, SerialNumber, Description, nullptr);
        if (!FT_FAILED(ftStatus))
        {
            USBDescriptor desc;
            desc.vid = (deviceId >> 16);
            desc.pid = static_cast<WORD>(deviceId);

            if (!ids.empty() && ids.find({ desc.vid, desc.pid }) == ids.end())
                continue;

            desc.product = Description;
            desc.serial = SerialNumber;
            devDescriptors.push_back(desc);
        }
    }
    return devDescriptors;
#endif
}

FT601::FT601()
#ifdef __unix__
    : mUsbCounter(0)
#else
    : mFTHandle(nullptr)
#endif
{
}

FT601::~FT601()
{
    Disconnect();
}

bool FT601::Connect(uint16_t vid, uint16_t pid, const char* serial)
{
    Disconnect();
#ifdef __unix__
    if (libusb_impl.Connect(vid, pid, serial))
    {
        OpStatus status = libusb_impl.ClaimInterface(1);
        if (status != OpStatus::Success)
        {
            Disconnect();
            return false;
        }
        FT_FlushPipe(CONTROL_BULK_READ_ADDRESS); // Clear control endpoint rx buffer
        FT_SetStreamPipe(CONTROL_BULK_READ_ADDRESS, 64);
        FT_SetStreamPipe(CONTROL_BULK_WRITE_ADDRESS, 64);
        ResetStreamBuffers();
        return true;
    }
    return false;
#else
    DWORD devCount;
    FT_STATUS ftStatus = FT_OK;
    DWORD dwNumDevices = 0;
    // Open a device
    ftStatus = FT_Create(reinterpret_cast<void*>(const_cast<char*>(serial)), FT_OPEN_BY_SERIAL_NUMBER, &mFTHandle);

    if (FT_FAILED(ftStatus))
        return false;

    FT_AbortPipe(mFTHandle, CONTROL_BULK_READ_ADDRESS);
    FT_AbortPipe(mFTHandle, CONTROL_BULK_WRITE_ADDRESS);
    FT_SetStreamPipe(mFTHandle, FALSE, FALSE, CONTROL_BULK_READ_ADDRESS, 64);
    FT_SetStreamPipe(mFTHandle, FALSE, FALSE, CONTROL_BULK_WRITE_ADDRESS, 64);
    ResetStreamBuffers();

    FT_SetPipeTimeout(mFTHandle, CONTROL_BULK_WRITE_ADDRESS, 500);
    FT_SetPipeTimeout(mFTHandle, CONTROL_BULK_READ_ADDRESS, 500);
    FT_SetPipeTimeout(mFTHandle, STREAM_BULK_READ_ADDRESS, 100);
    FT_SetPipeTimeout(mFTHandle, STREAM_BULK_WRITE_ADDRESS, 100);
    return true;
#endif
}

void FT601::Disconnect()
{
#ifdef __unix__
    if (!IsConnected())
        return;

    FT_FlushPipe(CONTROL_BULK_WRITE_ADDRESS);
    FT_FlushPipe(CONTROL_BULK_READ_ADDRESS);
    FT_FlushPipe(STREAM_BULK_WRITE_ADDRESS);
    FT_FlushPipe(STREAM_BULK_READ_ADDRESS);

    libusb_impl.Disconnect();
#else
    FT_Close(mFTHandle);
#endif
}

bool FT601::IsConnected()
{
#ifdef __unix__
    return libusb_impl.IsConnected();
#else
    return mFTHandle != nullptr;
#endif
}

int32_t FT601::BulkTransfer(uint8_t endPointAddr, uint8_t* data, size_t length, int32_t timeout_ms)
{
#ifdef __unix__
    return libusb_impl.BulkTransfer(endPointAddr, data, length, timeout_ms);
#else
    ULONG ulBytesTransferred = 0;
    FT_STATUS ftStatus = FT_OK;
    OVERLAPPED vOverlapped = { 0 };
    FT_InitializeOverlapped(mFTHandle, &vOverlapped);

    if (endPointAddr == CONTROL_BULK_WRITE_ADDRESS)
        ftStatus = FT_WritePipe(mFTHandle, CONTROL_BULK_WRITE_ADDRESS, data, length, &ulBytesTransferred, &vOverlapped);
    else
        ftStatus = FT_ReadPipe(mFTHandle, CONTROL_BULK_READ_ADDRESS, data, length, &ulBytesTransferred, &vOverlapped);

    bool failed = ftStatus != FT_IO_PENDING;
    if (!failed)
    {
        DWORD dwRet = WaitForSingleObject(vOverlapped.hEvent, timeout_ms);
        failed = !(dwRet == WAIT_OBJECT_0 || dwRet == WAIT_TIMEOUT);
        if (!failed)
            failed = FT_GetOverlappedResult(mFTHandle, &vOverlapped, &ulBytesTransferred, FALSE) != FT_OK;
    }

    if (failed)
    {
        FT_ReleaseOverlapped(mFTHandle, &vOverlapped);
        ReinitPipe(endPointAddr);
        ulBytesTransferred = 0;
    }
    return ulBytesTransferred;
#endif
}

int32_t FT601::ControlTransfer(int requestType, int request, int value, int index, uint8_t* data, size_t length, int32_t timeout_ms)
{
    return -1;
}

void* FT601::AllocateAsyncContext()
{
#ifdef __unix__
    return libusb_impl.AllocateAsyncContext();
#else
    return new FTDIAsyncContext();
#endif
}

OpStatus FT601::BeginDataXfer(void* context, uint8_t* buffer, size_t length, uint8_t endPointAddr)
{
#ifdef __unix__
    return libusb_impl.BeginDataXfer(context, buffer, length, endPointAddr);
#else
    ULONG ulActual{ 0 };
    FTDIAsyncContext* async{ reinterpret_cast<FTDIAsyncContext*>(context) };

    FT_STATUS ftStatus{ FT_OK };
    FT_InitializeOverlapped(mFTHandle, &async->inOvLap);
    bool manualReset{ true };
    bool initialState{ false };
    async->inOvLap.hEvent = CreateEvent(NULL, manualReset, initialState, NULL);
    if (endPointAddr == STREAM_BULK_READ_ADDRESS)
        ftStatus = FT_ReadPipe(mFTHandle, STREAM_BULK_READ_ADDRESS, buffer, length, &ulActual, &async->inOvLap);
    else
        ftStatus = FT_WritePipe(mFTHandle, STREAM_BULK_WRITE_ADDRESS, buffer, length, &ulActual, &async->inOvLap);

    async->endPointAddr = endPointAddr;
    return ftStatus != FT_IO_PENDING ? OpStatus::Error : OpStatus::Success;
#endif
}

OpStatus FT601::WaitForXfer(void* context, int32_t timeout_ms)
{
    assert(context);
#ifdef __unix__
    return libusb_impl.WaitForXfer(context, timeout_ms);
#else
    FTDIAsyncContext* async{ reinterpret_cast<FTDIAsyncContext*>(context) };
    DWORD dwRet = WaitForSingleObject(async->inOvLap.hEvent, timeout_ms);
    return (dwRet == WAIT_OBJECT_0) ? OpStatus::Success : OpStatus::Timeout;
#endif
}

size_t FT601::FinishDataXfer(void* context)
{
    assert(context);
#ifdef __unix__
    return libusb_impl.FinishDataXfer(context);
#else
    FTDIAsyncContext* async{ reinterpret_cast<FTDIAsyncContext*>(context) };

    ULONG ulActualBytesTransferred{ 0 };
    FT_STATUS ftStatus = FT_GetOverlappedResult(mFTHandle, &async->inOvLap, &ulActualBytesTransferred, FALSE);
    FT_ReleaseOverlapped(mFTHandle, &async->inOvLap);
    return (ftStatus != FT_OK) ? 0 : ulActualBytesTransferred;
#endif
}

OpStatus FT601::AbortXfer(void* context)
{
    assert(context);
#ifdef __unix__
    return libusb_impl.AbortXfer(context);
#else
    FTDIAsyncContext* async = reinterpret_cast<FTDIAsyncContext*>(context);

    FT_AbortPipe(mFTHandle, async->endPointAddr);
    FT_ReleaseOverlapped(mFTHandle, &async->inOvLap);

    if (async->endPointAddr == STREAM_BULK_READ_ADDRESS)
        FT_FlushPipe(mFTHandle, STREAM_BULK_READ_ADDRESS);
    return OpStatus::Success;
#endif
}

void FT601::FreeAsyncContext(void* context)
{
    assert(context);
#ifdef __unix__
    libusb_impl.FreeAsyncContext(context);
#else
    delete reinterpret_cast<FTDIAsyncContext*>(context);
#endif
}

OpStatus FT601::ResetStreamBuffers()
{
    constexpr int streamBufferSize = 4096;
#ifdef __unix__
    if (FT_FlushPipe(STREAM_BULK_WRITE_ADDRESS) != 0)
        return OpStatus::Error;

    if (FT_FlushPipe(STREAM_BULK_READ_ADDRESS) != 0)
        return OpStatus::Error;

    if (FT_SetStreamPipe(STREAM_BULK_WRITE_ADDRESS, streamBufferSize) != 0)
        return OpStatus::Error;

    if (FT_SetStreamPipe(STREAM_BULK_READ_ADDRESS, streamBufferSize) != 0)
        return OpStatus::Error;
#else
    if (FT_AbortPipe(mFTHandle, STREAM_BULK_READ_ADDRESS) != FT_OK)
        return OpStatus::Error;

    if (FT_AbortPipe(mFTHandle, STREAM_BULK_WRITE_ADDRESS) != FT_OK)
        return OpStatus::Error;

    if (FT_SetStreamPipe(mFTHandle, FALSE, FALSE, STREAM_BULK_READ_ADDRESS, streamBufferSize) != 0)
        return OpStatus::Error;

    if (FT_SetStreamPipe(mFTHandle, FALSE, FALSE, STREAM_BULK_WRITE_ADDRESS, streamBufferSize) != 0)
        return OpStatus::Error;
#endif
    return OpStatus::Success;
}

#ifndef __unix__
int FT601::ReinitPipe(unsigned char ep)
{
    FT_AbortPipe(mFTHandle, ep);
    FT_FlushPipe(mFTHandle, ep);
    FT_SetStreamPipe(mFTHandle, FALSE, FALSE, ep, 64);
    return 0;
}
#endif

#ifdef __unix__
int FT601::FT_FlushPipe(unsigned char ep)
{
    uint8_t wbuffer[20];
    std::memset(wbuffer, 0, sizeof(wbuffer));

    mUsbCounter++;
    wbuffer[0] = mUsbCounter & 0xFF;
    wbuffer[1] = (mUsbCounter >> 8) & 0xFF;
    wbuffer[2] = (mUsbCounter >> 16) & 0xFF;
    wbuffer[3] = (mUsbCounter >> 24) & 0xFF;
    wbuffer[4] = ep;

    int actual = BulkTransfer(0x01, wbuffer, 20, 1000);
    if (actual != 20)
        return -1;

    mUsbCounter++;
    wbuffer[0] = mUsbCounter & 0xFF;
    wbuffer[1] = (mUsbCounter >> 8) & 0xFF;
    wbuffer[2] = (mUsbCounter >> 16) & 0xFF;
    wbuffer[3] = (mUsbCounter >> 24) & 0xFF;
    wbuffer[4] = ep;
    wbuffer[5] = 0x03;

    actual = BulkTransfer(0x01, wbuffer, 20, 1000);
    if (actual != 20)
        return -1;

    return 0;
}

int FT601::FT_SetStreamPipe(unsigned char ep, size_t size)
{
    uint8_t wbuffer[20];
    memset(wbuffer, 0, sizeof(wbuffer));

    mUsbCounter++;
    wbuffer[0] = mUsbCounter & 0xFF;
    wbuffer[1] = (mUsbCounter >> 8) & 0xFF;
    wbuffer[2] = (mUsbCounter >> 16) & 0xFF;
    wbuffer[3] = (mUsbCounter >> 24) & 0xFF;
    wbuffer[4] = ep;

    int actual = BulkTransfer(0x01, wbuffer, 20, 1000);
    if (actual != 20)
        return -1;

    mUsbCounter++;
    wbuffer[0] = mUsbCounter & 0xFF;
    wbuffer[1] = (mUsbCounter >> 8) & 0xFF;
    wbuffer[2] = (mUsbCounter >> 16) & 0xFF;
    wbuffer[3] = (mUsbCounter >> 24) & 0xFF;
    wbuffer[5] = 0x02;
    wbuffer[8] = size & 0xFF;
    wbuffer[9] = (size >> 8) & 0xFF;
    wbuffer[10] = (size >> 16) & 0xFF;
    wbuffer[11] = (size >> 24) & 0xFF;

    actual = BulkTransfer(0x01, wbuffer, 20, 1000);
    if (actual != 20)
        return -1;

    return 0;
}
#endif

void FT601::AddOnHotplugDisconnectCallback(const IUSB::HotplugDisconnectCallbackType& function, void* userData)
{
#ifdef __unix__
    libusb_impl.AddOnHotplugDisconnectCallback(function, userData);
#else
    // Hotplug events are not supported by the library
#endif
}

} // namespace lime
