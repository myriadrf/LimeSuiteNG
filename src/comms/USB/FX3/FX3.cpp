#include "FX3.h"

#include <cassert>

using namespace lime;
using namespace std::literals::string_literals;

#ifdef __unix__
    #ifdef __GNUC__
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Wpedantic"
    #endif
    #include <libusb.h>
    #ifdef __GNUC__
        #pragma GCC diagnostic pop
    #endif
const int FX3::CTR_WRITE_REQUEST_VALUE = LIBUSB_REQUEST_TYPE_VENDOR;
const int FX3::CTR_READ_REQUEST_VALUE = LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN;
#else
    #include "windows.h"
    #include "CyAPI.h"
    #include <codecvt>

class FX3AsyncContext
{
  public:
    FX3AsyncContext()
        : endpoint(nullptr)
    {
        memset(&inOvLap, 0, sizeof(OVERLAPPED));
        bool manualReset = true;
        bool initialState = false;
        inOvLap.hEvent = CreateEvent(NULL, manualReset, initialState, NULL);
    }
    ~FX3AsyncContext() { CloseHandle(inOvLap.hEvent); }

    void Reset() { ResetEvent(inOvLap.hEvent); }

    CCyUSBEndPoint* endpoint;
    OVERLAPPED inOvLap;
    PUCHAR pXmitBuf;
    uint8_t* buffer;
};

const int FX3::CTR_WRITE_REQUEST_VALUE = 0;
const int FX3::CTR_READ_REQUEST_VALUE = 1;
#endif // __unix__

std::vector<USBDescriptor> FX3::enumerateDevices(const std::set<VendorProductId>& ids)
{
#ifdef __unix__
    return libusb_impl.enumerateDevices(ids);
#else
    std::vector<USBDescriptor> devDescriptors;

    CCyUSBDevice device;
    if (device.DeviceCount() <= 0)
        return devDescriptors;

    for (int i = 0; i < device.DeviceCount(); ++i)
    {
        if (device.IsOpen())
            device.Close();
        if (!device.Open(i))
        {
            device.Reset();
            if (!device.Open(i))
                continue;
        }

        if (!ids.empty() && ids.find({ device.VendorID, device.ProductID }) == ids.end())
        {
            device.Close();
            continue;
        }

        USBDescriptor desc;
        desc.vid = device.VendorID;
        desc.pid = device.ProductID;
        desc.product = device.DeviceName;
        desc.serial = std::wstring_convert<std::codecvt_utf8<wchar_t>>().to_bytes(device.SerialNumber);
        devDescriptors.push_back(desc);
        device.Close();
    }
    return devDescriptors;
#endif
}

FX3::FX3()
#ifndef __unix__
    : fx3device(new CCyFX3Device())
#endif
{
}

FX3::~FX3()
{
    Disconnect();
}

bool FX3::Connect(uint16_t vid, uint16_t pid, const char* serial)
{
    Disconnect();
#ifdef __unix__
    return libusb_impl.Connect(vid, pid, serial);
#else // windows
    if (fx3device->DeviceCount() == 0)
        return false;

    if (fx3device->Open(0) == false)
        return false;

    for (int i = 0; i < fx3device->EndPointCount(); ++i)
    {
        CCyUSBEndPoint* ep = fx3device->EndPoints[i];
        long len = ep->MaxPktSize * 64;
        ep->SetXferSize(len);
        endpoints[ep->Address] = ep;
    }
#endif
    return true;
}

void FX3::Disconnect()
{
#ifdef __unix__
    libusb_impl.Disconnect();
#else
    if (!fx3device)
        return;

    endpoints.clear();
    fx3device->Close();
#endif
}

bool FX3::IsConnected()
{
#ifdef __unix__
    return libusb_impl.IsConnected();
#else
    return fx3device ? fx3device->IsOpen() : false;
#endif
}

int32_t FX3::BulkTransfer(uint8_t endPoint, uint8_t* data, size_t length, int32_t timeout_ms)
{
#ifdef __unix__
    return libusb_impl.BulkTransfer(endPoint, data, length, timeout_ms);
#else
    LONG longLength = static_cast<LONG>(length);
    if (endpoints[endPoint]->XferData(data, longLength))
        return length;
    else
        return 0;
#endif
}

int32_t FX3::ControlTransfer(int requestType, int request, int value, int index, uint8_t* data, size_t length, int32_t timeout_ms)
{
#ifdef __unix__
    return libusb_impl.ControlTransfer(requestType, request, value, index, data, length, timeout_ms);
#else
    LONG longLength{ static_cast<LONG>(length) };
    CCyControlEndPoint control(*fx3device->ControlEndPt);
    control.ReqCode = request;
    control.Value = value;
    control.Index = index;
    control.TimeOut = timeout_ms;

    switch (requestType)
    {
    case 0: // Write
        if (control.Write(data, longLength))
            return longLength;
        break;
    case 1: // Read
        if (control.Read(data, longLength))
            return longLength;
        break;
    default:
        return 0;
    }
    return 0;
#endif
}

void* FX3::AllocateAsyncContext()
{
#ifdef __unix__
    return libusb_impl.AllocateAsyncContext();
#else
    return new FX3AsyncContext();
#endif
}

OpStatus FX3::BeginDataXfer(void* context, uint8_t* buffer, size_t length, uint8_t endPointAddr)
{
#ifdef __unix__
    return libusb_impl.BeginDataXfer(context, buffer, length, endPointAddr);
#else
    FX3AsyncContext* async = reinterpret_cast<FX3AsyncContext*>(context);
    async->Reset();
    async->buffer = buffer;
    async->endpoint = endpoints[endPointAddr];
    async->pXmitBuf = async->endpoint->BeginDataXfer(buffer, length, &async->inOvLap);
    return OpStatus::Success;
#endif
}

OpStatus FX3::WaitForXfer(void* context, int32_t timeout_ms)
{
#ifdef __unix__
    return libusb_impl.WaitForXfer(context, timeout_ms);
#else
    FX3AsyncContext* async = reinterpret_cast<FX3AsyncContext*>(context);
    bool status = async->endpoint->WaitForXfer(&async->inOvLap, timeout_ms);
    return status ? OpStatus::Success : OpStatus::Timeout;
#endif
}

size_t FX3::FinishDataXfer(void* context)
{
#ifdef __unix__
    return libusb_impl.FinishDataXfer(context);
#else
    FX3AsyncContext* async = reinterpret_cast<FX3AsyncContext*>(context);
    LONG receivedBytes = 0;
    bool status = async->endpoint->FinishDataXfer(async->buffer, receivedBytes, &async->inOvLap, async->pXmitBuf);
    return status ? receivedBytes : 0;
#endif
}

OpStatus FX3::AbortXfer(void* context)
{
#ifdef __unix__
    return libusb_impl.AbortXfer(context);
#else
    FX3AsyncContext* async = reinterpret_cast<FX3AsyncContext*>(context);
    return async->endpoint->Abort() ? OpStatus::Success : OpStatus::Error;
#endif
}

void FX3::FreeAsyncContext(void* context)
{
    assert(context);
#ifdef __unix__
    libusb_impl.FreeAsyncContext(context);
#else
    delete reinterpret_cast<FX3AsyncContext*>(context);
#endif
}
