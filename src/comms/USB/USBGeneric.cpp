#include "USBGeneric.h"

#include <thread>
#include <cassert>

#ifdef __unix__
    #ifdef __GNUC__
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Wpedantic"
    #endif
    #include <libusb.h>
    #ifdef __GNUC__
        #pragma GCC diagnostic pop
    #endif
#endif

#include "limesuiteng/Logger.h"

using namespace std::literals::string_literals;

namespace lime {

static libusb_context* gContextLibUsb{ nullptr };
static int activeUSBconnections = 0;
static std::thread gUSBProcessingThread; // single thread for processing USB callbacks

static void HandleLibusbEvents(libusb_context* context)
{
    timeval tv{};
    tv.tv_sec = 0;
    tv.tv_usec = 100000;

    while (activeUSBconnections > 0)
    {
        int returnCode = libusb_handle_events_timeout_completed(context, &tv, NULL);
        if (returnCode != 0)
            lime::error("libusb_handle_events: %s", libusb_strerror(static_cast<libusb_error>(returnCode)));
    }
}

static int SessionRefCountIncrement()
{
    ++activeUSBconnections;
    if (activeUSBconnections == 1)
    {
        int returnCode = libusb_init(&gContextLibUsb); // Initialize the library for the session we just declared
        if (returnCode < 0)
            lime::error("USBGeneric::libusb_init Error %i", returnCode); // There was an error

#if LIBUSBX_API_VERSION < 0x01000106
        libusb_set_debug(gContextLibUsb, 3); // Set verbosity level to 3, as suggested in the documentation
#else
        libusb_set_option(gContextLibUsb,
            LIBUSB_OPTION_LOG_LEVEL,
            LIBUSB_LOG_LEVEL_INFO); // Set verbosity level to info, as suggested in the documentation
#endif
        if (gContextLibUsb)
            gUSBProcessingThread = std::thread(HandleLibusbEvents, gContextLibUsb);
    }
    return activeUSBconnections;
}

static int SessionRefCountDecrement()
{
    --activeUSBconnections;
    if (activeUSBconnections == 0 && gUSBProcessingThread.joinable())
    {
        gUSBProcessingThread.join();
        libusb_exit(gContextLibUsb);
        gContextLibUsb = nullptr;
    }
    return activeUSBconnections;
}

static void process_libusbtransfer(libusb_transfer* trans)
{
    USBGeneric::AsyncContext* context = static_cast<USBGeneric::AsyncContext*>(trans->user_data);
    std::unique_lock<std::mutex> lck(context->transferLock);
    switch (trans->status)
    {
    case LIBUSB_TRANSFER_CANCELLED:
        context->bytesXfered = trans->actual_length;
        context->done.store(true);
        break;
    case LIBUSB_TRANSFER_COMPLETED:
        context->bytesXfered = trans->actual_length;
        context->done.store(true);
        break;
    case LIBUSB_TRANSFER_ERROR:
        lime::error("USB TRANSFER ERROR"s);
        context->bytesXfered = trans->actual_length;
        context->done.store(true);
        break;
    case LIBUSB_TRANSFER_TIMED_OUT:
        context->bytesXfered = trans->actual_length;
        context->done.store(true);
        break;
    case LIBUSB_TRANSFER_OVERFLOW:
        lime::error("USB transfer overflow"s);
        break;
    case LIBUSB_TRANSFER_STALL:
        lime::error("USB transfer stalled"s);
        break;
    case LIBUSB_TRANSFER_NO_DEVICE:
        lime::error("USB transfer no device"s);
        context->done.store(true);
        break;
    }
    lck.unlock();
    context->cv.notify_one();
}

int USBGeneric::HotplugCallback(libusb_context* ctx, libusb_device* device, libusb_hotplug_event event, void* user_data)
{
    auto* usb = reinterpret_cast<USBGeneric*>(user_data);

    for (const auto& callback : usb->hotplugDisconnectCallbacks)
    {
        callback.function(callback.userData);
    }

    usb->Disconnect();
    return 1;
}

USBGeneric::AsyncContext::AsyncContext()
    : transfer(libusb_alloc_transfer(0))
    , bytesXfered(0)
    , done{ false }
{
}

USBGeneric::AsyncContext::~AsyncContext()
{
    libusb_free_transfer(transfer);
}

void USBGeneric::AsyncContext::Reset()
{
    done.store(false);
    bytesXfered = 0;
}

static std::string GetStringDescriptor(libusb_device_handle* handle, const libusb_device_descriptor& src, uint8_t index)
{
    char ctemp[512];
    int strLen = libusb_get_string_descriptor_ascii(handle, index, reinterpret_cast<unsigned char*>(ctemp), sizeof(ctemp));

    if (strLen < 0)
    {
        lime::warning("Failed to get descriptor string"s);
        return std::string();
    }
    return std::string(ctemp);
}

static void TransferUSBDescriptor(libusb_device* device, USBDescriptor& dest, const libusb_device_descriptor& src)
{
    dest.vid = src.idVendor;
    dest.pid = src.idProduct;

    libusb_device_handle* handle;
    int openStatus = libusb_open(device, &handle);
    if (openStatus != LIBUSB_SUCCESS)
        return;

    if (src.iSerialNumber > 0)
        dest.serial = GetStringDescriptor(handle, src, src.iSerialNumber);

    if (src.iProduct > 0)
        dest.product = GetStringDescriptor(handle, src, src.iProduct);

    libusb_close(handle);
}

std::vector<USBDescriptor> USBGeneric::enumerateDevices(const std::set<IUSB::VendorProductId>& ids)
{
    std::vector<USBDescriptor> devDescriptors;

    libusb_device** devs; // used to retrieve a list of devices
    int usbDeviceCount = libusb_get_device_list(gContextLibUsb, &devs);
    if (usbDeviceCount < 0)
    {
        lime::error("USBGeneric: Failed to get libusb device list: %s", libusb_strerror(libusb_error(usbDeviceCount)));
        return devDescriptors;
    }

    for (int i = 0; i < usbDeviceCount; ++i)
    {
        libusb_device_descriptor desc{};
        int returnCode = libusb_get_device_descriptor(devs[i], &desc);
        if (returnCode < 0)
        {
            lime::error("Failed to get device description"s);
            continue;
        }

        if (ids.find({ desc.idVendor, desc.idProduct }) == ids.end())
            continue;

        USBDescriptor dest;
        TransferUSBDescriptor(devs[i], dest, desc);
        devDescriptors.push_back(dest);
    }
    libusb_free_device_list(devs, 1);
    return devDescriptors;
}

USBGeneric::USBGeneric()
    : dev_handle(nullptr)
{
    SessionRefCountIncrement();
}

USBGeneric::~USBGeneric()
{
    Disconnect();
    SessionRefCountDecrement();
}

bool USBGeneric::Connect(uint16_t vid, uint16_t pid, const char* serial)
{
    libusb_device** devs; // Pointer to pointer of device, used to retrieve a list of devices
    int usbDeviceCount = libusb_get_device_list(gContextLibUsb, &devs);

    if (usbDeviceCount < 0)
    {
        lime::error("libusb_get_device_list failed: %s", libusb_strerror(static_cast<libusb_error>(usbDeviceCount)));
        return false;
    }

    for (int i = 0; i < usbDeviceCount; ++i)
    {
        libusb_device_descriptor desc{};
        int returnCode = libusb_get_device_descriptor(devs[i], &desc);

        if (returnCode < 0)
        {
            lime::error("failed to get device description"s);
            continue;
        }

        if (desc.idProduct != pid || desc.idVendor != vid)
            continue;

        int openStatus = libusb_open(devs[i], &dev_handle);
        switch (openStatus)
        {
        case LIBUSB_ERROR_ACCESS:
            lime::error("Insufficient permissions to open USB device");
            break;
        case LIBUSB_ERROR_NO_MEM:
            lime::error("USB device memory allocation failed");
            break;
        case LIBUSB_ERROR_NO_DEVICE:
            lime::error("Expected device has been disconnected");
            continue;
        }

        std::string foundSerial;
        if (desc.iSerialNumber > 0)
        {
            char data[255];
            int stringLength = libusb_get_string_descriptor_ascii(
                dev_handle, desc.iSerialNumber, reinterpret_cast<unsigned char*>(data), sizeof(data));

            if (stringLength < 0)
                lime::error("Failed to get serial number"s);
            else
                foundSerial = std::string(data, static_cast<size_t>(stringLength));
        }

        if (std::string{ serial }.empty() || std::string{ serial } == foundSerial)
        {
            libusb_hotplug_register_callback(gContextLibUsb,
                LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT,
                0,
                desc.idVendor,
                desc.idProduct,
                LIBUSB_HOTPLUG_MATCH_ANY,
                HotplugCallback,
                this,
                nullptr);

            break; //found it
        }

        libusb_close(dev_handle);
        dev_handle = nullptr;
    }

    libusb_free_device_list(devs, 1);

    if (!dev_handle)
    {
        lime::error("libusb_open failed"s);
        return false;
    }

    OpStatus status = ClaimInterface(0);
    if (status != OpStatus::Success)
    {
        Disconnect();
        return false;
    }
    return true;
}

bool USBGeneric::IsConnected()
{
    return dev_handle != nullptr;
}

void USBGeneric::Disconnect()
{
    if (dev_handle)
    {
        libusb_release_interface(dev_handle, 0);
        libusb_release_interface(dev_handle, 1);
        libusb_close(dev_handle);
    }
    dev_handle = nullptr;
}

int32_t USBGeneric::BulkTransfer(uint8_t endPointAddr, uint8_t* data, size_t length, int32_t timeout_ms)
{
    assert(data);
    if (!IsConnected())
        throw std::runtime_error("BulkTransfer: USB device is not connected"s);

    int actualTransferred = 0;
    int status = libusb_bulk_transfer(dev_handle, endPointAddr, data, length, &actualTransferred, timeout_ms);
    if (status != 0)
    {
        lime::error("USBGeneric::BulkTransfer(0x%02X) : %s, transferred: %i, expected: %i",
            endPointAddr,
            libusb_error_name(status),
            actualTransferred,
            length);
    }
    return actualTransferred;
}

int32_t USBGeneric::ControlTransfer(
    int requestType, int request, int value, int index, uint8_t* data, size_t length, int32_t timeout_ms)
{
    assert(data);
    if (!IsConnected())
        return -1;
    return libusb_control_transfer(dev_handle, requestType, request, value, index, data, length, timeout_ms);
}

void* USBGeneric::AllocateAsyncContext()
{
    return new AsyncContext();
}

OpStatus USBGeneric::BeginDataXfer(void* context, uint8_t* buffer, size_t length, uint8_t endPointAddr)
{
    assert(context);
    AsyncContext* xfer = reinterpret_cast<AsyncContext*>(context);
    assert(xfer->done.load() == false);

    libusb_transfer* tr = xfer->transfer;
    libusb_fill_bulk_transfer(tr, dev_handle, endPointAddr, buffer, length, process_libusbtransfer, xfer, 0);
    xfer->done.store(false);
    xfer->bytesXfered = 0;
    int status = libusb_submit_transfer(tr);
    if (status != 0)
    {
        lime::error("USBGeneric::BeginDataXfer error: %s", libusb_error_name(status));
        return OpStatus::Error;
    }
    return OpStatus::Success;
}

OpStatus USBGeneric::WaitForXfer(void* context, int32_t timeout_ms)
{
    assert(context);

    AsyncContext* xfer = reinterpret_cast<AsyncContext*>(context);

    // Blocking not to waste CPU
    std::unique_lock<std::mutex> lck(xfer->transferLock);
    bool value = xfer->cv.wait_for(lck, std::chrono::milliseconds(timeout_ms), [&]() { return xfer->done.load(); });
    return value ? OpStatus::Success : OpStatus::Timeout;
}

size_t USBGeneric::FinishDataXfer(void* context)
{
    assert(context);
    AsyncContext* xfer = reinterpret_cast<AsyncContext*>(context);
    size_t length = xfer->bytesXfered;
    xfer->Reset();
    return length;
}

OpStatus USBGeneric::AbortXfer(void* context)
{
    assert(context);
    AsyncContext* xfer = reinterpret_cast<AsyncContext*>(context);
    libusb_cancel_transfer(xfer->transfer);
    return OpStatus::Success;
}

void USBGeneric::FreeAsyncContext(void* context)
{
    assert(context);
    delete reinterpret_cast<AsyncContext*>(context);
}

OpStatus USBGeneric::ClaimInterface(int32_t interface_number)
{
    assert(dev_handle);
    if (libusb_kernel_driver_active(dev_handle, interface_number) == 1) // Find out if kernel driver is attached
    {
        lime::debug("libusb: kernel driver is active."s);
        int returnCode = libusb_detach_kernel_driver(dev_handle, interface_number);
        if (returnCode == LIBUSB_SUCCESS)
            lime::debug("libusb: kernel driver detached!"s);
        else
        {
            lime::error("libusb: failed to detach kernel driver, %s.", libusb_strerror(libusb_error(returnCode)));
            return OpStatus::Error;
        }
    }

    int returnCode = libusb_claim_interface(dev_handle, interface_number); // Claim interface 0 (the first) of device
    if (returnCode != LIBUSB_SUCCESS)
    {
        lime::error("libusb: cannot claim USB interface: %s", libusb_strerror(libusb_error(returnCode)));
        return OpStatus::Error;
    }
    return OpStatus::Success;
}

void USBGeneric::AddOnHotplugDisconnectCallback(const IUSB::HotplugDisconnectCallbackType& function, void* userData)
{
    hotplugDisconnectCallbacks.push_back({ function, userData });
}

} // namespace lime
