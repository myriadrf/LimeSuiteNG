#include "USBGeneric.h"
#include "limesuiteng/Logger.h"

#include <cassert>

using namespace std::literals::string_literals;

namespace lime {

#ifdef __unix__
int USBGeneric::activeUSBconnections = 0;
std::thread USBGeneric::gUSBProcessingThread{};

void USBGeneric::HandleLibusbEvents()
{
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;

    while (activeUSBconnections > 0)
    {
        int returnCode = libusb_handle_events_timeout_completed(ctx, &tv, NULL);

        if (returnCode != 0)
        {
            lime::error("error libusb_handle_events %s", libusb_strerror(static_cast<libusb_error>(returnCode)));
        }
    }
}
#endif // __UNIX__

USBGeneric::USBGeneric(void* usbContext)
    : contexts(nullptr)
    , isConnected(false)
{
#ifdef __unix__
    dev_handle = nullptr;
    ctx = reinterpret_cast<libusb_context*>(usbContext);

    if (ctx == nullptr)
    {
        return;
    }

    ++activeUSBconnections;

    if (activeUSBconnections == 1)
    {
        gUSBProcessingThread = std::thread(&USBGeneric::HandleLibusbEvents, this);
    }
#endif
}

USBGeneric::~USBGeneric()
{
    Disconnect();
#ifdef __unix__
    if (ctx == nullptr)
    {
        return;
    }

    --activeUSBconnections;

    if (activeUSBconnections == 0)
    {
        if (gUSBProcessingThread.joinable())
        {
            gUSBProcessingThread.join();
        }
    }
#endif
}

bool USBGeneric::Connect(uint16_t vid, uint16_t pid, const std::string_view serial)
{
#ifdef __unix__
    libusb_device** devs; // Pointer to pointer of device, used to retrieve a list of devices
    int usbDeviceCount = libusb_get_device_list(ctx, &devs);

    if (usbDeviceCount < 0)
    {
        lime::error("libusb_get_device_list failed: %s", libusb_strerror(static_cast<libusb_error>(usbDeviceCount)));
        return false;
    }

    for (int i = 0; i < usbDeviceCount; ++i)
    {
        libusb_device_descriptor desc;
        int returnCode = libusb_get_device_descriptor(devs[i], &desc);

        if (returnCode < 0)
        {
            lime::error("failed to get device description"s);
            continue;
        }

        if (desc.idProduct != pid || desc.idVendor != vid)
        {
            continue;
        }

        if (libusb_open(devs[i], &dev_handle) != 0)
        {
            continue;
        }

        std::string foundSerial;
        if (desc.iSerialNumber > 0)
        {
            char data[255];
            int stringLength = libusb_get_string_descriptor_ascii(
                dev_handle, desc.iSerialNumber, reinterpret_cast<unsigned char*>(data), sizeof(data));

            if (stringLength < 0)
            {
                lime::error("Failed to get serial number"s);
            }
            else
            {
                foundSerial = std::string(data, static_cast<size_t>(stringLength));
            }
        }

        if (serial == foundSerial)
        {
            break; //found it
        }

        libusb_close(dev_handle);
        dev_handle = nullptr;
    }

    libusb_free_device_list(devs, 1);

    if (dev_handle == nullptr)
    {
        lime::error("libusb_open failed"s);
        return false;
    }

    if (libusb_kernel_driver_active(dev_handle, 0) == 1) // Find out if kernel driver is attached
    {
        lime::info("Kernel Driver Active"s);

        if (libusb_detach_kernel_driver(dev_handle, 0) == 0) // Detach it
        {
            lime::info("Kernel Driver Detached!"s);
        }
    }

    int returnCode = libusb_claim_interface(dev_handle, 0); // Claim interface 0 (the first) of device
    if (returnCode != LIBUSB_SUCCESS)
    {
        lime::error("Cannot claim USB interface: %s", libusb_strerror(libusb_error(returnCode)));
        return false;
    }
#endif
    isConnected = true;
    return true;
}

inline bool USBGeneric::IsConnected()
{
    return isConnected;
}

void USBGeneric::Disconnect()
{
    if (contexts == nullptr)
    {
        return;
    }

#ifdef __unix__
    const libusb_version* ver = libusb_get_version();
    // Fix #358 libusb crash when freeing transfers(never used ones) without valid device handle. Bug in libusb 1.0.25 https://github.com/libusb/libusb/issues/1059
    const bool isBuggy_libusb_free_transfer = ver->major == 1 && ver->minor == 0 && ver->micro == 25;

    if (isBuggy_libusb_free_transfer)
    {
        for (int i = 0; i < USB_MAX_CONTEXTS; ++i)
        {
            contexts[i].transfer->dev_handle = dev_handle;
        }
    }

    std::unique_lock<std::mutex> lock{ contextsLock };

    for (int i = 0; i < USB_MAX_CONTEXTS; ++i)
    {
        if (contexts[i].isTransferUsed)
        {
            AbortEndpointXfers(contexts[i].transfer->endpoint);
        }
    }
#endif

    delete[] contexts;
    contexts = nullptr;
}

int32_t USBGeneric::BulkTransfer(uint8_t endPointAddr, uint8_t* data, int length, int32_t timeout_ms)
{
    long len = 0;
    if (!IsConnected())
    {
        throw std::runtime_error("BulkTransfer: USB device is not connected"s);
    }

    assert(data);

#ifdef __unix__
    int actualTransferred = 0;
    int status = libusb_bulk_transfer(dev_handle, endPointAddr, data, length, &actualTransferred, timeout_ms);
    len = actualTransferred;

    if (status != 0)
    {
        lime::error("USBGeneric::BulkTransfer(0x%02X) : %s, transferred: %i, expected: %i",
            endPointAddr,
            libusb_error_name(status),
            actualTransferred,
            length);
    }
#endif
    return len;
}

int32_t USBGeneric::ControlTransfer(
    int requestType, int request, int value, int index, uint8_t* data, uint32_t length, int32_t timeout_ms)
{
    long len = length;
    if (!IsConnected())
    {
        throw std::runtime_error("ControlTransfer: USB device is not connected"s);
    }

    assert(data);
#ifdef __unix__
    len = libusb_control_transfer(dev_handle, requestType, request, value, index, data, length, timeout_ms);
#endif
    return len;
}

#ifdef __unix__
/** @brief Function for handling libusb callbacks
*/
static void process_libusbtransfer(libusb_transfer* trans)
{
    USBTransferContext* context = static_cast<USBTransferContext*>(trans->user_data);
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
#endif

int USBGeneric::BeginDataXfer(uint8_t* buffer, uint32_t length, uint8_t endPointAddr)
{
#ifdef __unix__
    int i = GetUSBContextIndex();

    if (i < 0)
    {
        return -1;
    }

    libusb_transfer* tr = contexts[i].transfer;
    libusb_fill_bulk_transfer(tr, dev_handle, endPointAddr, buffer, length, process_libusbtransfer, &contexts[i], 0);
    contexts[i].done = false;
    contexts[i].bytesXfered = 0;
    int status = libusb_submit_transfer(tr);

    if (status != 0)
    {
        lime::error("BEGIN DATA TRANSFER %s", libusb_error_name(status));
        contexts[i].isTransferUsed = false;
        return -1;
    }

    return i;
#endif
    return 0;
}

bool USBGeneric::WaitForXfer(int contextHandle, int32_t timeout_ms)
{
#ifdef __unix__
    if (contextHandle >= 0 && contexts[contextHandle].isTransferUsed == true)
    {
        // Blocking not to waste CPU
        std::unique_lock<std::mutex> lck(contexts[contextHandle].transferLock);
        return contexts[contextHandle].cv.wait_for(
            lck, std::chrono::milliseconds(timeout_ms), [&]() { return contexts[contextHandle].done.load(); });
    }
#endif
    return true; // There is nothing to wait for (signal wait finished)
}

int USBGeneric::FinishDataXfer(uint8_t* buffer, uint32_t length, int contextHandle)
{
#ifdef __unix__
    if (contextHandle >= 0 && contexts[contextHandle].isTransferUsed == true)
    {
        length = contexts[contextHandle].bytesXfered;
        contexts[contextHandle].isTransferUsed = false;
        contexts[contextHandle].Reset();
        return length;
    }
#endif

    return 0;
}

void USBGeneric::AbortEndpointXfers(uint8_t endPointAddr)
{
    if (contexts == nullptr)
    {
        return;
    }

#ifdef __unix__
    for (int i = 0; i < USB_MAX_CONTEXTS; ++i)
    {
        if (contexts[i].isTransferUsed && contexts[i].transfer->endpoint == endPointAddr)
        {
            libusb_cancel_transfer(contexts[i].transfer);
        }
    }
#endif
    WaitForXfers(endPointAddr);
}

int USBGeneric::GetUSBContextIndex()
{
    std::unique_lock<std::mutex> lock{ contextsLock };

    if (contexts == nullptr)
    {
        return -1;
    }

    int i = 0;
    bool contextFound = false;
    // Find not used context
    for (i = 0; i < USB_MAX_CONTEXTS; i++)
    {
        if (!contexts[i].isTransferUsed)
        {
            contextFound = true;
            break;
        }
    }

    if (!contextFound)
    {
        lime::error("No contexts left for reading or sending data"s);
        return -1;
    }

    contexts[i].isTransferUsed = true;

    return i;
}

void USBGeneric::WaitForXfers(uint8_t endPointAddr)
{
#ifdef __unix__
    for (int i = 0; i < USB_MAX_CONTEXTS; ++i)
    {
        if (contexts[i].isTransferUsed && contexts[i].transfer->endpoint == endPointAddr)
        {
            WaitForXfer(i, 250);
            FinishDataXfer(nullptr, 0, i);
        }
    }
#endif
}

} // namespace lime
