#pragma once
#include "IUSB.h"

#include <atomic>
#include <condition_variable>
#include <mutex>

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

namespace lime {

/// @brief A generic class to communicate with a USB device. Implementation based on libusb.
class USBGeneric : IUSB
{
  public:
    /// @brief Data context for tracking asynchronous transfers.
    class AsyncContext
    {
      public:
        AsyncContext();
        ~AsyncContext();
        void Reset();

        libusb_transfer* transfer;
        size_t bytesXfered;
        std::atomic<bool> done;
        std::mutex transferLock;
        std::condition_variable cv;
    };

    std::vector<USBDescriptor> enumerateDevices(const std::set<VendorProductId>& ids) override;

    USBGeneric();
    virtual ~USBGeneric();
    static constexpr int32_t defaultTimeout = 1000; ///< The default timeout to use if none is specified.

    bool Connect(uint16_t vid, uint16_t pid, const char* serial) override;
    bool IsConnected() override;

    void Disconnect() override;

    int32_t BulkTransfer(uint8_t endPoint, uint8_t* data, size_t length, int32_t timeout_ms = defaultTimeout) override;
    int32_t ControlTransfer(
        int requestType, int request, int value, int index, uint8_t* data, size_t length, int32_t timeout_ms = defaultTimeout)
        override;

    void* AllocateAsyncContext() override;
    OpStatus BeginDataXfer(void* context, uint8_t* buffer, size_t length, uint8_t endPointAddr) override;
    OpStatus WaitForXfer(void* context, int32_t timeout_ms) override;
    size_t FinishDataXfer(void* context) override;
    OpStatus AbortXfer(void* context) override;
    void FreeAsyncContext(void* context) override;

    OpStatus ClaimInterface(int32_t interface_number);

    void AddOnHotplugDisconnectCallback(const HotplugDisconnectCallbackType& function, void* userData) override;

  private:
    libusb_device_handle* dev_handle; //a device handle

    std::vector<IUSB::CallbackInfo<IUSB::HotplugDisconnectCallbackType>> hotplugDisconnectCallbacks{};

    static int HotplugCallback(libusb_context* ctx, libusb_device* device, libusb_hotplug_event event, void* user_data);
};

} // namespace lime
