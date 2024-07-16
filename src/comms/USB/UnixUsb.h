#pragma once
#include "IUSB.h"

#include <atomic>
#include <condition_variable>
#include <mutex>

struct libusb_device_handle;
struct libusb_transfer;

namespace lime {

/// @brief Class to communicate with a USB device on UNIX. Implementation based on libusb.
class UnixUsb : IUSB
{
  public:
    std::vector<USBDescriptor> enumerateDevices(const std::set<VendorProductId>& ids) override;

    UnixUsb();
    virtual ~UnixUsb();
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

    /// @brief Claim the specified interface of the device.
    /// @param interface_number The number of the interface to claim.
    /// @return The status of the operation.
    virtual OpStatus ClaimInterface(int32_t interface_number);

  private:
    libusb_device_handle* dev_handle; //a device handle

    static void process_libusbtransfer(libusb_transfer* trans);

    /// @brief Data context for tracking asynchronous transfers.
    class AsyncContext
    {
      public:
        AsyncContext();
        ~AsyncContext();
        void Reset();

        libusb_transfer* transfer;
        std::size_t bytesXfered;
        std::atomic<bool> done;
        std::mutex transferLock;
        std::condition_variable cv;
    };
};

} // namespace lime
