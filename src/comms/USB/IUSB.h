#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <set>
#include "limesuiteng/OpStatus.h"

namespace lime {

/// @brief The description of a given USB device.
struct USBDescriptor {
    std::string product; ///< The Product Name of the device.
    std::string serial; ///< The serial number of the device.
    uint16_t vid; ///< The Vendor ID of the device.
    uint16_t pid; ///< The Product ID of the device.
};

/// @brief A generic class to communicate with a USB devices.
class IUSB
{
  public:
    /// @brief Structure holding a vendor/product ID combo.
    struct VendorProductId {
        uint16_t vendorId; ///< The Vendor ID of the device.
        uint16_t productId; ///< The Product ID of the device.

        /// @brief The comparison operator (needed for use in std::set)
        /// @param rhs The other VID/PID pair to compare to.
        /// @return True if this element is "smaller" than the other one.
        bool operator<(const VendorProductId& rhs) const { return vendorId < rhs.vendorId || productId < rhs.productId; }
    };
    /**
     * @brief Returns list of detected devices descriptors used for connecting to device.
     * @param ids Set of vendor and product IDs to search for
     * @return A list of found USB descriptors
    */
    virtual std::vector<USBDescriptor> enumerateDevices(const std::set<VendorProductId>& ids) = 0;
    virtual ~IUSB(){};

    /**
      @brief Connects to a given USB device.
      @param vid The vendor ID of the device.
      @param pid The prduct ID of the device.
      @param serial The serial number of the device.
      @return The status of the operation (true on success).
     */
    virtual bool Connect(uint16_t vid, uint16_t pid, const char* serial = nullptr) = 0;

    /**
      @brief Returns whether this instance is connected to a device.
      @return The state of the connection (true = connected).
     */
    virtual bool IsConnected() = 0;

    /** @brief Disconnects from the USB device. */
    virtual void Disconnect() = 0;

    /**
      @brief Transfers data through USB bulk endpoint.
      @param endPointAddress The address to use for the transfer.
      @param data The pointer to the data buffer.
      @param length The length of data being transferred.
      @param timeout_ms The amount of time to wait (in ms) until the transfer is considered failed.
      @return Actual number of bytes transferred.
     */
    virtual int32_t BulkTransfer(uint8_t endPointAddress, uint8_t* data, size_t length, int32_t timeout_ms) = 0;

    /**
      @brief Transfers data through USB control endpoint.
      @param requestType The request type to use in the setup packet.
      @param request The type of request being made.
      @param value The value of the request being made.
      @param index The index of the request being made.
      @param data The pointer to the data buffer.
      @param length The length of data being transferred.
      @param timeout_ms The amount of time to wait (in ms) until the transfer is considered failed.
      @return Actual number of bytes transferred.
     */
    virtual int32_t ControlTransfer(
        int requestType, int request, int value, int index, uint8_t* data, size_t length, int32_t timeout_ms) = 0;

    /**
     * @brief Allocate context for use in asynchronous data transfers.
     * @return Pointer to context.
    */
    virtual void* AllocateAsyncContext() = 0;

    /**
      @brief Begins an asynchronous data transfer through bulk endpoint.
      @param context Pointer to transfer context returned by AllocateAsyncContext
      @param buffer The pointer to the data buffer.
      @param length The length of the data being transferred.
      @param endPointAddr The endpoint address to use for the transfer.
      @return Operation success status.
     */
    virtual OpStatus BeginDataXfer(void* context, uint8_t* buffer, size_t length, uint8_t endPointAddr) = 0;

    /**
      @brief Waits until an asynchronous data transfer finishes.
      @param context The context handle to wait for (used in BeginDataXfer).
      @param timeout_ms The timeout (in ms) to wait for the transfer.
      @return Operation status, OpStatus::Success if transfer completed, or OpStatus::Timeout if not completed withing given time.
     */
    virtual OpStatus WaitForXfer(void* context, int32_t timeout_ms) = 0;

    /**
      @brief Finishes an asynchronous data transfer.
      @param context Pointer to transfer context (used in BeginDataXfer).
      @return The amount of bytes transferred in the transfer.
     */
    virtual size_t FinishDataXfer(void* context) = 0;

    /**
      @brief Aborts asynchronous transfer for given context.
      @param context Pointer to transfer context (used in BeginDataXfer).
     */
    virtual OpStatus AbortXfer(void* context) = 0;

    /**
      @brief Deallocate asynchronous transfer context.
      @param context Pointer to transfer context (retuned by AllocateAsyncContext).
     */
    virtual void FreeAsyncContext(void* context) = 0;
};

} // namespace lime
