#pragma once
#include "comms/USB/IUSB.h"

#include <vector>
#include <map>
#include <memory>

#ifdef __unix__
    #include "comms/USB/USBGeneric.h"
#endif // !__unix__

class CCyFX3Device;
class CCyUSBEndPoint;

namespace lime {

/// @brief A class for communicating with devices using the Cypress USB 3.0 CYUSB3014-BZXC USB controller.
class FX3 : public IUSB
{
  public:
    std::vector<USBDescriptor> enumerateDevices(const std::set<VendorProductId>& ids) override;

    FX3();
    virtual ~FX3();

    bool Connect(uint16_t vid, uint16_t pid, const std::string& serial) override;
    void Disconnect() override;
    bool IsConnected() override;

    void* AllocateAsyncContext() override;
    OpStatus BeginDataXfer(void* context, uint8_t* buffer, size_t length, uint8_t endPointAddr) override;
    OpStatus WaitForXfer(void* context, int32_t timeout_ms) override;
    size_t FinishDataXfer(void* context) override;
    OpStatus AbortXfer(void* context) override;
    void FreeAsyncContext(void* context) override;

    int32_t BulkTransfer(uint8_t endPoint, uint8_t* data, size_t length, int32_t timeout_ms) override;
    int32_t ControlTransfer(
        int requestType, int request, int value, int index, uint8_t* data, size_t length, int32_t timeout_ms) override;

    void AddOnHotplugDisconnectCallback(const HotplugDisconnectCallbackType& function, void* userData) override;

    static const int CTR_WRITE_REQUEST_VALUE;
    static const int CTR_READ_REQUEST_VALUE;

    static constexpr uint8_t CONTROL_BULK_OUT_ADDRESS =
        0x0F; ///< The memory address for writing information via the bulk transfer protocol.
    static constexpr uint8_t CONTROL_BULK_IN_ADDRESS =
        0x8F; ///< THe memory address for reading information via the bulk transfer protocol.
  protected:
#ifdef __unix__
    USBGeneric libusb_impl;
#else
    // using pointer so that windows.h header would only be needed inside cpp file
    std::shared_ptr<CCyFX3Device> fx3device;

    //end points for samples reading and writing
    std::map<uint8_t, CCyUSBEndPoint*> endpoints{};
#endif
};

} // namespace lime
