#pragma once

#include "comms/USB/IUSB.h"

#ifdef __unix__
    #include "comms/USB/USBGeneric.h"
#else
    #include "FTD3XXLibrary/FTD3XX.h"
#endif

namespace lime {

/// @brief A class for communicating with devices using the FTDI FT601 USB controller.
class FT601 : public IUSB
{
  public:
    std::vector<USBDescriptor> enumerateDevices(const std::set<VendorProductId>& ids) override;

    FT601();
    virtual ~FT601();

    bool Connect(uint16_t vid, uint16_t pid, const char* serial) override;
    void Disconnect() override;
    bool IsConnected() override;

    void* AllocateAsyncContext() override;
    OpStatus BeginDataXfer(void* context, uint8_t* buffer, size_t length, uint8_t endPointAddr) override;
    OpStatus WaitForXfer(void* context, int32_t timeout_ms) override;
    size_t FinishDataXfer(void* context) override;
    OpStatus AbortXfer(void* context) override;
    void FreeAsyncContext(void* context) override;

    int32_t ControlTransfer(
        int requestType, int request, int value, int index, uint8_t* data, size_t length, int32_t timeout_ms) override;

    int32_t BulkTransfer(uint8_t endPoint, uint8_t* data, size_t length, int32_t timeout_ms) override;

    /**
      @brief Resets the stream buffers of the device.
      @return Status of the operation.
     */
    OpStatus ResetStreamBuffers();

    void AddOnHotplugDisconnectCallback(const HotplugDisconnectCallbackType& function, void* userData) override;

  protected:
#ifdef __unix__
    USBGeneric libusb_impl;
    int FT_SetStreamPipe(unsigned char ep, size_t size);
    int FT_FlushPipe(unsigned char ep);
    uint32_t mUsbCounter;
#else
    FT_HANDLE mFTHandle;
    int ReinitPipe(unsigned char ep);
#endif
};

} // namespace lime
