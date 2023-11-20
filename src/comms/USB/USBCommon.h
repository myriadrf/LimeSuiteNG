#ifndef USBCOMMON_H
#define USBCOMMON_H

#include "limesuite/IComms.h"
#include "LMS64CProtocol.h"
#include "limesuite/DeviceHandle.h"
#include "limesuite/DeviceRegistry.h"

#include <atomic>
#include <condition_variable>
#include <memory>
#include <set>

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

class USBTransferContext
{
  public:
    explicit USBTransferContext();
    virtual ~USBTransferContext();
    virtual bool Reset();

    bool used;

#ifdef __unix__
    libusb_transfer* transfer;
    long bytesXfered;
    std::atomic<bool> done;
    std::mutex transferLock;
    std::condition_variable cv;
#endif
};

struct VidPid {
    uint16_t vid;
    uint16_t pid;

    bool operator<(const VidPid& other) const
    {
        if (vid == other.vid)
        {
            return pid < other.pid;
        }

        return vid < other.vid;
    }
};

class USBEntry : public DeviceRegistryEntry
{
  public:
    USBEntry(const std::string& name, const std::set<VidPid>& deviceIds);
    virtual ~USBEntry();

    virtual std::vector<DeviceHandle> enumerate(const DeviceHandle& hint);

  protected:
#ifdef __unix__
    static libusb_context* ctx;
    static uint ctxRefCount;
#endif
  private:
    std::set<VidPid> mDeviceIds;
#ifdef __unix__
    std::string GetUSBDeviceSpeedString(libusb_device* device);
    DeviceHandle GetDeviceHandle(libusb_device_handle* tempHandle, libusb_device* device, const libusb_device_descriptor& desc);
#endif
};

class USB_CSR_Pipe : public ISerialPort
{
  public:
    explicit USB_CSR_Pipe(){};

    virtual int Write(const uint8_t* data, size_t length, int timeout_ms) override = 0;
    virtual int Read(uint8_t* data, size_t length, int timeout_ms) override = 0;
};

class LMS64C_LMS7002M_Over_USB : public IComms
{
  public:
    LMS64C_LMS7002M_Over_USB(std::shared_ptr<USB_CSR_Pipe> dataPort);

    virtual void SPI(const uint32_t* MOSI, uint32_t* MISO, uint32_t count) override;
    virtual void SPI(uint32_t spiBusAddress, const uint32_t* MOSI, uint32_t* MISO, uint32_t count) override;

    virtual int ResetDevice(int chipSelect) override;

  private:
    std::shared_ptr<USB_CSR_Pipe> pipe;
};

class LMS64C_FPGA_Over_USB : public IComms
{
  public:
    LMS64C_FPGA_Over_USB(std::shared_ptr<USB_CSR_Pipe> dataPort);

    void SPI(const uint32_t* MOSI, uint32_t* MISO, uint32_t count) override;
    void SPI(uint32_t spiBusAddress, const uint32_t* MOSI, uint32_t* MISO, uint32_t count) override;

    virtual int GPIODirRead(uint8_t* buffer, const size_t bufLength) override;
    virtual int GPIORead(uint8_t* buffer, const size_t bufLength) override;
    virtual int GPIODirWrite(const uint8_t* buffer, const size_t bufLength) override;
    virtual int GPIOWrite(const uint8_t* buffer, const size_t bufLength) override;

    virtual int CustomParameterWrite(const std::vector<CustomParameterIO>& parameters) override;
    virtual int CustomParameterRead(std::vector<CustomParameterIO>& parameters) override;

    virtual int ProgramWrite(
        const char* data, size_t length, int prog_mode, int target, ProgressCallback callback = nullptr) override;

  private:
    std::shared_ptr<USB_CSR_Pipe> pipe;
};

} // namespace lime

#endif // USBCOMMON_H