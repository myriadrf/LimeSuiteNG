#include "LimeSDR_Nano.h"

#include "LMSBoards.h"

namespace lime {

/// @brief Constructs a new LimeSDR_Nano object
///
/// @param spiRFsoc The communications port to the LMS7002M chip.
/// @param spiFPGA The communications port to the device's FPGA.
/// @param sampleStream The communications port to send and receive sample data.
/// @param control The serial port communication of the device.
/// @param refClk The reference clock of the device.
LimeSDR_Nano::LimeSDR_Nano(std::shared_ptr<IComms> spiRFsoc,
    std::shared_ptr<IComms> spiFPGA,
    std::shared_ptr<LitePCIe> sampleStream,
    std::shared_ptr<ISerialPort> control,
    double refClk)
    : LimeSDR_XTRX(spiRFsoc, spiFPGA, sampleStream, control, refClk)
{
    /// Do not perform any unnecessary configuring to device in constructor, so you
    /// could read back it's state for debugging purposes.
    SDRDevice::Descriptor& desc = mDeviceDescriptor;
    desc.name = GetDeviceName(LMS_DEV_LIMESDR_NANO);
}

LimeSDR_Nano::~LimeSDR_Nano()
{
}

} //namespace lime
