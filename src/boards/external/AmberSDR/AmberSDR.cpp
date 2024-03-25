#include "AmberSDR.h"

#include "LMSBoards.h"

namespace lime {

/// @brief Constructs a new AmberSDR object
///
/// @param spiRFsoc The communications port to the LMS7002M chip.
/// @param spiFPGA The communications port to the device's FPGA.
/// @param sampleStream The communications port to send and receive sample data.
/// @param control The serial port communication of the device.
/// @param refClk The reference clock of the device.
AmberSDR::AmberSDR(std::shared_ptr<IComms> spiRFsoc,
    std::shared_ptr<IComms> spiFPGA,
    std::shared_ptr<LitePCIe> sampleStream,
    std::shared_ptr<ISerialPort> control,
    double refClk)
    : LimeSDR_XTRX(spiRFsoc, spiFPGA, sampleStream, control, refClk)
{
    SDRDevice::Descriptor& desc = mDeviceDescriptor;
    desc.name = GetDeviceName(LMS_DEV_EXTERNAL_AMBERSDR);
}

AmberSDR::~AmberSDR()
{
}

} //namespace lime
