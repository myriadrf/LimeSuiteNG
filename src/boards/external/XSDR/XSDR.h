#ifndef LIME_EXTERNAL_XSDR_H
#define LIME_EXTERNAL_XSDR_H

#include "boards/LimeSDR_XTRX/LimeSDR_XTRX.h"

namespace lime {

static const float XSDR_DEFAULT_REFERENCE_CLOCK = 26e6;

/// @brief Non LimeSDR board, but with compatible gateware similar to XTRX
class XSDR : public LimeSDR_XTRX
{
  public:
    XSDR() = delete;
    XSDR(std::shared_ptr<IComms> spiLMS7002M,
        std::shared_ptr<IComms> spiFPGA,
        std::shared_ptr<LimePCIe> sampleStream,
        std::shared_ptr<ISerialPort> control,
        double refClk = XSDR_DEFAULT_REFERENCE_CLOCK);
    virtual ~XSDR();
};

} // namespace lime

#endif // LIME_EXTERNAL_XSDR_H
