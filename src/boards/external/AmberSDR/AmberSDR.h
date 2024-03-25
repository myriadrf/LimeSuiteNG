#ifndef LIME_EXTERNAL_AMBERSDR_H
#define LIME_EXTERNAL_AMBERSDR_H

#include "boards/LimeSDR_XTRX/LimeSDR_XTRX.h"

namespace lime {

static const float AMBERSDR_DEFAULT_REFERENCE_CLOCK = 26e6;

// Non LimeSDR board, but with compatible gateware similar to XTRX

class AmberSDR : public LimeSDR_XTRX
{
  public:
    AmberSDR() = delete;
    AmberSDR(std::shared_ptr<IComms> spiLMS7002M,
        std::shared_ptr<IComms> spiFPGA,
        std::shared_ptr<LitePCIe> sampleStream,
        std::shared_ptr<ISerialPort> control,
        double refClk = AMBERSDR_DEFAULT_REFERENCE_CLOCK);
    virtual ~AmberSDR();
};

} // namespace lime

#endif // LIME_EXTERNAL_AMBERSDR_H
