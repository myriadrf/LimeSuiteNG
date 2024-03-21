#ifndef LIME_LIMESDR_NANO_H
#define LIME_LIMESDR_NANO_H

#include "boards/LimeSDR_XTRX/LimeSDR_XTRX.h"

namespace lime {

static const float NANO_DEFAULT_REFERENCE_CLOCK = 26e6;

/** @brief Class for managing the LimeSDR Nano device. */
class LimeSDR_Nano : public LimeSDR_XTRX
{
  public:
    LimeSDR_Nano() = delete;
    LimeSDR_Nano(std::shared_ptr<IComms> spiLMS7002M,
        std::shared_ptr<IComms> spiFPGA,
        std::shared_ptr<LitePCIe> sampleStream,
        std::shared_ptr<ISerialPort> control,
        double refClk = NANO_DEFAULT_REFERENCE_CLOCK);
    virtual ~LimeSDR_Nano();
};

} // namespace lime

#endif // LIME_LIMESDR_NANO_H
