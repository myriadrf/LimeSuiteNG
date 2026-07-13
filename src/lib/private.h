/* Internal helpers shared by the C API wrapper files. Not installed. */
#ifndef LIMESUITENG_CAPI_LIB_PRIVATE_H
#define LIMESUITENG_CAPI_LIB_PRIVATE_H

#include "limesuiteng/types.h"
#include "limesuiteng/sdrdevice.h"
#include "limesuiteng/rfstream.h"

#include "limesuiteng/RFStream.hpp"
#include "limesuiteng/SDRDescriptor.hpp"
#include "limesuiteng/SDRDevice.hpp"
#include "limesuiteng/types.hpp"

#include <limits>
#include <memory>

/* Transitional: the generic device tree collapses onto SDRDevice, so the
 * device, SDR, GPIO, and SPI handles are all the same underlying pointer. */
static inline lime::SDRDevice* sdr(lime_SDRDevice* d)
{
    return reinterpret_cast<lime::SDRDevice*>(d);
}

static inline lime::TRXDir dir(lime_TRXDir d)
{
    return static_cast<lime::TRXDir>(d != lime_TRXDir_Rx);
}

static inline const lime::SDRDescriptor* desc(const lime_SDRDescriptor* d)
{
    return reinterpret_cast<const lime::SDRDescriptor*>(d);
}

/* The public C surface takes 32-bit indices so the ABI never has to widen;
 * the internal C++ addressing is currently 8-bit, so out-of-range values are
 * rejected rather than silently truncated. */
static inline bool narrows(uint32_t module, uint32_t channel)
{
    return module > std::numeric_limits<uint8_t>::max() || channel > std::numeric_limits<uint8_t>::max();
}

/* The stream handle owns the RFStream and remembers the application sample
 * format so recv/send can dispatch the matching typed overload. */
struct lime_Stream {
    std::unique_ptr<lime::RFStream> impl;
    lime::DataFormat format;
};

#endif /* LIMESUITENG_CAPI_LIB_PRIVATE_H */
