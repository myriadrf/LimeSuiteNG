#ifndef LIMESUITENG_IRFSOC_H
#define LIMESUITENG_IRFSOC_H

#include "limesuiteng/types.h"

#include <cstdint>

namespace lime {

class IRFChannel;

/// @brief Generic RF SoC (transceiver chip), made up of Rx and Tx channels.
class IRFSOC
{
  public:
    virtual ~IRFSOC() = default;

    /// @return The number of channels in the given direction.
    virtual uint8_t GetChannelCount(TRXDir dir) const = 0;

    /// @return The channel, owned by the RF SoC and valid for as long as it exists, or nullptr if the index is out of range.
    virtual IRFChannel* GetChannel(TRXDir dir, uint8_t index) = 0;
};

} // namespace lime

#endif
