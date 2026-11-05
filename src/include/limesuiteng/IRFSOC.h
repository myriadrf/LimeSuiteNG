#ifndef LIMESUITENG_IRFSOC_H
#define LIMESUITENG_IRFSOC_H

#include "limesuiteng/types.h"
#include "limesuiteng/OpStatus.h"

#include <cstdint>
#include <string>

namespace lime {

class IRFChannel;
class ISensor;

/// @brief Generic RF SoC (transceiver chip), made up of Rx and Tx channels.
class IRFSOC
{
  public:
    virtual ~IRFSOC() = default;

    /// @return The number of channels in the given direction.
    virtual uint8_t GetChannelCount(TRXDir dir) const = 0;

    /// @return The channel, owned by the RF SoC and valid for as long as it exists, or nullptr if the index is out of range.
    virtual IRFChannel* GetChannel(TRXDir dir, uint8_t index) = 0;

    /// @return The number of sensors of the chip.
    virtual uint8_t GetSensorCount() const = 0;
    /// @return The sensor, owned by the RF SoC and valid for as long as it exists, or nullptr if the index is out of range.
    virtual ISensor* GetSensor(uint8_t index) = 0;

    /// @brief Reads a configuration file and uploads its register values to the chip.
    /// @param filename The configuration source file.
    /// @param tuneDynamicValues Whether to retune values that depend on the hardware state (e.g. VCOs) after loading.
    /// @return The status of the operation.
    virtual OpStatus LoadConfig(const std::string& filename, bool tuneDynamicValues = true) = 0;
};

} // namespace lime

#endif
