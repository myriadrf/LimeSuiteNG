#ifndef LIME_CUSTOMPARAMETERTRIMDAC_H
#define LIME_CUSTOMPARAMETERTRIMDAC_H

#include "limesuiteng/ITrimDAC.h"
#include "limesuiteng/SDRDescriptor.h"

#include <memory>
#include <string>

namespace lime {

class SDRDevice;

/// @brief Trim DAC of the boards that control it with a custom parameter (the volatile value) and
/// keep its power up value in a memory region (the non-volatile value), both through the SDRDevice API.
class CustomParameterTrimDAC : public ITrimDAC
{
  public:
    /// @param device The device that owns the parameter and the memory.
    /// @param name The name of the oscillator the DAC tunes.
    /// @param parameter The custom parameter that sets the DAC.
    /// @param storage The memory that holds the non-volatile value, nullptr if the device does not store it.
    /// @param regionName The name of the storage region that holds the value (stored little endian).
    CustomParameterTrimDAC(SDRDevice* device,
        const std::string& name,
        const CustomParameter& parameter,
        std::shared_ptr<DataStorage> storage = nullptr,
        const std::string& regionName = "");

    const std::string& GetName() const override;
    Range<uint32_t> GetRange() const override;
    OpStatus GetValue(uint32_t& value) override;
    OpStatus SetValue(uint32_t value) override;
    OpStatus GetNonVolatileValue(uint32_t& value) override;
    OpStatus SetNonVolatileValue(uint32_t value) override;

  private:
    SDRDevice* mDevice;
    std::string mName;
    CustomParameter mParameter;
    std::shared_ptr<DataStorage> mStorage;
    Region mRegion{ 0, 0 };
};

} // namespace lime

#endif
