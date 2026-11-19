#include "CustomParameterTrimDAC.h"

#include "limesuiteng/Logger.h"
#include "limesuiteng/SDRDevice.h"

#include <array>
#include <cmath>
#include <vector>

using namespace std::literals::string_literals;

namespace lime {

CustomParameterTrimDAC::CustomParameterTrimDAC(SDRDevice* device,
    const std::string& name,
    const CustomParameter& parameter,
    std::shared_ptr<DataStorage> storage,
    const std::string& regionName)
    : mDevice(device)
    , mName(name)
    , mParameter(parameter)
{
    if (!storage)
        return;
    const auto region = storage->regions.find(regionName);
    if (region == storage->regions.end() || region->second.size <= 0 || region->second.size > 4)
    {
        lime::warning("%s trim DAC: no usable '%s' storage region, the value can not be stored", name.c_str(), regionName.c_str());
        return;
    }
    mStorage = storage;
    mRegion = region->second;
}

const std::string& CustomParameterTrimDAC::GetName() const
{
    return mName;
}

Range<uint32_t> CustomParameterTrimDAC::GetRange() const
{
    return Range<uint32_t>(mParameter.minValue, mParameter.maxValue);
}

OpStatus CustomParameterTrimDAC::GetValue(uint32_t& value)
{
    std::vector<CustomParameterIO> io{ { mParameter.id, 0, ""s } };
    const OpStatus status = mDevice->CustomParameterRead(io);
    if (status != OpStatus::Success)
        return status;
    value = static_cast<uint32_t>(std::lround(io[0].value));
    return OpStatus::Success;
}

OpStatus CustomParameterTrimDAC::SetValue(uint32_t value)
{
    const Range<uint32_t> range = GetRange();
    if (value < range.min || value > range.max)
        return OpStatus::OutOfRange;
    const std::vector<CustomParameterIO> io{ { mParameter.id, static_cast<double>(value), ""s } };
    return mDevice->CustomParameterWrite(io);
}

OpStatus CustomParameterTrimDAC::GetNonVolatileValue(uint32_t& value)
{
    if (!mStorage)
        return OpStatus::NotSupported;
    std::array<uint8_t, 4> data{};
    const OpStatus status = mDevice->MemoryRead(mStorage, mRegion, data.data());
    if (status != OpStatus::Success)
        return status;
    value = 0;
    for (int i = mRegion.size - 1; i >= 0; --i)
        value = (value << 8) | data[i];
    return OpStatus::Success;
}

OpStatus CustomParameterTrimDAC::SetNonVolatileValue(uint32_t value)
{
    if (!mStorage)
        return OpStatus::NotSupported;
    const Range<uint32_t> range = GetRange();
    if (value < range.min || value > range.max)
        return OpStatus::OutOfRange;
    if (mRegion.size < 4 && value >= (uint32_t{ 1 } << (8 * mRegion.size)))
        return OpStatus::OutOfRange;

    std::array<uint8_t, 4> data{};
    for (int i = 0; i < mRegion.size; ++i)
        data[i] = static_cast<uint8_t>(value >> (8 * i));
    OpStatus status = mDevice->MemoryWrite(mStorage, mRegion, data.data());
    if (status != OpStatus::Success)
        return status;

    uint32_t readback = 0;
    status = GetNonVolatileValue(readback);
    if (status != OpStatus::Success)
        return status;
    if (readback != value)
        return ReportError(OpStatus::Error, "%s trim DAC: stored %u, but reads back %u", mName.c_str(), value, readback);
    return OpStatus::Success;
}

} // namespace lime
