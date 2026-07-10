/* C wrapper over lime::SDRDescriptor: read-only capability accessors. */
#include "limesuiteng/sdrdescriptor_c.h"
#include "capi_private.h"

extern "C" {

const char* lime_descriptor_name(const lime_SDRDescriptor* d)
{
    return d != nullptr ? desc(d)->name.c_str() : nullptr;
}

uint64_t lime_descriptor_serial(const lime_SDRDescriptor* d)
{
    return d != nullptr ? desc(d)->serialNumber : 0;
}

size_t lime_descriptor_rfsoc_count(const lime_SDRDescriptor* d)
{
    return d != nullptr ? desc(d)->rfSOC.size() : 0;
}

const char* lime_descriptor_rfsoc_name(const lime_SDRDescriptor* d, size_t soc)
{
    if (d == nullptr || soc >= desc(d)->rfSOC.size())
        return nullptr;
    return desc(d)->rfSOC[soc].name.c_str();
}

uint32_t lime_descriptor_channel_count(const lime_SDRDescriptor* d, size_t soc)
{
    if (d == nullptr || soc >= desc(d)->rfSOC.size())
        return 0;
    return desc(d)->rfSOC[soc].channelCount;
}

size_t lime_descriptor_antenna_count(const lime_SDRDescriptor* d, size_t soc, lime_TRXDir dr)
{
    if (d == nullptr || soc >= desc(d)->rfSOC.size())
        return 0;
    const auto& paths = desc(d)->rfSOC[soc].pathNames;
    const auto it = paths.find(dir(dr));
    return it != paths.end() ? it->second.size() : 0;
}

const char* lime_descriptor_antenna_name(const lime_SDRDescriptor* d, size_t soc, lime_TRXDir dr, size_t index)
{
    if (d == nullptr || soc >= desc(d)->rfSOC.size())
        return nullptr;
    const auto& paths = desc(d)->rfSOC[soc].pathNames;
    const auto it = paths.find(dir(dr));
    if (it == paths.end() || index >= it->second.size())
        return nullptr;
    return it->second[index].c_str();
}

} /* extern "C" */
