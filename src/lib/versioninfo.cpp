/* C wrapper over lime::VersionInfo. */
#include "limesuiteng/versioninfo.h"

#include "limesuiteng/VersionInfo.hpp"

#include <string>

extern "C" {

const char* lime_get_library_version(void)
{
    static const std::string v = lime::GetLibraryVersion();
    return v.c_str();
}

const char* lime_get_api_version(void)
{
    static const std::string v = lime::GetAPIVersion();
    return v.c_str();
}

const char* lime_get_abi_version(void)
{
    static const std::string v = lime::GetABIVersion();
    return v.c_str();
}

} /* extern "C" */
