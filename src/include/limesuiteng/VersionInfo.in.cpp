/**
@file VersionInfo.in.cpp
@author Lime Microsystems
@brief API for querying version and build information.
*/

#include "limesuiteng/VersionInfo.h"
#include <string>
#include <sstream>

using namespace std::literals::string_literals;

std::string lime::GetLibraryVersion(void)
{
    return "@LIME_SUITE_VERSION@"s;
}

std::string lime::GetBuildTimestamp(void)
{
    return "@BUILD_TIMESTAMP@"s;
}

std::string lime::GetAPIVersion(void)
{
    const std::string verStr{ std::to_string(LIMESUITENG_API_VERSION) };
    std::stringstream ss;
    ss << std::stoi(verStr.substr(2, 4)) << "." << std::stoi(verStr.substr(6, 2)) << "." << std::stoi(verStr.substr(8, 2));
    return ss.str();
}

std::string lime::GetABIVersion(void)
{
    return "@LIME_SUITE_SOVER@"s;
}
