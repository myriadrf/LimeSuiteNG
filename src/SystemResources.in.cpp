/**
@file SystemResources.h
@author Lime Microsystems
@brief APIs for locating system resources.
*/

#include "SystemResources.h"
#include "limesuiteng/Logger.h"

#include <cstdlib> //getenv, system
#include <filesystem>
#include <vector>
#include <string_view>
#include <sstream>

#ifdef _MSC_VER
    #include <windows.h>
    #include <shlobj.h>
    #include <io.h>

    //access mode constants
    #define F_OK 0
    #define R_OK 2
    #define W_OK 4
#endif

#ifdef __unix__
    #include <pwd.h>
    #include <unistd.h>
#endif

using namespace std::literals::string_literals;
using namespace std::literals::string_view_literals;

namespace lime {

std::string getLimeSuiteRoot(void)
{
    //first check the environment variable
    const char* limeSuiteRoot = std::getenv("LIME_SUITE_ROOT");
    if (limeSuiteRoot != nullptr)
        return limeSuiteRoot;

// Get the path to the current dynamic linked library.
// The path to this library can be used to determine
// the installation root without prior knowledge.
#if defined(_MSC_VER) && defined(LIME_DLL)
    char path[MAX_PATH];
    HMODULE hm = NULL;
    if (GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
            (LPCSTR)&lime::getLimeSuiteRoot,
            &hm))
    {
        const DWORD size = GetModuleFileNameA(hm, path, sizeof(path));
        if (size != 0)
        {
            const std::string libPath(path, size);
            const size_t slash0Pos = libPath.find_last_of("/\\"sv);
            const size_t slash1Pos = libPath.substr(0, slash0Pos).find_last_of("/\\"sv);
            if (slash0Pos != std::string::npos && slash1Pos != std::string::npos)
                return libPath.substr(0, slash1Pos);
        }
    }
#endif //_MSC_VER && LIME_DLL

    return "@LIME_SUITE_ROOT@"s;
}

std::string getHomeDirectory(void)
{
    //first check the HOME environment variable
    const char* userHome = std::getenv("HOME");
    if (userHome != nullptr)
        return userHome;

//use unix user id lookup to get the home directory
#ifdef __unix__
    const char* pwDir = getpwuid(getuid())->pw_dir;
    if (pwDir != nullptr)
        return pwDir;
#endif

    return ""s;
}

/*!
 * The generic location for data storage with user permission level.
 */
static std::string getBareAppDataDirectory(void)
{
    //always check APPDATA (usually windows, but can be set for linux)
    const char* appDataDir = std::getenv("APPDATA");
    if (appDataDir != nullptr)
        return appDataDir;

//use windows API to query for roaming app data directory
#ifdef _MSC_VER
    char csidlAppDataDir[MAX_PATH];
    if (SUCCEEDED(SHGetFolderPathA(NULL, CSIDL_APPDATA, NULL, 0, csidlAppDataDir)))
    {
        return csidlAppDataDir;
    }
#endif

//xdg freedesktop standard location environment variable
#ifdef __unix__
    const char* xdgDataHome = std::getenv("XDG_DATA_HOME");
    if (xdgDataHome != nullptr)
        return xdgDataHome;
#endif

    //xdg freedesktop standard location for data in home directory
    return getHomeDirectory() + "/.local/share"s;
}

std::string getAppDataDirectory(void)
{
    return getBareAppDataDirectory() + "/LimeSuite"s;
}

std::string getConfigDirectory(void)
{
    //xdg standard is XDG_CONFIG_HOME or $HOME/.config
    //but historically we have used $HOME/.limesuite
    return getHomeDirectory() + "/.limesuite"s;
}

std::vector<std::string> listImageSearchPaths(void)
{
    std::vector<std::string> imageSearchPaths;

//separator for search paths in the environment variable
#ifdef _MSC_VER
    static const char sep = ';';
#else
    static const char sep = ':';
#endif

    //check the environment's search path
    const char* imagePathEnv = std::getenv("LIME_IMAGE_PATH");
    if (imagePathEnv != nullptr)
    {
        std::stringstream imagePaths(imagePathEnv);
        std::string imagePath;
        while (std::getline(imagePaths, imagePath, sep))
        {
            if (imagePath.empty())
                continue;
            imageSearchPaths.push_back(imagePath);
        }
    }

    //search directories in the user's home directory
    imageSearchPaths.push_back(lime::getAppDataDirectory() + "/images"s);

    //search global installation directories
    imageSearchPaths.push_back(lime::getLimeSuiteRoot() + "/share/LimeSuite/images"s);

    return imageSearchPaths;
}

std::string locateImageResource(const std::string& name)
{
    for (const auto& searchPath : lime::listImageSearchPaths())
    {
        const std::string fullPath(searchPath + "/@VERSION_MAJOR@.@VERSION_MINOR@/"s + name);
        if (access(fullPath.c_str(), R_OK) == 0)
            return fullPath;
    }
    return ""s;
}

/*!
 * Download an image resource given only the file name.
 * The resource will be downloaded in the user's application data directory.
 * @param name a unique name for the resource file including file extension
 * @return 0 for success or error code upon error
 */
lime::OpStatus downloadImageResource(const std::string& name)
{
    const std::string destDir(lime::getAppDataDirectory() + "/images/@VERSION_MAJOR@.@VERSION_MINOR@"s);
    const std::string destFile(destDir + "/"s + name);
    const std::string sourceUrl("https://downloads.myriadrf.org/project/limesuite/@VERSION_MAJOR@.@VERSION_MINOR@/"s + name);

    if (std::filesystem::exists(destDir))
    {
        if (!std::filesystem::is_directory(destDir))
        {
            return lime::ReportError(OpStatus::InvalidValue, "Not a directory: "s + destDir);
        }
    }
    else
    {
        bool result = std::filesystem::create_directories(destDir);
        if (!result)
        {
            return lime::ReportError(OpStatus::Error, "Failed to create directory: "s + destDir);
        }
    }

    //check for write access
    if (access(destDir.c_str(), W_OK) != 0)
        lime::ReportError(OpStatus::PermissionDenied, "Cannot write: "s + destDir);

//download the file
#ifdef __unix__
    const std::string dnloadCmd("wget --output-document=\""s + destFile + "\" \""s + sourceUrl + "\""s);
#else
    const std::string dnloadCmd(
        "powershell.exe -Command \"(new-object System.Net.WebClient).DownloadFile('"s + sourceUrl + "', '"s + destFile + "')\""s);
#endif
    int result = std::system(dnloadCmd.c_str());
    if (result != 0)
        return lime::ReportError(OpStatus::Error, "Failed: "s + dnloadCmd);

    return OpStatus::Success;
}

} // namespace lime
