/**
@file VersionInfo.h
@author Lime Microsystems
@brief API for querying version and build information.
*/

#ifndef LIMESUITENG_VERSION_INFO_H
#define LIMESUITENG_VERSION_INFO_H

#include "limesuiteng/config.h"
#include <string>

/*!
 * API version number which can be used as a preprocessor check.
 * The format of the version number is encoded as follows:
 * <b>(major << 24) | (minor << 16) | (16 bit increment)</b>.
 * Where the increment can be used to indicate implementation
 * changes, fixes, or API additions within a minor release series.
 *
 * The macro is typically used in an application as follows:
 * \code
 * #if defined(LIMESUITENG_API_VERSION) && (LIMESUITENG_API_VERSION >= 0x01020003)
 * // Use a newer feature from the LimeSuite library API
 * #endif
 * \endcode
 */
#define LIMESUITENG_API_VERSION 0x00020000

namespace lime {
/*!
     * Get the library version as a dotted string.
     * The format is major.minor.patch.build-extra.
     */
LIME_API std::string GetLibraryVersion(void);

/*!
     * Get the date of the build in "%Y-%M-%d" format.
     */
LIME_API std::string GetBuildTimestamp(void);

/*!
     * Get the LimeSuite library API version as a string.
     * The format of the version string is <b>major.minor.increment</b>,
     * where the digits are taken directly from <b>LIMESUITENG_API_VERSION</b>.
     */
LIME_API std::string GetAPIVersion(void);

/*!
     * Get the ABI/so version of the library.
     */
LIME_API std::string GetABIVersion(void);

} // namespace lime

#endif //LIMESUITENG_VERSION_INFO_H
