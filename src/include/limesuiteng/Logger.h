/**
@file limesuiteng/Logger.h
@author Lime Microsystems
@brief API for logging library status messages.
*/

#ifndef LIMESUITE_LOGGER_H
#define LIMESUITE_LOGGER_H

#include "limesuiteng/config.h"
#include "limesuiteng/OpStatus.h"
#include <string>
#include <cstdarg>
#include <cerrno>
#include <stdexcept>
#include <cstdint>

namespace lime {

enum class LogLevel : uint8_t {
    CRITICAL, //!< A critical error. The application might not be able to continue running successfully.
    ERROR, //!< An error. An operation did not complete successfully, but the application as a whole is not affected.
    WARNING, //!< A warning. An operation completed with an unexpected result.
    INFO, //!< An informational message, usually denoting the successful completion of an operation.
    VERBOSE, //!< An informational message, detailing intermediate step results.
    DEBUG, //!< A debugging message, only shown in Debug configuration.
};

//! Typedef for the registered log handler function.
typedef void (*LogHandler)(const LogLevel level, const char* message);

/*!
 * Register a new system log handler.
 * Platforms should call this to replace the default stdio handler.
 */
LIME_API void registerLogHandler(const LogHandler handler);

//! Get the error code to string + any optional message reported.
LIME_API const char* GetLastErrorMessage(void);

// C-string versions
LIME_API void critical(const char* format, ...); //!< Log a critical error message with formatting
LIME_API void critical(const std::string& text); //!< Log a critical error message

LIME_API int error(const char* format, ...); //!< Log an error message with formatting
LIME_API int error(const std::string& text); //!< Log an error message

LIME_API void warning(const char* format, ...); //!< Log a warning message with formatting
LIME_API void warning(const std::string& text); //!< Log a warning message

LIME_API void info(const char* format, ...); //!< Log an information message with formatting
LIME_API void info(const std::string& text); //!< Log an information message

LIME_API void debug(const char* format, ...); //!< Log a debug message with formatting
LIME_API void debug(const std::string& text); //!< Log a debug message

//! Log a message with formatting and specified logging level
LIME_API void log(const LogLevel level, const char* format, ...);
//! Log a message with specified logging level
LIME_API void log(const LogLevel level, const std::string& text);

/*!
 * Report a typical errno style error.
 * The resulting error message comes from strerror().
 * \param errnum a recognized error code
 * \return passthrough errnum
 */
LIME_API lime::OpStatus ReportError(const lime::OpStatus errnum);

/*!
 * Report an error as an integer code and a formatted message string.
 * \param errnum a recognized error code
 * \param format a format string followed by args
 * \return passthrough errnum
 */
LIME_API lime::OpStatus ReportError(const lime::OpStatus errnum, const char* format, ...);

} // namespace lime

#endif //LIMESUITE_LOGGER_H
