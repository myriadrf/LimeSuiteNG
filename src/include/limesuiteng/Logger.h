/**
@file limesuiteng/Logger.h
@author Lime Microsystems
@brief API for logging library status messages.
*/

#ifndef LIMESUITE_LOGGER_H
#define LIMESUITE_LOGGER_H

#include "limesuiteng/config.h"
#include "limesuiteng/OpStatus.h"
#include <cstdint>
#include <string>
#include <functional>

namespace lime {

enum class LogLevel : std::uint8_t {
    Critical, //!< A critical error. The application might not be able to continue running successfully.
    Error, //!< An error. An operation did not complete successfully, but the application as a whole is not affected.
    Warning, //!< A warning. An operation completed with an unexpected result.
    Info, //!< An informational message, usually denoting the successful completion of an operation.
    Verbose, //!< An informational message, detailing intermediate step results.
    Debug, //!< A debugging message, only shown in Debug configuration.
};

//! Typedef for C string log handler function pointer type.
typedef void (*LogHandlerCString)(const LogLevel level, const char* message);

//! Typedef for C++ string log handler function pointer type.
typedef std::function<void(const LogLevel level, const std::string& message)> LogHandler;

/*!
 * Register a new system log handler that supports C style string format.
 * Platforms should call this to replace the default stdio handler.
 */
LIME_API void registerLogHandler(const LogHandlerCString handler);

/*!
 * Register a new system log handler that supports C++ style string format.
 * Platforms should call this to replace the default stdio handler.
 */
LIME_API void registerLogHandler(const LogHandler handler);

/// @brief Gets the error code to string + any optional message reported.
/// @return Returns a C style string.
LIME_API const char* GetLastErrorMessageCString(void);

/// @brief Gets the error code to string + any optional message reported.
/// @return Returns a C++ style string.
LIME_API const std::string& GetLastErrorMessage(void);

// C-string versions
LIME_API void critical [[gnu::format(printf, 1, 2)]] (const char* format, ...); //!< Log a formatted critical error message to log handler.
LIME_API void critical(const std::string& text); //!< Log a critical error message to log handler.

LIME_API int error [[gnu::format(printf, 1, 2)]] (const char* format, ...); //!< Log a formatted error message to log handler.
LIME_API int error(const std::string& text); //!< Log an error message to log handler.

LIME_API void warning [[gnu::format(printf, 1, 2)]] (const char* format, ...); //!< Log a formatted warning message to log handler.
LIME_API void warning(const std::string& text); //!< Log a warning message to log handler.

LIME_API void info [[gnu::format(printf, 1, 2)]] (const char* format, ...); //!< Log a formatted information message to log handler.
LIME_API void info(const std::string& text); //!< Log an information message to log handler.

LIME_API void debug [[gnu::format(printf, 1, 2)]] (const char* format, ...); //!< Log a formatted debug message to log handler.
LIME_API void debug(const std::string& text); //!< Log a debug message to log handler.

//! Log a C-style format message with specified logging level to log handler.
LIME_API void log [[gnu::format(printf, 2, 3)]] (const LogLevel level, const char* format, ...);
//! Log a string type message with specified logging level to log handler.
LIME_API void log(const LogLevel level, const std::string& text);

/*!
 * Report a typical errno style error to log handler.
 * The resulting error message comes from strerror().
 * \param errnum a recognized error code.
 * \return passthrough errnum.
 */
LIME_API OpStatus ReportError(const OpStatus errnum);

/*!
 * Reports operation status code as error and a formatted message string to log handler.
 * \param errnum a recognized error code.
 * \param format a format string followed by args.
 * \return passthrough errnum.
 */
LIME_API OpStatus ReportError [[gnu::format(printf, 2, 3)]] (const OpStatus errnum, const char* format, ...);

/**
 * @brief Reports operation status code and error message to log handler.
 * @param errnum Operation status enumeration code.
 * @param text Error message.
 * @return Operation status error code.
 */
LIME_API OpStatus ReportError(const OpStatus errnum, const std::string& text);

/**
 * @brief Reports error code as an integer code and a formated message string to log handler.
 * @param errnum Any error code as an integer.
 * @param format Error message format.
 * @return Reported error code. 
 */
LIME_API int ReportError [[gnu::format(printf, 2, 3)]] (const int errnum, const char* format, ...);

/**
 * @brief Reports error code as an integer code and error message to log handler.
 * @param errnum Any error code as an integer.
 * @param text Error message.
 * @return Reported error code.
 */
LIME_API int ReportError(const int errnum, const std::string& text);

} // namespace lime

#endif //LIMESUITE_LOGGER_H
