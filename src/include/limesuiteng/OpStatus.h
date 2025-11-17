#ifndef LIME_OPSTATUS_H
#define LIME_OPSTATUS_H

/**
@file limesuiteng/OpStatus.h
@author Lime Microsystems
@brief File with API operation status code list.
*/

#ifdef Success
    #error unix X11 header has preprocessor define of Success. It might be included through graphics headers.
#endif

namespace lime {

/// @brief The possible status codes from operations.
enum class OpStatus {
    Success = 0,                ///< Success code: 0.
    Error = -1,                 ///< Error code: -1 
    NotImplemented = -2,        ///< Not implemented code: -2
    IOFailure = -3,             ///< IO failure code: -3
    InvalidValue = -4,          ///< Invalid value code: -4
    FileNotFound = -5,          ///< File not found code: -5
    OutOfRange = -6,            ///< Out of range code: -6
    NotSupported = -7,          ///< Not supported code: -7
    Timeout = -8,               ///< Timeout code: -8
    Busy = -9,                  ///< Busy code: -9
    Aborted = -10,              ///< Abort code: -10
    PermissionDenied = -11,     ///< Permission denied code: -11
    NotConnected = -12,         ///< Not connected code: -12
};

} // namespace lime

#endif
