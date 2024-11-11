#ifndef LIME_OPSTATUS_H
#define LIME_OPSTATUS_H

#ifdef Success
    #error unix X11 header has preprocessor define of Success. It might be included through graphics headers.
#endif

namespace lime {

/// @brief The possible status codes from operations.
enum class OpStatus {
    Success = 0,
    Error = -1,
    NotImplemented = -2,
    IOFailure = -3,
    InvalidValue = -4,
    FileNotFound = -5,
    OutOfRange = -6,
    NotSupported = -7,
    Timeout = -8,
    Busy = -9,
    Aborted = -10,
    PermissionDenied = -11,
    NotConnected = -12,
};

} // namespace lime

#endif
