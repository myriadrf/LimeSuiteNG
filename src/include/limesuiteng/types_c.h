/**
 * @file limesuiteng/types_c.h
 * @author Lime Microsystems
 * @brief Common types of the LimeSuiteNG C API: status codes, direction, sample formats.
 */
#ifndef LIMESUITENG_TYPES_C_H
#define LIMESUITENG_TYPES_C_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#if defined(_WIN32)
    #if defined(LIME_C_BUILD)
        #define LIME_C_API __declspec(dllexport)
    #else
        #define LIME_C_API __declspec(dllimport)
    #endif
#elif defined(__GNUC__)
    #define LIME_C_API __attribute__((visibility("default")))
#else
    #define LIME_C_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** @brief The possible status codes of operations. 0 is success, negative is failure. */
typedef enum {
    lime_OpStatus_Success = 0, ///< Success code: 0
    lime_OpStatus_Error = -1, ///< Error code: -1
    lime_OpStatus_NotImplemented = -2, ///< Not implemented code: -2
    lime_OpStatus_IOFailure = -3, ///< IO failure code: -3
    lime_OpStatus_InvalidValue = -4, ///< Invalid value code: -4
    lime_OpStatus_FileNotFound = -5, ///< File not found code: -5
    lime_OpStatus_OutOfRange = -6, ///< Out of range code: -6
    lime_OpStatus_NotSupported = -7, ///< Not supported code: -7
    lime_OpStatus_Timeout = -8, ///< Timeout code: -8
    lime_OpStatus_Busy = -9, ///< Busy code: -9
    lime_OpStatus_Aborted = -10, ///< Abort code: -10
    lime_OpStatus_PermissionDenied = -11, ///< Permission denied code: -11
    lime_OpStatus_NotConnected = -12 ///< Not connected code: -12
} lime_OpStatus;

/** @brief Direction of the RF transmission. */
typedef enum {
    lime_TRXDir_Rx = 0, ///< Receive
    lime_TRXDir_Tx = 1 ///< Transmit
} lime_TRXDir;

/** @brief Sample data layouts. */
typedef enum {
    lime_DataFormat_I16 = 0, ///< 16-bit integers
    lime_DataFormat_I12 = 1, ///< 12-bit integers stored as int16_t, range [-2048; 2047]
    lime_DataFormat_F32 = 2 ///< 32-bit floating-point
} lime_DataFormat;

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* LIMESUITENG_TYPES_C_H */
