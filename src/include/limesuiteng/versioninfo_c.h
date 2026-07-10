/**
 * @file limesuiteng/versioninfo_c.h
 * @author Lime Microsystems
 * @brief Library version information.
 */
#ifndef LIMESUITENG_VERSIONINFO_C_H
#define LIMESUITENG_VERSIONINFO_C_H

#include "limesuiteng/types_c.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Gets the library version as a dotted string.
 * @return Static string in major.minor.patch-extra format.
 */
LIME_C_API const char* lime_get_library_version(void);

/**
 * @brief Gets the API version as a string.
 * @return Static string in major.minor.increment format.
 */
LIME_C_API const char* lime_get_api_version(void);

/**
 * @brief Gets the ABI/so version of the library.
 * @return Static version string.
 */
LIME_C_API const char* lime_get_abi_version(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* LIMESUITENG_VERSIONINFO_C_H */
