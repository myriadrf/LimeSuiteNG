/**
 * @file limesuiteng/config_c.h
 * @author Lime Microsystems
 * @brief Import/export visibility macro of the LimeSuiteNG C API.
 */
#ifndef LIMESUITENG_CONFIG_C_H
#define LIMESUITENG_CONFIG_C_H

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

#endif /* LIMESUITENG_CONFIG_C_H */
