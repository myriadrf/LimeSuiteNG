/**
 * @file limesuiteng/config_c.h
 * @author Lime Microsystems
 * @brief Import/export visibility macro of the LimeSuiteNG C API.
 */
#ifndef LIMESUITENG_CONFIG_C_H
#define LIMESUITENG_CONFIG_C_H

/* LIME_C_API marks the public C entry points. It DLL-imports or DLL-exports
 * when the library is built shared, and does nothing for a static build,
 * driven by the same LIME_DLL / LIME_DLL_EXPORTS definitions as the C++
 * LIME_API (see limesuiteng/config.h). */
#ifdef LIME_DLL
    #if defined(_WIN32) || defined(__CYGWIN__)
        #ifdef LIME_DLL_EXPORTS
            #define LIME_C_API __declspec(dllexport)
        #else
            #define LIME_C_API __declspec(dllimport)
        #endif
    #elif defined(__GNUC__)
        #define LIME_C_API __attribute__((visibility("default")))
    #else
        #define LIME_C_API
    #endif
#else /* static library */
    #define LIME_C_API
#endif

#endif /* LIMESUITENG_CONFIG_C_H */
