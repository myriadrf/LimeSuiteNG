include(FeatureSummary)
include(CMakeDependentOption)

########################################################################
## Octave plugin integration
########################################################################
cmake_dependent_option(BUILD_PLUGINS_AMARISOFT "Build LimeSuiteNG plugin for Amarisoft" ON "UNIX;BUILD_CORE" OFF)
if(UNIX) # show feature only on Linux
    add_feature_info("BUILD_PLUGINS_AMARISOFT" BUILD_PLUGINS_AMARISOFT "Build LimeSuiteNG plugin for Amarisoft")
endif()
if(BUILD_PLUGINS_AMARISOFT)
    add_subdirectory(amarisoft-plugin)
endif()
# add_subdirectory(octave)

########################################################################
## freedesktop environment integration
########################################################################
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    cmake_dependent_option(INSTALL_DESKTOP "Enable LimeSuiteNG launcher and icons" ON "BUILD_GUI" OFF)
    add_feature_info("INSTALL_FREEDESKTOP" INSTALL_DESKTOP "LimeSuiteNG freedesktop integration")
    if(INSTALL_FREEDESKTOP)
        add_subdirectory(Desktop)
    endif()
endif()

########################################################################
## SoapySDR bindings
########################################################################
find_package(SoapySDR CONFIG)
cmake_dependent_option(BUILD_PLUGINS_SOAPYSDR "Build LimeSuiteNG SoapySDR bindings" ON "BUILD_CORE;SoapySDR_FOUND" OFF)
add_feature_info("BUILD_PLUGINS_SOAPYSDR" BUILD_PLUGINS_SOAPYSDR "SoapySDR bindings for LimeSuiteNG")
if(BUILD_PLUGINS_SOAPYSDR)
    add_subdirectory(soapysdr)
endif()

########################################################################
## GNU Radio plugin
########################################################################
# find_package(Gnuradio "3.10") # Can't use find gnuradio here, as it copies python_compiler_helper.py to top project dir, which is later not found within gr plugin build directory
add_subdirectory(gr-limesuiteng)

########################################################################
## HDSDR plugin
########################################################################
# HDSDR is Windows only
if(CMAKE_HOST_WIN32)
    cmake_dependent_option(BUILD_PLUGINS_HDSDR "Build HDSDR plugin" ON "BUILD_CORE;MSVC;CMAKE_SIZEOF_VOID_P EQUAL 4;" OFF)
    add_feature_info("BUILD_PLUGINS_HDSDR" BUILD_PLUGINS_HDSDR "HDSDR (32bit only) plugin for limesuiteng")
    if(BUILD_PLUGINS_HDSDR)
        add_subdirectory(HDSDR)
    endif()
endif()
