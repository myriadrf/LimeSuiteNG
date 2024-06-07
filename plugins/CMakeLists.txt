########################################################################
## Octave plugin integration
########################################################################
add_subdirectory(amarisoft-plugin)
# add_subdirectory(octave)

########################################################################
## freedesktop environment integration
########################################################################
add_subdirectory(Desktop)

########################################################################
## SoapySDR bindings
########################################################################
add_subdirectory(Soapy_limesuiteng)

########################################################################
## GNU Radio plugin
########################################################################
include(CMakeDependentOption)
cmake_dependent_option(ENABLE_GNURADIO_PREREQ "Build LimeSuiteNG plugin for GNU Radio" ON "ENABLE_LIBRARY;BUILD_SHARED_LIBS" OFF)

if(ENABLE_GNURADIO_PREREQ)
    add_subdirectory(gr-limesdr)
else()
    add_feature_info(GNURADIO OFF "GNU Radio 3.9+ Plug-in")
endif()

########################################################################
## HDSDR plugin
########################################################################
add_subdirectory(HDSDR)
