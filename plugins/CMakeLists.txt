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
add_subdirectory(SoapyLMS7)

########################################################################
## GNU Radio plugin
########################################################################
find_package(Gnuradio QUIET) # precheck if any gnuradio package is available
cmake_dependent_option(ENABLE_GNURADIO "Build LimeSuiteNG plugin for GNU Radio" ON "ENABLE_LIBRARY;Gnuradio_FOUND" OFF)
add_feature_info(GNURADIO ENABLE_GNURADIO "GNU Radio 3.9+ Plug-in")
if(ENABLE_GNURADIO)
    add_subdirectory(gr-limesdr)
endif()

########################################################################
## HDSDR plugin
########################################################################
add_subdirectory(HDSDR)
