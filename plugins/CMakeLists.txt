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
## QuickTest software
########################################################################
# add_subdirectory(QuickTest)

########################################################################
## SoapySDR bindings
########################################################################
add_subdirectory(SoapyLMS7)

########################################################################
## GNU Radio plugin
########################################################################
option(ENABLE_GNURADIO "Enable build library" OFF)
add_feature_info(GNURADIO ENABLE_GNURADIO "GNU Radio 3.9+ Plug-in")
if (ENABLE_GNURADIO)
    add_subdirectory(gr-limesdr)
endif()

########################################################################
## HDSDR plugin
########################################################################
cmake_dependent_option(ENABLE_HDSDR "Enable HDSDR plugin" ON "ENABLE_LIBRARY;MSVC;CMAKE_SIZEOF_VOID_P EQUAL 4" OFF)
add_feature_info(HDSDR ENABLE_HDSDR "HDSDR plugin for limesuiteng")
if (ENABLE_HDSDR)
	add_subdirectory(HDSDR)
endif()
