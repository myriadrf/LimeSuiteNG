# Copyright 2020 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

########################################################################
# Check if there is C++ code at all
########################################################################
if(NOT limesuiteng_sources)
    message(STATUS "No C++ sources... skipping python bindings")
    return()
endif(NOT limesuiteng_sources)

########################################################################
# Check for pygccxml
########################################################################
gr_python_check_module_raw("pygccxml" "import pygccxml" PYGCCXML_FOUND)

# GrPybind cmake scripts that were before 3.10.7 had wrong paths when used from nested project
string(COMPARE LESS ${Gnuradio_VERSION} "3.10.7.0" USE_LOCAL_GRPYBIND)
if(${USE_LOCAL_GRPYBIND})
    message(
        AUTHOR_WARNING "host GnuRadio version ${Gnuradio_VERSION}, overriding it's GrPybind cmake file to work for nested projects."
    )
    include(${PROJECT_SOURCE_DIR}/cmake/GrPybind.cmake)
else()
    include(GrPybind)
endif()

########################################################################
# Python Bindings
########################################################################

list(APPEND limesuiteng_python_files sdrdevice_source_python.cc sdrdevice_sink_python.cc python_bindings.cc)

gr_pybind_make_oot(limesuiteng ../../.. gr::limesuiteng "${limesuiteng_python_files}")

# copy bindings extension for use in QA test module
add_custom_command(
    TARGET limesuiteng_python
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:limesuiteng_python> ${PROJECT_BINARY_DIR}/test_modules/gnuradio/limesuiteng/)

install(
    TARGETS limesuiteng_python
    DESTINATION ${GR_PYTHON_DIR}/gnuradio/limesuiteng
    COMPONENT gr-limesuiteng)
