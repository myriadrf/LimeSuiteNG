# Copyright 2020 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

########################################################################
# Check if there is C++ code at all
########################################################################
if(NOT limesdr_sources)
    message(STATUS "No C++ sources... skipping python bindings")
    return()
endif(NOT limesdr_sources)

########################################################################
# Check for pygccxml
########################################################################
gr_python_check_module_raw("pygccxml" "import pygccxml" PYGCCXML_FOUND)

include(GrPybind)

########################################################################
# Python Bindings
########################################################################

list(APPEND limesdr_python_files sink_python.cc source_python.cc python_bindings.cc)

# Generates the *_python.cc files, they could be generated with each CMake config run and not require Git tracking.
# But because the bindings rarely change and bindings generation script can have issues when crawling though system header includes,
# the generated bindings files are added to Git tracking, and marked to auto re-generate if needed.
# execute_process(COMMAND gr_modtool bind source WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
# execute_process(COMMAND gr_modtool bind sink WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})

if(ENABLE_RFE)
    list(APPEND limesdr_python_files rfe_python.cc)
    add_compile_definitions(ENABLE_RFE)

    execute_process(COMMAND gr_modtool bind rfe WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
endif()

gr_pybind_make_oot(limesdr ../.. gr::limesdr "${limesdr_python_files}")

install(
    TARGETS limesdr_python
    DESTINATION ${GR_PYTHON_DIR}/limesdr
    COMPONENT pythonapi)
