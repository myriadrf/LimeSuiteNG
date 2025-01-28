########################################################################
# LMS_API Wrapper lib
# wrapper functions split into separate library to avoid symbols collision
# if both limesuiteng and LimeSuite would be linked to application.
########################################################################
project(
    "limesuiteng-legacyapi"
    VERSION 1.0.0
    DESCRIPTION "Legacy LMS_API wrapper of LimeSuiteNG"
    LANGUAGES C CXX)
checkdebianchangelogversion(${PROJECT_NAME})

option(BUILD_LEGACYAPI "Build legacy LMS_API" ON)
if(NOT BUILD_LEGACYAPI)
    return()
endif()

add_library(limesuiteng-legacyapi LMS_APIWrapper.cpp)
add_library(limesuiteng::limesuiteng-legacyapi ALIAS limesuiteng-legacyapi)

set_target_properties(
    limesuiteng-legacyapi
    PROPERTIES POSITION_INDEPENDENT_CODE TRUE
               SOVERSION ${PROJECT_VERSION}
               DEFINE_SYMBOL "LIME_DLL_EXPORTS")

target_include_directories(
    limesuiteng-legacyapi PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(limesuiteng-legacyapi PUBLIC limesuiteng)

install(
    TARGETS limesuiteng-legacyapi
    EXPORT limesuitengTarget
    COMPONENT legacyapi
    DESTINATION lib${LIB_SUFFIX} # .so file
    ARCHIVE DESTINATION lib${LIB_SUFFIX} # .lib file
)

file(
    GLOB LEGACY_PUBLIC_HEADERS
    RELATIVE "${CMAKE_CURRENT_LIST_DIR}"
    "${CMAKE_CURRENT_LIST_DIR}/include/lime/*.h")

install(
    FILES ${LEGACY_PUBLIC_HEADERS}
    DESTINATION include/lime
    COMPONENT dev)
