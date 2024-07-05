add_library(lms7002m STATIC lms7002m.c calibrations.c privates.c save_state.c)

if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
    target_compile_options(lms7002m PRIVATE -fvisibility=hidden)
endif(CMAKE_C_COMPILER_ID STREQUAL "GNU")

target_include_directories(lms7002m PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
set_property(TARGET lms7002m PROPERTY POSITION_INDEPENDENT_CODE TRUE)

option(FLOATING_POINT_AVAILABLE "embedded lms7002m: can use floating point math" ON)

if (FLOATING_POINT_AVAILABLE)
    target_link_libraries(lms7002m PRIVATE lms_gfir)
    target_include_directories(lms7002m PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/gfir>)
    target_compile_definitions(lms7002m PRIVATE "FLOATING_POINT_AVAILABLE")
else()
    message(STATUS "COMPILER " ${CMAKE_CXX_COMPILER_ID}})
    if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
        target_compile_options(lms7002m PRIVATE -mgeneral-regs-only)
    endif()
    if(CMAKE_C_COMPILER_ID STREQUAL "Clang")
        target_compile_options(lms7002m PRIVATE -mno-sse)
    endif()
endif(FLOATING_POINT_AVAILABLE)

# Don't install for now.
# install(
#     TARGETS lms7002m
#     EXPORT lms7002m
#     LIBRARY DESTINATION lib${LIB_SUFFIX} # .so file
#     ARCHIVE DESTINATION lib${LIB_SUFFIX} # .lib file
#     RUNTIME DESTINATION bin # .dll file
# )
