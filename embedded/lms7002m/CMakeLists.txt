add_library(lms7002m STATIC lms7002m.c calibrations.c privates.c save_state.c spi.c spi_batch.c)

set_target_properties(lms7002m PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

# deny usage of floating point operations, ideally everything should be done using integers only
if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
    target_compile_options(lms7002m PRIVATE -mgeneral-regs-only -fvisibility=hidden)
elseif(CMAKE_C_COMPILER_ID STREQUAL "Clang")
    target_compile_options(lms7002m PRIVATE -mno-sse)
endif()

# public headers
target_include_directories(lms7002m PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)

# private headers, for building tests
target_include_directories(lms7002m PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>)

if(NOT BUILD_SHARED_LIBS)
    install(
        TARGETS lms7002m
        EXPORT limesuitengTarget
        LIBRARY DESTINATION lib${LIB_SUFFIX} # .so file
        ARCHIVE DESTINATION lib${LIB_SUFFIX} # .lib file
        RUNTIME DESTINATION bin # .dll file
    )
endif()

set_target_properties(lms7002m PROPERTIES OUTPUT_NAME "limesuiteng-embedded-lms7002m")
