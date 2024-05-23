add_library(lms7002m STATIC lms7002m.c calibrations.c privates.c)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
	target_compile_options(lms7002m PRIVATE -fvisibility=hidden -fvisibility-inlines-hidden)
endif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")

target_include_directories(lms7002m PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)

target_link_libraries(lms7002m PRIVATE lms_gfir)
target_include_directories(lms7002m PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/gfir>)

# Don't install for now.
# install(
#     TARGETS lms7002m
#     EXPORT lms7002m
#     LIBRARY DESTINATION lib${LIB_SUFFIX} # .so file
#     ARCHIVE DESTINATION lib${LIB_SUFFIX} # .lib file
#     RUNTIME DESTINATION bin # .dll file
# )
