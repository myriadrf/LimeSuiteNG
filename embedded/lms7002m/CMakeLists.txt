add_library(lms7002m STATIC lms7002m.c)

target_compile_options(lms7002m PRIVATE -fvisibility=hidden -fvisibility-inlines-hidden)
target_include_directories(lms7002m PUBLIC "${PROJECT_SOURCE_DIR}/include")

target_include_directories(lms7002m PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)

# Don't install for now.
# install(
#     TARGETS lms7002m
#     EXPORT lms7002m
#     LIBRARY DESTINATION lib${LIB_SUFFIX} # .so file
#     ARCHIVE DESTINATION lib${LIB_SUFFIX} # .lib file
#     RUNTIME DESTINATION bin # .dll file
# )
