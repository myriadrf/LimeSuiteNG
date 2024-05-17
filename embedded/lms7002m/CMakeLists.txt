add_library(lms7002m STATIC lms7002m.c)

target_compile_options(lms7002m PRIVATE -fvisibility=hidden -fvisibility-inlines-hidden)
target_include_directories(lms7002m PUBLIC "${PROJECT_SOURCE_DIR}/include")