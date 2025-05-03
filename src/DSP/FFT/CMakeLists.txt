find_package(kissfft CONFIG REQUIRED CMAKE_FIND_ROOT_PATH_BOTH)

target_sources(limesuiteng PRIVATE FFT.cpp)
target_link_libraries(limesuiteng PRIVATE kissfft)
