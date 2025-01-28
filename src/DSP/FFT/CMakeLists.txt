find_package(kissfft CONFIG REQUIRED)

target_sources(limesuiteng PRIVATE FFT.cpp)
target_link_libraries(limesuiteng PRIVATE kissfft)
