target_sources(limesuiteng PRIVATE FFT.cpp)

# find_package(kissfft CONFIG REQUIRED CMAKE_FIND_ROOT_PATH_BOTH)
# Directly add KissFFT source into limesuiteng to avoid producing static library dependency to kissfft.a
target_include_directories(limesuiteng PRIVATE ${kissfft_SOURCE_DIR})
target_sources(limesuiteng PRIVATE ${kissfft_SOURCE_DIR}/kiss_fft.c)
