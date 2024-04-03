
set(KISS_FFT_DIR ${CMAKE_CURRENT_LIST_DIR}/kissFFT)
add_library(kissfft STATIC ${KISS_FFT_DIR}/kiss_fft.c)
target_include_directories(kissfft PUBLIC ${KISS_FFT_DIR})
