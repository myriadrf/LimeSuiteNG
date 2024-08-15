add_subdirectory(USB)
add_subdirectory(PCIe)

target_sources(limesuiteng PRIVATE SPIChipSelectShim.cpp SPI_utilities.cpp)
