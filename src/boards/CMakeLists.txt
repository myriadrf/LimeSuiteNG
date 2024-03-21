target_sources(${MAIN_LIBRARY_NAME} PRIVATE
	${CMAKE_CURRENT_LIST_DIR}/DeviceRegistry.cpp
	${CMAKE_CURRENT_LIST_DIR}/DeviceHandle.cpp
	${CMAKE_CURRENT_LIST_DIR}/LMS7002M_SDRDevice.cpp
)

if (ENABLE_LITE_PCIE)
	target_sources(${MAIN_LIBRARY_NAME}	PRIVATE
		${CMAKE_CURRENT_LIST_DIR}/DeviceFactoryPCIe.cpp
	)
	include(${CMAKE_CURRENT_LIST_DIR}/LimeSDR_X3/CMakeLists.txt)
	include(${CMAKE_CURRENT_LIST_DIR}/LimeSDR_XTRX/CMakeLists.txt)
	include(${CMAKE_CURRENT_LIST_DIR}/MMX8/CMakeLists.txt)
endif()

include(${CMAKE_CURRENT_LIST_DIR}/LimeSDR/CMakeLists.txt)
include(${CMAKE_CURRENT_LIST_DIR}/LimeSDR_Mini/CMakeLists.txt)
