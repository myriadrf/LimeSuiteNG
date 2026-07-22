include(FeatureSummary)

# the kernel release is decided once here, all driver subdirectories share it
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(ResolveKernelRelease)
resolve_kernel_release()

option(BUILD_DRIVERS_LIMEPCIE "Build limepcie linux kernel module" ON)
add_feature_info("BUILD_DRIVERS_LIMEPCIE" BUILD_DRIVERS_LIMEPCIE "Build limepcie driver")
if(BUILD_DRIVERS_LIMEPCIE)
    add_subdirectory(limepcie)
endif()
