# Find the kernel release
execute_process(
        COMMAND uname -r
        OUTPUT_VARIABLE KERNEL_RELEASE
        OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Find the headers
find_path(KERNELHEADERS_DIR
        include/linux/user.h
        PATHS /usr/src/linux-headers-${KERNEL_RELEASE}
        )

# Get architecture
# execute_process(
#     COMMAND uname -m
#     OUTPUT_VARIABLE ARCH
#     OUTPUT_STRIP_TRAILING_WHITESPACE
# )

if (KERNELHEADERS_DIR)
    set(LINUXKERNELHEADERS_INCLUDE_DIRS
            ${KERNELHEADERS_DIR}/include
            # ${KERNELHEADERS_DIR}/arch/${ARCH}/include
            CACHE PATH "Kernel headers include dirs"
            )
    message(STATUS "Linux Kernel release: ${KERNEL_RELEASE} (${ARCH})")
    message(STATUS "Linux Kernel headers: ${KERNELHEADERS_DIR}")
    set(LINUXKERNELHEADERS_FOUND 1 CACHE STRING "Set to 1 if kernel headers were found")
else (KERNELHEADERS_DIR)
    set(LINUXKERNELHEADERS_FOUND 0 CACHE STRING "Set to 1 if kernel headers were found")
endif (KERNELHEADERS_DIR)

mark_as_advanced(LINUXKERNELHEADERS_FOUND)
