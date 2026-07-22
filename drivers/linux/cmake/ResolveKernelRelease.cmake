# Decides once, for all drivers, which kernel release the modules target.
# The results are shared through global properties, so parallel driver
# directories cannot clash and no function writes to its parent scope:
#   KMOD_KERNEL_RELEASE       - the kernel release string
#   KMOD_KERNEL_AUTODETECTED  - TRUE when taken from the running kernel
#   KMOD_KERNEL_RELEASE_STAMP - stamp file path (autodetected mode only)
# In autodetected mode a single 'kernel-release-stamp' target refreshes the
# stamp file when the running kernel changes, and every module depending on
# it gets rebuilt for the new kernel. A manually set KMOD_KERNEL_RELEASE
# (cross-compiling, packaging in chroots) is trusted as is, no uname involved.

function(resolve_kernel_release)
    get_property(ALREADY_RESOLVED GLOBAL PROPERTY KMOD_KERNEL_RELEASE SET)
    if(ALREADY_RESOLVED)
        return()
    endif()

    if(KMOD_KERNEL_RELEASE)
        set_property(GLOBAL PROPERTY KMOD_KERNEL_RELEASE "${KMOD_KERNEL_RELEASE}")
        set_property(GLOBAL PROPERTY KMOD_KERNEL_AUTODETECTED FALSE)
        return()
    endif()

    execute_process(
        COMMAND uname -r
        OUTPUT_VARIABLE DETECTED_KERNEL_RELEASE
        OUTPUT_STRIP_TRAILING_WHITESPACE)
    set_property(GLOBAL PROPERTY KMOD_KERNEL_RELEASE "${DETECTED_KERNEL_RELEASE}")
    set_property(GLOBAL PROPERTY KMOD_KERNEL_AUTODETECTED TRUE)

    set(KERNEL_RELEASE_STAMP ${CMAKE_BINARY_DIR}/kernel.release)
    set_property(GLOBAL PROPERTY KMOD_KERNEL_RELEASE_STAMP "${KERNEL_RELEASE_STAMP}")
    if(NOT TARGET kernel-release-stamp)
        add_custom_target(
            kernel-release-stamp
            COMMAND sh -c "uname -r | cmp -s - '${KERNEL_RELEASE_STAMP}' || uname -r > '${KERNEL_RELEASE_STAMP}'"
            BYPRODUCTS ${KERNEL_RELEASE_STAMP}
            VERBATIM)
    endif()
endfunction()
