#ifndef LA9310_VSPA_LOADER_H
#define LA9310_VSPA_LOADER_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct vspa_regs;
struct vspa_recovery;

/// @brief Number of commands the DMA channel's FIFO holds, they run in order, each after the previous one completed.
#define VSPA_LOADER_DMA_FIFO_DEPTH 2
/// @brief Number of staging buffers. A buffer is refilled before waiting for a free FIFO slot, so it has to be older
/// than everything the FIFO can still hold plus the transfer that is running.
#define VSPA_LOADER_STAGING_BUFFERS (VSPA_LOADER_DMA_FIFO_DEPTH + 2)
/// @brief Staging memory that lets every DMA transfer be of the largest size, the loader doesn't use more than this.
#define VSPA_LOADER_STAGING_SIZE (VSPA_LOADER_STAGING_BUFFERS * 0x10000u)

enum vspa_loader_status {
    VSPA_LOADER_OK = 0,
    VSPA_LOADER_INVALID_IMAGE = -1, ///< the firmware image is malformed or not for VSPA
    VSPA_LOADER_NO_MEMORY = -2, ///< the staging memory is too small
    VSPA_LOADER_BUSY = -3, ///< the core is still running
    VSPA_LOADER_TIMEOUT = -4, ///< a DMA transfer or the boot didn't complete
    VSPA_LOADER_DMA_ERROR = -5, ///< the DMA engine reported an error
    VSPA_LOADER_SYNC_ERROR = -6, ///< the staging memory sync callback failed
};

enum vspa_loader_log_level {
    VSPA_LOADER_LOG_ERROR,
    VSPA_LOADER_LOG_DEBUG,
};

/// @brief A part of the firmware image that has to be copied into the core's memory.
struct vspa_eld_section {
    const uint8_t* data; ///< the section's contents, points into the firmware image
    uint32_t size; ///< size of data in bytes
    uint32_t address; ///< destination address in the core's memory
    uint32_t xfr_ctrl; ///< DMA_XFR_CTRL transfer type that selects the destination memory
};

/// @brief Iterates over the loadable sections of a VSPA ELD (ELF) firmware image, without allocating memory.
struct vspa_eld_parser {
    const uint8_t* image;
    size_t size;
    uint32_t phoff;
    uint32_t shoff;
    uint16_t phnum;
    uint16_t phentsize;
    uint16_t shnum;
    uint16_t shentsize;
    uint32_t names_offset; ///< section name string table
    uint32_t names_size;
    uint32_t next; ///< index of the next section header to examine
    const char* error; ///< reason of the last VSPA_LOADER_INVALID_IMAGE
};

/// @brief Checks the image's ELF header and tables, and starts the iteration over its sections.
/// @param image The firmware image, it has to outlive the parser and the sections it returns.
/// @return VSPA_LOADER_OK or VSPA_LOADER_INVALID_IMAGE.
int vspa_eld_open(struct vspa_eld_parser* parser, const void* image, size_t size);

/// @brief Finds the next section that has to be loaded, in the order of the image's section table.
/// Sections without contents are skipped, the core's memory doesn't need them.
/// @return 1 if a section was returned, 0 at the end of the image, or VSPA_LOADER_INVALID_IMAGE.
int vspa_eld_next(struct vspa_eld_parser* parser, struct vspa_eld_section* section);

/// @brief Makes the CPU's writes to a part of the staging memory visible to the device.
/// @param offset Offset from the start of the staging memory.
/// @return 0 on success.
typedef int (*vspa_loader_sync_fn)(void* user, size_t offset, size_t size);

typedef void (*vspa_loader_log_fn)(void* user, enum vspa_loader_log_level level, const char* message);

/// @brief Loads the VSPA core's memory with the core's own DMA engine, and boots the firmware.
/// The transfers are queued into the DMA channel's command FIFO. While the DMA runs the queued ones, the host parses
/// the next part of the image and copies it into a free staging buffer, then waits for a free FIFO slot to queue it.
struct vspa_loader {
    /// Filled in by the caller
    volatile struct vspa_regs* regs; ///< the VSPA CCSR, mapped into the host
    uint8_t* staging; ///< host mapping of the DMA memory used to stage the image
    uint32_t staging_pa; ///< the DMA memory's address on the device's AXI bus
    size_t staging_size;
    vspa_loader_sync_fn sync_for_device;
    vspa_loader_log_fn log; ///< optional
    void* user; ///< passed to the callbacks
    /// optional, forces the core into idle when it is still busy, so a crashed firmware can be replaced
    struct vspa_recovery* recovery;

    /// Filled in by vspa_loader_prepare()
    uint32_t axi_align; ///< DMA transfer granularity, the AXI bus width in bytes
    uint32_t max_transfer; ///< largest single DMA transfer, also the size of one staging buffer
    unsigned next_buffer; ///< staging buffer for the next transfer, used in rotation
    uint32_t queued; ///< transfers queued since the start of the load
};

/// @brief Checks the whole firmware image and sets up the staging memory. Doesn't disturb the running firmware.
/// Don't call before LA9310 reports the RCW as completed, accessing the VSPA registers before it reboots the host.
int vspa_loader_prepare(struct vspa_loader* loader, const void* image, size_t size);

/// @brief Replaces the core's memory contents with the prepared image, and boots it.
/// The firmware that was running on the core has to be already stopped. If it isn't and recovery is set, the core is
/// forced into idle through the M4 debug proxy (see vspa_recovery.h).
/// The firmware reports completed boot by clearing the HOST_GO bit in the CONTROL register.
int vspa_loader_load(struct vspa_loader* loader, const void* image, size_t size);

const char* vspa_loader_strerror(int status);

#ifdef __cplusplus
}
#endif

#endif // LA9310_VSPA_LOADER_H
