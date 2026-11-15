#ifndef LIME_LA9310_PCIe_lime_H
#define LIME_LA9310_PCIe_lime_H

#include "limesuiteng/config.h"
#include "limesuiteng/OpStatus.h"

#include <cstdint>
#include <filesystem>
#include <vector>
#include <string>
#include <array>

#include "comms/PCIe/LimePCIe.h"
#include "comms/DMA_Buffer.h"

#include "drivers/linux/la9310_limesdr/la9310_ioctl.h"

struct la9310_hif;

namespace lime {

struct mmaped_region {
    void* vaddr{ 0 };
    size_t size{ 0 };
};

class LA9310_PCIe;

/// @brief The la9310_limesdr driver's host DMA memory pools.
enum class LA9310_DMAPool : uint8_t {
    Cached, ///< Bulk data that the CPU processes, such as samples. Sync it with DMA_SyncForCPU() and DMA_SyncForDevice().
    Uncached, ///< Data that the device posts and the CPU polls, firmware staging. Needs no syncing.
};

class PCIe_CSR_Access
{
  public:
    PCIe_CSR_Access(LA9310_PCIe* port, uint32_t window_id, size_t base_offset);
    OpStatus iowrite32(uint32_t value, size_t offset);
    uint32_t ioread32(size_t offset);

  private:
    LA9310_PCIe* port;
    uint32_t window_id;
    size_t base_offset;
};

/// @brief Class for communicating with a PCIe device.
class LIME_API LA9310_PCIe : public LimePCIe
{
  public:
    LA9310_PCIe();
    virtual ~LA9310_PCIe();

    /// @brief Opens the specified PCIe device for communications.
    /// @param deviceFilename The filename of the device to open.
    /// @param flags The flags to pass to `open()`.
    /// @return The status of the operation.
    OpStatus Open(const std::filesystem::path& deviceFilename, uint32_t flags) override;

    /// @brief Closes this PCIe device.
    void Close() override;

    // Write/Read for communicating to control end points (SPI, I2C...)

    /// @brief Sends the given data buffer to the control port of the device.
    /// @param buffer The data to send to the device.
    /// @param length The length of data to send to the device.
    /// @param timeout_ms The communications timeout in ms to wait for the operation.
    /// @return The amount of bytes transferred (or -1 on error).
    virtual int WriteControl(const uint8_t* buffer, int length, int timeout_ms = 100);

    /// @brief Reads data from the device's control port.
    /// @param buffer The buffer to read the data to.
    /// @param length The maximum length of the data to read.
    /// @param timeout_ms The timeout to wait for for the operation to succeed.
    /// @return The amount of bytes read (or -1 on error).
    virtual int ReadControl(uint8_t* buffer, int length, int timeout_ms = 100);
    virtual OpStatus RunControlCommand(uint8_t* data, size_t length, int timeout_ms = 100);
    virtual OpStatus RunControlCommand(uint8_t* request, uint8_t* response, size_t length, int timeout_ms = 100);
    /// @brief Downloads the Firmware for the ARM M4 core on the LA9310 device
    /// @param Firmware data
    /// @param size of firmware data
    /// @return 0 on success, error code else
    OpStatus LoadArmM4Firmware(const char* data, size_t length);

    /// @brief Downloads the Firmware for the VSPA core on the LA9310 device
    /// @param Firmware data
    /// @param size of firmware data
    /// @return 0 on success, error code else

    mmaped_region GetBar(uint8_t i);
    std::shared_ptr<PCIe_CSR_Access> GetCSRAccess(uint32_t window_id, size_t base_offset = 0)
    {
        return std::make_shared<PCIe_CSR_Access>(this, window_id, base_offset);
    }

    /// @brief Allocates and maps a buffer from one of the driver's DMA memory pools.
    /// The buffer belongs to this port's file descriptor, the driver frees it when the descriptor is closed.
    /// @param size The requested size, rounded up to whole pages.
    /// @param buffer The allocated buffer.
    /// @param pool The pool to allocate from.
    /// @return NoMemory if the pool doesn't have enough free memory.
    OpStatus AllocDMABuffer(size_t size, DMA_Buffer& buffer, LA9310_DMAPool pool = LA9310_DMAPool::Cached);
    /// @brief Unmaps and frees a buffer returned by AllocDMABuffer().
    OpStatus FreeDMABuffer(const DMA_Buffer& buffer);

    OpStatus iowrite32(uint32_t window_id, uint32_t value, uint64_t address);
    uint32_t ioread32(uint32_t window_id, uint64_t address);

    OpStatus SendSignal(uint32_t bit);
    OpStatus WaitSIRQ(uint32_t bit, std::chrono::milliseconds timeout);
    OpStatus ClearSIRQ(uint32_t bits);
    /// @brief Makes the device's writes to the buffer visible to the CPU. Nothing to do for coherent buffers.
    OpStatus DMA_SyncForCPU(const DMA_Buffer& buffer);
    /// @brief Makes the CPU's writes to the buffer visible to the device. Nothing to do for coherent buffers.
    OpStatus DMA_SyncForDevice(const DMA_Buffer& buffer);

  private:
    int mFileDescriptor;
    std::filesystem::path mFilePath;
    volatile struct la9310_hif* hostInterface;
    LA9310_IOCTL_memory_layout memoryLayout;

    std::array<mmaped_region, LA9310_WINDOW_COUNT> mapped_ranges;
};

} // namespace lime

#endif // LIME_LA9310_PCIe_lime_H
