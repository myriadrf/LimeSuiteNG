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

#include "drivers/linux/la9310_limesdr/la9310_ioctl.h"

struct la9310_hif;

namespace lime {

struct mmaped_region {
    void* vaddr{ 0 };
    size_t size{ 0 };
};

class LA9310_PCIe;

class PCIe_CSR_Access
{
  public:
    PCIe_CSR_Access(LA9310_PCIe* port, uint32_t window_id, size_t base_offset);
    void iowrite32(uint32_t value, size_t offset);
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
    // Methods for cache coherency

    /// @brief Clean & invalidate cache for DMEM Proxy before a read access
    /// @param pointer to address to sync
    /// @param data size to sync
    void sync_dmem_proxy_before_read(uint8_t* addr, uint32_t data_size);

    /// @brief Clean & invalidate cache for DMEM Proxy after a write access
    /// @param pointer to address to sync
    /// @param data size to sync
    void sync_dmem_proxy_after_write(uint8_t* addr, uint32_t data_size);

    /// @brief Clean & invalidate cache for IQ Flood before a read access
    /// @param pointer to address to sync
    /// @param data size to sync
    void sync_iq_flood_before_read(uint8_t* addr, uint32_t data_size);

    /// @brief Clean & invalidate cache for IQ Flood after a write access
    /// @param pointer to address to sync
    /// @param data size to sync
    void sync_iq_flood_after_write(uint8_t* addr, uint32_t data_size);

    mmaped_region GetBar(uint8_t i);
    std::shared_ptr<PCIe_CSR_Access> GetCSRAccess(uint32_t window_id, size_t base_offset = 0)
    {
        return std::make_shared<PCIe_CSR_Access>(this, window_id, base_offset);
    }

    OpStatus iowrite32(uint32_t window_id, uint32_t value, uint64_t address);
    uint32_t ioread32(uint32_t window_id, uint64_t address);

  private:
    std::filesystem::path mFilePath;
    int mFileDescriptor;
    volatile struct la9310_hif* hostInterface;
    LA9310_IOCTL_memory_layout memoryLayout;

    std::array<mmaped_region, LA9310_WINDOW_COUNT> mapped_ranges;
};

} // namespace lime

#endif // LIME_LA9310_PCIe_lime_H
