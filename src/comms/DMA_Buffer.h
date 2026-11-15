#pragma once

#include <algorithm>
#include <cstddef>
#include <stdint.h>

namespace lime {

class DMA_Buffer
{
  public:
    DMA_Buffer()
        : host_va(nullptr)
        , host_bus(0)
        , ep_pa(0)
        , mem_size(0)
        , is_coherent(false)
    {
    }

    /// @param coherent The memory is uncached, the CPU and the device see the same contents without cache maintenance.
    DMA_Buffer(void* host_va, size_t host_bus, size_t ep_pa, size_t size, bool coherent = false)
        : host_va(host_va)
        , host_bus(host_bus)
        , ep_pa(ep_pa)
        , mem_size(size)
        , is_coherent(coherent)
    {
    }

    DMA_Buffer subspan(size_t offset, size_t size) const
    {
        if (offset > mem_size)
            return DMA_Buffer();
        size = std::min(size, mem_size - offset);

        return DMA_Buffer(va<uint8_t>() + offset, host_bus + offset, ep_pa + offset, size, is_coherent);
    }

    template<typename T> T* va() const { return reinterpret_cast<T*>(host_va); }

    size_t bus() const { return host_bus; }

    size_t endpoint_pa() const { return ep_pa; }

    size_t size() const { return mem_size; }

    bool coherent() const { return is_coherent; }

  private:
    void* host_va;
    size_t host_bus;
    size_t ep_pa;
    size_t mem_size;
    bool is_coherent;
};

} // namespace lime