#pragma once

#include <stdint.h>

namespace lime {

class DMA_Buffer
{
  public:
    DMA_Buffer() {}

    DMA_Buffer(void* host_va, size_t host_bus, size_t ep_pa, size_t size)
        : host_va(host_va)
        , host_bus(host_bus)
        , ep_pa(ep_pa)
        , mem_size(size)
    {
    }

    DMA_Buffer subspan(size_t offset, size_t size)
    {
        if (offset > mem_size)
            return DMA_Buffer(0, 0, 0, 0);
        size = std::min(size, mem_size - offset);

        return DMA_Buffer(va<uint8_t>() + offset, host_bus + offset, ep_pa + offset, size);
    }

    template<typename T> T* va() const { return reinterpret_cast<T*>(host_va); }

    size_t bus() const { return host_bus; }

    size_t endpoint_pa() const { return ep_pa; }

    size_t size() const { return mem_size; }

  private:
    void* host_va;
    size_t host_bus;
    size_t ep_pa;
    size_t mem_size;
};

} // namespace lime