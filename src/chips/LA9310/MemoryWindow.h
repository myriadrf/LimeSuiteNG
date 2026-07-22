#pragma once

namespace lime {

class MemoryWindow
{
  public:
    MemoryWindow()
        : host_va(nullptr)
        , ep_pa(0)
        , size(0){};
    MemoryWindow(void* host_va, size_t ep_pa, size_t size)
        : host_va(host_va)
        , ep_pa(ep_pa)
        , size(size){};

    MemoryWindow subspan(size_t offset, size_t size)
    {
        MemoryWindow chunk;
        chunk.host_va = static_cast<uint8_t*>(this->host_va) + offset;
        chunk.size = size;
        chunk.ep_pa = this->ep_pa + offset;
        return chunk;
    }

    void* host_va; // host virtual address
    uint32_t ep_pa; // end point physical address
    uint32_t size;
};

} // namespace lime