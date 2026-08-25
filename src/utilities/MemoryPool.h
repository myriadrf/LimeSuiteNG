#ifndef LIME_MEMORY_POOL_H
#define LIME_MEMORY_POOL_H

namespace lime {

class MemoryArenaPool
{
  public:
    MemoryArenaPool(void* mem, size_t size)
        : mem_buffer(mem)
        , remaining(size)
    {
    }

    void* Allocate(size_t size, size_t alignment)
    {
        if (size > remaining)
            return nullptr;

        void* ptr = mem_buffer;
        mem_buffer += size;
        remaining -= size;
        return ptr;
    }

  private:
    uint8_t* mem_buffer;
    size_t remaining;
}

} // namespace lime

#endif // LIME_MEMORY_POOL_H