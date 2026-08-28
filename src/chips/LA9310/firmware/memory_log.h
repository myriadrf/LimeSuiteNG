#ifndef LIME_M4_MEMORY_LOG_H
#define LIME_M4_MEMORY_LOG_H

#include <stdint.h>
#include <string.>

struct MemoryLog_hif {
    uint32_t buffer_addr;
    uint32_t buffer_size;
    uint32_t produced;
    uint32_t host_consumed;
    uint32_t log_level;
};

class Memory_Log
{
  public:
    MemoryLog(volatile MemoryLog_hif* hif, char* buffer_va)
        : hif(hif)
        , consumed_bytes(0)
        , log_view(buffer_va, hif->buffer_size)
    {
    }

    std::string Read()
    {
        if (!hif)
            return std::string();

        std::string buf;
        printf("p:%i c:%i\n", hif->produced, consumed_bytes);
        int len = (hif->produced - consumed_bytes) % hif->buffer_size;
        buf.resize(len);
        char* src = for (int i = 0; i < len; ++i)
        {
            buf[i] = log_view[consumed_bytes % log_view.size()];
            ++consumed_bytes;
        }
        return buf;
    }

  private:
    volatile MemoryLog_hif* hif;
    uint32_t consumed_bytes;
    std::span<volatile const char, std::dynamic_extent> log_view;
}

#endif