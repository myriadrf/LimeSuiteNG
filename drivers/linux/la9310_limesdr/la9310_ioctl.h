#ifndef _LINUX_LA9310_IOCTL_H
#define _LINUX_LA9310_IOCTL_H

struct LA9310_IOCTL_firmware {
    void const* firmware_data;
    size_t size;
};

typedef enum {
    LA9310_WINDOW_BAR0,
    LA9310_WINDOW_BAR1,
    LA9310_WINDOW_BAR2,
    LA9310_WINDOW_PERSISTENT, // the driver's own uncached DMA memory: firmware images, logs
    LA9310_WINDOW_MSI,
    // DMA memory pools, their window_size is the pool size. They can't be mapped as a whole, map the buffers that
    // LA9310_IOCTL_DMA_ALLOC returns instead.
    LA9310_WINDOW_UNCACHED_POOL,
    LA9310_WINDOW_CACHED_POOL,
    LA9310_WINDOW_COUNT
} la9310_window_t;

// offsets within mmap'ed regions
struct la9310_vm_layout {
    la9310_window_t window_id; // which memory window
    size_t start_offset;
    size_t size;
};

struct LA9310_IOCTL_memory_layout {
    size_t window_size[LA9310_WINDOW_COUNT];
    struct la9310_vm_layout host_interface;
};

struct LA9310_IOCTL_CSR_op {
    la9310_window_t window_id; // which memory window
    size_t offset;
    uint32_t value;
    bool write;
};

// user space software interrupts
struct LA9310_IOCTL_SIRQ {
    uint32_t irq_index;
    uint32_t timeout_ms;
};

// user space software interrupts
struct LA9310_IOCTL_SIRQ_CTRL {
    uint32_t enable_bits;
    uint32_t enable_mask;
    uint32_t clear_bits;
    uint32_t clear_mask;
};

// A DMA buffer, as seen by the host (host_bus) and by the LA9310 (ep_pa)
struct la9310_atu {
    size_t host_bus;
    size_t ep_pa;
    size_t size;
    size_t flags; // LA9310_DMA_FLAG_*
    size_t mmap_offset;
};

// LA9310_IOCTL_DMA_ALLOC input: allocate from the uncached pool instead of the cached one
#define LA9310_DMA_FLAG_UNCACHED (1 << 0)
// LA9310_IOCTL_DMA_ALLOC output: the buffer is coherent, it needs no LA9310_IOCTL_CACHE_SYNC_* calls
#define LA9310_DMA_FLAG_COHERENT (1 << 1)

#define LA9310_IOCTL 'S'

#define LA9310_IOCTL_GET_MEMORY_LAYOUT _IOR(LA9310_IOCTL, 25, struct LA9310_IOCTL_memory_layout)
#define LA9310_IOCTL_CSR_OP _IOR(LA9310_IOCTL, 28, struct LA9310_IOCTL_CSR_op)
#define LA9310_IOCTL_LOAD_M4_FW _IOR(LA9310_IOCTL, 29, struct LA9310_IOCTL_firmware)
#define LA9310_IOCTL_LOAD_VSPA_FW _IOR(LA9310_IOCTL, 30, struct LA9310_IOCTL_firmware)
// #define LA9310_IOCTL_WAIT_FOR_DATA _IOW(LA9310_IOCTL, 31, unsigned int)
#define LA9310_IOCTL_SIRQ_WAIT _IOW(LA9310_IOCTL, 32, struct LA9310_IOCTL_SIRQ)
#define LA9310_IOCTL_SIRQ_CONTROL _IOW(LA9310_IOCTL, 33, struct LA9310_IOCTL_SIRQ_CTRL)
// Cache maintenance of a range (host_bus, size) inside a buffer that this file has allocated
#define LA9310_IOCTL_CACHE_SYNC_FOR_CPU _IOW(LA9310_IOCTL, 35, struct la9310_atu)
#define LA9310_IOCTL_CACHE_SYNC_FOR_DEVICE _IOW(LA9310_IOCTL, 36, struct la9310_atu)
// Allocates a buffer from a DMA pool. Set size and flags, the size is rounded up to whole pages, all of the fields are
// filled on return. Map the buffer with mmap_offset, on the same file descriptor. The buffer belongs to the file and is
// valid only for its lifetime, it's freed when the file is closed.
#define LA9310_IOCTL_DMA_ALLOC _IOWR(LA9310_IOCTL, 37, struct la9310_atu)
// Frees the buffer identified by host_bus. Its memory is reused only after all of its mappings are gone.
#define LA9310_IOCTL_DMA_FREE _IOW(LA9310_IOCTL, 38, struct la9310_atu)

long la9310_ioctl(struct file* file, unsigned int cmd, unsigned long arg);

#endif /* _LINUX_LIMEPCIE_H */
