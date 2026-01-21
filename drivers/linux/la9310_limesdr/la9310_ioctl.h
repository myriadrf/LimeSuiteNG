#ifndef _LINUX_LA9310_IOCTL_H
#define _LINUX_LA9310_IOCTL_H

struct LA9310_IOCTL_firmware {
    void const* firmware_data;
    size_t size;
};

struct LA9310_IOCTL_flush_cache {
    uint32_t offset;
    uint32_t size;
    uint8_t dir;
    uint8_t sync_to_cpu;
};

typedef enum {
    LA9310_WINDOW_BAR0,
    LA9310_WINDOW_BAR1,
    LA9310_WINDOW_BAR2,
    LA9310_WINDOW_SCRATCH,
    LA9310_WINDOW_MSI,
    LA9310_WINDOW_IPC,
    LA9310_WINDOW_IQFLOOD,
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

#define LA9310_IOCTL 'S'

#define LA9310_IOCTL_GET_MEMORY_LAYOUT _IOR(LA9310_IOCTL, 25, struct LA9310_IOCTL_memory_layout)
#define LA9310_IOCTL_FLUSH_CACHE_VSPA_DMEM _IOR(LA9310_IOCTL, 26, struct LA9310_IOCTL_flush_cache)
#define LA9310_IOCTL_FLUSH_CACHE_IQFLOOD _IOR(LA9310_IOCTL, 27, struct LA9310_IOCTL_flush_cache)
#define LA9310_IOCTL_CSR_OP _IOR(LA9310_IOCTL, 28, struct LA9310_IOCTL_CSR_op)
#define LA9310_IOCTL_LOAD_M4_FW _IOR(LA9310_IOCTL, 29, struct LA9310_IOCTL_firmware)
#define LA9310_IOCTL_LOAD_VSPA_FW _IOR(LA9310_IOCTL, 30, struct LA9310_IOCTL_firmware)
#define LA9310_IOCTL_WAIT_FOR_DATA _IOW(LA9310_IOCTL, 31, unsigned int)

long la9310_ioctl(struct file* file, unsigned int cmd, unsigned long arg);

#endif /* _LINUX_LIMEPCIE_H */
