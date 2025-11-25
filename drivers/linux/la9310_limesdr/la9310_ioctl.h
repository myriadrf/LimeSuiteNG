#ifndef _LINUX_LA9310_IOCTL_H
#define _LINUX_LA9310_IOCTL_H

struct LA9310_IOCTL_firmware {
    void const* firmware_data;
    uint32_t size;
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
#define LA9310_IOCTL_CSR_OP _IOR(LA9310_IOCTL, 26, struct LA9310_IOCTL_CSR_op)

long la9310_ioctl(struct file *file, unsigned int cmd, unsigned long arg);

#endif /* _LINUX_LIMEPCIE_H */
