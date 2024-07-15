// SPDX-License-Identifier: BSD-2-Clause

#define DEBUG

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/pci.h>
#include <linux/pci_regs.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/version.h>
#include <linux/ctype.h>
#include <linux/mutex.h>
#include <linux/jiffies.h>
#include <linux/wait.h>
#include <linux/mutex.h>

#include "limepcie.h"
#include "bsp/csr.h"
#include "bsp/config.h"
#include "boards.h"

#define XILINX_FPGA_VENDOR_ID 0x10EE
#define XILINX_FPGA_DEVICE_ID 0x7022
#define ALTERA_FPGA_VENDOR_ID 0x1172
#define ALTERA_FPGA_DEVICE_ID 0xe001
#define XTRX_FPGA_DEVICE_ID 0x7023

#define EXPECTED_PCI_REVISION_ID 1

//#define DEBUG_CSR
//#define DEBUG_MSI
//#define DEBUG_POLL
//#define DEBUG_READ
//#define DEBUG_WRITE
//#define DEBUG_MEM

#define DRIVER_NAME "limepcie"
#define LIMEPCIE_MINOR_COUNT 32

#define VERSION "1.0.0"
#define VERSION_MAJOR 1
#define VERSION_MINOR 0
#define VERSION_PATCH 0

MODULE_INFO(version, VERSION);
MODULE_INFO(author, "Lime Microsystems");

#define MAX_DMA_BUFFER_COUNT 256
#define MAX_DMA_CHANNEL_COUNT 16

struct limepcie_device;

struct limepcie_device_attributes {
    uint32_t vendor;
    uint32_t product;
};

struct deviceInfo {
    uint64_t serialNumber;
    uint16_t firmware;
    uint8_t boardId;
    uint8_t protocol;
    uint8_t hardwareVer;
    char devName[256];
};

struct limepcie_dma {
    struct limepcie_device *owner;
    uint64_t transferCounter;
    uint32_t bar_offset;

    void *buffers[MAX_DMA_BUFFER_COUNT];
    uint32_t bufferSize;
    uint16_t bufferCount;
    uint8_t interrupt_vector_index;
    bool enabled;

    enum dma_data_direction direction;
    dma_addr_t dmaAddrHandles[MAX_DMA_BUFFER_COUNT];
    wait_queue_head_t wait_transfer; // wait for ongoing transfer

    uint8_t id;

    struct {
        uint32_t enable;
        uint32_t table_value;
        uint32_t table_write_enable;
        uint32_t table_loop_prog_n;
        uint32_t table_loop_status;
        uint32_t table_flush;
    } csr_addr;
};

struct limepcie_data_cdev {
    struct limepcie_device *owner;
    struct limepcie_dma *fromDevice;
    struct limepcie_dma *toDevice;
    struct cdev cdevNode;
    int minor;
};

struct limepcie_device {
    struct pci_dev *pciContext;
    resource_size_t bar0_size;
    uint8_t *bar0_addr; // virtual address of BAR0

    struct limepcie_dma channels[MAX_DMA_CHANNEL_COUNT * 2];
    uint8_t channelsCount;

    struct limepcie_data_cdev data_cdevs[MAX_DMA_CHANNEL_COUNT];
    int data_cdevs_count;

    spinlock_t lock;

    struct deviceInfo info;
    struct limepcie_device_attributes attr;
    struct semaphore control_semaphore;
    int minor_base;
    int irq_count;
    struct limepcie_data_cdev control_cdev; // non DMA channel for control packets
};

static int gDeviceCounter = 0;

static const char *dma_dir_str(enum dma_data_direction direction)
{
    switch (direction)
    {
    case DMA_FROM_DEVICE:
        return "FROM_DEVICE";
    case DMA_TO_DEVICE:
        return "TO_DEVICE";
    default:
        return "NONE";
    }
}

static void *limepcie_dma_buffer_alloc(struct limepcie_dma *dma, uint32_t bufferSize, dma_addr_t *dmaHandle)
{
    struct device *sysDev = &dma->owner->pciContext->dev;

    void *memoryBuffer = kmalloc(bufferSize, GFP_KERNEL);
    if (!memoryBuffer)
    {
        dev_err(sysDev, "Failed to allocate dma buffer\n");
        return NULL;
    }

    dma_addr_t dma_bus = dma_map_single(sysDev, memoryBuffer, bufferSize, dma->direction);
    if (dma_mapping_error(sysDev, dma_bus))
    {
        dev_err(sysDev, "dma_map_single error @ va:%p pa:%llX\n", memoryBuffer, virt_to_phys(memoryBuffer));
        kfree(memoryBuffer);
        return NULL;
    }

    *dmaHandle = dma_bus;
    return memoryBuffer;
}

static void limepcie_dma_buffer_free(struct limepcie_dma *dma, void *va_memory, dma_addr_t dmaHandle)
{
    if (dmaHandle)
        dma_unmap_single(&dma->owner->pciContext->dev, dmaHandle, dma->bufferSize, dma->direction);
    if (va_memory)
        kfree(va_memory);
}

// Initializes a single direction data transfer controls
static int limepcie_dma_init(
    struct limepcie_dma *dma, uint8_t moduleIndex, enum dma_data_direction direction, uint32_t bufferSize, uint16_t bufferCount)
{
    struct device *sysDev = &dma->owner->pciContext->dev;

    memset(&dma->buffers, 0, sizeof(dma->buffers));
    memset(&dma->dmaAddrHandles, 0, sizeof(dma->dmaAddrHandles));

    dma->id = moduleIndex;
    dma->direction = direction;
    dma->bufferCount = 0;
    init_waitqueue_head(&dma->wait_transfer);
    switch (moduleIndex)
    {
#define CASE_DMA(x) \
    case x: { \
        dma->bar_offset = CSR_PCIE_DMA##x##_BASE; \
        dma->interrupt_vector_index = \
            (direction == DMA_FROM_DEVICE) ? PCIE_DMA##x##_WRITER_INTERRUPT : PCIE_DMA##x##_READER_INTERRUPT; \
        break; \
    }

        CASE_DMA(7);
        CASE_DMA(6);
        CASE_DMA(5);
        CASE_DMA(4);
        CASE_DMA(3);
        CASE_DMA(2);
        CASE_DMA(1);
    default:
        CASE_DMA(0);
#undef CASE_DMA
    }

    uint32_t base = dma->bar_offset;
    if (direction == DMA_FROM_DEVICE)
    {
        dma->csr_addr.enable = base + PCIE_DMA_WRITER_ENABLE_OFFSET;
        dma->csr_addr.table_value = base + PCIE_DMA_WRITER_TABLE_VALUE_OFFSET;
        dma->csr_addr.table_write_enable = base + PCIE_DMA_WRITER_TABLE_WE_OFFSET;
        dma->csr_addr.table_loop_prog_n = base + PCIE_DMA_WRITER_TABLE_LOOP_PROG_N_OFFSET;
        dma->csr_addr.table_loop_status = base + PCIE_DMA_WRITER_TABLE_LOOP_STATUS_OFFSET;
        dma->csr_addr.table_flush = base + PCIE_DMA_WRITER_TABLE_FLUSH_OFFSET;
    }
    else
    {
        dma->csr_addr.enable = base + PCIE_DMA_READER_ENABLE_OFFSET;
        dma->csr_addr.table_value = base + PCIE_DMA_READER_TABLE_VALUE_OFFSET;
        dma->csr_addr.table_write_enable = base + PCIE_DMA_READER_TABLE_WE_OFFSET;
        dma->csr_addr.table_loop_prog_n = base + PCIE_DMA_READER_TABLE_LOOP_PROG_N_OFFSET;
        dma->csr_addr.table_loop_status = base + PCIE_DMA_READER_TABLE_LOOP_STATUS_OFFSET;
        dma->csr_addr.table_flush = base + PCIE_DMA_READER_TABLE_FLUSH_OFFSET;
    }

    dma->bufferSize = bufferSize;
    for (int i = 0; i < bufferCount; ++i)
    {
        void *memoryBuffer = limepcie_dma_buffer_alloc(dma, bufferSize, &dma->dmaAddrHandles[i]);
        if (!memoryBuffer)
        {
            dev_err(sysDev, "Failed to allocate dma buffer\n");
            return -ENOMEM;
        }
#ifdef DEBUG_MEM
        dev_dbg(sysDev,
            "DMA%i %s buffer[%i]: va:%p pa:%llx dma:%llx\n",
            dma->id,
            dma_dir_str(dma->direction),
            i,
            memoryBuffer,
            virt_to_phys(memoryBuffer),
            dma->dmaAddrHandles[i]);
#endif
        dma->buffers[dma->bufferCount] = memoryBuffer;
        ++dma->bufferCount;
    }
    return 0;
}

static void limepcie_dma_destroy(struct limepcie_dma *dma)
{
    if (!dma || !dma->owner)
        return;

    for (int i = 0; i < dma->bufferCount; ++i)
        limepcie_dma_buffer_free(dma, dma->buffers[i], dma->dmaAddrHandles[i]);

    memset(&dma->buffers, 0, sizeof(dma->buffers));
    memset(&dma->dmaAddrHandles, 0, sizeof(dma->dmaAddrHandles));
    dma->bufferCount = 0;
    dma->bufferSize = 0;
}

static int limepcie_major;
static int limepcie_minor_idx;
static struct class *limepcie_class;
static dev_t limepcie_dev_t;

static inline uint32_t limepcie_readl(struct limepcie_device *s, uint32_t addr)
{
    uint32_t val = readl(s->bar0_addr + addr - CSR_BASE);
#ifdef DEBUG_CSR
    dev_dbg(&s->dev->dev, "csr_read: 0x%08x @ 0x%08x", val, addr);
#endif
    return val;
}

static inline void limepcie_writel(struct limepcie_device *s, uint32_t addr, uint32_t val)
{
#ifdef DEBUG_CSR
    dev_dbg(&s->dev->dev, "csr_write: 0x%08x @ 0x%08x", val, addr);
#endif
    return writel(val, s->bar0_addr + addr - CSR_BASE);
}

static void limepcie_enable_interrupt(struct limepcie_device *s, int irq_num)
{
    uint32_t v = limepcie_readl(s, CSR_PCIE_MSI_ENABLE_ADDR);
    v |= (1 << irq_num);
    limepcie_writel(s, CSR_PCIE_MSI_ENABLE_ADDR, v);
}

static void limepcie_disable_interrupt(struct limepcie_device *s, int irq_num)
{
    uint32_t v = limepcie_readl(s, CSR_PCIE_MSI_ENABLE_ADDR);
    v &= ~(1 << irq_num);
    limepcie_writel(s, CSR_PCIE_MSI_ENABLE_ADDR, v);
}

static uint64_t GetContinuousTransfersCount(struct limepcie_device *s, struct limepcie_dma *dma, int bufferCount)
{
    uint32_t status = limepcie_readl(s, dma->csr_addr.table_loop_status);
    uint16_t tableCount = status >> 16;
    uint16_t tableRow = status & 0xFFFF;
    return tableCount * bufferCount + tableRow;
}

static int limepcie_dma_request(struct limepcie_dma *dma, dma_addr_t dmaAddrHandle, uint32_t size, bool genIRQ)
{
    struct limepcie_device *myDevice = dma->owner;
    struct device *sysDev = &myDevice->pciContext->dev;
    if (size == 0 || size > dma->bufferSize)
    {
        dev_err(sysDev, "DMA request: invalid size %i\n", size);
        return -EINVAL;
    }

    // Fill DMA Writer descriptor, buffer size + parameters.
    limepcie_writel(myDevice,
        dma->csr_addr.table_value,
#ifndef DMA_BUFFER_ALIGNED
        DMA_LAST_DISABLE |
#endif
            (!genIRQ) * DMA_IRQ_DISABLE | /* generate an msi */
            size);
    // Fill 32-bit Address LSB.
    limepcie_writel(myDevice, dma->csr_addr.table_value + 4, (dmaAddrHandle >> 0) & 0xffffffff);
    // Write descriptor (and fill 32-bit Address MSB for 64-bit mode).
    limepcie_writel(myDevice, dma->csr_addr.table_write_enable, (dmaAddrHandle >> 32) & 0xffffffff);
#ifdef DEBUG_READ
    dev_dbg(sysDev, "DMA request - buf:x%llx size:%i irq:%i\n", dmaAddrHandle, size, genIRQ ? 1 : 0);
#endif
    return 0;
}

static int limepcie_dma_start_continuous(struct limepcie_dma *dma, uint32_t transferSize, uint8_t irqFreq)
{
    struct limepcie_device *myDevice = dma->owner;
    struct device *sysDev = &myDevice->pciContext->dev;
    if (transferSize == 0 || transferSize > dma->bufferSize)
    {
        dev_err(sysDev, "DMA start: invalid write size %i\n", transferSize);
        return -EINVAL;
    }

    // Fill DMA Writer descriptors.
    limepcie_writel(myDevice, dma->csr_addr.enable, 0);
    limepcie_writel(myDevice, dma->csr_addr.table_flush, 1);
    limepcie_writel(myDevice, dma->csr_addr.table_loop_prog_n, 0);
    for (int i = 0; i < dma->bufferCount; i++)
    {
        const bool genIRQ = (i % irqFreq == 0);
        int ret = limepcie_dma_request(dma, dma->dmaAddrHandles[i], transferSize, genIRQ);
        if (ret != 0)
        {
            dev_err(sysDev, "DMA%i start failed\n", dma->id);
            return ret;
        }
    }
    limepcie_writel(myDevice, dma->csr_addr.table_loop_prog_n, 1);
    dma->transferCounter = 0;

    /* Start DMA Writer. */
    limepcie_writel(myDevice, dma->csr_addr.enable, 1);
    limepcie_enable_interrupt(dma->owner, dma->interrupt_vector_index);
    dma->enabled = true;
    dev_info(sysDev,
        "DMA%i %s start continuous, buffer size: %u, IRQ every: %i buffers, IRQi:%i\n",
        dma->id,
        dma_dir_str(dma->direction),
        transferSize,
        irqFreq,
        dma->interrupt_vector_index);
    return 0;
}

static void limepcie_dma_start(struct limepcie_dma *dma)
{
    if (!dma || !dma->owner)
    {
        pr_err("limepcie : limepcie_dma_start nullptr");
        return;
    }
    struct limepcie_device *myDevice = dma->owner;
    // Fill DMA Reader descriptors.
    limepcie_writel(myDevice, dma->csr_addr.enable, 0);
    limepcie_writel(myDevice, dma->csr_addr.table_flush, 1);
    limepcie_writel(myDevice, dma->csr_addr.table_loop_prog_n, 0);
    dma->transferCounter = 0;
    dma->enabled = true;
    // start dma reader
    limepcie_writel(myDevice, dma->csr_addr.enable, 1);
    limepcie_enable_interrupt(myDevice, dma->interrupt_vector_index);
    dev_info(
        &myDevice->pciContext->dev, "DMA%i %s start, IRQi:%i\n", dma->id, dma_dir_str(dma->direction), dma->interrupt_vector_index);
}

static void limepcie_dma_stop(struct limepcie_dma *dma)
{
    if (!dma || !dma->owner)
    {
        pr_err("limepcie : limepcie_dma_stop nullptr");
        return;
    }
    struct limepcie_device *myDevice = dma->owner;
    // Flush and stop DMA Writer.
    limepcie_disable_interrupt(dma->owner, dma->interrupt_vector_index);
    const int transfersDone = GetContinuousTransfersCount(myDevice, dma, MAX_DMA_BUFFER_COUNT);
    limepcie_writel(myDevice, dma->csr_addr.table_loop_prog_n, 0);
    limepcie_writel(myDevice, dma->csr_addr.table_flush, 1);
    udelay(1000);
    limepcie_writel(myDevice, dma->csr_addr.enable, 0);
    limepcie_writel(myDevice, dma->csr_addr.table_flush, 1);
    if (dma->enabled)
        dev_info(&myDevice->pciContext->dev,
            "DMA%i %s stop, transfers completed: %i\n",
            dma->id,
            dma_dir_str(dma->direction),
            transfersDone);
    dma->enabled = false;
    wake_up_interruptible(&dma->wait_transfer);
}

static irqreturn_t limepcie_handle_interrupt(int irq, void *data)
{
    struct limepcie_device *myDevice = (struct limepcie_device *)data;
    uint32_t irq_enable = limepcie_readl(myDevice, CSR_PCIE_MSI_ENABLE_ADDR);
    if (!irq_enable)
        return IRQ_NONE;

    uint32_t irq_vector;
// Single MSI
#ifdef CSR_PCIE_MSI_CLEAR_ADDR
    irq_vector = limepcie_readl(myDevice, CSR_PCIE_MSI_VECTOR_ADDR);
// Multi-Vector MSI
#else
    irq_vector = 0;
    for (int index = 0; index < myDevice->irq_count; ++index)
    {
        if (irq == pci_irq_vector(myDevice->pciContext, index))
        {
            irq_vector = (1 << index);
            break;
        }
    }
#endif

#ifdef DEBUG_MSI
    struct device *sysDev = &myDevice->pciContext->dev;
    dev_dbg(sysDev, "MSI: 0x%x , enabled: 0x%x\n", irq_vector, irq_enable);
#endif
    irq_vector &= irq_enable;

    const bool forceWake = irq_vector == 0;
    if (irq_vector == 0)
        return IRQ_NONE;

    uint32_t clear_mask = 0;
    for (int i = 0; i < myDevice->channelsCount; ++i)
    {
        struct limepcie_dma *dma = &myDevice->channels[i];
        if (!(irq_vector & (1 << dma->interrupt_vector_index)) || !dma->enabled)
            continue;

        // GetContinuousTransfersCount(s, &dma->dma, MAX_DMA_BUFFER_COUNT)
        uint32_t counters = limepcie_readl(myDevice, dma->csr_addr.table_loop_status);
        if (dma->direction == DMA_FROM_DEVICE)
            dma->transferCounter = ((counters >> 8) & 0xFF00) | (counters & 0xFF);
        else
            dma->transferCounter = (counters & 0xFFFF);
#ifdef DEBUG_MSI
        dev_dbg(
            sysDev, "MSI DMA%d %s cnt:%llu, status:%08X\n", dma->id, dma_dir_str(dma->direction), dma->transferCounter, counters);
#endif
        // int index = (dma->transferCounter - 1) & 0xFF;
        // dma_sync_single_for_cpu(&myDevice->pciContext->dev, dma->dmaAddrHandles[index], dma->bufferSize, dma->direction);
        // uint8_t* bytes = dma->buffers[index];
        // uint64_t timestamp;
        // memcpy(&timestamp, &bytes[8], sizeof(uint64_t));
        // dev_dbg(&myDevice->pciContext->dev, "buf[%i] PKT TS: %llu\n", index, timestamp);
        wake_up_interruptible(&dma->wait_transfer);
        if (!forceWake)
            clear_mask |= (1 << dma->interrupt_vector_index);
    }

#ifdef CSR_PCIE_MSI_CLEAR_ADDR
    if (clear_mask)
        limepcie_writel(myDevice, CSR_PCIE_MSI_CLEAR_ADDR, clear_mask);
#endif
    return IRQ_HANDLED;
}

static int limepcie_open(struct inode *inode, struct file *file)
{
    struct limepcie_data_cdev *channel = container_of(inode->i_cdev, struct limepcie_data_cdev, cdevNode);
    file->private_data = channel;
    dev_dbg(&channel->owner->pciContext->dev, "Open %s\n", file->f_path.dentry->d_iname);
    return 0;
}

static int limepcie_release(struct inode *inode, struct file *file)
{
    struct limepcie_data_cdev *channel = file->private_data;
    if (channel == NULL)
        return 0;

    dev_dbg(&channel->owner->pciContext->dev, "Release %s\n", file->f_path.dentry->d_iname);

    if (channel->fromDevice)
        limepcie_dma_stop(channel->fromDevice);
    if (channel->toDevice)
        limepcie_dma_stop(channel->toDevice);
    return 0;
}

static int limepcie_mmap(struct file *file, struct vm_area_struct *vma)
{
    struct limepcie_data_cdev *cdev = file->private_data;
    struct limepcie_dma *dma = cdev->fromDevice;
    struct limepcie_device *myDevice = dma->owner;
    struct device *sysDev = &myDevice->pciContext->dev;

    unsigned long pfn;
    int is_tx, i;

    const int totalBufferSize = dma->bufferCount * dma->bufferSize;

    if (vma->vm_pgoff == 0)
    {
        dma = cdev->toDevice;
        is_tx = 1;
    }
    else if (vma->vm_pgoff == (totalBufferSize >> PAGE_SHIFT))
    {
        dma = cdev->fromDevice;
        is_tx = 0;
    }
    else
    {
        dev_err(sysDev, "mmap page offset bad\n");
        return -EINVAL;
    }

    if (vma->vm_end - vma->vm_start != totalBufferSize)
    {
        dev_err(sysDev, "vma->vm_end - vma->vm_start != %i, got: %lu\n", totalBufferSize, vma->vm_end - vma->vm_start);
        return -EINVAL;
    }

    vm_flags_set(vma, VM_IO | VM_DONTEXPAND | VM_DONTDUMP);

    for (i = 0; i < dma->bufferCount; i++)
    {
        void *va = dma->buffers[i];
        pfn = virt_to_phys((void *)va) >> PAGE_SHIFT;
        /*
		 * Note: the memory is cached, so the user must explicitly
		 * flush the CPU caches on architectures which require it.
		 */

        size_t usrPtr = vma->vm_start + i * dma->bufferSize;
        int remapRet = remap_pfn_range(vma, usrPtr, pfn, dma->bufferSize, vma->vm_page_prot);
        if (remapRet)
        {
            dev_err(sysDev, "mmap io_remap_pfn_range failed %i\n", remapRet);
            return -EAGAIN;
        }
    }

    return 0;
}

static long limepcie_ioctl_control(struct file *file, unsigned int cmd, unsigned long arg)
{
    long ret = 0;
    struct limepcie_data_cdev *cdev = file->private_data;
    struct limepcie_device *myDevice = cdev->owner;

    switch (cmd)
    {
    case LIMEPCIE_IOCTL_RUN_CONTROL_COMMAND: {
        // struct semaphore *sem = &(myDevice->control_semaphore);
        struct limepcie_control_packet m;

        if (copy_from_user(&m, (void *)arg, sizeof(m)))
        {
            ret = -EFAULT;
            break;
        }

        uint64_t end_time = ktime_get_raw_ns() + m.timeout_ms * 1000000llu;

        // int success = down_timeout(sem, msecs_to_jiffies(m.timeout_ms));
        // if (success != 0) // on failure
        // {
        //     ret = -EBUSY;
        //     break;
        // }

        uint32_t byteCount = min(m.length, (uint32_t)(CSR_CNTRL_CNTRL_SIZE * sizeof(uint32_t)));
        uint32_t value;
        for (int i = 0; i < byteCount; i += sizeof(value))
        {
            memcpy(&value, m.request + i, sizeof(value));
            limepcie_writel(myDevice, CSR_CNTRL_BASE + i, value);
        }

        int success = false;
        while (ktime_get_raw_ns() < end_time)
        {
            uint32_t status = limepcie_readl(myDevice, CSR_CNTRL_BASE);
            if ((status & 0xFF00) != 0)
            {
                success = true;
                break;
            }
        }

        if (!success)
        {
            // up(sem);
            ret = -EFAULT;
            break;
        }

        for (int i = 0; i < byteCount; i += sizeof(uint32_t))
        {
            value = limepcie_readl(myDevice, CSR_CNTRL_BASE + i);
            memcpy(m.response + i, &value, sizeof(uint32_t));
        }

        if (copy_to_user((void *)arg, &m, sizeof(m)))
        {
            // up(sem);
            ret = -EFAULT;
            break;
        }

        // up(sem);
    }
    break;
    case LIMEPCIE_IOCTL_VERSION: {
        struct limepcie_version m;

        m.major = VERSION_MAJOR;
        m.minor = VERSION_MINOR;
        m.patch = VERSION_PATCH;

        if (copy_to_user((void *)arg, &m, sizeof(m)))
        {
            ret = -EFAULT;
            break;
        }
    }
    break;
    default:
        ret = -ENOIOCTLCMD;
        break;
    }
    return ret;
}

static long limepcie_ioctl_trx(struct file *file, unsigned int cmd, unsigned long arg)
{
    long ret = 0;

    struct limepcie_data_cdev *cdev = file->private_data;
    struct limepcie_dma *fromDevice = cdev->fromDevice;
    struct limepcie_dma *toDevice = cdev->toDevice;

    switch (cmd)
    {
    case LIMEPCIE_IOCTL_VERSION: {
        struct limepcie_version m;

        m.major = VERSION_MAJOR;
        m.minor = VERSION_MINOR;
        m.patch = VERSION_PATCH;

        if (copy_to_user((void *)arg, &m, sizeof(m)))
        {
            ret = -EFAULT;
            break;
        }
    }
    break;
    case LIMEPCIE_IOCTL_DMA_CONTROL: {
        struct limepcie_ioctl_dma_control m;

        if (copy_from_user(&m, (void *)arg, sizeof(m)))
        {
            ret = -EFAULT;
            break;
        }
        struct limepcie_dma *dma = m.directionFromDevice ? fromDevice : toDevice;
        if (m.enabled != dma->enabled)
        {
            if (m.enabled)
                limepcie_dma_start(dma);
            else
                limepcie_dma_stop(dma);
        }
    }
    break;
    case LIMEPCIE_IOCTL_DMA_CONTROL_CONTINUOUS: {
        struct limepcie_ioctl_dma_control_continuous m;

        if (copy_from_user(&m, (void *)arg, sizeof(m)))
        {
            ret = -EFAULT;
            break;
        }
        struct limepcie_dma *dma = m.control.directionFromDevice ? fromDevice : toDevice;

        if (m.control.enabled != dma->enabled)
        {
            if (m.control.enabled)
                ret = limepcie_dma_start_continuous(dma, m.transferSize, m.irqPeriod);
            else
                limepcie_dma_stop(dma);
        }
    }
    break;
    case LIMEPCIE_IOCTL_DMA_STATUS: {
        struct limepcie_ioctl_dma_status m;

        if (copy_from_user(&m, (void *)arg, sizeof(m)))
        {
            ret = -EFAULT;
            break;
        }

        if (m.wait_for_read)
        {
            uint64_t before = fromDevice->transferCounter;
            wait_event_interruptible(fromDevice->wait_transfer, (!fromDevice->enabled || fromDevice->transferCounter != before));
        }

        if (m.wait_for_write)
        {
            uint64_t before = toDevice->transferCounter;
            wait_event_interruptible(toDevice->wait_transfer, (!toDevice->enabled || toDevice->transferCounter != before));
        }

        m.fromDeviceCounter = fromDevice ? fromDevice->transferCounter : 0;
        m.toDeviceCounter = toDevice ? toDevice->transferCounter : 0;

        if (copy_to_user((void *)arg, &m, sizeof(m)))
        {
            ret = -EFAULT;
            break;
        }
    }
    break;
    case LIMEPCIE_IOCTL_MMAP_DMA_INFO: {
        struct limepcie_ioctl_mmap_dma_info m;

        m.dma_tx_buf_offset = 0;
        m.dma_tx_buf_size = toDevice->bufferSize;
        m.dma_tx_buf_count = toDevice->bufferCount;

        m.dma_rx_buf_offset = fromDevice->bufferCount * fromDevice->bufferSize;
        m.dma_rx_buf_size = fromDevice->bufferSize;
        m.dma_rx_buf_count = fromDevice->bufferCount;

        if (copy_to_user((void *)arg, &m, sizeof(m)))
        {
            ret = -EFAULT;
            break;
        }
    }
    break;
    case LIMEPCIE_IOCTL_DMA_REQUEST: {
        struct limepcie_ioctl_dma_request m;

        if (copy_from_user(&m, (void *)arg, sizeof(m)))
        {
            dev_dbg(&cdev->owner->pciContext->dev, "LIMEPCIE_IOCTL_MMAP_DMA_READER_UPDATE copy_from_user fail");
            ret = -EFAULT;
            break;
        }
        uint8_t bufferIndex = m.bufferIndex & 0xFF;
        struct limepcie_dma *dma = m.directionFromDevice ? fromDevice : toDevice;
        ret = limepcie_dma_request(dma, dma->dmaAddrHandles[bufferIndex], m.transferSize, m.generateIRQ);
    }
    break;
    case LIMEPCIE_IOCTL_CACHE_FLUSH: {
        struct limepcie_cache_flush m;

        if (copy_from_user(&m, (void *)arg, sizeof(m)))
        {
            dev_dbg(&cdev->owner->pciContext->dev, "LIMEPCIE_IOCTL_CACHE_FLUSH copy_from_user fail");
            ret = -EFAULT;
            break;
        }

        uint8_t bufferIndex = m.bufferIndex & 0xFF;

        struct limepcie_dma *dma = m.directionFromDevice ? fromDevice : toDevice;
        if (m.sync_to_cpu)
            dma_sync_single_for_cpu(
                &dma->owner->pciContext->dev, dma->dmaAddrHandles[bufferIndex], dma->bufferSize, dma->direction);
        else
            dma_sync_single_for_device(
                &dma->owner->pciContext->dev, dma->dmaAddrHandles[bufferIndex], dma->bufferSize, dma->direction);
        ret = 0;
    }
    break;
    case LIMEPCIE_IOCTL_LOCK: {
        /*
        struct limepcie_ioctl_lock m;

        if (copy_from_user(&m, (void *)arg, sizeof(m)))
        {
            ret = -EFAULT;
            break;
        }

        m.dma_reader_status = 1;
        if (m.dma_reader_request)
        {
            if (chan->dma.reader_lock)
            {
                m.dma_reader_status = 0;
            }
            else
            {
                chan->dma.reader_lock = 1;
                chan_priv->reader = 1;
            }
        }
        if (m.dma_reader_release)
        {
            chan->dma.reader_lock = 0;
            chan_priv->reader = 0;
        }

        m.dma_writer_status = 1;
        if (m.dma_writer_request)
        {
            if (chan->dma.writer_lock)
            {
                m.dma_writer_status = 0;
            }
            else
            {
                chan->dma.writer_lock = 1;
                chan_priv->writer = 1;
            }
        }
        if (m.dma_writer_release)
        {
            chan->dma.writer_lock = 0;
            chan_priv->writer = 0;
        }

        if (copy_to_user((void *)arg, &m, sizeof(m)))
        {
            ret = -EFAULT;
            break;
        }*/
    }
    break;
    default:
        ret = -ENOIOCTLCMD;
        break;
    }
    return ret;
}

static int limepcie_ctrl_open(struct inode *inode, struct file *file)
{
    struct limepcie_data_cdev *ctrlDevice = container_of(inode->i_cdev, struct limepcie_data_cdev, cdevNode);
    file->private_data = ctrlDevice;
    dev_dbg(&ctrlDevice->owner->pciContext->dev, "Open %s\n", file->f_path.dentry->d_iname);

    limepcie_writel(ctrlDevice->owner, CSR_CNTRL_TEST_ADDR, 0x55);
    if (limepcie_readl(ctrlDevice->owner, CSR_CNTRL_TEST_ADDR) != 0x55)
    {
        printk(KERN_ERR DRIVER_NAME " CSR register test failed\n");
        return -EIO;
    }
    return 0;
}

static ssize_t limepcie_ctrl_write(struct file *file, const char __user *userbuf, size_t count, loff_t *offset)
{
    struct limepcie_device *s = file->private_data;
    uint32_t value;
    count = min(count, CSR_CNTRL_CNTRL_SIZE * sizeof(uint32_t));
    for (int i = 0; i < count; i += sizeof(value))
    {
        if (copy_from_user(&value, userbuf + i, sizeof(value)))
            return -EFAULT;
        limepcie_writel(s, CSR_CNTRL_BASE + i, value);
    }
    return count;
}

static ssize_t limepcie_ctrl_read(struct file *file, char __user *userbuf, size_t count, loff_t *offset)
{
    struct limepcie_device *s = file->private_data;
    uint32_t value;
    count = min(count, CSR_CNTRL_CNTRL_SIZE * sizeof(uint32_t));
    for (int i = 0; i < count; i += sizeof(value))
    {
        value = limepcie_readl(s, CSR_CNTRL_BASE + i);
        if (copy_to_user(userbuf + i, &value, sizeof(value)))
            return -EFAULT;
    }
    return count;
}

static pci_ers_result_t limepcie_error_detected(struct pci_dev *pciContext, pci_channel_state_t state)
{
    switch (state)
    {
    case pci_channel_io_normal:
        dev_err(&pciContext->dev, "PCI error_detected. Channel state(normal)\n");
        break;
    case pci_channel_io_frozen:
        dev_err(&pciContext->dev, "PCI error_detected. Channel state(frozen)\n");
        break;
    case pci_channel_io_perm_failure:
        dev_err(&pciContext->dev, "PCI error_detected. Channel state(dead)\n");
        break;
    default:
        dev_err(&pciContext->dev, "PCI error_detected\n");
    }
    return PCI_ERS_RESULT_NONE;
}
// int (*mmio_enabled)(struct pci_dev *dev);
static pci_ers_result_t limepcie_slot_reset(struct pci_dev *pciContext)
{
    dev_err(&pciContext->dev, "PCI slot reset\n");
    return PCI_ERS_RESULT_NONE;
}
// void (*resume)(struct pci_dev *dev);

static const struct pci_error_handlers pci_error_handlers_ops = {
    .error_detected = limepcie_error_detected,
    // .mmio_enabled = mmio_enabled,
    .slot_reset = limepcie_slot_reset,
    // .resume = resume
};

static const struct file_operations limepcie_fops_trx = {
    .owner = THIS_MODULE,
    .unlocked_ioctl = limepcie_ioctl_trx,
    .open = limepcie_open,
    .release = limepcie_release,
    // .read = limepcie_read,
    // .poll = limepcie_poll,
    // .write = limepcie_write,
    .mmap = limepcie_mmap,
};

static const struct file_operations limepcie_fops_control = {
    .owner = THIS_MODULE,
    .unlocked_ioctl = limepcie_ioctl_control,
    .open = limepcie_ctrl_open,
    .release = limepcie_release,
    .read = limepcie_ctrl_read,
    .write = limepcie_ctrl_write,
};

static ssize_t product_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct limepcie_device *limeDev = dev_get_drvdata(dev);
    return snprintf(buf, PAGE_SIZE, "0x%4x\n", limeDev->attr.product);
}

static ssize_t vendor_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct limepcie_device *limeDev = dev_get_drvdata(dev);
    return snprintf(buf, PAGE_SIZE, "0x%4x\n", limeDev->attr.vendor);
}

static DEVICE_ATTR(product, 0444, product_show, NULL);
static DEVICE_ATTR(vendor, 0444, vendor_show, NULL);

static int limepcie_cdev_create(
    struct limepcie_data_cdev *cdev, struct device *parentSysDev, const char *name, const struct file_operations *ops)
{
    WARN_ON(cdev->owner == NULL);

    struct device *sysDev = &cdev->owner->pciContext->dev;
    int index = limepcie_minor_idx;

    cdev_init(&cdev->cdevNode, ops);
    int ret = cdev_add(&cdev->cdevNode, MKDEV(limepcie_major, index), 1);
    if (ret < 0)
    {
        dev_err(&cdev->owner->pciContext->dev, "Failed to allocate cdev\n");
        return ret;
    }

    dev_info(sysDev, "Creating /dev/%s\n", name);
    struct device *trxDev = device_create(limepcie_class, parentSysDev, MKDEV(limepcie_major, index), NULL, "%s", name);
    if (!trxDev)
    {
        dev_err(sysDev, "Failed to create device\n");
        cdev_del(&cdev->cdevNode);
        return -EINVAL;
    }
    cdev->minor = index;
    ++limepcie_minor_idx;
    device_create_file(trxDev, &dev_attr_product);
    device_create_file(trxDev, &dev_attr_vendor);
    return 0;
}

static void limepcie_cdev_destroy(struct limepcie_data_cdev *cdev)
{
    if (!cdev)
        return;
    device_destroy(limepcie_class, MKDEV(limepcie_major, cdev->minor));
    cdev_del(&cdev->cdevNode);
}

static void FreeIRQs(struct limepcie_device *myDevice)
{
    if (myDevice->irq_count <= 0)
        return;

    dev_dbg(&myDevice->pciContext->dev, "FreeIRQs %i\n", myDevice->irq_count);
    for (int index = 0; index < myDevice->irq_count; ++index)
    {
        int irq = pci_irq_vector(myDevice->pciContext, index);
        free_irq(irq, myDevice);
    }
    myDevice->irq_count = 0;
    pci_free_irq_vectors(myDevice->pciContext);
}

static int AllocateIRQs(struct limepcie_device *myDevice)
{
    struct pci_dev *pciContext = myDevice->pciContext;
    int irq_count = pci_alloc_irq_vectors(pciContext, 1, 32, PCI_IRQ_MSI);
    if (irq_count < 0)
    {
        dev_err(&pciContext->dev, "Failed to enable MSI\n");
        return irq_count;
    }
    dev_info(&pciContext->dev, "%d MSI IRQs allocated.\n", irq_count);

    myDevice->irq_count = 0;
    for (int index = 0; index < irq_count; ++index)
    {
        int irq = pci_irq_vector(pciContext, index);
        int ret = request_irq(irq, limepcie_handle_interrupt, IRQF_SHARED, DRIVER_NAME, myDevice);
        if (ret < 0)
        {
            dev_err(&pciContext->dev, " Failed to allocate IRQ %d\n", irq);
            FreeIRQs(myDevice);
            return ret;
        }
        ++myDevice->irq_count;
    }
    return 0;
}

struct LMS64CPacket {
    uint8_t cmd;
    uint8_t status;
    uint8_t blockCount;
    uint8_t periphID;
    uint8_t reserved[4];
    uint8_t payload[56];
};

static int ReadInfo(struct limepcie_device *myDevice)
{
    memset(&myDevice->info, 0, sizeof(myDevice->info));

    struct LMS64CPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.cmd = 0x00; // GET_INFO
    uint8_t *buffer = (uint8_t *)&pkt;
    int count = 64;
    uint32_t value;
    for (int i = 0; i < count; i += sizeof(uint32_t))
    {
        memcpy(&value, buffer + i, sizeof(value));
        limepcie_writel(myDevice, CSR_CNTRL_BASE + i, value);
    }

    for (int n = 0; n < 10; ++n)
    {
        value = limepcie_readl(myDevice, CSR_CNTRL_BASE);
        memcpy(buffer, &value, sizeof(value));
        if (pkt.status != 0)
            break;
        mdelay(10); // wait between reads until MCU gives answer
    }
    if (pkt.status == 0)
        return -EIO;

    for (int i = 0; i < count; i += sizeof(uint32_t))
    {
        value = limepcie_readl(myDevice, CSR_CNTRL_BASE + i);
        memcpy(buffer + i, &value, sizeof(value));
    }

    myDevice->info.firmware = (pkt.payload[9] << 16) | pkt.payload[0];
    myDevice->info.boardId = pkt.payload[1];
    myDevice->info.protocol = pkt.payload[2];
    myDevice->info.hardwareVer = pkt.payload[3];
    uint64_t serialNumber = 0;
    for (int i = 10; i < 18; ++i)
    {
        serialNumber <<= 8;
        serialNumber |= pkt.payload[i];
    }
    myDevice->info.serialNumber = serialNumber;

    sprintf(myDevice->info.devName, "%s", GetDeviceName(myDevice->info.boardId));

    dev_info(&myDevice->pciContext->dev,
        "[device info] %s FW:%u HW:%u PROTOCOL:%u S/N:0x%016llX \n",
        myDevice->info.devName,
        myDevice->info.firmware,
        myDevice->info.hardwareVer,
        myDevice->info.protocol,
        myDevice->info.serialNumber);
    return 0;
}

static bool ValidIdentifier(const char *identifier)
{
    if (strncmp("Lime", identifier, 4) == 0)
        return true;
    return false;
}

static bool ValidChar(char c)
{
    return isalnum(c) || c == '-' || c == '_' || c == ' ';
}

static void SanitizeIdentifier(const char *identifier, char *destination, const size_t dstSize)
{
    uint8_t size = 0;
    while (size < dstSize && ValidChar(identifier[size]))
        size++;

    while (size > 0 && identifier[size - 1] == ' ')
        size--;

    size = size <= dstSize ? size : dstSize;
    strncpy(destination, identifier, size);

    for (size_t i = 0; i < size; i++)
        if (destination[i] == ' ')
            destination[i] = '-';
}

// Checks if the device provides human readable name
static int IdentifyDevice(struct limepcie_device *myDevice)
{
    char fpga_identifier[256];
    for (int i = 0; i < 256; i++)
        fpga_identifier[i] = limepcie_readl(myDevice, CSR_IDENTIFIER_MEM_BASE + i * 4);
    fpga_identifier[255] = '\0';

    if (fpga_identifier[0] != 0)
        dev_info(&myDevice->pciContext->dev, "Identifier: %s\n", fpga_identifier);
    if (ValidIdentifier(fpga_identifier))
        SanitizeIdentifier(fpga_identifier, myDevice->info.devName, sizeof(myDevice->info.devName));
    else if (ReadInfo(myDevice) != 0)
    {
        dev_err(&myDevice->pciContext->dev, "Failed to read device info from FPGA MCU\n");
        return -EIO;
    }
    return 0;
}

static int limepcie_configure_pci(struct limepcie_device *myDevice)
{
    struct pci_dev *pciContext = myDevice->pciContext;
    struct device *sysDev = &pciContext->dev;
    int ret = 0;
    if ((ret = pcim_enable_device(pciContext)))
    {
        dev_err(sysDev, "Cannot enable device\n");
        return ret;
    }

    // Check device version
    uint8_t rev_id;
    pci_read_config_byte(pciContext, PCI_REVISION_ID, &rev_id);
    if (rev_id != EXPECTED_PCI_REVISION_ID)
    {
        dev_err(sysDev, "Unexpected device PCI revision %d\n", rev_id);
        return -1;
    }

    // Check bar0 config
    if (!(pci_resource_flags(pciContext, 0) & IORESOURCE_MEM))
    {
        dev_err(sysDev, "Invalid BAR0 configuration\n");
        return -1;
    }

    if (pcim_iomap_regions(pciContext, BIT(0), DRIVER_NAME) < 0)
    {
        dev_err(sysDev, "Could not request iomap regions\n");
        return -1;
    }

    if ((myDevice->bar0_addr = pcim_iomap_table(pciContext)[0]) == NULL)
    {
        dev_err(sysDev, "Could not map BAR0\n");
        return -1;
    }

    dev_dbg(sysDev, "BAR0 address=0x%p\n", myDevice->bar0_addr);
    return 0;
}

static int limepcie_device_create_cdev_trx(
    struct limepcie_device *myDevice, struct limepcie_dma *toDevice, struct limepcie_dma *fromDevice, const char *name)
{
    struct device *sysDev = &myDevice->pciContext->dev;

    struct limepcie_data_cdev *dmaCdev = &myDevice->data_cdevs[myDevice->data_cdevs_count];
    dmaCdev->owner = myDevice;
    dmaCdev->toDevice = toDevice;
    dmaCdev->fromDevice = fromDevice;
    if (limepcie_cdev_create(dmaCdev, sysDev, name, &limepcie_fops_trx) != 0)
        return -1;
    ++myDevice->data_cdevs_count;
    return 0;
}

// Initialize requested number of DMA channels
// Return number of channels created
static int limepcie_device_trx_setup(struct limepcie_device *myDevice, uint8_t trxCount, uint32_t bufferSize)
{
    WARN_ON(myDevice->channelsCount > 0);
    if (trxCount > MAX_DMA_CHANNEL_COUNT)
    {
        dev_err(&myDevice->pciContext->dev, "Requesting too many DMA channels. Limiting to %i.\n", MAX_DMA_CHANNEL_COUNT);
        trxCount = MAX_DMA_CHANNEL_COUNT;
    }

    myDevice->channelsCount = 0;
    for (int i = 0; i < trxCount; ++i)
    {
        struct limepcie_dma *toDeviceChannel = &myDevice->channels[myDevice->channelsCount];
        toDeviceChannel->owner = myDevice;
        int ret = limepcie_dma_init(toDeviceChannel, i, DMA_TO_DEVICE, bufferSize, MAX_DMA_BUFFER_COUNT);
        if (ret)
            break;
        ++myDevice->channelsCount;

        struct limepcie_dma *fromDeviceChannel = &myDevice->channels[myDevice->channelsCount];
        fromDeviceChannel->owner = myDevice;
        ret = limepcie_dma_init(fromDeviceChannel, i, DMA_FROM_DEVICE, bufferSize, MAX_DMA_BUFFER_COUNT);
        if (ret)
            break;
        ++myDevice->channelsCount;

        char cdev_name[128];
        snprintf(cdev_name, sizeof(cdev_name), "limepcie%i/trx%i", gDeviceCounter, i);
        ret = limepcie_device_create_cdev_trx(myDevice, toDeviceChannel, fromDeviceChannel, cdev_name);
        if (ret)
            break;
    }
    return myDevice->channelsCount / 2;
}

static int limepcie_device_create_cdev_control(struct limepcie_device *myDevice, const char *name)
{
    struct device *sysDev = &myDevice->pciContext->dev;
    struct limepcie_data_cdev *cdev = &myDevice->control_cdev;
    cdev->owner = myDevice;
    cdev->fromDevice = NULL;
    cdev->toDevice = NULL;
    return limepcie_cdev_create(cdev, sysDev, name, &limepcie_fops_control);
}

static int limepcie_device_init(struct limepcie_device *myDevice, struct pci_dev *pciContext)
{
    struct device *sysDev = &pciContext->dev;
    myDevice->pciContext = pciContext;
    pci_set_drvdata(pciContext, myDevice);
    int ret = limepcie_configure_pci(myDevice);

    /* Reset LimePCIe core */
#ifdef CSR_CTRL_RESET_ADDR
    limepcie_writel(myDevice, CSR_CTRL_RESET_ADDR, 1);
    msleep(10);
#endif
    if ((ret = IdentifyDevice(myDevice)))
        return ret;

    pcie_print_link_status(pciContext);
    pci_set_master(pciContext);

    uint64_t required_dma_mask = dma_get_required_mask(sysDev);
    if (required_dma_mask > DMA_BIT_MASK(32))
    {
        dev_warn(sysDev,
            "System required DMA MASK: 0x%llX. Raspberry Pi needs to have dtoverlay=pcie-32bi-dma, to allow 32bit dma addressing. "
            "Data streaming might not work correctly.\n",
            required_dma_mask);
    }

    if ((ret = dma_set_mask(sysDev, DMA_BIT_MASK(32))))
    {
        dev_warn(sysDev, "Failed to set DMA mask 32bit. Falling back to 64bit\n");
        if ((ret = dma_set_mask(sysDev, DMA_BIT_MASK(64))))
        {
            dev_err(sysDev, "Failed to set DMA mask 64bit. Critical error.\n");
            return -1;
        }
    }

    ret = AllocateIRQs(myDevice);
    if (ret < 0)
        return -1;

    uint32_t dmaBufferSize = 8192;
    switch (myDevice->info.boardId)
    {
    case LMS_DEV_LIMESDR_X3:
    case LMS_DEV_LIMESDR_QPCIE:
        dmaBufferSize = 32768;
        break;
    case LMS_DEV_LIMESDR_XTRX:
    case LMS_DEV_LIME_MM_X8:
        dmaBufferSize = 8192;
        break;
    default:
        dmaBufferSize = 8192;
        dev_warn(sysDev, "DMA buffer size not specified, defaulting to %i", dmaBufferSize);
    }

    int32_t dmaFullDuplexChannels = limepcie_readl(myDevice, CSR_CNTRL_NDMA_ADDR);
    if (dmaFullDuplexChannels > MAX_DMA_CHANNEL_COUNT || dmaFullDuplexChannels < 0)
    {
        dev_err(sysDev, "Invalid DMA channel count(%i)\n", dmaFullDuplexChannels);
        dmaFullDuplexChannels = 0;
    }
    else
        dev_info(sysDev, "DMA channels: %i", dmaFullDuplexChannels);

    ret = limepcie_device_trx_setup(myDevice, dmaFullDuplexChannels, dmaBufferSize);
    if (ret < 0)
        return ret;
    else if (ret != dmaFullDuplexChannels)
        dev_err(sysDev, "Unable to create expected number of dma channels, expected:%i, got:%i\n", dmaFullDuplexChannels * 2, ret);

    char cdev_name[128];
    snprintf(cdev_name, sizeof(cdev_name), "limepcie%i/control0", gDeviceCounter);
    ret = limepcie_device_create_cdev_control(myDevice, cdev_name);
    if (ret < 0)
        return ret;

    ++gDeviceCounter;

    sema_init(&myDevice->control_semaphore, 1);
    return 0;
}

static void limepcie_device_destroy(struct limepcie_device *myDevice)
{
    for (int i = 0; i < myDevice->channelsCount; ++i)
        limepcie_dma_stop(&myDevice->channels[i]);

    // Disable all interrupts
    limepcie_writel(myDevice, CSR_PCIE_MSI_ENABLE_ADDR, 0);
    FreeIRQs(myDevice);

    for (int i = 0; i < myDevice->channelsCount; ++i)
        limepcie_dma_destroy(&myDevice->channels[i]);
    myDevice->channelsCount = 0;

    for (int i = 0; i < myDevice->data_cdevs_count; ++i)
        limepcie_cdev_destroy(&myDevice->data_cdevs[i]);

    limepcie_cdev_destroy(&myDevice->control_cdev);
}

static int limepcie_pci_probe(struct pci_dev *pciContext, const struct pci_device_id *id)
{
    struct device *sysDev = &pciContext->dev;
    dev_dbg(sysDev, "[Probing device] vid:%04X pid:%04X\n", id->vendor, id->device);

    struct limepcie_device *myDevice = devm_kzalloc(sysDev, sizeof(struct limepcie_device), GFP_KERNEL);
    if (!myDevice)
    {
        dev_err(sysDev, "Failed to allocate memory for device");
        return -ENOMEM;
    }

    int ret = limepcie_device_init(myDevice, pciContext);
    if (ret < 0)
    {
        dev_err(sysDev, "Probe fail:\n");
        limepcie_device_destroy(myDevice);
        return -1;
    }

    myDevice->attr.vendor = id->vendor;
    myDevice->attr.product = id->device;
    return 0;
}

static void limepcie_pci_device_remove(struct pci_dev *pciContext)
{
    dev_info(&pciContext->dev, "[Removing device] vid:%04X pid:%04X\n", pciContext->vendor, pciContext->device);
    struct limepcie_device *myDevice = pci_get_drvdata(pciContext);
    limepcie_device_destroy(myDevice);
}

static const struct pci_device_id limepcie_pci_ids[] = {{PCI_DEVICE(XILINX_FPGA_VENDOR_ID, XILINX_FPGA_DEVICE_ID)},
    {PCI_DEVICE(XILINX_FPGA_VENDOR_ID, XTRX_FPGA_DEVICE_ID)},
    {PCI_DEVICE(ALTERA_FPGA_VENDOR_ID, ALTERA_FPGA_DEVICE_ID)},
    {0}};
MODULE_DEVICE_TABLE(pci, limepcie_pci_ids);

static struct pci_driver limepcie_pci_driver = {
    .name = DRIVER_NAME,
    .id_table = limepcie_pci_ids,
    .probe = limepcie_pci_probe,
    .remove = limepcie_pci_device_remove,
    .err_handler = &pci_error_handlers_ops,
};

static int __init limepcie_module_init(void)
{
    int ret;
    pr_info("limepcie : module init\n");
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 4, 0)
    limepcie_class = class_create(THIS_MODULE, DRIVER_NAME);
#else
    limepcie_class = class_create(DRIVER_NAME);
#endif
    if (!limepcie_class)
    {
        ret = -EEXIST;
        pr_err(" Failed to create class \"limepcie\"\n");
        goto fail_create_class;
    }

    ret = alloc_chrdev_region(&limepcie_dev_t, 0, LIMEPCIE_MINOR_COUNT, DRIVER_NAME);
    if (ret < 0)
    {
        pr_err(" Could not allocate char device\n");
        goto fail_alloc_chrdev_region;
    }
    limepcie_major = MAJOR(limepcie_dev_t);
    limepcie_minor_idx = MINOR(limepcie_dev_t);

    ret = pci_register_driver(&limepcie_pci_driver);
    if (ret < 0)
    {
        pr_err(" Error while registering PCI driver\n");
        goto fail_register;
    }
    return 0;

fail_register:
    unregister_chrdev_region(limepcie_dev_t, LIMEPCIE_MINOR_COUNT);
fail_alloc_chrdev_region:
    class_destroy(limepcie_class);
fail_create_class:
    return ret;
}

static void __exit limepcie_module_exit(void)
{
    pci_unregister_driver(&limepcie_pci_driver);
    unregister_chrdev_region(limepcie_dev_t, LIMEPCIE_MINOR_COUNT);
    class_destroy(limepcie_class);
    pr_info("limepcie : module exit\n");
}

module_init(limepcie_module_init);
module_exit(limepcie_module_exit);

MODULE_LICENSE("GPL");
