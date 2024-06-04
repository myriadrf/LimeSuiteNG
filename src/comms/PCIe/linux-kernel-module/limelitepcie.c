// SPDX-License-Identifier: BSD-2-Clause

#define DEBUG

/*
 * LitePCIe driver
 *
 * This file is part of LitePCIe.
 *
 * Copyright (C) 2018-2020 / EnjoyDigital  / florent@enjoy-digital.fr
 */

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

#include "limelitepcie.h"
#include "csr.h"
#include "config.h"
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

#define LIMELITEPCIE_NAME "limelitepcie"
#define LIMELITEPCIE_MINOR_COUNT 32

#define VERSION "1.0.0"
#define VERSION_MAJOR 1
#define VERSION_MINOR 0
#define VERSION_PATCH 0

MODULE_INFO(version, VERSION);

#define MAX_DMA_BUFFER_COUNT 256
#define MAX_DMA_CHANNEL_COUNT 16

struct limelitepcie_dma_chan {
    uint32_t base;
    uint32_t writer_interrupt;
    uint32_t reader_interrupt;
    uint32_t bufferCount;
    uint32_t bufferSize;
    dma_addr_t reader_handle[MAX_DMA_BUFFER_COUNT];
    dma_addr_t writer_handle[MAX_DMA_BUFFER_COUNT];
    void *reader_addr[MAX_DMA_BUFFER_COUNT];
    void *writer_addr[MAX_DMA_BUFFER_COUNT];
    int64_t reader_hw_count;
    int64_t reader_hw_count_last;
    int64_t reader_sw_count;
    int64_t writer_hw_count;
    int64_t writer_hw_count_last;
    int64_t writer_sw_count;
    uint8_t writer_enable;
    uint8_t reader_enable;
    uint8_t writer_lock;
    uint8_t reader_lock;
    uint8_t reader_irqFreq;
};

struct deviceInfo {
    uint64_t serialNumber;
    uint16_t firmware;
    uint8_t boardId;
    uint8_t protocol;
    uint8_t hardwareVer;
    char devName[256];
};

struct limelitepcie_chan {
    struct limelitepcie_device *limelitepcie_dev;
    struct limelitepcie_dma_chan dma;
    struct cdev cdev;
    uint32_t block_size;
    uint32_t core_base;
    wait_queue_head_t wait_rd; /* to wait for an ongoing read */
    wait_queue_head_t wait_wr; /* to wait for an ongoing write */

    int index;
    int minor;
};

struct dev_attr {
    uint32_t vendor;
    uint32_t product;
};

struct limelitepcie_device {
    struct pci_dev *dev;
    struct platform_device *uart;
    resource_size_t bar0_size;
    phys_addr_t bar0_phys_addr;
    uint8_t *bar0_addr; /* virtual address of BAR0 */
    struct limelitepcie_chan chan[MAX_DMA_CHANNEL_COUNT];
    spinlock_t lock;
    int minor_base;
    int irqs;
    int channels;
    struct cdev control_cdev; // non DMA channel for control packets
    struct deviceInfo info;
    struct semaphore control_semaphore;
    spinlock_t device_spinlock;
    struct dev_attr attr;
};

struct limelitepcie_chan_priv {
    struct limelitepcie_chan *chan;
    bool reader;
    bool writer;
};

static int limelitepcie_major;
static int limelitepcie_minor_idx;
static struct class *limelitepcie_class;
static dev_t limelitepcie_dev_t;

static inline uint32_t limelitepcie_readl(struct limelitepcie_device *s, uint32_t addr)
{
    uint32_t val = readl(s->bar0_addr + addr - CSR_BASE);
#ifdef DEBUG_CSR
    dev_dbg(&s->dev->dev, "csr_read: 0x%08x @ 0x%08x", val, addr);
#endif
    return val;
}

static inline void limelitepcie_writel(struct limelitepcie_device *s, uint32_t addr, uint32_t val)
{
#ifdef DEBUG_CSR
    dev_dbg(&s->dev->dev, "csr_write: 0x%08x @ 0x%08x", val, addr);
#endif
    return writel(val, s->bar0_addr + addr - CSR_BASE);
}

static void limelitepcie_enable_interrupt(struct limelitepcie_device *s, int irq_num)
{
    uint32_t v = limelitepcie_readl(s, CSR_PCIE_MSI_ENABLE_ADDR);
    v |= (1 << irq_num);
    limelitepcie_writel(s, CSR_PCIE_MSI_ENABLE_ADDR, v);
}

static void limelitepcie_disable_interrupt(struct limelitepcie_device *s, int irq_num)
{
    uint32_t v = limelitepcie_readl(s, CSR_PCIE_MSI_ENABLE_ADDR);
    v &= ~(1 << irq_num);
    limelitepcie_writel(s, CSR_PCIE_MSI_ENABLE_ADDR, v);
}

static int limelitepcie_dma_init(struct limelitepcie_device *s)
{
    if (!s)
        return -ENODEV;

    /* for each dma channel */
    for (int i = 0; i < s->channels; i++)
    {
        struct limelitepcie_dma_chan *dmachan = &s->chan[i].dma;
        /* for each dma buffer */
        for (int j = 0; j < dmachan->bufferCount; j++)
        {
            dmachan->writer_addr[j] = kmalloc(dmachan->bufferSize, GFP_KERNEL);
            dmachan->writer_handle[j] = dma_map_single(&s->dev->dev, dmachan->writer_addr[j], dmachan->bufferSize, DMA_FROM_DEVICE);
#ifdef DEBUG_MEM
            dev_dbg(&s->dev->dev,
                "Writer[%i]: va:%p pa:%llx dma:%llx\n",
                j,
                dmachan->writer_addr[j],
                virt_to_phys(dmachan->writer_addr[j]),
                dmachan->writer_handle[j]);
#endif
            if (dma_mapping_error(&s->dev->dev, dmachan->writer_handle[j]))
            {
                dev_err(&s->dev->dev, "dma_mapping_error\n");
                return -ENOMEM;
            }
            if (!dmachan->writer_addr[j])
            {
                dev_err(&s->dev->dev, "Failed to allocate dma buffers\n");
                return -ENOMEM;
            }
        }

        for (int j = 0; j < dmachan->bufferCount; j++)
        {
            /* allocate rd */
            dmachan->reader_addr[j] = kmalloc(dmachan->bufferSize, GFP_KERNEL);
            dmachan->reader_handle[j] = dma_map_single(&s->dev->dev, dmachan->reader_addr[j], dmachan->bufferSize, DMA_TO_DEVICE);
#ifdef DEBUG_MEM
            dev_dbg(&s->dev->dev,
                "Reader[%i]: va:%p pa:%llx dma:%llx\n",
                j,
                dmachan->reader_addr[j],
                virt_to_phys(dmachan->reader_addr[j]),
                dmachan->reader_handle[j]);
#endif
            if (dma_mapping_error(&s->dev->dev, dmachan->reader_handle[j]))
            {
                dev_err(&s->dev->dev, "dma_mapping_error\n");
                return -ENOMEM;
            }

            if (!dmachan->reader_addr[j])
            {
                dev_err(&s->dev->dev, "Failed to allocate dma buffers\n");
                return -ENOMEM;
            }
        }
    }
    return 0;
}

static int limelitepcie_dma_free(struct limelitepcie_device *s)
{
    if (!s)
        return -ENODEV;

    /* for each dma channel */
    for (int i = 0; i < s->channels; i++)
    {
        struct limelitepcie_dma_chan *dmachan = &s->chan[i].dma;
        /* for each dma buffer */
        for (int j = 0; j < dmachan->bufferCount; j++)
        {
            /* allocate rd */
            if (dmachan->reader_addr[j])
            {
#ifdef DEBUG_MEM
                dev_dbg(&s->dev->dev,
                    "Free Reader[%i]: va:%p pa:%llx dma:%llx\n",
                    j,
                    dmachan->reader_addr[j],
                    virt_to_phys(dmachan->reader_addr[j]),
                    dmachan->reader_handle[j]);
#endif
                dma_unmap_single(&s->dev->dev, dmachan->reader_handle[j], dmachan->bufferSize, DMA_TO_DEVICE);
                kfree(dmachan->reader_addr[j]);
            }

            if (dmachan->writer_addr[j])
            {
#ifdef DEBUG_MEM
                dev_dbg(&s->dev->dev,
                    "Free Writer[%i]: va:%p pa:%llx dma:%llx\n",
                    j,
                    dmachan->writer_addr[j],
                    virt_to_phys(dmachan->writer_addr[j]),
                    dmachan->writer_handle[j]);
#endif
                dma_unmap_single(&s->dev->dev, dmachan->writer_handle[j], dmachan->bufferSize, DMA_FROM_DEVICE);
                kfree(dmachan->writer_addr[j]);
            }
        }
    }
    return 0;
}

static void ClearDMAWriterCounters(struct limelitepcie_dma_chan *dmachan)
{
    dmachan->writer_hw_count = 0;
    dmachan->writer_hw_count_last = 0;
    dmachan->writer_sw_count = 0;
}

static void ClearDMAReaderCounters(struct limelitepcie_dma_chan *dmachan)
{
    dmachan->reader_hw_count = 0;
    dmachan->reader_hw_count_last = 0;
    dmachan->reader_sw_count = 0;
}

static int limelitepcie_dma_writer_request(
    struct limelitepcie_device *s, struct limelitepcie_dma_chan *dmachan, dma_addr_t dmaBufAddr, uint32_t size, bool genIRQ)
{
    if (size == 0 || size > dmachan->bufferSize)
    {
        dev_err(&s->dev->dev, "DMA writer request: invalid size %i\n", size);
        return -EINVAL;
    }
    /* Fill DMA Writer descriptor, buffer size + parameters. */
    limelitepcie_writel(s,
        dmachan->base + PCIE_DMA_WRITER_TABLE_VALUE_OFFSET,
#ifndef DMA_BUFFER_ALIGNED
        DMA_LAST_DISABLE |
#endif
            (!genIRQ) * DMA_IRQ_DISABLE | /* generate an msi */
            size);
    /* Fill 32-bit Address LSB. */
    limelitepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_TABLE_VALUE_OFFSET + 4, (dmaBufAddr >> 0) & 0xffffffff);
    /* Write descriptor (and fill 32-bit Address MSB for 64-bit mode). */
    limelitepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_TABLE_WE_OFFSET, (dmaBufAddr >> 32) & 0xffffffff);
#ifdef DEBUG_READ
    dev_dbg(&s->dev->dev, "DMA writer request - buf:x%llx size:%i irq:%i\n", dmaBufAddr, size, genIRQ ? 1 : 0);
#endif
    return 0;
}

static int limelitepcie_dma_reader_request(
    struct limelitepcie_device *s, struct limelitepcie_dma_chan *dmachan, dma_addr_t dmaBufAddr, uint32_t size, bool genIRQ)
{
    if (size == 0 || size > dmachan->bufferSize)
    {
        dev_err(&s->dev->dev, "DMA reader request: invalid size %i\n", size);
        return -EINVAL;
    }
    /* Fill DMA Reader descriptor, buffer size + parameters. */
    limelitepcie_writel(s,
        dmachan->base + PCIE_DMA_READER_TABLE_VALUE_OFFSET,
#ifndef DMA_BUFFER_ALIGNED
        DMA_LAST_DISABLE |
#endif
            (!genIRQ) * DMA_IRQ_DISABLE | /* generate an msi */
            size);
    /* Fill 32-bit Address LSB. */
    limelitepcie_writel(s, dmachan->base + PCIE_DMA_READER_TABLE_VALUE_OFFSET + 4, (dmaBufAddr >> 0) & 0xffffffff);
    /* Write descriptor (and fill 32-bit Address MSB for 64-bit mode). */
    limelitepcie_writel(s, dmachan->base + PCIE_DMA_READER_TABLE_WE_OFFSET, (dmaBufAddr >> 32) & 0xffffffff);
#ifdef DEBUG_WRITE
    dev_dbg(&s->dev->dev, "DMA reader request - buf:x%llx size:%i irq:%i\n", dmaBufAddr, size, genIRQ ? 1 : 0);
#endif
    return 0;
}

static void limelitepcie_dma_writer_start(struct limelitepcie_device *s, int chan_num, uint32_t write_size, uint8_t irqFreq)
{
    struct limelitepcie_dma_chan *dmachan = &s->chan[chan_num].dma;
    if (write_size == 0 || write_size > dmachan->bufferSize)
    {
        dev_err(&s->dev->dev, "DMA writer start: invalid write size %i\n", write_size);
        return;
    }

    /* Fill DMA Writer descriptors. */
    limelitepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_ENABLE_OFFSET, 0);
    limelitepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_TABLE_FLUSH_OFFSET, 1);
    limelitepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_TABLE_LOOP_PROG_N_OFFSET, 0);
    for (int i = 0; i < dmachan->bufferCount; i++)
    {
        const bool genIRQ = (i % irqFreq == 0);
        if (limelitepcie_dma_writer_request(s, dmachan, dmachan->writer_handle[i], write_size, genIRQ) != 0)
        {
            dev_err(&s->dev->dev, "DMA%i writer start failed\n", chan_num);
            return;
        }
    }
    limelitepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_TABLE_LOOP_PROG_N_OFFSET, 1);
    ClearDMAWriterCounters(dmachan);

    /* Start DMA Writer. */
    limelitepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_ENABLE_OFFSET, 1);
    dev_info(&s->dev->dev, "Rx%i DMA writer start, buffer size: %u, IRQ every: %i buffers", chan_num, write_size, irqFreq);
}

static void limelitepcie_dma_writer_stop(struct limelitepcie_device *s, int chan_num)
{
    struct limelitepcie_dma_chan *dmachan = &s->chan[chan_num].dma;

    /* Flush and stop DMA Writer. */
    const int transfersDone = limelitepcie_readl(s, dmachan->base + PCIE_DMA_WRITER_TABLE_LOOP_STATUS_OFFSET);
    limelitepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_TABLE_LOOP_PROG_N_OFFSET, 0);
    limelitepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_TABLE_FLUSH_OFFSET, 1);
    udelay(1000);
    limelitepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_ENABLE_OFFSET, 0);
    limelitepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_TABLE_FLUSH_OFFSET, 1);

    dev_info(&s->dev->dev, "Rx%i DMA writer stop, DMA descriptors processed: %i\n", chan_num, transfersDone);
    ClearDMAWriterCounters(dmachan);
}

static void limelitepcie_dma_reader_start(struct limelitepcie_device *s, int chan_num)
{
    struct limelitepcie_dma_chan *dmachan = &s->chan[chan_num].dma;
    /* Fill DMA Reader descriptors. */
    limelitepcie_writel(s, dmachan->base + PCIE_DMA_READER_ENABLE_OFFSET, 0);
    limelitepcie_writel(s, dmachan->base + PCIE_DMA_READER_TABLE_FLUSH_OFFSET, 1);
    limelitepcie_writel(s, dmachan->base + PCIE_DMA_READER_TABLE_LOOP_PROG_N_OFFSET, 0);

    ClearDMAReaderCounters(dmachan);
    /* start dma reader */
    limelitepcie_writel(s, dmachan->base + PCIE_DMA_READER_ENABLE_OFFSET, 1);
    dev_info(&s->dev->dev, "Tx%i DMA reader start", chan_num);
}

static void limelitepcie_dma_reader_stop(struct limelitepcie_device *s, int chan_num)
{
    struct limelitepcie_dma_chan *dmachan = &s->chan[chan_num].dma;

    const uint32_t loop_status = limelitepcie_readl(s, dmachan->base + PCIE_DMA_READER_TABLE_LOOP_STATUS_OFFSET);
    /* flush and stop dma reader */
    limelitepcie_writel(s, dmachan->base + PCIE_DMA_READER_TABLE_LOOP_PROG_N_OFFSET, 0);
    limelitepcie_writel(s, dmachan->base + PCIE_DMA_READER_TABLE_FLUSH_OFFSET, 1);
    udelay(1000);
    limelitepcie_writel(s, dmachan->base + PCIE_DMA_READER_ENABLE_OFFSET, 0);
    limelitepcie_writel(s, dmachan->base + PCIE_DMA_READER_TABLE_FLUSH_OFFSET, 1);
    dev_info(&s->dev->dev, "Tx%i DMA reader stop, DMA descriptors processed: %i\n", chan_num, loop_status);
    ClearDMAReaderCounters(dmachan);
}

void limelitepcie_stop_dma(struct limelitepcie_device *s)
{
    struct limelitepcie_dma_chan *dmachan;
    for (int i = 0; i < s->channels; i++)
    {
        dmachan = &s->chan[i].dma;
        limelitepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_ENABLE_OFFSET, 0);
        limelitepcie_writel(s, dmachan->base + PCIE_DMA_READER_ENABLE_OFFSET, 0);
    }
}

static irqreturn_t limelitepcie_interrupt(int irq, void *data)
{
    struct limelitepcie_device *s = (struct limelitepcie_device *)data;
    uint32_t irq_enable = limelitepcie_readl(s, CSR_PCIE_MSI_ENABLE_ADDR);
    uint32_t irq_vector;
/* Single MSI */
#ifdef CSR_PCIE_MSI_CLEAR_ADDR
    irq_vector = limelitepcie_readl(s, CSR_PCIE_MSI_VECTOR_ADDR);
/* Multi-Vector MSI */
#else
    irq_vector = 0;
    for (i = 0; i < s->irqs; i++)
    {
        if (irq == pci_irq_vector(s->dev, i))
        {
            irq_vector = (1 << i);
            break;
        }
    }
#endif

#ifdef DEBUG_MSI
    dev_dbg(&s->dev->dev, "MSI: 0x%x 0x%x\n", irq_vector, irq_enable);
#endif
    irq_vector &= irq_enable;

    const bool forceWake = irq_vector == 0;
    if (irq_vector == 0)
        return IRQ_NONE;

    uint32_t clear_mask = 0;
    for (int i = 0; i < s->channels; i++)
    {
        struct limelitepcie_chan *chan = &s->chan[i];
        /* dma reader interrupt handling */
        if (irq_vector & (1 << chan->dma.reader_interrupt) && chan->dma.reader_enable)
        {
            uint32_t readTransfers = limelitepcie_readl(s, chan->dma.base + PCIE_DMA_READER_TABLE_LOOP_STATUS_OFFSET);
            chan->dma.reader_hw_count = readTransfers & 0xFFFF;
            const uint16_t pendingBuffersCount = (uint16_t)chan->dma.reader_sw_count - (uint16_t)chan->dma.reader_hw_count;
#ifdef DEBUG_MSI
            dev_dbg(&s->dev->dev,
                "MSI DMA%d Reader, sw:%lli hw:%lli (%i), dmaCnt:%08X\n",
                i,
                chan->dma.reader_sw_count,
                chan->dma.reader_hw_count,
                pendingBuffersCount,
                readTransfers);
#endif
            if (pendingBuffersCount < chan->dma.bufferCount - 32)
                wake_up_interruptible(&chan->wait_wr);
            if (!forceWake)
                clear_mask |= (1 << chan->dma.reader_interrupt);
        }
        /* dma writer interrupt handling */
        if ((irq_vector & (1 << chan->dma.writer_interrupt) || forceWake) && chan->dma.writer_enable)
        {
            uint32_t writeTransfers = limelitepcie_readl(s, chan->dma.base + PCIE_DMA_WRITER_TABLE_LOOP_STATUS_OFFSET);
            chan->dma.writer_hw_count = ((writeTransfers >> 8) | (writeTransfers & 0xFF)) & 0xFFFF;
            const uint16_t freeBuffers = (uint16_t)chan->dma.writer_hw_count - (uint16_t)chan->dma.writer_sw_count;
            if (freeBuffers > 0 || forceWake)
            {
#ifdef DEBUG_MSI
                if (freeBuffers >= 255)
                    dev_dbg(&s->dev->dev,
                        "MSI DMA%d Writer sw:%lli hw:%lli, (%i)\n",
                        i,
                        chan->dma.writer_sw_count,
                        chan->dma.writer_hw_count,
                        freeBuffers);
#endif
                wake_up_interruptible(&chan->wait_rd);
            }
            if (!forceWake)
                clear_mask |= (1 << chan->dma.writer_interrupt);
        }
    }

#ifdef CSR_PCIE_MSI_CLEAR_ADDR
    if (clear_mask != 0)
        limelitepcie_writel(s, CSR_PCIE_MSI_CLEAR_ADDR, clear_mask);
#endif
    return IRQ_HANDLED;
}

static int limelitepcie_open(struct inode *inode, struct file *file)
{
    struct limelitepcie_chan *chan = container_of(inode->i_cdev, struct limelitepcie_chan, cdev);
    struct limelitepcie_chan_priv *chan_priv = kzalloc(sizeof(*chan_priv), GFP_KERNEL);
    pr_info("Open %s\n", file->f_path.dentry->d_iname);

    if (!chan_priv)
        return -ENOMEM;

    chan_priv->chan = chan;
    file->private_data = chan_priv;

    if (chan->dma.reader_enable == 0)
    { /* clear only if disabled */
        ClearDMAReaderCounters(&chan->dma);
    }

    if (chan->dma.writer_enable == 0)
    { /* clear only if disabled */
        ClearDMAWriterCounters(&chan->dma);
    }
    return 0;
}

static int limelitepcie_release(struct inode *inode, struct file *file)
{
    struct limelitepcie_chan_priv *chan_priv = file->private_data;
    struct limelitepcie_chan *chan = chan_priv->chan;
    pr_info("Release %s\n", file->f_path.dentry->d_iname);

    if (chan_priv->reader)
    {
        /* disable interrupt */
        limelitepcie_disable_interrupt(chan->limelitepcie_dev, chan->dma.reader_interrupt);
        /* disable DMA */
        limelitepcie_dma_reader_stop(chan->limelitepcie_dev, chan->index);
        chan->dma.reader_lock = 0;
        chan->dma.reader_enable = 0;
        wake_up_interruptible(&chan->wait_wr);
    }

    if (chan_priv->writer)
    {
        /* disable interrupt */
        limelitepcie_disable_interrupt(chan->limelitepcie_dev, chan->dma.writer_interrupt);
        /* disable DMA */
        limelitepcie_dma_writer_stop(chan->limelitepcie_dev, chan->index);
        chan->dma.writer_lock = 0;
        chan->dma.writer_enable = 0;
        wake_up_interruptible(&chan->wait_rd);
    }
    kfree(chan_priv);
    return 0;
}

static ssize_t limelitepcie_read(struct file *file, char __user *data, size_t size, loff_t *offset)
{
    struct limelitepcie_chan_priv *chan_priv = file->private_data;
    struct limelitepcie_chan *chan = chan_priv->chan;
    struct limelitepcie_device *s = chan->limelitepcie_dev;
    struct limelitepcie_dma_chan *dmachan = &chan->dma;
    int bufCount = 0;
    const size_t maxTransferSize = dmachan->bufferSize;

#ifdef DEBUG_READ
    dev_dbg(&s->dev->dev, "read(%ld)\n", size);
#endif

    // If DMA is not yet active, fill table with requests and start transfers
    if (!dmachan->writer_enable)
    {
        const int totalBufferSize = dmachan->bufferCount * dmachan->bufferSize;
        if (size > totalBufferSize)
        {
            dev_err(&s->dev->dev, "Read too large (%ld), max DMA buffer size (%i)\n", size, totalBufferSize);
            return -EINVAL;
        }

        // Fill DMA transactions table
        limelitepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_ENABLE_OFFSET, 0);
        limelitepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_TABLE_FLUSH_OFFSET, 1);
        limelitepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_TABLE_LOOP_PROG_N_OFFSET, 0); // don't loop table
        limelitepcie_enable_interrupt(chan->limelitepcie_dev, chan->dma.writer_interrupt);

        size_t bytesRemaining = size;
        while (bytesRemaining > 0)
        {
            const int requestSize = min(bytesRemaining, maxTransferSize);
            bytesRemaining -= requestSize;
            bool genIRQ = bytesRemaining == 0; // generate IRQ on last transfer
            int status = limelitepcie_dma_writer_request(s, dmachan, dmachan->writer_handle[bufCount], requestSize, genIRQ);
            if (status != 0)
                return status;
            ++bufCount;
        }

        ClearDMAWriterCounters(dmachan);
        //litepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_TABLE_LOOP_PROG_N_OFFSET, 1); // don't loop table
        limelitepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_ENABLE_OFFSET, 1); // start DMA writing
        //dev_dbg(&s->dev->dev, "DMA writer enable for %i buffers\n", bufCount);
        dmachan->writer_enable = true;
    }

    if (file->f_flags & O_NONBLOCK) // TODO
    {
        if (chan->dma.writer_sw_count <= chan->dma.writer_hw_count)
            return -EAGAIN;
    }
    else
    {
        int ret = wait_event_interruptible(chan->wait_rd, (dmachan->writer_hw_count - dmachan->writer_sw_count) == bufCount);
        if (ret < 0)
            return ret;
    }

    int bytesReceived = 0;
    for (int i = 0; i < bufCount; ++i)
    {
        const int bufSize = min(size - bytesReceived, maxTransferSize);
        if (copy_to_user(data + bytesReceived, dmachan->writer_addr[i], bufSize))
            return -EFAULT;
        bytesReceived += bufSize;
        ++dmachan->writer_sw_count;
    }
    limelitepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_ENABLE_OFFSET, 0);
    limelitepcie_disable_interrupt(chan->limelitepcie_dev, chan->dma.writer_interrupt);
    dmachan->writer_enable = false; // TODO: implement for non blocking

#ifdef DEBUG_READ
    dev_dbg(&s->dev->dev, "read %i/%ld bytes\n", bytesReceived, size);
#endif
    return bytesReceived;
}

static ssize_t submiteWrite(struct file *file, size_t bufSize, bool genIRQ)
{
    int ret;
    struct limelitepcie_chan_priv *chan_priv = file->private_data;
    struct limelitepcie_chan *chan = chan_priv->chan;
    struct limelitepcie_device *s = chan->limelitepcie_dev;

    if (bufSize > chan->dma.bufferSize || bufSize <= 0)
    {
        dev_dbg(&s->dev->dev, "DMA writer invalid bufSize(%li) DMA_BUFFER_SIZE(%i)", bufSize, chan->dma.bufferSize);
        return -EINVAL;
    }

    chan->dma.reader_hw_count = limelitepcie_readl(s, chan->dma.base + PCIE_DMA_READER_TABLE_LOOP_STATUS_OFFSET) & 0xFFFF;

    const uint8_t pendingBuffers = chan->dma.reader_sw_count - chan->dma.reader_hw_count;
    const uint8_t maxDMApending = chan->dma.bufferCount - 1;
    const bool canSubmit = pendingBuffers < maxDMApending;

    if (file->f_flags & O_NONBLOCK)
    {
        if (!canSubmit)
        {
#ifdef DEBUG_WRITE
            dev_dbg(&s->dev->dev, "submitWrite: failed, %u buffers already pending", pendingBuffers);
#endif
            ret = -EAGAIN;
        }
        else
            ret = 0;
    }
    else
    {
        if (!canSubmit)
            ret = wait_event_interruptible(
                chan->wait_wr, (uint16_t)chan->dma.reader_sw_count - (uint16_t)chan->dma.reader_hw_count < maxDMApending);
        else
            ret = 0;
    }

    if (ret < 0)
        return ret;

    struct limelitepcie_dma_chan *dmachan = &chan->dma;
    const int bufIndex = chan->dma.reader_sw_count % chan->dma.bufferCount;
    ++chan->dma.reader_sw_count;
    ret = limelitepcie_dma_reader_request(s, dmachan, dmachan->reader_handle[bufIndex], bufSize, genIRQ);
    if (ret != 0)
        return ret;

    chan->dma.reader_sw_count &= 0xFFFF;
    return 0;
}

static ssize_t limelitepcie_write(struct file *file, const char __user *data, size_t size, loff_t *offset)
{
    struct limelitepcie_chan_priv *chan_priv = file->private_data;
    struct limelitepcie_chan *chan = chan_priv->chan;
    struct limelitepcie_device *s = chan->limelitepcie_dev;
    struct limelitepcie_dma_chan *dmachan = &chan->dma;
    const int maxTransferSize = dmachan->bufferSize;
#ifdef DEBUG_WRITE
    dev_dbg(&s->dev->dev, "write(%ld)\n", size);
#endif

    if (!dmachan->reader_enable)
    {
        const int totalBufferSize = dmachan->bufferCount * dmachan->bufferSize;
        if (size > totalBufferSize)
        {
            dev_err(&s->dev->dev, "Write too large (%ld), max DMA buffer size (%i)\n", size, totalBufferSize);
            return -EINVAL;
        }

        // Fill DMA transactions table
        limelitepcie_writel(s, dmachan->base + PCIE_DMA_READER_ENABLE_OFFSET, 0);
        limelitepcie_writel(s, dmachan->base + PCIE_DMA_READER_TABLE_FLUSH_OFFSET, 1);
        limelitepcie_writel(s, dmachan->base + PCIE_DMA_READER_TABLE_LOOP_PROG_N_OFFSET, 0); // don't loop table
        limelitepcie_enable_interrupt(s, dmachan->reader_interrupt);

        /* Clear counters. */
        ClearDMAReaderCounters(dmachan);

        int bytesRemaining = size;
        int bufCount = 0;
        while (bytesRemaining > 0)
        {
            const int requestSize = min(bytesRemaining, maxTransferSize);
            bytesRemaining -= requestSize;
            bool genIRQ = bytesRemaining == 0; // generate IRQ on last transfer

            int ret = copy_from_user(dmachan->reader_addr[bufCount], data + maxTransferSize * bufCount, requestSize);
            if (ret)
                return -EFAULT;
            int status = limelitepcie_dma_reader_request(s, dmachan, dmachan->reader_handle[bufCount], requestSize, genIRQ);
            if (status != 0)
            {
                dev_dbg(&s->dev->dev, "DMA reader request failed %i\n", status);
                return status;
            }
            ++bufCount;
        }

        limelitepcie_writel(s, dmachan->base + PCIE_DMA_READER_ENABLE_OFFSET, 1); // start DMA reading
        dev_dbg(&s->dev->dev, "DMA reader enable for %i buffers\n", bufCount);
        dmachan->reader_enable = true;
        ++dmachan->reader_sw_count;
    }

    int ret = wait_event_interruptible(chan->wait_wr, dmachan->reader_sw_count - dmachan->reader_hw_count == 0);
    if (ret < 0)
        return ret;
    limelitepcie_writel(s, dmachan->base + PCIE_DMA_READER_ENABLE_OFFSET, 0);
    limelitepcie_disable_interrupt(s, dmachan->reader_interrupt);
    dmachan->reader_enable = false;
#ifdef DEBUG_WRITE
    dev_dbg(&s->dev->dev, "write(%ld) done\n", size);
#endif
    return size;
}

static int limelitepcie_mmap(struct file *file, struct vm_area_struct *vma)
{
    struct limelitepcie_chan_priv *chan_priv = file->private_data;
    struct limelitepcie_chan *chan = chan_priv->chan;
    struct limelitepcie_device *s = chan->limelitepcie_dev;

    unsigned long pfn;
    int is_tx, i;

    const int totalBufferSize = chan->dma.bufferCount * chan->dma.bufferSize;
    if (vma->vm_end - vma->vm_start != totalBufferSize)
    {
        dev_err(&s->dev->dev, "vma->vm_end - vma->vm_start != %i, got: %lu\n", totalBufferSize, vma->vm_end - vma->vm_start);
        return -EINVAL;
    }

    if (vma->vm_pgoff == 0)
        is_tx = 1;
    else if (vma->vm_pgoff == (totalBufferSize >> PAGE_SHIFT))
        is_tx = 0;
    else
    {
        dev_err(&s->dev->dev, "mmap page offset bad\n");
        return -EINVAL;
    }

    for (i = 0; i < chan->dma.bufferCount; i++)
    {
        void *va;
        if (is_tx)
            va = chan->dma.reader_addr[i];
        else
            va = chan->dma.writer_addr[i];
        pfn = virt_to_phys((void *)va) >> PAGE_SHIFT;
        //		vma->vm_flags |= VM_DONTDUMP | VM_DONTEXPAND;
        /*
		 * Note: the memory is cached, so the user must explicitly
		 * flush the CPU caches on architectures which require it.
		 */

        size_t usrPtr = vma->vm_start + i * chan->dma.bufferSize;
        int remapRet = remap_pfn_range(vma, usrPtr, pfn, chan->dma.bufferSize, vma->vm_page_prot);
        if (remapRet)
        {
            dev_err(&s->dev->dev, "mmap io_remap_pfn_range failed %i\n", remapRet);
            return -EAGAIN;
        }
    }

    return 0;
}

static unsigned int limelitepcie_poll(struct file *file, poll_table *wait)
{
    struct limelitepcie_chan_priv *chan_priv = file->private_data;
    struct limelitepcie_chan *chan = chan_priv->chan;

    poll_wait(file, &chan->wait_rd, wait);
    poll_wait(file, &chan->wait_wr, wait);

    unsigned int mask = 0;
    uint16_t rxPending = (uint16_t)chan->dma.writer_hw_count - (uint16_t)chan->dma.writer_sw_count;
    if (rxPending > 0)
        mask |= POLLIN | POLLRDNORM;

    uint16_t txPending = (uint16_t)chan->dma.reader_sw_count - (uint16_t)chan->dma.reader_hw_count;
    if (txPending < chan->dma.bufferCount * 3 / 4)
        mask |= POLLOUT | POLLWRNORM;

#ifdef DEBUG_POLL
    dev_dbg(&dev->dev->dev,
        "poll: writer sw: %10lld / hw: %10lld | reader sw: %10lld / hw: %10lld, IN:%i, OUT:%i\n",
        chan->dma.writer_sw_count,
        chan->dma.writer_hw_count,
        chan->dma.reader_sw_count,
        chan->dma.reader_hw_count,
        mask & POLLIN ? 1 : 0,
        mask & POLLOUT ? 1 : 0);
#endif

    return mask;
}

#ifdef CSR_FLASH_BASE
/* SPI */

    #define SPI_TIMEOUT 100000 /* in us */

static int limelitepcie_flash_spi(struct limelitepcie_device *s, struct limelitepcie_ioctl_flash *m)
{
    int i;

    if (m->tx_len < 8 || m->tx_len > 40)
        return -EINVAL;

    limelitepcie_writel(s, CSR_FLASH_SPI_MOSI_ADDR, m->tx_data >> 32);
    limelitepcie_writel(s, CSR_FLASH_SPI_MOSI_ADDR + 4, m->tx_data);
    limelitepcie_writel(s, CSR_FLASH_SPI_CONTROL_ADDR, SPI_CTRL_START | (m->tx_len * SPI_CTRL_LENGTH));
    udelay(16);
    for (i = 0; i < SPI_TIMEOUT; i++)
    {
        if (limelitepcie_readl(s, CSR_FLASH_SPI_STATUS_ADDR) & SPI_STATUS_DONE)
            break;
        udelay(1);
    }
    m->rx_data =
        ((uint64_t)limelitepcie_readl(s, CSR_FLASH_SPI_MISO_ADDR) << 32) | limelitepcie_readl(s, CSR_FLASH_SPI_MISO_ADDR + 4);
    return 0;
}
#endif

static long limelitepcie_ioctl_control(struct file *file, unsigned int cmd, unsigned long arg)
{
    long ret = 0;
    struct limelitepcie_device *dev = file->private_data;

    switch (cmd)
    {
    case LIMELITEPCIE_IOCTL_RUN_CONTROL_COMMAND: {
        struct semaphore *sem = &(dev->control_semaphore);
        struct limelitepcie_control_packet m;

        if (copy_from_user(&m, (void *)arg, sizeof(m)))
        {
            ret = -EFAULT;
            break;
        }

        uint64_t end_time = ktime_get_raw_ns() + m.timeout_ms * 1000000llu;

        int success = down_timeout(sem, msecs_to_jiffies(m.timeout_ms));
        if (success != 0) // on failure
        {
            ret = -EBUSY;
            break;
        }

        uint32_t byteCount = min(m.length, (uint32_t)(CSR_CNTRL_CNTRL_SIZE * sizeof(uint32_t)));
        uint32_t value;
        for (int i = 0; i < byteCount; i += sizeof(value))
        {
            memcpy(&value, m.request + i, sizeof(value));
            limelitepcie_writel(dev, CSR_CNTRL_BASE + i, value);
        }

        success = false;
        while (ktime_get_raw_ns() < end_time)
        {
            uint32_t status = limelitepcie_readl(dev, CSR_CNTRL_BASE);
            if ((status & 0xFF00) != 0)
            {
                success = true;
                break;
            }
        }

        if (!success)
        {
            up(sem);
            ret = -EFAULT;
            break;
        }

        for (int i = 0; i < byteCount; i += sizeof(uint32_t))
        {
            value = limelitepcie_readl(dev, CSR_CNTRL_BASE + i);
            memcpy(m.response + i, &value, sizeof(uint32_t));
        }

        if (copy_to_user((void *)arg, &m, sizeof(m)))
        {
            up(sem);
            ret = -EFAULT;
            break;
        }

        up(sem);
    }
    break;
    case LIMELITEPCIE_IOCTL_VERSION: {
        struct limelitepcie_version m;

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

static long limelitepcie_ioctl_trx(struct file *file, unsigned int cmd, unsigned long arg)
{
    long ret = 0;

    struct limelitepcie_chan_priv *chan_priv = file->private_data;
    struct limelitepcie_chan *chan = chan_priv->chan;
    struct limelitepcie_device *dev = chan->limelitepcie_dev;

    switch (cmd)
    {
    case LIMELITEPCIE_IOCTL_VERSION: {
        struct limelitepcie_version m;

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
    case LIMELITEPCIE_IOCTL_DMA_WRITER: {
        struct limelitepcie_ioctl_dma_writer m;

        if (copy_from_user(&m, (void *)arg, sizeof(m)))
        {
            ret = -EFAULT;
            break;
        }

        if (m.enable != chan->dma.writer_enable)
        {
            /* enable / disable DMA */
            if (m.enable)
            {
                if (m.write_size > chan->dma.bufferSize || m.write_size <= 0)
                {
                    dev_info(&dev->dev->dev, "%s DMA writer invalid write size %i\n", file->f_path.dentry->d_iname, m.write_size);
                    ret = -EINVAL;
                    break;
                }
                limelitepcie_dma_writer_start(chan->limelitepcie_dev, chan->index, m.write_size, m.irqFreq);
                limelitepcie_enable_interrupt(chan->limelitepcie_dev, chan->dma.writer_interrupt);
            }
            else
            {
                //dev_info(&dev->dev->dev, "%s DMA writer disable\n", file->f_path.dentry->d_iname);
                limelitepcie_disable_interrupt(chan->limelitepcie_dev, chan->dma.writer_interrupt);
                limelitepcie_dma_writer_stop(chan->limelitepcie_dev, chan->index);
            }
        }

        chan->dma.writer_enable = m.enable;

        m.hw_count = chan->dma.writer_hw_count & 0XFFFF;
        m.sw_count = chan->dma.writer_sw_count & 0XFFFF;

        if (copy_to_user((void *)arg, &m, sizeof(m)))
        {
            ret = -EFAULT;
            break;
        }
    }
    break;
    case LIMELITEPCIE_IOCTL_DMA_READER: {
        struct limelitepcie_ioctl_dma_reader m;

        if (copy_from_user(&m, (void *)arg, sizeof(m)))
        {
            ret = -EFAULT;
            break;
        }

        if (m.enable != chan->dma.reader_enable)
        {
            /* enable / disable DMA */
            if (m.enable)
            {
                chan->dma.reader_irqFreq = m.irqFreq;
                //dev_info(&dev->dev->dev, "%s DMA reader enable\n", file->f_path.dentry->d_iname);
                limelitepcie_dma_reader_start(chan->limelitepcie_dev, chan->index);
                limelitepcie_enable_interrupt(chan->limelitepcie_dev, chan->dma.reader_interrupt);
            }
            else
            {
                //dev_info(&dev->dev->dev, "%s DMA reader disable\n", file->f_path.dentry->d_iname);
                limelitepcie_disable_interrupt(chan->limelitepcie_dev, chan->dma.reader_interrupt);
                limelitepcie_dma_reader_stop(chan->limelitepcie_dev, chan->index);
            }
        }

        chan->dma.reader_enable = m.enable;

        uint32_t readTransfers = limelitepcie_readl(dev, chan->dma.base + PCIE_DMA_READER_TABLE_LOOP_STATUS_OFFSET);
        m.hw_count = readTransfers & 0xFFFF;
        m.sw_count = chan->dma.reader_sw_count & 0xFFFF;

        if (copy_to_user((void *)arg, &m, sizeof(m)))
        {
            ret = -EFAULT;
            break;
        }
    }
    break;
    case LIMELITEPCIE_IOCTL_MMAP_DMA_INFO: {
        struct limelitepcie_ioctl_mmap_dma_info m;

        m.dma_tx_buf_offset = 0;
        m.dma_tx_buf_size = chan->dma.bufferSize;
        m.dma_tx_buf_count = chan->dma.bufferCount;

        m.dma_rx_buf_offset = chan->dma.bufferCount * chan->dma.bufferSize;
        m.dma_rx_buf_size = chan->dma.bufferSize;
        m.dma_rx_buf_count = chan->dma.bufferCount;

        if (copy_to_user((void *)arg, &m, sizeof(m)))
        {
            ret = -EFAULT;
            break;
        }
    }
    break;
    case LIMELITEPCIE_IOCTL_MMAP_DMA_WRITER_UPDATE: {
        struct limelitepcie_ioctl_mmap_dma_update m;

        if (copy_from_user(&m, (void *)arg, sizeof(m)))
        {
            ret = -EFAULT;
            break;
        }

        chan->dma.writer_sw_count = m.sw_count & 0xFFFF;
    }
    break;
    case LIMELITEPCIE_IOCTL_MMAP_DMA_READER_UPDATE: {
        struct limelitepcie_ioctl_mmap_dma_update m;

        if (copy_from_user(&m, (void *)arg, sizeof(m)))
        {
            dev_dbg(&chan->limelitepcie_dev->dev->dev, "LITEPCIE_IOCTL_MMAP_DMA_READER_UPDATE copy_from_user fail");
            ret = -EFAULT;
            break;
        }

        chan->dma.reader_sw_count = m.sw_count & 0xFFFF;

        ret = submiteWrite(file, m.buffer_size, m.genIRQ);
    }
    break;

    case LIMELITEPCIE_IOCTL_CACHE_FLUSH: {
        struct limelitepcie_cache_flush m;

        if (copy_from_user(&m, (void *)arg, sizeof(m)))
        {
            dev_dbg(&chan->limelitepcie_dev->dev->dev, "LITEPCIE_IOCTL_CACHE_FLUSH copy_from_user fail");
            ret = -EFAULT;
            break;
        }

        int index = m.bufferIndex & 0xFF;
        if (m.isTx)
        {
            if (m.toDevice)
                dma_sync_single_for_device(&dev->dev->dev, chan->dma.reader_handle[index], chan->dma.bufferSize, DMA_TO_DEVICE);
            else
                dma_sync_single_for_cpu(&dev->dev->dev, chan->dma.reader_handle[index], chan->dma.bufferSize, DMA_TO_DEVICE);
        }
        else
        {
            if (m.toDevice)
                dma_sync_single_for_device(&dev->dev->dev, chan->dma.writer_handle[index], chan->dma.bufferSize, DMA_FROM_DEVICE);
            else
                dma_sync_single_for_cpu(&dev->dev->dev, chan->dma.writer_handle[index], chan->dma.bufferSize, DMA_FROM_DEVICE);
        }
        ret = 0;
    }
    break;
    case LIMELITEPCIE_IOCTL_LOCK: {
        struct limelitepcie_ioctl_lock m;

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
        }
    }
    break;
    default:
        ret = -ENOIOCTLCMD;
        break;
    }
    return ret;
}

static int limelitepcie_ctrl_open(struct inode *inode, struct file *file)
{
    pr_info("Open %s\n", file->f_path.dentry->d_iname);
    struct limelitepcie_device *ctrlDevice = container_of(inode->i_cdev, struct limelitepcie_device, control_cdev);
    file->private_data = ctrlDevice;

    limelitepcie_writel(ctrlDevice, CSR_CNTRL_TEST_ADDR, 0x55);
    if (limelitepcie_readl(ctrlDevice, CSR_CNTRL_TEST_ADDR) != 0x55)
    {
        printk(KERN_ERR LIMELITEPCIE_NAME " CSR register test failed\n");
        return -EIO;
    }
    return 0;
}

static int limelitepcie_ctrl_release(struct inode *inode, struct file *file)
{
    pr_info("Release %s\n", file->f_path.dentry->d_iname);
    return 0;
}

static ssize_t limelitepcie_ctrl_write(struct file *file, const char __user *userbuf, size_t count, loff_t *offset)
{
    struct limelitepcie_device *s = file->private_data;
    uint32_t value;
    count = min(count, CSR_CNTRL_CNTRL_SIZE * sizeof(uint32_t));
    for (int i = 0; i < count; i += sizeof(value))
    {
        if (copy_from_user(&value, userbuf + i, sizeof(value)))
            return -EFAULT;
        limelitepcie_writel(s, CSR_CNTRL_BASE + i, value);
    }
    return count;
}

static ssize_t limelitepcie_ctrl_read(struct file *file, char __user *userbuf, size_t count, loff_t *offset)
{
    struct limelitepcie_device *s = file->private_data;
    uint32_t value;
    count = min(count, CSR_CNTRL_CNTRL_SIZE * sizeof(uint32_t));
    for (int i = 0; i < count; i += sizeof(value))
    {
        value = limelitepcie_readl(s, CSR_CNTRL_BASE + i);
        if (copy_to_user(userbuf + i, &value, sizeof(value)))
            return -EFAULT;
    }
    return count;
}

static pci_ers_result_t limelitepcie_error_detected(struct pci_dev *dev, pci_channel_state_t state)
{
    switch (state)
    {
    case pci_channel_io_normal:
        dev_err(&dev->dev, "PCI error_detected. Channel state(normal)\n");
        break;
    case pci_channel_io_frozen:
        dev_err(&dev->dev, "PCI error_detected. Channel state(frozen)\n");
        break;
    case pci_channel_io_perm_failure:
        dev_err(&dev->dev, "PCI error_detected. Channel state(dead)\n");
        break;
    default:
        dev_err(&dev->dev, "PCI error_detected\n");
    }
    return PCI_ERS_RESULT_NONE;
}
// int (*mmio_enabled)(struct pci_dev *dev);
static pci_ers_result_t limelitepcie_slot_reset(struct pci_dev *dev)
{
    dev_err(&dev->dev, "PCI slot reset\n");
    return PCI_ERS_RESULT_NONE;
}
// void (*resume)(struct pci_dev *dev);

static const struct pci_error_handlers pci_error_handlers_ops = {
    .error_detected = limelitepcie_error_detected,
    // .mmio_enabled = mmio_enabled,
    .slot_reset = limelitepcie_slot_reset,
    // .resume = resume
};

static const struct file_operations limelitepcie_fops = {
    .owner = THIS_MODULE,
    .unlocked_ioctl = limelitepcie_ioctl_trx,
    .open = limelitepcie_open,
    .release = limelitepcie_release,
    .read = limelitepcie_read,
    .poll = limelitepcie_poll,
    .write = limelitepcie_write,
    .mmap = limelitepcie_mmap,
};

static const struct file_operations limelitepcie_fops_control = {
    .owner = THIS_MODULE,
    .unlocked_ioctl = limelitepcie_ioctl_control,
    .open = limelitepcie_ctrl_open,
    .release = limelitepcie_ctrl_release,
    .read = limelitepcie_ctrl_read,
    .write = limelitepcie_ctrl_write,
};

static ssize_t product_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct limelitepcie_device *limeDev = dev_get_drvdata(dev);
    return snprintf(buf, PAGE_SIZE, "0x%4x\n", limeDev->attr.product);
}

static ssize_t vendor_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct limelitepcie_device *limeDev = dev_get_drvdata(dev);
    return snprintf(buf, PAGE_SIZE, "0x%4x\n", limeDev->attr.vendor);
}

static DEVICE_ATTR(product, 0444, product_show, NULL);
static DEVICE_ATTR(vendor, 0444, vendor_show, NULL);

static int limelitepcie_alloc_chdev(struct limelitepcie_device *s, struct device *parent)
{
    int ret;

    int index = limelitepcie_minor_idx;
    char deviceName[64];
    static int suffix = 0;
    snprintf(deviceName, sizeof(deviceName) - 1, "%s%i", "limelitepcie", suffix++);

    s->minor_base = limelitepcie_minor_idx;
    for (int i = 0; i < s->channels; i++)
    {
        cdev_init(&s->chan[i].cdev, &limelitepcie_fops);
        ret = cdev_add(&s->chan[i].cdev, MKDEV(limelitepcie_major, index), 1);
        if (ret < 0)
        {
            dev_err(&s->dev->dev, "Failed to allocate cdev\n");
            goto fail_alloc;
        }
        index++;
    }

    // alloc control
    cdev_init(&s->control_cdev, &limelitepcie_fops_control);
    ret = cdev_add(&s->control_cdev, MKDEV(limelitepcie_major, index), 1);
    if (ret < 0)
    {
        dev_err(&s->dev->dev, "Failed to allocate control cdev\n");
        goto fail_alloc_control;
    }

    index = limelitepcie_minor_idx;
    for (int i = 0; i < s->channels; i++)
    {
        char trxName[64];
        snprintf(trxName, sizeof(trxName) - 1, "trx%d", i);
        // when creating device linux replaces all '/' with a '!', so just use '!' directly
        struct device *trxDev =
            device_create(limelitepcie_class, parent, MKDEV(limelitepcie_major, index), NULL, "%s!%s", deviceName, trxName);
        if (!trxDev)
        {
            ret = -EINVAL;
            dev_err(&s->dev->dev, "Failed to create device\n");
            goto fail_create;
        }

        device_create_file(trxDev, &dev_attr_product);
        device_create_file(trxDev, &dev_attr_vendor);

        index++;
    }

    // additionally alloc control channel
    dev_info(&s->dev->dev, "Creating /dev/%s/control0\n", deviceName);
    // // when creating device linux replaces all '/' with a '!', so just use '!' directly

    struct device *ctrlDev =
        device_create(limelitepcie_class, parent, MKDEV(limelitepcie_major, index), s, "%s!control0", deviceName);
    if (!ctrlDev)
    {
        ret = -EINVAL;
        dev_err(&s->dev->dev, "Failed to create device\n");
        goto fail_create;
    }

    device_create_file(ctrlDev, &dev_attr_product);
    device_create_file(ctrlDev, &dev_attr_vendor);

    ++index;

    limelitepcie_minor_idx = index;
    return 0;

fail_create:
    while (index >= limelitepcie_minor_idx)
        device_destroy(limelitepcie_class, MKDEV(limelitepcie_major, index--));

fail_alloc_control:
    cdev_del(&s->control_cdev);

fail_alloc:
    for (int i = 0; i < s->channels; i++)
        cdev_del(&s->chan[i].cdev);

    return ret;
}

static void limelitepcie_free_chdev(struct limelitepcie_device *s)
{
    device_destroy(limelitepcie_class, MKDEV(limelitepcie_major, s->minor_base + s->channels));
    cdev_del(&s->control_cdev);
    for (int i = 0; i < s->channels; i++)
    {
        device_destroy(limelitepcie_class, MKDEV(limelitepcie_major, s->minor_base + i));
        cdev_del(&s->chan[i].cdev);
    }
}

/* from stackoverflow */
void sfind(char *string, char *format, ...)
{
    va_list arglist;

    va_start(arglist, format);
    vsscanf(string, format, arglist);
    va_end(arglist);
}

struct revision {
    int yy;
    int mm;
    int dd;
};

int compare_revisions(struct revision d1, struct revision d2)
{
    if (d1.yy < d2.yy)
        return -1;
    else if (d1.yy > d2.yy)
        return 1;

    if (d1.mm < d2.mm)
        return -1;
    else if (d1.mm > d2.mm)
        return 1;
    else if (d1.dd < d2.dd)
        return -1;
    else if (d1.dd > d2.dd)
        return 1;

    return 0;
}
/* from stackoverflow */

void FreeIRQs(struct limelitepcie_device *device)
{
    dev_info(&device->dev->dev, "FreeIRQs %i\n", device->irqs);
    if (device->irqs <= 0)
        return;

    for (int i = device->irqs - 1; i >= 0; --i)
    {
        int irq = pci_irq_vector(device->dev, i);
        free_irq(irq, device);
    }
    device->irqs = 0;
    pci_free_irq_vectors(device->dev);
}

int AllocateIRQs(struct limelitepcie_device *device)
{
    struct pci_dev *dev = device->dev;
    int irqs = pci_alloc_irq_vectors(dev, 1, 32, PCI_IRQ_MSI);
    if (irqs < 0)
    {
        dev_err(&dev->dev, "Failed to enable MSI\n");
        return irqs;
    }
    dev_info(&dev->dev, "%d MSI IRQs allocated.\n", irqs);

    device->irqs = 0;
    for (int i = 0; i < irqs; i++)
    {
        int irq = pci_irq_vector(dev, i);
        int ret = request_irq(irq, limelitepcie_interrupt, IRQF_SHARED, LIMELITEPCIE_NAME, device);
        if (ret < 0)
        {
            dev_err(&dev->dev, " Failed to allocate IRQ %d\n", dev->irq);
            FreeIRQs(device);
            return ret;
        }
        device->irqs += 1;
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

static int ReadInfo(struct limelitepcie_device *s)
{
    memset(&s->info, 0, sizeof(s->info));

    struct LMS64CPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.cmd = 0x00; // GET_INFO
    uint8_t *buffer = (uint8_t *)&pkt;
    int count = 64;
    uint32_t value;
    for (int i = 0; i < count; i += sizeof(uint32_t))
    {
        memcpy(&value, buffer + i, sizeof(value));
        limelitepcie_writel(s, CSR_CNTRL_BASE + i, value);
    }

    for (int n = 0; n < 10; ++n)
    {
        value = limelitepcie_readl(s, CSR_CNTRL_BASE);
        memcpy(buffer, &value, sizeof(value));
        if (pkt.status != 0)
            break;
        mdelay(10); // wait between reads until MCU gives answer
    }
    if (pkt.status == 0)
        return -EIO;

    for (int i = 0; i < count; i += sizeof(uint32_t))
    {
        value = limelitepcie_readl(s, CSR_CNTRL_BASE + i);
        memcpy(buffer + i, &value, sizeof(value));
    }

    s->info.firmware = (pkt.payload[9] << 16) | pkt.payload[0];
    s->info.boardId = pkt.payload[1];
    s->info.protocol = pkt.payload[2];
    s->info.hardwareVer = pkt.payload[3];
    uint64_t serialNumber = 0;
    for (int i = 10; i < 18; ++i)
    {
        serialNumber <<= 8;
        serialNumber |= pkt.payload[i];
    }
    s->info.serialNumber = serialNumber;

    sprintf(s->info.devName, "%s", GetDeviceName(s->info.boardId));

    dev_info(&s->dev->dev,
        "[device info] %s FW:%u HW:%u PROTOCOL:%u S/N:0x%016llX \n",
        s->info.devName,
        s->info.firmware,
        s->info.hardwareVer,
        s->info.protocol,
        s->info.serialNumber);
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

static void ParseIdentifier(const char *identifier, char *destination, const size_t dstSize)
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

static struct mutex probe_lock;

int PCIeSpeedToGen(int speed)
{
    switch (speed)
    {
    case PCIE_SPEED_2_5GT:
        return 1;
    case PCIE_SPEED_5_0GT:
        return 2;
    case PCIE_SPEED_8_0GT:
        return 3;
    case PCIE_SPEED_16_0GT:
        return 4;
    case PCIE_SPEED_32_0GT:
        return 5;
    case PCIE_SPEED_64_0GT:
        return 6;
    }
    return 0;
}

static void pcie_max_link_cap(struct pci_dev *dev, enum pci_bus_speed *speed, enum pcie_link_width *width)
{
    *speed = dev->bus->max_bus_speed;
    *width = pcie_get_width_cap(dev);
}

static void pcie_cur_link_sta(struct pci_dev *dev, enum pci_bus_speed *speed, enum pcie_link_width *width)
{
    uint16_t lnksta;
    pcie_capability_read_word(dev, PCI_EXP_LNKSTA, &lnksta);
    *speed = dev->bus->cur_bus_speed;
    *width = (lnksta & PCI_EXP_LNKSTA_NLW) >> PCI_EXP_LNKSTA_NLW_SHIFT;
}

static void validate_pcie(struct pci_dev *dev)
{
    uint maxSpeed, maxWidth;
    uint curSpeed, curWidth;

    pcie_max_link_cap(dev, &maxSpeed, &maxWidth);
    pcie_cur_link_sta(dev, &curSpeed, &curWidth);

    uint maxGen = PCIeSpeedToGen(maxSpeed);
    uint curGen = PCIeSpeedToGen(curSpeed);

    if (maxSpeed != curSpeed)
        dev_warn(&dev->dev, "PCIe Warning: Max PCIe Gen %u, but current is Gen %u\n", maxGen, curGen);

    if (maxWidth != curWidth)
        dev_warn(&dev->dev, "PCIe Warning: Max PCIe Width x%u, but current is x%u\n", maxWidth, curWidth);
}

static int limelitepcie_pci_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
    mutex_lock(&probe_lock);

    int ret = 0;
    uint8_t rev_id;
    int i;
    char fpga_identifier[256];
    struct limelitepcie_device *limelitepcie_dev = NULL;

    dev_info(&dev->dev, "[Probing device]\n");

    limelitepcie_dev = devm_kzalloc(&dev->dev, sizeof(struct limelitepcie_device), GFP_KERNEL);
    if (!limelitepcie_dev)
    {
        ret = -ENOMEM;
        goto fail1;
    }

    pci_set_drvdata(dev, limelitepcie_dev);

    limelitepcie_dev->attr.vendor = id->vendor;
    limelitepcie_dev->attr.product = id->device;

    validate_pcie(dev);

    limelitepcie_dev->dev = dev;
    spin_lock_init(&limelitepcie_dev->lock);

    ret = pcim_enable_device(dev);
    if (ret != 0)
    {
        dev_err(&dev->dev, "Cannot enable device\n");
        goto fail1;
    }

    ret = -EIO;

    /* Check device version */
    pci_read_config_byte(dev, PCI_REVISION_ID, &rev_id);
    if (rev_id != EXPECTED_PCI_REVISION_ID)
    {
        dev_err(&dev->dev, "Unsupported device version %d\n", rev_id);
        goto fail1;
    }

    /* Check bar0 config */
    if (!(pci_resource_flags(dev, 0) & IORESOURCE_MEM))
    {
        dev_err(&dev->dev, "Invalid BAR0 configuration\n");
        goto fail1;
    }

    if (pcim_iomap_regions(dev, BIT(0), LIMELITEPCIE_NAME) < 0)
    {
        dev_err(&dev->dev, "Could not request regions\n");
        goto fail1;
    }

    limelitepcie_dev->bar0_addr = pcim_iomap_table(dev)[0];
    if (!limelitepcie_dev->bar0_addr)
    {
        dev_err(&dev->dev, "Could not map BAR0\n");
        goto fail1;
    }
    dev_info(&dev->dev, "BAR0 address=0x%p\n", limelitepcie_dev->bar0_addr);

    /* Reset LitePCIe core */
#ifdef CSR_CTRL_RESET_ADDR
    limelitepcie_writel(limelitepcie_dev, CSR_CTRL_RESET_ADDR, 1);
    msleep(10);
#endif
    /* Show identifier */
    for (i = 0; i < 256; i++)
        fpga_identifier[i] = limelitepcie_readl(limelitepcie_dev, CSR_IDENTIFIER_MEM_BASE + i * 4);

    fpga_identifier[255] = '\0';
    dev_info(&dev->dev, "Identifier %s\n", fpga_identifier);

    pci_set_master(dev);

    ret = dma_set_mask_and_coherent(&dev->dev, DMA_BIT_MASK(64));
    if (ret)
        dev_warn(&dev->dev, "Failed to set DMA mask 64bit. Falling back to 32bit\n");

    ret = dma_set_mask_and_coherent(&dev->dev, DMA_BIT_MASK(32));
    if (ret)
    {
        dev_err(&dev->dev, "Failed to set DMA mask 32bit. Critical error.\n");
        goto fail1;
    };

    ret = AllocateIRQs(limelitepcie_dev);
    if (ret < 0)
        goto fail2;

    if (ValidIdentifier(fpga_identifier))
        ParseIdentifier(fpga_identifier, limelitepcie_dev->info.devName, sizeof(limelitepcie_dev->info.devName));
    else if (ReadInfo(limelitepcie_dev) != 0)
    {
        dev_err(&dev->dev, "Failed to read limelitepcie device info\n");
        goto fail2;
    }

    uint32_t dmaBufferSize = 8192;

    switch (limelitepcie_dev->info.boardId)
    {
    case LMS_DEV_LIMESDR_X3:
    case LMS_DEV_LIMESDR_QPCIE:
        dmaBufferSize = 32768;
        break;
    case LMS_DEV_LIMESDR_XTRX:
    case LMS_DEV_LIME_MM_X8:
        dmaBufferSize = 8192;
        break;
    default: {
        dmaBufferSize = 8192;
        dev_warn(&dev->dev, "DMA buffer size not specified, defaulting to %i", dmaBufferSize);
    }
    }

    uint32_t dmaBufferCount = MAX_DMA_BUFFER_COUNT;
    int32_t dmaChannels = limelitepcie_readl(limelitepcie_dev, CSR_CNTRL_NDMA_ADDR);
    if (dmaChannels > MAX_DMA_CHANNEL_COUNT || dmaChannels < 0)
    {
        dev_err(&dev->dev, "Invalid DMA channel count(%i)\n", dmaChannels);
        dmaChannels = 0;
    }
    dev_info(&dev->dev, "DMA channels: %i, buffer size: %i, buffers count: %i", dmaChannels, dmaBufferSize, dmaBufferCount);
    limelitepcie_dev->channels = dmaChannels;

    /* create all chardev in /dev */
    ret = limelitepcie_alloc_chdev(limelitepcie_dev, &dev->dev);
    if (ret)
    {
        dev_err(&dev->dev, "Failed to allocate character device\n");
        goto fail2;
    }

    for (i = 0; i < limelitepcie_dev->channels; i++)
    {
        limelitepcie_dev->chan[i].dma.bufferCount = dmaBufferCount;
        limelitepcie_dev->chan[i].dma.bufferSize = dmaBufferSize;
        limelitepcie_dev->chan[i].index = i;
        limelitepcie_dev->chan[i].block_size = dmaBufferSize;
        limelitepcie_dev->chan[i].minor = limelitepcie_dev->minor_base + i;
        limelitepcie_dev->chan[i].limelitepcie_dev = limelitepcie_dev;
        limelitepcie_dev->chan[i].dma.writer_lock = 0;
        limelitepcie_dev->chan[i].dma.reader_lock = 0;
        init_waitqueue_head(&limelitepcie_dev->chan[i].wait_rd);
        init_waitqueue_head(&limelitepcie_dev->chan[i].wait_wr);
        switch (i)
        {
#define CASE_DMA(x) \
    case x: { \
        limelitepcie_dev->chan[i].dma.base = CSR_PCIE_DMA##x##_BASE; \
        limelitepcie_dev->chan[i].dma.writer_interrupt = PCIE_DMA##x##_WRITER_INTERRUPT; \
        limelitepcie_dev->chan[i].dma.reader_interrupt = PCIE_DMA##x##_READER_INTERRUPT; \
    } \
    break

            CASE_DMA(7);
            CASE_DMA(6);
            CASE_DMA(5);
            CASE_DMA(4);
            CASE_DMA(3);
            CASE_DMA(2);
            CASE_DMA(1);
#undef CASE_DMA
        default: {
            limelitepcie_dev->chan[i].dma.base = CSR_PCIE_DMA0_BASE;
            limelitepcie_dev->chan[i].dma.writer_interrupt = PCIE_DMA0_WRITER_INTERRUPT;
            limelitepcie_dev->chan[i].dma.reader_interrupt = PCIE_DMA0_READER_INTERRUPT;
        }
        break;
        }
    }

    /* allocate all dma buffers */
    ret = limelitepcie_dma_init(limelitepcie_dev);
    if (ret)
    {
        dev_err(&dev->dev, "Failed to allocate DMA\n");
        goto fail3;
    }

    sema_init(&limelitepcie_dev->control_semaphore, 1);

#ifdef CSR_UART_XOVER_RXTX_ADDR
    tty_res = devm_kzalloc(&dev->dev, sizeof(struct resource), GFP_KERNEL);
    if (!tty_res)
    {
        mutex_unlock(&probe_lock);
        return -ENOMEM;
    }
    tty_res->start = (resource_size_t)litepcie_dev->bar0_addr + CSR_UART_XOVER_RXTX_ADDR - CSR_BASE;
    tty_res->flags = IORESOURCE_REG;
    limelitepcie_dev->uart = platform_device_register_simple("liteuart", limelitepcie_minor_idx, tty_res, 1);
    if (IS_ERR(limelitepcie_dev->uart))
    {
        ret = PTR_ERR(limelitepcie_dev->uart);
        goto fail3;
    }
#endif
    mutex_unlock(&probe_lock);
    return 0;

fail3:
    limelitepcie_free_chdev(limelitepcie_dev);
fail2:
    FreeIRQs(limelitepcie_dev);
fail1:
    mutex_unlock(&probe_lock);
    return ret;
}

static void limelitepcie_pci_remove(struct pci_dev *dev)
{
    struct limelitepcie_device *limelitepcie_dev = pci_get_drvdata(dev);
    dev_info(&dev->dev, "[Removing device]\n");

    /* Stop the DMAs */
    limelitepcie_stop_dma(limelitepcie_dev);

    /* Disable all interrupts */
    limelitepcie_writel(limelitepcie_dev, CSR_PCIE_MSI_ENABLE_ADDR, 0);

    limelitepcie_dma_free(limelitepcie_dev);

    FreeIRQs(limelitepcie_dev);
    platform_device_unregister(limelitepcie_dev->uart);
    limelitepcie_free_chdev(limelitepcie_dev);
}

static const struct pci_device_id limelitepcie_pci_ids[] = {{PCI_DEVICE(XILINX_FPGA_VENDOR_ID, XILINX_FPGA_DEVICE_ID)},
    {PCI_DEVICE(XILINX_FPGA_VENDOR_ID, XTRX_FPGA_DEVICE_ID)},
    {PCI_DEVICE(ALTERA_FPGA_VENDOR_ID, ALTERA_FPGA_DEVICE_ID)},
    {0}};
MODULE_DEVICE_TABLE(pci, limelitepcie_pci_ids);

static struct pci_driver limelitepcie_pci_driver = {
    .name = LIMELITEPCIE_NAME,
    .id_table = limelitepcie_pci_ids,
    .probe = limelitepcie_pci_probe,
    .remove = limelitepcie_pci_remove,
    .err_handler = &pci_error_handlers_ops,
};

static int __init limelitepcie_module_init(void)
{
    int ret;
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 4, 0)
    limelitepcie_class = class_create(THIS_MODULE, LIMELITEPCIE_NAME);
#else
    limelitepcie_class = class_create(LIMELITEPCIE_NAME);
#endif
    if (!limelitepcie_class)
    {
        ret = -EEXIST;
        pr_err(" Failed to create class\n");
        goto fail_create_class;
    }

    ret = alloc_chrdev_region(&limelitepcie_dev_t, 0, LIMELITEPCIE_MINOR_COUNT, LIMELITEPCIE_NAME);
    if (ret < 0)
    {
        pr_err(" Could not allocate char device\n");
        goto fail_alloc_chrdev_region;
    }
    limelitepcie_major = MAJOR(limelitepcie_dev_t);
    limelitepcie_minor_idx = MINOR(limelitepcie_dev_t);

    ret = pci_register_driver(&limelitepcie_pci_driver);
    if (ret < 0)
    {
        pr_err(" Error while registering PCI driver\n");
        goto fail_register;
    }

    mutex_init(&probe_lock);
    return 0;

fail_register:
    unregister_chrdev_region(limelitepcie_dev_t, LIMELITEPCIE_MINOR_COUNT);
fail_alloc_chrdev_region:
    class_destroy(limelitepcie_class);
fail_create_class:
    return ret;
}

static void __exit limelitepcie_module_exit(void)
{
    pci_unregister_driver(&limelitepcie_pci_driver);
    unregister_chrdev_region(limelitepcie_dev_t, LIMELITEPCIE_MINOR_COUNT);
    class_destroy(limelitepcie_class);
}

module_init(limelitepcie_module_init);
module_exit(limelitepcie_module_exit);

MODULE_LICENSE("GPL");
