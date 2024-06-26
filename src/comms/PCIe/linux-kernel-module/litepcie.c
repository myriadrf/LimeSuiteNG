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

#include "litepcie.h"
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

#define LITEPCIE_NAME "litepcie"
#define LITEPCIE_MINOR_COUNT 32

#define MAX_DMA_BUFFER_COUNT 256
#define MAX_DMA_CHANNEL_COUNT 16

struct litepcie_dma_chan {
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
    char devName[128];
};

struct litepcie_chan {
    struct litepcie_device *litepcie_dev;
    struct litepcie_dma_chan dma;
    struct cdev cdev;
    uint32_t block_size;
    uint32_t core_base;
    wait_queue_head_t wait_rd; /* to wait for an ongoing read */
    wait_queue_head_t wait_wr; /* to wait for an ongoing write */

    int index;
    int minor;
};

struct litepcie_device {
    struct pci_dev *dev;
    struct platform_device *uart;
    resource_size_t bar0_size;
    phys_addr_t bar0_phys_addr;
    uint8_t *bar0_addr; /* virtual address of BAR0 */
    struct litepcie_chan chan[MAX_DMA_CHANNEL_COUNT];
    spinlock_t lock;
    int minor_base;
    int irqs;
    int channels;
    struct cdev control_cdev; // non DMA channel for control packets
    struct deviceInfo info;
};

struct litepcie_chan_priv {
    struct litepcie_chan *chan;
    bool reader;
    bool writer;
};

static int litepcie_major;
static int litepcie_minor_idx;
static struct class *litepcie_class;
static dev_t litepcie_dev_t;

static inline uint32_t litepcie_readl(struct litepcie_device *s, uint32_t addr)
{
    uint32_t val = readl(s->bar0_addr + addr - CSR_BASE);
#ifdef DEBUG_CSR
    dev_dbg(&s->dev->dev, "csr_read: 0x%08x @ 0x%08x", val, addr);
#endif
    return val;
}

static inline void litepcie_writel(struct litepcie_device *s, uint32_t addr, uint32_t val)
{
#ifdef DEBUG_CSR
    dev_dbg(&s->dev->dev, "csr_write: 0x%08x @ 0x%08x", val, addr);
#endif
    return writel(val, s->bar0_addr + addr - CSR_BASE);
}

static void litepcie_enable_interrupt(struct litepcie_device *s, int irq_num)
{
    uint32_t v = litepcie_readl(s, CSR_PCIE_MSI_ENABLE_ADDR);
    v |= (1 << irq_num);
    litepcie_writel(s, CSR_PCIE_MSI_ENABLE_ADDR, v);
}

static void litepcie_disable_interrupt(struct litepcie_device *s, int irq_num)
{
    uint32_t v = litepcie_readl(s, CSR_PCIE_MSI_ENABLE_ADDR);
    v &= ~(1 << irq_num);
    litepcie_writel(s, CSR_PCIE_MSI_ENABLE_ADDR, v);
}

static int litepcie_dma_init(struct litepcie_device *s)
{
    if (!s)
        return -ENODEV;

    /* for each dma channel */
    for (int i = 0; i < s->channels; i++)
    {
        struct litepcie_dma_chan *dmachan = &s->chan[i].dma;
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

static int litepcie_dma_free(struct litepcie_device *s)
{
    if (!s)
        return -ENODEV;

    /* for each dma channel */
    for (int i = 0; i < s->channels; i++)
    {
        struct litepcie_dma_chan *dmachan = &s->chan[i].dma;
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

static void ClearDMAWriterCounters(struct litepcie_dma_chan *dmachan)
{
    dmachan->writer_hw_count = 0;
    dmachan->writer_hw_count_last = 0;
    dmachan->writer_sw_count = 0;
}

static void ClearDMAReaderCounters(struct litepcie_dma_chan *dmachan)
{
    dmachan->reader_hw_count = 0;
    dmachan->reader_hw_count_last = 0;
    dmachan->reader_sw_count = 0;
}

static int litepcie_dma_writer_request(
    struct litepcie_device *s, struct litepcie_dma_chan *dmachan, dma_addr_t dmaBufAddr, uint32_t size, bool genIRQ)
{
    if (size == 0 || size > dmachan->bufferSize)
    {
        dev_err(&s->dev->dev, "DMA writer request: invalid size %i\n", size);
        return -EINVAL;
    }
    /* Fill DMA Writer descriptor, buffer size + parameters. */
    litepcie_writel(s,
        dmachan->base + PCIE_DMA_WRITER_TABLE_VALUE_OFFSET,
#ifndef DMA_BUFFER_ALIGNED
        DMA_LAST_DISABLE |
#endif
            (!genIRQ) * DMA_IRQ_DISABLE | /* generate an msi */
            size);
    /* Fill 32-bit Address LSB. */
    litepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_TABLE_VALUE_OFFSET + 4, (dmaBufAddr >> 0) & 0xffffffff);
    /* Write descriptor (and fill 32-bit Address MSB for 64-bit mode). */
    litepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_TABLE_WE_OFFSET, (dmaBufAddr >> 32) & 0xffffffff);
#ifdef DEBUG_READ
    dev_dbg(&s->dev->dev, "DMA writer request - buf:x%llx size:%i irq:%i\n", dmaBufAddr, size, genIRQ ? 1 : 0);
#endif
    return 0;
}

static int litepcie_dma_reader_request(
    struct litepcie_device *s, struct litepcie_dma_chan *dmachan, dma_addr_t dmaBufAddr, uint32_t size, bool genIRQ)
{
    if (size == 0 || size > dmachan->bufferSize)
    {
        dev_err(&s->dev->dev, "DMA reader request: invalid size %i\n", size);
        return -EINVAL;
    }
    /* Fill DMA Reader descriptor, buffer size + parameters. */
    litepcie_writel(s,
        dmachan->base + PCIE_DMA_READER_TABLE_VALUE_OFFSET,
#ifndef DMA_BUFFER_ALIGNED
        DMA_LAST_DISABLE |
#endif
            (!genIRQ) * DMA_IRQ_DISABLE | /* generate an msi */
            size);
    /* Fill 32-bit Address LSB. */
    litepcie_writel(s, dmachan->base + PCIE_DMA_READER_TABLE_VALUE_OFFSET + 4, (dmaBufAddr >> 0) & 0xffffffff);
    /* Write descriptor (and fill 32-bit Address MSB for 64-bit mode). */
    litepcie_writel(s, dmachan->base + PCIE_DMA_READER_TABLE_WE_OFFSET, (dmaBufAddr >> 32) & 0xffffffff);
#ifdef DEBUG_WRITE
    dev_dbg(&s->dev->dev, "DMA reader request - buf:x%llx size:%i irq:%i\n", dmaBufAddr, size, genIRQ ? 1 : 0);
#endif
    return 0;
}

static void litepcie_dma_writer_start(struct litepcie_device *s, int chan_num, uint32_t write_size, uint8_t irqFreq)
{
    struct litepcie_dma_chan *dmachan = &s->chan[chan_num].dma;
    if (write_size == 0 || write_size > dmachan->bufferSize)
    {
        dev_err(&s->dev->dev, "DMA writer start: invalid write size %i\n", write_size);
        return;
    }

    /* Fill DMA Writer descriptors. */
    litepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_ENABLE_OFFSET, 0);
    litepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_TABLE_FLUSH_OFFSET, 1);
    litepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_TABLE_LOOP_PROG_N_OFFSET, 0);
    for (int i = 0; i < dmachan->bufferCount; i++)
    {
        const bool genIRQ = (i % irqFreq == 0);
        if (litepcie_dma_writer_request(s, dmachan, dmachan->writer_handle[i], write_size, genIRQ) != 0)
        {
            dev_err(&s->dev->dev, "DMA%i writer start failed\n", chan_num);
            return;
        }
    }
    litepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_TABLE_LOOP_PROG_N_OFFSET, 1);
    ClearDMAWriterCounters(dmachan);

    /* Start DMA Writer. */
    litepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_ENABLE_OFFSET, 1);
    dev_info(&s->dev->dev, "Rx%i DMA writer start, buffer size: %u, IRQ every: %i buffers", chan_num, write_size, irqFreq);
}

static void litepcie_dma_writer_stop(struct litepcie_device *s, int chan_num)
{
    struct litepcie_dma_chan *dmachan = &s->chan[chan_num].dma;

    /* Flush and stop DMA Writer. */
    const int transfersDone = litepcie_readl(s, dmachan->base + PCIE_DMA_WRITER_TABLE_LOOP_STATUS_OFFSET);
    litepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_TABLE_LOOP_PROG_N_OFFSET, 0);
    litepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_TABLE_FLUSH_OFFSET, 1);
    udelay(1000);
    litepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_ENABLE_OFFSET, 0);
    litepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_TABLE_FLUSH_OFFSET, 1);

    dev_info(&s->dev->dev, "Rx%i DMA writer stop, DMA descriptors processed: %i\n", chan_num, transfersDone);
    ClearDMAWriterCounters(dmachan);
}

static void litepcie_dma_reader_start(struct litepcie_device *s, int chan_num)
{
    struct litepcie_dma_chan *dmachan = &s->chan[chan_num].dma;
    /* Fill DMA Reader descriptors. */
    litepcie_writel(s, dmachan->base + PCIE_DMA_READER_ENABLE_OFFSET, 0);
    litepcie_writel(s, dmachan->base + PCIE_DMA_READER_TABLE_FLUSH_OFFSET, 1);
    litepcie_writel(s, dmachan->base + PCIE_DMA_READER_TABLE_LOOP_PROG_N_OFFSET, 0);

    ClearDMAReaderCounters(dmachan);
    /* start dma reader */
    litepcie_writel(s, dmachan->base + PCIE_DMA_READER_ENABLE_OFFSET, 1);
    dev_info(&s->dev->dev, "Tx%i DMA reader start", chan_num);
}

static void litepcie_dma_reader_stop(struct litepcie_device *s, int chan_num)
{
    struct litepcie_dma_chan *dmachan = &s->chan[chan_num].dma;

    const uint32_t loop_status = litepcie_readl(s, dmachan->base + PCIE_DMA_READER_TABLE_LOOP_STATUS_OFFSET);
    /* flush and stop dma reader */
    litepcie_writel(s, dmachan->base + PCIE_DMA_READER_TABLE_LOOP_PROG_N_OFFSET, 0);
    litepcie_writel(s, dmachan->base + PCIE_DMA_READER_TABLE_FLUSH_OFFSET, 1);
    udelay(1000);
    litepcie_writel(s, dmachan->base + PCIE_DMA_READER_ENABLE_OFFSET, 0);
    litepcie_writel(s, dmachan->base + PCIE_DMA_READER_TABLE_FLUSH_OFFSET, 1);
    dev_info(&s->dev->dev, "Tx%i DMA reader stop, DMA descriptors processed: %i\n", chan_num, loop_status);
    ClearDMAReaderCounters(dmachan);
}

void litepcie_stop_dma(struct litepcie_device *s)
{
    struct litepcie_dma_chan *dmachan;
    for (int i = 0; i < s->channels; i++)
    {
        dmachan = &s->chan[i].dma;
        litepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_ENABLE_OFFSET, 0);
        litepcie_writel(s, dmachan->base + PCIE_DMA_READER_ENABLE_OFFSET, 0);
    }
}

static irqreturn_t litepcie_interrupt(int irq, void *data)
{
    struct litepcie_device *s = (struct litepcie_device *)data;
    uint32_t irq_enable = litepcie_readl(s, CSR_PCIE_MSI_ENABLE_ADDR);
    uint32_t irq_vector;
/* Single MSI */
#ifdef CSR_PCIE_MSI_CLEAR_ADDR
    irq_vector = litepcie_readl(s, CSR_PCIE_MSI_VECTOR_ADDR);
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
        struct litepcie_chan *chan = &s->chan[i];
        /* dma reader interrupt handling */
        if (irq_vector & (1 << chan->dma.reader_interrupt) && chan->dma.reader_enable)
        {
            uint32_t readTransfers = litepcie_readl(s, chan->dma.base + PCIE_DMA_READER_TABLE_LOOP_STATUS_OFFSET);
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
            uint32_t writeTransfers = litepcie_readl(s, chan->dma.base + PCIE_DMA_WRITER_TABLE_LOOP_STATUS_OFFSET);
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
        litepcie_writel(s, CSR_PCIE_MSI_CLEAR_ADDR, clear_mask);
#endif
    return IRQ_HANDLED;
}

static int litepcie_open(struct inode *inode, struct file *file)
{
    struct litepcie_chan *chan = container_of(inode->i_cdev, struct litepcie_chan, cdev);
    struct litepcie_chan_priv *chan_priv = kzalloc(sizeof(*chan_priv), GFP_KERNEL);
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

static int litepcie_release(struct inode *inode, struct file *file)
{
    struct litepcie_chan_priv *chan_priv = file->private_data;
    struct litepcie_chan *chan = chan_priv->chan;
    pr_info("Release %s\n", file->f_path.dentry->d_iname);

    if (chan_priv->reader)
    {
        /* disable interrupt */
        litepcie_disable_interrupt(chan->litepcie_dev, chan->dma.reader_interrupt);
        /* disable DMA */
        litepcie_dma_reader_stop(chan->litepcie_dev, chan->index);
        chan->dma.reader_lock = 0;
        chan->dma.reader_enable = 0;
        wake_up_interruptible(&chan->wait_wr);
    }

    if (chan_priv->writer)
    {
        /* disable interrupt */
        litepcie_disable_interrupt(chan->litepcie_dev, chan->dma.writer_interrupt);
        /* disable DMA */
        litepcie_dma_writer_stop(chan->litepcie_dev, chan->index);
        chan->dma.writer_lock = 0;
        chan->dma.writer_enable = 0;
        wake_up_interruptible(&chan->wait_rd);
    }
    kfree(chan_priv);
    return 0;
}

static ssize_t litepcie_read(struct file *file, char __user *data, size_t size, loff_t *offset)
{
    struct litepcie_chan_priv *chan_priv = file->private_data;
    struct litepcie_chan *chan = chan_priv->chan;
    struct litepcie_device *s = chan->litepcie_dev;
    struct litepcie_dma_chan *dmachan = &chan->dma;
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
        litepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_ENABLE_OFFSET, 0);
        litepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_TABLE_FLUSH_OFFSET, 1);
        litepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_TABLE_LOOP_PROG_N_OFFSET, 0); // don't loop table
        litepcie_enable_interrupt(chan->litepcie_dev, chan->dma.writer_interrupt);

        size_t bytesRemaining = size;
        while (bytesRemaining > 0)
        {
            const int requestSize = min(bytesRemaining, maxTransferSize);
            bytesRemaining -= requestSize;
            bool genIRQ = bytesRemaining == 0; // generate IRQ on last transfer
            int status = litepcie_dma_writer_request(s, dmachan, dmachan->writer_handle[bufCount], requestSize, genIRQ);
            if (status != 0)
                return status;
            ++bufCount;
        }

        ClearDMAWriterCounters(dmachan);
        //litepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_TABLE_LOOP_PROG_N_OFFSET, 1); // don't loop table
        litepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_ENABLE_OFFSET, 1); // start DMA writing
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
    litepcie_writel(s, dmachan->base + PCIE_DMA_WRITER_ENABLE_OFFSET, 0);
    litepcie_disable_interrupt(chan->litepcie_dev, chan->dma.writer_interrupt);
    dmachan->writer_enable = false; // TODO: implement for non blocking

#ifdef DEBUG_READ
    dev_dbg(&s->dev->dev, "read %i/%ld bytes\n", bytesReceived, size);
#endif
    return bytesReceived;
}

static ssize_t submiteWrite(struct file *file, size_t bufSize, bool genIRQ)
{
    int ret;
    struct litepcie_chan_priv *chan_priv = file->private_data;
    struct litepcie_chan *chan = chan_priv->chan;
    struct litepcie_device *s = chan->litepcie_dev;

    if (bufSize > chan->dma.bufferSize || bufSize <= 0)
    {
        dev_dbg(&s->dev->dev, "DMA writer invalid bufSize(%li) DMA_BUFFER_SIZE(%i)", bufSize, chan->dma.bufferSize);
        return -EINVAL;
    }

    chan->dma.reader_hw_count = litepcie_readl(s, chan->dma.base + PCIE_DMA_READER_TABLE_LOOP_STATUS_OFFSET) & 0xFFFF;

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

    struct litepcie_dma_chan *dmachan = &chan->dma;
    const int bufIndex = chan->dma.reader_sw_count % chan->dma.bufferCount;
    ++chan->dma.reader_sw_count;
    ret = litepcie_dma_reader_request(s, dmachan, dmachan->reader_handle[bufIndex], bufSize, genIRQ);
    if (ret != 0)
        return ret;

    chan->dma.reader_sw_count &= 0xFFFF;
    return 0;
}

static ssize_t litepcie_write(struct file *file, const char __user *data, size_t size, loff_t *offset)
{
    struct litepcie_chan_priv *chan_priv = file->private_data;
    struct litepcie_chan *chan = chan_priv->chan;
    struct litepcie_device *s = chan->litepcie_dev;
    struct litepcie_dma_chan *dmachan = &chan->dma;
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
        litepcie_writel(s, dmachan->base + PCIE_DMA_READER_ENABLE_OFFSET, 0);
        litepcie_writel(s, dmachan->base + PCIE_DMA_READER_TABLE_FLUSH_OFFSET, 1);
        litepcie_writel(s, dmachan->base + PCIE_DMA_READER_TABLE_LOOP_PROG_N_OFFSET, 0); // don't loop table
        litepcie_enable_interrupt(s, dmachan->reader_interrupt);

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
            int status = litepcie_dma_reader_request(s, dmachan, dmachan->reader_handle[bufCount], requestSize, genIRQ);
            if (status != 0)
            {
                dev_dbg(&s->dev->dev, "DMA reader request failed %i\n", status);
                return status;
            }
            ++bufCount;
        }

        litepcie_writel(s, dmachan->base + PCIE_DMA_READER_ENABLE_OFFSET, 1); // start DMA reading
        dev_dbg(&s->dev->dev, "DMA reader enable for %i buffers\n", bufCount);
        dmachan->reader_enable = true;
        ++dmachan->reader_sw_count;
    }

    int ret = wait_event_interruptible(chan->wait_wr, dmachan->reader_sw_count - dmachan->reader_hw_count == 0);
    if (ret < 0)
        return ret;
    litepcie_writel(s, dmachan->base + PCIE_DMA_READER_ENABLE_OFFSET, 0);
    litepcie_disable_interrupt(s, dmachan->reader_interrupt);
    dmachan->reader_enable = false;
#ifdef DEBUG_WRITE
    dev_dbg(&s->dev->dev, "write(%ld) done\n", size);
#endif
    return size;
}

static int litepcie_mmap(struct file *file, struct vm_area_struct *vma)
{
    struct litepcie_chan_priv *chan_priv = file->private_data;
    struct litepcie_chan *chan = chan_priv->chan;
    struct litepcie_device *s = chan->litepcie_dev;
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

static unsigned int litepcie_poll(struct file *file, poll_table *wait)
{
    struct litepcie_chan_priv *chan_priv = file->private_data;
    struct litepcie_chan *chan = chan_priv->chan;

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
    struct litepcie_device *s = chan->litepcie_dev;
    dev_dbg(&s->dev->dev,
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

static int litepcie_flash_spi(struct litepcie_device *s, struct litepcie_ioctl_flash *m)
{
    int i;

    if (m->tx_len < 8 || m->tx_len > 40)
        return -EINVAL;

    litepcie_writel(s, CSR_FLASH_SPI_MOSI_ADDR, m->tx_data >> 32);
    litepcie_writel(s, CSR_FLASH_SPI_MOSI_ADDR + 4, m->tx_data);
    litepcie_writel(s, CSR_FLASH_SPI_CONTROL_ADDR, SPI_CTRL_START | (m->tx_len * SPI_CTRL_LENGTH));
    udelay(16);
    for (i = 0; i < SPI_TIMEOUT; i++)
    {
        if (litepcie_readl(s, CSR_FLASH_SPI_STATUS_ADDR) & SPI_STATUS_DONE)
            break;
        udelay(1);
    }
    m->rx_data = ((uint64_t)litepcie_readl(s, CSR_FLASH_SPI_MISO_ADDR) << 32) | litepcie_readl(s, CSR_FLASH_SPI_MISO_ADDR + 4);
    return 0;
}
#endif

static long litepcie_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    long ret = 0;

    struct litepcie_chan_priv *chan_priv = file->private_data;
    struct litepcie_chan *chan = chan_priv->chan;
    struct litepcie_device *dev = chan->litepcie_dev;

    switch (cmd)
    {
    case LITEPCIE_IOCTL_REG: {
        struct litepcie_ioctl_reg m;

        if (copy_from_user(&m, (void *)arg, sizeof(m)))
        {
            ret = -EFAULT;
            break;
        }
        if (m.is_write)
            litepcie_writel(dev, m.addr, m.val);
        else
            m.val = litepcie_readl(dev, m.addr);

        if (copy_to_user((void *)arg, &m, sizeof(m)))
        {
            ret = -EFAULT;
            break;
        }
    }
    break;
#ifdef CSR_FLASH_BASE
    case LITEPCIE_IOCTL_FLASH: {
        struct litepcie_ioctl_flash m;

        if (copy_from_user(&m, (void *)arg, sizeof(m)))
        {
            ret = -EFAULT;
            break;
        }
        ret = litepcie_flash_spi(dev, &m);
        if (ret == 0)
        {
            if (copy_to_user((void *)arg, &m, sizeof(m)))
            {
                ret = -EFAULT;
                break;
            }
        }
    }
    break;
#endif
#ifdef CSR_ICAP_BASE
    case LITEPCIE_IOCTL_ICAP: {
        struct litepcie_ioctl_icap m;

        if (copy_from_user(&m, (void *)arg, sizeof(m)))
        {
            ret = -EFAULT;
            break;
        }

        litepcie_writel(dev, CSR_ICAP_ADDR_ADDR, m.addr);
        litepcie_writel(dev, CSR_ICAP_DATA_ADDR, m.data);
        litepcie_writel(dev, CSR_ICAP_WRITE_ADDR, 1);
    }
    break;
#endif
    case LITEPCIE_IOCTL_DMA: {
        struct litepcie_ioctl_dma m;

        if (copy_from_user(&m, (void *)arg, sizeof(m)))
        {
            ret = -EFAULT;
            break;
        }

        /* loopback */
        //litepcie_writel(chan->litepcie_dev, chan->dma.base + PCIE_DMA_LOOPBACK_ENABLE_OFFSET, m.loopback_enable);
    }
    break;
    case LITEPCIE_IOCTL_DMA_WRITER: {
        struct litepcie_ioctl_dma_writer m;

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
                litepcie_dma_writer_start(chan->litepcie_dev, chan->index, m.write_size, m.irqFreq);
                litepcie_enable_interrupt(chan->litepcie_dev, chan->dma.writer_interrupt);
            }
            else
            {
                //dev_info(&dev->dev->dev, "%s DMA writer disable\n", file->f_path.dentry->d_iname);
                litepcie_disable_interrupt(chan->litepcie_dev, chan->dma.writer_interrupt);
                litepcie_dma_writer_stop(chan->litepcie_dev, chan->index);
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
    case LITEPCIE_IOCTL_DMA_READER: {
        struct litepcie_ioctl_dma_reader m;

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
                litepcie_dma_reader_start(chan->litepcie_dev, chan->index);
                litepcie_enable_interrupt(chan->litepcie_dev, chan->dma.reader_interrupt);
            }
            else
            {
                //dev_info(&dev->dev->dev, "%s DMA reader disable\n", file->f_path.dentry->d_iname);
                litepcie_disable_interrupt(chan->litepcie_dev, chan->dma.reader_interrupt);
                litepcie_dma_reader_stop(chan->litepcie_dev, chan->index);
            }
        }

        chan->dma.reader_enable = m.enable;

        uint32_t readTransfers = litepcie_readl(dev, chan->dma.base + PCIE_DMA_READER_TABLE_LOOP_STATUS_OFFSET);
        m.hw_count = readTransfers & 0xFFFF;
        m.sw_count = chan->dma.reader_sw_count & 0xFFFF;

        if (copy_to_user((void *)arg, &m, sizeof(m)))
        {
            ret = -EFAULT;
            break;
        }
    }
    break;
    case LITEPCIE_IOCTL_MMAP_DMA_INFO: {
        struct litepcie_ioctl_mmap_dma_info m;

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
    case LITEPCIE_IOCTL_MMAP_DMA_WRITER_UPDATE: {
        struct litepcie_ioctl_mmap_dma_update m;

        if (copy_from_user(&m, (void *)arg, sizeof(m)))
        {
            ret = -EFAULT;
            break;
        }

        chan->dma.writer_sw_count = m.sw_count & 0xFFFF;
    }
    break;
    case LITEPCIE_IOCTL_MMAP_DMA_READER_UPDATE: {
        struct litepcie_ioctl_mmap_dma_update m;

        if (copy_from_user(&m, (void *)arg, sizeof(m)))
        {
            dev_dbg(&chan->litepcie_dev->dev->dev, "LITEPCIE_IOCTL_MMAP_DMA_READER_UPDATE copy_from_user fail");
            ret = -EFAULT;
            break;
        }

        chan->dma.reader_sw_count = m.sw_count & 0xFFFF;

        ret = submiteWrite(file, m.buffer_size, m.genIRQ);
    }
    break;

    case LITEPCIE_IOCTL_CACHE_FLUSH: {
        struct litepcie_cache_flush m;

        if (copy_from_user(&m, (void *)arg, sizeof(m)))
        {
            dev_dbg(&chan->litepcie_dev->dev->dev, "LITEPCIE_IOCTL_CACHE_FLUSH copy_from_user fail");
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
    case LITEPCIE_IOCTL_LOCK: {
        struct litepcie_ioctl_lock m;

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

static int litepcie_ctrl_open(struct inode *inode, struct file *file)
{
    pr_info("Open %s\n", file->f_path.dentry->d_iname);
    struct litepcie_device *ctrlDevice = container_of(inode->i_cdev, struct litepcie_device, control_cdev);
    file->private_data = ctrlDevice;

    litepcie_writel(ctrlDevice, CSR_CNTRL_TEST_ADDR, 0x55);
    if (litepcie_readl(ctrlDevice, CSR_CNTRL_TEST_ADDR) != 0x55)
    {
        printk(KERN_ERR LITEPCIE_NAME " CSR register test failed\n");
        return -EIO;
    }
    return 0;
}

static int litepcie_ctrl_release(struct inode *inode, struct file *file)
{
    pr_info("Release %s\n", file->f_path.dentry->d_iname);
    return 0;
}

static ssize_t litepcie_ctrl_write(struct file *file, const char __user *userbuf, size_t count, loff_t *offset)
{
    struct litepcie_device *s = file->private_data;
    uint32_t value;
    count = min(count, CSR_CNTRL_CNTRL_SIZE * sizeof(uint32_t));
    for (int i = 0; i < count; i += sizeof(value))
    {
        if (copy_from_user(&value, userbuf + i, sizeof(value)))
            return -EFAULT;
        litepcie_writel(s, CSR_CNTRL_BASE + i, value);
    }
    return count;
}

static ssize_t litepcie_ctrl_read(struct file *file, char __user *userbuf, size_t count, loff_t *offset)
{
    struct litepcie_device *s = file->private_data;
    uint32_t value;
    count = min(count, CSR_CNTRL_CNTRL_SIZE * sizeof(uint32_t));
    for (int i = 0; i < count; i += sizeof(value))
    {
        value = litepcie_readl(s, CSR_CNTRL_BASE + i);
        if (copy_to_user(userbuf + i, &value, sizeof(value)))
            return -EFAULT;
    }
    return count;
}

static pci_ers_result_t litepcie_error_detected(struct pci_dev *dev, pci_channel_state_t state)
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
static pci_ers_result_t litepcie_slot_reset(struct pci_dev *dev)
{
    dev_err(&dev->dev, "PCI slot reset\n");
    return PCI_ERS_RESULT_NONE;
}
// void (*resume)(struct pci_dev *dev);

static const struct pci_error_handlers pci_error_handlers_ops = {
    .error_detected = litepcie_error_detected,
    // .mmio_enabled = mmio_enabled,
    .slot_reset = litepcie_slot_reset,
    // .resume = resume
};

static const struct file_operations litepcie_fops = {
    .owner = THIS_MODULE,
    .unlocked_ioctl = litepcie_ioctl,
    .open = litepcie_open,
    .release = litepcie_release,
    .read = litepcie_read,
    .poll = litepcie_poll,
    .write = litepcie_write,
    .mmap = litepcie_mmap,
};

static const struct file_operations litepcie_fops_control = {
    .owner = THIS_MODULE,
    .open = litepcie_ctrl_open,
    .release = litepcie_ctrl_release,
    .read = litepcie_ctrl_read,
    .write = litepcie_ctrl_write,
};

static int litepcie_alloc_chdev(struct litepcie_device *s)
{
    int ret;

    int index = litepcie_minor_idx;
    char deviceName[64];
    int suffix = 0;
    snprintf(deviceName, sizeof(deviceName) - 1, "%s%i", s->info.devName, suffix);

    s->minor_base = litepcie_minor_idx;
    for (int i = 0; i < s->channels; i++)
    {
        cdev_init(&s->chan[i].cdev, &litepcie_fops);
        ret = cdev_add(&s->chan[i].cdev, MKDEV(litepcie_major, index), 1);
        if (ret < 0)
        {
            dev_err(&s->dev->dev, "Failed to allocate cdev\n");
            goto fail_alloc;
        }
        index++;
    }

    // alloc control
    cdev_init(&s->control_cdev, &litepcie_fops_control);
    ret = cdev_add(&s->control_cdev, MKDEV(litepcie_major, index), 1);
    if (ret < 0)
    {
        dev_err(&s->dev->dev, "Failed to allocate control cdev\n");
        goto fail_alloc_control;
    }

    index = litepcie_minor_idx;
    for (int i = 0; i < s->channels; i++)
    {
        char tempName[64];
        snprintf(tempName, sizeof(tempName) - 1, "%s_trx%d", deviceName, i);
        dev_info(&s->dev->dev, "Creating /dev/%s\n", tempName);
        if (!device_create(litepcie_class, NULL, MKDEV(litepcie_major, index), NULL, tempName))
        {
            ret = -EINVAL;
            dev_err(&s->dev->dev, "Failed to create device\n");
            goto fail_create;
        }
        index++;
    }

    // additionally alloc control channel
    dev_info(&s->dev->dev, "Creating /dev/%s_control\n", deviceName);
    if (!device_create(litepcie_class, NULL, MKDEV(litepcie_major, index), NULL, "%s_control", deviceName))
    {
        ret = -EINVAL;
        dev_err(&s->dev->dev, "Failed to create device\n");
        goto fail_create;
    }
    ++index;

    litepcie_minor_idx = index;
    return 0;

fail_create:
    while (index >= litepcie_minor_idx)
        device_destroy(litepcie_class, MKDEV(litepcie_major, index--));

fail_alloc_control:
    cdev_del(&s->control_cdev);

fail_alloc:
    for (int i = 0; i < s->channels; i++)
        cdev_del(&s->chan[i].cdev);

    return ret;
}

static void litepcie_free_chdev(struct litepcie_device *s)
{
    device_destroy(litepcie_class, MKDEV(litepcie_major, s->minor_base + s->channels));
    cdev_del(&s->control_cdev);
    for (int i = 0; i < s->channels; i++)
    {
        device_destroy(litepcie_class, MKDEV(litepcie_major, s->minor_base + i));
        cdev_del(&s->chan[i].cdev);
    }
}

/* from stackoverflow */
void __attribute__((format(scanf, 2, 3))) sfind(char *string, char *format, ...)
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

void FreeIRQs(struct litepcie_device *device)
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

int AllocateIRQs(struct litepcie_device *device)
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
        int ret = request_irq(irq, litepcie_interrupt, IRQF_SHARED, LITEPCIE_NAME, device);
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

static int ReadInfo(struct litepcie_device *s)
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
        litepcie_writel(s, CSR_CNTRL_BASE + i, value);
    }

    for (int n = 0; n < 10; ++n)
    {
        value = litepcie_readl(s, CSR_CNTRL_BASE);
        memcpy(buffer, &value, sizeof(value));
        if (pkt.status != 0)
            break;
        mdelay(10); // wait between reads until MCU gives answer
    }
    if (pkt.status == 0)
        return -EIO;

    for (int i = 0; i < count; i += sizeof(uint32_t))
    {
        value = litepcie_readl(s, CSR_CNTRL_BASE + i);
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

    snprintf(s->info.devName, sizeof(s->info.devName), "%s", GetDeviceName(s->info.boardId));

    dev_info(&s->dev->dev,
        "[device info] %s FW:%u HW:%u PROTOCOL:%u S/N:0x%016llX \n",
        s->info.devName,
        s->info.firmware,
        s->info.hardwareVer,
        s->info.protocol,
        s->info.serialNumber);
    return 0;
}

static int litepcie_pci_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
    int ret = 0;
    uint8_t rev_id;
    int i;
    char fpga_identifier[256];
    struct litepcie_device *litepcie_dev = NULL;

    dev_info(&dev->dev, "[Probing device]\n");

    litepcie_dev = devm_kzalloc(&dev->dev, sizeof(struct litepcie_device), GFP_KERNEL);
    if (!litepcie_dev)
    {
        ret = -ENOMEM;
        goto fail1;
    }

    pci_set_drvdata(dev, litepcie_dev);
    litepcie_dev->dev = dev;
    spin_lock_init(&litepcie_dev->lock);

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

    if (pcim_iomap_regions(dev, BIT(0), LITEPCIE_NAME) < 0)
    {
        dev_err(&dev->dev, "Could not request regions\n");
        goto fail1;
    }

    litepcie_dev->bar0_addr = pcim_iomap_table(dev)[0];
    if (!litepcie_dev->bar0_addr)
    {
        dev_err(&dev->dev, "Could not map BAR0\n");
        goto fail1;
    }
    dev_info(&dev->dev, "BAR0 address=0x%p\n", litepcie_dev->bar0_addr);

    /* Reset LitePCIe core */
#ifdef CSR_CTRL_RESET_ADDR
    litepcie_writel(litepcie_dev, CSR_CTRL_RESET_ADDR, 1);
    msleep(10);
#endif
    /* Show identifier */
    for (i = 0; i < 256; i++)
        fpga_identifier[i] = litepcie_readl(litepcie_dev, CSR_IDENTIFIER_MEM_BASE + i * 4);
    dev_info(&dev->dev, "Version %s\n", fpga_identifier);

    pci_set_master(dev);

    ret = dma_set_mask_and_coherent(&dev->dev, DMA_BIT_MASK(32));
    if (ret)
    {
        dev_err(&dev->dev, "Failed to set DMA mask 32bit\n");
        goto fail1;
    };

    ret = AllocateIRQs(litepcie_dev);
    if (ret < 0)
        goto fail2;

    if (ReadInfo(litepcie_dev) != 0)
        goto fail2;

    uint32_t dmaBufferSize = 8192;

    switch (litepcie_dev->info.boardId)
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
    int32_t dmaChannels = litepcie_readl(litepcie_dev, CSR_CNTRL_NDMA_ADDR);
    if (dmaChannels > MAX_DMA_CHANNEL_COUNT || dmaChannels < 0)
    {
        dev_err(&dev->dev, "Invalid DMA channel count(%i)\n", dmaChannels);
        dmaChannels = 0;
    }
    dev_info(&dev->dev, "DMA channels: %i, buffer size: %i, buffers count: %i", dmaChannels, dmaBufferSize, dmaBufferCount);
    litepcie_dev->channels = dmaChannels;

    /* create all chardev in /dev */
    ret = litepcie_alloc_chdev(litepcie_dev);
    if (ret)
    {
        dev_err(&dev->dev, "Failed to allocate character device\n");
        goto fail2;
    }

    for (i = 0; i < litepcie_dev->channels; i++)
    {
        litepcie_dev->chan[i].dma.bufferCount = dmaBufferCount;
        litepcie_dev->chan[i].dma.bufferSize = dmaBufferSize;
        litepcie_dev->chan[i].index = i;
        litepcie_dev->chan[i].block_size = dmaBufferSize;
        litepcie_dev->chan[i].minor = litepcie_dev->minor_base + i;
        litepcie_dev->chan[i].litepcie_dev = litepcie_dev;
        litepcie_dev->chan[i].dma.writer_lock = 0;
        litepcie_dev->chan[i].dma.reader_lock = 0;
        init_waitqueue_head(&litepcie_dev->chan[i].wait_rd);
        init_waitqueue_head(&litepcie_dev->chan[i].wait_wr);
        switch (i)
        {
#define CASE_DMA(x) \
    case x: { \
        litepcie_dev->chan[i].dma.base = CSR_PCIE_DMA##x##_BASE; \
        litepcie_dev->chan[i].dma.writer_interrupt = PCIE_DMA##x##_WRITER_INTERRUPT; \
        litepcie_dev->chan[i].dma.reader_interrupt = PCIE_DMA##x##_READER_INTERRUPT; \
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
            litepcie_dev->chan[i].dma.base = CSR_PCIE_DMA0_BASE;
            litepcie_dev->chan[i].dma.writer_interrupt = PCIE_DMA0_WRITER_INTERRUPT;
            litepcie_dev->chan[i].dma.reader_interrupt = PCIE_DMA0_READER_INTERRUPT;
        }
        break;
        }
    }

    /* allocate all dma buffers */
    ret = litepcie_dma_init(litepcie_dev);
    if (ret)
    {
        dev_err(&dev->dev, "Failed to allocate DMA\n");
        goto fail3;
    }

#ifdef CSR_UART_XOVER_RXTX_ADDR
    tty_res = devm_kzalloc(&dev->dev, sizeof(struct resource), GFP_KERNEL);
    if (!tty_res)
        return -ENOMEM;
    tty_res->start = (resource_size_t)litepcie_dev->bar0_addr + CSR_UART_XOVER_RXTX_ADDR - CSR_BASE;
    tty_res->flags = IORESOURCE_REG;
    litepcie_dev->uart = platform_device_register_simple("liteuart", litepcie_minor_idx, tty_res, 1);
    if (IS_ERR(litepcie_dev->uart))
    {
        ret = PTR_ERR(litepcie_dev->uart);
        goto fail3;
    }
#endif

    return 0;

fail3:
    litepcie_free_chdev(litepcie_dev);
fail2:
    FreeIRQs(litepcie_dev);
fail1:
    return ret;
}

static void litepcie_pci_remove(struct pci_dev *dev)
{
    struct litepcie_device *litepcie_dev = pci_get_drvdata(dev);
    dev_info(&dev->dev, "[Removing device]\n");

    /* Stop the DMAs */
    litepcie_stop_dma(litepcie_dev);

    /* Disable all interrupts */
    litepcie_writel(litepcie_dev, CSR_PCIE_MSI_ENABLE_ADDR, 0);

    litepcie_dma_free(litepcie_dev);

    FreeIRQs(litepcie_dev);
    platform_device_unregister(litepcie_dev->uart);
    litepcie_free_chdev(litepcie_dev);
}

static const struct pci_device_id litepcie_pci_ids[] = {{PCI_DEVICE(XILINX_FPGA_VENDOR_ID, XILINX_FPGA_DEVICE_ID)},
    {PCI_DEVICE(XILINX_FPGA_VENDOR_ID, XTRX_FPGA_DEVICE_ID)},
    {PCI_DEVICE(ALTERA_FPGA_VENDOR_ID, ALTERA_FPGA_DEVICE_ID)},
    {0}};
MODULE_DEVICE_TABLE(pci, litepcie_pci_ids);

static struct pci_driver litepcie_pci_driver = {
    .name = LITEPCIE_NAME,
    .id_table = litepcie_pci_ids,
    .probe = litepcie_pci_probe,
    .remove = litepcie_pci_remove,
    .err_handler = &pci_error_handlers_ops,
};

static int __init litepcie_module_init(void)
{
    int ret;
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 4, 0)
    litepcie_class = class_create(THIS_MODULE, LITEPCIE_NAME);
#else
    litepcie_class = class_create(LITEPCIE_NAME);
#endif
    if (!litepcie_class)
    {
        ret = -EEXIST;
        pr_err(" Failed to create class\n");
        goto fail_create_class;
    }

    ret = alloc_chrdev_region(&litepcie_dev_t, 0, LITEPCIE_MINOR_COUNT, LITEPCIE_NAME);
    if (ret < 0)
    {
        pr_err(" Could not allocate char device\n");
        goto fail_alloc_chrdev_region;
    }
    litepcie_major = MAJOR(litepcie_dev_t);
    litepcie_minor_idx = MINOR(litepcie_dev_t);

    ret = pci_register_driver(&litepcie_pci_driver);
    if (ret < 0)
    {
        pr_err(" Error while registering PCI driver\n");
        goto fail_register;
    }
    return 0;

fail_register:
    unregister_chrdev_region(litepcie_dev_t, LITEPCIE_MINOR_COUNT);
fail_alloc_chrdev_region:
    class_destroy(litepcie_class);
fail_create_class:
    return ret;
}

static void __exit litepcie_module_exit(void)
{
    pci_unregister_driver(&litepcie_pci_driver);
    unregister_chrdev_region(litepcie_dev_t, LITEPCIE_MINOR_COUNT);
    class_destroy(litepcie_class);
}

module_init(litepcie_module_init);
module_exit(litepcie_module_exit);

MODULE_LICENSE("GPL");
