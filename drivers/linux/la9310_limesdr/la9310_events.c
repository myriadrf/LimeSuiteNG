#include "la9310_events.h"

#include "la9310_base.h"
#include "la9310_dma_pool.h"
#include "common_headers/la9310_pci_def.h"

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/log2.h>
#include <linux/math.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/pci_regs.h>

// every bit of the mask register, events above the event count stay masked
#define LA9310_EVENT_MASK_ALL GENMASK(LA9310_EVENT_MAX_COUNT - 1, 0)

static unsigned int la9310_max_events = LA9310_EVENT_COUNT;
module_param_named(max_events, la9310_max_events, uint, 0444);
MODULE_PARM_DESC(max_events,
    "Number of events that the M4 firmware can signal, up to " __stringify(LA9310_EVENT_MAX_COUNT) " (default " __stringify(
        LA9310_EVENT_COUNT) ")");

static unsigned long la9310_events_all(const struct la9310_events* events)
{
    return GENMASK(events->count - 1, 0);
}

// counters are read 4 at a time, a byte each
#define LA9310_COUNTERS_PER_WORD 4

// Called with mask_lock held
static u8 la9310_event_read_counter(struct la9310_events* events, unsigned int hwirq)
{
    if (!events->shared)
        return events->event[hwirq].last_counter;
    return ioread8(&events->shared->counters[hwirq]);
}

// Called with mask_lock held
static void la9310_events_write_mask(struct la9310_events* events)
{
    if (events->shared)
        iowrite32(~events->enabled_events & LA9310_EVENT_MASK_ALL, &events->shared->mask);
}

static void la9310_event_mask(struct irq_data* d)
{
    struct la9310_events* events = irq_data_get_irq_chip_data(d);
    unsigned long flags;

    spin_lock_irqsave(&events->mask_lock, flags);
    clear_bit(irqd_to_hwirq(d), &events->enabled_events);
    la9310_events_write_mask(events);
    spin_unlock_irqrestore(&events->mask_lock, flags);
}

static void la9310_event_unmask(struct irq_data* d)
{
    struct la9310_events* events = irq_data_get_irq_chip_data(d);
    const irq_hw_number_t hwirq = irqd_to_hwirq(d);
    unsigned long flags;

    spin_lock_irqsave(&events->mask_lock, flags);
    // The mux handler doesn't look at masked events. Firmware that ignores the mask keeps counting them, resync so the
    // counting while masked doesn't show up as an event.
    if (test_bit(hwirq, &events->mux_events) && !test_bit(hwirq, &events->enabled_events))
        events->event[hwirq].last_counter = la9310_event_read_counter(events, hwirq);
    smp_mb__before_atomic();
    set_bit(hwirq, &events->enabled_events);
    la9310_events_write_mask(events);
    spin_unlock_irqrestore(&events->mask_lock, flags);
}

static struct irq_chip la9310_event_chip = {
    .name = "la9310-event",
    .irq_mask = la9310_event_mask,
    .irq_unmask = la9310_event_unmask,
    // not lazy, disable_irq() stops the device from signalling the event
    .irq_disable = la9310_event_mask,
    .irq_enable = la9310_event_unmask,
};

static int la9310_event_domain_map(struct irq_domain* domain, unsigned int virq, irq_hw_number_t hwirq)
{
    irq_set_chip_data(virq, domain->host_data);
    irq_set_chip_and_handler(virq, &la9310_event_chip, handle_simple_irq);
    irq_set_noprobe(virq);
    return 0;
}

static const struct irq_domain_ops la9310_event_domain_ops = {
    .map = la9310_event_domain_map,
    .xlate = irq_domain_xlate_onecell,
};

// MSI vector 0: dispatches the events whose counters changed
static irqreturn_t la9310_event_mux_handler(int irq, void* data)
{
    struct la9310_events* events = data;
    unsigned long pending = 0;
    unsigned int hwirq;

    spin_lock(&events->mask_lock);
    const unsigned long checked = events->mux_events & events->enabled_events;
    for (unsigned int word = 0; events->shared && word < DIV_ROUND_UP(events->count, LA9310_COUNTERS_PER_WORD); ++word)
    {
        const unsigned int first = word * LA9310_COUNTERS_PER_WORD;
        const unsigned long word_events = (checked >> first) & GENMASK(LA9310_COUNTERS_PER_WORD - 1, 0);
        if (!word_events)
            continue; // don't spend a PCIe read on counters nobody listens to

        const u32 counters = ioread32(&events->shared->counters[first]);
        for (unsigned int i = 0; i < LA9310_COUNTERS_PER_WORD; ++i)
        {
            if (!(word_events & BIT(i)))
                continue;
            struct la9310_event* ev = &events->event[first + i];
            const u8 counter = (counters >> (8 * i)) & 0xFF;
            if (counter != ev->last_counter)
            {
                ev->last_counter = counter;
                __set_bit(ev->hwirq, &pending);
            }
        }
    }
    spin_unlock(&events->mask_lock);

    for_each_set_bit(hwirq, &pending, events->count)
    {
        generic_handle_domain_irq(events->domain, hwirq);
    }

    // The vector isn't shared, an MSI with no changed counter is an event of a masked IRQ, or a repeated one
    return IRQ_HANDLED;
}

// MSI vectors 1..N-1 (multiple MSI): one event each, no need to read the counters
static irqreturn_t la9310_event_vector_handler(int irq, void* data)
{
    struct la9310_event* ev = data;
    generic_handle_domain_irq(ev->parent->domain, ev->hwirq);
    return IRQ_HANDLED;
}

// Driver's handler for the events that user space waits for
static irqreturn_t la9310_event_wait_handler(int virq, void* data)
{
    struct la9310_event* ev = data;
    atomic_inc(&ev->seq);
    wake_up_interruptible(&ev->wait);
    return IRQ_HANDLED;
}

int la9310_events_init(struct la9310_dev* la9310_dev, u32 __iomem* scratch_registers)
{
    struct la9310_events* events = &la9310_dev->events;
    struct device* dev = la9310_dev->dev;
    char name[64];
    int rc;

    memset(events, 0, sizeof(*events));
    spin_lock_init(&events->mask_lock);
    mutex_init(&events->user_lock);
    events->count = clamp_val(la9310_max_events, 1, LA9310_EVENT_MAX_COUNT);
    if (events->count != la9310_max_events)
        dev_warn(dev, "max_events=%u is out of range, using %u\n", la9310_max_events, events->count);
    events->addr_reg = scratch_registers + LA9310_SCRATCH_MSI_EVENTS_ADDR;

    // As many vectors as the host can give, up to what the device advertises (the M4 bootloader sets it to 8). On x86
    // multiple MSI needs interrupt remapping, otherwise there is one vector and all events go through the mux vector.
    int nvec = pci_alloc_irq_vectors(la9310_dev->pdev, 1, LA9310_MSI_MAX_CNT, PCI_IRQ_MSI);
    if (nvec > 1 && !is_power_of_2(nvec))
    {
        // The device sees a power of 2 (Multiple Message Enable), it would send on vectors that have no handler
        pci_free_irq_vectors(la9310_dev->pdev);
        nvec = rounddown_pow_of_two(nvec);
        nvec = pci_alloc_irq_vectors(la9310_dev->pdev, nvec, nvec, PCI_IRQ_MSI);
    }
    if (nvec < 0)
    {
        dev_warn(dev, "MSI not available (%i), the device can't signal events\n", nvec);
        nvec = 0;
    }
    events->vector_count = nvec;

    // The M4 sends events 1..N-1 on their own vectors, the rest on the mux vector
    events->mux_events = la9310_events_all(events);
    if (events->vector_count > 1 && events->count > 1)
        events->mux_events &= ~GENMASK(min(events->vector_count, events->count) - 1, 1);

    for (unsigned int i = 0; i < events->count; ++i)
    {
        struct la9310_event* ev = &events->event[i];
        ev->parent = events;
        ev->hwirq = i;
        snprintf(ev->name, sizeof(ev->name), "la9310-ev%u", i);
        init_waitqueue_head(&ev->wait);
        atomic_set(&ev->seq, 0);
        atomic_set(&ev->consumed, 0);
    }

    snprintf(name, sizeof(name), "la9310-events-%s", dev_name(dev));
    events->fwnode = irq_domain_alloc_named_fwnode(name);
    if (!events->fwnode)
        return -ENOMEM;

    events->domain = irq_domain_create_linear(events->fwnode, events->count, &la9310_event_domain_ops, events);
    if (!events->domain)
    {
        rc = -ENOMEM;
        goto err;
    }

    for (unsigned int i = 0; i < events->count; ++i)
    {
        struct la9310_event* ev = &events->event[i];
        ev->virq = irq_create_mapping(events->domain, i);
        if (!ev->virq)
        {
            dev_err(dev, "event %u: irq_create_mapping failed\n", i);
            rc = -EINVAL;
            goto err;
        }
    }

    // The parent handlers only dispatch into the domain, IRQF_NO_THREAD keeps them in hard IRQ context with forced
    // IRQ threading too; the event handlers get threaded instead.
    for (unsigned int v = 0; v < events->vector_count && v < events->count; ++v)
    {
        // Named after the vector only, the events are listed under the la9310-event domain
        const char* irq_name = devm_kasprintf(dev, GFP_KERNEL, "la9310-msi%u", v);
        if (!irq_name)
        {
            rc = -ENOMEM;
            goto err;
        }
        const unsigned int irq = pci_irq_vector(la9310_dev->pdev, v);
        if (v == LA9310_EVENT_MUX_VECTOR)
            rc = request_irq(irq, la9310_event_mux_handler, IRQF_NO_THREAD, irq_name, events);
        else
            rc = request_irq(irq, la9310_event_vector_handler, IRQF_NO_THREAD, irq_name, &events->event[v]);
        if (rc)
        {
            dev_err(dev, "MSI vector %u: request_irq failed (%i)\n", v, rc);
            goto err;
        }
        events->parents_requested = v + 1;
    }

    dev_info(dev, "MSI %u vector(s), events through mux vector: 0x%08lx\n", events->vector_count, events->mux_events);
    // The M4 firmware is loaded by user space, usually after the driver has probed. It calls LA9310_IOCTL_EVENT_REINIT.
    rc = la9310_events_reinit(la9310_dev);
    if (rc == -ENODEV)
        dev_info(dev, "M4 firmware isn't running, events need LA9310_IOCTL_EVENT_REINIT after it's loaded\n");
    return 0;

err:
    la9310_events_destroy(la9310_dev);
    return rc;
}

void la9310_events_destroy(struct la9310_dev* la9310_dev)
{
    struct la9310_events* events = &la9310_dev->events;

    for (unsigned int v = 0; v < events->parents_requested; ++v)
    {
        if (v == LA9310_EVENT_MUX_VECTOR)
            free_irq(pci_irq_vector(la9310_dev->pdev, v), events);
        else
            free_irq(pci_irq_vector(la9310_dev->pdev, v), &events->event[v]);
    }
    events->parents_requested = 0;

    for (unsigned int i = 0; i < events->count; ++i)
    {
        struct la9310_event* ev = &events->event[i];
        if (ev->user_refs)
        {
            free_irq(ev->virq, ev);
            ev->user_refs = 0;
        }
        if (ev->virq)
        {
            irq_dispose_mapping(ev->virq);
            ev->virq = 0;
        }
        wake_up_interruptible_all(&ev->wait);
    }

    if (events->domain)
    {
        irq_domain_remove(events->domain);
        events->domain = NULL;
    }
    if (events->fwnode)
    {
        irq_domain_free_fwnode(events->fwnode);
        events->fwnode = NULL;
    }
    if (events->vector_count)
    {
        pci_free_irq_vectors(la9310_dev->pdev);
        events->vector_count = 0;
    }
    // Nobody listens any more
    if (events->shared)
        iowrite32(LA9310_EVENT_MASK_ALL, &events->shared->mask);
    events->shared = NULL;
}

// Called with mask_lock held. Writes the MSI configuration of the device's PCIe controller (the registers that the M4
// firmware reads on startup) into the firmware's cache, the same way the firmware does.
static void la9310_events_update_msi_cache(struct la9310_dev* la9310_dev)
{
    struct la9310_msi_events __iomem* shared = la9310_dev->events.shared;
    const u8 __iomem* dbi = (u8 __iomem*)la9310_dev->mem_regions[LA9310_MEM_REGION_CCSR].vaddr + PCIE_RHOM_DBI_BASE;
    const u16 flags = ioread32(dbi + PCIE_MSI_BASE) >> 16; // Message Control
    const u32 addr = (flags & PCI_MSI_FLAGS_ENABLE)
                         ? LA9310_EP_TOHOST_MSI_PHY_ADDR | (ioread32(dbi + PCIE_MSI_MSG_ADDR_OFF) & (PCIE_MSI_OB_SIZE - 1))
                         : 0;
    const u32 data = ioread32(dbi + PCIE_MSI_MSG_DATA_OFF) & 0xFFFF;
    const u32 vector_count = 1u << FIELD_GET(PCI_MSI_FLAGS_QSIZE, flags);

    if (ioread32(&shared->msi_addr) == addr && ioread32(&shared->msi_data) == data &&
        ioread32(&shared->msi_vector_count) == vector_count)
        return;

    // The firmware reads the configuration only after it finds the event unmasked
    iowrite32(LA9310_EVENT_MASK_ALL, &shared->mask);
    ioread32(&shared->mask); // flush the posted write
    iowrite32(addr, &shared->msi_addr);
    iowrite32(data, &shared->msi_data);
    iowrite32(vector_count, &shared->msi_vector_count);
    ioread32(&shared->msi_vector_count);
    dev_info(la9310_dev->dev,
        "updated the M4 firmware's MSI configuration: address 0x%08x, data 0x%04x, %u vector(s)\n",
        addr,
        data,
        vector_count);
}

int la9310_events_reinit(struct la9310_dev* la9310_dev)
{
    struct la9310_events* events = &la9310_dev->events;
    const size_t size = sizeof(struct la9310_msi_events);
    struct la9310_msi_events __iomem* shared = NULL;
    unsigned long flags;
    int rc = 0;

    // The address comes from the firmware, the interface must be within the device memory that the driver has mapped
    const u32 addr = ioread32(events->addr_reg);
    if (addr == 0)
        rc = -ENODEV;
    else
    {
        u8 __iomem* const first = (u8 __iomem*)endpoint_pa_to_va(la9310_dev, addr);
        u8 __iomem* const last = (u8 __iomem*)endpoint_pa_to_va(la9310_dev, addr + size - 1);
        if (!IS_ALIGNED(addr, sizeof(u32)) || !first || last != first + size - 1)
        {
            dev_err(la9310_dev->dev, "M4 firmware events interface address 0x%08x is not valid\n", addr);
            rc = -EINVAL;
        }
        else
            shared = (struct la9310_msi_events __iomem*)first;
    }

    spin_lock_irqsave(&events->mask_lock, flags);
    events->shared = shared;
    if (shared)
    {
        la9310_events_update_msi_cache(la9310_dev);
        // The counters start from wherever the firmware has them, only changes count as events
        for (unsigned int i = 0; i < events->count; ++i)
            events->event[i].last_counter = la9310_event_read_counter(events, i);
        la9310_events_write_mask(events);
    }
    spin_unlock_irqrestore(&events->mask_lock, flags);

    if (shared)
    {
        const u32 fw_count = ioread32(&shared->event_count);
        dev_info(la9310_dev->dev,
            "M4 firmware events interface at 0x%08x, the firmware signals %u events, enabled 0x%08lx\n",
            addr,
            fw_count,
            READ_ONCE(events->enabled_events));
        if (fw_count < events->count)
            dev_info(la9310_dev->dev, "events %u-%u never happen with this firmware\n", fw_count, events->count - 1);
    }
    return rc;
}

unsigned int la9310_event_to_irq(struct la9310_dev* la9310_dev, unsigned int event)
{
    if (event >= la9310_dev->events.count)
        return 0;
    return la9310_dev->events.event[event].virq;
}

// Called with user_lock held
static int la9310_event_user_get(struct la9310_event* ev)
{
    if (ev->user_refs == 0)
    {
        // events from before the enable don't count
        atomic_set(&ev->consumed, atomic_read(&ev->seq));
        // shared with kernel users, requesting it unmasks the event
        const int rc = request_irq(ev->virq, la9310_event_wait_handler, IRQF_SHARED, ev->name, ev);
        if (rc)
            return rc;
    }
    ++ev->user_refs;
    return 0;
}

// Called with user_lock held
static void la9310_event_user_put(struct la9310_event* ev)
{
    if (--ev->user_refs == 0)
        free_irq(ev->virq, ev); // masks the event, unless a kernel user has it too
    // waiters of the file that disabled it return
    wake_up_interruptible_all(&ev->wait);
}

int la9310_events_enable(struct la9310_file* file_ctx, uint32_t bits, uint32_t mask)
{
    struct la9310_events* events = &file_ctx->dev->events;
    unsigned long enable, disable, enabled_now = 0;
    unsigned int i;
    int rc = 0;

    if (mask & ~la9310_events_all(events))
        return -EINVAL;

    mutex_lock(&events->user_lock);
    enable = bits & mask & ~file_ctx->enabled_events;
    disable = ~bits & mask & file_ctx->enabled_events;

    for_each_set_bit(i, &disable, events->count)
    {
        clear_bit(i, &file_ctx->enabled_events);
        la9310_event_user_put(&events->event[i]);
    }

    for_each_set_bit(i, &enable, events->count)
    {
        rc = la9310_event_user_get(&events->event[i]);
        if (rc)
        {
            dev_err(file_ctx->dev->dev, "event %u: request_irq failed (%i)\n", i, rc);
            break;
        }
        set_bit(i, &file_ctx->enabled_events);
        __set_bit(i, &enabled_now);
    }

    if (rc)
    {
        for_each_set_bit(i, &enabled_now, events->count)
        {
            clear_bit(i, &file_ctx->enabled_events);
            la9310_event_user_put(&events->event[i]);
        }
    }
    mutex_unlock(&events->user_lock);
    return rc;
}

void la9310_events_file_release(struct la9310_file* file_ctx)
{
    la9310_events_enable(file_ctx, 0, file_ctx->enabled_events);
}

void la9310_events_clear(struct la9310_dev* la9310_dev, uint32_t bits, uint32_t mask)
{
    struct la9310_events* events = &la9310_dev->events;
    bits &= mask;
    for (unsigned int i = 0; i < events->count; ++i)
    {
        if (bits & BIT(i))
            atomic_set(&events->event[i].consumed, atomic_read(&events->event[i].seq));
    }
}

int la9310_event_wait(struct la9310_file* file_ctx, uint32_t event, uint32_t timeout_jiffies)
{
    if (event >= file_ctx->dev->events.count || !test_bit(event, &file_ctx->enabled_events))
        return -EINVAL;

    struct la9310_event* ev = &file_ctx->dev->events.event[event];
    long ret = wait_event_interruptible_timeout(ev->wait,
        atomic_read(&ev->seq) != atomic_read(&ev->consumed) || !test_bit(event, &file_ctx->enabled_events),
        timeout_jiffies);
    if (ret == 0)
        return -ETIMEDOUT;
    if (ret < 0)
        return ret;
    if (!test_bit(event, &file_ctx->enabled_events))
        return -ECANCELED;

    // Everything up to now is consumed. An event that arrives after this read makes the next wait return immediately,
    // and one that arrived just before it is covered too: the caller checks the stream state after it returns.
    atomic_set(&ev->consumed, atomic_read(&ev->seq));
    return 0;
}
