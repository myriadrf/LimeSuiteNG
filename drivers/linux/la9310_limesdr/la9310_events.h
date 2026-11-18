#ifndef LIME_LA9310_EVENTS_H
#define LIME_LA9310_EVENTS_H

#include <linux/atomic.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/wait.h>

#include "common_headers/la9310_host_if.h"

/*
 * Events that the M4 firmware signals to the host (see LA9310_EVENT_COUNT in la9310_host_if.h). They are only indexed
 * here: what an event means is agreed between the M4 firmware and the user space application, the driver doesn't
 * interpret them. The number of events is the max_events module parameter, up to LA9310_EVENT_MAX_COUNT.
 *
 * Every event is a Linux IRQ in the driver's "la9310-events" irq_domain, hwirq = event index, so kernel code can
 * request_irq() it (la9310_event_to_irq(), IRQF_SHARED to coexist with user space) and it's listed separately in
 * /proc/interrupts. How the events arrive depends on the number of MSI vectors N, the parent MSI handlers dispatch into
 * the domain:
 *  - vectors 1..N-1 carry events 1..N-1, one each, their handlers dispatch it without accessing the device,
 *  - vector 0 (LA9310_EVENT_MUX_VECTOR) carries event 0 and events >= N, its handler reads their counters and
 *    dispatches every enabled event whose counter changed. With a single MSI vector every event goes through it.
 *
 * The counters, the mask and the M4's MSI configuration cache are in struct la9310_msi_events, which the M4 firmware
 * sets up in its memory when it starts. The driver looks it up when it probes and on LA9310_IOCTL_EVENT_REINIT, which
 * user space calls after it has loaded the M4 firmware. Until then no events arrive. When it finds it, it updates the
 * MSI configuration cache, the firmware may have started with a previous driver's MSI vectors.
 *
 * Masking an event IRQ masks it on the device (la9310_msi_events.mask), the M4 doesn't signal events that nobody has
 * enabled. User space enables the events it wants with LA9310_IOCTL_EVENT_ENABLE, per open file, the driver
 * attaches its waiting handler to an event while at least one file has it enabled. Then LA9310_IOCTL_EVENT_WAIT waits
 * for it.
 */

struct la9310_events;
struct la9310_file;

struct la9310_event {
    struct la9310_events* parent;
    unsigned int hwirq;
    unsigned int virq;
    u8 last_counter; // device counter value that the mux handler saw last
    char name[16]; // IRQ name of the user space waiting handler
    // user space waiting
    unsigned int user_refs; // number of files that have the event enabled, protected by la9310_events.user_lock
    wait_queue_head_t wait;
    atomic_t seq; // number of times the event happened
    atomic_t consumed; // seq value that user space has seen
};

struct la9310_events {
    struct irq_domain* domain;
    struct fwnode_handle* fwnode;
    unsigned int count; // number of events, max_events module parameter
    u32 __iomem* addr_reg; // LA9310_SCRATCH_MSI_EVENTS_ADDR
    struct la9310_msi_events __iomem* shared; // in M4 TCMU, NULL while the M4 firmware isn't running
    unsigned int vector_count; // allocated MSI vectors, 0 if MSI isn't available
    unsigned long mux_events; // events that come through the mux vector
    unsigned long enabled_events; // unmasked events, the mux handler doesn't check others
    spinlock_t mask_lock; // enabled_events changes, shared changes and accesses
    struct mutex user_lock; // user space enables
    unsigned int parents_requested; // MSI vectors that have handlers
    struct la9310_event event[LA9310_EVENT_MAX_COUNT];
};

struct la9310_dev;

// Allocates the MSI vectors and sets up the event IRQ domain. scratch_registers: DCFG SCRATCHRW registers.
int la9310_events_init(struct la9310_dev* la9310_dev, u32 __iomem* scratch_registers);
void la9310_events_destroy(struct la9310_dev* la9310_dev);
// Looks up the events interface of the running M4 firmware, to be called after the firmware has been (re)loaded.
// Updates its MSI configuration cache, applies the event mask and resyncs the counters, pending events stay pending. Returns -ENODEV if the firmware hasn't
// published it, the device doesn't signal events then.
int la9310_events_reinit(struct la9310_dev* la9310_dev);

// Linux IRQ number of an event, for request_irq(), or 0
unsigned int la9310_event_to_irq(struct la9310_dev* la9310_dev, unsigned int event);

// Enables (bit set in bits) or disables (bit clear) the events selected by mask for the file. On error the events that
// this call would have enabled stay disabled.
int la9310_events_enable(struct la9310_file* file_ctx, uint32_t bits, uint32_t mask);
// Disables every event that the file has enabled
void la9310_events_file_release(struct la9310_file* file_ctx);

// Discards pending events (bits & mask), so the next wait blocks until a new event
void la9310_events_clear(struct la9310_dev* la9310_dev, uint32_t bits, uint32_t mask);

// Waits until the event happens, the file must have it enabled. Events that happened since the previous wait returned
// make it return immediately. Returns -ECANCELED if the file disables the event during the wait.
int la9310_event_wait(struct la9310_file* file_ctx, uint32_t event, uint32_t timeout_jiffies);

#endif // LIME_LA9310_EVENTS_H
