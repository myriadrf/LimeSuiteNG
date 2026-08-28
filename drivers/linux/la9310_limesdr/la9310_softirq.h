#ifndef LIME_LA9310_SOFTIRQ_H
#define LIME_LA9310_SOFTIRQ_H

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/semaphore.h>

#define LA9310_SOFTIRQ_COUNT 32

// Software interrupts for signaling user space
struct la9310_softirq {
    wait_queue_head_t wait[LA9310_SOFTIRQ_COUNT];
    uint32_t irq_counter;
    uint32_t status;
    spinlock_t clearing_lock;
    uint32_t* scratch_registers;
};

struct la9310_dev;

void la9310_softirq_init(struct la9310_softirq* sirq, uint32_t* scratch_registers);
void la9310_softirq_enable(struct la9310_softirq* sirq, uint32_t bits, uint32_t mask);

void la9310_softirq_clear_local(struct la9310_dev* la9310_dev, uint32_t bits, uint32_t mask);
void la9310_softirq_clear_device(struct la9310_dev* la9310_dev, uint32_t bits, uint32_t mask);

int la9310_softirq_wait(struct la9310_softirq* sirq, uint32_t bit, uint32_t timeout_jiffies);
void la9310_softirq_signal(struct la9310_dev* la9310_dev, uint32_t bit);

#endif // LIME_LA9310_SOFTIRQ_H