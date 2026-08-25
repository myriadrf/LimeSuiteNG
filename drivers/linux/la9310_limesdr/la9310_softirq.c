#include "la9310_softirq.h"

#include "la9310_base.h"

#include <linux/io.h>

#define LA9310_SCRATCH_SIRQ_STATUS_REG 5
#define LA9310_SCRATCH_SIRQ_COUNT_REG 6
#define LA9310_SCRATCH_SIRQ_ENABLE_REG 7
#define LA9310_SCRATCH_SIRQ_CLEAR_REG 8

void la9310_softirq_init(struct la9310_softirq* sirq, uint32_t* scratch_registers)
{
    sema_init(&sirq->clear_semaphore, 1);
    sirq->irq_counter = 0;
    for (int i = 0; i < LA9310_SOFTIRQ_COUNT; ++i)
    {
        init_waitqueue_head(&sirq->wait[i]);
    }
    sirq->status = 0;
    sirq->scratch_registers = scratch_registers;
}

void la9310_softirq_enable(struct la9310_softirq* sirq, uint32_t bits, uint32_t mask)
{
    sirq->status &= ~bits;
}

void la9310_softirq_clear_local(struct la9310_dev* la9310_dev, uint32_t bits, uint32_t mask)
{
    struct la9310_softirq* sirq = &la9310_dev->soft_irq;
    bits &= mask;
    sirq->status &= ~bits;
}

void la9310_softirq_clear_device(struct la9310_dev* la9310_dev, uint32_t bits, uint32_t mask)
{
    struct la9310_softirq* sirq = &la9310_dev->soft_irq;
    bits &= mask;
    uint32_t clear_bits = ioread32(&sirq->scratch_registers[LA9310_SCRATCH_SIRQ_CLEAR_REG]);
    iowrite32(clear_bits | bits, &sirq->scratch_registers[LA9310_SCRATCH_SIRQ_CLEAR_REG]);
    uint32_t status = ioread32(&sirq->scratch_registers[LA9310_SCRATCH_SIRQ_STATUS_REG]);
    status &= ~bits;
    iowrite32(status, &sirq->scratch_registers[LA9310_SCRATCH_SIRQ_STATUS_REG]);
    la9310_raise_msgunit_irq(la9310_dev, 0, 0);
}

int la9310_softirq_wait(struct la9310_softirq* sirq, uint32_t bit, uint32_t timeout_jiffies)
{
    if (bit > LA9310_SOFTIRQ_COUNT)
        return -EINVAL;

    int ret = wait_event_interruptible_timeout(sirq->wait[bit], (sirq->status & (1 << bit)), timeout_jiffies);
    if (!ret)
        return -ETIMEDOUT;
    else if (ret < 0)
        return -ERESTARTSYS;
    return 0;
}

void la9310_softirq_signal(struct la9310_dev* la9310_dev, uint32_t bit)
{
    struct la9310_softirq* sirq = &la9310_dev->soft_irq;

    sirq->status |= (1 << bit);
    wake_up_interruptible(&sirq->wait[bit]);
    la9310_softirq_clear_device(la9310_dev, (1 << bit), (1 << bit));
}