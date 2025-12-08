
//#define DEBUG

#include <linux/console.h>
#include <linux/kfifo.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/tty_flip.h>
#include <linux/version.h>
#include <linux/xarray.h>

#include "la9310_uart.h"

#define DRIVER_NAME "la9310uart"

static struct resource* local_platform_get_mem_or_io(struct platform_device* dev, unsigned int num)
{
    u32 i;

    for (i = 0; i < dev->num_resources; i++)
    {
        struct resource* r = &dev->resource[i];

        if ((resource_type(r) & (IORESOURCE_MEM | IORESOURCE_IO)) && num-- == 0)
            return r;
    }
    return NULL;
}

#define UART_BASE_ADDR 0x21C0000
#define URBR1 0x500
#define UTHR1 0x500
#define ULSR1 0x505

#define TEMT (1 << 6)

struct la9310uart_port {
    struct uart_port port;
    struct timer_list timer;
    u32 index;
    char suggestedSymlink[128];
};

#define to_la9310uart_port(port) container_of(port, struct la9310uart_port, port)

static DEFINE_XARRAY_FLAGS(la9310uart_array, XA_FLAGS_ALLOC);

#define CONFIG_SERIAL_la9310uart_MAX_PORTS 16

static struct uart_driver la9310uart_driver = {
    .owner = THIS_MODULE,
    .driver_name = DRIVER_NAME,
    .dev_name = "ttyLA9310_UART",
    .major = 0,
    .minor = 0,
    .nr = CONFIG_SERIAL_la9310uart_MAX_PORTS, // maximum number of UART ports
#ifdef CONFIG_SERIAL_la9310uart_CONSOLE
    .cons = &la9310uart_console,
#endif
};

static inline uint8_t la9310_read8(void __iomem* addr)
{
    return readb(addr);
}

static inline void la9310_write8(void __iomem* addr, uint8_t val)
{
    writeb(val, addr);
}

static void la9310uart_timer(struct timer_list* t)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 16, 0)
    struct la9310uart_port* uart = from_timer(uart, t, timer);
#else
    struct la9310uart_port* uart = timer_container_of(uart, t, timer);
#endif
    struct uart_port* port = &uart->port;
    // dev_info(port->dev, "%s\n", __func__);
    unsigned char __iomem* membase = port->membase;
    unsigned int flg = TTY_NORMAL;
    int ch;
    unsigned long status;

    const uint8_t DR = 0x1; // Data ready

    int32_t iter = 128;

    while ((la9310_read8(membase + ULSR1) & DR) && iter > 0)
    {
        --iter;

        ch = la9310_read8(membase + URBR1);
        dev_dbg(port->dev, "%s, read (%i)(%02X)(%c)\n", __func__, ch, ch, (char)ch);
        port->icount.rx++;

        /* no overflow bits in status */
        if (!(uart_handle_sysrq_char(port, ch)))
            uart_insert_char(port, status, 0, ch, flg);

        tty_flip_buffer_push(&port->state->port);
    }
    // dev_info(port->dev, "%s done\n", __func__);
    mod_timer(&uart->timer, jiffies + uart_poll_timeout(port));
}

static void la9310uart_putchar(struct uart_port* port, int ch)
{
    dev_dbg(port->dev, "%s (%i)(%02X)(%c)\n", __func__, ch, ch, (char)ch);
    while ((la9310_read8(port->membase + ULSR1) & TEMT) == false)
        cpu_relax();

    la9310_write8(port->membase + UTHR1, ch);
}

static unsigned int la9310uart_tx_empty(struct uart_port* port)
{
    bool isEmpty = (la9310_read8(port->membase + ULSR1) & TEMT);
    dev_dbg(port->dev, "%s %i\n", __func__, isEmpty);
    // not really tx empty, just checking if tx is not full
    if (isEmpty)
        return TIOCSER_TEMT;

    return 0;
}

static void la9310uart_set_mctrl(struct uart_port* port, unsigned int mctrl)
{
    dev_dbg(port->dev, "%s\n", __func__);
    // modem control register is not present in LiteUART
}

static unsigned int la9310uart_get_mctrl(struct uart_port* port)
{
    dev_dbg(port->dev, "%s\n", __func__);
    return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
}

static void la9310uart_stop_tx(struct uart_port* port)
{
    dev_dbg(port->dev, "%s\n", __func__);
}

static void la9310uart_start_tx(struct uart_port* port)
{
    dev_dbg(port->dev, "%s\n", __func__);
    unsigned char ch;

// https://github.com/torvalds/linux/commit/4e2a44c1408b6a6a46122704511234f68cf012b8
// before kfifo was added to tty_port
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 10, 0)
    struct circ_buf* xmit = &port->state->xmit;
    if (unlikely(port->x_char))
    {
        la9310_write8(port->membase + UTHR1, port->x_char);
        port->icount.tx++;
        port->x_char = 0;
    }
    else if (!uart_circ_empty(xmit))
    {
        while (xmit->head != xmit->tail)
        {
            ch = xmit->buf[xmit->tail];
            xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
            port->icount.tx++;
            la9310uart_putchar(port, ch);
        }
    }

    if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
        uart_write_wakeup(port);
#else
    struct tty_port* tport = &port->state->port;
    if (unlikely(port->x_char))
    {
        la9310_write8(port->membase + UTHR1, port->x_char);
        port->icount.tx++;
        port->x_char = 0;
    }
    else if (!kfifo_is_empty(&tport->xmit_fifo))
    {
        while (kfifo_get(&tport->xmit_fifo, &ch))
        {
            port->icount.tx++;
            la9310uart_putchar(port, ch);
        }
    }

    if (kfifo_len(&tport->xmit_fifo) < WAKEUP_CHARS)
        uart_write_wakeup(port);
#endif
}

static void la9310uart_stop_rx(struct uart_port* port)
{
    dev_dbg(port->dev, "%s\n", __func__);
    struct la9310uart_port* uart = to_la9310uart_port(port);

    /* just delete timer */
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 16, 0)
    del_timer(&uart->timer);
#else
    timer_delete(&uart->timer);
#endif
}

static void la9310uart_break_ctl(struct uart_port* port, int break_state)
{
    dev_dbg(port->dev, "%s\n", __func__);
    // LiteUART doesn't support sending break signal
}

static int la9310uart_startup(struct uart_port* port)
{
    dev_dbg(port->dev, "%s\n", __func__);
    struct la9310uart_port* uart = to_la9310uart_port(port);

    // verify if UART is functioning, otherwise Rx polling will get stuck in infinite loop
    // uint32_t txfull = la9310_read8(port->membase + OFF_TXFULL);
    // uint32_t txempty = la9310_read8(port->membase + OFF_TXEMPTY);

    // if (!txfull && !txempty)
    // {
    //     dev_warn(port->dev, "UART is not present\n");
    //     return -ENODEV;
    // }

    // disable events
    // la9310_write8(port->membase + OFF_EV_ENABLE, 0);

    // loopback
    la9310_write8(port->membase + 0x504, 0x00);

    // prepare timer for polling
    timer_setup(&uart->timer, la9310uart_timer, 0);
    mod_timer(&uart->timer, jiffies + uart_poll_timeout(port));

    return 0;
}

static void la9310uart_shutdown(struct uart_port* port)
{
    dev_dbg(port->dev, "%s\n", __func__);
}

static void la9310uart_set_termios(struct uart_port* port,
    struct ktermios* new,
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
    struct ktermios* old)
#else
    const struct ktermios* old)
#endif
{
    dev_dbg(port->dev, "%s\n", __func__);
    unsigned int baud;
    unsigned long flags;

    spin_lock_irqsave(&port->lock, flags);

    // update baudrate
    baud = uart_get_baud_rate(port, new, old, 0, 460800);
    uart_update_timeout(port, new->c_cflag, baud);

    spin_unlock_irqrestore(&port->lock, flags);
}

static const char* la9310uart_type(struct uart_port* port)
{
    return DRIVER_NAME;
}

static void la9310uart_release_port(struct uart_port* port)
{
    dev_dbg(port->dev, "%s\n", __func__);
}

static int la9310uart_request_port(struct uart_port* port)
{
    dev_dbg(port->dev, "%s\n", __func__);
    return 0;
}

static void la9310uart_config_port(struct uart_port* port, int flags)
{
    dev_dbg(port->dev, "%s\n", __func__);
    /*
     * Driver core for serial ports forces a non-zero value for port type.
     * Write an arbitrary value here to accommodate the serial core driver,
     * as ID part of UAPI is redundant.
     */
    port->type = 1;
}

static int la9310uart_verify_port(struct uart_port* port, struct serial_struct* ser)
{
    dev_dbg(port->dev, "%s\n", __func__);
    if (port->type != PORT_UNKNOWN && ser->type != 1)
        return -EINVAL;

    return 0;
}

static const struct uart_ops la9310uart_ops = {
    .tx_empty = la9310uart_tx_empty,
    .set_mctrl = la9310uart_set_mctrl,
    .get_mctrl = la9310uart_get_mctrl,
    .stop_tx = la9310uart_stop_tx,
    .start_tx = la9310uart_start_tx,
    .stop_rx = la9310uart_stop_rx,
    .break_ctl = la9310uart_break_ctl,
    .startup = la9310uart_startup,
    .shutdown = la9310uart_shutdown,
    .set_termios = la9310uart_set_termios,
    .type = la9310uart_type,
    .release_port = la9310uart_release_port,
    .request_port = la9310uart_request_port,
    .config_port = la9310uart_config_port,
    .verify_port = la9310uart_verify_port,
};

static int la9310uart_uart_port_init(
    struct uart_port* uport, struct device* parent, struct resource* res, int line_id, int ctrl_id, int port_id)
{
    if (res->flags & IORESOURCE_REG)
        uport->membase = (unsigned char __iomem*)res->start;
    else
    {
        uport->membase = devm_ioremap_resource(parent, res);
        if (IS_ERR(uport->membase))
        {
            dev_err(parent, "Failed to devm_ioremap_resource\n");
            return PTR_ERR(uport->membase);
        }
    }

    //uport->attr_group = &tty_dev_attr_group; // gets copied to tty_groups

    /* values not from device tree */
    uport->dev = parent; // serial port physical parent device
    uport->iotype = UPIO_MEM;
    uport->flags = UPF_BOOT_AUTOCONF;
    uport->ops = &la9310uart_ops;
    uport->regshift = 2;
    uport->fifosize = 16;
    uport->iobase = 1;
    uport->type = PORT_UNKNOWN;

    // serial-base kernel naming: name.line:ctrl_id.port_id
    uport->line = line_id;

    // not available in ubuntu 20.04 kernel version
    // uport->ctrl_id = ctrl_id; // optional
    // uport->port_id = port_id; // optional

    // set by uart_add_one_port()
    // uport->name to ${uart_driver.dev_name}.${line_id}
    // uport->minor

    spin_lock_init(&uport->lock);
    return 0;
}

static int la9310uart_probe(struct platform_device* pdev)
{
    int ret;
    dev_info(&pdev->dev, "%s\n", __func__);

    struct resource* res = local_platform_get_mem_or_io(pdev, 0);
    if (!res)
        return -ENODEV;
    dev_dbg(&pdev->dev, "resource %s @ %llx\n", res->name, res->start);

    struct la9310uart_port* luart = devm_kzalloc(&pdev->dev, sizeof(struct la9310uart_port), GFP_KERNEL);
    if (!luart)
    {
        dev_dbg(&pdev->dev, "Failed to allocate memroy\n");
        return -ENOMEM;
    }

    struct xa_limit limit = XA_LIMIT(0, CONFIG_SERIAL_la9310uart_MAX_PORTS);
    if ((ret = xa_alloc(&la9310uart_array, &luart->index, luart, limit, GFP_KERNEL)))
    {
        dev_dbg(&pdev->dev, "Failed xa alloc\n");
        goto err_erase_id;
    }

    snprintf(luart->suggestedSymlink, sizeof(luart->suggestedSymlink), "%s", res->name);
    int line_id = luart->index;
    int ctrl_id = 0;
    int port_id = 0;
    if ((ret = la9310uart_uart_port_init(&luart->port, &pdev->dev, res, line_id, ctrl_id, port_id)))
        return ret;

    if ((ret = uart_add_one_port(&la9310uart_driver, &luart->port)))
        goto err_erase_id;

    platform_set_drvdata(pdev, &luart->port);

err_erase_id:
    xa_erase(&la9310uart_array, luart->index);

    return ret;
}

// https://github.com/torvalds/linux/commit/0edb555a65d1ef047a9805051c36922b52a38a9d
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 11, 0)
static int la9310uart_remove(struct platform_device* pdev)
#else
static void la9310uart_remove(struct platform_device* pdev)
#endif
{
    dev_dbg(&pdev->dev, "%s\n", __func__);
    struct uart_port* port = platform_get_drvdata(pdev);
    struct la9310uart_port* luart = to_la9310uart_port(port);

    platform_set_drvdata(pdev, NULL);

    uart_remove_one_port(&la9310uart_driver, port);

    xa_erase(&la9310uart_array, luart->index);
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 11, 0)
    return 0;
#endif
}

static ssize_t driver_dev_symlink_name_show(struct device* dev, struct device_attribute* attr, char* buf)
{
    struct uart_port* port = dev_get_drvdata(dev);
    struct la9310uart_port* luart = to_la9310uart_port(port);
    return snprintf(buf, PAGE_SIZE, "%s\n", luart->suggestedSymlink);
}

static DEVICE_ATTR(driver_dev_symlink_name, 0444, driver_dev_symlink_name_show, NULL);

static struct attribute* la9310uart_dev_attrs[] = { &dev_attr_driver_dev_symlink_name.attr, NULL };

static struct attribute_group dev_attr_group = {
    .attrs = la9310uart_dev_attrs,
};

static const struct attribute_group* dev_groups[] = {
    &dev_attr_group,
    NULL,
};

static const struct of_device_id la9310uart_of_match[] = { { .compatible = "la9310_limesdr,la9310uart" }, {} };
// MODULE_DEVICE_TABLE(of, la9310uart_of_match);

static struct platform_driver la9310uart_platform_driver = {
    .probe = la9310uart_probe,
    .remove = la9310uart_remove,
    .driver =
        {
            .name = DRIVER_NAME,
            .of_match_table = la9310uart_of_match,
            .dev_groups = dev_groups,
        },
};

int __init la9310uart_init(void)
{
    pr_info("la9310uart : module init\n");
    int res;
    res = uart_register_driver(&la9310uart_driver);
    if (res)
        return res;

    res = platform_driver_register(&la9310uart_platform_driver);
    if (res)
    {
        uart_unregister_driver(&la9310uart_driver);
        return res;
    }

    return 0;
}

void __exit la9310uart_exit(void)
{
    pr_info("la9310uart : module exit\n");
    platform_driver_unregister(&la9310uart_platform_driver);
    uart_unregister_driver(&la9310uart_driver);
}

module_init(la9310uart_init);
module_exit(la9310uart_exit);
MODULE_LICENSE("GPL");
