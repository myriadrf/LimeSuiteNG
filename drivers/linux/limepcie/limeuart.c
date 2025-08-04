
// #define DEBUG

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

#include "litex.h"

#include "version.h"
#include "limeuart.h"

// MODULE_VERSION(LIMEPCIE_VERSION);
// MODULE_AUTHOR("Lime Microsystems");
// MODULE_DESCRIPTION("LimeUART serial driver");
// MODULE_LICENSE("GPL");
// MODULE_ALIAS("platform: limeuart");

#define DRIVER_NAME "limeuart"

static struct resource *local_platform_get_mem_or_io(struct platform_device *dev, unsigned int num)
{
    u32 i;

    for (i = 0; i < dev->num_resources; i++)
    {
        struct resource *r = &dev->resource[i];

        if ((resource_type(r) & (IORESOURCE_MEM | IORESOURCE_IO)) && num-- == 0)
            return r;
    }
    return NULL;
}

/*
 * CSRs definitions (base address offsets + width)
 *
 * The definitions below are true for LiteX SoC configured for 8-bit CSR Bus,
 * 32-bit aligned.
 *
 * Supporting other configurations might require new definitions or a more
 * generic way of indexing the LiteX CSRs.
 *
 * For more details on how CSRs are defined and handled in LiteX, see comments
 * in the LiteX SoC Driver: drivers/soc/litex/litex_soc_ctrl.c
 */
#define OFF_RXTX 0x00
#define OFF_TXFULL 0x04
#define OFF_RXEMPTY 0x08
#define OFF_TXEMPTY 0x18
#define OFF_RXFULL 0x1c
#define OFF_EV_STATUS 0x0c
#define OFF_EV_PENDING 0x10
#define OFF_EV_ENABLE 0x14

/* events */
#define EV_TX 0x1
#define EV_RX 0x2

struct limeuart_port {
    struct uart_port port;
    struct timer_list timer;
    u32 index;
    char suggestedSymlink[128];
};

#define to_limeuart_port(port) container_of(port, struct limeuart_port, port)

static DEFINE_XARRAY_FLAGS(limeuart_array, XA_FLAGS_ALLOC);

#ifdef CONFIG_SERIAL_LIMEUART_CONSOLE
static struct console limeuart_console;
#endif

#ifndef CONFIG_SERIAL_LIMEUART_MAX_PORTS
    #define CONFIG_SERIAL_LIMEUART_MAX_PORTS 16
#endif

static struct uart_driver limeuart_driver = {
    .owner = THIS_MODULE,
    .driver_name = DRIVER_NAME,
    .dev_name = "ttyLimeUART",
    .major = 0,
    .minor = 0,
    .nr = CONFIG_SERIAL_LIMEUART_MAX_PORTS, // maximum number of UART ports
#ifdef CONFIG_SERIAL_LIMEUART_CONSOLE
    .cons = &limeuart_console,
#endif
};

static void limeuart_timer(struct timer_list *t)
{
    struct limeuart_port *uart = from_timer(uart, t, timer);
    struct uart_port *port = &uart->port;
    unsigned char __iomem *membase = port->membase;
    unsigned int flg = TTY_NORMAL;
    int ch;
    unsigned long status;

    while ((status = !litex_read8(membase + OFF_RXEMPTY)) == 1)
    {
        ch = litex_read8(membase + OFF_RXTX);
        port->icount.rx++;

        /* no overflow bits in status */
        if (!(uart_handle_sysrq_char(port, ch)))
            uart_insert_char(port, status, 0, ch, flg);

        tty_flip_buffer_push(&port->state->port);
    }

    mod_timer(&uart->timer, jiffies + uart_poll_timeout(port));
}

static void limeuart_putchar(struct uart_port *port, int ch)
{
    dev_dbg(port->dev, "%s\n", __func__);
    while (litex_read8(port->membase + OFF_TXFULL))
        cpu_relax();

    litex_write8(port->membase + OFF_RXTX, ch);
}

static unsigned int limeuart_tx_empty(struct uart_port *port)
{
    dev_dbg(port->dev, "%s\n", __func__);
    /* not really tx empty, just checking if tx is not full */
    if (!litex_read8(port->membase + OFF_TXFULL))
        return TIOCSER_TEMT;

    return 0;
}

static void limeuart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
    dev_dbg(port->dev, "%s\n", __func__);
    // modem control register is not present in LiteUART
}

static unsigned int limeuart_get_mctrl(struct uart_port *port)
{
    dev_dbg(port->dev, "%s\n", __func__);
    return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
}

static void limeuart_stop_tx(struct uart_port *port)
{
    dev_dbg(port->dev, "%s\n", __func__);
}

static void limeuart_start_tx(struct uart_port *port)
{
    dev_dbg(port->dev, "%s\n", __func__);
    unsigned char ch;

// https://github.com/torvalds/linux/commit/4e2a44c1408b6a6a46122704511234f68cf012b8
// before kfifo was added to tty_port
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 18, 0)
    struct circ_buf *xmit = &port->state->xmit;
    if (unlikely(port->x_char))
    {
        litex_write8(port->membase + OFF_RXTX, port->x_char);
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
            limeuart_putchar(port, ch);
        }
    }

    if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
        uart_write_wakeup(port);
#else
    struct tty_port *tport = &port->state->port;
    if (unlikely(port->x_char))
    {
        litex_write8(port->membase + OFF_RXTX, port->x_char);
        port->icount.tx++;
        port->x_char = 0;
    }
    else if (!kfifo_is_empty(&tport->xmit_fifo))
    {
        while (kfifo_get(&tport->xmit_fifo, &ch))
        {
            port->icount.tx++;
            limeuart_putchar(port, ch);
        }
    }

    if (kfifo_len(&tport->xmit_fifo) < WAKEUP_CHARS)
        uart_write_wakeup(port);
#endif
}

static void limeuart_stop_rx(struct uart_port *port)
{
    dev_dbg(port->dev, "%s\n", __func__);
    struct limeuart_port *uart = to_limeuart_port(port);

    /* just delete timer */
    del_timer(&uart->timer);
}

static void limeuart_break_ctl(struct uart_port *port, int break_state)
{
    dev_dbg(port->dev, "%s\n", __func__);
    // LiteUART doesn't support sending break signal
}

static int limeuart_startup(struct uart_port *port)
{
    dev_dbg(port->dev, "%s\n", __func__);
    struct limeuart_port *uart = to_limeuart_port(port);

    // verify if UART is functioning, otherwise Rx polling will get stuck in infinite loop
    uint32_t txfull = litex_read8(port->membase + OFF_TXFULL);
    uint32_t txempty = litex_read8(port->membase + OFF_TXEMPTY);

    if (!txfull && !txempty)
    {
        dev_warn(port->dev, "UART is not present\n");
        return -ENODEV;
    }

    // disable events
    litex_write8(port->membase + OFF_EV_ENABLE, 0);

    // prepare timer for polling
    timer_setup(&uart->timer, limeuart_timer, 0);
    mod_timer(&uart->timer, jiffies + uart_poll_timeout(port));

    return 0;
}

static void limeuart_shutdown(struct uart_port *port)
{
    dev_dbg(port->dev, "%s\n", __func__);
}

static void limeuart_set_termios(struct uart_port *port,
    struct ktermios *new,
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 1, 0)
    struct ktermios *old)
#else
    const struct ktermios *old)
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

static const char *limeuart_type(struct uart_port *port)
{
    return DRIVER_NAME;
}

static void limeuart_release_port(struct uart_port *port)
{
    dev_dbg(port->dev, "%s\n", __func__);
}

static int limeuart_request_port(struct uart_port *port)
{
    dev_dbg(port->dev, "%s\n", __func__);
    return 0;
}

static void limeuart_config_port(struct uart_port *port, int flags)
{
    dev_dbg(port->dev, "%s\n", __func__);
    /*
     * Driver core for serial ports forces a non-zero value for port type.
     * Write an arbitrary value here to accommodate the serial core driver,
     * as ID part of UAPI is redundant.
     */
    port->type = 1;
}

static int limeuart_verify_port(struct uart_port *port, struct serial_struct *ser)
{
    dev_dbg(port->dev, "%s\n", __func__);
    if (port->type != PORT_UNKNOWN && ser->type != 1)
        return -EINVAL;

    return 0;
}

static const struct uart_ops limeuart_ops = {
    .tx_empty = limeuart_tx_empty,
    .set_mctrl = limeuart_set_mctrl,
    .get_mctrl = limeuart_get_mctrl,
    .stop_tx = limeuart_stop_tx,
    .start_tx = limeuart_start_tx,
    .stop_rx = limeuart_stop_rx,
    .break_ctl = limeuart_break_ctl,
    .startup = limeuart_startup,
    .shutdown = limeuart_shutdown,
    .set_termios = limeuart_set_termios,
    .type = limeuart_type,
    .release_port = limeuart_release_port,
    .request_port = limeuart_request_port,
    .config_port = limeuart_config_port,
    .verify_port = limeuart_verify_port,
};

static int limeuart_uart_port_init(
    struct uart_port *uport, struct device *parent, struct resource *res, int line_id, int ctrl_id, int port_id)
{
    if (res->flags & IORESOURCE_REG)
        uport->membase = (unsigned char __iomem *)res->start;
    else
    {
        uport->membase = devm_ioremap_resource(parent, res);
        if (IS_ERR(uport->membase))
        {
            dev_dbg(parent, "Failed to devm_ioremap_resource\n");
            return PTR_ERR(uport->membase);
        }
    }

    //uport->attr_group = &tty_dev_attr_group; // gets copied to tty_groups

    /* values not from device tree */
    uport->dev = parent; // serial port physical parent device
    uport->iotype = UPIO_MEM;
    uport->flags = UPF_BOOT_AUTOCONF;
    uport->ops = &limeuart_ops;
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

static int limeuart_probe(struct platform_device *pdev)
{
    int ret;
    dev_dbg(&pdev->dev, "%s\n", __func__);

    struct resource *res = local_platform_get_mem_or_io(pdev, 0);
    if (!res)
        return -ENODEV;
    dev_dbg(&pdev->dev, "resource %s @ %llx\n", res->name, res->start);

    struct limeuart_port *luart = devm_kzalloc(&pdev->dev, sizeof(struct limeuart_port), GFP_KERNEL);
    if (!luart)
    {
        dev_dbg(&pdev->dev, "Failed to allocate memroy\n");
        return -ENOMEM;
    }

    struct xa_limit limit = XA_LIMIT(0, CONFIG_SERIAL_LIMEUART_MAX_PORTS);
    if ((ret = xa_alloc(&limeuart_array, &luart->index, luart, limit, GFP_KERNEL)))
    {
        dev_dbg(&pdev->dev, "Failed xa alloc\n");
        goto err_erase_id;
    }

    snprintf(luart->suggestedSymlink, sizeof(luart->suggestedSymlink), "%s", res->name);
    int line_id = luart->index;
    int ctrl_id = 0;
    int port_id = 0;
    if ((ret = limeuart_uart_port_init(&luart->port, &pdev->dev, res, line_id, ctrl_id, port_id)))
        return ret;

    if ((ret = uart_add_one_port(&limeuart_driver, &luart->port)))
        goto err_erase_id;

    platform_set_drvdata(pdev, &luart->port);

    return 0;

err_erase_id:
    xa_erase(&limeuart_array, luart->index);

    return ret;
}

// https://github.com/torvalds/linux/commit/0edb555a65d1ef047a9805051c36922b52a38a9d
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 11, 0)
static int limeuart_remove(struct platform_device *pdev)
#else
static void limeuart_remove(struct platform_device *pdev)
#endif
{
    dev_dbg(&pdev->dev, "%s\n", __func__);
    struct uart_port *port = platform_get_drvdata(pdev);
    struct limeuart_port *luart = to_limeuart_port(port);

    platform_set_drvdata(pdev, NULL);

    uart_remove_one_port(&limeuart_driver, port);

    xa_erase(&limeuart_array, luart->index);
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 11, 0)
    return 0;
#endif
}

static ssize_t driver_dev_symlink_name_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct uart_port *port = dev_get_drvdata(dev);
    struct limeuart_port *luart = to_limeuart_port(port);
    return snprintf(buf, PAGE_SIZE, "%s\n", luart->suggestedSymlink);
}

static DEVICE_ATTR(driver_dev_symlink_name, 0444, driver_dev_symlink_name_show, NULL);

static struct attribute *limeuart_dev_attrs[] = {&dev_attr_driver_dev_symlink_name.attr, NULL};

static struct attribute_group dev_attr_group = {
    .attrs = limeuart_dev_attrs,
};

static const struct attribute_group *dev_groups[] = {
    &dev_attr_group,
    NULL,
};

static const struct of_device_id limeuart_of_match[] = {{.compatible = "limepcie,limeuart"}, {}};
// MODULE_DEVICE_TABLE(of, limeuart_of_match);

static struct platform_driver limeuart_platform_driver = {
    .probe = limeuart_probe,
    .remove = limeuart_remove,
    .driver =
        {
            .name = DRIVER_NAME,
            .of_match_table = limeuart_of_match,
            .dev_groups = dev_groups,
        },
};

#ifdef CONFIG_SERIAL_LIMEUART_CONSOLE

static void limeuart_console_write(struct console *co, const char *s, unsigned int count)
{
    dev_dbg(port->dev, "%s\n", __func__);
    struct limeuart_port *uart;
    struct uart_port *port;
    unsigned long flags;

    uart = (struct limeuart_port *)xa_load(&limeuart_array, co->index);
    port = &uart->port;

    spin_lock_irqsave(&port->lock, flags);
    uart_console_write(port, s, count, limeuart_putchar);
    spin_unlock_irqrestore(&port->lock, flags);
}

static int limeuart_console_setup(struct console *co, char *options)
{
    dev_dbg(port->dev, "%s\n", __func__);
    struct limeuart_port *uart;
    struct uart_port *port;
    int baud = 115200;
    int bits = 8;
    int parity = 'n';
    int flow = 'n';

    uart = (struct limeuart_port *)xa_load(&limeuart_array, co->index);
    if (!uart)
        return -ENODEV;

    port = &uart->port;
    if (!port->membase)
        return -ENODEV;

    if (options)
        uart_parse_options(options, &baud, &parity, &bits, &flow);

    return uart_set_options(port, co, baud, parity, bits, flow);
}

static struct console limeuart_console = {
    .name = "limeuart",
    .write = limeuart_console_write,
    .device = uart_console_device,
    .setup = limeuart_console_setup,
    .flags = CON_PRINTBUFFER,
    .index = -1,
    .data = &limeuart_driver,
};

static int __init limeuart_console_init(void)
{
    pr_info(" limeuart_console_init \n");
    register_console(&limeuart_console);

    return 0;
}
console_initcall(limeuart_console_init);

static void early_limeuart_write(struct console *console, const char *s, unsigned int count)
{
    dev_dbg(port->dev, "%s\n", __func__);
    struct earlycon_device *device = console->data;
    struct uart_port *port = &device->port;

    uart_console_write(port, s, count, limeuart_putchar);
}

static int __init early_limeuart_setup(struct earlycon_device *device, const char *options)
{
    dev_dbg(port->dev, "%s\n", __func__);
    if (!device->port.membase)
        return -ENODEV;

    device->con->write = early_limeuart_write;
    return 0;
}

OF_EARLYCON_DECLARE(limeuart, "limepcie,limeuart", early_limeuart_setup);
#endif /* CONFIG_SERIAL_LIMEUART_CONSOLE */

int __init limeuart_init(void)
{
    pr_info("limeuart : module init v%s-g%s\n", LIMEPCIE_VERSION, LIMEPCIE_GIT_HASH);
    int res;
    res = uart_register_driver(&limeuart_driver);
    if (res)
        return res;

    res = platform_driver_register(&limeuart_platform_driver);
    if (res)
    {
        uart_unregister_driver(&limeuart_driver);
        return res;
    }

    return 0;
}

void __exit limeuart_exit(void)
{
    pr_info("limeuart : module exit\n");
    platform_driver_unregister(&limeuart_platform_driver);
    uart_unregister_driver(&limeuart_driver);
}

// module_init(limeuart_init);
// module_exit(limeuart_exit);
