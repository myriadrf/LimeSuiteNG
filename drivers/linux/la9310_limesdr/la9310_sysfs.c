/* SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0)
 * Copyright 2017-2024 NXP
 */

#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/of_device.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/version.h>

#include "common_headers/la9310_host_if.h"
#include "la9310_base.h"

static ssize_t la9310_collect_ep_log(struct la9310_ep_log* ep_log, char* buf)
{
    int log_len, max_len, str_len;
    char* ep_log_str;

    log_len = 0;
    str_len = 0;
    max_len = 0;

    ep_log_str = ep_log->buf + ep_log->offset;
    max_len = ep_log->len - ep_log->offset;
    str_len = strnlen(ep_log_str, max_len);
    if (str_len)
    {
        memcpy_fromio(buf, ep_log_str, str_len);
        memset_io(ep_log_str, 0, str_len);
        ep_log->offset += str_len;
        if (ep_log->offset >= ep_log->len)
            ep_log->offset = 0;
        log_len += str_len;
        buf += str_len;

        if (max_len == str_len)
        {
            ep_log_str = ep_log->buf;
            str_len = strlen(ep_log_str);
            if (str_len)
            {
                memcpy_fromio(buf, ep_log_str, str_len);
                memset_io(ep_log_str, 0, str_len);
                log_len += str_len;
                buf += str_len;
            }
            ep_log->offset = str_len;
            if (ep_log->offset >= ep_log->len)
                ep_log->offset = 0;
        }
    }

    return log_len;
}

static ssize_t target_log_show(struct device* dev, struct device_attribute* attr, char* buf)
{
    struct la9310_dev* la9310_dev;
    struct la9310_ep_log* ep_log;
    int log_len, i;

    la9310_dev = dev_get_drvdata(dev);
    ep_log = &la9310_dev->ep_log;
    log_len = 0;

    dev_info(la9310_dev->dev, "LA9310 log buf dump, vaddr %px, offset %d\n", ep_log->buf, ep_log->offset);
    // #if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0)
    //     dma_map_single(&((struct pci_dev *)la9310_dev->pdev)->dev,
    //             ep_log->buf, ep_log->len,
    //             DMA_FROM_DEVICE);
    // #elif LINUX_VERSION_CODE >= KERNEL_VERSION(5, 5, 0)
    //     dma_map_page_attrs(&la9310_dev->pdev->dev,
    //             virt_to_page(ep_log->buf),
    //             offset_in_page(ep_log->buf), ep_log->len,
    //             DMA_FROM_DEVICE, 0);
    // #else
    //     pci_map_single(la9310_dev->pdev, ep_log->buf, ep_log->len,
    //                PCI_DMA_FROMDEVICE);
    // #endif
    log_len = la9310_collect_ep_log(ep_log, buf);
    if (log_len == 0)
    {
        for (i = 0; i < ep_log->len; i++)
        {
            if (ep_log->buf[i] != 0)
            {
                ep_log->offset = i;
                log_len = la9310_collect_ep_log(ep_log, buf);
            }
        }
    }

    dev_info(la9310_dev->dev, "log len: %d, offset : %d\n", log_len, ep_log->offset);

    return log_len;
}

static ssize_t target_log_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t count)
{

    struct la9310_dev* la9310_dev;
    int rc = 0;
    unsigned long val;
    struct la9310_ep_log* ep_log;

    la9310_dev = dev_get_drvdata(dev);
    ep_log = &la9310_dev->ep_log;

    rc = kstrtoul(buf, 0, &val);
    if (rc)
    {
        dev_err(la9310_dev->dev, "%s is not hex or decimal\n", buf);
        goto out;
    }

    if (val)
    {
        dev_err(la9310_dev->dev, "%d not valid. Write 0 to reset buffer\n", (int)val);
        goto out;
    }

    /* reset LA9310 End-Point debug log buffer */
    memset_io(ep_log->buf, 0, ep_log->len);
    ep_log->offset = 0;

    dev_info(la9310_dev->dev, "LA9310 log buf reset, vaddr %p, offset %d\n", ep_log->buf, ep_log->offset);
out:
    return strnlen(buf, count);
}

static ssize_t target_log_show_level(struct device* dev, struct device_attribute* attr, char* buf)
{
    struct la9310_dev* la9310_dev;
    struct debug_log_regs* dbg_log_regs;

    la9310_dev = dev_get_drvdata(dev);
    dbg_log_regs = &la9310_dev->hif->dbg_log_regs;
    return snprintf(buf, LA9310_DBG_LOG_MAX_STRLEN, "LA9310 log level - %d\n", readl(&dbg_log_regs->log_level));
}

static ssize_t la9310_set_ep_log_level(struct device* dev, struct device_attribute* attr, const char* buf, size_t count)
{

    struct la9310_dev* la9310_dev;
    struct debug_log_regs* dbg_log_regs;
    int rc = 0;
    unsigned long val;

    la9310_dev = dev_get_drvdata(dev);

    rc = kstrtoul(buf, 0, &val);
    if (rc)
    {
        dev_err(la9310_dev->dev, "%s is not hex or decimal\n", buf);
        goto out;
    }
    if ((val < LA9310_LOG_LEVEL_ERR) || (val > LA9310_LOG_LEVEL_ALL))
    {
        dev_err(la9310_dev->dev, "Invalid level %d, valid [%d - %d]\n", (int)val, LA9310_LOG_LEVEL_ERR, LA9310_LOG_LEVEL_ALL);
        goto out;
    }

    dbg_log_regs = &la9310_dev->hif->dbg_log_regs;
    writel(val, &dbg_log_regs->log_level);
out:
    return strnlen(buf, count);
}

static DEVICE_ATTR(target_log, S_IWUSR | S_IRUGO, target_log_show, target_log_store);
static DEVICE_ATTR(target_log_level, S_IWUSR | S_IRUGO, target_log_show_level, la9310_set_ep_log_level);

static struct attribute* la9310_sysfs_entries[] = { &dev_attr_target_log.attr, &dev_attr_target_log_level.attr, NULL };

struct attribute_group la9310_attribute_group = {
    .name = "la9310sysfs",
    .attrs = la9310_sysfs_entries,
};

int la9310_init_sysfs(struct la9310_dev* la9310_dev)
{
    int rc = 0;

    /*XXX:FIXME when WLAN registration is done. Remove
     * dev_set_drvdata(), get it through: struct ieee80211_hw *hw =
     * dev_get_drvdata(d);
     */
    dev_set_drvdata(la9310_dev->dev, la9310_dev);

    rc = sysfs_create_group(&la9310_dev->pdev->dev.kobj, &la9310_attribute_group);
    if (rc)
    {
        dev_err(la9310_dev->dev, "Failed to create sysfs group\n");
        goto out;
    }
    INIT_LIST_HEAD(&la9310_dev->host_stats.list);
    dev_info(la9310_dev->dev, "Created sysfs group %s\n", la9310_attribute_group.name);
out:
    return rc;
}

void la9310_remove_sysfs(struct la9310_dev* la9310_dev)
{
    sysfs_remove_group(&la9310_dev->pdev->dev.kobj, &la9310_attribute_group);
}
