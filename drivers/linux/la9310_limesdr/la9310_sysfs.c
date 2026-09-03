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

#define LA9310_DBG_LOG_MAX_STRLEN (100)

static ssize_t la9310_collect_ep_log(struct la9310_ep_log* ep_log, char* buf, size_t buf_size)
{
    size_t got_bytes = 0;

    // last read position by the host.
    size_t max_len = readl(&ep_log->hif->buffer_size);
    if (max_len == 0)
        return 0;
    if (max_len > 32 * 1024) // precaution against bad data, LA9310 TCMU is 64K
    {
        pr_err("Bad length %i", max_len);
        max_len = 32 * 1024;
    }

    size_t offset = readl(&ep_log->hif->host_consumed);
    if (offset >= max_len)
    {
        // corrupted offset?
        // still try to read whatever is in log's memory
        pr_err("corrupt offset\n");
        offset = 0;
    }

    if (!ep_log->buf)
        return 0;

    char* ep_log_str = ep_log->buf + offset;
    buf[buf_size - 1] = 0; // null terminate

    size_t data_been_produced = (readl(&ep_log->hif->produced) - offset) % max_len;
    if (data_been_produced >= max_len - offset)
        data_been_produced = max_len - offset; // device log overflowed

    size_t buf_remaining = data_been_produced > buf_size - 1 ? buf_size - 1 : data_been_produced;
    size_t str_len = strnlen(ep_log_str, buf_remaining);

    if (str_len)
    {
        memcpy_fromio(buf, ep_log_str, str_len);
        // memset_io(ep_log_str, 0, str_len);
        offset += str_len;
        got_bytes += str_len;
        buf += str_len;
        buf_remaining -= str_len;

        if (offset >= max_len)
        {
            offset = 0;
            ep_log_str = ep_log->buf;
            str_len = strnlen(ep_log_str, buf_remaining);
            if (str_len)
            {
                memcpy_fromio(buf, ep_log_str, str_len);
                // memset_io(ep_log_str, 0, str_len);
                got_bytes += str_len;
                buf += str_len;
                buf_remaining -= str_len;
            }
        }
    }
    writel(offset, &ep_log->hif->host_consumed);

    return got_bytes;
}

#define DCR_OFFSET 0x1e00000
static ssize_t target_log_show(struct device* dev, struct device_attribute* attr, char* buf)
{
    struct la9310_dev* la9310_dev = dev_get_drvdata(dev);

    // find log hif
    struct la9310_hif* hif = la9310_dev->hif;

    const struct la9310_ccsr_dcr* ccsr_dcr =
        (struct la9310_ccsr_dcr*)(la9310_dev->mem_regions[LA9310_MEM_REGION_CCSR].vaddr + DCR_OFFSET);
    const uint32_t mlog_addr = ccsr_dcr->scratchrw[LA9310_SCRATCH_MLOG_ADDR];
    if (mlog_addr == 0)
        return 0;

    struct la9310_ep_log ep_log;
    ep_log.hif = (volatile struct MemoryLog_hif*)endpoint_pa_to_va(la9310_dev, mlog_addr);
    if (!ep_log.hif)
    {
        dev_err(la9310_dev->dev, "LA9310 log no hif");
        return 0;
    }

    uint32_t data_addr = ioread32(&ep_log.hif->buffer_addr);
    dev_info(la9310_dev->dev, "LA9310 log adr @ 0x%x", data_addr);

    ep_log.buf = endpoint_pa_to_va(la9310_dev, data_addr);
    if (!ep_log.buf)
    {
        dev_err(la9310_dev->dev, "LA9310 log no buffer");
        return 0;
    }

    size_t bytes_got = la9310_collect_ep_log(&ep_log, buf, PAGE_SIZE);
    return bytes_got;
}

static ssize_t target_log_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t count)
{
    return 0;
    struct la9310_dev* la9310_dev;
    int rc = 0;
    unsigned long val;
    struct la9310_ep_log* ep_log;

    la9310_dev = dev_get_drvdata(dev);
    // ep_log = &la9310_dev->ep_log;

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
    // memset_io(ep_log->buf, 0, ep_log->len);

    // dev_info(la9310_dev->dev, "LA9310 log buf reset, vaddr %p, offset %d\n", ep_log->buf, ep_log->offset);
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
    dev_info(la9310_dev->dev, "Created sysfs group %s\n", la9310_attribute_group.name);
out:
    return rc;
}

void la9310_remove_sysfs(struct la9310_dev* la9310_dev)
{
    sysfs_remove_group(&la9310_dev->pdev->dev.kobj, &la9310_attribute_group);
}
