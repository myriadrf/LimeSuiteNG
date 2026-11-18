/* SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0)
 * Copyright 2017-2024 NXP
 */

#ifndef __LA9310_HOST_IF_H__
#define __LA9310_HOST_IF_H__

#define LA9310_MAX_SCRATCH_BUF_SIZE (256 * 1024 * 1024)
#define PCIE_MSI_OB_SIZE (4 * 1024) /* 4K */

/*Common Addresses and offsets*/
/* PCIe outbound window base address for address allocator */
#define PCI_OUTBOUND_WINDOW_BASE_ADDR 0xA0000000
/* Persistent host memory, owned by the driver: firmware images, logs */
#define LA9310_EP_DMA_BUF_PHYS_ADDR PCI_OUTBOUND_WINDOW_BASE_ADDR
#define LA9310_EP_TOHOST_MSI_PHY_ADDR (LA9310_EP_DMA_BUF_PHYS_ADDR + LA9310_MAX_SCRATCH_BUF_SIZE)

/* Host memory pools that user space allocates buffers from, each pool can take up to LA9310_EP_DMA_POOL_MAX_SIZE.
 * The LA9310's PCIe outbound address space is 0xA000_0000 - 0xDFFF_FFFF. */
#define LA9310_EP_DMA_POOL_MAX_SIZE (0x10000000 - PCIE_MSI_OB_SIZE)
#define LA9310_EP_CACHED_POOL_PHYS_ADDR (LA9310_EP_TOHOST_MSI_PHY_ADDR + PCIE_MSI_OB_SIZE)
#define LA9310_EP_UNCACHED_POOL_PHYS_ADDR (LA9310_EP_CACHED_POOL_PHYS_ADDR + LA9310_EP_DMA_POOL_MAX_SIZE)
#define LA9310_EP_HIF_OFFSET 0x1B000

#define LA9310_MSI_MAX_CNT 8

enum la9310_msg_unit_id {
    LA9310_MSG_UNIT_1 = 0,
    LA9310_MSG_UNIT_2,
    LA9310_MSG_UNIT3,
    LA9310_MSG_UNIT_CNT,
};

enum la9310_outbound_win {
    OUTBOUND_0,
    OUTBOUND_1,
    OUTBOUND_2,
    OUTBOUND_3,
};

#define LA9310_PERSISTENT_OUTBOUND_WIN OUTBOUND_0
#define LA9310_MSI_OUTBOUND_WIN OUTBOUND_1
#define LA9310_UNCACHED_POOL_OUTBOUND_WIN OUTBOUND_2
#define LA9310_CACHED_POOL_OUTBOUND_WIN OUTBOUND_3

#define BITMASK(n) (1 << n)

struct la9310_msg_unit {
    uint32_t msiir;
    uint32_t msir;
};

/*Scratch register for Host <> LA9310 Boot hand shake*/
#define LA9310_BOOT_HSHAKE_SCRATCH_REG 1
#define LA9310_BOOT_HSHAKE_HIF_REG 3
#define LA9310_BOOT_HSHAKE_HIF_SIZ_REG 4

/* Events that the M4 signals to the host, identified only by their index. What an event means is agreed between the M4
 * firmware and the host application, the transport below doesn't depend on it.
 *
 * The interface is struct la9310_msi_events, in the M4's TCMU (host BAR2). On startup the M4 firmware initializes it,
 * with every event masked and the MSI configuration read from its PCIe controller, and then publishes its M4 address in
 * scratch register LA9310_SCRATCH_MSI_EVENTS_ADDR. The bootloader clears that register when the M4 firmware is
 * reloaded, so the host has to look the interface up again after a reload (the driver's LA9310_IOCTL_EVENT_REINIT).
 *
 * Every event has an 8-bit counter. Only the M4 writes them, it increments the counter and then sends an MSI:
 *  - if the host enabled N > 1 MSI vectors (MSI Multiple Message Enable), events 1..N-1 are sent on the vector of the
 *    same number,
 *  - event 0 and events >= N are sent on vector LA9310_EVENT_MUX_VECTOR. Its handler finds which of them happened by
 *    comparing the counters with the values it saw last time.
 * The M4 sends MSIs with the cached MSI configuration (msi_*), it doesn't read its PCIe controller again. A host that
 * changes the MSI configuration while the firmware is running (e.g. its driver is reloaded) updates the cache, with
 * every event masked meanwhile.
 * The host owns the mask: the M4 neither counts nor sends the events whose bit is set in it. */
#define LA9310_EVENT_COUNT 16 /* events that the M4 firmware signals */
#define LA9310_EVENT_MAX_COUNT 32 /* the width of the mask */
#define LA9310_EVENT_MUX_VECTOR 0
#define LA9310_SCRATCH_MSI_EVENTS_ADDR 2 /* M4 address of struct la9310_msi_events */
#define LA9310_SCRATCH_MLOG_ADDR 9 /* M4 address of the firmware's memory log */

/* The counters are last, so they can grow with the event count */
struct la9310_msi_events {
    uint32_t mask; /* written by the host, events that the M4 doesn't signal */
    uint32_t event_count; /* LA9310_EVENT_COUNT of the firmware */
    uint32_t msi_count; /* total number of MSIs that the M4 sent, for debugging */
    /* MSI configuration cache */
    uint32_t msi_addr; /* M4 address that MSIs are written to: LA9310_EP_TOHOST_MSI_PHY_ADDR window + offset of the
                          host's MSI address in its 4K page, 0 if MSI is disabled */
    uint32_t msi_data; /* message data of vector 0, vector v is sent as msi_data | v */
    uint32_t msi_vector_count; /* vectors that the host enabled, a power of 2 */
    uint8_t counters[LA9310_EVENT_MAX_COUNT]; /* counters[i]: number of times event i happened, modulo 256 */
};

enum la9310_boot_fsm {
    NONE = 0,
    LA9310_HOST_START_CLOCK_CONFIG,
    LA9310_HOST_COMPLETE_CLOCK_CONFIG,
    LA9310_HOST_START_DRIVER_INIT,
};

struct debug_log_regs {
    uint32_t buf;
    uint32_t len;
    uint32_t log_level;
};

#define LA9310_LOG_LEVEL_ERR 1
#define LA9310_LOG_LEVEL_INFO 2
#define LA9310_LOG_LEVEL_DBG 3
#define LA9310_LOG_LEVEL_ISR 4
#define LA9310_LOG_LEVEL_ALL 5

/* XXX:NOTE: Always increment HIF version when you add anything in
 * struct la9310_hif. Following are rules for MAJOR/MINOR increment
 * MAJOR version: If a new register/register group is added.
 * MINOR version: If a new bit/flag of a register is added.
 */

#define LA9310_HIF_MAJOR_VERSION (1)
#define LA9310_HIF_MINOR_VERSION (0)

enum la9310_sw_cmd_status {
    LA9310_SW_CMD_STATUS_FREE,
    LA9310_SW_CMD_STATUS_POSTED,
    LA9310_SW_CMD_STATUS_IN_PROGRESS,
    LA9310_SW_CMD_STATUS_TIMEOUT,
    LA9310_SW_CMD_STATUS_ERROR,
    LA9310_SW_CMD_STATUS_DONE
};

enum la9310_sw_cmd { LA9310_SW_CMD_STD_FW_UPGRADE, LA9310_SW_CMD_TOTAL_COUNT };

#define LA9310_SW_CMD_DATA_SIZE (32)

struct la9310_sw_cmd_desc {
    uint32_t cmd; /* command id */
    enum la9310_sw_cmd_status status; /* desc status */
    uint32_t data[LA9310_SW_CMD_DATA_SIZE]; /* command specific data */
};

struct la9310_hif {
    uint32_t ver;
    uint32_t hif_ver;
    uint32_t status;
    uint32_t host_ready;
    uint32_t mod_ready;
    uint32_t adc_mask;
    uint32_t adc_rate_mask; /* Set indicates half_rate */
    uint32_t dac_mask;
    uint32_t dac_rate_mask; /* Set indicates half_rate */
    uint32_t iq_phys_addr;
    uint32_t iq_mem_addr;
    uint32_t iq_mem_size;
    const uint32_t modem_rf_data_size;
    const uint32_t irq_evt_regs[7]; // placeholder
    struct debug_log_regs dbg_log_regs;
    const uint32_t stats[119]; // placeholder
    const uint32_t ipc_regs[2]; // placeholder
    struct la9310_sw_cmd_desc sw_cmd_desc;
    const uint32_t rf_hif[24]; // placeholder
};

#define LA9310_VER_MAJOR(ver) ((ver >> 16) & 0xffff)
#define LA9310_VER_MINOR(ver) (ver & 0xffff)
#define LA9310_VER_MAKE(major, minor) (((major & 0xffff) << 16) | (minor & 0xffff))

#endif
