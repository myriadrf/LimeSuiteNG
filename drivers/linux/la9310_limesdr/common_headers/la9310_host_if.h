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
#define LA9310_EP_DMA_BUF_PHYS_ADDR PCI_OUTBOUND_WINDOW_BASE_ADDR
#define LA9310_EP_TOHOST_MSI_PHY_ADDR (LA9310_EP_DMA_BUF_PHYS_ADDR + LA9310_MAX_SCRATCH_BUF_SIZE)

#define LA9310_IQFLOOD_PHYS_ADDR (LA9310_EP_TOHOST_MSI_PHY_ADDR + PCIE_MSI_OB_SIZE)
#define LA9310_EP_HIF_OFFSET 0x1B000

#define LA9310_MSI_MAX_CNT 8

enum la9310_msi_id {
    MSI_IRQ_MUX = 0,
    MSI_IRQ_V2H,
    MSI_IRQ_WDOG,
    MSI_IRQ_UNUSED_1,
    MSI_IRQ_IPC_1,
    MSI_IRQ_IPC_2,
    MSI_IRQ_UNUSED_2,
    MSI_IRQ_UNUSED_3,
};

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

#define LA9310_SCRATCH_OUTBOUND_WIN OUTBOUND_0
#define LA9310_MSI_OUTBOUND_WIN OUTBOUND_1
#define LA9310_IPC_OUTBOUND_WIN OUTBOUND_2
#define LA9310_V2H_OUTBOUND_WIN OUTBOUND_3

#define BITMASK(n) (1 << n)

struct la9310_msg_unit {
    uint32_t msiir;
    uint32_t msir;
} __attribute__((packed));

/*Scratch register for Host <> LA9310 Boot hand shake*/
#define LA9310_BOOT_HSHAKE_SCRATCH_REG 1
#define LA9310_BOOT_HSHAKE_HIF_REG 3
#define LA9310_BOOT_HSHAKE_HIF_SIZ_REG 4
#define LA9310_SCRATCH_SIRQ_STATUS_REG 5
#define LA9310_SCRATCH_SIRQ_COUNT_REG 6
#define LA9310_SCRATCH_SIRQ_ENABLE_REG 7
#define LA9310_SCRATCH_SIRQ_CLEAR_REG 8

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
} __attribute__((packed));

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
