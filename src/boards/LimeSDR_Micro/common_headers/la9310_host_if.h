/* SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0)
 * Copyright 2017-2024 NXP
 */

#ifndef __LA9310_HOST_IF_H__
#define __LA9310_HOST_IF_H__
#include "la9310_v2h_if.h"
#include "rfic_hif.h"

#define LA9310_DEV_NAME_PREFIX	"shiva"
#define LA9310_SUBDEV_NAME(B) LA9310_DEV_NAME_PREFIX ## B

#define LA9310_MAX_SCRATCH_BUF_SIZE	(256 * 1024 * 1024)

/*Common Addresses and offsets*/
/* PCIe outbound window base address for address allocator */
#define PCI_OUTBOUND_WINDOW_BASE_ADDR	0xA0000000
#define LA9310_EP_DMA_BUF_PHYS_ADDR		PCI_OUTBOUND_WINDOW_BASE_ADDR
#define LA9310_EP_TOHOST_MSI_PHY_ADDR	(LA9310_EP_DMA_BUF_PHYS_ADDR + LA9310_MAX_SCRATCH_BUF_SIZE)
#define LA9310_IQFLOOD_PHYS_ADDR		(LA9310_EP_TOHOST_MSI_PHY_ADDR + PCIE_MSI_OB_SIZE)
#define LA9310_USER_HUGE_PAGE_PHYS_ADDR	0xC0000000
#define MAX_OUTBOUND_WINDOW		(1024 * 1024 * 1024)

#define LA9310_EP_FREERTOS_LOAD_ADDR	0x1f800000
#define LA9310_EP_BOOT_HDR_OFFSET		0x00000000
#define LA9310_EP_DMA_PHYS_OFFSET(addr) (addr - LA9310_EP_DMA_BUF_PHYS_ADDR)
#define LA9310_EP_HIF_OFFSET		0x1B000
#define LA9310_EP_IPC_OFFSET		0x1C000
#define LA9310_EP_HIF_SIZE			(4 * 1024)
#define LA9310_EP_IPC_SIZE			(16 * 1024)

/*TODO - remove hardcoding*/
#define MSI_IRQ_FLOOD_0 6
#define MSI_IRQ_FLOOD_1 7

#define LA9310_MSI_MAX_CNT		8
#define LA9310_eDMA_CHANNELS	14

enum la9310_msi_id {
	MSI_IRQ_MUX = 0,
	MSI_IRQ_V2H,
	MSI_IRQ_WDOG,
#ifndef LA9310_RESET_HANDSHAKE_POLLING_ENABLE
	MSI_IRQ_HOST_HANDSHAKE,
#else
	MSI_IRQ_UNUSED_1,
#endif
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

#define LA9310_SCRATCH_OUTBOUND_WIN    OUTBOUND_0
#define LA9310_MSI_OUTBOUND_WIN    OUTBOUND_1
#define LA9310_IPC_OUTBOUND_WIN    OUTBOUND_2
#define LA9310_V2H_OUTBOUND_WIN    OUTBOUND_3

#define LA9310_IRQ_MUX_MSG_UNIT		LA9310_MSG_UNIT_1
#define BITMASK(n)                     (1 << n)
#define LA9310_IRQ_MUX_MSG_UNIT_BIT    (0)
#ifdef __RFIC
#define LA9310_RF_SW_CMD_MSG_UNIT_BIT    ( 1 )
#endif

struct la9310_msg_unit {
	uint32_t msiir;
	uint32_t msir;
}  __attribute__( ( packed ) );

/*Scratch register for Host <> LA9310 Boot hand shake*/
#define LA9310_BOOT_HSHAKE_SCRATCH_REG	1
#define LA9310_BOOT_HSHAKE_HIF_REG         3
#define LA9310_BOOT_HSHAKE_HIF_SIZ_REG     4

#define LA9310_UPGRADE_TIMESYNC_FW 0
#if LA9310_UPGRADE_TIMESYNC_FW
enum la9310_boot_fsm {
	LA9310_HOST_NOT_READY = 0,
	LA9310_HOST_TIMESYNC_FW_LOAD,
	LA9310_HOST_TIMESYNC_FW_LOADED,
	LA9310_HOST_START_CLOCK_CONFIG,
	LA9310_HOST_COMPLETE_CLOCK_CONFIG,
	LA9310_HOST_START_DRIVER_INIT
};
#else
enum la9310_boot_fsm {
	NONE = 0,
	LA9310_HOST_START_CLOCK_CONFIG,
	LA9310_HOST_COMPLETE_CLOCK_CONFIG,
	LA9310_HOST_START_DRIVER_INIT,
};
#endif

struct la9310_boot_header {
	uint32_t preamble;
	uint32_t plugin_size;
	uint32_t plugin_offset;
	uint32_t bl_size;
	uint32_t bl_src_offset;
	uint32_t bl_dest;
	uint32_t bl_entry;
	uint32_t reserved;
}  __attribute__( ( packed ) );

#define LA9310_BOOT_HDR_BYPASS_BOOT_PLUGIN	(1 << 16)
#define LA9310_BOOT_HDR_BYPASS_BOOT_EDMA	(1 << 0)

struct irq_evt_regs {
	uint32_t irq_evt_cfg;
	uint32_t irq_evt_en;
	uint32_t irq_evt_status;
	uint32_t irq_evt_clr;
	uint32_t vspa_evt_mask;
	uint32_t ipc_evt_mask;
	uint32_t test_evt_mask;
}  __attribute__( ( packed ) );

#define LA9310_EVT_UPDATE_EVT_CFG(pIrqEvtRegs, nIrqEvts) do {	\
	pIrqEvtRegs->irq_evt_cfg &= ~(0xff00);			\
	pIrqEvtRegs->irq_evt_cfg |= (nIrqEvts << 8);		\
} while (0)

#define LA9310_EVT_SET_EVT_CFG(pIrqEvtRegs, nIrqWrds, nIrqEvts) do {	\
	pIrqEvtRegs->irq_evt_cfg = ((nIrqEvts << 8) | nIrqWrds);	\
} while (0)

#define LA9310_DBG_LOG_MAX_STRLEN	(100)

struct debug_log_regs {
	uint32_t buf;
	uint32_t len;
	uint32_t log_level;
}  __attribute__( ( packed ) );

#define LA9310_LOG_LEVEL_ERR	1
#define LA9310_LOG_LEVEL_INFO	2
#define LA9310_LOG_LEVEL_DBG	3
#define LA9310_LOG_LEVEL_ISR	4
#define LA9310_LOG_LEVEL_ALL	5

struct la9310_eDMA {
	uint8_t  status;
	uint32_t xfer_req;
	uint32_t success_interrupt;
	uint32_t error_interrupt;
	uint32_t no_callback_reg;
};

typedef enum {
       LA9310_BOOT_SRC_RSV0,
       LA9310_BOOT_SRC_RSV1,
       LA9310_BOOT_SRC_I2C,
       LA9310_BOOT_SRC_PCIE
} la9310_boot_src_t;

struct la9310_stats {
	uint32_t disabled_evt_try_cnt;
	uint32_t irq_evt_raised;
	uint32_t irq_evt_cleared;
	uint32_t irq_mux_tx_msi_cnt;
	uint32_t irq_mux_rx_msi_cnt;
	uint32_t v2h_sent_pkt;
	uint32_t v2h_dropped_pkt;
	uint32_t v2h_resumed;
	uint32_t v2h_last_sent_pkt;
	uint32_t v2h_last_dropped_pkt;
	uint32_t v2h_last_sent_pkt_resumed[MAX_SENT_RESUME];
	uint32_t v2h_last_dropped_pkt_resumed[MAX_SENT_RESUME];
	uint32_t v2h_final_ring_owner[V2H_MAX_BD];
	uint32_t v2h_backout_count;
	uint32_t avi_cm4_mbox0_tx_cnt;
	uint32_t avi_cm4_mbox1_tx_cnt;
	uint32_t avi_cm4_mbox0_rx_cnt;
	uint32_t avi_cm4_mbox1_rx_cnt;
	uint32_t avi_err_queue_full;
	uint32_t avi_intr_raised;
	uint32_t avi_mbox_intr_raised;
	uint32_t eDMA_ch_allocated;
	struct la9310_eDMA la9310_eDMA_ch[LA9310_eDMA_CHANNELS];
	uint32_t WDOG_interrupt;
	uint32_t v2h_intr_enabled;
	uint32_t dbg1;
	uint32_t dbg2;
};

struct hif_ipc_regs {
	uint32_t ipc_mdata_offset;
	uint32_t ipc_mdata_size;
};

#define CHK_HIF_MOD_RDY(hif, RDY_MASK) (hif->mod_ready & RDY_MASK)
#define SET_HIF_HOST_RDY(hif, RDY_MASK) (hif->host_ready |= RDY_MASK)
#define CHK_HIF_HOST_RDY( hif, RDY_MASK )    ( hif->host_ready & RDY_MASK )
#define SET_HIF_MOD_RDY( hif, RDY_MASK )     ( hif->mod_ready |= RDY_MASK )
#define CLEAR_HIF_MOD_RDY( hif )	(hif->mod_ready = 0)
#define CLEAR_HIF_HOST_RDY( hif )	(hif->host_ready = 0)

/* XXX:NOTE: Always increment HIF version when you add anything in
 * struct la9310_hif. Following are rules for MAJOR/MINOR increment
 * MAJOR version: If a new register/register group is added.
 * MINOR version: If a new bit/flag of a register is added.
 */

#define LA9310_HIF_MAJOR_VERSION		(1)
#define LA9310_HIF_MINOR_VERSION		(0)

enum la9310_sw_cmd_status {
        LA9310_SW_CMD_STATUS_FREE,
        LA9310_SW_CMD_STATUS_POSTED,
        LA9310_SW_CMD_STATUS_IN_PROGRESS,
        LA9310_SW_CMD_STATUS_TIMEOUT,
        LA9310_SW_CMD_STATUS_ERROR,
        LA9310_SW_CMD_STATUS_DONE
};

enum la9310_sw_cmd {
	LA9310_SW_CMD_STD_FW_UPGRADE,
	LA9310_SW_CMD_TOTAL_COUNT
};

#define LA9310_SW_CMD_DATA_SIZE		( 32 )

struct la9310_sw_cmd_desc {
        uint32_t cmd;                           /* command id */
        enum la9310_sw_cmd_status status;              /* desc status */
        uint32_t data[LA9310_SW_CMD_DATA_SIZE];      /* command specific data */
};

#define LA9310_MAX_STD_FW_COUNT		( 4 )

struct la9310_std_fw_info {
	uint32_t addr;
	uint32_t size;
};

struct la9310_std_fwupgrade_data {
	uint32_t fwcount;
	struct la9310_std_fw_info fwinfo[LA9310_MAX_STD_FW_COUNT];
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
	uint32_t modem_rf_data_size;
	struct irq_evt_regs irq_evt_regs;
	struct debug_log_regs dbg_log_regs;
	struct la9310_stats stats;
	struct hif_ipc_regs ipc_regs;
	struct la9310_sw_cmd_desc sw_cmd_desc;
	rf_host_if_t rf_hif;
}  __attribute__( ( packed ) );

#define LA9310_VER_MAJOR(ver) ((ver >> 16) & 0xffff)
#define LA9310_VER_MINOR(ver) (ver & 0xffff)
#define LA9310_VER_MAKE(major, minor) (((major & 0xffff) << 16) \
				       | (minor & 0xffff))

/* Host ready bits */
#define LA9310_HIF_STATUS_IPC_LIB_READY	(1 << 0)
#define LA9310_HIF_STATUS_VSPA_READY  (1 << 1)
#define LA9310_HIF_STATUS_IPC_APP_READY	(1 << 2)
#define LA9310_HIF_STATUS_V2H_READY   (1 << 3)
#define LA9310_HIF_STATUS_WDOG_READY  (1 << 4)

/* Modem Ready bits */
#define LA9310_HIF_MOD_READY_IPC_LIB	(1 << 0)
#define LA9310_HIF_MOD_READY_IPC_APP	(1 << 1)

/* Set IRQ_REAL_MSI_BIT to enable dedicated MSI interrupt line ,
 * and virtual irq line can be used by setting the TEST or LAST
 * EVT bits */

typedef enum {
	IRQ_EVT_IPC_CH1_BIT = 0,
	IRQ_EVT_IPC_CH2_BIT,
	IRQ_EVT_IPC_CH3_BIT,
	IRQ_EVT_IPC_CH4_BIT,
	IRQ_EVT_VSPA_BIT,
	IRQ_EVT_TEST_BIT,
	IRQ_EVT_LAST_BIT,
	IRQ_REAL_MSI_BIT
} la9310_irq_evt_bits_t;

/* This enum will specify the dedicated MSI lines shared between
 * the host and EP (**MSI_IRQ_MAX_CNT cant be used as a MSI line)
 */

#define LA9310_IRQ_EVT(bit) (1 << bit)
#define LA9310_EVT_BTS_PER_WRD	32

/* XXX:NOTE: If you add an EVT in la9310_irq_evt_bits_t, add the bit
 * in relevant mask below as well. If you add new Event group in additional to
 * the groups (VSPA, IPC, TEST) below. Define a new mask and add handling
 * in la9310_get_subdrv_virqmap() function
 */
#define IRQ_EVT_VSPA_EVT_MASK	(LA9310_IRQ_EVT(IRQ_EVT_VSPA_BIT))
#define IRQ_EVT_IPC_EVT_MASK	(LA9310_IRQ_EVT(IRQ_EVT_IPC_CH1_BIT) |	\
				 LA9310_IRQ_EVT(IRQ_EVT_IPC_CH2_BIT) |	\
				 LA9310_IRQ_EVT(IRQ_EVT_IPC_CH3_BIT) |	\
				 LA9310_IRQ_EVT(IRQ_EVT_IPC_CH4_BIT))
#define IRQ_EVT_TEST_EVT_MASK	(LA9310_IRQ_EVT(IRQ_EVT_TEST_BIT))
#define IRQ_EVT_MSI_MASK	(LA9310_IRQ_EVT(IRQ_REAL_MSI_BIT))

#endif
