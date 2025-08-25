/* SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0)
 * Copyright 2024 NXP
 */

#ifndef __DFE_HOST_IF_H__
#define __DFE_HOST_IF_H__


/* BBDEV queues */
#define BBDEV_IPC_DEV_ID_0      0
#define BBDEV_IPC_H2M_QUEUE     0
#define BBDEV_IPC_M2H_QUEUE     1

enum dfe_host_to_modem_msg_type {
	/* IPC operations */
	DFE_IPC_RESET = 0,
	DFE_IPC_HOST_CONNECT,
	DFE_IPC_HOST_DISCONNECT,
	/* DFE App specific */
	DFE_TDD_START,
	DFE_TDD_STOP,
	DFE_FDD_START,
	DFE_FDD_STOP,
	DFE_CFG_SCS,
	DFE_CFG_RX_ANTENNA,
	DFE_CFG_SYM_SIZE,
	DFE_CFG_RX_ADDR,
	DFE_CFG_RX_SYM_NUM,
	DFE_CFG_TX_ADDR,
	DFE_CFG_TX_SYM_NUM,
	DFE_CFG_QEC_PARAM,
	DFE_CFG_AXIQ_LB_ENABLE,
	DFE_CFG_AXIQ_LB_DISABLE,
	DFE_DEBUG_CMD,
	DFE_VSPA_DMA_BENCH,
	DFE_VSPA_PROD_HOST_BYPASS,
	DFE_TDD_CFG_PATTERN_NEW,
	DFE_TDD_SWITCH_TX,
	DFE_TDD_SWITCH_RX,
	/* cell search */
	DFE_CELL_SEARCH_START,
	DFE_CELL_SEARCH_STOP,
	DFE_CELL_ATTACH,
	DFE_TDD_TICK_KEEPALIVE,
	DFE_TDD_UL_TIME_ADVANCE,
	DFE_TDD_TIME_OFFSET_CORR,
	DFE_TDD_SFN_SLOT_SET,
	DFE_TDD_SFN_SLOT_DELTA,
	DFE_FDD_ALL_PATHS_START,
	DFE_FDD_ALL_PATHS_STOP,
	DFE_H2M_MAX_OPS,
};

enum dfe_modem_to_host_msg_type {
	/* FreeRTOS related */
	DFE_MALLOC_FAILED = DFE_H2M_MAX_OPS + 1,
	DFE_STACK_OVERFLOW,
	DFE_TASK_CREATE_ERROR,
	DFE_VSPA_ERROR,
	DFE_CM4_ERROR,
	/* DFE App specific */
	DFE_TTI_MESSAGE,
	DFE_M2H_MAX_OPS,
};

enum dfe_error_codes {
	DFE_NO_ERROR = 0,
	DFE_ERROR_GENERIC,
	DFE_ERROR_VSPA_TIMEOUT,
	DFE_INVALID_COMMAND,
	DFE_INVALID_PARAM,
	DFE_OPERATION_RUNNING,
	DFE_ERROR_CODE_MAX,
};

/* QEC-related - imported from VSPA-M4 mailbox definitions */
typedef enum {
	MBOX_IQ_CORR_FTAP0 = 0,
	MBOX_IQ_CORR_FTAP1,
	MBOX_IQ_CORR_FTAP2,
	MBOX_IQ_CORR_FTAP3,
	MBOX_IQ_CORR_FTAP4,
	MBOX_IQ_CORR_FTAP5,
	MBOX_IQ_CORR_FTAP6,
	MBOX_IQ_CORR_FTAP7,
	MBOX_IQ_CORR_FTAP8,
	MBOX_IQ_CORR_FTAP9,
	MBOX_IQ_CORR_FTAP10,
	MBOX_IQ_CORR_FTAP11,
	MBOX_IQ_CORR_DC_I,
	MBOX_IQ_CORR_DC_Q,
	MBOX_IQ_CORR_FDELAY,
	MBOX_IQ_CORR_MAX,
} mbox_qec_factor_e;

typedef enum {
	QEC_IQ_DC_OFFSET_CORR = 0,
	QEC_IQ_DC_OFFSET_PASSTHROUGH = 1,
} qec_corr_mode_e;

typedef enum {
	QEC_TX_CORR = 0,
	QEC_RX_CORR = 1,
	QEC_MAX_CORR
} qec_tx_rx_mode_e;

#define MAX_SLOTS           20 /* number of total configurable slots */

#define MAX_MSG_PAYLOAD     8  /* words */
struct dfe_msg {
	uint32_t type;          /* msg type */
	uint32_t status;        /* msg status, typically used for responses only */
	uint32_t payload[MAX_MSG_PAYLOAD];
}__attribute__((packed));

#define DFE_MSG_SIZE    	(sizeof(struct dfe_msg))

#define MAX(a,b) \
	({ __typeof__ (a) _a = (a); \
		__typeof__ (b) _b = (b); \
		_a > _b ? _a : _b; })

#define MIN(a,b) \
	({ __typeof__ (a) _a = (a); \
		__typeof__ (b) _b = (b); \
		_a < _b ? _a : _b; })

#endif
