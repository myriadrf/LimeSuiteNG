/* SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0)
 * Copyright 2021-2024 NXP
 *
 */
#ifndef __LA93XX_IPC_IOCTL__
#define __LA93XX_IPC_IOCTL__

#include <linux/ioctl.h>

#define LA9310_IPC_DEVNAME_PREFIX  "-ipc"

enum la93xx_ipc_flags {
	LA93XX_IPC_NONBLOCK = 0,
	LA93XX_IPC_BLOCK,
};

struct ipc_msg {
	int chid;
	void *addr;
	uint32_t len;
	uint8_t flags;
};

#define LA93XX_IPC_MAGIC	'R'

#define IOCTL_LA93XX_IPC_GET_SYS_MAP			_IOW(LA93XX_IPC_MAGIC, 1, struct ipc_msg *)
#define IOCTL_LA93XX_IPC_CHANNEL_REGISTER		_IOWR(LA93XX_IPC_MAGIC, 4, struct ipc_msg *)
#define IOCTL_LA93XX_IPC_CHANNEL_DEREGISTER		_IOWR(LA93XX_IPC_MAGIC, 5, struct ipc_msg *)
#define IOCTL_LA93XX_IPC_CHANNEL_RAISE_INTERRUPT	_IOW(LA93XX_IPC_MAGIC, 6, int *)

typedef struct {
	uint64_t host_phys;
	uint32_t modem_phys;
	uint32_t size;
} mem_strt_addr_t;

typedef struct {
	mem_strt_addr_t		modem_ccsrbar;
	mem_strt_addr_t		tcml_start; /* TCML meta data */
	mem_strt_addr_t		mhif_start; /* MHIF meta daat */
	mem_strt_addr_t		nlm_ops;    /* NLM operations region metadata */
	mem_strt_addr_t		hugepg_start; /* create outbound for Modem to access hugepage */
} sys_map_t;

typedef struct ipc_eventfd {
	uint32_t	efd;
	uint32_t	ipc_channel_num;
	uint32_t	msi_value;
} ipc_eventfd_t;

#endif /*__LA93XX_IPC_IOCTL__*/
