/* SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0)
 * Copyright 2022-2024 NXP
 **/
#ifndef __LA93XX_MODINFO__
#define __LA93XX_MODINFO__

#include <linux/ioctl.h>
#include <la9310_host_if.h>

#define LA9310_MODINFO_DEVNAME_PREFIX  "la9310"
#define LA9310_MODINFO_MAGIC    'R'

typedef struct modinfo_addrmap {
	uint64_t host_phy_addr;
	uint32_t modem_phy_addr;
	uint32_t size;
} modinfo_addrmap_t;

typedef struct modinfo_socrev {
	/* Modem ID to get the revision from */
	/* revision of active modem at 0 index will be returned if not set*/
	int32_t id;
	uint32_t rev;
} modinfo_socrev_t;

typedef struct modinfo_stat {
	char target_stat[1560];
} modinfo_s;

typedef struct modinfo {
	uint32_t active;
	uint32_t id;
	uint32_t rev;
	char board_name[64];
	char name[64];
	char pci_addr[64];
	char fw_name[64];
	modinfo_addrmap_t ccsr;
	modinfo_addrmap_t tcml;
	modinfo_addrmap_t tcmu;
	modinfo_addrmap_t dbg;
	modinfo_addrmap_t iqr;
	modinfo_addrmap_t ov;
	modinfo_addrmap_t nlmops;
	modinfo_addrmap_t hif;
	modinfo_addrmap_t vspa;
	modinfo_addrmap_t fw;
	modinfo_addrmap_t stdfw;
	modinfo_addrmap_t pciwin;
	modinfo_addrmap_t scratchbuf;
	modinfo_addrmap_t iqflood;
	modinfo_addrmap_t rfcal;
	int dac_mask;
	int adc_mask;
	int adc_rate_mask;
	int dac_rate_mask;
} modinfo_t;

#define IOCTL_LA93XX_MODINFO_GET                _IOW(LA9310_MODINFO_MAGIC, 1, modinfo_t *)
#define IOCTL_LA93XX_MODINFO_GET_STATS                _IOW(LA9310_MODINFO_MAGIC, 2, modinfo_s *)
#endif
