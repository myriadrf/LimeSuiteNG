/* SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0)
 * Copyright 2017-2024 NXP
 */

#ifndef __LA9310_PCI_DEF_H__
#define __LA9310_PCI_DEF_H__

#define PCI_DEVICE_ID_LA9310			0x1c10
#define PCI_DEVICE_ID_LA9310_DISABLE_CIP	0x1c12
#define PCI_DEVICE_ID_LS1043A			0x8080
#define PCI_DEVICE_ID_LS1046A			0x81c0

#define MAX_LENS_NUM 16

#define FLAG_MSI_ENABLED 1

/* Interrupt modes, as used by the IntMode parameter */
#define PCI_INT_MODE_NONE		0
#define PCI_INT_MODE_LEGACY		1
#define PCI_INT_MODE_MSI		2
#define PCI_INT_MODE_MSIX		3
#define PCI_INT_MODE_MULTIPLE_MSI	4

/* Synopsis specific PCIE configuration registers */
#define PCIE_ATU_VIEWPORT		0x900
#define PCIE_ATU_REGION_INBOUND		(0x1 << 31)
#define PCIE_ATU_REGION_OUTBOUND	(0x0 << 31)
#define PCIE_ATU_REGION_INDEX3		(0x3 << 0)
#define PCIE_ATU_REGION_INDEX2		(0x2 << 0)
#define PCIE_ATU_REGION_INDEX1		(0x1 << 0)
#define PCIE_ATU_REGION_INDEX0		(0x0 << 0)
#define PCIE_ATU_CR1			0x904
#define PCIE_ATU_TYPE_MEM		(0x0 << 0)
#define PCIE_ATU_TYPE_IO		(0x2 << 0)
#define PCIE_ATU_TYPE_CFG0		(0x4 << 0)
#define PCIE_ATU_TYPE_CFG1		(0x5 << 0)
#define PCIE_ATU_CR2			0x908
#define PCIE_ATU_ENABLE			(0x1 << 31)
#define PCIE_ATU_BAR_MODE_ENABLE	(0x1 << 30)
#define PCIE_ATU_LOWER_BASE		0x90C
#define PCIE_ATU_UPPER_BASE		0x910
#define PCIE_ATU_LIMIT			0x914
#define PCIE_ATU_LOWER_TARGET		0x918
#define PCIE_ATU_BUS(x)			(((x) & 0xff) << 24)
#define PCIE_ATU_DEV(x)			(((x) & 0x1f) << 19)
#define PCIE_ATU_FUNC(x)		(((x) & 0x7) << 16)
#define PCIE_ATU_UPPER_TARGET		0x91C

#define AMBA_ERROR_RESPONSE_DEFAULT_OFF 0x8D0
#define AMBA_ERROR_RESPONSE_DEFAULT_VALUE 0x9401
/* MSIs Registers */
#define PCIE_MSI_BASE		0x50
#define PCIE_MSI_CONTROL	0x52
#define PCIE_MSI_MSG_ADDR_OFF	0x54
#define PCIE_MSI_MSG_DATA_OFF	0x5c

#define MIN_MSI_ITR_LINES	1

#define PCIE_MSI_OB_SIZE	(4 * 1024) /* 4K */


#define PREAMBLE	0xaa55aa55

/* ---------------start--------------- */
/* start :copy from m4-bootrom-devel code !!! */
#define BOOT_HDR_OFFSET		0x0		/* Boot header offset */
#define PCIE_BOOT_HDR_ADR	0x20000000	/* PCIe boot hdr address */
#define DCR_OFFSET		0x1e00000
#define MSG_UNIT_OFFSET		0x1fc0000
#define LLCP_OFFSET		0x22E0000

/*
 * ROM configures an iATU inbound translation for BAR1 to map the 16MB
 * BAR to a base of 0x1F80_0000, which is the base of TCM Code memory.
 * This BAR also encompasses TCM Data memory and VSPA DMEM.
 */
/*--------------end------------------ */

#define PCIE_RHOM_DBI_BASE	0x3400000

/*
 * ROM code sets up an eDMA transfer to copy the Boot Loader from host
 * memory using the size and adding the offset from the base of local PCIe
 * address space, 0xA000_0000. The data is stored at the location in TCM
 * memory specified by BL_DEST.
 */

#define PCIE_RHOM_DBI_SIZE	(4*1024)
#define PCIE_RHOM_HEADER_SIZE	(4*1024)
#define BL_SRC_OFFSET		0

#endif /* __LA9310_PCI_DEF_H__ */
