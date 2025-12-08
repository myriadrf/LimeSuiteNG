/* SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0)
 * Copyright 2017, 2021-2024 NXP
 */

#ifndef _PCI_UTILITIES_H
#define _PCI_UTILITIES_H

#define PCIE_ABSERR 0x8d0
#define PCIE_ABSERR_SETTING 0x9401

#define GUL_MMAP_CCSR_OFFSET 0x8000000
#define GUL_MMAP_CCSR_SIZE 0x4000000

#define GUL_MMAP_DCSR_OFFSET 0xC000000
#define GUL_MMAP_DCSR_SIZE 0x100000

#define GUL_MMAP_PEBM_OFFSET 0x200000
#define GUL_MMAP_PEBM_SIZE 0x200000

#define LA9310_UART_SEL_GPIO 1
#if defined(SEEVE)
    #define LA9310_RESET_HANDSHAKE_POLLING_ENABLE 1
#endif

#define LA9310_LIMESDR_DRIVER_NAME "la9310_limesdr"

int la9310_limesdr_pci_register_driver(void);
void la9310_limesdr_pci_unregister_driver(void);

#endif
