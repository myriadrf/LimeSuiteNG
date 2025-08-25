/* SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0)
 * Copyright 2024 NXP
 */

#ifndef __LA9310_TVD_IOCTL__
#define __LA9310_TVD_IOCTL__

#include <linux/ioctl.h>

#define LA9310_TVD_DEV_NAME_PREFIX "-tvd"
#define LA9310_TVD_MAGIC	'S'

/**
 * TVD IOCTLs
 *
 */
#define IOCTL_LA9310_TVD_MTD_GET_TEMP		_IOWR(LA9310_TVD_MAGIC, 2, struct tvd *)
#define IOCTL_LA9310_TVD_CTD_GET_TEMP		_IOWR(LA9310_TVD_MAGIC, 3, struct tvd *)

#define MTD_TEMP_INVALID	(-70)

struct mtdTemp{
	int8_t vspa_temp;
	int8_t dcs_temp;
};
/**
 * Enum for mtd available sites.
 */
enum mtd_temp_sites {
    VSPA_TEMP 	= 0,		/**<MTD VSPA Temperature site at 0th position. >**/
    DCS_TEMP 	= 1,		/**<MTD FECA Temperature site at 1st position. >**/
};

/**
 * Structure for tvd
 */

struct tvd {
	uint32_t			tvdid;		/**<TVD id*/
	int				dev_tvd_handle;	/**<TVD handle for fd*/
	enum mtd_temp_sites		mtd_site;
					/**<MTD site to get MTD current temp */
	struct mtdTemp			get_mtd_curr_temp;
					/**<Get MTD current temp*/
	int				get_ctd_curr_temp;
					/**<Get CTD current temp*/
};

#endif /*__LA9310_TVD_IOCTL__*/
