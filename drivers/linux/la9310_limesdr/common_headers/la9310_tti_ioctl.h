/* SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0)
 * @ la9310_tti_ioctl
 *
 * Copyright 2024 NXP
 */


#ifndef __LA9310_TTI_IOCTL__
#define __LA9310_TTI_IOCTL__

#include <linux/ioctl.h>

#define LA9310_TTI_DEVNAME_PREFIX "-tti"

#define LA9310_TTI_MAGIC   'T'

#define IOCTL_LA9310_MODEM_TTI_REGISTER	_IOWR(LA9310_TTI_MAGIC, 1, struct tti *)
#define IOCTL_LA9310_MODEM_TTI_DEREGISTER	_IOWR(LA9310_TTI_MAGIC, 2, struct tti *)



/** \addtogroup  HOST_LIBTTI_API
 *  @{
 */

/**
 * Maximum number of TTI instances
 */
#define MAX_TTI_COUNT	2

#define MAX_MODEM	4
/**
 * A structure holds modem TTI information
 */
struct tti {
	uint32_t	ttid;/**< Modem tti id 0|1 */
	int32_t	dev_tti_handle;/**< file descriptor of modem tti device */
	int32_t tti_eventfd;
};

struct la9310_ccsr_info {
	off_t offset;
	long size;
};

/**
 * @brief Register modem timed interrupt event
 * @param[in] tti_t refrence to struct tti
 * @param[in] modem_id (0|1)
 * @param[in] tti_event_flag for tti operating mode event/normal
 * @return On success 0. On failure negative error number
 */
int modem_tti_register(struct tti *tti_t, int modem_id, int tti_event_flag);

/**
 * @brief Unregister interrupt for modem timed interrupt event and free IRQ
 * @param[in] tti_t refrence to struct tti
 * @return On success 0. On failure negative error number
 */
int modem_tti_deregister(struct tti *tti_t);


/**
 * @brief To get the modem TBGen freq info
 * @param[in] modem_id modem_id
 * @return On success 0. On failure negative error number
 */
void *get_ccsr_addr(int modem_id);
/** @} */
#endif /*__la9310_TTI_IOCTL__*/
