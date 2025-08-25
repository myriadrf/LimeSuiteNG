/* SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0)
 * Copyright 2017-2025 NXP
 */

#ifndef __LA9310_HOST_V2H_IF_H__
#define __LA9310_HOST_V2H_IF_H__

/* Enable V2H application support */
#if !defined(SDR) && !defined(LA9310_DFE_APP) && !defined(SEEVE)
#define NLM_ENABLE_V2H
#endif

/* Enable V2H Test Debug */
//#define DEBUG_V2H_TEST

/* Enable RUN_V2H_TEST_APP on FreeRTOS */
#if !defined(SDR) && !defined(LA9310_DFE_APP) && !defined(SEEVE)
#define RUN_V2H_TEST_APP
#endif

/* No of buffer descriptor  in ring */
#define V2H_MAX_BD			16
/*2K frame buffer */
#define V2H_MAX_SKB_BUFF_SIZE		2048
/*4K page aligned pci address */
#define V2H_PCI_ADDR_MAP		4096
/* remaining space , out of 256 head room */
#define V2H_MAX_UNSED_32BIT_WORD	60
/*start from END of Cal table*/
#define LA9310_V2H_RING_OFFSET		0x1AC00

#define MAX_SENT_RESUME			5
//#define NLM_ENABLE_V2H
/*Buffer descriptors  owner ship */
enum v2h_buff_owner {
	OWNER_VSPA = 0,
	OWNER_HOST,
};

/* Receive frame control and statistics info */
struct v2h_frame_ctrl {
	/* frame length */
	uint32_t   len;
	/* TBD , may contain power , signal strength etc */
	uint32_t   flags;
}  __attribute__( ( packed ) );

/* Buffer descriptor structure*/
struct v2h_buffer_desc {
	/*Host physcal address */
	uint64_t   host_phys_addr;
	/*LA9310 PCI address */
	uint32_t   la9310_pci_addr;
	/*Owner HOST or VSPA */
	uint32_t   owner;
}  __attribute__( ( packed ) );

/*
V2H Rx ring shared between VSPA and HOST. Both VSPA and Host
would define there own head_vspa and head_host
to access buffer ring
*/
struct v2h_ring {
	struct v2h_buffer_desc bd_info[V2H_MAX_BD];
}  __attribute__( ( packed ) );

/*
Head room to store allocated sk_buff pointer and VSPA
received frame stats and control info
*/
struct v2h_headroom {
	uint64_t skb_ptr;
	uint32_t unused[V2H_MAX_UNSED_32BIT_WORD];
	struct v2h_frame_ctrl   frm_ctrl;
}  __attribute__( ( packed ) );


#endif /* __LA9310_HOST_V2H_IF_H__ */
