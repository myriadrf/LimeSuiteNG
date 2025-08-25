/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2024 NXP
 */

#ifndef __LIB_IQ_API_H__
#define __LIB_IQ_API_H__

#ifdef __cplusplus
extern "C" {
#endif

int iq_player_init(uint32_t* v_iqflood, uint32_t iqflood_size, uint32_t* v_la9310_pci_bar2, uint32_t* dmem_proxy_vaddr);

int iq_player_init_tx(uint32_t fifo_start, uint32_t fifo_size);
int iq_player_send_data(uint32_t* v_buffer, uint32_t size);

int iq_player_init_rx(uint32_t chan, uint32_t fifo_start, uint32_t fifo_size);
int iq_player_receive_data(uint32_t chan, uint32_t* v_buffer, uint32_t max_size);

#ifdef __cplusplus
}
#endif

#endif
