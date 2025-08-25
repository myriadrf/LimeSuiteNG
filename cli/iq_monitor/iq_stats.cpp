/* SPDX-License-Identifier: BSD-3-Clause
* Copyright 2022-2024 NXP
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <inttypes.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/timerfd.h>
#include <sys/mman.h>
#include <errno.h>
#include <stdbool.h>
#include <pthread.h>
#include <sched.h>
#include <assert.h>
#include <semaphore.h>
#include <signal.h>
#include <argp.h>
#include "la9310_regs.h"
// #include "imx8-host.h"
#include "stats.h"
#include "chips/LA9310/vspa_dmem_proxy.h"

#define pr_info printf
void la9310_hexdump(const void *ptr, size_t sz);
void print_vspa_stats(void);
void monitor_vspa_stats(void);
void print_vspa_trace(void);

extern uint32_t *v_vspa_dmem_proxy_ro;
extern uint32_t *v_tx_vspa_proxy_wo;
extern uint32_t *BAR0_addr;
extern uint32_t *BAR2_addr;
extern volatile uint32_t running;

static void dccivac(uint32_t* addr)
{

}

#define TX_DDR_STEP		(((t_vspa_dmem_proxy *)v_vspa_dmem_proxy_ro)->tx_state_readonly.tx_ddr_step)
#define RX_DDR_STEP		(((t_vspa_dmem_proxy *)v_vspa_dmem_proxy_ro)->tx_state_readonly.rx_ddr_step)
#define RX_DECIM			(((t_vspa_dmem_proxy *)v_vspa_dmem_proxy_ro)->tx_state_readonly.rx_decim)
#define TX_UPSMP			(((t_vspa_dmem_proxy *)v_vspa_dmem_proxy_ro)->tx_state_readonly.tx_upsmp)
#define RX_NUM_CHAN			(((t_vspa_dmem_proxy *)v_vspa_dmem_proxy_ro)->tx_state_readonly.rx_num_chan)

#define m_gbl_stats_fetch	(((t_tx_ch_host_proxy *)v_tx_vspa_proxy_wo)->gbl_stats_fetch)

static inline void __hexdump(unsigned long start, unsigned long end,
		unsigned long p, size_t sz, const unsigned char *c)
{
	while (start < end) {
		unsigned int pos = 0;
		char buf[64];
		int nl = 0;

		pos += sprintf(buf + pos, "%08lx: ", start);
		do {
			if ((start < p) || (start >= (p + sz)))
				pos += sprintf(buf + pos, "..");
			else
				pos += sprintf(buf + pos, "%02x", *(c++));
			if (!(++start & 15)) {
				buf[pos++] = '\n';
				nl = 1;
			} else {
				nl = 0;
			if (!(start & 1))
				buf[pos++] = ' ';
			if (!(start & 3))
				buf[pos++] = ' ';
			}
		} while (start & 15);
		if (!nl)
			buf[pos++] = '\n';
		buf[pos] = '\0';
		pr_info("%s", buf);
	}
}

void la9310_hexdump(const void *ptr, size_t sz)
{
	unsigned long p = (unsigned long)ptr;
	unsigned long start = p & ~15UL;
	unsigned long end = (p + sz + 15) & ~15UL;
	const unsigned char *c = reinterpret_cast<const unsigned char*>(ptr);

	__hexdump(start, end, p, sz, c);
}

t_stats local_vspa_stats[2];
uint32_t vspa_stats_tab;
volatile uint32_t *_VSPA_DMA_regs, *_GP_IN0;
void print_vspa_stats(void)
{
	int i, j;
	t_stats *cur_stats, *prev_stats;
	t_stats *host_stats =  &(((t_vspa_dmem_proxy *)v_vspa_dmem_proxy_ro)->host_stats);
	t_stats *vspa_stats =  &(((t_vspa_dmem_proxy *)v_vspa_dmem_proxy_ro)->vspa_stats);
	t_stats *app_stats =  &(((t_vspa_dmem_proxy *)v_vspa_dmem_proxy_ro)->app_stats);

	// pinpong tables to keep prev values
	if (vspa_stats_tab) {
		vspa_stats_tab = 0;
		cur_stats = &local_vspa_stats[0];
		prev_stats = &local_vspa_stats[1];
	} else {
		vspa_stats_tab = 1;
		cur_stats = &local_vspa_stats[1];
		prev_stats = &local_vspa_stats[0];
	}

	// Take snap shot
	for (i = 0; i < sizeof(t_stats)/4; i++) {
		if ((i%16) == 0) {
			dccivac((uint32_t *)(host_stats)+i);
			dccivac((uint32_t *)(vspa_stats)+i);
			dccivac((uint32_t *)(app_stats)+i);
		}
		((uint32_t *)cur_stats)[i] =  *((uint32_t*)(host_stats)+i)+ *((uint32_t*)(vspa_stats)+i)+ *((uint32_t*)(app_stats)+i);
	}
	printf("\nRX stats :");
	for (i = 0; i < STATS_RX_MAX; i++) {
		printf("\n %s", VSPA_stat_rx_string[i]);
		for (j = 0; j < RX_NUM_CHAN; j++) {
			uint32_t cur_val =  *((uint32_t*)(cur_stats->rx_stats[j])+i);
			uint32_t prev_val =  *((uint32_t*)(prev_stats->rx_stats[j])+i);

			printf("\t0x%08x", cur_val);
			if (i <= STAT_DMA_AXIQ_READ)
				printf("(%08d MB/s)", (uint32_t)((uint64_t)(cur_val-prev_val)*RX_DDR_STEP*RX_DECIM/1000000));
			else if (i == STAT_DMA_DDR_WR)
				printf("(%08d MB/s)", (uint32_t)((uint64_t)(cur_val-prev_val)*RX_DDR_STEP/1000000));
			else if (i == STAT_EXT_DMA_DDR_WR)
				printf("(%08d MB/s)", (uint32_t)((uint64_t)(cur_val-prev_val)*RX_DDR_STEP/1000000));
			else
				printf("               ");
		}
	}
	printf("\nTX stats :");
	for (i = 0; i < STATS_TX_MAX; i++) {
		uint32_t cur_val =  *((uint32_t*)(cur_stats->tx_stats)+i);
		uint32_t prev_val =  *((uint32_t*)(prev_stats->tx_stats)+i);

		printf("\n %s", VSPA_stat_tx_string[i]);
		printf("\t0x%08x", cur_val);
		if (i <= STAT_DMA_AXIQ_WRITE)
			printf("(%08d MB/s)", (uint32_t)((uint64_t)(cur_val-prev_val)*TX_DDR_STEP*TX_UPSMP/1000000));
		else if (i == STAT_DMA_DDR_RD)
			printf("(%08d MB/s)", (uint32_t)((uint64_t)(cur_val-prev_val)*TX_DDR_STEP/1000000));
		else if (i == STAT_EXT_DMA_DDR_RD)
			printf("(%08d MB/s)", (uint32_t)((uint64_t)(cur_val-prev_val)*TX_DDR_STEP/1000000));
		else
			printf("               ");
	}
	printf("\nGlobal stats :");
	for (i = 0; i < STATS_GBL_MAX; i++) {
		uint32_t cur_val =  *((uint32_t*)(cur_stats->gbl_stats)+i);
		uint32_t prev_val =  *((uint32_t*)(prev_stats->gbl_stats)+i);

		printf("\n %s", VSPA_stat_gbl_string[i]);
		printf("\t0x%08x", cur_val);
	}

	_VSPA_DMA_regs = (volatile uint32_t *)((uint64_t)BAR0_addr + VSPA_CCSR + DMA_DMEM_PRAM_ADDR);
	printf("\n\nVSPA DMA regs (IP reg 0xB0):\n");
	la9310_hexdump((void *)_VSPA_DMA_regs, 0x30);
	_GP_IN0 = (volatile uint32_t *)((uint64_t)BAR0_addr + VSPA_CCSR + GP_IN0);
	printf("VSPA AXI regs (GP_IN0-9 IPREG 0x580):\n");
	la9310_hexdump((void *)_GP_IN0, 0x30);
	printf("\n");

	return;
}

void monitor_vspa_stats(void)
{
    printf("\033[2J");
	while (running) {
		printf("\033[H");
		print_vspa_stats();
		m_gbl_stats_fetch = 1;
		sleep(1);
	}
	return;
}


