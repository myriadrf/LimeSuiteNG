/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright 2024 NXP
 */

#ifndef __L1_TRACE_H__
#define __L1_TRACE_H__

#ifdef IQMOD_RX_1T1R
    #define L1_TRACE 1
    #define L1_TRACE_SIZE 400
#endif
#ifdef IQMOD_RX_1T2R
    #define L1_TRACE 1
    #define L1_TRACE_SIZE 10
#endif
#ifdef IQMOD_RX_1T4R
    #define L1_TRACE 1
    #define L1_TRACE_SIZE 100
#endif
#if defined(IQMOD_2DEC2INT) || defined(IQMOD_4DEC4INT)
    #define L1_TRACE 1
    #define L1_TRACE_SIZE 100
#endif

#define L1_TRACE_HOST_SIZE 100
#define L1_TRACE_M7_SIZE 100

enum l1_trace_msg_type {
    L1_TRACE_MSG_DMA = 0x100,
    L1_TRACE_MSG_L1C = 0x200,
    L1_TRACE_MSG_L1APP = 0x300,
    L1_TRACE_MSG_OVLY = 0xA00,
    L1_TRACE_MSG_IPPU = 0xB00,
    L1_TRACE_MSG_ENTRY = 0xC00,
    L1_TRACE_MSG_COMMON = 0xD00,
    L1_TRACE_MSG_FIFO = 0xE00,
};

/**
 * L1 trace DMA:
 */
enum l1_trace_msg_axiq {
    /* 0x100 */ L1_TRACE_MSG_DMA_XFR = L1_TRACE_MSG_DMA,
    /* 0x101 */ L1_TRACE_MSG_DMA_CFGERR,
    /* 0x102 */ L1_TRACE_MSG_DMA_AXIQ_RX_START,
    /* 0x103 */ L1_TRACE_MSG_DMA_AXIQ_TX_START,
    /* 0x104 */ L1_TRACE_MSG_DMA_AXIQ_RX_OVER,
    /* 0x105 */ L1_TRACE_MSG_DMA_AXIQ_RX_UNDER,
    /* 0x106 */ L1_TRACE_MSG_DMA_AXIQ_TX_OVER,
    /* 0x107 */ L1_TRACE_MSG_DMA_AXIQ_TX_UNDER,
    /* 0x108 */ L1_TRACE_MSG_DMA_AXIQ_RX_COMP,
    /* 0x109 */ L1_TRACE_MSG_DMA_AXIQ_TX_COMP,
    /* 0x10A */ L1_TRACE_MSG_DMA_AXIQ_RX_XFER_ERROR,
    /* 0x10B */ L1_TRACE_MSG_DMA_AXIQ_TX_XFER_ERROR,
    /* 0x10C */ L1_TRACE_MSG_DMA_XFR_SIZE,
    /* 0x10D */ L1_TRACE_MSG_DMA_DDR_RD_START,
    /* 0x10E */ L1_TRACE_MSG_DMA_DDR_RD_COMP,
    /* 0x10F */ L1_TRACE_MSG_DMA_DDR_RD_UNDERRUN,
    /* 0x110 */ L1_TRACE_MSG_DMA_DDR_WR_START,
    /* 0x111 */ L1_TRACE_MSG_DMA_DDR_WR_COMP,
    /* 0x112 */ L1_TRACE_MSG_DMA_DDR_WR_OVERRUN,
    L1_TRACE_MSG_DMA_PTR_RST,
    L1_TRACE_MSG_DMA_COMPLETE,
    L1_TRACE_MSG_DMA_AVAILABLE,
    L1_TRACE_MSG_DMA_PENDING,
    L1_TRACE_MSG_TX_AXIQ,
    L1_TRACE_MSG_RX_AXIQ,
    L1_TRACE_MSG_TX_CONTROL,
    L1_TRACE_MSG_TX_CONFIG,
    L1_TRACE_MSG_TX_SET_BURST_SIZE,
    L1_TRACE_MSG_RX_CONTROL,
    L1_TRACE_MSG_RX_CONFIG,
    L1_TRACE_MSG_RX_SET_BURST_SIZE,
    L1_TRACE_MSG_RX_CHANNEL_SELECT,
    L1_TRACE_MSG_RX_FIFO_SET,
    L1_TRACE_MSG_TX_FIFO_SET,
    L1_TRACE_MSG_TX_BURST_END,
    L1_TRACE_MSG_TX_DMA_ALLOWED,
};

/**
 * L1 trace L1C:
 */
enum l1_trace_msg_l1c {
    /* 0x200 */ L1_TRACE_MSG_L1C_SSB_SCAN = L1_TRACE_MSG_L1C,
    /* 0x201 */ L1_TRACE_MSG_L1C_SSB_STOP,
    /* 0x202 */ L1_TRACE_MSG_L1C_DL_MSG,
    /* 0x203 */ L1_TRACE_MSG_L1C_DL_SLOTCONFIG,
    /* 0x204 */ L1_TRACE_MSG_L1C_DL_SEMISTATIC,
    /* 0x205 */ L1_TRACE_MSG_L1C_DL_SLOTUPDATE,
    /* 0x206 */ L1_TRACE_MSG_L1C_UL_START,
    /* 0x207 */ L1_TRACE_MSG_L1C_UL_MSG,
    /* 0x208 */ L1_TRACE_MSG_L1C_DL_CONSUME,
    /* 0x209 */ L1_TRACE_MSG_L1C_UL_CONSUME,
    /* 0x20A */ L1_TRACE_MSG_L1C_FREE,
};

/**
 * L1 trace L1APP:
 */
enum l1_trace_msg_l1app {
    /* 0x300 */ L1_TRACE_L1APP_FETCH_NEXT_TC = L1_TRACE_MSG_L1APP,
    /* 0x301 */ L1_TRACE_L1APP_TX_QEC_START,
    /* 0x302 */ L1_TRACE_L1APP_TX_QEC_COMP,
    /* 0x303 */ L1_TRACE_L1APP_RX_QEC_START,
    /* 0x304 */ L1_TRACE_L1APP_RX_QEC_COMP,
    /* 0x305 */ L1_TRACE_L1APP_BUFF_ID_HS,
    /* 0x306 */ L1_TRACE_L1APP_SLOT_ID,
    /* 0x307 */ L1_TRACE_L1APP_SYM_ID,
    /* 0x308 */ L1_TRACE_L1APP_LAST_SYM,
    /* 0x309 */ L1_TRACE_L1APP_BUFF_ID_OUT,
    /* 0x30a */ L1_TRACE_L1APP_RX_DEC_START,
    /* 0x30b */ L1_TRACE_L1APP_RX_DEC_COMP,
    /* 0x30c */ L1_TRACE_L1APP_RX_CMP_START,
    /* 0x30d */ L1_TRACE_L1APP_RX_CMP_COMP,
    /* 0x30e */ L1_TRACE_L1APP_TX_INTERP_START,
    /* 0x30f */ L1_TRACE_L1APP_TX_INTERP_COMP,
};

/**
 * L1 trace OVLY:
 */
enum l1_trace_msg_ovly {
    /* 0xA00 */ L1_TRACE_MSG_OVLY_SSB = L1_TRACE_MSG_OVLY,
    /* 0xA01 */ L1_TRACE_MSG_OVLY_DL,
    /* 0xA02 */ L1_TRACE_MSG_OVLY_UL,
    /* 0xA03 */ L1_TRACE_MSG_OVLY_ERR,
    /* 0xA04 */ L1_TRACE_MSG_OVLY_DL_DYNAMIC
};

/**
 * L1 trace IPPU:
 */
enum l1_trace_msg_ippu {
    /* 0xB00 */ L1_TRACE_MSG_IPPU_ERROR = L1_TRACE_MSG_IPPU
};

/**
 * L1 trace entry:
 */
enum l1_trace_msg_entry {
    /* 0xC00 */ L1_TRACE_MSG_ENTRY_IDLE = L1_TRACE_MSG_ENTRY,
    /* 0xC01 */ L1_TRACE_MSG_ENTRY_MAIN,
    /* 0xC02 */ L1_TRACE_MSG_ENTRY_SSB,
    /* 0xC03 */ L1_TRACE_MSG_ENTRY_DL_A,
    /* 0xC04 */ L1_TRACE_MSG_ENTRY_DL_B,
    /* 0xC05 */ L1_TRACE_MSG_ENTRY_DL_C,
    /* 0xC06 */ L1_TRACE_MSG_ENTRY_UL_A,
    /* 0xC07 */ L1_TRACE_MSG_ENTRY_UL_B,
    /* 0xC08 */ L1_TRACE_MSG_ENTRY_UL_C,
};

/**
 * L1 error code:
 */
enum l1_trace_msg_common {
    /* 0xD00 */ L1_TRACE_MSG_EXCEPTION = L1_TRACE_MSG_COMMON, //!< Generic exception with code.
    /* 0xD01 */ L1_TRACE_MSG_AXIS_DLA2DLB,
    /* 0xD02 */ L1_TRACE_MSG_AXIS_DLB2DLC_DLC,
    /* 0xD03 */ L1_TRACE_MSG_AXIS_DLA2DLB_SSB240,
    /* 0xD04 */ L1_TRACE_MSG_DL_B2A_UL_MSG,
};

enum l1_trace_msg_mem {
    L1_TRACE_MSG_FIFO_PUSH = L1_TRACE_MSG_FIFO,
    L1_TRACE_MSG_FIFO_POP,
    L1_TRACE_MSG_FIFO_SIZE,
    L1_TRACE_MSG_FIFO_FAULT1,
    L1_TRACE_MSG_FIFO_FAULT2,
    L1_TRACE_MSG_FIFO_FAULT3,
};

#ifndef __VSPA__

typedef struct l1_trace_data_s {
    uint64_t cnt;
    uint16_t msg;
    uint16_t dma_mask;
    uint32_t param;
} l1_trace_data_t;

typedef struct l1_trace_code_s {
    uint32_t msg;
    char text[100];
} l1_trace_code_t;

void l1_trace(uint32_t msg, uint32_t param);
void l1_trace_clear(void);

#else
    #include "ccnt.h"

typedef struct l1_trace_data_s {
    ccnt_t cnt;
    uint32_t msg;
    uint32_t param;
} l1_trace_data_t;

    #pragma cplusplus on
    #if L1_TRACE
void l1_trace(uint32_t msg, uint32_t param);
void l1_trace(uint32_t msg);
void l1_trace_nr(uint32_t msg);
void l1_trace_nr(uint32_t msg, uint32_t param);
void l1_trace_clear(void);

    #else // L1_TRACE
inline void l1_trace_clear(void)
{
}

inline void l1_trace(uint32_t msg)
{
}
inline void l1_trace(uint32_t msg, uint32_t param)
{
}
inline void l1_trace_nr(uint32_t msg)
{
}
inline void l1_trace_nr(uint32_t msg, uint32_t param)
{
}
    #endif // L1_TRACE

    #if defined(__DEBUG__)
inline void l1_trace_dbg(uint32_t msg, uint32_t param)
{
    l1_trace(msg, param);
}

inline void l1_trace_dbg(uint32_t msg)
{
    l1_trace(msg);
}
    #else // __DEBUG__
inline void l1_trace_dbg(uint32_t, uint32_t)
{
}

inline void l1_trace_dbg(uint32_t)
{
}
    #endif //__DEBUG__
    #pragma cplusplus reset
#endif

#ifndef __M7__
extern l1_trace_data_t l1_trace_data[] __attribute__((aligned(64)));
#else
extern l1_trace_data_t* l1_trace_data;
#endif
extern volatile uint32_t l1_trace_disable;

#endif // __L1_TRACE_H__
