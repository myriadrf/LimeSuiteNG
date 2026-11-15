#define _POSIX_C_SOURCE 200809L

#include "vspa_recovery.h"

#include <stdarg.h>
#include <stdio.h>
#include <time.h>

// --- M4 debug proxy (arm_m4 src/vspa_debug.h) ------------------------------------------------------------------------

#define SCRATCHRW32_OFFSET 0x1E0027Cu // BAR0 offset, holds the proxy struct's M4 address
#define MSIIR3_OFFSET 0x1FC0010u // BAR0 offset, any write interrupts the M4 (MSG3)
#define MSIR3_OFFSET 0x1FC0014u // BAR0 offset, clear on read
#define TCML_BASE 0x1F800000u // M4 address that BAR1 maps

#define PROXY_MAGIC 0x47424456u // "VDBG"
#define PROXY_VERSION 1u
#define PROXY_OP_READ 1u
#define PROXY_OP_WRITE 2u
#define PROXY_FLAG_FIXED (1u << 8) // burst on one register

// struct vspa_dbg_proxy word indices
#define P_MAGIC 0
#define P_VERSION 1
#define P_SELF 2
#define P_MAX_WORDS 3
#define P_CMD 4
#define P_OFFSET 5
#define P_COUNT 6
#define P_STATUS 7
#define P_DATA 8

#define PROXY_TIMEOUT_NS 100000000ll

// --- VSPA debug block (byte offsets), the lower 2 KB is a gateway to the VSPA IP registers ---------------------------

#define IP_CONTROL 0x008u
#define IP_STATUS 0x010u
#define IP_VCPU_GO_ADDR 0x180u
#define CONTROL_REG_MASK (~0x000100FFu) // bits that keep their value when the host writes CONTROL
#define CONTROL_VCPU_RESET (1u << 16)
#define CONTROL_HOST_GO (1u << 0)
#define STATUS_BUSY (1u << 8)

#define GDBEN 0x800u
#define RCR 0x804u
#define RCSTATUS 0x808u
#define HACR 0x83Cu // which events halt the core
#define RAVAP 0x870u // portal address: a_mode << 24 | word address << 2
#define RAVFD 0x874u // portal data, no auto-increment
#define RAVID 0x878u // portal data, auto-increment
#define DVR 0x87Cu

#define GDBEN_IDBG_EN (1u << 0)
#define RCR_RESUME (1u << 1)
#define RCR_FORCE_HALT (1u << 2)
#define RCR_HALT_CYC_COUNTER (1u << 8)
#define HACR_EVENTS ((1u << 29) | (1u << 28) | 0xFFu) // software breakpoint, illegal instruction, comparators
#define DVR_DBG_DBGEN (1u << 13)

#define AMODE_ARCH 0u // architectural registers
#define AMODE_PRAM 3u // VCPU PRAM, 64-bit instructions written as pairs of words
#define ARCH_FPC 0u // prefetch PC, counts 32-bit words
#define PC_BYTES 4u
#define PRAM_SIZE 0x8000u

#define HALT_TIMEOUT_NS 100000000ll
#define IDLE_TIMEOUT_NS 100000000ll
#define ATTEMPTS 3

// Idle stub, assembled with vspa2-as, one { low word, high word } pair per instruction in PRAM order.
// It doesn't jump and doesn't use any address of its own, so it runs from wherever it's placed.
static const uint32_t idle_stub[][2] = {
    // padding before the prefetch PC
    { 0x00000000, 0x60000000 }, // fnop
    { 0x00000000, 0x60000000 }, // fnop
    // the core continues fetching here
    { 0x00000000, 0x60000000 }, // fnop
    { 0x00000000, 0x60000000 }, // fnop
    { 0x00000000, 0x0280C030 }, // mvip 0x180 >> 2, 0               VCPU_GO_ADDR = 0
    { 0x00000000, 0x02801430 }, // mvip 0x28 >> 2, 0                EXT_GO_ENA: no external go events
    { 0xFFFFFFFC, 0x02801633 }, // mvip 0x2C >> 2, 0xFFFFFFFF       EXT_GO_STAT: clear pending
    { 0x00000000, 0x02803430 }, // mvip 0x68 >> 2, 0                AXISLV_GOEN0
    { 0x00000000, 0x02803630 }, // mvip 0x6C >> 2, 0                AXISLV_GOEN1
    { 0xFFFFFFFC, 0x02803033 }, // mvip 0x60 >> 2, 0xFFFFFFFF       AXISLV_FLAGS0: clear
    { 0xFFFFFFFC, 0x02803233 }, // mvip 0x64 >> 2, 0xFFFFFFFF       AXISLV_FLAGS1: clear
    { 0xFFFFFFFC, 0x02800E33 }, // mvip 0x1C >> 2, 0xFFFFFFFF       HOST_VCPU_FLAGS0: clear
    { 0xFFFFFFFC, 0x02801033 }, // mvip 0x20 >> 2, 0xFFFFFFFF       HOST_VCPU_FLAGS1: clear
    // an unread host message keeps its HOST_MSG_GO pending, and would wake the next firmware with it
    { 0xFFFFFFFC, 0x02A32803 }, // mvip g0, 0x650 >> 2, 0xFFFFFFFF  empty host mailbox #0 MSB
    { 0xFFFFFFFC, 0x02A32C03 }, // mvip g0, 0x658 >> 2, 0xFFFFFFFF  empty host mailbox #1 MSB
    { 0xFFFFFFFC, 0x02A32A03 }, // mvip g0, 0x654 >> 2, 0xFFFFFFFF  empty host mailbox #0 LSB
    { 0xFFFFFFFC, 0x02A32E03 }, // mvip g0, 0x65C >> 2, 0xFFFFFFFF  empty host mailbox #1 LSB
    { 0x00000000, 0x02800230 }, // mvip 0x4 >> 2, 0                 SWVERSION = 0: the host sends no reset request
    // like iqplayer's terminate(): with host mailbox GO left enabled, the loader's boot message would start the new
    // image before HOST_GO, and leave a mailbox GO event without a message
    { 0x03C0038C, 0x02400000 }, // mv   g0, 0x00F000E3
    { 0x3FC06BCC, 0x02800400 }, // mvip 0x8 >> 2, g0, 0x0FF01AF3    CONTROL: clear go events, mailbox GO disabled
    { 0x03800000, 0x80000000 }, // nop
    { 0x00000000, 0x00400000 }, // done
    { 0x00000000, 0x60000000 }, // fnop
    { 0x00000000, 0x60000000 }, // fnop
};
#define IDLE_STUB_WORDS (sizeof(idle_stub) / sizeof(idle_stub[0][0]))
#define IDLE_STUB_PADDING 16u // bytes before the prefetch PC

static void log_message(const struct vspa_recovery* recovery, enum vspa_loader_log_level level, const char* format, ...)
{
    if (!recovery->log)
        return;
    char message[192];
    va_list args;
    va_start(args, format);
    vsnprintf(message, sizeof(message), format, args);
    va_end(args);
    recovery->log(recovery->user, level, message);
}

static int64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000000ll + ts.tv_nsec;
}

static void pause_us(long us)
{
    const struct timespec ts = { 0, us * 1000l };
    nanosleep(&ts, NULL);
}

static uint32_t ccsr_read(const struct vspa_recovery* recovery, uint32_t offset)
{
    return recovery->ccsr[offset / 4];
}

/// Checks that SCRATCHRW32 points at the firmware's proxy struct. Nothing is written to the M4 before that.
static int find_proxy(struct vspa_recovery* recovery)
{
    const uint32_t address = ccsr_read(recovery, SCRATCHRW32_OFFSET);
    const uint32_t offset = address - TCML_BASE;
    if ((address & 3) || address < TCML_BASE || offset > recovery->tcml_size - P_DATA * 4)
    {
        log_message(recovery, VSPA_LOADER_LOG_ERROR, "M4 firmware has no VSPA debug proxy (SCRATCHRW32=0x%08X)", address);
        return VSPA_LOADER_BUSY;
    }
    const uint32_t index = offset / 4;
    volatile const uint32_t* proxy = recovery->tcml + index;
    const uint32_t max_words = proxy[P_MAX_WORDS];
    if (proxy[P_MAGIC] != PROXY_MAGIC || proxy[P_SELF] != address || max_words == 0 ||
        max_words > recovery->tcml_size / 4 - index - P_DATA)
    {
        log_message(recovery, VSPA_LOADER_LOG_ERROR, "M4 firmware has no VSPA debug proxy at 0x%08X", address);
        return VSPA_LOADER_BUSY;
    }
    if (proxy[P_VERSION] != PROXY_VERSION)
    {
        log_message(recovery,
            VSPA_LOADER_LOG_ERROR,
            "M4 firmware VSPA debug proxy version %u, expected %u",
            proxy[P_VERSION],
            PROXY_VERSION);
        return VSPA_LOADER_BUSY;
    }
    recovery->proxy_index = index;
    recovery->max_words = max_words;
    return VSPA_LOADER_OK;
}

/// One proxy request of up to max_words words. For reads, values receives the data.
static int proxy_request(
    const struct vspa_recovery* recovery, uint32_t op, uint32_t offset, uint32_t* values, uint32_t count, int fixed)
{
    volatile uint32_t* proxy = recovery->tcml + recovery->proxy_index;
    const uint32_t tag = ((proxy[P_STATUS] >> 24) + 1) & 0xFF;
    if (op == PROXY_OP_WRITE)
        for (uint32_t i = 0; i < count; ++i)
            proxy[P_DATA + i] = values[i];
    proxy[P_OFFSET] = offset;
    proxy[P_COUNT] = count;
    proxy[P_CMD] = (tag << 24) | (fixed ? PROXY_FLAG_FIXED : 0) | op;
    (void)proxy[P_CMD]; // read back: the request is in TCML before the doorbell
    recovery->ccsr[MSIIR3_OFFSET / 4] = 0;

    const int64_t deadline = now_ns() + PROXY_TIMEOUT_NS;
    uint32_t status;
    while (((status = proxy[P_STATUS]) >> 24) != tag)
    {
        if (now_ns() >= deadline)
        {
            // Release the M4 if the interrupt is stuck, a MSG3 handler that never reads MSIR3 re-enters forever
            (void)ccsr_read(recovery, MSIR3_OFFSET);
            log_message(recovery, VSPA_LOADER_LOG_ERROR, "M4 firmware VSPA debug proxy doesn't respond");
            return VSPA_LOADER_TIMEOUT;
        }
    }
    if (status & 0xFF)
    {
        log_message(recovery,
            VSPA_LOADER_LOG_ERROR,
            "M4 firmware VSPA debug proxy error %u (op %u, offset 0x%X, count %u)",
            status & 0xFF,
            op,
            offset,
            count);
        return VSPA_LOADER_BUSY;
    }
    if (op == PROXY_OP_READ)
        for (uint32_t i = 0; i < count; ++i)
            values[i] = proxy[P_DATA + i];
    return VSPA_LOADER_OK;
}

static int dbg_access(
    const struct vspa_recovery* recovery, uint32_t op, uint32_t offset, uint32_t* values, uint32_t count, int fixed)
{
    while (count)
    {
        const uint32_t n = count < recovery->max_words ? count : recovery->max_words;
        const int status = proxy_request(recovery, op, offset, values, n, fixed);
        if (status != VSPA_LOADER_OK)
            return status;
        if (!fixed)
            offset += 4 * n;
        values += n;
        count -= n;
    }
    return VSPA_LOADER_OK;
}

static int dbg_read(const struct vspa_recovery* recovery, uint32_t offset, uint32_t* value)
{
    return dbg_access(recovery, PROXY_OP_READ, offset, value, 1, 0);
}

static int dbg_write(const struct vspa_recovery* recovery, uint32_t offset, uint32_t value)
{
    return dbg_access(recovery, PROXY_OP_WRITE, offset, &value, 1, 0);
}

static int portal_seek(const struct vspa_recovery* recovery, uint32_t a_mode, uint32_t address)
{
    return dbg_write(recovery, RAVAP, (a_mode << 24) | (address & 0x0007FFFCu));
}

/// After PMEM was written while halted, the PRAM read data pins have to be restored before resuming, by reading the
/// word that the core fetched last.
static int fpc_restore(const struct vspa_recovery* recovery)
{
    uint32_t fpc;
    int status = portal_seek(recovery, AMODE_ARCH, ARCH_FPC << 2);
    if (status == VSPA_LOADER_OK)
        status = dbg_read(recovery, RAVFD, &fpc);
    if (status == VSPA_LOADER_OK)
        status = portal_seek(recovery, AMODE_PRAM, fpc);
    if (status == VSPA_LOADER_OK)
        status = dbg_read(recovery, RAVFD, &fpc);
    return status;
}

static int halt_core(const struct vspa_recovery* recovery)
{
    uint32_t rcstatus;
    int status = dbg_read(recovery, RCSTATUS, &rcstatus);
    if (status != VSPA_LOADER_OK || rcstatus != 0)
        return status;

    status = dbg_write(recovery, RCR, RCR_HALT_CYC_COUNTER | RCR_FORCE_HALT);
    const int64_t deadline = now_ns() + HALT_TIMEOUT_NS;
    while (status == VSPA_LOADER_OK && (status = dbg_read(recovery, RCSTATUS, &rcstatus)) == VSPA_LOADER_OK && rcstatus == 0)
    {
        if (now_ns() >= deadline)
        {
            log_message(recovery, VSPA_LOADER_LOG_ERROR, "VSPA core doesn't halt");
            return VSPA_LOADER_BUSY;
        }
    }
    return status;
}

/// Halts the core, places the idle stub at its prefetch PC and resumes it
static int run_idle_stub(const struct vspa_recovery* recovery)
{
    int status = halt_core(recovery);
    if (status != VSPA_LOADER_OK)
        return status;

    uint32_t fpc;
    status = portal_seek(recovery, AMODE_ARCH, ARCH_FPC << 2);
    if (status == VSPA_LOADER_OK)
        status = dbg_read(recovery, RAVFD, &fpc);
    if (status != VSPA_LOADER_OK)
        return status;

    // After an illegal instruction the execution PC can be in the middle of an instruction, the prefetch PC is aligned.
    // The padding is what fits below the prefetch PC.
    const uint32_t fetch = (fpc * PC_BYTES) & ~7u;
    const int restart = fetch > PRAM_SIZE || IDLE_STUB_WORDS * 4 - IDLE_STUB_PADDING > PRAM_SIZE - fetch;
    const uint32_t padding = restart ? 0 : (fetch < IDLE_STUB_PADDING ? fetch : IDLE_STUB_PADDING);
    const uint32_t address = restart ? 0 : fetch - padding;
    const uint32_t first = (IDLE_STUB_PADDING - padding) / 4;
    const uint32_t count = (uint32_t)IDLE_STUB_WORDS - first;
    log_message(recovery,
        VSPA_LOADER_LOG_DEBUG,
        "VSPA prefetch PC 0x%X, idle stub at PRAM 0x%04X%s",
        fpc,
        address,
        restart ? ", restarting the core there" : "");

    uint32_t words[IDLE_STUB_WORDS];
    for (uint32_t i = 0; i < count; ++i)
        words[i] = idle_stub[(first + i) / 2][(first + i) % 2];
    status = portal_seek(recovery, AMODE_PRAM, address);
    if (status == VSPA_LOADER_OK)
        status = dbg_access(recovery, PROXY_OP_WRITE, RAVID, words, count, 1);
    if (status == VSPA_LOADER_OK)
        status = portal_seek(recovery, AMODE_PRAM, address);
    if (status == VSPA_LOADER_OK)
        status = dbg_access(recovery, PROXY_OP_READ, RAVID, words, count, 1);
    if (status != VSPA_LOADER_OK)
        return status;
    for (uint32_t i = 0; i < count; ++i)
    {
        if (words[i] != idle_stub[(first + i) / 2][(first + i) % 2])
        {
            log_message(recovery, VSPA_LOADER_LOG_ERROR, "VSPA idle stub verification failed at PRAM 0x%04X", address + i * 4);
            return VSPA_LOADER_BUSY;
        }
    }

    if (restart)
    {
        // No room for the stub before the end of PRAM: reset the core and start it at PRAM 0. HOST_GO starts at
        // VCPU_GO_ADDR, which the reset doesn't clear. The stub clears HOST_GO.
        uint32_t control;
        status = dbg_write(recovery, IP_VCPU_GO_ADDR, 0);
        if (status == VSPA_LOADER_OK)
            status = dbg_read(recovery, IP_CONTROL, &control);
        if (status == VSPA_LOADER_OK)
            status = dbg_write(recovery, IP_CONTROL, (control & CONTROL_REG_MASK) | CONTROL_VCPU_RESET);
        if (status == VSPA_LOADER_OK)
            status = dbg_write(recovery, IP_CONTROL, (control & CONTROL_REG_MASK) | CONTROL_HOST_GO);
        if (status != VSPA_LOADER_OK)
            return status;
    }

    status = fpc_restore(recovery);
    if (status != VSPA_LOADER_OK)
        return status;

    // A halt on an illegal instruction stays latched: with halting on it enabled, every resume halts again after one
    // instruction. Stale comparators would halt the core as well.
    uint32_t hacr;
    status = dbg_read(recovery, HACR, &hacr);
    if (status == VSPA_LOADER_OK && (hacr & HACR_EVENTS))
        status = dbg_write(recovery, HACR, hacr & ~HACR_EVENTS);
    if (status == VSPA_LOADER_OK)
        status = dbg_write(recovery, RCR, RCR_HALT_CYC_COUNTER | RCR_RESUME);
    return status;
}

static int is_idle(const struct vspa_recovery* recovery, int* idle)
{
    uint32_t rcstatus, control, vspa_status, go_addr;
    int status = dbg_read(recovery, RCSTATUS, &rcstatus);
    if (status == VSPA_LOADER_OK)
        status = dbg_read(recovery, IP_CONTROL, &control);
    if (status == VSPA_LOADER_OK)
        status = dbg_read(recovery, IP_STATUS, &vspa_status);
    if (status == VSPA_LOADER_OK)
        status = dbg_read(recovery, IP_VCPU_GO_ADDR, &go_addr);
    if (status != VSPA_LOADER_OK)
        return status;
    *idle = rcstatus == 0 && !(control & CONTROL_HOST_GO) && !(vspa_status & STATUS_BUSY) && go_addr == 0;
    return VSPA_LOADER_OK;
}

int vspa_recovery_force_idle(struct vspa_recovery* recovery)
{
    if (!recovery->ccsr || !recovery->tcml || recovery->ccsr_size <= MSIR3_OFFSET || recovery->tcml_size < P_DATA * 4)
        return VSPA_LOADER_BUSY;

    int status = find_proxy(recovery);
    if (status != VSPA_LOADER_OK)
        return status;

    uint32_t dvr, gdben;
    status = dbg_read(recovery, DVR, &dvr);
    if (status != VSPA_LOADER_OK)
        return status;
    if (!(dvr & DVR_DBG_DBGEN))
    {
        log_message(recovery, VSPA_LOADER_LOG_ERROR, "VSPA invasive debug is disabled (DVR=0x%08X)", dvr);
        return VSPA_LOADER_BUSY;
    }
    status = dbg_read(recovery, GDBEN, &gdben);
    if (status == VSPA_LOADER_OK && !(gdben & GDBEN_IDBG_EN))
        status = dbg_write(recovery, GDBEN, gdben | GDBEN_IDBG_EN);
    if (status != VSPA_LOADER_OK)
        return status;

    // A branch that was already fetched can take the core past the stub, then it's placed again at the new PC
    for (int attempt = 0; attempt < ATTEMPTS; ++attempt)
    {
        status = run_idle_stub(recovery);
        if (status != VSPA_LOADER_OK)
            return status;

        const int64_t deadline = now_ns() + IDLE_TIMEOUT_NS;
        int idle = 0;
        while ((status = is_idle(recovery, &idle)) == VSPA_LOADER_OK && !idle && now_ns() < deadline)
            pause_us(100);
        if (status != VSPA_LOADER_OK)
            return status;
        if (idle)
        {
            log_message(recovery, VSPA_LOADER_LOG_DEBUG, "VSPA core forced into idle");
            return VSPA_LOADER_OK;
        }
    }
    log_message(recovery, VSPA_LOADER_LOG_ERROR, "VSPA core can't be forced into idle");
    return VSPA_LOADER_BUSY;
}
