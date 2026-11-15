#ifndef LA9310_VSPA_RECOVERY_H
#define LA9310_VSPA_RECOVERY_H

#include <stddef.h>
#include <stdint.h>

#include "vspa_loader.h"

#ifdef __cplusplus
extern "C" {
#endif

/// @brief Forces a VSPA core that doesn't stop on its own (crashed, halted on an illegal instruction, or stuck in a
/// loop) into idle, so that its memory can be loaded again with the core's DMA engine.
///
/// The VSPA debug block (0xE0046000) is on the Arm M4's private peripheral bus and no PCIe BAR reaches it, so the
/// accesses go through the debug proxy of the M4 firmware: the request is written into struct vspa_dbg_proxy in the
/// M4 TCML (its address is published in DCFG SCRATCHRW32) and message unit 3 (MSIIR3) makes the M4 perform it.
/// See arm_m4 src/vspa_debug.h for the protocol.
///
/// The core is halted, an idle stub is written into PRAM at its prefetch PC, and the core is resumed with the debug
/// halt events (illegal instruction, software breakpoint, comparators) disabled; they stay disabled. When the stub
/// doesn't fit before the end of PRAM, it is written at PRAM 0 and the core is reset and started there instead.
/// Like iqplayer's terminate(), the stub sets VCPU_GO_ADDR and SW_VERSION to 0, disables and clears the go events
/// (host mailbox GO included), empties the host mailboxes, clears HOST_GO and enters idle with `done`.
/// The PRAM contents are lost. ILLOP_STATUS keeps the address of the last illegal instruction.
struct vspa_recovery {
    volatile uint32_t* ccsr; ///< BAR0 (CCSR), mapped into the host
    size_t ccsr_size;
    volatile uint32_t* tcml; ///< BAR1 (M4 TCML from 0x1F800000), mapped into the host
    size_t tcml_size;
    vspa_loader_log_fn log; ///< optional
    void* user; ///< passed to the callback

    /// Filled in by vspa_recovery_force_idle()
    uint32_t proxy_index; ///< index of struct vspa_dbg_proxy in tcml
    uint32_t max_words;
};

/// @brief Forces the core into idle through the M4 debug proxy.
/// @return VSPA_LOADER_OK once the core is idle (not halted, not busy, HOST_GO clear), VSPA_LOADER_BUSY if the M4
/// firmware has no debug proxy, invasive debug is disabled or the core doesn't reach idle, VSPA_LOADER_TIMEOUT if the
/// M4 doesn't answer.
int vspa_recovery_force_idle(struct vspa_recovery* recovery);

#ifdef __cplusplus
}
#endif

#endif // LA9310_VSPA_RECOVERY_H
