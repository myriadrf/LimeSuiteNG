#include "vspa_loader.h"

#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "vspa_recovery.h"
#include "vspa_regs.h"

#define CONTROL_REG_MASK (~0x000100FFu)
#define CONTROL_PDN_EN (1u << 31)
#define CONTROL_HOST_GO (1u << 0)
#define CONTROL_WAKEUP_GO_EN ((1u << 25) | (1u << 24) | (1u << 11))
#define STATUS_BUSY (1u << 8)
#define STATUS_MAILBOX_IRQS 0xF000u
#define MBOX_STATUS_IN_0 (1u << 2)

#define DMA_DMEM_ADDR_MASK 0x0003FFFFu
#define DMA_BYTE_COUNT_MAX 0xFFFFu
#define DMA_CHAN_MASK (1u << VSPA_LOADER_DMA_CHAN)
#define VSPA_LOADER_DMA_CHAN 0

#define XFR_CTRL_DMEM (0u << 8)
#define XFR_CTRL_PRAM (2u << 8)
#define XFR_CTRL_IPPU_PRAM (3u << 8)

// The staging buffers start on cache line boundaries, so syncing one buffer doesn't touch its neighbours
#define STAGING_SLOT_ALIGN 64u

// Older firmware waits for this message during its boot
#define LEGACY_BOOT_CONTINUE_MSG 0x70000000u

#define MACHINE_GCC 0xBE
#define MACHINE_ISCAPE 0x40C8
#define SECTION_TYPE_NOBITS 8
// The firmware linker sets the upper address bits by the memory type, the core doesn't use them
#define LOAD_ADDRESS_MASK 0xF0FFFFFFu

#define ELF_HEADER_SIZE 52
#define ELF_SECTION_HEADER_SIZE 40
#define ELF_PROGRAM_HEADER_SIZE 32

#define IDLE_TIMEOUT_NS 100000000ll // the core stops shortly after acknowledging the reset message
#define DMA_TIMEOUT_NS 1000000000ll
#define BOOT_TIMEOUT_NS 1000000000ll

struct elf_section_header {
    uint32_t name;
    uint32_t type;
    uint32_t flags;
    uint32_t addr;
    uint32_t offset;
    uint32_t size;
    uint32_t link;
    uint32_t info;
    uint32_t addralign;
    uint32_t entsize;
};

struct elf_program_header {
    uint32_t type;
    uint32_t offset;
    uint32_t vaddr;
    uint32_t paddr;
    uint32_t filesz;
    uint32_t memsz;
    uint32_t flags;
    uint32_t align;
};

struct loadable_section {
    const char* prefix;
    uint32_t xfr_ctrl;
};

// Matched as a prefix, this would include .vpram_overlay. Everything else in the image doesn't get loaded.
static const struct loadable_section loadable_sections[] = {
    { ".vpram", XFR_CTRL_PRAM },
    { ".vdram", XFR_CTRL_DMEM },
    { ".ipram", XFR_CTRL_IPPU_PRAM },
    { ".idram", XFR_CTRL_DMEM },
};

// ---------------------------------------------------------------------------------------------------------------------
// ELD parsing
// ---------------------------------------------------------------------------------------------------------------------

static int in_bounds(size_t size, uint64_t offset, uint64_t length)
{
    return offset <= size && length <= size - offset;
}

static uint16_t get16(const uint8_t* p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}

static uint32_t get32(const uint8_t* p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void read_section_header(const struct vspa_eld_parser* parser, uint32_t index, struct elf_section_header* sh)
{
    const uint8_t* p = parser->image + parser->shoff + (uint64_t)index * parser->shentsize;
    sh->name = get32(p + 0);
    sh->type = get32(p + 4);
    sh->flags = get32(p + 8);
    sh->addr = get32(p + 12);
    sh->offset = get32(p + 16);
    sh->size = get32(p + 20);
    sh->link = get32(p + 24);
    sh->info = get32(p + 28);
    sh->addralign = get32(p + 32);
    sh->entsize = get32(p + 36);
}

static void read_program_header(const struct vspa_eld_parser* parser, uint32_t index, struct elf_program_header* ph)
{
    const uint8_t* p = parser->image + parser->phoff + (uint64_t)index * parser->phentsize;
    ph->type = get32(p + 0);
    ph->offset = get32(p + 4);
    ph->vaddr = get32(p + 8);
    ph->paddr = get32(p + 12);
    ph->filesz = get32(p + 16);
    ph->memsz = get32(p + 20);
    ph->flags = get32(p + 24);
    ph->align = get32(p + 28);
}

static int invalid(struct vspa_eld_parser* parser, const char* reason)
{
    parser->error = reason;
    return VSPA_LOADER_INVALID_IMAGE;
}

int vspa_eld_open(struct vspa_eld_parser* parser, const void* image, size_t size)
{
    memset(parser, 0, sizeof(*parser));
    parser->image = (const uint8_t*)image;
    parser->size = size;

    const uint8_t* p = parser->image;
    if (!p || size < ELF_HEADER_SIZE)
        return invalid(parser, "too short");
    if (memcmp(p,
            "\x7F"
            "ELF",
            4) != 0)
        return invalid(parser, "not an ELF file");
    if (p[4] != 1 || p[5] != 1)
        return invalid(parser, "not a 32 bit little endian ELF file");
    const uint16_t machine = get16(p + 18);
    if (machine != MACHINE_GCC && machine != MACHINE_ISCAPE)
        return invalid(parser, "not built for VSPA");

    parser->phoff = get32(p + 28);
    parser->shoff = get32(p + 32);
    parser->phentsize = get16(p + 42);
    parser->phnum = get16(p + 44);
    parser->shentsize = get16(p + 46);
    parser->shnum = get16(p + 48);
    const uint16_t shstrndx = get16(p + 50);

    if (parser->shentsize < ELF_SECTION_HEADER_SIZE || parser->phentsize < ELF_PROGRAM_HEADER_SIZE)
        return invalid(parser, "unexpected table entry size");
    // The last entries only need their known fields to be present
    if (parser->phnum &&
        !in_bounds(size, parser->phoff, (uint64_t)(parser->phnum - 1) * parser->phentsize + ELF_PROGRAM_HEADER_SIZE))
        return invalid(parser, "program header table is out of bounds");
    if (parser->shnum &&
        !in_bounds(size, parser->shoff, (uint64_t)(parser->shnum - 1) * parser->shentsize + ELF_SECTION_HEADER_SIZE))
        return invalid(parser, "section header table is out of bounds");
    if (shstrndx >= parser->shnum)
        return invalid(parser, "section name table is missing");

    struct elf_section_header names;
    read_section_header(parser, shstrndx, &names);
    if (!in_bounds(size, names.offset, names.size))
        return invalid(parser, "section name table is out of bounds");
    parser->names_offset = names.offset;
    parser->names_size = names.size;
    return VSPA_LOADER_OK;
}

static const struct loadable_section* find_loadable(const char* name, size_t length)
{
    for (size_t i = 0; i < sizeof(loadable_sections) / sizeof(loadable_sections[0]); ++i)
    {
        const size_t prefix = strlen(loadable_sections[i].prefix);
        if (length >= prefix && memcmp(name, loadable_sections[i].prefix, prefix) == 0)
            return &loadable_sections[i];
    }
    return NULL;
}

int vspa_eld_next(struct vspa_eld_parser* parser, struct vspa_eld_section* section)
{
    while (parser->next < parser->shnum)
    {
        struct elf_section_header sh;
        read_section_header(parser, parser->next++, &sh);

        if (sh.name >= parser->names_size)
            return invalid(parser, "section name is out of bounds");
        const char* name = (const char*)parser->image + parser->names_offset + sh.name;
        const size_t max_length = parser->names_size - sh.name;
        const char* end = memchr(name, '\0', max_length);
        const size_t length = end ? (size_t)(end - name) : max_length;

        const struct loadable_section* loadable = find_loadable(name, length);
        if (!loadable)
            continue;
        // Section without contents has nothing to add
        if (sh.size == 0 || sh.type == SECTION_TYPE_NOBITS)
            continue;
        if (!in_bounds(parser->size, sh.offset, sh.size))
            return invalid(parser, "section contents are out of bounds");

        uint32_t i = 0;
        struct elf_program_header ph;
        for (; i < parser->phnum; ++i)
        {
            read_program_header(parser, i, &ph);
            if (ph.vaddr == sh.addr)
                break;
        }
        if (i == parser->phnum)
            return invalid(parser, "section has no load address");

        section->data = parser->image + sh.offset;
        section->size = sh.size;
        section->address = ph.paddr & LOAD_ADDRESS_MASK;
        section->xfr_ctrl = loadable->xfr_ctrl;
        return 1;
    }
    return 0;
}

// ---------------------------------------------------------------------------------------------------------------------
// Loading
// ---------------------------------------------------------------------------------------------------------------------

static void log_message(const struct vspa_loader* loader, enum vspa_loader_log_level level, const char* format, ...)
{
    if (!loader->log)
        return;
    char message[192];
    va_list args;
    va_start(args, format);
    vsnprintf(message, sizeof(message), format, args);
    va_end(args);
    loader->log(loader->user, level, message);
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

static uint32_t align_up(uint32_t value, uint32_t align)
{
    return (value + align - 1) & ~(align - 1);
}

/// Granularity of a transfer's destination address and byte count. Every transfer has to start on an AXI bus word,
/// PRAM words are 8 bytes, and IPPU PRAM transfers have to start 64-bit aligned. Byte counts follow the same rule, so
/// the parts of a section stay aligned too.
static uint32_t transfer_align(const struct vspa_loader* loader, uint32_t xfr_ctrl)
{
    if (xfr_ctrl == XFR_CTRL_DMEM || loader->axi_align >= 8)
        return loader->axi_align;
    return 8;
}

int vspa_loader_prepare(struct vspa_loader* loader, const void* image, size_t size)
{
    volatile struct vspa_regs* regs = loader->regs;

    loader->axi_align = 4u << ((regs->param1 >> 28) & 7);
    const uint32_t align = loader->axi_align;

    // Check the whole image now, so a malformed one doesn't get the running firmware stopped
    struct vspa_eld_parser parser;
    struct vspa_eld_section section;
    int status = vspa_eld_open(&parser, image, size);
    unsigned count = 0;
    while (status == VSPA_LOADER_OK && (status = vspa_eld_next(&parser, &section)) == 1)
    {
        status = VSPA_LOADER_OK;
        ++count;
        if (section.address & (transfer_align(loader, section.xfr_ctrl) - 1))
            status = invalid(&parser, "section's load address is not aligned for DMA");
    }
    if (status == VSPA_LOADER_OK && count == 0)
        status = invalid(&parser, "there is nothing to load");
    if (status != VSPA_LOADER_OK)
    {
        log_message(loader, VSPA_LOADER_LOG_ERROR, "VSPA firmware image is invalid: %s", parser.error);
        return status;
    }

    // The staging memory is split into equal buffers, used in rotation
    const uint32_t buffer_align = align > STAGING_SLOT_ALIGN ? align : STAGING_SLOT_ALIGN;
    const size_t buffer_size = (loader->staging_size / VSPA_LOADER_STAGING_BUFFERS) & ~(size_t)(buffer_align - 1);
    const uint32_t byte_count_max = DMA_BYTE_COUNT_MAX & ~(buffer_align - 1);
    loader->max_transfer = buffer_size < byte_count_max ? (uint32_t)buffer_size : byte_count_max;
    if (!loader->staging || (loader->staging_pa & (buffer_align - 1)) || loader->max_transfer < align)
    {
        log_message(loader,
            VSPA_LOADER_LOG_ERROR,
            "VSPA firmware loading DMA memory is unusable, %zu bytes at 0x%08X",
            loader->staging_size,
            loader->staging_pa);
        return VSPA_LOADER_NO_MEMORY;
    }

    loader->next_buffer = 0;
    loader->queued = 0;
    return VSPA_LOADER_OK;
}

static int wait_idle(const struct vspa_loader* loader)
{
    const int64_t deadline = now_ns() + IDLE_TIMEOUT_NS;
    while (loader->regs->vspa_status & STATUS_BUSY)
    {
        if (now_ns() >= deadline)
            return VSPA_LOADER_BUSY;
        pause_us(100);
    }
    return VSPA_LOADER_OK;
}

/// Stops whatever the previous firmware left running on the loader's channel, and clears its status
static int reset_channel(const struct vspa_loader* loader)
{
    volatile struct vspa_regs* regs = loader->regs;
    const uint32_t mask = DMA_CHAN_MASK;

    if ((regs->dma_stat_abort | regs->dma_xrun_stat) & mask)
    {
        // The abort clears the enabled bit and the FIFO right away, the channel has stopped when it isn't running
        regs->dma_stat_abort = mask;
        const int64_t deadline = now_ns() + DMA_TIMEOUT_NS;
        while (regs->dma_xrun_stat & mask)
        {
            if (now_ns() >= deadline)
            {
                log_message(loader, VSPA_LOADER_LOG_ERROR, "VSPA DMA channel %u can't be aborted", VSPA_LOADER_DMA_CHAN);
                return VSPA_LOADER_BUSY;
            }
            pause_us(10);
        }
    }

    regs->dma_comp_stat = mask;
    regs->dma_xfrerr_stat = mask;
    regs->dma_cfgerr_stat = mask;
    return VSPA_LOADER_OK;
}

/// Checks the channel for errors, a configuration error might not get the transfer completed at all
static int check_dma_error(const struct vspa_loader* loader)
{
    volatile struct vspa_regs* regs = loader->regs;
    const uint32_t xfrerr = regs->dma_xfrerr_stat & DMA_CHAN_MASK;
    const uint32_t cfgerr = regs->dma_cfgerr_stat & DMA_CHAN_MASK;
    if (!(xfrerr | cfgerr))
        return VSPA_LOADER_OK;

    log_message(loader,
        VSPA_LOADER_LOG_ERROR,
        "VSPA firmware DMA %s error after %u queued transfers",
        cfgerr ? "configuration" : "transfer",
        loader->queued);
    regs->dma_xfrerr_stat = DMA_CHAN_MASK;
    regs->dma_cfgerr_stat = DMA_CHAN_MASK;
    return VSPA_LOADER_DMA_ERROR;
}

/// Waits until the channel's command FIFO can take another transfer
static int wait_fifo_slot(const struct vspa_loader* loader)
{
    volatile struct vspa_regs* regs = loader->regs;
    const int64_t deadline = now_ns() + DMA_TIMEOUT_NS;
    while (1)
    {
        const int status = check_dma_error(loader);
        if (status != VSPA_LOADER_OK)
            return status;
        // Set while the channel's FIFO has an empty entry
        if (regs->dma_fifo_stat & DMA_CHAN_MASK)
            return VSPA_LOADER_OK;
        if (now_ns() >= deadline)
        {
            log_message(loader,
                VSPA_LOADER_LOG_ERROR,
                "VSPA firmware DMA FIFO stays full, DMA_FIFO_STAT 0x%08X, after %u queued transfers",
                regs->dma_fifo_stat,
                loader->queued);
            return VSPA_LOADER_TIMEOUT;
        }
        pause_us(10);
    }
}

/// Waits until the channel has finished all the queued transfers
static int wait_all(const struct vspa_loader* loader)
{
    volatile struct vspa_regs* regs = loader->regs;
    const int64_t deadline = now_ns() + DMA_TIMEOUT_NS;
    while (1)
    {
        const int status = check_dma_error(loader);
        if (status != VSPA_LOADER_OK)
            return status;
        // The channel stays enabled while it has a transfer running or queued
        if (!((regs->dma_stat_abort | regs->dma_xrun_stat) & DMA_CHAN_MASK))
            break;
        if (now_ns() >= deadline)
        {
            log_message(loader,
                VSPA_LOADER_LOG_ERROR,
                "VSPA firmware DMA timeout, DMA_STAT_ABORT 0x%08X, DMA_XRUN_STAT 0x%08X, DMA_FIFO_STAT 0x%08X, after %u "
                "queued transfers",
                regs->dma_stat_abort,
                regs->dma_xrun_stat,
                regs->dma_fifo_stat,
                loader->queued);
            return VSPA_LOADER_TIMEOUT;
        }
        pause_us(10);
    }
    if (loader->queued && !(regs->dma_comp_stat & DMA_CHAN_MASK))
    {
        log_message(loader, VSPA_LOADER_LOG_ERROR, "VSPA firmware DMA stopped without completing");
        return VSPA_LOADER_DMA_ERROR;
    }
    regs->dma_comp_stat = DMA_CHAN_MASK;
    return VSPA_LOADER_OK;
}

/// Copies one part of a section into the next staging buffer, and queues its transfer.
/// The DMA keeps running the transfers queued before, while the copy is made.
static int submit(
    struct vspa_loader* loader, uint32_t address, uint32_t xfr_ctrl, const uint8_t* data, uint32_t copied, uint32_t transfer)
{
    volatile struct vspa_regs* regs = loader->regs;

    // When the previous transfer got its FIFO slot, at most the FIFO depth of transfers plus the running one were
    // left before it. The transfer that last used this buffer is older than those, so the DMA is done reading it.
    const uint32_t offset = loader->next_buffer * loader->max_transfer;
    uint8_t* staging = loader->staging + offset;
    if (copied)
        memcpy(staging, data, copied);
    memset(staging + copied, 0, transfer - copied);
    if (loader->sync_for_device && loader->sync_for_device(loader->user, offset, transfer) != 0)
    {
        log_message(loader, VSPA_LOADER_LOG_ERROR, "VSPA firmware staging memory sync failed");
        return VSPA_LOADER_SYNC_ERROR;
    }

    const int status = wait_fifo_slot(loader);
    if (status != VSPA_LOADER_OK)
        return status;

    __sync_synchronize(); // the staged data has to be in memory before the DMA is started
    regs->dma_dmem_pram_addr = address & DMA_DMEM_ADDR_MASK;
    regs->dma_axi_address = loader->staging_pa + offset;
    regs->dma_axi_byte_cnt = transfer;
    regs->dma_xfr_ctrl = xfr_ctrl | VSPA_LOADER_DMA_CHAN;

    loader->next_buffer = (loader->next_buffer + 1) % VSPA_LOADER_STAGING_BUFFERS;
    ++loader->queued;
    return VSPA_LOADER_OK;
}

/// Queues the section's transfers, it returns as soon as the last one is queued
static int write_section(struct vspa_loader* loader, const struct vspa_eld_section* section)
{
    // The transfers have to be whole AXI bus words, the part that's not covered by data is written as zeroes
    const uint32_t size = align_up(section->size, transfer_align(loader, section->xfr_ctrl));
    uint32_t done = 0;
    while (done < size)
    {
        const uint32_t remaining = size - done;
        const uint32_t transfer = remaining < loader->max_transfer ? remaining : loader->max_transfer;
        const uint32_t available = done < section->size ? section->size - done : 0;
        const uint32_t copied = available < transfer ? available : transfer;

        const int status = submit(loader, section->address + done, section->xfr_ctrl, section->data + done, copied, transfer);
        if (status != VSPA_LOADER_OK)
            return status;
        done += transfer;
    }
    return VSPA_LOADER_OK;
}

static void drain_mailbox(const struct vspa_loader* loader)
{
    volatile struct vspa_regs* regs = loader->regs;
    if (regs->host_mbox_status & MBOX_STATUS_IN_0)
    {
        const uint32_t msb = regs->host_in_0_msb;
        const uint32_t lsb = regs->host_in_0_lsb;
        log_message(loader, VSPA_LOADER_LOG_DEBUG, "VSPA firmware message 0x%08X%08X", msb, lsb);
    }
}

static int boot(const struct vspa_loader* loader)
{
    volatile struct vspa_regs* regs = loader->regs;

    // HOST_GO starts execution at VCPU_GO_ADDR, not at PMEM 0, and neither loading nor a VCPU reset changes it. The
    // previous firmware usually points it at its own main(), where the new image would start without initialization.
    // The reference manual says only the VCPU can write it, but on LA9310 silicon host writes take effect (bit 0 is
    // ignored, the address is 64-bit aligned).
    regs->vcpu_go_addr = 0;

    // Older firmware waits for this message before it clears HOST_GO, so it has to be sent first.
    // Current firmware drops it when it initializes, before clearing HOST_GO.
    // Mailbox GO is disabled since the load, so the message doesn't start the image before HOST_GO.
    regs->host_out_0_msb = LEGACY_BOOT_CONTINUE_MSG;
    regs->host_out_0_lsb = 0;

    regs->vcpu_ctrl = (regs->vcpu_ctrl & CONTROL_REG_MASK) | CONTROL_HOST_GO;

    // The firmware clears HOST_GO once its initialization is done and it accepts commands
    const int64_t deadline = now_ns() + BOOT_TIMEOUT_NS;
    while (1)
    {
        // Read out the boot messages of older firmware, so they aren't taken as replies to the first commands
        drain_mailbox(loader);

        if (!(regs->vcpu_ctrl & CONTROL_HOST_GO))
            break;

        if (now_ns() >= deadline)
        {
            log_message(loader, VSPA_LOADER_LOG_ERROR, "VSPA firmware boot timeout, HOST_GO wasn't cleared");
            return VSPA_LOADER_TIMEOUT;
        }
        pause_us(100);
    }

    log_message(loader, VSPA_LOADER_LOG_DEBUG, "VSPA firmware ready, version 0x%08X", regs->sw_version);
    return VSPA_LOADER_OK;
}

int vspa_loader_load(struct vspa_loader* loader, const void* image, size_t size)
{
    volatile struct vspa_regs* regs = loader->regs;

    int status = wait_idle(loader);
    if (status == VSPA_LOADER_BUSY && loader->recovery)
    {
        // The firmware didn't stop, e.g. it crashed or ignored the reset message: force the core into idle
        log_message(loader, VSPA_LOADER_LOG_ERROR, "VSPA core is busy, forcing it into idle");
        status = vspa_recovery_force_idle(loader->recovery);
        if (status == VSPA_LOADER_OK)
            status = wait_idle(loader);
    }
    if (status != VSPA_LOADER_OK)
    {
        log_message(loader, VSPA_LOADER_LOG_ERROR, "VSPA core is busy, can't load firmware");
        return status;
    }

    // Enable core power gating. Interrupt enables are not touched, they are owned by the Arm M4 firmware.
    // Nothing may wake the idle core while its memory is replaced, or before HOST_GO boots the new image: iqplayer
    // disables these GO events when it stops, but a firmware that crashed or was stopped otherwise may not have.
    // The new firmware enables the ones it uses.
    regs->vcpu_ctrl = (regs->vcpu_ctrl & CONTROL_REG_MASK & ~CONTROL_WAKEUP_GO_EN) | CONTROL_PDN_EN;
    regs->ext_go_ena = 0;

    // Drop whatever the previous firmware has left for the host
    regs->vcpu_host_flags0 = 0xFFFFFFFF;
    regs->vcpu_host_flags1 = 0xFFFFFFFF;
    regs->vspa_status = STATUS_MAILBOX_IRQS;
    drain_mailbox(loader);

    status = reset_channel(loader);
    if (status != VSPA_LOADER_OK)
        return status;
    loader->next_buffer = 0;
    loader->queued = 0;

    // Parsing the next section and staging it overlaps with the DMA running the queued transfers
    struct vspa_eld_parser parser;
    struct vspa_eld_section section;
    status = vspa_eld_open(&parser, image, size);
    while (status == VSPA_LOADER_OK && (status = vspa_eld_next(&parser, &section)) == 1)
        status = write_section(loader, &section);
    if (status == VSPA_LOADER_INVALID_IMAGE)
        log_message(loader, VSPA_LOADER_LOG_ERROR, "VSPA firmware image is invalid: %s", parser.error);

    // Even on failure, nothing may be left in flight from the staging memory
    const int drained = wait_all(loader);
    if (status != VSPA_LOADER_OK)
        return status;
    if (drained != VSPA_LOADER_OK)
        return drained;

    return boot(loader);
}

const char* vspa_loader_strerror(int status)
{
    switch (status)
    {
    case VSPA_LOADER_OK:
        return "success";
    case VSPA_LOADER_INVALID_IMAGE:
        return "invalid firmware image";
    case VSPA_LOADER_NO_MEMORY:
        return "no usable staging memory";
    case VSPA_LOADER_BUSY:
        return "VSPA busy";
    case VSPA_LOADER_TIMEOUT:
        return "timeout";
    case VSPA_LOADER_DMA_ERROR:
        return "DMA error";
    case VSPA_LOADER_SYNC_ERROR:
        return "staging memory sync failed";
    default:
        return "unknown error";
    }
}
