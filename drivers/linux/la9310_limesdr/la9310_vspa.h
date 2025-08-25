/* SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0)
 * Copyright 2017, 2021 NXP
 */

#include <linux/cdev.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/wait.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/completion.h>
#include "la9310_pci.h"

#ifndef __VSPA_H_
#define __VSPA_H_

/* Maximum allowed size for any individual command */
#define CMD_MAX_SZ_BYTES	(128)
#define NLM_ENABLE_VSPA_LOAD

enum vspa_state {
	VSPA_STATE_UNKNOWN = 0,
	VSPA_STATE_POWER_DOWN,
	VSPA_STATE_STARTUP_ERR,
	VSPA_STATE_UNPROGRAMMED_IDLE,
	VSPA_STATE_UNPROGRAMMED_BUSY,
	VSPA_STATE_LOADING,
	VSPA_STATE_RUNNING_IDLE,
	VSPA_STATE_RUNNING_BUSY
};

enum vspa_power {
	VSPA_POWER_DOWN = 0,
	VSPA_POWER_UP,
	VSPA_POWER_CYCLE
};

enum vspa_event_type {
	VSPA_EVENT_NONE = 0,
	VSPA_EVENT_DMA,
	VSPA_EVENT_CMD,
	VSPA_EVENT_REPLY,
	VSPA_EVENT_SPM,
	VSPA_EVENT_MB64_IN,
	VSPA_EVENT_MB32_IN,
	VSPA_EVENT_MB64_OUT,
	VSPA_EVENT_MB32_OUT,
	VSPA_EVENT_ERROR
};

#define VSPA_FLAG_EXPECT_CMD_REPLY	(1<<0)
#define VSPA_FLAG_REPORT_CMD_REPLY	(1<<1)
#define VSPA_FLAG_REPORT_CMD_CONSUMED	(1<<2)
#define VSPA_FLAG_REPORT_DMA_COMPLETE	(1<<3)
#define VSPA_FLAG_REPORT_MB_COMPLETE	(1<<4)


#define VSPA_MSG(x)			(0x10 << x)

#define VSPA_MSG_PEEK			VSPA_MSG(VSPA_EVENT_NONE)
#define VSPA_MSG_DMA			VSPA_MSG(VSPA_EVENT_DMA)
#define VSPA_MSG_CMD			VSPA_MSG(VSPA_EVENT_CMD)
#define VSPA_MSG_REPLY			VSPA_MSG(VSPA_EVENT_REPLY)
#define VSPA_MSG_SPM			VSPA_MSG(VSPA_EVENT_SPM)
#define VSPA_MSG_MB64_IN		VSPA_MSG(VSPA_EVENT_MB64_IN)
#define VSPA_MSG_MB32_IN		VSPA_MSG(VSPA_EVENT_MB32_IN)
#define VSPA_MSG_MB64_OUT		VSPA_MSG(VSPA_EVENT_MB64_OUT)
#define VSPA_MSG_MB32_OUT		VSPA_MSG(VSPA_EVENT_MB32_OUT)
#define VSPA_MSG_ERROR			VSPA_MSG(VSPA_EVENT_ERROR)
#define VSPA_MSG_ALL			(0xFFF0)
#define VSPA_MSG_ALL_EVENTS		(0xFFE0)

#define VSPA_ERR_DMA_CFGERR		(0x10)
#define VSPA_ERR_DMA_XFRERR		(0x11)
#define VSPA_ERR_WATCHDOG		(0x12)

struct vspa_event {
	union {
	  uint32_t	control;
	  struct {
	    uint8_t	id;
	    uint8_t	err;
	    uint16_t	type;
	  };
	};
	uint16_t	pkt_size;
	uint16_t	buf_size;
	uint32_t	data[0];
};

struct vspa_dma_req {
	union {
	  uint32_t	control;
	  struct {
	    uint8_t	id;
	    uint8_t	flags;
	    uint8_t	rsvd;
	    uint8_t	type;
	  };
	};
	uint32_t	dmem_addr;
	dma_addr_t	axi_addr;
	uint32_t	byte_cnt;
	uint32_t	xfr_ctrl;
};

#define VSPA_MAX_ELD_FILENAME (256)
struct vspa_startup {
	uint32_t	cmd_buf_size;
	uint32_t	cmd_buf_addr;
	uint32_t	spm_buf_size;
	uint32_t	spm_buf_addr;
	uint8_t		cmd_dma_chan;
	char		filename[VSPA_MAX_ELD_FILENAME];
};

struct vspa_versions {
	uint32_t	vspa_hw_version;
	uint32_t	ippu_hw_version;
	uint32_t	vspa_sw_version;
	uint32_t	ippu_sw_version;
};

struct vspa_hardware {
	uint32_t	param0;
	uint32_t	param1;
	uint32_t	param2;
	uint32_t	axi_data_width;
	uint32_t	dma_channels;
	uint32_t	gp_out_regs;
	uint32_t	gp_in_regs;
	uint32_t	dmem_bytes;
	uint32_t	ippu_bytes;
	uint32_t	arithmetic_units;
};

struct vspa_reg {
	uint32_t	reg;
	uint32_t	val;
};

struct vspa_mb32 {
	union {
	  uint32_t      control;
	  struct {
	    uint8_t     id;
	    uint8_t     flags;
	    uint8_t     rsvd0;
	    uint8_t     rsvd1;
	  };
	};
	uint32_t        data;
};

struct vspa_mb64 {
	union {
	  uint32_t      control;
	  struct {
	    uint8_t     id;
	    uint8_t     flags;
	    uint8_t     rsvd0;
	    uint8_t     rsvd1;
	  };
	};
	uint32_t        data_msb;
	uint32_t        data_lsb;
};

struct vspa_event_read {
	uint32_t	event_mask; /* bit field of events to match*/
	int		timeout; /* delay in mSecs (0 = noblock, -1 = forever)*/
	size_t		buf_len; /* max reply length in bytes*/
	struct vspa_event *buf_ptr;
};

/* Set Watchdog interval */
#define VSPA_IOC_WATCHDOG_INT	_IO(VSPA_MAGIC_NUM, 10)
#define VSPA_WATCHDOG_INTERVAL_DEFAULT	 (1000)
#define VSPA_WATCHDOG_INTERVAL_MIN	  (100)
#define VSPA_WATCHDOG_INTERVAL_MAX	(60000)

/*VSPA Firmware Max size for buffer*/
#define LA9310_VSPA_FW_SIZE	(2 * 1024 * 1024)
#define VSPA_FW_NAME		"apm.eld"

/*MAX number of Overlay Section supported (including default overlay)*/
#define DEBUG_VSPA
#define MAX_SECTION_NAME		20
#define MAX_OVERLAY_SECTIONS		3
#define DEFAULT_OVERLAY_SEC_NAME	".vpram_overlay"


/* ELF FILE configuration */
/* Arch code in the binary file fo VSPA */
/* Unique machine code to identify VSPA f/w compiler */
#define GCC_MACH_CODE           0xBE
#define ISCAPE_MACH_CODE        0x40c8

/* Number of section in the VSPA ELF image */
#define MAX_VSP_SECTION         6

#define EI_NIDENT               16

/*Set for booting support only */
#define VSPA_POLL_BASED_BOOT 1

/*VSPA offset in pci mmap and size*/
#define VSPA_CCSR_OFFSET 0x1000000
#define LA9310_VSPA_SIZE 0x10000

#define OCRAM_MAX_SIZE  (64 * 1024)

/* Flags for the libvspa_dma_send() */
#define BLOCK           0x80000000
#define NO_BLOCK        0x00000000


#define MAX_DMA_TRANSFER 65536
#define PRAM_ADDR_MASK  0x0003FFFF
#define DMA_BCNT_MASK   0x0000FFFF

#define PRAM_MEM_SIZE	0x8000
#define DMEM_ADDR	0x0

/* Macros to perform address alignment */
#define isaligned(ptr, align)  \
	(((ptr) & (align - 1)) == 0)

#define _align_mask(x, mask)    (((x) + (mask)) & ~(mask))

#define align(x, a)     _align_mask(x, (typeof(x))(a) - 1)

/* Debug and error reporting macros */
#define ERR(...)	{ if (vspadev->debug & DEBUG_MESSAGES) \
				pr_err(VSPA_DEVICE_NAME __VA_ARGS__); }
#define MAX_VSPA 1
#define VSPA_DEVICE_NAME "-vspa"

/* Debug bit assignments */
#define DEBUG_MESSAGES			(1<<0)
#define DEBUG_STARTUP			(1<<1)
#define DEBUG_CMD			(1<<2)
#define DEBUG_REPLY			(1<<3)
#define DEBUG_SPM			(1<<4)
#define DEBUG_DMA			(1<<5)
#define DEBUG_EVENT			(1<<6)
#define DEBUG_WATCHDOG			(1<<7)
#define DEBUG_MBOX64_OUT		(1<<8)
#define DEBUG_MBOX32_OUT		(1<<9)
#define DEBUG_MBOX64_IN			(1<<10)
#define DEBUG_MBOX32_IN			(1<<11)
#define DEBUG_DMA_IRQ			(1<<12)
#define DEBUG_FLAGS0_IRQ		(1<<13)
#define DEBUG_FLAGS1_IRQ		(1<<14)
#define DEBUG_IOCTL			(1<<15)
#define DEBUG_SEQID			(1<<16)
#define DEBUG_CMD_BD			(1<<17)
#define DEBUG_REPLY_BD			(1<<18)
#define DEBUG_TEST_SPM			(1<<24)

/* IP register offset for the registers used by the driver */
#define HWVERSION_REG_OFFSET		(0x0000>>2)
#define SWVERSION_REG_OFFSET		(0x0004>>2)
#define CONTROL_REG_OFFSET		(0x0008>>2)
#define IRQEN_REG_OFFSET		(0x000C>>2)
#define STATUS_REG_OFFSET		(0x0010>>2)
#define HOST_FLAGS0_REG_OFFSET		(0x0014>>2)
#define HOST_FLAGS1_REG_OFFSET		(0x0018>>2)
#define VCPU_FLAGS0_REG_OFFSET		(0x001C>>2)
#define VCPU_FLAGS1_REG_OFFSET		(0x0020>>2)
#define EXT_GO_ENABLE_REG_OFFSET	(0x0028>>2)
#define EXT_GO_STATUS_REG_OFFSET	(0x002C>>2)
#define PARAM0_REG_OFFSET		(0x0040>>2)
#define PARAM1_REG_OFFSET		(0x0044>>2)
#define PARAM2_REG_OFFSET		(0x0048>>2)
#define DMA_DMEM_ADDR_REG_OFFSET	(0x00B0>>2)
#define DMA_AXI_ADDR_REG_OFFSET		(0x00B4>>2)
#define DMA_BYTE_CNT_REG_OFFSET		(0x00B8>>2)
#define DMA_XFR_CTRL_REG_OFFSET		(0x00BC>>2)
#define DMA_STAT_ABORT_REG_OFFSET	(0x00C0>>2)
#define DMA_IRQ_STAT_REG_OFFSET		(0x00C4>>2)
#define DMA_COMP_STAT_REG_OFFSET	(0x00C8>>2)
#define DMA_XFRERR_STAT_REG_OFFSET	(0x00CC>>2)
#define DMA_CFGERR_STAT_REG_OFFSET	(0x00D0>>2)
#define DMA_XRUN_STAT_REG_OFFSET	(0x00D4>>2)
#define DMA_GO_STAT_REG_OFFSET		(0x00D8>>2)
#define DMA_FIFO_STAT_REG_OFFSET	(0x00DC>>2)

#define GP_OUT1_REG_OFFSET		(0x0580>>2)
#define GP_OUT2_REG_OFFSET		(0x0584>>2)
#define GP_OUT3_REG_OFFSET		(0x0588>>2)

#define HOST_OUT_64_MSB_REG_OFFSET	(0x0680>>2)
#define HOST_OUT_64_LSB_REG_OFFSET	(0x0684>>2)
#define HOST_OUT_1_64_MSB_REG_OFFSET	(0x0688>>2)
#define HOST_OUT_1_64_LSB_REG_OFFSET	(0x068C>>2)
#define HOST_IN_64_MSB_REG_OFFSET	(0x0690>>2)
#define HOST_IN_64_LSB_REG_OFFSET	(0x0694>>2)
#define HOST_IN_1_64_MSB_REG_OFFSET	(0x0698>>2)
#define HOST_IN_1_64_LSB_REG_OFFSET	(0x069C>>2)
#define HOST_MBOX_STATUS_REG_OFFSET     (0x06A0>>2)

#define IPPU_CONTROL_REG_OFFSET		(0x0700>>2)
#define IPPU_STATUS_REG_OFFSET		(0x0704>>2)
#define IPPU_ARG_BASEADDR_REG_OFFSET	(0x070C>>2)
#define IPPU_HWVERSION_REG_OFFSET	(0x0710>>2)
#define IPPU_SWVERSION_REG_OFFSET	(0x0714>>2)

#define DBG_GDBEN_REG_OFFSET		(0x800>>2)
#define DBG_RCR_REG_OFFSET		(0x804>>2)
#define DBG_RCSTATUS_REG_OFFSET		(0x808>>2)
#define DBG_RAVAP_REG_OFFSET		(0x870>>2)
#define DBG_DVR_REG_OFFSET		(0x87C>>2)


#define STATUS_REG_PDN_ACTIVE		(0x80000000)
#define STATUS_REG_PDN_DONE		(0x40000000)
#define STATUS_REG_IRQ_VCPU_READ_MSG	(0x0000C000)
#define STATUS_REG_IRQ_VCPU_SENT_MSG	(0x00003000)
#define STATUS_REG_IRQ_VCPU_MSG		(STATUS_REG_IRQ_VCPU_READ_MSG | \
					STATUS_REG_IRQ_VCPU_SENT_MSG)
#define STATUS_REG_BUSY			(0x00000100)
#define STATUS_REG_IRQ_DMA_ERR		(0x00000020)
#define STATUS_REG_IRQ_DMA_COMP		(0x00000010)
#define STATUS_REG_IRQ_FLAGS1		(0x00000008)
#define STATUS_REG_IRQ_FLAGS0		(0x00000004)
#define STATUS_REG_IRQ_IPPU_DONE	(0x00000002)
#define STATUS_REG_IRQ_DONE		(0x00000001)
#define STATUS_REG_IRQ_NON_GEN         (STATUS_REG_IRQ_FLAGS1 |		\
		STATUS_REG_IRQ_FLAGS0 | STATUS_REG_IRQ_DMA_COMP |	\
		STATUS_REG_IRQ_DMA_ERR | STATUS_REG_IRQ_VCPU_SENT_MSG)

#define MBOX_STATUS_IN_1_64_BIT		(0x00000008)
#define MBOX_STATUS_IN_64_BIT		(0x00000004)
#define MBOX_STATUS_OUT_1_64_BIT	(0x00000002)
#define MBOX_STATUS_OUT_64_BIT		(0x00000001)

#define VSPA_DMA_CHANNELS		(32)

/* mmap offset argument for vspa regsiters */
#define VSPA_REG_OFFSET			(0)

/* mmap offset argument for dbg regsiter */
#define VSPA_DBG_OFFSET			(4096)

#define DMA_FLAG_COMPLETE		(1<<0)
#define DMA_FLAG_XFRERR			(1<<1)
#define DMA_FLAG_CFGERR			(1<<2)

#define VSPA_ATU_OWBAR			(0x110U >> 2)
#define VSPA_ATU_OWAR			(0x114U >> 2)
#define VSPA_ATU_OTEAR			(0x118U >> 2)
#define VSPA_ATU_OTAR			(0x11CU >> 2)

#define MBOX_QUEUE_ENTRIES		(16)
struct mbox_queue {
	volatile int	idx_enqueue;
	volatile int	idx_dequeue;
	volatile int	idx_complete;
};

#define EVENT_LIST_ENTRIES		(256)
struct event_list {
	struct list_head list;
	union {
	  uint32_t	control;
	  struct {
	    uint8_t	id;
	    uint8_t	err;
	    uint16_t	type;
	  };
	};
	uint32_t	data0;
	uint32_t	data1;
};

struct event_entry {
	union {
	  uint32_t      control;
	  struct {
	    uint8_t     id;
	    uint8_t     err;
	    uint8_t     rsvd;
	    uint8_t     type;
	  };
	};
	uint32_t	data0;
	uint32_t	data1;
	uint32_t	lost;
};

#define EVENT_QUEUE_ENTRIES		(256)
struct event_queue {
	struct event_entry entry[EVENT_QUEUE_ENTRIES];
	volatile int    idx_enqueue;	/* Index for next item to enqueue */
	volatile int    idx_queued;	/* Index for last item enqueued */
	volatile int    idx_dequeue;	/* Index for next item to dequeue */
};

#define DMA_QUEUE_ENTRIES		(16)
struct dma_queue {
	struct vspa_dma_req entry[DMA_QUEUE_ENTRIES];
	int		chan;
	int		idx_queue;	/* Index for next item to enqueue */
	int		idx_dma;	/* Index for next item to DMA */
	int		idx_chk;	/* Index for next item to check */
};

struct memory_pool_bd {
	uint32_t	start;
	uint32_t	size;
	uint32_t	free;
	uint32_t	wstart;
	int8_t		next;
};

struct dma_param {
	uint32_t xfr_ctrl;
	uint32_t offset;
	dma_addr_t phys;
	uint32_t size;
};

/*TODO - support 32 reply bds */
#define BD_ENTRIES		(16)
struct memory_pool {
	uint32_t	free_bds;
	struct memory_pool_bd bd[BD_ENTRIES];
	uint32_t	tail;
	uint32_t	size;
	uint32_t	*paddr;
	uint32_t	*vaddr;
};

struct circular_buffer {
	uint32_t	write_idx;
	uint32_t	read_idx;
	uint32_t	size;
	uint32_t	used;
	uint32_t	*paddr;
	uint32_t	*vaddr;
};

struct file_header {
	uint8_t  fh_ident[EI_NIDENT];
	uint16_t fh_type;
	uint16_t fh_machine;
	uint32_t fh_version;
	uint32_t fh_entry;
	uint32_t fh_phoff;
	uint32_t fh_shoff;
	uint32_t fh_flags;
	uint16_t fh_ehsize;
	uint16_t fh_phentsize;
	uint16_t fh_phnum;
	uint16_t fh_shentsize;
	uint16_t fh_shnum;
	uint16_t fh_shstrndx;
};
struct section_header {
	uint32_t sec_name;      /* Section name (string tbl index) */
	uint32_t sec_type;      /* Section type */
	uint32_t sec_flags;     /* Section flags */
	uint32_t sec_addr;      /* Section virtual addr at execution */
	uint32_t sec_offset;    /* Section file offset */
	uint32_t sec_size;      /* Section size in bytes */
	uint32_t sec_link;      /* Link to another section */
	uint32_t sec_info;      /* Additional section information */
	uint32_t sec_addralign; /* Section alignment */
	uint32_t sec_entsize;   /* Entry size if section holds table */
};



#define VPRAM           (0x00)
#define VDRAM           (VPRAM + 0x01)
#define CMDBUFF         (VPRAM + 0x02)
#define IPRAM           (VPRAM + 0x03)
#define IDRAM           (VPRAM + 0x04)
#define OCRAMVSPA       (VPRAM + 0x05)
#define DEFAULT_OVERLAY	VPRAM
#define OVERLAY_1	VPRAM
#define OVERLAY_2	VPRAM


#define DMEM_NC         (0 << 8)
#define DMEM_WC         (1 << 8)
#define PRAM            (2 << 8)
#define IPPUPRAM        (3 << 8)
#define AXI_NC          (6 << 8)
#define AXI_WC          (7 << 8)
#define NO_LOAD     0xFFFFFFFF

enum libvspa_error {
	LIBVSPA_ERR_OK = 0,     /*No error occurred*/
	LIBVSPA_ERR_EPERM,      /*Operation not permitted in current state*/
	LIBVSPA_ERR_ENOENT,     /* No such file exists*/
	LIBVSPA_ERR_NOCMDBUF,   /*No CMDBUF section in ELF file*/
	LIBVSPA_ERR_EINTR,      /*Interrupted system call*/
	LIBVSPA_ERR_EIO,        /*I/O error*/
	LIBVSPA_ERR_START_FAIL, /*Bad VSPA startup sequence msg*/
	LIBVSPA_ERR_OPEN_FAIL,  /*Cannot open log, ELF image or /dev/mem*/
	LIBVSPA_ERR_CLOSE_FAIL, /*Error closing log file or device*/
	LIBVSPA_ERR_EBADF,      /*Bad VSPA handle*/
	LIBVSPA_ERR_ETIMEOUT,   /*Shutdown/wakeup hardware time out*/
	LIBVSPA_ERR_EAGAIN,     /*No matching message found before timeout*/
	LIBVSPA_ERR_ENOMEM,     /*No device buffer / queue space left*/
	LIBVSPA_ERR_EACCESS,    /*File permission denied*/
	LIBVSPA_ERR_EFAULT,     /*Illegal memory address*/
	LIBVSPA_ERR_MISALIGNED, /*Command/SPM buffer is not AXI aligned*/
	LIBVSPA_ERR_EBUSY,      /*Device is already open*/
	LIBVSPA_ERR_EINVOFFSET, /*ELF buffer is not AXI aligned*/
	LIBVSPA_ERR_INVALID_FILE,/*ELF file structure/content errors*/
	LIBVSPA_ERR_ENODEV,     /*No such device*/
	LIBVSPA_ERR_EVENT,      /*Event message has internal error*/
	LIBVSPA_ERR_OTHER,      /*Unknown error*/
	LIBVSPA_ERR_EINVAL,     /*Invalid or NULL argument*/
	LIBVSPA_MAX_ERR_CNT     /*Dummy for range check*/
};

enum vspa_memory_type {
	AXI_TO_DMEM = 0,
	AXI_TO_DMEM_16,
	AXI_TO_PRAM,
	AXI_TO_IPPU_PRAM,
	RESERVED_1,
	RESERVED_2,
	DMEM_TO_AXI,
	DMEM_TO_AXI_16,
};

struct program_header {
	uint32_t prg_type;            /* Segment type */
	uint32_t prg_offset;          /* Segment file offset */
	uint32_t prg_vaddr;           /* Segment virtual address */
	uint32_t prg_paddr;           /* Segment physical address */
	uint32_t prg_filesz;          /* Segment size in file */
	uint32_t prg_memsz;           /* Segment size in memory */
	uint32_t prg_flags;           /* Segment flags */
	uint32_t prg_align;           /* Segment alignment */
};


#define MAX_SEQIDS			(16)
struct seqid {
	uint32_t	flags;
	int		cmd_id;
	int		cmd_buffer_idx;
	int		cmd_buffer_size;
	int		cmd_bd_index;
	int		reply_bd_index;
	int		reply_size;
	uint32_t	payload1;
};

struct vspa_stats_info {
	uint32_t vspa_loading_count;
};

struct overlay_section {
	char *name;
	struct dma_param dpram;
};

/* The below structure contains all the information for the
* vspa device required by the kernel driver
*/
struct vspa_device {

	/* VSPA instance */
	int		id;

	struct device	*dev;

	/* Char device structure */
	struct cdev	cdev;

	/* Major minor information */
	dev_t		dev_t;

	/* Current state of the device */
	enum vspa_state	state;
	uint32_t	debug;

	/*Dma region for vspa from la9310_dev*/
	struct la9310_mem_region_info *vspa_dma_region;

	/*PCI device address*/
	struct pci_dev *pdev;

	/* IRQ numbers */
	uint32_t	vspa_irq_no;
	/* IP registers */
	resource_size_t	mem_size; /* size */
	u32 __iomem	*mem_addr;    /* physical address */
	u32 __iomem	*regs;	/* virtual address */

	u32 __iomem	*vspa_atu_regs; /* Virtual address */
	u32 __iomem	*vspa_atu_addr; /* Physical address */
	uint32_t	vspa_atu_size;
	u32 __iomem	*vspa_atu_bar;
	dma_addr_t	vspa_atu_tar;

	/* Debug registers */
	resource_size_t	dbg_size; /* size */
	u32 __iomem	*dbg_addr;    /* physical address */
	u32 __iomem	*dbg_regs;    /* virtual address */

	/* Buffer sizes */
	uint32_t	spm_buffer_bytes;
	uint32_t	cmd_buffer_bytes;
	uint32_t	reply_buffer_bytes;
	uint32_t	*spm_buffer_paddr;
	uint32_t	*spm_buffer_vaddr;

	/* Working set for SPM buffer */
	int		spm_user_buf;
	uint32_t	spm_buf_bytes;
	uint32_t	*spm_buf_paddr;
	uint32_t	*spm_buf_vaddr;

	/* DMA queue */
	struct dma_queue dma_queue;
	spinlock_t	dma_enqueue_lock;
	spinlock_t	dma_tx_queue_lock; /* called from irq handler */
	unsigned long	dma_tx_queue_lock_flags;

	/* Event queue */
	struct event_queue event_queue;
	struct event_list events[EVENT_LIST_ENTRIES];
	struct list_head events_free_list;
	struct list_head events_queued_list;
	spinlock_t	event_list_lock;
	spinlock_t	event_queue_lock; /* called from irq handler */

	/* Memory pools */
	struct circular_buffer cmd_buffer;
	struct memory_pool cmd_pool;
	struct memory_pool reply_pool;

	uint32_t	cmdbuf_addr;
	uint32_t	spm_addr;

	/* Sequence IDs */
	uint32_t	active_seqids;
	uint32_t	last_seqid;
	struct seqid	seqid[MAX_SEQIDS];

	uint32_t	first_cmd;
	uint32_t	last_wstart;

	struct mbox_queue mb32_queue;
	struct mbox_queue mb64_queue;
	struct vspa_mb32 mb32[MBOX_QUEUE_ENTRIES];
	struct vspa_mb64 mb64[MBOX_QUEUE_ENTRIES];
	spinlock_t	mb32_lock; /* called from irq handler */
	spinlock_t	mb64_lock; /* called from irq handler */
	spinlock_t	mbchk_lock; /* called from irq handler */
	struct timer_list mbchk_timer;
	struct completion mbchk_complete;

	/* DMA channel usage */
	uint8_t		spm_dma_chan;
	uint8_t		bulk_dma_chan;
	uint8_t		reply_dma_chan;
	uint8_t		cmd_dma_chan;

	uint8_t		legacy_cmd_dma_chan;
	char		eld_filename[VSPA_MAX_ELD_FILENAME];
	struct vspa_versions versions;
	struct vspa_hardware hardware;

	/* Watchdog */
	struct timer_list watchdog_timer;
	uint32_t	watchdog_interval_msecs;
	uint32_t	watchdog_value;
	struct completion watchdog_complete;

	/* IRQ handling */
/* TODO	spinlock_t irq_lock; */
	uint32_t	irq_bits; /* TODO - test code */

	spinlock_t	control_lock;

	/* Wait queue for event notifications*/
	wait_queue_head_t event_wait_q;
	uint32_t	event_list_mask;
	uint32_t	event_queue_mask;
	uint32_t	poll_mask;
	struct vspa_stats_info vspa_stats;

	/*Overlay structure*/
	struct overlay_section overlay_sec[MAX_OVERLAY_SECTIONS];
	char *overlay_sec_loaded;
	uint32_t max_sec_size;
};

extern const struct attribute_group attr_group;

/*
 * Accessor Functions
 */

static inline void vspa_reg_write(void __iomem *addr, u32 val)
{
	return iowrite32(val, addr);
}

static inline unsigned int vspa_reg_read(void __iomem *addr)
{
	return ioread32(addr);
}

int full_state(struct vspa_device *vspadev);
int overlay_initiate(struct device *dev, struct overlay_section
		     overlay_sec);
#endif /* _VSPA_H */
