#ifndef __LA9310_DMA_POOL_H__
#define __LA9310_DMA_POOL_H__

#include <linux/fs.h>
#include <linux/kref.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/mutex.h>

#include "la9310_base.h"

struct la9310_atu;
struct gen_pool;

/*
 * Host DMA memory that user space allocates buffers from (enum la9310_dma_pool_id). Each pool is one contiguous
 * allocation, mapped to the LA9310 through its own outbound window. A buffer belongs to the file that allocated it, and
 * is freed when the file is closed.
 */

/* mmap offsets at or above this page offset select a pool buffer, below it they are a la9310_window_t */
#define LA9310_DMA_BUF_PGOFF_BASE 0x10000UL
/* page offset range of each pool, it limits the pool size */
#define LA9310_DMA_POOL_PGOFF_SPAN 0x10000UL

struct la9310_dma_pool {
    /* The device holds one reference and each buffer holds one, the memory is freed when the last one is dropped.
     * Buffers can outlive the device, if it's unbound while files are open or buffers are mapped. */
    struct kref ref;
    struct device* dev;
    const char* name;
    struct la9310_mem_region_info region;
    u32 ep_addr; /* address of the region on the LA9310 */
    unsigned long pgoff_base;
    bool coherent; /* uncached memory, needs no cache maintenance */
    bool leak; /* don't free the memory on release, see la9310_dev.buggy_ats */
    struct gen_pool* gen;
};

/* Per open file state, stored in file->private_data */
struct la9310_file {
    struct la9310_dev* dev;
    struct mutex dma_lock; /* protects dma_bufs */
    struct list_head dma_bufs; /* buffers this file has allocated and not freed */
};

/* Allocates host DMA memory: cacheable (synced by the users), or coherent */
int la9310_dma_mem_alloc(struct device* dev, const char* name, struct la9310_mem_region_info* region, size_t size, bool cacheable);
void la9310_dma_mem_free(struct device* dev, const char* name, struct la9310_mem_region_info* region);

int la9310_dma_pool_create(struct la9310_dev* la9310_dev, enum la9310_dma_pool_id id, size_t size, u32 ep_addr, bool cacheable);
/* Drops the device's references to its pools */
void la9310_dma_pools_destroy(struct la9310_dev* la9310_dev);

void la9310_dma_file_init(struct la9310_file* file_ctx, struct la9310_dev* la9310_dev);
/* Called when the last reference to the file is gone, frees everything the file still owns */
void la9310_dma_file_release(struct la9310_file* file_ctx);

/* On input atu->size and atu->flags select the buffer, on success all of the fields describe the new buffer */
int la9310_dma_buf_alloc(struct la9310_file* file_ctx, struct la9310_atu* atu);
int la9310_dma_buf_free(struct la9310_file* file_ctx, size_t host_bus);
/* Checks that the range is within a buffer owned by this file, and does the cache maintenance if it's needed */
int la9310_dma_buf_sync(struct la9310_file* file_ctx, const struct la9310_atu* atu, bool for_cpu);
int la9310_dma_buf_mmap(struct la9310_file* file_ctx, struct vm_area_struct* vma);

#endif
