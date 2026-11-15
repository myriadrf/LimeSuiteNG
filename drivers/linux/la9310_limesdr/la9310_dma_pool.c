#include "la9310_dma_pool.h"

#include <linux/dma-mapping.h>
#include <linux/genalloc.h>
#include <linux/pci.h>
#include <linux/slab.h>

#include "la9310_ioctl.h"

/*
 * A buffer's memory goes back to its pool when the buffer's last reference is dropped. The owning file holds one
 * reference until the buffer is freed, and each VMA holds one, so a buffer can't be handed out again while it's still
 * mapped.
 */
struct la9310_dma_buf {
    struct list_head node; /* on la9310_file.dma_bufs */
    struct kref ref;
    struct la9310_dma_pool* pool;
    unsigned long pool_addr;
    size_t offset; /* from the start of the pool's region */
    size_t size;
};

static const char* const la9310_dma_pool_names[LA9310_DMA_POOL_COUNT] = {
    [LA9310_DMA_POOL_CACHED] = "Cached DMA pool",
    [LA9310_DMA_POOL_UNCACHED] = "Uncached DMA pool",
};

int la9310_dma_mem_alloc(struct device* dev, const char* name, struct la9310_mem_region_info* region, size_t size, bool cacheable)
{
    region->page = NULL;
    if (cacheable)
    {
        /* Cacheable buffer: user space maps it cacheable with dma_mmap_pages(), and keeps it coherent with the
         * LA9310_IOCTL_CACHE_SYNC_FOR_CPU/DEVICE calls. Uncached (coherent) memory makes the sample memcpy in user space
         * several times slower on Cortex-A55 class cores. */
        region->page = dma_alloc_pages(dev, size, &region->phys_addr, DMA_BIDIRECTIONAL, GFP_KERNEL);
        region->vaddr = region->page ? page_address(region->page) : NULL;
    }
    else
    {
        /* Coherent allocation, it's also mapped to user space with dma_mmap_coherent(). Don't mix in a cacheable alias
         * (kzalloc() + dma_map_single()) on non-coherent hosts (arm64 i.MX95/LX2160 PCIe), it can corrupt the data. */
        region->vaddr = dma_alloc_coherent(dev, size, &region->phys_addr, GFP_KERNEL);
    }
    if (!region->vaddr)
    {
        dev_err(dev, "Failed to allocate %s (%zu bytes %s DMA memory)\n", name, size, cacheable ? "cacheable" : "coherent");
        return -ENOMEM;
    }
    memset(region->vaddr, 0, size);
    if (region->page)
        dma_sync_single_for_device(dev, region->phys_addr, size, DMA_BIDIRECTIONAL);
    region->size = size;

    dev_info(dev,
        "%s size: %zu, va:0x%px bus:0x%llx %s\n",
        name,
        size,
        region->vaddr,
        region->phys_addr,
        cacheable ? "cached" : "uncached");
    return 0;
}

void la9310_dma_mem_free(struct device* dev, const char* name, struct la9310_mem_region_info* region)
{
    if (!region->vaddr)
        return;

    dev_info(dev, "Free %s size: %zu, va:0x%px bus:0x%llx\n", name, region->size, region->vaddr, region->phys_addr);
    if (region->page)
        dma_free_pages(dev, region->size, region->page, region->phys_addr, DMA_BIDIRECTIONAL);
    else
        dma_free_coherent(dev, region->size, region->vaddr, region->phys_addr);
    region->vaddr = NULL;
    region->page = NULL;
    region->phys_addr = 0;
    region->size = 0;
}

static void la9310_dma_pool_release(struct kref* ref)
{
    struct la9310_dma_pool* pool = container_of(ref, struct la9310_dma_pool, ref);

    gen_pool_destroy(pool->gen);
    la9310_dma_mem_free(pool->dev, pool->name, &pool->region);
    put_device(pool->dev);
    kfree(pool);
}

int la9310_dma_pool_create(struct la9310_dev* la9310_dev, enum la9310_dma_pool_id id, size_t size, u32 ep_addr, bool cacheable)
{
    struct la9310_dma_pool* pool;
    int rc;

    size = PAGE_ALIGN(size);
    if (size == 0 || size > LA9310_EP_DMA_POOL_MAX_SIZE || (size >> PAGE_SHIFT) > LA9310_DMA_POOL_PGOFF_SPAN)
    {
        dev_err(la9310_dev->dev,
            "%s: invalid size %zu, the maximum is %u\n",
            la9310_dma_pool_names[id],
            size,
            LA9310_EP_DMA_POOL_MAX_SIZE);
        return -EINVAL;
    }

    pool = kzalloc(sizeof(*pool), GFP_KERNEL);
    if (!pool)
        return -ENOMEM;

    kref_init(&pool->ref);
    pool->dev = get_device(la9310_dev->dev);
    pool->name = la9310_dma_pool_names[id];
    pool->ep_addr = ep_addr;
    pool->pgoff_base = LA9310_DMA_BUF_PGOFF_BASE + id * LA9310_DMA_POOL_PGOFF_SPAN;

    rc = la9310_dma_mem_alloc(pool->dev, pool->name, &pool->region, size, cacheable);
    if (rc)
        goto put_dev;
    pool->coherent = !pool->region.page;

    pool->gen = gen_pool_create(PAGE_SHIFT, NUMA_NO_NODE);
    if (!pool->gen)
    {
        rc = -ENOMEM;
        goto free_mem;
    }
    rc = gen_pool_add(pool->gen, (unsigned long)pool->region.vaddr, size, NUMA_NO_NODE);
    if (rc)
        goto destroy_gen;

    la9310_dev->dma_pools[id] = pool;
    return 0;

destroy_gen:
    gen_pool_destroy(pool->gen);
free_mem:
    la9310_dma_mem_free(pool->dev, pool->name, &pool->region);
put_dev:
    put_device(pool->dev);
    kfree(pool);
    return rc;
}

void la9310_dma_pools_destroy(struct la9310_dev* la9310_dev)
{
    for (int i = 0; i < LA9310_DMA_POOL_COUNT; ++i)
    {
        struct la9310_dma_pool* pool = la9310_dev->dma_pools[i];

        if (!pool)
            continue;
        la9310_dev->dma_pools[i] = NULL;
        if (gen_pool_avail(pool->gen) != gen_pool_size(pool->gen))
            dev_warn(la9310_dev->dev, "%s still has buffers in use, it's freed when they are gone\n", pool->name);
        kref_put(&pool->ref, la9310_dma_pool_release);
    }
}

static void la9310_dma_buf_release(struct kref* ref)
{
    struct la9310_dma_buf* buf = container_of(ref, struct la9310_dma_buf, ref);
    struct la9310_dma_pool* pool = buf->pool;

    gen_pool_free(pool->gen, buf->pool_addr, buf->size);
    kfree(buf);
    kref_put(&pool->ref, la9310_dma_pool_release);
}

static inline dma_addr_t la9310_dma_buf_bus(const struct la9310_dma_buf* buf)
{
    return buf->pool->region.phys_addr + buf->offset;
}

static inline unsigned long la9310_dma_buf_pgoff(const struct la9310_dma_buf* buf)
{
    return buf->pool->pgoff_base + (buf->offset >> PAGE_SHIFT);
}

static void la9310_dma_buf_vm_open(struct vm_area_struct* vma)
{
    struct la9310_dma_buf* buf = vma->vm_private_data;

    kref_get(&buf->ref);
}

/* Runs under mmap_lock, so it must not take la9310_file.dma_lock */
static void la9310_dma_buf_vm_close(struct vm_area_struct* vma)
{
    struct la9310_dma_buf* buf = vma->vm_private_data;

    kref_put(&buf->ref, la9310_dma_buf_release);
}

static const struct vm_operations_struct la9310_dma_buf_vm_ops = {
    .open = la9310_dma_buf_vm_open,
    .close = la9310_dma_buf_vm_close,
};

void la9310_dma_file_init(struct la9310_file* file_ctx, struct la9310_dev* la9310_dev)
{
    file_ctx->dev = la9310_dev;
    mutex_init(&file_ctx->dma_lock);
    INIT_LIST_HEAD(&file_ctx->dma_bufs);
}

/* Each VMA keeps its file referenced, so this can't run until all of the file's mappings are gone */
void la9310_dma_file_release(struct la9310_file* file_ctx)
{
    struct la9310_dma_buf* buf;
    struct la9310_dma_buf* tmp;
    unsigned int count = 0;

    list_for_each_entry_safe(buf, tmp, &file_ctx->dma_bufs, node)
    {
        list_del(&buf->node);
        kref_put(&buf->ref, la9310_dma_buf_release);
        ++count;
    }
    if (count)
        pr_info("la9310: released %u DMA buffers left by a closed file\n", count);
    mutex_destroy(&file_ctx->dma_lock);
}

int la9310_dma_buf_alloc(struct la9310_file* file_ctx, struct la9310_atu* atu)
{
    struct la9310_dev* la9310_dev = file_ctx->dev;
    const enum la9310_dma_pool_id id = (atu->flags & LA9310_DMA_FLAG_UNCACHED) ? LA9310_DMA_POOL_UNCACHED : LA9310_DMA_POOL_CACHED;
    struct la9310_dma_pool* pool = la9310_dev->dma_pools[id];
    struct la9310_dma_buf* buf;

    if (atu->flags & ~(size_t)(LA9310_DMA_FLAG_UNCACHED | LA9310_DMA_FLAG_COHERENT))
        return -EINVAL;
    if (!pool)
        return -ENODEV;
    if (atu->size == 0 || atu->size > pool->region.size)
        return -EINVAL;

    buf = kzalloc(sizeof(*buf), GFP_KERNEL);
    if (!buf)
        return -ENOMEM;

    buf->size = PAGE_ALIGN(atu->size);
    buf->pool_addr = gen_pool_alloc(pool->gen, buf->size);
    if (!buf->pool_addr)
    {
        kfree(buf);
        return -ENOMEM;
    }
    kref_get(&pool->ref);
    buf->pool = pool;
    buf->offset = buf->pool_addr - (unsigned long)pool->region.vaddr;
    kref_init(&buf->ref);

    /* The memory was possibly used by another process */
    memset((void*)buf->pool_addr, 0, buf->size);
    if (!pool->coherent)
        dma_sync_single_for_device(pool->dev, la9310_dma_buf_bus(buf), buf->size, DMA_BIDIRECTIONAL);

    mutex_lock(&file_ctx->dma_lock);
    list_add_tail(&buf->node, &file_ctx->dma_bufs);
    mutex_unlock(&file_ctx->dma_lock);

    atu->host_bus = la9310_dma_buf_bus(buf);
    atu->ep_pa = pool->ep_addr + buf->offset;
    atu->size = buf->size;
    atu->flags = (id == LA9310_DMA_POOL_UNCACHED ? LA9310_DMA_FLAG_UNCACHED : 0) | (pool->coherent ? LA9310_DMA_FLAG_COHERENT : 0);
    atu->mmap_offset = la9310_dma_buf_pgoff(buf) << PAGE_SHIFT;

    dev_dbg(la9310_dev->dev, "%s alloc: offset 0x%zx size 0x%zx\n", pool->name, buf->offset, buf->size);
    return 0;
}

/* If the buffer is still mapped its memory returns to the pool when the last mapping goes away */
int la9310_dma_buf_free(struct la9310_file* file_ctx, size_t host_bus)
{
    struct la9310_dma_buf* buf;
    int rc = -EINVAL;

    mutex_lock(&file_ctx->dma_lock);
    list_for_each_entry(buf, &file_ctx->dma_bufs, node)
    {
        if (la9310_dma_buf_bus(buf) == host_bus)
        {
            list_del(&buf->node);
            kref_put(&buf->ref, la9310_dma_buf_release);
            rc = 0;
            break;
        }
    }
    mutex_unlock(&file_ctx->dma_lock);
    return rc;
}

int la9310_dma_buf_sync(struct la9310_file* file_ctx, const struct la9310_atu* atu, bool for_cpu)
{
    const size_t end = atu->host_bus + atu->size;
    const struct la9310_dma_buf* buf;
    int rc = -EINVAL;

    if (end < atu->host_bus)
        return -EINVAL;

    /* Under the lock, so the buffer can't be freed while it's synced */
    mutex_lock(&file_ctx->dma_lock);
    list_for_each_entry(buf, &file_ctx->dma_bufs, node)
    {
        const size_t buf_bus = la9310_dma_buf_bus(buf);

        if (atu->host_bus < buf_bus || end > buf_bus + buf->size)
            continue;

        if (!buf->pool->coherent)
        {
            if (for_cpu)
                dma_sync_single_for_cpu(buf->pool->dev, atu->host_bus, atu->size, DMA_BIDIRECTIONAL);
            else
                dma_sync_single_for_device(buf->pool->dev, atu->host_bus, atu->size, DMA_BIDIRECTIONAL);
        }
        rc = 0;
        break;
    }
    mutex_unlock(&file_ctx->dma_lock);
    return rc;
}

int la9310_dma_buf_mmap(struct la9310_file* file_ctx, struct vm_area_struct* vma)
{
    const size_t map_size = vma->vm_end - vma->vm_start;
    struct la9310_dma_buf* buf;
    struct la9310_dma_buf* found = NULL;
    struct la9310_dma_pool* pool;
    int rc;

    /* A private mapping would turn into a copy on write, that the device doesn't see */
    if (!(vma->vm_flags & VM_MAYSHARE))
        return -EINVAL;

    mutex_lock(&file_ctx->dma_lock);
    list_for_each_entry(buf, &file_ctx->dma_bufs, node)
    {
        if (la9310_dma_buf_pgoff(buf) == vma->vm_pgoff && map_size <= buf->size)
        {
            kref_get(&buf->ref);
            found = buf;
            break;
        }
    }
    mutex_unlock(&file_ctx->dma_lock);

    if (!found)
    {
        dev_err(file_ctx->dev->dev, "mmap: no buffer at page offset 0x%lx of size %zu\n", vma->vm_pgoff, map_size);
        return -EINVAL;
    }

    /* dma_mmap_*() have to be given the whole allocation, a part of it is selected by vm_pgoff. A pointer into the middle
     * of it wouldn't work if the allocation is not physically contiguous (IOMMU). */
    pool = found->pool;
    vma->vm_pgoff = found->offset >> PAGE_SHIFT;
    if (pool->region.page)
        rc = dma_mmap_pages(pool->dev, vma, pool->region.size, pool->region.page);
    else
        rc = dma_mmap_coherent(pool->dev, vma, pool->region.vaddr, pool->region.phys_addr, pool->region.size);
    if (rc)
    {
        kref_put(&found->ref, la9310_dma_buf_release);
        return rc;
    }

    /* vm_ops are set only now, as close() is not called on a failed mmap */
    vma->vm_private_data = found;
    vma->vm_ops = &la9310_dma_buf_vm_ops;
    return 0;
}
