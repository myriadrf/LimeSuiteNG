#include "la9310_character_device.h"

// #define DEBUG

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/dma-mapping.h>
// #include <linux/dma-attrs.h>
#include <linux/sched.h>

#include "la9310_limesdr_device.h"
#include "la9310_base.h"
#include "la9310_pci.h"
#include "la9310_ioctl.h"

#include "la9310_base.h"

#define LA9310_LIMESDR_MINOR_COUNT 32

int la9310_modinfo_mmap(struct file* filp, struct vm_area_struct* vma);

static int la9310_limesdr_major;
static int la9310_limesdr_minor_idx;
static dev_t la9310_limesdr_dev_t;

struct char_dev_data {
    struct la9310_dev* myDevice;
    struct cdev cdev_node;
};

static struct char_dev_data la9310_cdevs[LA9310_LIMESDR_MINOR_COUNT];

static const struct vm_operations_struct mmap_mem_ops = {
#ifdef CONFIG_HAVE_IOREMAP_PROT
    .access = generic_access_phys
#endif
};

// VSPA memory on BAR2 must not be touched before RCW_COMPLETION_DONE is set, the clock is gated and core won't work
// attempts to access that memory region while core is disabled can cause reboot of the host. The host library maps
// every window when it opens the device, which can be before the M4 firmware is loaded. In that state only the VSPA
// DMEM part of a BAR2 mapping is left unpopulated, the rest of BAR2 (TCM) is mapped up front. VSPA pages are inserted
// on fault, once the RCW has completed; until then a fault raises SIGBUS instead of reaching the device. The flag is
// sticky, so mappings made after it is set are populated directly.
static vm_fault_t la9310_bar2_fault(struct vm_fault* vmf)
{
    struct vm_area_struct* vma = vmf->vma;
    struct la9310_dev* la9310_dev = vma->vm_private_data;

    const unsigned long offset = vmf->pgoff << PAGE_SHIFT;
    if (la9310_bar2_overlaps_vspa(offset, PAGE_SIZE) && !la9310_rcw_completed(la9310_dev))
    {
        dev_err_ratelimited(la9310_dev->dev,
            "%s (PID: %d): BAR2 VSPA memory access at offset 0x%lx before RCW completion, sending SIGBUS\n",
            current->comm,
            current->pid,
            offset);
        return VM_FAULT_SIGBUS;
    }

    const unsigned long pfn = (la9310_dev->mem_regions[LA9310_MEM_REGION_TCMU].phys_addr >> PAGE_SHIFT) + vmf->pgoff;
    return vmf_insert_pfn(vma, vmf->address, pfn);
}

static const struct vm_operations_struct bar2_lazy_mem_ops = {
    .fault = la9310_bar2_fault,
#ifdef CONFIG_HAVE_IOREMAP_PROT
    .access = generic_access_phys /* only reaches pages that have been inserted, i.e. after RCW completion */
#endif
};

static inline void la9310_vma_add_flags(struct vm_area_struct* vma, vm_flags_t flags)
{
// https://github.com/torvalds/linux/commit/bc292ab00f6c7a661a8a605c714e8a148f629ef6
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 3, 0)
    vma->vm_flags |= flags;
#else
    vm_flags_set(vma, flags);
#endif
}

static int la9310_mmap(struct file* file, struct vm_area_struct* vma)
{
    struct la9310_dev* la9310_dev = file->private_data;
    struct device* sysDev = la9310_dev->dev;

    la9310_window_t window_id = vma->vm_pgoff;
    bool isDMA = false;

    struct la9310_mem_region_info* region;
    switch (window_id)
    {
    case LA9310_WINDOW_BAR0:
        region = &la9310_dev->mem_regions[LA9310_MEM_REGION_CCSR];
        break;
    case LA9310_WINDOW_BAR1:
        region = &la9310_dev->mem_regions[LA9310_MEM_REGION_TCML];
        break;
    case LA9310_WINDOW_BAR2:
        region = &la9310_dev->mem_regions[LA9310_MEM_REGION_TCMU];
        break;
    case LA9310_WINDOW_SCRATCH:
        isDMA = true;
        region = &la9310_dev->dma_info.host_buf;
        break;
    case LA9310_WINDOW_MSI:
        return -EINVAL;
    // case LA9310_WINDOW_IPC:
    //     isDMA = true;
    //     region = &la9310_dev->dmem_proxy;
    //     break;
    case LA9310_WINDOW_IQFLOOD:
        isDMA = true;
        region = &la9310_dev->iqflood_region;
        break;
    default:
        dev_err(sysDev, "%s: invalid page offset\n", __func__);
        return -EINVAL;
    }

    const vm_flags_t add_flags = VM_DONTDUMP | VM_DONTEXPAND | VM_IO;
// https://github.com/torvalds/linux/commit/bc292ab00f6c7a661a8a605c714e8a148f629ef6
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 3, 0)
    vma->vm_flags |= add_flags;
#else
    vm_flags_set(vma, add_flags);
#endif
    vma->vm_pgoff = 0;
    const size_t mapSize = vma->vm_end - vma->vm_start;
    if (mapSize > region->size)
    {
        dev_err(sysDev, "mmap window %u: invalid size (%lu), expected (%lu)\n", window_id, mapSize, region->size);
        return -EINVAL;
    }

    int remapRet = 0;
    if (isDMA)
    {
        // dynamically allocated mappings
        dev_dbg(sysDev,
            "MMAP %i - vm_start:%lu cpu:%p bus:%llx size:%lu\n",
            window_id,
            vma->vm_start,
            region->vaddr,
            region->phys_addr,
            mapSize);
        if (region->page)
            remapRet = dma_mmap_pages(sysDev, vma, mapSize, region->page); /* cacheable; synced via ioctls */
        else
            remapRet = dma_mmap_coherent(sysDev, vma, region->vaddr, region->phys_addr, mapSize);
    }
    else
    {
        // BAR mappings
        size_t pfn = region->phys_addr >> PAGE_SHIFT;
        dev_dbg(sysDev, "MMAP %i - vm_start:%lx pfn:%lx size:%lu\n", window_id, vma->vm_start, pfn, mapSize);
        vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

        if (window_id == LA9310_WINDOW_BAR2 && la9310_bar2_overlaps_vspa(0, mapSize) && !la9310_rcw_completed(la9310_dev))
        {
            // vmf_insert_pfn() and partial remap_pfn_range() can't be used on copy-on-write mappings
            if (!(vma->vm_flags & VM_SHARED))
            {
                dev_err(sysDev, "mmap window %u: must be MAP_SHARED\n", window_id);
                return -EINVAL;
            }
            const unsigned long vspa_start = LA9310_BAR2_VSPA_OFFSET;
            const unsigned long vspa_end = min_t(unsigned long, mapSize, LA9310_BAR2_VSPA_OFFSET + LA9310_BAR2_VSPA_SIZE);
            dev_dbg(sysDev,
                "MMAP %i: RCW not completed, VSPA pages 0x%lx-0x%lx are inserted on fault\n",
                window_id,
                vspa_start,
                vspa_end);
            la9310_vma_add_flags(vma, VM_PFNMAP);
            vma->vm_private_data = la9310_dev;
            vma->vm_ops = &bar2_lazy_mem_ops;

            // TCM before the VSPA window
            remapRet = remap_pfn_range(vma, vma->vm_start, pfn, vspa_start, vma->vm_page_prot);
            // whatever follows the VSPA window
            if (!remapRet && mapSize > vspa_end)
                remapRet = remap_pfn_range(vma,
                    vma->vm_start + vspa_end,
                    pfn + (vspa_end >> PAGE_SHIFT),
                    mapSize - vspa_end,
                    vma->vm_page_prot);
        }
        else
        {
            vma->vm_ops = &mmap_mem_ops;
            remapRet = remap_pfn_range(vma, vma->vm_start, pfn, mapSize, vma->vm_page_prot);
        }
    }
    if (remapRet)
    {
        dev_err(sysDev, "mmap failed %i\n", remapRet);
        return -EAGAIN;
    }
    return 0;
}

static int la9310_limesdr_open(struct inode* inode, struct file* file)
{
    struct char_dev_data* node_data = container_of(inode->i_cdev, struct char_dev_data, cdev_node);
    file->private_data = node_data->myDevice;
    dev_info(
        &node_data->myDevice->pdev->dev, "Open %s by %s (PID: %d)\n", file->f_path.dentry->d_iname, current->comm, current->pid);
    return 0;
}

static int la9310_limesdr_release(struct inode* inode, struct file* file)
{
    struct la9310_dev* myDevice = file->private_data;
    if (myDevice == NULL)
    {
        return 0;
    }

    dev_info(&myDevice->pdev->dev, "Release %s\n", file->f_path.dentry->d_iname);
    return 0;
}

static const struct file_operations limepcie_fops_control = {
    .owner = THIS_MODULE,
    .unlocked_ioctl = la9310_ioctl,
    .open = la9310_limesdr_open,
    .release = la9310_limesdr_release,
    // .read = limepcie_ctrl_read,
    // .write = limepcie_ctrl_write,
    .mmap = la9310_mmap,
    // .mmap = la9310_modinfo_mmap,
};

dev_t la9310_cdev_create(struct la9310_dev* myDevice)
{
    int index = la9310_limesdr_minor_idx;

    struct char_dev_data* node_data = &la9310_cdevs[index];
    node_data->myDevice = myDevice;
    cdev_init(&node_data->cdev_node, &limepcie_fops_control);
    dev_t major_minor = MKDEV(la9310_limesdr_major, index);
    int ret = cdev_add(&node_data->cdev_node, major_minor, 1);
    if (ret < 0)
        return MKDEV(0, 0);

    ++la9310_limesdr_minor_idx;
    return major_minor;
}

void la9310_cdev_destroy(struct la9310_dev* myDevice)
{
    struct char_dev_data* node_data;
    for (int i = 0; i < LA9310_LIMESDR_MINOR_COUNT && i < la9310_limesdr_minor_idx; ++i)
    {
        if (la9310_cdevs[i].myDevice == myDevice)
        {
            node_data = &la9310_cdevs[i];
            break;
        }
    }
    if (node_data)
        cdev_del(&node_data->cdev_node);
}

int la9310_limesdr_alloc_chrdev_region()
{
    int ret = alloc_chrdev_region(&la9310_limesdr_dev_t, 0, LA9310_LIMESDR_MINOR_COUNT, LA9310_LIMESDR_DRIVER_NAME);
    if (ret < 0)
        pr_err("%s (%i): Could not allocate char device\n", __func__, ret);

    la9310_limesdr_major = MAJOR(la9310_limesdr_dev_t);
    la9310_limesdr_minor_idx = MINOR(la9310_limesdr_dev_t);
    return ret;
}

void la9310_limesdr_unregister_chrdev_region()
{
    unregister_chrdev_region(la9310_limesdr_dev_t, LA9310_LIMESDR_MINOR_COUNT);
}

dev_t la9310_get_major_minor(struct la9310_dev* myDevice)
{
    for (int i = 0; i < LA9310_LIMESDR_MINOR_COUNT && i < la9310_limesdr_minor_idx; ++i)
    {
        if (la9310_cdevs[i].myDevice == myDevice)
            return la9310_cdevs[i].cdev_node.dev;
    }
    return 0;
}