
#include <linux/kernel.h>
#include <linux/slab.h>

#include <linux/ioport.h>

#include "la9310_memory.h"

static int scratch_buf_size = 0;
static uint64_t scratch_buf_phys_addr = 0;

static struct gen_pool *reserved_physical_mem_pool = NULL;

int la9310_initialize_reserved_memory_allocator(void)
{
	if (scratch_buf_phys_addr != 0)
	{
		// use user provided RAM chunk
		if (scratch_buf_size < 0 || scratch_buf_size > LA9310_MAX_SCRATCH_BUF_SIZE)
		{
			pr_err("Invalid scratch_buf_size %i\n", scratch_buf_size);
			return -ENOMEM;
		}

		const int min_alloc_order = 6; // 2^6
		const int nodeid = -1; // don't care
		reserved_physical_mem_pool = gen_pool_create(min_alloc_order, nodeid);

		struct resource* res = request_mem_region(scratch_buf_phys_addr, scratch_buf_size, "la9310_scratch_buffer");
		if (!res)
		{
			pr_err("Failed request mem region\n");
			return -1;
		}

		int ret = gen_pool_add(reserved_physical_mem_pool, (unsigned long)scratch_buf_phys_addr, scratch_buf_size, nodeid);
		if (ret)
		{
			gen_pool_destroy(reserved_physical_mem_pool);
			return ret;
		}
		pr_info("LA9310 Reserved memory pool: pa:%llx, size:%i\n", scratch_buf_phys_addr, scratch_buf_size);
	}
	return 0;
}

void la9310_free_reserved_memory_allocator(void)
{
	if (reserved_physical_mem_pool)
	{
		gen_pool_destroy(reserved_physical_mem_pool);
		release_mem_region(scratch_buf_phys_addr, scratch_buf_size);
	}
}

struct gen_pool* la9310_get_physical_memory_pool(void)
{
	return reserved_physical_mem_pool;
}

module_param(scratch_buf_size, int, 0);
MODULE_PARM_DESC(scratch_buf_size, "Scratch buffer size for LA9310 Device");
module_param(scratch_buf_phys_addr, ullong, 0);
MODULE_PARM_DESC(scratch_buf_phys_addr, "Scratch buffer start physical address");