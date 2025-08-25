/* SPDX-License-Identifier: (BSD-3-Clause OR GPL-2.0)
 * Copyright 2017-2018, 2021-2024 NXP
 */


#include <linux/irq.h>
#include <linux/interrupt.h>
#include "la9310_base.h"

#include "common_headers/la9310_host_if.h"
#include "common_headers/la9310_pci_def.h"

/* IRQ event regitster masks */
#define IRQ_EVT_EN_OFFSET	0x1
#define NIRQ_WORDS_MASK		0xff
#define NIRQ_MASK		0xff00

/* IRQ event clear message unit interrupt bit num */
#define LA9310_RAISE_CLR_EVT	0x0

#define la9310_mem_r(addr) readl(addr)
#define la9310_mem_w(val, addr) writel(val, addr)

static int la9310_init_irq_mux(struct irq_evt_regs *irq_evt_regs,
		     struct la9310_irq_mux_pram *irq_mux)
{
	u32 num_irq_words = la9310_mem_r(&irq_evt_regs->irq_evt_cfg) &
		NIRQ_WORDS_MASK;

	irq_mux->irq_evt_cfg_reg = &irq_evt_regs->irq_evt_cfg;
	irq_mux->irq_evt_en_reg = irq_mux->irq_evt_cfg_reg + IRQ_EVT_EN_OFFSET;
	irq_mux->irq_evt_sts_reg = irq_mux->irq_evt_en_reg + num_irq_words;
	irq_mux->irq_evt_clr_reg = irq_mux->irq_evt_sts_reg + num_irq_words;
	irq_mux->irq_base = irq_alloc_descs(-1, 0, irq_mux->num_irq, 0);

	return irq_mux->irq_base;
}

static ssize_t la9310_show_irq_mux_stats(void *irq_mux_stats, char *buf,
				      struct la9310_dev *la9310_dev)
{
	int print_len;
	struct la9310_irq_mux_stats *irq_stats =
		(struct la9310_irq_mux_stats *)irq_mux_stats;

	if ((la9310_dev->stats_control & (1 << HOST_CONTROL_IRQ_STATS)) == 0)
		return 0;

	print_len = sprintf(buf, "Host VIRQ stats\n");
	print_len += sprintf((buf + print_len), "num_virq_evt_raised: %7lu\n",
			    irq_stats->num_virq_evt_raised);
	print_len += sprintf((buf + print_len), "num_hw_irq_recv: %7lu\n",
			     irq_stats->num_hw_irq_recv);
	print_len += sprintf((buf + print_len),
			     "num_msg_unit_irq_evt_raised: %7lu\n",
			     irq_stats->num_msg_unit_irq_evt_raised);

	return print_len;
}

static void la9310_reset_irq_mux_stats(void *irq_mux_stats)
{
	struct la9310_irq_mux_stats *irq_stats =
		(struct la9310_irq_mux_stats *)irq_mux_stats;

	irq_stats->num_virq_evt_raised = 0;
	irq_stats->num_hw_irq_recv = 0;
	irq_stats->num_msg_unit_irq_evt_raised = 0;
}

/* -------------------- IRQ chip functions -------------------- */
static void la9310_set_imask(struct la9310_irq_mux_pram *irq_mux, int irq, u32 set_flg)
{
	int irq_bit = irq - irq_mux->irq_base;
	unsigned long irq_evt_en_reg;

	irq_evt_en_reg = la9310_mem_r(irq_mux->irq_evt_en_reg + (irq_bit /
					LA9310_EVT_BTS_PER_WRD));
	if (set_flg)
		set_bit((irq_bit % LA9310_EVT_BTS_PER_WRD),
			(void *)&irq_evt_en_reg);
	else
		clear_bit((irq_bit %
			   LA9310_EVT_BTS_PER_WRD), (void *)&irq_evt_en_reg);
	la9310_mem_w(irq_evt_en_reg, irq_mux->irq_evt_en_reg + (irq_bit /
					LA9310_EVT_BTS_PER_WRD));
}

static void la9310_irq_mask(struct irq_data *d)
{
	struct la9310_irq_mux_pram *irq_mux = irq_data_get_irq_chip_data(d);

	la9310_set_imask(irq_mux, d->irq, 0);

}

static void la9310_irq_unmask(struct irq_data *d)
{
	struct la9310_irq_mux_pram *irq_mux = irq_data_get_irq_chip_data(d);

	la9310_set_imask(irq_mux, d->irq, 1);
}

static struct irq_chip la9310_irq_chip = {
	.name			= "la9310_irq_mux",
	.irq_mask		= la9310_irq_mask,
	.irq_unmask		= la9310_irq_unmask,
};

static void la9310_set_irq_chip(struct la9310_irq_mux_pram *irq_mux)
{
	int i;
	int irq;

	for (i = 0; i < irq_mux->num_irq; i++) {
		unsigned long clr = 0, set = IRQ_NOREQUEST |
				IRQ_NOPROBE | IRQ_NOAUTOEN;
		irq = irq_mux->irq_base + i;
		irq_set_chip_data(irq, (void *)irq_mux);
		/* Setup irq  */
		irq_set_chip_and_handler(irq, &la9310_irq_chip,
						handle_simple_irq);
		irq_clear_status_flags(irq, IRQ_NOREQUEST);
		//set_irq_flags(irq, IRQF_VALID); /* Deprecated */
		clr |= IRQ_NOREQUEST;
		irq_modify_status(irq, clr, set & ~clr);
	}
}

static void create_virq_bit_map(struct la9310_dev *la9310_dev,
			 struct la9310_irq_mux_pram *irq_mux)
{
	u32 num_irq = irq_mux->num_irq;
	struct virq_evt_map *virq_map = irq_mux->virq_map;
	int i;

	for (i = 0; i < IRQ_EVT_LAST_BIT; i++) {
		if (num_irq) {
				virq_map  = (irq_mux->virq_map + i);
				virq_map->evt = i;
				virq_map->virq = irq_mux->irq_base + i;
				dev_dbg(la9310_dev->dev,
				       "%s: virq_map: %p, evt: %d, virq: %d\n",
					__func__, virq_map, virq_map->evt,
					virq_map->virq);
		} else {
			dev_warn(la9310_dev->dev, "Num irq insufficient");
			break;
		}
		num_irq -= 1;
	}
}

static int la9310_init_irq_stats(struct la9310_dev *la9310_dev,
			struct la9310_irq_mux_pram *irq_mux)
{
	struct la9310_stats_ops stats_ops;

	stats_ops.la9310_show_stats = la9310_show_irq_mux_stats;
	stats_ops.la9310_reset_stats = la9310_reset_irq_mux_stats;
	stats_ops.stats_args = (void *)&la9310_dev->la9310_irq_priv->irq_stats;
	// TODO: return la9310_host_add_stats(la9310_dev, &stats_ops);
	return 0;
}

static int la9310_probe_irq_mux(struct la9310_dev *la9310_dev,
		struct irq_evt_regs *irq_evt_regs)
{
	struct la9310_irq_mux_pram *irq_mux;
	u32 num_irq = (la9310_mem_r(&irq_evt_regs->irq_evt_cfg) & NIRQ_MASK) >>
									    8;
	int ret = 0;

	irq_mux = kzalloc(sizeof(struct la9310_irq_mux_pram), GFP_KERNEL);
	if (irq_mux == NULL)
		return -ENOMEM;
	irq_mux->virq_map = kzalloc((sizeof(struct virq_evt_map) * num_irq),
				    GFP_KERNEL);
	if (irq_mux->virq_map == NULL) {
		ret = -ENOMEM;
		goto err1;
	}
	la9310_dev->la9310_irq_priv = irq_mux;
	irq_mux->num_irq = num_irq;
	irq_mux->irq_base = la9310_init_irq_mux(irq_evt_regs, irq_mux);
	if (irq_mux->irq_base < 0) {
		dev_err(la9310_dev->dev,
			"Failed to allocate IRQ irq_base:	%d\n",
			irq_mux->irq_base);
		ret = irq_mux->irq_base;
		goto err2;
	}
	create_virq_bit_map(la9310_dev, la9310_dev->la9310_irq_priv);
	la9310_set_irq_chip(irq_mux);
	ret = la9310_init_irq_stats(la9310_dev, irq_mux);
	if (ret < 0)
		goto err2;
	return ret;
err2:
	kfree(irq_mux->virq_map);
err1:
	kfree(irq_mux);
	return ret;
}

static void la9310_irq_processed(int irq, void *dev)
{
	struct la9310_dev *la9310_dev = (struct la9310_dev *)dev;
	struct la9310_irq_mux_pram *irq_mux = la9310_dev->la9310_irq_priv;
	unsigned long irq_evt_clr_reg;
	struct la9310_msg_unit *ccsr_mur;
	u32 *msiir1_reg;
	u8 *mem;
	int irq_bit = irq - irq_mux->irq_base;

	mem = (la9310_dev->mem_regions[LA9310_MEM_REGION_CCSR].vaddr +
		MSG_UNIT_OFFSET);
	ccsr_mur = (struct la9310_msg_unit *)mem;
	ccsr_mur += LA9310_MSG_UNIT_1;

	irq_evt_clr_reg = la9310_mem_r(irq_mux->irq_evt_clr_reg + (irq_bit /
				LA9310_EVT_BTS_PER_WRD));
	set_bit((irq_bit %
		   LA9310_EVT_BTS_PER_WRD), (void *)&irq_evt_clr_reg);
	la9310_mem_w(irq_evt_clr_reg, irq_mux->irq_evt_clr_reg + (irq_bit /
					LA9310_EVT_BTS_PER_WRD));

	dma_wmb();
	/* raise message unit interrupt to la9310 */
	msiir1_reg = &ccsr_mur->msiir;
	writel(LA9310_RAISE_CLR_EVT, msiir1_reg);
	irq_mux->irq_stats.num_msg_unit_irq_evt_raised++;
}

static irqreturn_t la9310_irq_handler(int irq, void *dev)
{
	struct la9310_dev *la9310_dev = (struct la9310_dev *)dev;
	struct la9310_irq_mux_pram *irq_mux = la9310_dev->la9310_irq_priv;
	int i;
	int j;
	int sts_bits;
	int match = 0;
	int num_words = la9310_mem_r(irq_mux->irq_evt_cfg_reg) &
							NIRQ_WORDS_MASK;
	u32 num_irq = irq_mux->num_irq;
	u32 irq_evt_sts_reg;
	u32 irq_evt_en_reg;
	int irq_base;

	irq_mux->irq_stats.num_hw_irq_recv++;
	for (j = 0; j < num_words; j++) {
		sts_bits = (num_irq < LA9310_EVT_BTS_PER_WRD) ? num_irq :
			LA9310_EVT_BTS_PER_WRD;
		irq_base = irq_mux->irq_base + (j * LA9310_EVT_BTS_PER_WRD);
		irq_evt_sts_reg = la9310_mem_r(irq_mux->irq_evt_sts_reg + j);
		irq_evt_en_reg = la9310_mem_r(irq_mux->irq_evt_en_reg + j);
		for (i = 0; i < sts_bits; i++) {
			if ((irq_evt_sts_reg & BIT(i)) & irq_evt_en_reg) {
				generic_handle_irq(irq_base + i);
				match = 1;
				la9310_irq_processed((irq_base + i), dev);
				irq_mux->irq_stats.num_virq_evt_raised++;
			}
		}
	}

	if (!match)
		dev_warn(la9310_dev->dev,
			 "No irq_mux line matched irq %d\n", irq);

	return IRQ_HANDLED;
}

int la9310_request_irq(struct la9310_dev *la9310_dev,
			struct irq_evt_regs *irq_evt_regs)
{
	int err = 0;
	u32 num_irq = (la9310_mem_r(&irq_evt_regs->irq_evt_cfg) & NIRQ_MASK) >> 8;
    dev_info(la9310_dev->dev,"num_irq %d",num_irq);

	la9310_create_outbound_msi(la9310_dev);
	if (!num_irq) {
		dev_err(la9310_dev->dev,
			"Invalid la9310 config! 'num irq 0'\n");
		return -EINVAL;
	}
	err = la9310_probe_irq_mux(la9310_dev, irq_evt_regs);
	if (!err) {
		err = request_irq(la9310_get_msi_irq(la9310_dev, MSI_IRQ_MUX),
					la9310_irq_handler, 0, la9310_dev->name,
					(void *)la9310_dev);
		if (!err) {
			dev_info(la9310_dev->dev, "irq mux %d allocated successfully\n",
				la9310_get_msi_irq(la9310_dev, MSI_IRQ_MUX));
		} else {
			irq_free_descs(la9310_dev->la9310_irq_priv->irq_base,
				num_irq);
			kfree(la9310_dev->la9310_irq_priv);
			la9310_dev->la9310_irq_priv = NULL;
		}
	}

	return err;
}

int la9310_clean_request_irq(struct la9310_dev *la9310_dev,
			struct irq_evt_regs *irq_evt_regs)
{
	u32 num_irq = (la9310_mem_r(&irq_evt_regs->irq_evt_cfg) & NIRQ_MASK) >>
									    8;

	if (la9310_dev->la9310_irq_priv) {
		if (num_irq && la9310_dev->la9310_irq_priv->irq_base)
			irq_free_descs(la9310_dev->la9310_irq_priv->irq_base,
					num_irq);
		kfree(la9310_dev->la9310_irq_priv->virq_map);
		kfree(la9310_dev->la9310_irq_priv);
	}
	return 0;
}
