/*
 *	irq->	hirq->	pin->	vhwirq->	rirq(remapped irq)
 *GPIO0	40	100	0	vb+0	rib+0 200
 *GPIO1	40	100	1	vb+1	rib+1 201
 *GPIO2	41	101	2	vb+2	rib+2 202
 *GPIO3	41	101	3	vb+3	rib+3 203
 *		|HIRQ-PIN-MAP	|VIRQ-IRQ-MAP
 *		|hirq-pin-group	|IRQ DOMAIN & CHIPGENERIC
 *				|vhwirq-base irq-base
 * */
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/syscore_ops.h>
#include <linux/gpio/driver.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/bug.h>

#define MAX_IRQNUM_PER_CHIP 64

#define REG_GPIO_DIR(s)	(s->regtable->dir)
#define REG_GPIO_OUT(s)	(s->regtable->out)
#define REG_GPIO_IN(s)	(s->regtable->in)
#define REG_GPIO_IRQ(s)	(s->regtable->irq)
#define REG_GPIO_IRQPOL(s)	(s->regtable->irqpol)
#define REG_GPIO_IRQEDG(s)	(s->regtable->irqedg)
#define REG_GPIO_IRQCLR(s)	(s->regtable->irqclr)
#define REG_GPIO_IRQSTA(s)	(s->regtable->irqsta)
#define REG_GPIO_IRQDUL(s)	(s->regtable->irqdul)

#define lsirq_gpio_set_reg_dir(self, p, val) \
	writeb(val, self->reg + REG_GPIO_DIR(self) + p)
#define lsirq_gpio_set_reg_out(self, p, val) \
	writeb(val, self->reg + REG_GPIO_OUT(self) + p)
#define lsirq_gpio_get_reg_in(self, p) \
	readb(self->reg + REG_GPIO_IN(self) + p)
#define lsirq_gpio_set_reg_irq(self, p, val) \
	writeb(val, self->reg + REG_GPIO_IRQ(self) + p)
#define lsirq_gpio_get_reg_irq(self, p) \
	readb(self->reg + REG_GPIO_IRQ(self) + p)
#define lsirq_gpio_set_reg_irqpol(self, p, val) \
	writeb(val, self->reg + REG_GPIO_IRQPOL(self) + p)
#define lsirq_gpio_set_reg_irqedg(self, p, val) \
	writeb(val, self->reg + REG_GPIO_IRQEDG(self) + p)
#define lsirq_gpio_set_reg_irqclr(self, p, val) \
	writeb(val, self->reg + REG_GPIO_IRQCLR(self) + p)
#define lsirq_gpio_get_reg_irqsta(self, p) \
	readb(self->reg + REG_GPIO_IRQSTA(self) + p)
#define lsirq_gpio_set_reg_irqdul(self, p, val) \
	writeb(val, self->reg + REG_GPIO_IRQDUL(self) + p)

typedef struct lsirq_gpio_regtable_s {
	u32 dir;
	u32 out;
	u32 in;
	u32 irq;
	u32 irqpol;
	u32 irqedg;
	u32 irqclr;
	u32 irqsta;
	u32 irqdul;
} lsirq_gpio_regtable_t;

typedef struct lsirq_gpio_irqpinmap_s {
	int irq;
	int hwirq;
	int pin_start;
	int pin_num;
	void* parent;
	struct list_head next;
} lsirq_gpio_irqpinmap_t;

typedef struct lsirq_gpio_irqdesc_s {
	char name[32];
	int rirq_base;
	int vhwirq_base;
	int nr_irqs;
	// HIRQ-PIN MAP
	struct list_head ipm;
	// VIRQ-IRQ MAP
	struct irq_domain* domain;
	struct irq_chip* ic;
} lsirq_gpio_irqdesc_t;

typedef struct lsirq_gpio_priv_s {
	spinlock_t	lock;
	void __iomem*	reg;
	struct device*	dev;
	struct gpio_chip	gc;
	lsirq_gpio_irqdesc_t	irqd;
	const lsirq_gpio_regtable_t*	regtable;
} lsirq_gpio_priv_t;

static lsirq_gpio_regtable_t generic_reg_table = {
	.dir	= 0x800,
	.out	= 0x900,
	.in	= 0xa00,
	.irq	= 0xb00,
	.irqpol = 0xc00,
	.irqedg = 0xd00,
	.irqclr = 0xe00,
	.irqsta = 0xf00,
	.irqdul = 0xf80,
};

static int __map_hwirq_pin(lsirq_gpio_irqpinmap_t* map, int hwirq,
		lsirq_gpio_priv_t* priv)
{
	int pin;

	if (map->hwirq == hwirq)
	{
		for (pin = map->pin_start;
			pin < map->pin_start + map->pin_num;
			pin++)
		{
			// 2k300 must check irq enable
			if (lsirq_gpio_get_reg_irq(priv, pin) == 1 &&
				lsirq_gpio_get_reg_irqsta(priv, pin) == 1)
				return pin;
		}
	}

	return -1;
}

static int __map_vhwirq_pin(lsirq_gpio_priv_t* priv, int vhwirq)
{
	return vhwirq - priv->irqd.vhwirq_base;
}

static int __map_pin_vhwirq(lsirq_gpio_priv_t* priv, int pin)
{
	return pin + priv->irqd.vhwirq_base;
}

static int __map_vhwirq_rirq(lsirq_gpio_priv_t* priv, int vhwirq)
{
	return irq_find_mapping(priv->irqd.domain, vhwirq);
}

static void irqfhd_lsirq_gpio_handler(struct irq_desc *desc)
{
	lsirq_gpio_irqpinmap_t* map = irq_desc_get_handler_data(desc);
	lsirq_gpio_priv_t *priv = map->parent;
	struct irq_chip *chip = irq_desc_get_chip(desc);
	int pin, vhwirq, rirq;

	chained_irq_enter(chip, desc);

	pin = __map_hwirq_pin(map, desc->irq_data.hwirq, priv);
	if (pin < 0)
		goto end;
	lsirq_gpio_set_reg_irqclr(priv, pin, 1);

	vhwirq = __map_pin_vhwirq(priv, pin);
	rirq = __map_vhwirq_rirq(priv, vhwirq);
	generic_handle_irq(rirq);
end:
	chained_irq_exit(chip, desc);
}

static int mthd_lsirq_gpio_request(struct gpio_chip *gc, unsigned pin)
{
	if (pin >= gc->ngpio)
		return -EINVAL;
	else
		return 0;
}

static int mthd_lsirq_gpio_direction_input(struct gpio_chip *gc, unsigned pin)
{
	lsirq_gpio_priv_t *priv = gpiochip_get_data(gc);
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	lsirq_gpio_set_reg_dir(priv, pin, 1);
	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static int mthd_lsirq_gpio_direction_output(struct gpio_chip *gc,
		unsigned pin, int value)
{
	lsirq_gpio_priv_t *priv = gpiochip_get_data(gc);
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	lsirq_gpio_set_reg_out(priv, pin, value);
	lsirq_gpio_set_reg_dir(priv, pin, 0);
	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static int mthd_lsirq_gpio_get(struct gpio_chip *gc, unsigned pin)
{
	lsirq_gpio_priv_t *priv = gpiochip_get_data(gc);

	return lsirq_gpio_get_reg_in(priv, pin) & 1;
}

static void mthd_lsirq_gpio_set(struct gpio_chip *gc, unsigned pin, int value)
{
	lsirq_gpio_priv_t *priv = gpiochip_get_data(gc);
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	lsirq_gpio_set_reg_out(priv, pin, value);
	spin_unlock_irqrestore(&priv->lock, flags);
}

static int mthd_lsirq_gpio_to_irq(struct gpio_chip *gc, unsigned pin)
{
	int vhwirq, rirq;
	lsirq_gpio_priv_t *priv = gpiochip_get_data(gc);

	vhwirq = __map_pin_vhwirq(priv, (int)pin);
	rirq = __map_vhwirq_rirq(priv, vhwirq);

	return rirq;
}

static int mthd_lsirq_gpio_irqsettype(struct irq_data *d, u32 type)
{
	lsirq_gpio_priv_t *priv = irq_data_get_irq_chip_data(d);
	int vhwirq = d->hwirq;
	int pin = __map_vhwirq_pin(priv, vhwirq);
	unsigned long flags;

	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		lsirq_gpio_set_reg_irq(priv, pin, 1);
		lsirq_gpio_set_reg_irqdul(priv, pin, 0);
		lsirq_gpio_set_reg_irqpol(priv, pin, 1);
		lsirq_gpio_set_reg_irqedg(priv, pin, 1);
		break;
	case IRQ_TYPE_EDGE_FALLING:
		lsirq_gpio_set_reg_irq(priv, pin, 1);
		lsirq_gpio_set_reg_irqdul(priv, pin, 0);
		lsirq_gpio_set_reg_irqpol(priv, pin, 0);
		lsirq_gpio_set_reg_irqedg(priv, pin, 1);
		break;
	case IRQ_TYPE_EDGE_BOTH:
		lsirq_gpio_set_reg_irq(priv, pin, 1);
		lsirq_gpio_set_reg_irqdul(priv, pin, 1);
		lsirq_gpio_set_reg_irqedg(priv, pin, 1);
		break;
	case IRQ_TYPE_LEVEL_LOW:
		lsirq_gpio_set_reg_irq(priv, pin, 1);
		lsirq_gpio_set_reg_irqdul(priv, pin, 0);
		lsirq_gpio_set_reg_irqpol(priv, pin, 0);
		lsirq_gpio_set_reg_irqedg(priv, pin, 0);
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		lsirq_gpio_set_reg_irq(priv, pin, 1);
		lsirq_gpio_set_reg_irqdul(priv, pin, 0);
		lsirq_gpio_set_reg_irqpol(priv, pin, 1);
		lsirq_gpio_set_reg_irqedg(priv, pin, 0);
		break;
	case IRQ_TYPE_NONE:
		lsirq_gpio_set_reg_irq(priv, pin, 0);
		break;
	default:
		return -EINVAL;
	}
	spin_lock_irqsave(&priv->lock, flags);
	lsirq_gpio_set_reg_dir(priv, pin, 1);
	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static void mthd_lsirq_gpio_irqack(struct irq_data *d)
{

}

static void mthd_lsirq_gpio_irqmask(struct irq_data *d)
{
	lsirq_gpio_priv_t *priv = irq_data_get_irq_chip_data(d);
	int vhwirq = d->hwirq;
	int pin = __map_vhwirq_pin(priv, vhwirq);
	unsigned long flags;

	// interrupt must gpio input
	spin_lock_irqsave(&priv->lock, flags);
	lsirq_gpio_set_reg_dir(priv, pin, 1);
	spin_unlock_irqrestore(&priv->lock, flags);

	lsirq_gpio_set_reg_irq(priv, pin, 0);
}

static void mthd_lsirq_gpio_irqunmask(struct irq_data *d)
{
	lsirq_gpio_priv_t *priv = irq_data_get_irq_chip_data(d);
	int vhwirq = d->hwirq;
	int pin = __map_vhwirq_pin(priv, vhwirq);

	lsirq_gpio_set_reg_irq(priv, pin, 1);
}

static int lsirq_gpio_gc_remove(lsirq_gpio_priv_t* priv)
{
	struct gpio_chip* gc = &priv->gc;

	gpiochip_remove(gc);

	return 0;
}

static int lsirq_gpio_gc_setup(lsirq_gpio_priv_t* priv)
{
	struct gpio_chip* gc = &priv->gc;
	struct device_node *np = priv->dev->of_node;

	of_property_read_u32(np, "ngpios", (u32 *)&gc->ngpio);
	of_property_read_u32(np, "gpio_base", (u32 *)&gc->base);
	gc->request = mthd_lsirq_gpio_request;
	gc->direction_input = mthd_lsirq_gpio_direction_input;
	gc->get = mthd_lsirq_gpio_get;
	gc->direction_output = mthd_lsirq_gpio_direction_output;
	gc->set = mthd_lsirq_gpio_set;
	gc->to_irq = mthd_lsirq_gpio_to_irq;
	gc->can_sleep = 0;
	gc->fwnode = &np->fwnode;
	gc->parent = priv->dev;

	if (devm_gpiochip_add_data(priv->dev, gc, priv))
	{
		dev_err(priv->dev, "gpiochip setup fail\n");
		return -1;
	}

	return 0;
}

static int lsirqmap_fill(struct list_head* map,
		struct device_node *np, void* parent)
{
	int i = 0;
	int irq, hwirq, pin_start, pin_num, nr_irqs;
	struct irq_desc *desc;
	lsirq_gpio_irqpinmap_t* m;

	nr_irqs = 0;

	INIT_LIST_HEAD(map);
	for (i = 0; i < MAX_IRQNUM_PER_CHIP; i++)
	{
		irq = of_irq_get(np, i);
		if (irq < 0)
			break;

		desc = irq_to_desc(irq);
		if (!desc)
			break;
		hwirq = desc->irq_data.hwirq;

		if (of_property_read_u32_index(np, "loongson,irqpin-group",
					i*2, &pin_start))
			break;
		if (of_property_read_u32_index(np, "loongson,irqpin-group",
					i*2+1, &pin_num))
			break;

		m = kzalloc(sizeof(*m), GFP_KERNEL);
		if (!m)
			break;

		m->irq = irq;
		m->hwirq = hwirq;
		m->pin_start = pin_start;
		m->pin_num = pin_num;
		m->parent = parent;

		list_add_tail(&m->next, map);

		irq_set_chained_handler_and_data(irq,
				irqfhd_lsirq_gpio_handler, m);
		nr_irqs += pin_num;
	}
	return nr_irqs;
}

static int lsirqmap_clear(struct list_head* map)
{
	lsirq_gpio_irqpinmap_t* m;

	while (!list_empty(map)) {
		m = list_entry(map->next, lsirq_gpio_irqpinmap_t, next);
		list_del(&m->next);
		kfree(m);
	}
	return 0;
}

static int mthd_irq_domain_map(struct irq_domain *domain,
		unsigned int irq, irq_hw_number_t hwirq)
{
	lsirq_gpio_priv_t* priv = domain->host_data;
	lsirq_gpio_irqdesc_t* irqd = &priv->irqd;

	irq_set_chip_data(irq, priv);
	irq_set_chip_and_handler(irq, irqd->ic, handle_edge_irq);
	irq_set_probe(irq);

	return 0;
}


static const struct irq_domain_ops domain_ops_gpic = {
	.map = mthd_irq_domain_map,
	.xlate = irq_domain_xlate_onetwocell,
};

static int lsirqd_setup(lsirq_gpio_priv_t* priv)
{
	lsirq_gpio_irqdesc_t* irqd = &priv->irqd;
	struct device_node *np = priv->dev->of_node;

	of_property_read_u32(np, "loongson,vhwirq-base", (u32 *)&irqd->vhwirq_base);
	snprintf(irqd->name, sizeof(irqd->name), "GPIC-%s", np->name);

	/*
	 * Setup HWIRQ-PIN-MAP
	 * */
	irqd->nr_irqs = lsirqmap_fill(&irqd->ipm, np, priv);
	if (irqd->nr_irqs <= 0)
	{
		dev_err(priv->dev, "IRQ Pin Map fail\n");
		goto err;
	}
	/*
	 * Setup VIRQ-IRQ-MAP
	 * Use IRQ DOMAIN and chip generic
	 * */
	// find valid ib
	irqd->rirq_base = devm_irq_alloc_descs(priv->dev, -1,
			0, irqd->nr_irqs, numa_node_id());
	if (irqd->rirq_base < 0) {
		dev_err(priv->dev, "Alloc IRQ fail\n");
		goto clear_pin_map;
	}
	// set ic prop
	irqd->ic = kzalloc(sizeof(*irqd->ic), GFP_KERNEL);
	if (!irqd->ic)
	{
		dev_err(priv->dev, "IRQ chip alloc fail\n");
		goto free_irq_desc;
	}
	irqd->ic->name = irqd->name;
	irqd->ic->irq_ack = mthd_lsirq_gpio_irqack;
	irqd->ic->irq_mask = mthd_lsirq_gpio_irqmask;
	irqd->ic->irq_unmask = mthd_lsirq_gpio_irqunmask;
	irqd->ic->irq_set_type = mthd_lsirq_gpio_irqsettype;
	irqd->ic->flags = IRQCHIP_MASK_ON_SUSPEND;
	// map ib&vb
	irqd->domain = irq_domain_add_legacy(np,
			irqd->nr_irqs, irqd->rirq_base, irqd->vhwirq_base,
			&domain_ops_gpic, priv);
	if (!irqd->domain) {
		dev_err(priv->dev, "IRQ domain add fail\n");
		goto free_ic;
	}
	dev_info(priv->dev, "register irqnum=%d, vhwirq-base=%d, rirq-base=%d\n",
			irqd->nr_irqs, irqd->vhwirq_base, irqd->rirq_base);
	return 0;

free_ic:
	kfree(irqd->ic);
free_irq_desc:
	irq_free_descs(irqd->rirq_base, irqd->nr_irqs);
clear_pin_map:
	lsirqmap_clear(&irqd->ipm);
err:
	return -1;
}

static const struct of_device_id drvids_lsirq_gpio[] = {
	{ .compatible = "loongson,ls2p-gpio", .data = &generic_reg_table },
	{ .compatible = "loongson,ls2k0300-gpio", .data = &generic_reg_table },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, drvids_lsirq_gpio);

static int drv_lsirq_gpio_probe(struct platform_device *pdev)
{
	lsirq_gpio_priv_t* priv;
	const struct of_device_id *of_id =
		of_match_device(drvids_lsirq_gpio, &pdev->dev);

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->reg = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->reg))
		goto ioremap_fail;
	spin_lock_init(&priv->lock);
	priv->dev = &pdev->dev;
	priv->regtable = of_id->data;

	if (lsirq_gpio_gc_setup(priv))
		goto gc_fail;

	if (lsirqd_setup(priv))
		goto irqd_fail;

	platform_set_drvdata(pdev, priv);

	return 0;

irqd_fail:
	lsirq_gpio_gc_remove(priv);
gc_fail:
	devm_iounmap(&pdev->dev, priv->reg);
ioremap_fail:
	devm_kfree(&pdev->dev, priv);
	return -1;
}

static struct platform_driver lsirq_gpio_driver = {
	.driver		= {
		.name	= "gpio-lsirq",
		.owner	= THIS_MODULE,
		.of_match_table = drvids_lsirq_gpio,
	},
	.probe		= drv_lsirq_gpio_probe,
};

static int __init lsirq_gpio_init(void)
{
	return platform_driver_register(&lsirq_gpio_driver);
}
subsys_initcall(lsirq_gpio_init);

MODULE_AUTHOR("Yize Niu <niuyize@loongson.cn>");
MODULE_DESCRIPTION("Loongson GPIO With IRQ Driver");
MODULE_LICENSE("GPL");