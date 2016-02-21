/* Copyright (c) 2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <media/rc-core.h>
//specifique
#include <mach/irqs.h>
#include <mach/system.h>
#include <mach/hardware.h>
#include <plat/sys_config.h>
#include <linux/clk.h>
#include <linux/of.h>
#include "sunxi_lirc_new.h"


#define SUNXI_IR_DRIVER_NAME	"ir-rc-recv"
#define SUNXI_IR_DEVICE_NAME	"sunxi_ir_recv"

struct sunxi_ir {

	spinlock_t      ir_lock;
	struct rc_dev   *rcdev;
	int             irq;
	struct clk      *clk;
	struct clk      *apb_clk;
	u32             ir_gpio_hdle;
	const char      *map_name;


};
static int debug = 0;
static irqreturn_t sunxi_ir_recv_irq(int irq, void *dev_id)
{
	unsigned long status;
	unsigned char dt;
	unsigned int cnt, rc;
	struct sunxi_ir *ir = dev_id;
	DEFINE_IR_RAW_EVENT(rawir);
	spin_lock(&ir->ir_lock);

	status = readl(IR_BASE + SUNXI_IR_RXSTA_REG);

	/* clean all pending statuses */
	writel(status | REG_RXSTA_CLEARALL, IR_BASE + SUNXI_IR_RXSTA_REG);

	if (status & REG_RXINT_RAI_EN) {
		/* How many messages in fifo */
		rc  = REG_RXSTA_GET_AC(status);
		/* Sanity check */
		rc = rc > SUNXI_IR_FIFO_SIZE ? SUNXI_IR_FIFO_SIZE : rc;
		/* If we have data */
		for (cnt = 0; cnt < rc; cnt++) {
			/* for each bit in fifo */
			dt = readb(IR_BASE + SUNXI_IR_RXFIFO_REG);
			rawir.pulse = (dt & 0x80) != 0;
			rawir.duration = ((dt & 0x7f) + 1) * SUNXI_IR_SAMPLE;
			ir_raw_event_store_with_filter(ir->rcdev, &rawir);
		}
	}

	if (status & REG_RXINT_ROI_EN) {
		ir_raw_event_reset(ir->rcdev);
	} else if (status & REG_RXINT_RPEI_EN) {
		ir_raw_event_set_idle(ir->rcdev, true);
		ir_raw_event_handle(ir->rcdev);
	}

	spin_unlock(&ir->ir_lock);

	return IRQ_HANDLED;

}

static int __devinit sunxi_ir_recv_probe(struct platform_device *pdev)
{
	struct sunxi_ir *sunxi_dev;
	struct device *dev = &pdev->dev;
	struct rc_dev *rcdev;
	const struct sunxi_ir_recv_platform_data *pdata =
					pdev->dev.platform_data;
    struct device_node *dn = dev->of_node;
	int rc;
	unsigned long tmp;
	unsigned long rate = SUNXI_IR_BASE_CLK; /* 8 MHz */

	if (!pdata)
		return -EINVAL;

	/*if (pdata->gpio_nr < 0) ///un test sur le contenu des data
		return -EINVAL;*/

	sunxi_dev = kzalloc(sizeof(struct sunxi_ir), GFP_KERNEL);
	if (!sunxi_dev)
		return -ENOMEM;

	rcdev = rc_allocate_device();
	if (!rcdev) {
		rc = -ENOMEM;
		goto err_allocate_device;
	}
    sunxi_dev->map_name = of_get_property(dn, "linux,rc-map-name", NULL);
	rcdev->driver_type = RC_DRIVER_IR_RAW;
	rcdev->allowed_protos = RC_TYPE_ALL;
	rcdev->input_name = SUNXI_IR_DEVICE_NAME;
	rcdev->input_id.bustype = BUS_HOST;
	rcdev->driver_name = SUNXI_IR_DRIVER_NAME;
	rcdev->map_name = sunxi_dev->map_name ?: RC_MAP_EMPTY;
	rcdev->input_phys = "sunxi-ir/input0";
	rcdev->input_id.vendor = 0x0001;
	rcdev->input_id.product = 0x0001;
	rcdev->input_id.version = 0x0100;
    rcdev->dev.parent = dev;
    rcdev->rx_resolution = SUNXI_IR_SAMPLE;
    rcdev->timeout = MS_TO_NS(SUNXI_IR_TIMEOUT);
	rcdev->priv = sunxi_dev;

	sunxi_dev->rcdev = rcdev;



	/*debut initialisation hardware */

	sunxi_dev->irq = IR_IRQNO;
	if (request_irq(sunxi_dev->irq, sunxi_ir_recv_irq, 0, "RemoteIR",
			sunxi_dev)) {
		rc = -EBUSY;
		goto exit_irq;
	}
	dprintk("ir_setup: ir setup start!!\n");
	sunxi_dev->ir_gpio_hdle = gpio_request_ex("ir_para", "ir0_rx");
	if (0 == sunxi_dev->ir_gpio_hdle)
		printk(KERN_ERR "try to request ir_para gpio failed\n");

	sunxi_dev->apb_clk = clk_get(NULL, "apb_ir0");
	if (!sunxi_dev->apb_clk) {
		printk(KERN_ERR "try to get apb_ir0 clock failed\n");
		rc = -ENODATA;
		goto exit_clk_apb;

	}

	sunxi_dev->clk = clk_get(NULL, "ir0");
	if (!sunxi_dev->clk) {
		printk(KERN_ERR "try to get ir0 clock failed\n");
		rc = -ENODATA;
		goto exit_clk;
	}
	dprintk("trying to set clock via SYS_CLK_CFG_EN, when no error follows -> succeeded\n");
	if (clk_set_rate(sunxi_dev->clk, rate))
		printk(KERN_ERR "set ir0 clock freq to 8M failed\n");

	if (clk_enable(sunxi_dev->apb_clk))
		printk(KERN_ERR "try to enable apb_ir_clk failed\n");

	if (clk_enable(sunxi_dev->clk))
		printk(KERN_ERR "try to enable apb_ir_clk failed\n");

	/* Enable CIR Mode */
    writel(REG_CTL_MD, IR_BASE + SUNXI_IR_CTL_REG);
    /* Set noise threshold and idle threshold and clock divisor*/
    writel(REG_CIR_NTHR(SUNXI_IR_RXNOISE) | REG_CIR_ITHR(SUNXI_IR_RXIDLE),
   IR_BASE  + SUNXI_IR_CIR_REG);
    /* Invert Input Signal */
    writel(REG_RXCTL_RPPI, IR_BASE + SUNXI_IR_RXCTL_REG);
    /* Clear All Rx Interrupt Status */
    writel(REG_RXSTA_CLEARALL, IR_BASE + SUNXI_IR_RXSTA_REG);
    /*
    * Enable IRQ on overflow, packet end, FIFO available with trigger
    * level
    */
    writel(REG_RXINT_ROI_EN | REG_RXINT_RPEI_EN |
    REG_RXINT_RAI_EN | REG_RXINT_RAL(TRIGGER_LEVEL - 1),
     IR_BASE+ SUNXI_IR_RXINT_REG);
    /* Enable IR Module */
    tmp = readl(IR_BASE + SUNXI_IR_CTL_REG);
    writel(tmp | REG_CTL_GEN | REG_CTL_RXEN, IR_BASE + SUNXI_IR_CTL_REG);
    printk(KERN_INFO "initialized sunXi IR driver\n");

	printk(KERN_INFO "ir_setup: ir setup end!!\n");

    /*rc dev stuf */
	rc = rc_register_device(rcdev);
	if (rc < 0) {
		dev_err(&pdev->dev, "failed to register rc device\n");
		goto err_register_rc_device;
	}

	platform_set_drvdata(pdev, sunxi_dev);
    printk(KERN_INFO "device registred\n");

	return 0;

err_register_rc_device:
	clk_put(sunxi_dev->clk);
exit_clk:
    clk_put(sunxi_dev->apb_clk);
exit_clk_apb:
	free_irq(sunxi_dev->irq, sunxi_dev);
exit_irq:
	rc_free_device(rcdev);
	rcdev = NULL;
err_allocate_device:
	kfree(sunxi_dev);
	return rc;
}

static int __devexit sunxi_ir_recv_remove(struct platform_device *pdev)
{
	struct sunxi_ir *sunxi_dev = platform_get_drvdata(pdev);
    free_irq(sunxi_dev->irq, sunxi_dev);
	gpio_release(sunxi_dev->ir_gpio_hdle, 2);
	clk_put(sunxi_dev->apb_clk);
	clk_put(sunxi_dev->clk);
	platform_set_drvdata(pdev, NULL);
	rc_unregister_device(sunxi_dev->rcdev);
	rc_free_device(sunxi_dev->rcdev);
	kfree(sunxi_dev);
	return 0;
}


static struct platform_driver sunxi_ir_recv_driver = {
	.probe  = sunxi_ir_recv_probe,
	.remove = __devexit_p(sunxi_ir_recv_remove),
	.driver = {
		.name   = SUNXI_IR_DRIVER_NAME,
		.owner  = THIS_MODULE,
	},
};

static int __init sunxi_ir_recv_init(void)
{
	return platform_driver_register(&sunxi_ir_recv_driver);
}
module_init(sunxi_ir_recv_init);

static void __exit sunxi_ir_recv_exit(void)
{
	platform_driver_unregister(&sunxi_ir_recv_driver);
}
module_exit(sunxi_ir_recv_exit);

MODULE_DESCRIPTION("SUNXI IR Receiver driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Damien Pageot");
module_param(debug, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Enable debugging messages");

