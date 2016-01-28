/*
 * sunxi_lirc_new.c
 *
 *  Created on: 21 janv. 2016
 *      Author: K005425
 */

#include "sunxi_lirc_new.h"
#ifndef DEBUG
static int debug=0;
#else
static int debug=1;
#define len 200
u32 intvalue,hexvalue;
struct dentry *dirret,*fileret,*u32int,*u32hex;
char ker_buf[len];
int filevalue;
/* read file operation */
static ssize_t myreader(struct file *fp, char __user *user_buffer,
                                size_t count, loff_t *position)
{
     return simple_read_from_buffer(user_buffer, count, position, ker_buf, len);
}

/* write file operation */
static ssize_t mywriter(struct file *fp, const char __user *user_buffer,
                                size_t count, loff_t *position)
{
        if(count > len )
                return -EINVAL;

        return simple_write_to_buffer(ker_buf, len, position, user_buffer, count);
}

static const struct file_operations fops_debug = {
        .read = myreader,
        .write = mywriter,
};
#endif


/* le driver lui même */
static struct platform_device *lirc_sunxi_dev;
static struct sunxi_ir* ir;

/* buffer de lirc */
static struct lirc_buffer rbuf;
static struct platform_driver sunxi_ir_driver = {
		.driver = {
		.name = LIRC_DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

static const struct file_operations lirc_fops = {
        .owner                = THIS_MODULE,
        .unlocked_ioctl        = lirc_ioctl,
        .read                = lirc_dev_fop_read, // this and the rest is default
        .write                = lirc_dev_fop_write,
        .poll                = lirc_dev_fop_poll,
        .open                = lirc_dev_fop_open,
        .release        = lirc_dev_fop_close,
        .llseek                = no_llseek,
};

static struct lirc_driver driver = {
        .name                = LIRC_DRIVER_NAME,
        .minor                = -1,           // assing automatically
        .code_length        = 1,
        .sample_rate        = 0,
        .data                = NULL,
        .add_to_buf        = NULL,
        .rbuf                = &rbuf,
        .set_use_inc        = set_use_inc,
        .set_use_dec        = set_use_dec,
        .fops                = &lirc_fops,
        .dev                = NULL,
        .owner                = THIS_MODULE,
};

static inline int ir_packet_handler(struct sunxi_ir *ir,struct lirc_buffer *buffer){

unsigned char pulse;
unsigned int index = 0;

	while (ir->rawbuf.dcnt)//tant que le buffer n'est pas vide
	{

		pulse = ir->rawbuf.buf[index].pulse; //récupération de la polarité
		index ++;
		ir->rawbuf.dcnt--;
		if (pulse == (ir->rawbuf.buf[index].pulse))
		{
			/*si le pulse est de la méme polarité que le précédent
			 * alors on l'additionne au suivant
			 */
			ir->rawbuf.buf[index].duration=ir->rawbuf.buf[index-1].duration+ir->rawbuf.buf[index].duration;
		}
		else
		{
			if (ir->rawbuf.buf[index].duration>PULSE_MASK)
			{
				printk(KERN_INFO "pulse are %d and is too long",ir->rawbuf.buf[index].duration);
				return (-1);
			}
			if (pulse==1)
				ir->rawbuf.buf[index].duration |= PULSE_BIT; // on met le pulse à 1
			else
				ir->rawbuf.buf[index].duration &= PULSE_MASK; //on met le pulse à 0
#ifdef LIRC
			lirc_buffer_write(buffer,(unsigned char*)&ir->rawbuf.buf[index].duration);
#endif
		}
	}

return 0;
}

/*
 * routine de traitement de l'interruption
 */
static irqreturn_t sunxi_ir_irq(int irqno, void *dev_id)
{
	unsigned long status;
	unsigned char dt;
	unsigned int cnt, rc;
	struct ir_raw_pulse rawir;

	spin_lock(&ir->ir_lock); //vérrouille l'execution
	status = readl(ir->base + SUNXI_IR_RXSTA_REG);

	/* clean all pending statuses */
	writel(status | REG_RXSTA_CLEARALL, ir->base + SUNXI_IR_RXSTA_REG);

	if (status & REG_RXINT_RAI_EN) {
		/* How many messages in fifo */
		rc  = REG_RXSTA_GET_AC(status);
		/* Sanity check */
		rc = rc > SUNXI_IR_FIFO_SIZE ? SUNXI_IR_FIFO_SIZE : rc;
		/* If we have data */
		for (cnt = 0; cnt < rc; cnt++) {
			/* for each bit in fifo */
			dt = readb(ir->base + SUNXI_IR_RXFIFO_REG);
			rawir.pulse = (dt & 0x80) != 0;
			rawir.duration = ((dt & 0x7f) + 1) * SUNXI_IR_SAMPLE;
			ir_raw_event_store(rawir,&ir->rawbuf);
		}
	}

	if (status & REG_RXINT_ROI_EN) {
		/* FIFO Overflow */
		ir_reset_rawbuffer(ir);
	} else if (status & REG_RXINT_RPEI_EN) {
		/* packet end */
		ir_packet_handler(ir,&rbuf);
		ir_reset_rawbuffer(ir);
		wake_up_interruptible(&rbuf.wait_poll); /*TODO ou est fait l'inverse */

	}

	spin_unlock(&ir->ir_lock); //libére l'execution

	return IRQ_HANDLED;
}
/* control de lirc */
#ifdef LIRC
static long lirc_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{

        switch (cmd) {
        case LIRC_GET_SEND_MODE:
            return -ENOIOCTLCMD;
            break;

            /* driver cannot send */
        case LIRC_SET_SEND_MODE:
            return -ENOSYS;
            break;

        case LIRC_GET_LENGTH:
            return -ENOSYS;
            break;

        case LIRC_SET_SEND_DUTY_CYCLE:
            return -ENOSYS;
             break;

        case LIRC_SET_SEND_CARRIER:
            return -ENOSYS;
            break;

        default:
            return lirc_dev_fop_ioctl(filep, cmd, arg);
        }
        return 0;
}
#endif
/*set_use_inc will be called after device is opened*/
static int set_use_inc(void* data) {
	return 0;
}
/* set_use_dec will be called after device is closed */
static void set_use_dec(void* data) {

}

/* end of lirc device/driver stuff */

static int __init sunxi_ir_probe(void) {
#ifdef DEBUG
	/* create a directory by the name dell in /sys/kernel/debugfs */
	    dirret = debugfs_create_dir("sunxi_lirc_new", NULL);

	    /* create a file in the above directory
	    This requires read and write file operations */
	    fileret = debugfs_create_file("log", 0644, dirret, &filevalue, &fops_debug);
#endif
	int ret = 0;
	unsigned long tmp = 0;
	/* Init read buffer. */
#ifdef LIRC
	ret = lirc_buffer_init(&rbuf, sizeof(int), RBUF_LEN);
	if (ret < 0)
		return -ENOMEM;

	ret = platform_driver_register(&sunxi_ir_driver);
	if (ret) {
		printk(KERN_ERR LIRC_DRIVER_NAME
				": lirc register returned %d\n", ret);
		goto exit_buffer_free;
	}

	lirc_sunxi_dev = platform_device_alloc(LIRC_DRIVER_NAME, 0);
	if (!lirc_sunxi_dev) {
		ret = -ENOMEM;
		goto exit_driver_unregister;
	}

	ret = platform_device_add(lirc_sunxi_dev);
	if (ret) {
		platform_device_put(lirc_sunxi_dev);
		goto exit_driver_unregister;
	}
#endif
	/* initialisation du vérroulliage d'execution
	 * ce n'était pas présent avant */
		spin_lock_init(&ir->ir_lock);
	/* IRQ dans le driver original les irq sont demander avec les clock*/
	if (request_irq(IR_IRQNO, sunxi_ir_irq, 0, "RemoteIR", (void*) 0)) {
		ret = -EBUSY;
		goto exit_device_unregister;
	}

	ir->irq = gpio_request_ex("ir_para", "ir0_rx");
	if (0 == ir->irq)
		printk("try to request ir_para gpio failed\n");
	ir_reset_rawbuffer(ir);

	ir->base = (void *)IR_BASE; /* on la rempli en dynamique
	 puisque l'on peu pas l'avoir autrement car on est pas dans le mainline */

	/* Clock */
	ir->apb_clk = clk_get(NULL, "apb_ir0");
	if (!ir->apb_clk) {
		printk("try to get apb_ir0 clock failed\n");
		goto exit_clkdisable_apb_clk;/* TODO géré les erreurs
		a priori il faut mettres les sortie dans l'orde inverse pour defaire ce qui a été fait */
	}

	ir->clk = clk_get(NULL, "ir0");
	if (!ir->clk) {
		printk("try to get ir0 clock failed\n");
		goto exit_clkdisable_clk;
	}
	dprintk(
			"trying to set clock via SYS_CLK_CFG_EN, when no error follows -> succeeded\n");
	if (clk_set_rate(ir->clk, (unsigned long) SUNXI_IR_BASE_CLK)) // 125kHz
		printk("set ir0 clock freq failed\n");

	if (clk_enable(ir->apb_clk))
		printk("try to enable apb_ir_clk failed\n");

	if (clk_enable(ir->clk))
		printk("try to enable apb_ir_clk failed\n");

	/* Enable CIR Mode */
	writel(REG_CTL_MD, ir->base + SUNXI_IR_CTL_REG);

	/* Set noise threshold and idle threshold */
	writel(REG_CIR_NTHR(SUNXI_IR_RXNOISE) | REG_CIR_ITHR(SUNXI_IR_RXIDLE),
			ir->base + SUNXI_IR_CIR_REG);

	/* Invert Input Signal */
	writel(REG_RXCTL_RPPI, ir->base + SUNXI_IR_RXCTL_REG);

	/* Clear All Rx Interrupt Status */
	writel(REG_RXSTA_CLEARALL, ir->base + SUNXI_IR_RXSTA_REG);

	/*
	 * Enable IRQ on overflow, packet end, FIFO available with trigger
	 * level
	 */
	writel(REG_RXINT_ROI_EN | REG_RXINT_RPEI_EN |
	REG_RXINT_RAI_EN | REG_RXINT_RAL(TRIGGER_LEVEL - 1),
			ir->base + SUNXI_IR_RXINT_REG);

	/* Enable IR Module */
	tmp = readl(ir->base + SUNXI_IR_CTL_REG);
	writel(tmp | REG_CTL_GEN | REG_CTL_RXEN, ir->base + SUNXI_IR_CTL_REG);

	printk(KERN_INFO "initialized sunXi IR driver\n");

	printk("IR Initial OK\n");
#ifdef LIRC
	// 'driver' is the lirc driver
	driver.features = LIRC_CAN_SEND_PULSE | LIRC_CAN_REC_MODE2;

	driver.dev = &lirc_sunxi_dev->dev; // link THIS platform device to lirc driver
	driver.minor = lirc_register_driver(&driver);

	if (driver.minor < 0) {
		printk(KERN_ERR LIRC_DRIVER_NAME
				": device registration failed with %d\n", ret);

		ret = -EIO;
		goto exit_free_irq;
	}

	printk(KERN_INFO LIRC_DRIVER_NAME ": driver registered!\n");
#endif
	// normalment on sort ici
	return 0; //sortie normale
	exit_free_irq:
	free_irq(IR_IRQNO, (void*) 0);
	exit_device_unregister:
	platform_device_unregister(lirc_sunxi_dev);
	exit_driver_unregister:
	platform_driver_unregister(&sunxi_ir_driver);
	exit_buffer_free:
	lirc_buffer_free(&rbuf);
	exit_clkdisable_clk:
	clk_put(ir->clk);
	exit_clkdisable_apb_clk:
	clk_put(ir->apb_clk);

	return ret;
}

static int __exit sunxi_ir_remove(void) {
	unsigned long flags;
	/* libération de l'irq*/
	free_irq(IR_IRQNO, (void*) 0);
	gpio_release(ir->irq, 2);
	/* déconfiguration de l'horloge */
	clk_put(ir->clk);
	clk_put(ir->apb_clk);

	spin_lock_irqsave(&ir->ir_lock, flags);
	/* disable IR IRQ */
	writel(0, ir->base + SUNXI_IR_RXINT_REG);
	/* clear All Rx Interrupt Status */
	writel(REG_RXSTA_CLEARALL, ir->base + SUNXI_IR_RXSTA_REG);
	/* disable IR */
	writel(0, ir->base + SUNXI_IR_CTL_REG);
	spin_unlock_irqrestore(&ir->ir_lock, flags);
#ifdef LIRC
	platform_device_unregister(lirc_sunxi_dev);

	platform_driver_unregister(&sunxi_ir_driver);

	lirc_buffer_free(&rbuf);
	lirc_unregister_driver(driver.minor);
	printk(KERN_INFO LIRC_DRIVER_NAME ": cleaned up module\n");
#endif
#ifdef DEBUG
	debugfs_remove_recursive(dirret);
#endif
	return 0;
}


//module_platform_driver(sunxi_ir_driver); //remplace les init et exit pratique! marche pas
module_init(sunxi_ir_probe);
module_exit(sunxi_ir_remove);

MODULE_DESCRIPTION("Remote IR driver");
MODULE_AUTHOR("Matthias Hoelling / Damien Pageot ");
MODULE_LICENSE("GPL");

module_param(debug, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Enable debugging messages");

