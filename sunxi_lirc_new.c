/*
 *
 * sunxi_lirc.c
 *
 * sunxi_lirc - Device driver that uses Allwinner A1X or A20 IR module in CIR mode
 *              for LIRC. Tested on a Cubietruck with Allwinner A20
 *              a lot of code from the sunxi-ir module,
 *              so I would like say thanks to the authors.
 *        		Difference to sunxi-ir is that no verification of IR code
 *        		against NEC protocol is made, whatsoever
 *        		but just passed on to lirc buffer to let lirc do any decoding
 *
 * Copyright (C) 2014 Matthias Hoelling <mhoel....@gmail.nospam.com>,
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/spinlock.h>
#include <asm/irq.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <mach/clock.h>
#include <media/lirc.h>
#include <media/lirc_dev.h>


#include <mach/irqs.h>
#include <mach/system.h>
#include <mach/hardware.h>
#include <plat/sys_config.h>

#include <linux/clk.h>
#include <linux/kfifo.h> //nouveau pour remplacer raw buf
#include "sunxi_lirc_new.h"


#define LIRC

static struct platform_device *lirc_sunxi_dev;

static struct clk *apb_ir_clk;
static struct clk *ir_clk;
static u32 ir_gpio_hdle;





#define dprintk(fmt, args...)               \
do {                                        \
if (debug)                                  \
printk(KERN_DEBUG LIRC_DRIVER_NAME ": "     \
fmt, ## args);                              \
} while (0)



static struct kfifo rawfifo;

static struct lirc_buffer rbuf;

DEFINE_SPINLOCK(sunxi_lirc_spinlock);



static int debug=0;



static void ir_clk_cfg(void)
{

	unsigned long rate = 3000000; /* 6 MHz */



	apb_ir_clk = clk_get(NULL, "apb_ir0");
	if (!apb_ir_clk) {
		printk("try to get apb_ir0 clock failed\n");
		return;
	}

	ir_clk = clk_get(NULL, "ir0");
	if (!ir_clk) {
		printk("try to get ir0 clock failed\n");
		return;
	}
	dprintk("trying to set clock via SYS_CLK_CFG_EN, when no error follows -> succeeded\n");
	if (clk_set_rate(ir_clk, rate))
		printk("set ir0 clock freq to 6M failed\n");

	if (clk_enable(apb_ir_clk))
		printk("try to enable apb_ir_clk failed\n");

	if (clk_enable(ir_clk))
		printk("try to enable apb_ir_clk failed\n");

	return;
}

static void ir_clk_uncfg(void)
{

	clk_put(apb_ir_clk);
	clk_put(ir_clk);
return;
}
static void ir_sys_cfg(void)
{

	ir_gpio_hdle = gpio_request_ex("ir_para", "ir0_rx");
	if (0 == ir_gpio_hdle)
		printk("try to request ir_para gpio failed\n");
	ir_clk_cfg();

	return;
}

static void ir_sys_uncfg(void)
{

	gpio_release(ir_gpio_hdle, 2);

	ir_clk_uncfg();

	return;
}

static void ir_reg_cfg(void)
{
    unsigned long tmp;
	/* Enable CIR Mode */
    writel(REG_CTL_MD, IR_BASE + SUNXI_IR_CTL_REG);
    /* Set noise threshold and idle threshold */
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
	return;
}



static void ir_setup(void)
{
	dprintk("ir_setup: ir setup start!!\n");
    kfifo_reset(&rawfifo);
	ir_sys_cfg();
	ir_reg_cfg();

	dprintk("ir_setup: ir setup end!!\n");

	return;
}

static inline unsigned char ir_get_data(void)
{
	return (unsigned char)(readl(IR_BASE + SUNXI_IR_RXFIFO_REG));
}

static inline unsigned long ir_get_intsta(void)
{
	return readl(IR_BASE + SUNXI_IR_RXSTA_REG);
}

static inline void ir_clr_intsta(unsigned long bitmap)
{
	unsigned long tmp = readl(IR_BASE + SUNXI_IR_RXSTA_REG);

	tmp &= ~0xff;
	tmp |= bitmap&0xff;
	writel(tmp, IR_BASE + SUNXI_IR_RXSTA_REG);
}

//new handler
static void ir_packet_handler(void)
{
    unsigned char pulse,dt,next_pulse,dt_next;
    int duration = 0;


    while (!kfifo_is_empty(&rawfifo))
    {
        if (kfifo_out(&rawfifo,&dt,sizeof(dt))!=1) //kfifi_out lit la valeur et l'enléve de la fifo
            break;


        pulse = (dt & 0x80) != 0; // donne la polarité du pulse

        if (kfifo_out_peek(&rawfifo,&dt_next,sizeof(dt))!=1) //kfifi_out_peek lit la valeur et l'enléve PAS de la fifo
            break;

        next_pulse = (dt_next & 0x80) != 0;

        if (pulse == next_pulse)
        {
            /*si le pulse est de la méme polarité que le suivant
            * alors on l'additionne au suivant
            */
            duration += ((dt_next & 0x7f) + 1) * SUNXI_IR_SAMPLE; // et sa durée
        }
        else
        {
            if (duration == 0) //cas improblable ou un pulse est plus court que 127 echantillons
                duration = ((dt & 0x7f) + 1) * SUNXI_IR_SAMPLE; // et sa durée

            if (duration>PULSE_MASK)
            {
                printk(KERN_INFO "pulse are %d and is too long",duration);
                return;
            }
            if (pulse==1)
                duration |= PULSE_BIT; // on met le pulse à 1
            else
                duration &= PULSE_MASK; //on met le pulse à 0
#ifdef LIRC
            lirc_buffer_write(&rbuf,(unsigned char*)duration);
#endif
        }
    }
    return ;
}


static irqreturn_t ir_irq_service(int irqno, void *dev_id)
{
    unsigned long status;
    unsigned char dt,overflow_error = 0;
    unsigned int cnt, rc;



    status = readl(IR_BASE + SUNXI_IR_RXSTA_REG);
    /* clean all pending statuses */
    writel(status | REG_RXSTA_CLEARALL, IR_BASE + SUNXI_IR_RXSTA_REG);
    if (status & REG_RXINT_RAI_EN)
    {
        /* How many messages in fifo */
        rc = REG_RXSTA_GET_AC(status);
        /* Sanity check */
        rc = rc > SUNXI_IR_FIFO_SIZE ? SUNXI_IR_FIFO_SIZE : rc;
        /* If we have data */
        for (cnt = 0; cnt < rc; cnt++)
        {
            /* for each bit in fifo */
            dt = readb(IR_BASE + SUNXI_IR_RXFIFO_REG);

            kfifo_in(&rawfifo,&dt,sizeof(dt));

        }
    }
    if (status & REG_RXINT_ROI_EN)
    {
        /* FIFO Overflow */

        kfifo_reset(&rawfifo);

        dprintk("hardware fifo overflow trame ir perdue");
        overflow_error = 1;
    }
    else if (status & REG_RXINT_RPEI_EN)
    {
        /* packet end */
        if (!overflow_error)
            ir_packet_handler(); /* TODO à modifier pas dbesoin de passer une donnée globale...*/
        else
            overflow_error = 0;// on a atteint le fin de la trame perdu on peu tanter de récupéré la suivante

        kfifo_reset(&rawfifo);
        wake_up_interruptible(&rbuf.wait_poll);
    }

    return IRQ_HANDLED;
}



/* interpret lirc commands */
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


static int set_use_inc(void* data) {
	return 0;
}

static void set_use_dec(void* data) {

}

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

/* end of lirc device/driver stuff */

/* now comes THIS driver, above is lirc */
static struct platform_driver lirc_sunxi_driver = {
        .driver = {
                .name   = LIRC_DRIVER_NAME,
                .owner  = THIS_MODULE,
        },
};





static int __init ir_init(void)
{
	int result;
    /* Init read buffer. */
    result = lirc_buffer_init(&rbuf, sizeof(int), RBUF_LEN);
    if (result < 0)
            return -ENOMEM;

    /*init rawfifo */

    result = kfifo_alloc(&rawfifo,IR_RAW_BUF_SIZE,GFP_KERNEL);
    if (result < 0)
        return -ENOMEM;

    result = platform_driver_register(&lirc_sunxi_driver);
    if (result) {
            printk(KERN_ERR LIRC_DRIVER_NAME
                   ": lirc register returned %d\n", result);
            goto exit_buffer_free;
    }

    lirc_sunxi_dev = platform_device_alloc(LIRC_DRIVER_NAME, 0);
    if (!lirc_sunxi_dev) {
            result = -ENOMEM;
            goto exit_driver_unregister;
    }

    result = platform_device_add(lirc_sunxi_dev);
    if (result) {
    	platform_device_put(lirc_sunxi_dev);
    	goto exit_driver_unregister;
    }

	if (request_irq(IR_IRQNO, ir_irq_service, 0, "RemoteIR",
			(void*) 0)) {
		result = -EBUSY;
		goto exit_device_unregister;
	}

	ir_setup();

	printk("IR Initial OK\n");



    // 'driver' is the lirc driver
        driver.features = LIRC_CAN_SEND_PULSE | LIRC_CAN_REC_MODE2;

        driver.dev = &lirc_sunxi_dev->dev;  // link THIS platform device to lirc driver
        driver.minor = lirc_register_driver(&driver);

        if (driver.minor < 0) {
                printk(KERN_ERR LIRC_DRIVER_NAME
                       ": device registration failed with %d\n", result);

                result = -EIO;
                goto exit_free_irq;
        }

        printk(KERN_INFO LIRC_DRIVER_NAME ": driver registered!\n");

    return 0;


exit_free_irq:
	free_irq(IR_IRQNO, (void*) 0);

exit_device_unregister:
    platform_device_unregister(lirc_sunxi_dev);

exit_driver_unregister:
    platform_driver_unregister(&lirc_sunxi_driver);

exit_buffer_free:
    lirc_buffer_free(&rbuf);

	return result;
}

static void __exit ir_exit(void)
{

	free_irq(IR_IRQNO, (void*) 0);
	kfifo_free(&rawfifo);
	ir_sys_uncfg();
    platform_device_unregister(lirc_sunxi_dev);

    platform_driver_unregister(&lirc_sunxi_driver);

    lirc_buffer_free(&rbuf);
    lirc_unregister_driver(driver.minor);
    printk(KERN_INFO LIRC_DRIVER_NAME ": cleaned up module\n");

}

module_init(ir_init);
module_exit(ir_exit);

MODULE_DESCRIPTION("Remote IR driver");
MODULE_AUTHOR("Damien Pageot");
MODULE_LICENSE("GPL");

module_param(debug, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Enable debugging messages");

