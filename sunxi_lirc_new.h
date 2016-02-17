/*
 * sunxi_lirc_new.h
 *
 *  Created on: 22 janv. 2016
 *      Author: k005425
 */
#ifndef SUNXI_LIRC_NEW_H_
#define SUNXI_LIRC_NEW_H_
#define LIRC
#define LIRC_DRIVER_NAME "sunxi_lirc_new"
#define IR_RAW_BUF_SIZE 512
#define RBUF_LEN 256 // longueur du buffer raw c'est ici qu'es vidé la fifo
/* le symbole LIRC permet de compilé le lien avec lirc_dev ce qui permet de débugger séparément*/
//#define LIRC
/* Registers */
#if 1
/* base ir register */
#define IR_BASE		(0xf1c21800) // à voir ou on l'utilise mais il faut c'est sur!
/* IRQ */
#define IR_IRQNO	(SW_INT_IRQNO_IR0)
/* IR Control */
#define SUNXI_IR_CTL_REG      0x00
/* Global Enable */
#define REG_CTL_GEN			BIT(0)
/* RX block enable */
#define REG_CTL_RXEN			BIT(1)
/* CIR mode */
#define REG_CTL_MD			(BIT(4) | BIT(5))

/* Rx Config */
#define SUNXI_IR_RXCTL_REG    0x10
/* Pulse Polarity Invert flag */
#define REG_RXCTL_RPPI			BIT(2)

/* Rx Data */
#define SUNXI_IR_RXFIFO_REG   0x20

/* Rx Interrupt Enable */
#define SUNXI_IR_RXINT_REG    0x2C
/* Rx FIFO Overflow */
#define REG_RXINT_ROI_EN		BIT(0)
/* Rx Packet End */
#define REG_RXINT_RPEI_EN		BIT(1)
/* Rx FIFO Data Available */
#define REG_RXINT_RAI_EN		BIT(4)

/* Rx FIFO available byte level */
#define REG_RXINT_RAL(val)    (((val) << 8) & (GENMASK(11, 8)))

/* Rx Interrupt Status */
#define SUNXI_IR_RXSTA_REG    0x30
/* RX FIFO Get Available Counter */
#define REG_RXSTA_GET_AC(val) (((val) >> 8) & (GENMASK(5, 0)))
/* Clear all interrupt status value */
#define REG_RXSTA_CLEARALL    0xff

/* IR Sample Config */
#define SUNXI_IR_CIR_REG      0x34
/* CIR_REG register noise threshold */
#define REG_CIR_NTHR(val)    (((val) << 2) & (GENMASK(7, 2)))
/* CIR_REG register idle threshold */
#define REG_CIR_ITHR(val)    (((val) << 8) & (GENMASK(15, 8)))

/* Hardware supported fifo size */
#define SUNXI_IR_FIFO_SIZE    16
/* How many messages in FIFO trigger IRQ */
#define TRIGGER_LEVEL         8
/* Required frequency for IR0 or IR1 clock in CIR mode */
#define SUNXI_IR_BASE_CLK     8000000
/* Frequency after IR internal divider  */
#define SUNXI_IR_CLK          (SUNXI_IR_BASE_CLK / 64)
/* Sample period in ns */
#define SUNXI_IR_SAMPLE       (1000000000ul / SUNXI_IR_CLK)
/* Noise threshold in samples  */
#define SUNXI_IR_RXNOISE      1
/* Idle Threshold in samples */
#define SUNXI_IR_RXIDLE       20
/* Time after which device stops sending data in ms */
#define SUNXI_IR_TIMEOUT      120
#define dprintk(fmt, args...)                                        \
do {                                                        \
if (debug)                                        \
printk(KERN_DEBUG LIRC_DRIVER_NAME ": "        \
fmt, ## args);                        \
} while (0)

#define GENMASK(h, l) \
       (((~0UL) << (l)) & (~0UL >> (BITS_PER_LONG - 1 - (h))))
#endif
/* déclaration des fonction d'initailisation */
//static int sunxi_ir_probe(struct platform_device * pdev);
//static int sunxi_ir_remove(struct platform_device * pdev);
///* les données du driver */
//
//struct ir_raw_pulse {
// 	unsigned char pulse;
// 	unsigned int duration;
// };
//struct ir_raw_buffer {
// 	unsigned int dcnt;	/*Packet Count*/
// 	struct ir_raw_pulse buf[RBUF_LEN];
// };
//struct sunxi_ir {
//	spinlock_t      ir_lock;
//	void __iomem    *base;//cela doit etre l'adresse de base du driver
//	u32             irq;//vérifier dans quel lib c'est déclaré
//	struct clk      *clk;
//	struct clk      *apb_clk;
//	struct ir_raw_buffer rawbuf;
//	};
//static inline int ir_packet_handler(struct sunxi_ir *ir,struct lirc_buffer *buffer);
//#ifdef LIRC
//static long lirc_ioctl(struct file *filep, unsigned int cmd, unsigned long arg);
//#endif
//static int set_use_inc(void* data);
//static void set_use_dec(void* data);
//
//
//
//static inline void ir_reset_rawbuffer(struct sunxi_ir *ir){
//	ir->rawbuf.dcnt = 0;
//}
//static inline int ir_rawbuffer_full(struct sunxi_ir *ir){
//	return (ir->rawbuf.dcnt>=RBUF_LEN);
//}
//static inline int ir_rawbuffer_empty(struct sunxi_ir *ir)
//{
//	return (ir->rawbuf.dcnt == 0);
//}
//
//static inline void ir_raw_event_store(struct ir_raw_pulse pulse,
//		struct ir_raw_buffer *buf) {
//	if (buf->dcnt < RBUF_LEN) {
//		buf->buf[buf->dcnt] = pulse;
//		buf->dcnt++;
//	} else
//		printk("ir_write_rawbuffer: IR Rx buffer full\n");
//}


#endif /* SUNXI_LIRC_NEW_H_ */
