/*
 *  linux/drivers/serial/9865.h
 *
 *  Based on drivers/serial/8250.c by Russell King.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This code is modified to support moschip 9865 series serial devices
 */

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,15)
#include <linux/config.h>
#endif


struct old_serial_port {
	unsigned int 	uart;
	unsigned int 	baud_base;
	unsigned int	 port;
	unsigned int	 irq;
	unsigned int 	flags;
	unsigned char 	hub6;
	unsigned char 	io_type;
	unsigned char 	*iomem_base;
	unsigned short 	iomem_reg_shift;
};

struct serial9865_config {
	unsigned short	fifo_size;
	unsigned short	tx_loadsz;
	unsigned char		fcr;
	unsigned int		flags;
};

#define DIV 1

#define UART_CAP_FIFO	(1 << 8)	/* UART has FIFO */
#define UART_CAP_EFR	(1 << 9)	/* UART has EFR */
#define UART_CAP_SLEEP	(1 << 10)	/* UART has IER sleep */
#define UART_CAP_AFE	(1 << 11)	/* MCR-based hw flow control */
#define UART_CAP_UUE	(1 << 12)	/* UART needs IER bit 6 set (Xscale) */

#define REG_TX_DMA_START_ADDRESS_LOW	((0x80)/DIV)
#define REG_TX_DMA_START_ADDRESS_HIGH	((0x84)/DIV)
#define REG_TX_DMA_LENGTH				((0x88)/DIV)
#define REG_TX_DMA_START				((0x8C)/DIV)
#define REG_TX_DMA_STOP					((0x90)/DIV)
#define REG_TX_DMA_STOP_DONE			((0x94)/DIV)
#define REG_TX_BYTES_TRANSFERRED		((0x98)/DIV)
#define REG_TX_DMA_BUSY					((0x9C)/DIV)
#define REG_TX_DMA_DONE					((0xA0)/DIV)
#define REG_TX_RDY_1					((0xA4)/DIV)

#define REG_RX_DMA_START_ADDRESS_LOW	((0x100)/DIV)
#define REG_RX_DMA_START_ADDRESS_HIGH	((0x104)/DIV)
#define REG_RX_DMA_LENGTH				((0x108)/DIV)
#define REG_RX_DMA_START				((0x10C)/DIV)
#define REG_RX_DMA_STOP					((0x110)/DIV)
#define REG_RX_TRIG_LVL					((0x114)/DIV)
#define REG_RX_DMA_STOP_DONE			((0x118)/DIV)
#define REG_RX_BYTES_NEED_TO_RECV		((0x11C)/DIV)
#define REG_RX_DMA_BUSY					((0x120)/DIV)
#define REG_RX_DMA_DONE					((0x124)/DIV)
#define REG_RX_RDY_1					((0x128)/DIV)
#define REG_RX_MEM_4K_LMT				((0x12C)/DIV)


#define REG_GLBL_ISR			((0x3A0)/DIV)
#define REG_GLBL_ICLR			((0x3A4)/DIV)
#define REG_GLBL_IER			((0x3A8)/DIV)

#define TX_DMA_START_BIT		1<<0
#define TX_DMA_STOP_BIT			1<<0
#define TX_DMA_STOP_DONE_BIT 	1<<0
#define TX_DMA_DONE				1<<0
#define TX_DMA_BUSY				1<<0
#define TX_DMA_RDY				1<<0

#define RX_DMA_START_BIT		1<<0
#define RX_DMA_STOP_BIT			1<<0
#define RX_DMA_STOP_DONE_BIT 	1<<0
#define RX_DMA_DONE				1<<0
#define RX_DMA_BUSY				1<<0
#define RX_DMA_RDY				1<<0

#define SPINTR_DMA					0x01
#define SPINTR_TXDMA_ABORT_DONE		0x02
#define SPINTR_TXDMA_STOP_DONE		0x04
#define SPINTR_TXDMA_DONE 			0x08
#define SPINTR_RXDMA_ABORT_DONE 	0x10
#define SPINTR_RXDMA_STOP_DONE		0x20
#define SPINTR_RXDMA_DONE			0x40
#define SPINTR_RXDMA_PARTDONE		0x80

#define SERIAL_450MODE		0x5470
#define SERIAL_550MODE		0x5471
#define SERIAL_550AMODE		0x5472
#define SERIAL_650MODE		0x5473
#define SERIAL_750MODE		0x5474
#define SERIAL_850MODE		0x5475
#define SERIAL_950MODE		0x5476

// Default xon/xoff characters.
#define SERIAL_DEF_XON		0x11
#define SERIAL_DEF_XOFF		0x13

// UART mode
#define MCS9865_RS232_MODE					0
#define MCS9865_RS422_MODE					1
#define MCS9865_RS485_HALF_DUPLEX			2
#define MCS9865_RS485_HALF_DUPLEX_ECHO		4
#define MCS9865_RS485_FULL_DUPLEX			5
#define MCS9865_DTR_DSR_HW_FLOWCONTROL 		6
#define MCS9865_XON_XOFF_HW_FLOWCONTROL		7
#define MCS9865_RTS_CTS_HW_FLOWCONTROL		8
#define MCS9865_IRDA_MODE					9

#define PORT_ENHANC			14

//CommSet Registers
//Common Registers Set (memory mapped)
#define SER_DCR_DIN_REG			((0x200)/DIV)
#define SER_VEN_REG				((0x204)/DIV)
#define SP_SIGNAL_VALID_REG		((0x208)/DIV)
#define SP_IO_CONTROL_ENABLE_REG	((0x20C)/DIV)
#define SP_OUTPUT_REG_VALID		((0x210)/DIV)
#define SP_TX_TRIGGER_LVL		((0x21C)/DIV)
#define SP_CLK_SELECT_REG		((0x214)/DIV)
#define PP_DIV_REG				((0x250)/DIV)
#define PP_RX_TRIG_LEVEL		((0x254)/DIV)
#define PP_TX_TRIG_LEVEL		((0x258)/DIV)
#define PP_PERI_HOST_HIGH_REG	((0x25C)/DIV)
#define SER_TFL_REG				((0x220)/DIV) 
#define SER_RFL_REG				((0x224)/DIV) 
#define SER_SOFT_RESET_REG		((0x238)/DIV) 

#define COM_DMA_MODE_EN			0x10000000
#define COM_550EX_MODE_EN		0x00001000


#define SER_TFL_REG				((0x220)/DIV)
#define SER_RFL_REG				((0x224)/DIV)
#define SER_SOFT_RESET_REG		((0x238)/DIV)


#define PCI_DEVICE_ID_NETMOS_9865 0x9865
#define PCI_SUBVEN_ID_9865	  0x1000
#define PCI_SUBDEV_ID_9865	  0xa000	

#define BAR_IO		0x001
#define BAR_MEM 	0x000
#define BAR_FMT  	0x001

#define DMA_RX_SZ 		256
#define DMA_RX_BUFFER_SZ 	131072
#define DMA_TX_BUFFER_SZ 	4096

#if defined(__i386__) && (defined(CONFIG_M386) || defined(CONFIG_M486))
#define _INLINE_ inline
#else
#define _INLINE_
#endif

#define DEFAULT9865_BAUD 115200

static unsigned int serial9865_get_mctrl(struct uart_port *);
