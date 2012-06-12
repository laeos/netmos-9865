/*
 *  linux/drivers/serial/9865-isa.h
 *
 *  Driver for PCI based ISA 8250/16550-type serial ports
 *
 *  Based on drivers/serial/8250.h by Russell King
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */
#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18)
#include <linux/config.h>
#endif

#define MAX_ISA_PORTS 32
#define STX_NR_PORTS 0x000F

#define UART_CAP_FIFO	(1 << 8)	/* UART has FIFO */
#define UART_CAP_EFR	(1 << 9)	/* UART has EFR */
#define UART_CAP_SLEEP	(1 << 10)	/* UART has IER sleep */
#define UART_CAP_AFE	(1 << 11)	/* MCR-based hw flow control */
#define UART_CAP_UUE	(1 << 12)	/* UART needs IER bit 6 set (Xscale) */

#define BAR_IO 	0x001
#define BAR_MEM 0x000
#define BAR_FMT 0x001

#define DEFAULT9865_BAUD 115200

#define PCI_DEVICE_ID_NETMOS_9865 0x9865
#define PCI_SUBDEV_ID_9865      0xa000

#define PCI_SUBVEN_ID_9865_4S      0x3004
#define PCI_SUBVEN_ID_9865_3S      0x3003
#define PCI_SUBVEN_ID_9865_2S      0x3002
#define PCI_SUBVEN_ID_9865_1S      0x3001
#define PCI_SUBVEN_ID_9865_2P      0x3020
#define PCI_SUBVEN_ID_9865_1S1P      0x3011
#define PCI_SUBVEN_ID_9865_2S1P      0x3012
