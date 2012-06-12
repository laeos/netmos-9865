/*
 *  linux/drivers/serial/9865-isa.c
 *
 *  Probe module for 9865  PCI based ISA serial ports.
 *
 *  Based on drivers/char/8250_pci.c, by Russell King.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/tty.h>
#include <linux/serial_core.h>
#include <linux/8250_pci.h>
#include <linux/bitops.h>
#include <linux/serial_8250.h>

#include <asm/byteorder.h>
#include <asm/io.h>

#include "mcs9865-isa.h"

#undef SERIAL_DEBUG_PCI

/*
 * Definitions for PCI support.
 */
#define FL_BASE_MASK	0x0007
#define FL_BASE0		0x0000
#define FL_BASE1		0x0001
#define FL_BASE2		0x0002
#define FL_BASE3		0x0003
#define FL_BASE4		0x0004
#define FL_BASE5		0x0005
#define FL_GET_BASE(x)	(x & FL_BASE_MASK)

/* Use successive BARs (PCI base address registers),
   else use offset into some specified BAR */
#define FL_BASE_BARS		0x0008

#define PCI_NUM_BAR_RESOURCES	6

struct isa_private {
		int nr_ports;
		int line[MAX_ISA_PORTS];	
}isa_priv;

#if 0
#define DEBUG(fmt...)   printk(fmt)
#else
#define DEBUG(fmt...)   do { } while (0)
#endif

/*
 * Probe one serial board.  Unfortunately, there is no rhyme nor reason
 * to the arrangement of serial ports on a PCI card.
 */
static int __devinit
pci_isa_serial_init_one(struct pci_dev *dev, const struct pci_device_id *ent)
{
	int rc,retval,i,nr_ports;
	rc = pci_enable_device(dev);
	if (rc)
		return rc;

        //To verify wether it is a serial communication hardware
        if ((dev->class >> 8) !=  PCI_CLASS_COMMUNICATION_OTHER){
                DEBUG("Not a communication hardware\n");
                retval = -ENODEV;
                goto disable;
        }
	
	//To verify wether it is a MCS9865 ISA type BARs
        if(((pci_resource_flags(dev,FL_BASE0) & BAR_FMT) ^ BAR_IO) || ((pci_resource_flags(dev,FL_BASE1) & BAR_FMT) ^ BAR_IO) ||((pci_resource_flags(dev,FL_BASE2) & BAR_FMT) ^ BAR_IO) || ((pci_resource_flags(dev,FL_BASE3) & BAR_FMT) ^ BAR_IO) || ((pci_resource_flags(dev,FL_BASE5) & BAR_FMT) ^ BAR_MEM)) {
                DEBUG("Not a MCS9865 ISA type device\n");
                retval = -ENOMEM;
                goto disable;
        }

	pci_set_master(dev);
	nr_ports=dev->subsystem_device & STX_NR_PORTS;

	DEBUG("No of ports detected is =%d \n",nr_ports);	

	for (i = 0; i < nr_ports; i++) {
		struct uart_port serial_port;
		memset(&serial_port, 0, sizeof(struct uart_port));

		serial_port.flags = UPF_SKIP_TEST | UPF_SHARE_IRQ;
		serial_port.uartclk = DEFAULT9865_BAUD * 16;
		serial_port.irq = dev->irq;
		serial_port.dev = &dev->dev;
		serial_port.iobase = pci_resource_start(dev,i);
		serial_port.iotype = UPIO_PORT;

		retval = serial8250_register_port(&serial_port);
		if (retval < 0) {
			printk(KERN_WARNING "Couldn't register serial port %s: %d\n", pci_name(dev), retval);
			goto disable;
		}

		isa_priv.line[isa_priv.nr_ports]=retval;
		isa_priv.nr_ports++;

	}

	return 0;

 disable:
	pci_disable_device(dev);
	return retval;
}

static void __devexit pci_isa_serial_remove_one(struct pci_dev *dev)
{
	int i;
	for (i = 0; i < isa_priv.nr_ports; i++)
		serial8250_unregister_port(isa_priv.line[i]);
}

static struct pci_device_id serial_pci_isa_tbl[] = {
	//{PCI_VENDOR_ID_NETMOS, PCI_DEVICE_ID_NETMOS_9865, PCI_SUBVEN_ID_9865,PCI_SUBDEV_ID_9865,0,0,0},
	{PCI_VENDOR_ID_NETMOS, PCI_DEVICE_ID_NETMOS_9865, PCI_SUBDEV_ID_9865, PCI_SUBVEN_ID_9865_4S, 0,0,0},
	{PCI_VENDOR_ID_NETMOS, PCI_DEVICE_ID_NETMOS_9865, PCI_SUBDEV_ID_9865, PCI_SUBVEN_ID_9865_3S, 0,0,0},
	{PCI_VENDOR_ID_NETMOS, PCI_DEVICE_ID_NETMOS_9865, PCI_SUBDEV_ID_9865, PCI_SUBVEN_ID_9865_2S, 0,0,0},
	{PCI_VENDOR_ID_NETMOS, PCI_DEVICE_ID_NETMOS_9865, PCI_SUBDEV_ID_9865, PCI_SUBVEN_ID_9865_1S, 0,0,0},
	{PCI_VENDOR_ID_NETMOS, PCI_DEVICE_ID_NETMOS_9865, PCI_SUBDEV_ID_9865, PCI_SUBVEN_ID_9865_2P, 0,0,0},
	{PCI_VENDOR_ID_NETMOS, PCI_DEVICE_ID_NETMOS_9865, PCI_SUBDEV_ID_9865, PCI_SUBVEN_ID_9865_1S1P, 0,0,0},
	{PCI_VENDOR_ID_NETMOS, PCI_DEVICE_ID_NETMOS_9865, PCI_SUBDEV_ID_9865, PCI_SUBVEN_ID_9865_2S1P, 0,0,0},
	{ 0, }
};

static struct pci_driver serial_pci_isa_driver = {
	.name		= "serial-isa",
	.probe		= pci_isa_serial_init_one,
	.remove		= __devexit_p(pci_isa_serial_remove_one),
	.id_table	= serial_pci_isa_tbl,
};

static int __init serial_isa_init(void)
{	
	isa_priv.nr_ports=0;
	return pci_register_driver(&serial_pci_isa_driver);
}

static void __exit serial_isa_exit(void)
{
	pci_unregister_driver(&serial_pci_isa_driver);
}

module_init(serial_isa_init);
module_exit(serial_isa_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MCS9865 ISA serial probe module");
MODULE_DEVICE_TABLE(pci, serial_pci_isa_tbl);
