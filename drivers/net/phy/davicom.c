// SPDX-License-Identifier: GPL-2.0+
/*
 * drivers/net/phy/davicom.c
 *
 * Driver for Davicom PHYs
 *
 * Author: Andy Fleming
 *
 * Copyright (c) 2004 Freescale Semiconductor, Inc.
 */
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/unistd.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/phy.h>
#include <linux/of.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <linux/uaccess.h>

#define MII_DM9161_SCR		0x10
#define MII_DM9161_SCR_INIT	0x0610
#define MII_DM9161_SCR_RMII	0x0100

/* DM9161 Interrupt Register */
#define MII_DM9161_INTR	0x15
#define MII_DM9161_INTR_PEND		0x8000
#define MII_DM9161_INTR_DPLX_MASK	0x0800
#define MII_DM9161_INTR_SPD_MASK	0x0400
#define MII_DM9161_INTR_LINK_MASK	0x0200
#define MII_DM9161_INTR_MASK		0x0100
#define MII_DM9161_INTR_DPLX_CHANGE	0x0010
#define MII_DM9161_INTR_SPD_CHANGE	0x0008
#define MII_DM9161_INTR_LINK_CHANGE	0x0004
#define MII_DM9161_INTR_INIT		0x0000
#define MII_DM9161_INTR_STOP	\
	(MII_DM9161_INTR_DPLX_MASK | MII_DM9161_INTR_SPD_MASK |	\
	 MII_DM9161_INTR_LINK_MASK | MII_DM9161_INTR_MASK)
#define MII_DM9161_INTR_CHANGE	\
	(MII_DM9161_INTR_DPLX_CHANGE | \
	 MII_DM9161_INTR_SPD_CHANGE | \
	 MII_DM9161_INTR_LINK_CHANGE)

/* DM9161 10BT Configuration/Status */
#define MII_DM9161_10BTCSR	0x12
#define MII_DM9161_10BTCSR_INIT	0x7800

MODULE_DESCRIPTION("Davicom PHY driver");
MODULE_AUTHOR("Andy Fleming");
MODULE_LICENSE("GPL");


static int dm9161_ack_interrupt(struct phy_device *phydev)
{
	int err = phy_read(phydev, MII_DM9161_INTR);

	return (err < 0) ? err : 0;
}

#define DM9161_DELAY 1
static int dm9161_config_intr(struct phy_device *phydev)
{
	int temp, err;

	temp = phy_read(phydev, MII_DM9161_INTR);

	if (temp < 0)
		return temp;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED) {
		err = dm9161_ack_interrupt(phydev);
		if (err)
			return err;

		temp &= ~(MII_DM9161_INTR_STOP);
		err = phy_write(phydev, MII_DM9161_INTR, temp);
	} else {
		temp |= MII_DM9161_INTR_STOP;
		err = phy_write(phydev, MII_DM9161_INTR, temp);
		if (err)
			return err;

		err = dm9161_ack_interrupt(phydev);
	}

	return err;
}

static irqreturn_t dm9161_handle_interrupt(struct phy_device *phydev)
{
	int irq_status;

	irq_status = phy_read(phydev, MII_DM9161_INTR);
	if (irq_status < 0) {
		phy_error(phydev);
		return IRQ_NONE;
	}

	if (!(irq_status & MII_DM9161_INTR_CHANGE))
		return IRQ_NONE;

	phy_trigger_machine(phydev);

	return IRQ_HANDLED;
}

static int dm9161_config_aneg(struct phy_device *phydev)
{
	int err;

	/* Isolate the PHY */
	err = phy_write(phydev, MII_BMCR, BMCR_ISOLATE);

	if (err < 0)
		return err;

	/* Configure the new settings */
	err = genphy_config_aneg(phydev);

	if (err < 0)
		return err;

	return 0;
}

static int dm9161_config_init(struct phy_device *phydev)
{
	int err, temp;

	/* Isolate the PHY */
	err = phy_write(phydev, MII_BMCR, BMCR_ISOLATE);

	if (err < 0)
		return err;

	switch (phydev->interface) {
	case PHY_INTERFACE_MODE_MII:
		temp = MII_DM9161_SCR_INIT;
		break;
	case PHY_INTERFACE_MODE_RMII:
		temp =  MII_DM9161_SCR_INIT | MII_DM9161_SCR_RMII;
		break;
	default:
		return -EINVAL;
	}

	/* Do not bypass the scrambler/descrambler */
	err = phy_write(phydev, MII_DM9161_SCR, temp);
	if (err < 0)
		return err;

	/* Clear 10BTCSR to default */
	err = phy_write(phydev, MII_DM9161_10BTCSR, MII_DM9161_10BTCSR_INIT);

	if (err < 0)
		return err;

	/* Reconnect the PHY, and enable Autonegotiation */
	return phy_write(phydev, MII_BMCR, BMCR_ANENABLE);
}

/* Stone add for DM9119 */
static int dm9119phy_config_init(struct phy_device *phydev)
{
	int ret;
	int err;
#ifdef CONFIG_OF	
	int num_data = 2,dataLen,num_regs,i,offset;
	int phyAddr[20]={0},phyValue[20]={0};
	struct mdio_device *mdio = &phydev->mdio;
	struct device *dev = &mdio->dev;

	if (!of_get_property(dev->of_node, "reg-values", &dataLen)){
		printk("Can not found reg-values property............\n");
	    return -1;
	}
	
	num_regs = dataLen / (sizeof(u32) * num_data);

	for (i = 0; i < num_regs; i++) {
	    offset = i * num_data;
	    if (of_property_read_u32_index(dev->of_node, "reg-values", offset, &phyAddr[i]))
	        return -1;
	    if (of_property_read_u32_index(dev->of_node, "reg-values", offset + 1, &phyValue[i]))
	        return -1;
		printk("DAVICOM Ethernet PHY Set Index: %d, Offset: 0x%x, Value: 0x%x\n",i,phyAddr[i],phyValue[i]);
		err = phy_write(phydev, 0x1f, phyAddr[i]);
		if (err < 0)
			ret = err;
		err = phy_write(phydev, 0x1e, phyValue[i]);
		if (err < 0) 
			ret = err;
	}
#else
	printk("DM9119 TX/RX setup. \r\n");
	/* Stone add for TX/RX enhance */
	/* 1. Unlock Extended registers */
	err = phy_write(phydev, 0x1f, 0x168);
	if (err < 0) ret = err;
	err = phy_write(phydev, 0x1e, 0x8040);
	if (err < 0) ret = err;
	/* 2. TX amplitude increase */
	err = phy_write(phydev, 0x1f, 0x488A);
	if (err < 0) ret = err;
	err = phy_write(phydev, 0x1e, 0x8030);
	if (err < 0) ret = err;
	err = phy_write(phydev, 0x1f, 0x4009);
	if (err < 0) ret = err;
	err = phy_write(phydev, 0x1e, 0x8031);
	if (err < 0) ret = err;
	err = phy_write(phydev, 0x1f, 0x240A);
	if (err < 0) ret = err;
	err = phy_write(phydev, 0x1e, 0x8032);
	if (err < 0) ret = err;
	err = phy_write(phydev, 0x1f, 0x2B);
	if (err < 0) ret = err;
	err = phy_write(phydev, 0x1e, 0x8034);
	if (err < 0) ret = err;
	err = phy_write(phydev, 0x1f, 0x2B);
	if (err < 0) ret = err;
	err = phy_write(phydev, 0x1e, 0x8037);
	if (err < 0) ret = err;
	err = phy_write(phydev, 0x1f, 0x2B);
	if (err < 0) ret = err;
	err = phy_write(phydev, 0x1e, 0x803A);
	if (err < 0) ret = err;
	err = phy_write(phydev, 0x1f, 0x2B);
	if (err < 0) ret = err;
	err = phy_write(phydev, 0x1e, 0x803D);
	if (err < 0) ret = err;
	/* 3. RX TAPs increase */
	err = phy_write(phydev, 0x1f, 0x06);
	if (err < 0) ret = err;
	err = phy_write(phydev, 0x1e, 0x8021);
	if (err < 0) ret = err;
#endif
	return ret;
}
static struct phy_driver dm91xx_driver[] = {
{
	.phy_id		= 0x0181b880,
	.name		= "Davicom DM9161E",
	.phy_id_mask	= 0x0ffffff0,
	/* PHY_BASIC_FEATURES */
	.config_init	= dm9161_config_init,
	.config_aneg	= dm9161_config_aneg,
	.config_intr	= dm9161_config_intr,
	.handle_interrupt = dm9161_handle_interrupt,
}, {
	.phy_id		= 0x0181b8b0,
	.name		= "Davicom DM9161B/C",
	.phy_id_mask	= 0x0ffffff0,
	/* PHY_BASIC_FEATURES */
	.config_init	= dm9161_config_init,
	.config_aneg	= dm9161_config_aneg,
	.config_intr	= dm9161_config_intr,
	.handle_interrupt = dm9161_handle_interrupt,
}, {
	.phy_id		= 0x0181b8a0,
	.name		= "Davicom DM9161A",
	.phy_id_mask	= 0x0ffffff0,
	/* PHY_BASIC_FEATURES */
	.config_init	= dm9161_config_init,
	.config_aneg	= dm9161_config_aneg,
	.config_intr	= dm9161_config_intr,
	.handle_interrupt = dm9161_handle_interrupt,
}, {
	.phy_id		= 0x00181b80,
	.name		= "Davicom DM9131",
	.phy_id_mask	= 0x0ffffff0,
	/* PHY_BASIC_FEATURES */
	.config_intr	= dm9161_config_intr,
}, {
	.phy_id         = 0x006e3212,
	.name           = "Davicom DM9119",
	.phy_id_mask    = 0x00ffffff,
	.config_init    = dm9119phy_config_init,
/*	.features       = PHY_GBIT_FEATURES | SUPPORTED_MII | SUPPORTED_AUI | SUPPORTED_FIBRE | SUPPORTED_BNC, */
	.config_aneg    = genphy_config_aneg,
	.aneg_done      = genphy_aneg_done,
	.suspend        = genphy_suspend,
	.resume         = genphy_resume,
} /* Stone add for DM9119 */
};

module_phy_driver(dm91xx_driver);

static struct mdio_device_id __maybe_unused davicom_tbl[] = {
	{ 0x0181b880, 0x0ffffff0 },
	{ 0x0181b8b0, 0x0ffffff0 },
	{ 0x0181b8a0, 0x0ffffff0 },
	{ 0x00181b80, 0x0ffffff0 },
	{ 0x006e3212, 0x00ffffff },  /* Stone add for DM9119 */
	{ }
};

MODULE_DEVICE_TABLE(mdio, davicom_tbl);
