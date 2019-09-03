// SPDX-License-Identifier: GPL-2.0+
/*
 *  DPAA backplane driver.
 *   Author: Shaohui Xie <Shaohui.Xie@freescale.com>
 *           Florinel Iordache <florinel.iordache@nxp.com>
 *
 * Copyright 2015 Freescale Semiconductor, Inc.
 * Copyright 2018-2019 NXP
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mii.h>
#include <linux/mdio.h>
#include <linux/ethtool.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_net.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/netdevice.h>

#include "fsl_backplane.h"


/* Training Algorithm version */
#define TRAIN_ALGORITHM_VERSION	"v1.4.36"

/* PCS Device Identifier */
#define PCS_PHY_DEVICE_ID			0x0083e400
#define PCS_PHY_DEVICE_ID_MASK		0xffffffff

/* It was decided on call from May 2nd
 * to disable happy condition no 4 on slide 4
 * because doesn't pass on LS1046 system
 */
//#define ENABLE_HAPPY_COND_4_ON_SLIDE_4

/* Perform all testing without less happy
 * and without even less happy conditions
 */
#define ENABLE_LESS_HAPPY_COND_2
#define ENABLE_EVEN_LESS_HAPPY_COND_3
#define ENABLE_SEEMINGLY_HAPPY_COND_4

/* Bin Modules order:
 * - defined: BinLong before BinM1 (as used by the Old algorithm)
 * - not defined: BinM1 before BinLong */
#define BIN_MODULES_ORDER_BINLONG_BINM1

/* Debug flags */
//#define DEBUG_MONITORING

/* Use Default PBL setup */
#define TECR_DEFAULT_RCW_SETUP

#ifndef TECR_DEFAULT_RCW_SETUP
/* 10G Long cables setup: 1 m to 2 m cables */
#define RATIO_PREQ_10G				0x3
#define RATIO_PST1Q_10G				0xd
#define RATIO_EQ_10G				0x20

/* 10G Short cables setup: up to 30 cm cable 
 * #define RATIO_PREQ_10G			0x3
 * #define RATIO_PST1Q_10G			0xa
 * #define RATIO_EQ_10G				0x29
 */

/* 40G Long cables setup: 1 m to 2 m cables */
#define RATIO_PREQ_40G				0x2
#define RATIO_PST1Q_40G				0xd
#define RATIO_EQ_40G				0x20

/* 40G Short cables setup: up to 30 cm cable 
 * #define RATIO_PREQ_40G			0x1
 * #define RATIO_PST1Q_40G			0x3
 * #define RATIO_EQ_40G				0x29
 */

/* LX2 2x40G default RCW setup 
 * #define RATIO_PREQ_40G			0x0
 * #define RATIO_PST1Q_40G			0x3
 * #define RATIO_EQ_40G				0x30
 */
#endif

/* Max/Min coefficient values */
#define PRE_COE_MAX					0x0
#define PRE_COE_MIN					0xc
#define POST_COE_MAX				0x0
#define POST_COE_MIN				0x10
#define ZERO_COE_MIN				0x14
#define ZERO_COE_MAX				0x30

/* KR PMD defines */
#define PMD_RESET					0x1
#define PMD_STATUS_SUP_STAT			0x4
#define PMD_STATUS_FRAME_LOCK		0x2
#define TRAIN_EN					0x3
#define TRAIN_DISABLE				0x1
#define RX_STAT						0x1

/* PCS STATUS 1 Register */
#define PCS_SR1             		0x1
#define PCS_RX_LINK_STAT_MASK		0x4
/* PCS BASE-R STATUS 1 Register */
#define PCS_BASE_R_SR1             	0x20
#define KR_RX_LINK_STAT_MASK		0x1000
#define PCS_HIGH_BER_MASK			0x0002

/* PCS BASE-R STATUS 2 Register */
#define PCS_BASE_R_SR2              0x21
#define PCS_BER_LOW_COUNTER_MASK	0x3f00
#define PCS_BER_LOW_COUNTER_SHIFT	8

/* PCS BER HIGH ORDER CNT Register */
#define PCS_BASE_R_BER_HIGH         0x2C
#define PCS_BER_HIGH_COUNTER_MASK	0xffff
#define PCS_BER_HIGH_COUNTER_SHIFT	6

/* KX PCS mode register */
#define KX_PCS_IF_MODE				0x8014

/* KX PCS mode register init value */
#define KX_IF_MODE_INIT				0x8

/* KX/KR AN registers */
#define AN_CTRL_INIT				0x1200
#define KX_AN_AD1_INIT				0x25
#define KR_AN_AD1_INIT_10G			0x85
#define KR_AN_AD1_INIT_40G			0x105
#define AN_LNK_UP_MASK				0x4
#define AN_COMPLETE_MASK			0x20
#define KR_AN_MASK_10G				0x8
#define KR_AN_MASK_40G				0x20
#define TRAIN_FAIL					0x8
#define KR_AN_40G_MDIO_OFFSET		4

/* XGKR Timeouts */
#define XGKR_TIMEOUT_1				100
#define XGKR_TIMEOUT_2				1000
#define XGKR_DENY_RT_INTERVAL		3000
#define XGKR_AN_WAIT_ITERATIONS 	5
#define TIMEOUT_LONG				3
#define TIMEOUT_M1					3
#define TIMEOUT_MOVE_BACK_PREV		6
#define TIMEOUT_REPEAT_REQUEST		10

/* XGKR Increment/Decrement Requests */
#define HOLD						0
#define INCREMENT					1
#define DECREMENT					2
#define RESERVED					3

/* XGKR Masks */
#define RX_READY_MASK				0x8000
#define PRESET_MASK					0x2000
#define INIT_MASK					0x1000
#define COP1_MASK					0x30
#define COP1_SHIFT					4
#define COZ_MASK					0xc
#define COZ_SHIFT					2
#define COM1_MASK					0x3
#define COM1_SHIFT					0
#define ALL_COE_MASK				(COP1_MASK | COZ_MASK | COM1_MASK)
#define LD_ALL_MASK					(PRESET_MASK | INIT_MASK | ALL_COE_MASK)
#define LP_STATUS_ALL_COE_UPDATED	0x15

/* Lanes definitions */
#define MASTER_LANE					0
#define SINGLE_LANE					0

/* Invalid value */
#define VAL_INVALID 				0xff

/* OSESTAT middle range */
#define OSESTAT_MIDRANGE_LOW		0x10
#define OSESTAT_MIDRANGE_HIGH		0x2F

/* Link_Training_Registers offsets */
static int lt_MDIO_MMD = 0;
static u32 lt_KR_PMD_CTRL = 0;
static u32 lt_KR_PMD_STATUS = 0;
static u32 lt_KR_LP_CU = 0;
static u32 lt_KR_LP_STATUS = 0;
static u32 lt_KR_LD_CU = 0;
static u32 lt_KR_LD_STATUS = 0;
static u32 lt_KR_PRBS_BERR_LOWER = 0;
static u32 lt_KR_PRBS_BERR_UPPER = 0;

/* KX/KR AN registers offsets */
static u32 g_an_STAT = 0;
static u32 g_an_AD1 = 0;
static u32 g_an_BP_STAT = 0;

#ifdef ALLOWED_VALUES_TABLE
/* Valid values table */
static const u32 preq_table[] = {0x0, 0x1, 0x3, 0x5,
				 0x7, 0x9, 0xb, 0xc, VAL_INVALID};
static const u32 pst1q_table[] = {0x0, 0x1, 0x3, 0x5, 0x7,
				  0x9, 0xb, 0xd, 0xf, 0x10, VAL_INVALID};
#endif //ALLOWED_VALUES_TABLE

enum backplane_mode {
	PHY_BACKPLANE_1000BASE_KX,
	PHY_BACKPLANE_10GBASE_KR,
	PHY_BACKPLANE_40GBASE_KR,
	PHY_BACKPLANE_INVAL
};

enum serdes_type {
	SERDES_10G,
	SERDES_28G,
	SERDES_INVAL
};

enum coe_field {
	COE_COP1,
	COE_COZ,
	COE_COM
};

enum coe_update {
	COE_NOTUPDATED,
	COE_UPDATED,
	COE_MIN,
	COE_MAX,
	COE_INV
};

/* XGKR phy statistics exported to ethtool */
enum xgkr_phy_stats_id {
	XGKR_PHY_STATS_LP_DETECTED,
	XGKR_PHY_STATS_PCS_LINK_UP,
	XGKR_PHY_STATS_PCS_LINKLOST_COUNT,
	XGKR_PHY_STATS_AN_LINK_UP,
	XGKR_PHY_STATS_AN_LINKLOST_COUNT,
#ifdef DEBUG_GBASE_R_LINK_UP
	XGKR_PHY_STATS_GBASE_R_LINK_UP,
#endif
	XGKR_PHY_STATS_ANEG_COMPLETE,
#ifdef DEBUG_AN_STATUS_ANEG_COMPLETE
	XGKR_PHY_STATS_AN_STATUS_ANEG_COMPLETE,
#endif
	XGKR_PHY_STATS_ANEG_RESTART_COUNT,
	XGKR_PHY_STATS_HIGH_BER,
	XGKR_PHY_STATS_BER_COUNTER,
	/*
	 * Add new xgkr stats attribute above and then update
	 * xgkr_phy_stats_strings[]
	 */
	XGKR_PHY_STATS_COUNT,
};

static const char xgkr_phy_stats_strings[XGKR_PHY_STATS_COUNT][ETH_GSTRING_LEN] = {
	[XGKR_PHY_STATS_LP_DETECTED]   			= "LP detected",
	[XGKR_PHY_STATS_PCS_LINK_UP]   			= "PCS Link up",
	[XGKR_PHY_STATS_PCS_LINKLOST_COUNT]   	= "PCS Link lost detected count",
	[XGKR_PHY_STATS_AN_LINK_UP]   			= "AN Link up",
	[XGKR_PHY_STATS_AN_LINKLOST_COUNT]   	= "AN Link lost detected count",
#ifdef DEBUG_GBASE_R_LINK_UP
	[XGKR_PHY_STATS_GBASE_R_LINK_UP]		= "10GBase-R Link up",
#endif
	[XGKR_PHY_STATS_ANEG_COMPLETE]			= "Autonegotiation complete",
#ifdef DEBUG_AN_STATUS_ANEG_COMPLETE
	[XGKR_PHY_STATS_AN_STATUS_ANEG_COMPLETE] = "AN status aneg complete",
#endif
	[XGKR_PHY_STATS_ANEG_RESTART_COUNT]		= "Autonegotiation restarted count",
	[XGKR_PHY_STATS_HIGH_BER]  				= "PCS reporting high BER",
	[XGKR_PHY_STATS_BER_COUNTER]  			= "BER counter",
};

/* XGKR lane statistics exported to ethtool */
enum xgkr_lane_stats_id {
	XGKR_LANE_STATS_INIT_RATIO_PREQ,
	XGKR_LANE_STATS_INIT_RATIO_PST1Q,
	XGKR_LANE_STATS_INIT_ADPT_EQ,
	XGKR_LANE_STATS_CRT_RATIO_PREQ,
	XGKR_LANE_STATS_CRT_RATIO_PST1Q,
	XGKR_LANE_STATS_CRT_ADPT_EQ,
	XGKR_LANE_STATS_TUNED_RATIO_PREQ,
	XGKR_LANE_STATS_TUNED_RATIO_PST1Q,
	XGKR_LANE_STATS_TUNED_ADPT_EQ,
	XGKR_LANE_STATS_INIT_TECR0,
	XGKR_LANE_STATS_TUNED_TECR0,
	XGKR_LANE_STATS_LT_COMPLETE,
	XGKR_LANE_STATS_LT_DURATION,
	XGKR_LANE_STATS_LT_STEPS,
	XGKR_LANE_STATS_LT_RESTARTED,
	XGKR_LANE_STATS_LT_FAILS,
	XGKR_LANE_STATS_LT_TIMEOUTS,
	XGKR_LANE_STATS_TRAIN_REMOTE_CYCLES,
	XGKR_LANE_STATS_TRAIN_LOCAL_CYCLES,
	XGKR_LANE_STATS_CU_TO_LP,
	XGKR_LANE_STATS_CU_FROM_LP,
	XGKR_LANE_STATS_INC_COP,
	XGKR_LANE_STATS_INC_COZ,
	XGKR_LANE_STATS_INC_COM,
	XGKR_LANE_STATS_DEC_COP,
	XGKR_LANE_STATS_DEC_COZ,
	XGKR_LANE_STATS_DEC_COM,
	XGKR_LANE_STATS_LD_PRESET,
	XGKR_LANE_STATS_LD_INIT,
	XGKR_LANE_STATS_LD_RX_RDY,
	XGKR_LANE_STATS_LP_RX_RDY,
	XGKR_LANE_STATS_RX_EQ_GAINK2,
	XGKR_LANE_STATS_PRBS_ERR_COUNTER,
	/*
	 * Add new xgkr stats attribute above and then update
	 * xgkr_lane_stats_strings[]
	 */
	XGKR_LANE_STATS_COUNT,
};

static const char xgkr_lane_stats_strings[XGKR_LANE_STATS_COUNT][ETH_GSTRING_LEN] = {
	[XGKR_LANE_STATS_INIT_RATIO_PREQ]   	= "Initial RATIO_PREQ",
	[XGKR_LANE_STATS_INIT_RATIO_PST1Q]  	= "Initial RATIO_PST1Q",
	[XGKR_LANE_STATS_INIT_ADPT_EQ]   		= "Initial ADPT_EQ",
	[XGKR_LANE_STATS_CRT_RATIO_PREQ]   		= "Current RATIO_PREQ",
	[XGKR_LANE_STATS_CRT_RATIO_PST1Q]   	= "Current RATIO_PST1Q",
	[XGKR_LANE_STATS_CRT_ADPT_EQ]   		= "Current ADPT_EQ",
	[XGKR_LANE_STATS_TUNED_RATIO_PREQ]   	= "Tuned RATIO_PREQ",
	[XGKR_LANE_STATS_TUNED_RATIO_PST1Q]  	= "Tuned RATIO_PST1Q",
	[XGKR_LANE_STATS_TUNED_ADPT_EQ]   		= "Tuned ADPT_EQ",
	[XGKR_LANE_STATS_INIT_TECR0]   			= "Initial TECR0",
	[XGKR_LANE_STATS_TUNED_TECR0]   		= "Tuned TECR0",
	[XGKR_LANE_STATS_LT_COMPLETE]			= "LT complete",
	[XGKR_LANE_STATS_LT_DURATION]			= "LT duration",
	[XGKR_LANE_STATS_LT_STEPS]   			= "Link training steps",
	[XGKR_LANE_STATS_LT_RESTARTED]   		= "Link training restarted",
	[XGKR_LANE_STATS_LT_FAILS]   			= "Link training fail count",
	[XGKR_LANE_STATS_LT_TIMEOUTS]   		= "Link training timeout count",
	[XGKR_LANE_STATS_TRAIN_REMOTE_CYCLES]   = "Remote Tx tuning cycles",
	[XGKR_LANE_STATS_TRAIN_LOCAL_CYCLES]   	= "Local Tx tuning cycles",
	[XGKR_LANE_STATS_CU_TO_LP]   			= "Coefficient Updates to LP",
	[XGKR_LANE_STATS_CU_FROM_LP] 			= "Coefficient Updates from LP",
	[XGKR_LANE_STATS_INC_COP]   			= "C(+1) increment count",
	[XGKR_LANE_STATS_INC_COZ]   			= "C(0) increment count",
	[XGKR_LANE_STATS_INC_COM]   			= "C(-1) increment count",
	[XGKR_LANE_STATS_DEC_COP]   			= "C(+1) decrement count",
	[XGKR_LANE_STATS_DEC_COZ]   			= "C(0) decrement count",
	[XGKR_LANE_STATS_DEC_COM]   			= "C(-1) decrement count",
	[XGKR_LANE_STATS_LD_PRESET]  			= "LD Preset count",
	[XGKR_LANE_STATS_LD_INIT]   			= "LD Init count",
	[XGKR_LANE_STATS_LD_RX_RDY]  			= "LD receiver ready",
	[XGKR_LANE_STATS_LP_RX_RDY]  			= "LP receiver ready",
	[XGKR_LANE_STATS_RX_EQ_GAINK2]  		= "Rx EQ Median Gaink2",
	[XGKR_LANE_STATS_PRBS_ERR_COUNTER]  	= "PRBS sequence bit errors",
};

static char crt_lane_stats_strings[XGKR_LANE_STATS_COUNT][ETH_GSTRING_LEN];


/* Debug logging and events tracing support */
/* Enable Debug logging and tracing */
#define ENABLE_DEBUG_TRACING

#ifdef ENABLE_DEBUG_TRACING

/* Enable desired interface for debug logging */
//#define DEBUG_LOG_ON_CONSOLE
#define DEBUG_LOG_ON_TRACE

/* Enable Debug print of snapshots */
//#define DEBUG_LOG_SNAPSHOTS

/* Events tracing
 * CREATE_TRACE_POINTS only needs to be defined once. Other files
 * using trace events only need to #include <trace/events/sched.h>
 */
#define CREATE_TRACE_POINTS
#include "fsl_backplane_trace.h"

#define DBG_LOG_PREFIX 		"xgkr_debug_log"
#define DBG_LOG_BUF_SIZE	200
#define DBG_LOG_PREPARE_ARGS(msg)  va_list args; va_start(args, msg); \
		vsnprintf(log_buffer, DBG_LOG_BUF_SIZE - 1, msg, args); va_end(args);
static char log_buffer[DBG_LOG_BUF_SIZE];
static void dbg_log_phy(struct phy_device *phydev, char *msg, ...)
{
	char phy_name[50];
	DBG_LOG_PREPARE_ARGS(msg);
	sprintf(phy_name, "%s", (phydev && phydev->attached_dev) ? dev_name(phydev->attached_dev->dev.parent) : "");
#ifdef DEBUG_LOG_ON_CONSOLE
    printk("%s: %s: %s\n", DBG_LOG_PREFIX, phy_name, log_buffer);
#endif
#ifdef DEBUG_LOG_ON_TRACE
    trace_xgkr_debug_log(phy_name, log_buffer);
#endif
}
static void dbg_log_lane(struct xgkr_params *xgkr, char *msg, ...)
{
	char phy_name[50];
	DBG_LOG_PREPARE_ARGS(msg);
	if (xgkr)
		sprintf(phy_name, "%s/ln%d", (xgkr->phydev && xgkr->phydev->attached_dev) ? dev_name(xgkr->phydev->attached_dev->dev.parent) : "", xgkr->idx);
	else
		phy_name[0] = '\0';
#ifdef DEBUG_LOG_ON_CONSOLE
	printk("%s: %s: %s\n", DBG_LOG_PREFIX, phy_name, log_buffer);
#endif
#ifdef DEBUG_LOG_ON_TRACE
	trace_xgkr_debug_log(phy_name, log_buffer);
#endif
}
#else
/* Don't use Debug logging and tracing */
#define dbg_log_phy(phydev, msg, ...)
#define dbg_log_lane(xgkr, msg, ...)
#define trace_xgkr_coe_update(xgkr, update, send)
#define trace_xgkr_coe_status(xgkr, status, local)
#define trace_xgkr_bin_snapshots(xgkr, bin_name, snapshot)
#define trace_xgkr_gain_snapshots(xgkr, gain_name, snapshot)
#endif //ENABLE_DEBUG_TRACING


static void setup_an_lt_ls(void)
{
	/* KR PMD registers */
	lt_MDIO_MMD = MDIO_MMD_PMAPMD;
	lt_KR_PMD_CTRL = 0x96;
	lt_KR_PMD_STATUS = 0x97;
	lt_KR_LP_CU = 0x98;
	lt_KR_LP_STATUS = 0x99;
	lt_KR_LD_CU = 0x9a;
	lt_KR_LD_STATUS = 0x9b;
	lt_KR_PRBS_BERR_LOWER = 0x8001;
	lt_KR_PRBS_BERR_UPPER = 0x8002;

	/* KX/KR AN registers */
	g_an_STAT = 1;
	g_an_AD1 = 0x11;
	g_an_BP_STAT = 0x30;
}

static void setup_an_lt_lx(void)
{
	/* Auto-Negotiation and Link Training Core Registers page 1: 256 = 0x100 */
	lt_MDIO_MMD = MDIO_MMD_AN;
	lt_KR_PMD_CTRL = 0x100;
	lt_KR_PMD_STATUS = 0x101;
	lt_KR_LP_CU = 0x102;
	lt_KR_LP_STATUS = 0x103;
	lt_KR_LD_CU = 0x104;
	lt_KR_LD_STATUS = 0x105;
	lt_KR_PRBS_BERR_LOWER = 0x806B;
	lt_KR_PRBS_BERR_UPPER = 0x806C;

	/* KX/KR AN registers */
	g_an_STAT = 1;

	//TODO: perhaps it should be like above: 0x11 si 0x30 - see chapter in LX2160ARM: Table 26-14. Backplane Auto-Negotiation Registers
	g_an_AD1 = 0x03;
	g_an_BP_STAT = 0x0F;
}

static u32 le_ioread32(u32 *reg)
{
	return ioread32(reg);
}

static void le_iowrite32(u32 value, u32 *reg)
{
	iowrite32(value, reg);
}

static u32 be_ioread32(u32 *reg)
{
	return ioread32be(reg);
}

static void be_iowrite32(u32 value, u32 *reg)
{
	iowrite32be(value, reg);
}

/**
 * xgkr_phy_write_mmd - Wrapper function for phy_write_mmd
 * for writing a register on an MMD on a given PHY.
 *
 * Same rules as for phy_write_mmd();
 */
static int xgkr_phy_write_mmd(struct xgkr_params *xgkr, int devad, u32 regnum, u16 val)
{
	struct phy_device *phydev = xgkr->phydev;
	struct xgkr_phy_data *xgkr_inst = phydev->priv;
	int mdio_addr = phydev->mdio.addr;
	int err;

	mutex_lock(&xgkr_inst->phy_lock);

	if (xgkr_inst->bp_mode == PHY_BACKPLANE_40GBASE_KR && devad == MDIO_MMD_AN) {
		/* 40G AN: prepare mdio address for writing phydev AN registers for 40G on respective lane */
		phydev->mdio.addr = KR_AN_40G_MDIO_OFFSET + xgkr->idx;
	}

	err = phy_write_mmd(phydev, devad, regnum, val);
	if (err)
		dev_err(&phydev->mdio.dev, "Writing PHY (%p) MMD = 0x%02x register = 0x%02x failed with error code: 0x%08x \n", phydev, devad, regnum, err);

	if (xgkr_inst->bp_mode == PHY_BACKPLANE_40GBASE_KR && devad == MDIO_MMD_AN) {
		/* 40G AN: restore mdio address */
		phydev->mdio.addr = mdio_addr;
	}

	mutex_unlock(&xgkr_inst->phy_lock);

	return err;
}

/**
 * xgkr_phy_read_mmd - Wrapper function for phy_read_mmd
 * for reading a register from an MMD on a given PHY.
 *
 * Same rules as for phy_read_mmd();
 */
static int xgkr_phy_read_mmd(struct xgkr_params *xgkr, int devad, u32 regnum)
{
	struct phy_device *phydev = xgkr->phydev;
	struct xgkr_phy_data *xgkr_inst = phydev->priv;
	int mdio_addr = phydev->mdio.addr;
	int ret;

	mutex_lock(&xgkr_inst->phy_lock);

	if (xgkr_inst->bp_mode == PHY_BACKPLANE_40GBASE_KR && devad == MDIO_MMD_AN) {
		/* 40G AN: prepare mdio address for reading phydev AN registers for 40G on respective lane */
		phydev->mdio.addr = KR_AN_40G_MDIO_OFFSET + xgkr->idx;
	}

	ret = phy_read_mmd(phydev, devad, regnum);

	if (xgkr_inst->bp_mode == PHY_BACKPLANE_40GBASE_KR && devad == MDIO_MMD_AN) {
		/* 40G AN: restore mdio address */
		phydev->mdio.addr = mdio_addr;
	}

	mutex_unlock(&xgkr_inst->phy_lock);

	return ret;
}

static void tx_condition_init(struct tx_condition *tx_c)
{
	tx_c->bin_m1_stop = false;
	tx_c->bin_long_stop = false;
	tx_c->done_training = false;
	tx_c->tx_complete = false;
	tx_c->sent_init = false;
	tx_c->m1_min_max_cnt = 0;
	tx_c->long_min_max_cnt = 0;
}

static void tune_tecr(struct xgkr_params *xgkr, bool reset_lane)
{
	struct phy_device *phydev = xgkr->phydev;
	struct xgkr_phy_data *xgkr_inst = phydev->priv;
	struct tecr_params tecr;
	bool reset = false;

	if (xgkr_inst->bp_mode == PHY_BACKPLANE_40GBASE_KR) {
		/* Reset only the Master Lane */
		reset = (xgkr->idx == MASTER_LANE);
	} else {
		reset = true;
	}
	
	//Do not reset the lane if this is how it was asked
	if (!reset_lane)
		reset = false;

	tecr.ratio_preq = xgkr->ratio_preq;
	tecr.ratio_pst1q = xgkr->ratio_pst1q;
	tecr.adpt_eq = xgkr->adpt_eq;
	tecr.amp_red = xgkr->def_amp_red;
	xgkr->srds->tune_tecr(xgkr->reg_base, &tecr, reset);

	xgkr->tuned_ratio_preq = xgkr->ratio_preq;
	xgkr->tuned_ratio_pst1q = xgkr->ratio_pst1q;
	xgkr->tuned_adpt_eq = xgkr->adpt_eq;
}

static void start_lt(struct xgkr_params *xgkr)
{
	xgkr_phy_write_mmd(xgkr, lt_MDIO_MMD, lt_KR_PMD_CTRL, TRAIN_EN);
	xgkr->stats.training_restarted_count++;
}

static void stop_lt(struct xgkr_params *xgkr)
{
	xgkr_phy_write_mmd(xgkr, lt_MDIO_MMD, lt_KR_PMD_CTRL, TRAIN_DISABLE);
}

static void reset_lt(struct xgkr_params *xgkr)
{
	xgkr_phy_write_mmd(xgkr, lt_MDIO_MMD, MDIO_CTRL1, PMD_RESET);
	xgkr_phy_write_mmd(xgkr, lt_MDIO_MMD, lt_KR_PMD_CTRL, TRAIN_DISABLE);
	xgkr_phy_write_mmd(xgkr, lt_MDIO_MMD, lt_KR_LD_CU, 0);
	xgkr_phy_write_mmd(xgkr, lt_MDIO_MMD, lt_KR_LD_STATUS, 0);
	xgkr_phy_write_mmd(xgkr, lt_MDIO_MMD, lt_KR_PMD_STATUS, 0);
	xgkr_phy_write_mmd(xgkr, lt_MDIO_MMD, lt_KR_LP_CU, 0);
	xgkr_phy_write_mmd(xgkr, lt_MDIO_MMD, lt_KR_LP_STATUS, 0);
}

static void ld_coe_status(struct xgkr_params *xgkr)
{
	/* 72.6.10.2.5 Coefficient update process
	 * Once the updated, maximum, or minimum state is reported it continues to be reported
	 * until a hold request is received, after which the status reverts to not_updated.
	 */
	xgkr_phy_write_mmd(xgkr, lt_MDIO_MMD,
		      lt_KR_LD_STATUS, xgkr->ld_status);

	/* tracing point */
	trace_xgkr_coe_status(xgkr, xgkr->ld_status, true);
}

static void ld_coe_update(struct xgkr_params *xgkr)
{
	//dev_dbg(&xgkr->phydev->mdio.dev, "sending request: %x\n", xgkr->ld_update);
	xgkr->stats.coe_updates_to_lp++;
	xgkr_phy_write_mmd(xgkr, lt_MDIO_MMD,
		      lt_KR_LD_CU, xgkr->ld_update);

	/* tracing point */
	trace_xgkr_coe_update(xgkr, xgkr->ld_update, true);
}

/**
 * RX_READY_MASK
 * Receiver Ready
 * 0b - The LP receiver is requesting that training continue
 * 1b - The LP receiver has determined that training is complete and is prepared to receive data.
 */
static int check_rx(struct xgkr_params *xgkr)
{
	return xgkr_phy_read_mmd(xgkr, lt_MDIO_MMD, lt_KR_LP_STATUS) &
			    RX_READY_MASK;
}

static u32 get_ber_counter(struct phy_device *phydev)
{
	u32 ber, ber_low, ber_high;
	struct xgkr_phy_data *xgkr_inst = phydev->priv;

	ber_low = phy_read_mmd(phydev, MDIO_MMD_PCS, PCS_BASE_R_SR2);
	ber = (ber_low & PCS_BER_LOW_COUNTER_MASK) >> PCS_BER_LOW_COUNTER_SHIFT;

	if (xgkr_inst->bp_mode == PHY_BACKPLANE_40GBASE_KR) {
		ber_high = phy_read_mmd(phydev, MDIO_MMD_PCS, PCS_BASE_R_BER_HIGH);
		ber |= (ber_high & PCS_BER_HIGH_COUNTER_MASK) << PCS_BER_HIGH_COUNTER_SHIFT;
	}

	return ber;
}

static u32 get_prbs_err_counter(struct xgkr_params *xgkr)
{
	u32 err, err_low, err_high;

	err_low = xgkr_phy_read_mmd(xgkr, lt_MDIO_MMD, lt_KR_PRBS_BERR_LOWER);
	err_high = xgkr_phy_read_mmd(xgkr, lt_MDIO_MMD, lt_KR_PRBS_BERR_UPPER);

	err = (err_high << 16) | err_low;

	return err;
}

/**
 * Read AN Link Status
 */
static int is_an_link_up(struct phy_device *phydev)
{
	struct xgkr_phy_data *xgkr_inst = phydev->priv;
	int ret, val = 0;

	mutex_lock(&xgkr_inst->phy_lock);

	// Read twice because Link_Status is LL (Latched Low) bit
	// we can use MDIO_STAT1 instead of g_an_STAT because it's also: 0x01
	val = phy_read_mmd(phydev, MDIO_MMD_AN, g_an_STAT);
	val = phy_read_mmd(phydev, MDIO_MMD_AN, g_an_STAT);

	mutex_unlock(&xgkr_inst->phy_lock);

	ret = (val & AN_LNK_UP_MASK) ? 1 : 0;

	if (xgkr_inst->stats.last_status_an_link_up && !ret)
		xgkr_inst->stats.an_link_lost_count++;

	xgkr_inst->stats.last_status_an_link_up = ret;

	return ret;
}

/**
 * Read PCS Link Status
 */
static int is_pcs_link_up(struct phy_device *phydev)
{
	struct xgkr_phy_data *xgkr_inst = phydev->priv;
	int ret, val = 0;

	mutex_lock(&xgkr_inst->phy_lock);

	//PCS status 1 Register / PCS Receive Link Status - is LL (Latched Low) bit
	val = phy_read_mmd(phydev, MDIO_MMD_PCS, PCS_SR1);
	val = phy_read_mmd(phydev, MDIO_MMD_PCS, PCS_SR1);

	mutex_unlock(&xgkr_inst->phy_lock);

	//PCS status 1 Register / PCS Receive Link Status
	ret = (val & PCS_RX_LINK_STAT_MASK) ? 1 : 0;

	if (xgkr_inst->stats.last_status_pcs_link_up && !ret)
		xgkr_inst->stats.pcs_link_lost_count++;

	xgkr_inst->stats.last_status_pcs_link_up = ret;

	return ret;
}

/**
 * Read PCS Link Status
 */
#ifdef DEBUG_GBASE_R_LINK_UP
static int is_10gbase_r_pcs_link_up(struct phy_device *phydev)
{
	struct xgkr_phy_data *xgkr_inst = phydev->priv;
	int ret, val = 0;

	mutex_lock(&xgkr_inst->phy_lock);

	//10GBASE-R PCS Status 1 Register / 10GBASE-R Receive Link Status
	val = phy_read_mmd(phydev, MDIO_MMD_PCS, PCS_BASE_R_SR1);

	mutex_unlock(&xgkr_inst->phy_lock);

	//10GBASE-R PCS Status 1 Register / 10GBASE-R Receive Link Status
	ret = (val & KR_RX_LINK_STAT_MASK) ? 1 : 0;

	return ret;
}
#endif

/**
 * Generic Link-up Status: use AN link-up
 */
static int is_link_up(struct phy_device *phydev)
{
	return is_an_link_up(phydev);
}

static int is_high_ber(struct phy_device *phydev)
{
	struct xgkr_phy_data *xgkr_inst = phydev->priv;
	int val = 0;

	mutex_lock(&xgkr_inst->phy_lock);

	val = phy_read_mmd(phydev, MDIO_MMD_PCS, PCS_BASE_R_SR1);

	mutex_unlock(&xgkr_inst->phy_lock);

	return (val & PCS_HIGH_BER_MASK) ? 1 : 0;
}

static int is_link_training_fail(struct xgkr_params *xgkr)
{
	struct phy_device *phydev = xgkr->phydev;
	int val;
	int timeout = 100;

	val = xgkr_phy_read_mmd(xgkr, lt_MDIO_MMD, lt_KR_PMD_STATUS);

	if (!(val & TRAIN_FAIL) && (val & RX_STAT)) {
		/* check LNK_STAT for sure */
		while (timeout--) {
			if (is_link_up(phydev))
				return 0;

			usleep_range(100, 500);
		}
	}

	return 1;
}

static int lanes_trained_count(struct xgkr_phy_data *xgkr_inst)
{
	int i, lanes_trained = 0;

	for (i = 0; i < xgkr_inst->phy_lanes; i++) {
		if (xgkr_inst->xgkr[i].state == TRAINED)
			lanes_trained++;
	}
	return lanes_trained;
}

static int are_all_lanes_trained(struct xgkr_phy_data *xgkr_inst)
{
	int i;

	for (i = 0; i < xgkr_inst->phy_lanes; i++) {
		if (xgkr_inst->xgkr[i].state != TRAINED)
			return 0;
	}
	return 1;
}

static void report_phy_stats(struct phy_device *phydev, u64 *data)
{
	struct xgkr_phy_data *xgkr_inst = phydev->priv;

#ifdef DEBUG_AN_STATUS_ANEG_COMPLETE
	int an_state = phy_read_mmd(phydev, MDIO_MMD_AN, g_an_STAT);
#endif

	data[XGKR_PHY_STATS_LP_DETECTED] = xgkr_inst->stats.lp_detected;
	data[XGKR_PHY_STATS_PCS_LINK_UP] = is_pcs_link_up(phydev);
	data[XGKR_PHY_STATS_PCS_LINKLOST_COUNT] = xgkr_inst->stats.pcs_link_lost_count;
	data[XGKR_PHY_STATS_AN_LINK_UP] = is_an_link_up(phydev);
	data[XGKR_PHY_STATS_AN_LINKLOST_COUNT] = xgkr_inst->stats.an_link_lost_count;
#ifdef DEBUG_GBASE_R_LINK_UP
	data[XGKR_PHY_STATS_GBASE_R_LINK_UP] = is_10gbase_r_pcs_link_up(phydev);
#endif
	data[XGKR_PHY_STATS_ANEG_COMPLETE] = xgkr_inst->aneg_done ? 1 : 0;
#ifdef DEBUG_AN_STATUS_ANEG_COMPLETE
	data[XGKR_PHY_STATS_AN_STATUS_ANEG_COMPLETE] = (an_state & AN_COMPLETE_MASK) ? 1 : 0;
#endif
	data[XGKR_PHY_STATS_ANEG_RESTART_COUNT] = xgkr_inst->stats.aneg_restarted_count;
	data[XGKR_PHY_STATS_HIGH_BER] = is_high_ber(phydev);
	data[XGKR_PHY_STATS_BER_COUNTER] = get_ber_counter(phydev);
}

static void report_lane_stats(struct xgkr_params *xgkr, u64 *data, int base_index)
{
	data[base_index + XGKR_LANE_STATS_INIT_RATIO_PREQ] = xgkr->def_ratio_preq;
	data[base_index + XGKR_LANE_STATS_INIT_RATIO_PST1Q] = xgkr->def_ratio_pst1q;
	data[base_index + XGKR_LANE_STATS_INIT_ADPT_EQ] = xgkr->def_adpt_eq;
	data[base_index + XGKR_LANE_STATS_CRT_RATIO_PREQ] = xgkr->ratio_preq;
	data[base_index + XGKR_LANE_STATS_CRT_RATIO_PST1Q] = xgkr->ratio_pst1q;
	data[base_index + XGKR_LANE_STATS_CRT_ADPT_EQ] = xgkr->adpt_eq;
	data[base_index + XGKR_LANE_STATS_TUNED_RATIO_PREQ] = xgkr->tuned_ratio_preq;
	data[base_index + XGKR_LANE_STATS_TUNED_RATIO_PST1Q] = xgkr->tuned_ratio_pst1q;
	data[base_index + XGKR_LANE_STATS_TUNED_ADPT_EQ] = xgkr->tuned_adpt_eq;
	data[base_index + XGKR_LANE_STATS_INIT_TECR0] = xgkr->stats.init_tecr0;
	data[base_index + XGKR_LANE_STATS_TUNED_TECR0] = xgkr->stats.tuned_tecr0;
	data[base_index + XGKR_LANE_STATS_LT_COMPLETE] = (xgkr->state == TRAINED) ? 1 : 0;
	data[base_index + XGKR_LANE_STATS_LT_DURATION] = xgkr->stats.lt_duration;
	data[base_index + XGKR_LANE_STATS_LT_STEPS] = xgkr->stats.training_steps;
	data[base_index + XGKR_LANE_STATS_LT_RESTARTED] = xgkr->stats.training_restarted_count;
	data[base_index + XGKR_LANE_STATS_LT_FAILS] = xgkr->stats.training_failed_count;
	data[base_index + XGKR_LANE_STATS_LT_TIMEOUTS] = xgkr->stats.training_timeouts;
	data[base_index + XGKR_LANE_STATS_TRAIN_REMOTE_CYCLES] = xgkr->stats.training_cycles_remote_tx;
	data[base_index + XGKR_LANE_STATS_TRAIN_LOCAL_CYCLES] = xgkr->stats.training_cycles_local_tx;
	data[base_index + XGKR_LANE_STATS_CU_TO_LP] = xgkr->stats.coe_updates_to_lp;
	data[base_index + XGKR_LANE_STATS_CU_FROM_LP] = xgkr->stats.coe_updates_from_lp;
	data[base_index + XGKR_LANE_STATS_INC_COP] = xgkr->stats.inc_coe_count[COE_COP1];
	data[base_index + XGKR_LANE_STATS_INC_COZ] = xgkr->stats.inc_coe_count[COE_COZ];
	data[base_index + XGKR_LANE_STATS_INC_COM] = xgkr->stats.inc_coe_count[COE_COM];
	data[base_index + XGKR_LANE_STATS_DEC_COP] = xgkr->stats.dec_coe_count[COE_COP1];
	data[base_index + XGKR_LANE_STATS_DEC_COZ] = xgkr->stats.dec_coe_count[COE_COZ];
	data[base_index + XGKR_LANE_STATS_DEC_COM] = xgkr->stats.dec_coe_count[COE_COM];
	data[base_index + XGKR_LANE_STATS_LD_PRESET] = xgkr->stats.ld_preset_count;
	data[base_index + XGKR_LANE_STATS_LD_INIT] = xgkr->stats.ld_init_count;
	data[base_index + XGKR_LANE_STATS_LD_RX_RDY] = (xgkr->ld_status & RX_READY_MASK) ? 1 : 0;
	data[base_index + XGKR_LANE_STATS_LP_RX_RDY] = check_rx(xgkr) ? 1 : 0;
	data[base_index + XGKR_LANE_STATS_RX_EQ_GAINK2] = xgkr->stats.median_gaink2;
	data[base_index + XGKR_LANE_STATS_PRBS_ERR_COUNTER] = get_prbs_err_counter(xgkr);
}

static void start_xgkr_state_machine(struct delayed_work *work, unsigned long timeout)
{
	queue_delayed_work(system_power_efficient_wq, work,
			   msecs_to_jiffies(timeout));
}

static void start_xgkr_an(struct xgkr_params *xgkr)
{
	struct phy_device *phydev = xgkr->phydev;
	struct xgkr_phy_data *xgkr_inst = phydev->priv;
	int i;
	int err;

	switch (xgkr_inst->bp_mode)
	{
	case PHY_BACKPLANE_1000BASE_KX:
		dev_err(&phydev->mdio.dev, "Wrong call path for 1000Base-KX \n");
		break;

	case PHY_BACKPLANE_10GBASE_KR:
		err = xgkr_phy_write_mmd(xgkr, MDIO_MMD_AN, g_an_AD1, KR_AN_AD1_INIT_10G);
		if (err)
			dev_err(&phydev->mdio.dev, "Setting AN register 0x%02x failed with error code: 0x%08x \n", g_an_AD1, err);
		udelay(1);
		err = xgkr_phy_write_mmd(xgkr, MDIO_MMD_AN, MDIO_CTRL1, AN_CTRL_INIT);
		if (err)
			dev_err(&phydev->mdio.dev, "Setting AN register 0x%02x failed with error code: 0x%08x \n", MDIO_CTRL1, err);
		break;

	case PHY_BACKPLANE_40GBASE_KR:
		if (xgkr->idx == MASTER_LANE) {
			for (i = 0; i < xgkr_inst->phy_lanes; i++) {
				err = xgkr_phy_write_mmd(&xgkr_inst->xgkr[i], MDIO_MMD_AN, g_an_AD1, KR_AN_AD1_INIT_40G);
				if (err)
					dev_err(&phydev->mdio.dev, "Setting AN register 0x%02x on lane %d failed with error code: 0x%08x \n", g_an_AD1, xgkr_inst->xgkr[i].idx, err);
			}
			udelay(1);
			err = xgkr_phy_write_mmd(xgkr, MDIO_MMD_AN, MDIO_CTRL1, AN_CTRL_INIT);
			if (err)
				dev_err(&phydev->mdio.dev, "Setting AN register 0x%02x on Master Lane failed with error code: 0x%08x \n", MDIO_CTRL1, err);
		}
		break;
	}
}

static void start_1gkx_an(struct phy_device *phydev)
{
	phy_write_mmd(phydev, MDIO_MMD_PCS, KX_PCS_IF_MODE, KX_IF_MODE_INIT);
	phy_write_mmd(phydev, MDIO_MMD_AN, g_an_AD1, KX_AN_AD1_INIT);
	phy_read_mmd(phydev, MDIO_MMD_AN, MDIO_STAT1);
	phy_write_mmd(phydev, MDIO_MMD_AN, MDIO_CTRL1, AN_CTRL_INIT);
}

static void setup_default_tecr(struct xgkr_params *xgkr)
{
#ifdef TECR_DEFAULT_RCW_SETUP
	struct tecr_params tecr;
	xgkr->srds->read_tecr_params(xgkr->reg_base, &tecr);
	xgkr->def_ratio_preq = tecr.ratio_preq;
	xgkr->def_ratio_pst1q = tecr.ratio_pst1q;
	xgkr->def_adpt_eq = tecr.adpt_eq;
	xgkr->def_amp_red = tecr.amp_red;
#else
	struct phy_device *phydev = xgkr->phydev;
	struct xgkr_phy_data *xgkr_inst = phydev->priv;

	switch (xgkr_inst->bp_mode)
	{
	case PHY_BACKPLANE_1000BASE_KX:
		dev_err(&phydev->mdio.dev, "Wrong call path for 1000Base-KX \n");
		break;

	case PHY_BACKPLANE_10GBASE_KR:
		xgkr->def_ratio_preq = RATIO_PREQ_10G;
		xgkr->def_ratio_pst1q = RATIO_PST1Q_10G;
		xgkr->def_adpt_eq = RATIO_EQ_10G;
		xgkr->def_amp_red = 0;
		break;

	case PHY_BACKPLANE_40GBASE_KR:
		xgkr->def_ratio_preq = RATIO_PREQ_40G;
		xgkr->def_ratio_pst1q = RATIO_PST1Q_40G;
		xgkr->def_adpt_eq = RATIO_EQ_40G;
		xgkr->def_amp_red = 0;
		break;
	}
#endif
	xgkr->stats.init_tecr0 = xgkr->srds->read_tecr0(xgkr->reg_base);

	//dbg_log_lane(xgkr, "setup_default_tecr: RATIO_PREQ = 0x%x, RATIO_PST1Q = 0x%x, ADPT_EQ = 0x%x",
	//		xgkr->def_ratio_preq, xgkr->def_ratio_pst1q, xgkr->def_adpt_eq);
}

static void reset_tecr(struct xgkr_params *xgkr)
{
	xgkr->ratio_preq = xgkr->def_ratio_preq;
	xgkr->ratio_pst1q = xgkr->def_ratio_pst1q;
	xgkr->adpt_eq = xgkr->def_adpt_eq;

	tune_tecr(xgkr, true);
}

static void init_xgkr(struct xgkr_params *xgkr, bool reset)
{
	dbg_log_lane(xgkr, "init_xgkr: reset = %s", reset ? "true" : "false");

	if (reset)
		reset_tecr(xgkr);

	tx_condition_init(&xgkr->tx_c);
	xgkr->state = DETECTING_LP;
	xgkr->an_acquired = false;

	xgkr->ld_update = 0;
	xgkr->prev_ld_update = 0;
	xgkr->prev_ld_last_nonhold_update = 0;
	xgkr->prev_alg_ld_update = 0;
	xgkr->lp_status = 0;
	xgkr->lp_last_nonzero_status = 0;
	xgkr->ld_status = 0;
	xgkr->bin_m1_state = BIN_INVALID;
	xgkr->bin_long_state = BIN_INVALID;
	xgkr->prev_bin_m1_state = BIN_INVALID;
	xgkr->prev_bin_long_state = BIN_INVALID;
	xgkr->move_back_prev = false;
	xgkr->move_back_cnt = 0;
	xgkr->move_back_lp_status = 0;
}

/**
 * force certain parameters setup from debugfs
 */
void force_kr_setup(struct xgkr_params *xgkr)
{
	dbg_log_lane(xgkr, "force_kr_setup:");

	xgkr->ratio_preq = xgkr->set_ratio_preq;
	xgkr->ratio_pst1q = xgkr->set_ratio_pst1q;
	xgkr->adpt_eq = xgkr->set_adpt_eq;

	tune_tecr(xgkr, false);

	dev_info(&xgkr->phydev->mdio.dev, "Forced KR setup on lane %d (Tx equalization: RATIO_PREQ = 0x%x, RATIO_PST1Q = 0x%x, ADPT_EQ = 0x%x)\n",
			xgkr->idx, xgkr->ratio_preq, xgkr->ratio_pst1q, xgkr->adpt_eq);
}

void force_amp_red(struct xgkr_params *xgkr)
{
	dbg_log_lane(xgkr, "force_amp_red:");

	xgkr->srds->set_amp_red(xgkr->reg_base, xgkr->set_amp_red);

	dev_info(&xgkr->phydev->mdio.dev, "Forced amp_red on lane %d: AMP_RED = 0x%x)\n", xgkr->idx, xgkr->set_amp_red);
}

void force_restart_training(struct xgkr_phy_data *xgkr_inst)
{
	int i;

	dbg_log_phy(xgkr_inst->xgkr[0].phydev, "force_restart_training: ");

	switch (xgkr_inst->bp_mode)
	{
	case PHY_BACKPLANE_1000BASE_KX:
		break;

	case PHY_BACKPLANE_10GBASE_KR:
		/* initializations on force restart: they must not be part of init_xgkr */
		xgkr_inst->xgkr[SINGLE_LANE].first_recv_init = false;

		init_xgkr(&xgkr_inst->xgkr[SINGLE_LANE], false);
		reset_lt(&xgkr_inst->xgkr[SINGLE_LANE]);

		setup_default_tecr(&xgkr_inst->xgkr[SINGLE_LANE]);

		/* start state machine*/
		start_xgkr_state_machine(&xgkr_inst->xgkr[SINGLE_LANE].xgkr_wk, XGKR_TIMEOUT_1);
		break;

	case PHY_BACKPLANE_40GBASE_KR:
		for (i = 0; i < xgkr_inst->phy_lanes; i++) {
			/* initializations on force restart: they must not be part of init_xgkr */
			xgkr_inst->xgkr[i].first_recv_init = false;

			init_xgkr(&xgkr_inst->xgkr[i], false);
			reset_lt(&xgkr_inst->xgkr[i]);

			setup_default_tecr(&xgkr_inst->xgkr[i]);
		}
		/* start state machine*/
		for (i = 0; i < xgkr_inst->phy_lanes; i++) {
			start_xgkr_state_machine(&xgkr_inst->xgkr[i].xgkr_wk, XGKR_TIMEOUT_1);
		}
		break;
	}
}

//------------------------------------------------------------------------------------------------------
//
//            NEW ALGORITHM Training for Remote Tx
//

static void process_BinM1(struct xgkr_params *xgkr)
{
	/**
	 * IF the latest state it was at MIN/MAX sometime in the past
	 * (ignore NOT_UPDATED values because of the <72.6.10.2.5 Coefficient update process> functionality)
	 * and still want to INC/DEC
	 * THEN do we are done with this module
	 * so
	 * use: lp_last_nonzero_status (instead of: lp_status)
	 * use: prev_alg_ld_update (instead of: prev_ld_update)
	 * 	    or maybe: prev_ld_last_nonhold_update...
	 */
	u32 status_com1 = (xgkr->lp_last_nonzero_status & COM1_MASK) >> COM1_SHIFT;
	u32 prev_ld_update = xgkr->prev_alg_ld_update;
	u32 prev_req_com1 = (xgkr->prev_alg_ld_update & COM1_MASK) >> COM1_SHIFT;
	u32 temp;

	if (xgkr->bin_m1_state == BIN_INVALID) {
		//TODO: invalid state: should never happen
		return;
	}

	if (xgkr->bin_m1_state == BIN_TOGGLE) {
		/* Toggle path */
		if (xgkr->prev_bin_m1_state == xgkr->bin_m1_state) {
			/* Hold C- */
			temp = 0;
			xgkr->ld_update = temp;
		} else {
			temp = 0;
			/* according to: v1.0 */
			/* If previous step moved C- repeat C- move */
			if (prev_req_com1 == INCREMENT || prev_req_com1 == DECREMENT)
				temp = prev_ld_update & COM1_MASK;
#if 0
			/* according to: v0.8 */
			/* If previous step moved C-/C0 repeat C-/C0 move */
			temp = (xgkr->prev_ld_update & COM1_MASK) | (xgkr->prev_ld_update & COZ_MASK);
#endif
			xgkr->ld_update = temp;
		}
	} else {
		if (xgkr->prev_bin_m1_state == BIN_TOGGLE) {
			temp = 0;
			/* according to: v1.0 */
			/* If previous step moved C- go back on C- */
			if (prev_req_com1 == INCREMENT)
				temp |= DECREMENT << COM1_SHIFT;
			if (prev_req_com1 == DECREMENT)
				temp |= INCREMENT << COM1_SHIFT;
#if 0
			/* according to: v0.8 */
			/* If previous step moved C0 go back on C0 */
			if (prev_req_coz == INCREMENT)
				temp |= DECREMENT << COZ_SHIFT;
			if (prev_req_coz == DECREMENT)
				temp |= INCREMENT << COZ_SHIFT;
#endif
			xgkr->ld_update = temp;
		} else {
			if (xgkr->prev_bin_m1_state == xgkr->bin_m1_state) {

				if (xgkr->bin_m1_state == BIN_LATE) {
					/* Late path */
					if (status_com1 == COE_MIN) {
						/* Hold C(-1) */
						temp = 0;
						xgkr->ld_update = temp;
					} else {
						/* request Decrement c(-1) */
						temp = DECREMENT << COM1_SHIFT;
						xgkr->ld_update = temp;
					}
				} else {
					/* Early path */
					if (status_com1 == COE_MAX) {
						/* Hold c(-1) */
						temp = 0;
						xgkr->ld_update = temp;
					} else {
						/* request Increment c(-1) */
						temp = INCREMENT << COM1_SHIFT;
						xgkr->ld_update = temp;
					}
				}
			} else {
				/* according to: v1.0 */
				if (xgkr->bin_m1_state == BIN_LATE) {
					/* request Decrement c(-1) */
					temp = DECREMENT << COM1_SHIFT;
					xgkr->ld_update = temp;
				} else {
					/* Hold C(-1) */
					temp = 0;
					xgkr->ld_update = temp;
				}

#if 0
				/* according to: v0.6 */
				/* If previous step moved C- go back on C- */
				temp = 0;
				if (prev_req_com1 == INCREMENT)
					temp |= DECREMENT << COM1_SHIFT;
				if (prev_req_com1 == DECREMENT)
					temp |= INCREMENT << COM1_SHIFT;

				if (xgkr->bin_m1_state == BIN_LATE) {
					/* request Decrement C(0) */
					temp |= DECREMENT << COZ_SHIFT;
				} else {
					/* request Increment C(0) */
					temp |= INCREMENT << COZ_SHIFT;
				}

				/* Request move on C- and C0 */
				xgkr->ld_update = temp;
#endif
			}
		}
	}

	/* Store current algorithm decision as previous alg ld_update for next step */
	xgkr->prev_alg_ld_update = xgkr->ld_update;

	dbg_log_lane(xgkr, "process_BinM1: xgkr->ld_update = 0x%08x", xgkr->ld_update);
}

static void process_BinLong(struct xgkr_params *xgkr)
{
	/**
	 * IF the latest state it was at MIN/MAX sometime in the past
	 * (ignore NOT_UPDATED values because of the <72.6.10.2.5 Coefficient update process> functionality)
	 * and still want to INC/DEC
	 * THEN do we are done with this module
	 * so
	 * use: lp_last_nonzero_status (instead of: lp_status)
	 * use: prev_alg_ld_update (instead of: prev_ld_update)
	 * 	    or maybe: prev_ld_last_nonhold_update...
	 */
	u32 status_cop1 = (xgkr->lp_last_nonzero_status & COP1_MASK) >> COP1_SHIFT;
	u32 status_coz = (xgkr->lp_last_nonzero_status & COZ_MASK) >> COZ_SHIFT;
	u32 prev_ld_update = xgkr->prev_alg_ld_update;
	u32 prev_req_cop1 = (xgkr->prev_alg_ld_update & COP1_MASK) >> COP1_SHIFT;
	u32 prev_req_coz = (xgkr->prev_alg_ld_update & COZ_MASK) >> COZ_SHIFT;
	u32 temp;

	if (xgkr->bin_long_state == BIN_INVALID) {
		//TODO: invalid state: should never happen
		return;
	}

	if (xgkr->bin_long_state == BIN_TOGGLE) {
		/* Toggle path */
		if (xgkr->prev_bin_long_state == xgkr->bin_long_state) {
			/* Hold C+ and C0 */
			temp = 0;
			xgkr->ld_update = temp;
		} else {
			temp = 0;
			/* If previous step moved C+/C0 repeat C+/C0 move */
			if (prev_req_cop1 == INCREMENT || prev_req_cop1 == DECREMENT ||
				prev_req_coz == INCREMENT || prev_req_coz == DECREMENT)
				temp = (prev_ld_update & COP1_MASK) | (prev_ld_update & COZ_MASK);
			xgkr->ld_update = temp;
		}
	} else {
		if (xgkr->prev_bin_long_state == BIN_TOGGLE) {
			/* If previous step moved C+/C0 go back on C+/C0 */
			temp = 0;
			if (prev_req_cop1 == INCREMENT)
				temp |= DECREMENT << COP1_SHIFT;
			if (prev_req_cop1 == DECREMENT)
				temp |= INCREMENT << COP1_SHIFT;
			if (prev_req_coz == INCREMENT)
				temp |= DECREMENT << COZ_SHIFT;
			if (prev_req_coz == DECREMENT)
				temp |= INCREMENT << COZ_SHIFT;

			xgkr->ld_update = temp;
		} else {
			if (xgkr->prev_bin_long_state == xgkr->bin_long_state) {

				if (xgkr->bin_long_state == BIN_LATE) {
					/* Late path (make edge earlier) */
					if (status_cop1 == COE_MIN) {
						if (status_coz == COE_MIN) {
							/* Hold C(0) */
							temp = 0;
							xgkr->ld_update = temp;
						} else {
							/* request Decrement c(0) */
							temp = DECREMENT << COZ_SHIFT;
							xgkr->ld_update = temp;
						}
					} else {
						/* request Decrement c(+1) */
						temp = DECREMENT << COP1_SHIFT;
						xgkr->ld_update = temp;
					}
				} else {
					/* Early path (make edge later) */
					if (status_cop1 == COE_MAX) {
						if (status_coz == COE_MAX) {
							/* Hold C(+1), C(0) */
							temp = 0;
							xgkr->ld_update = temp;
						} else {
							/* request Increment C(0) and Decrement c(+1) */
							temp = (INCREMENT << COZ_SHIFT) | (DECREMENT << COP1_SHIFT);
							xgkr->ld_update = temp;
						}
					} else {
						/* request Increment c(+1) */
						temp = INCREMENT << COP1_SHIFT;
						xgkr->ld_update = temp;
					}
				}
			} else {
				/* If previous step moved C+ go back on C+ */
				temp = 0;
				if (prev_req_cop1 == INCREMENT)
					temp |= DECREMENT << COP1_SHIFT;
				if (prev_req_cop1 == DECREMENT)
					temp |= INCREMENT << COP1_SHIFT;

				if (xgkr->bin_long_state == BIN_LATE) {
					/* request Decrement C(0) */
					temp = DECREMENT << COZ_SHIFT;
				} else {
					/* request Increment C(0) */
					temp = INCREMENT << COZ_SHIFT;
				}

				/* Request move on C+ and C0 */
				xgkr->ld_update = temp;
			}
		}
	}

	/* Store current algorithm decision as previous alg ld_update for next step */
	xgkr->prev_alg_ld_update = xgkr->ld_update;

	dbg_log_lane(xgkr, "process_BinLong: xgkr->ld_update = 0x%08x", xgkr->ld_update);
}

static bool is_ld_coe_update(struct xgkr_params *xgkr)
{
	if (xgkr->ld_update == 0) {
		/* All C are in Hold */
		// Send Hold requests:
		ld_coe_update(xgkr);
		return false;
	}

	/* Some C Inc/Dec requests */
	ld_coe_update(xgkr);
	return true;
}

static void move_back_to_prev(struct xgkr_params *xgkr)
{
	u32 prev_req_cop1 = (xgkr->prev_ld_last_nonhold_update & COP1_MASK) >> COP1_SHIFT;
	u32 prev_req_coz = (xgkr->prev_ld_last_nonhold_update & COZ_MASK) >> COZ_SHIFT;
	u32 prev_req_com1 = (xgkr->prev_ld_last_nonhold_update & COM1_MASK) >> COM1_SHIFT;
	u32 temp;

	/* Move back to previous C-, C0, C+ and HOLD */
	temp = 0;
	if (prev_req_cop1 == INCREMENT)
		temp |= DECREMENT << COP1_SHIFT;
	if (prev_req_cop1 == DECREMENT)
		temp |= INCREMENT << COP1_SHIFT;
	if (prev_req_coz == INCREMENT)
		temp |= DECREMENT << COZ_SHIFT;
	if (prev_req_coz == DECREMENT)
		temp |= INCREMENT << COZ_SHIFT;
	if (prev_req_com1 == INCREMENT)
		temp |= DECREMENT << COM1_SHIFT;
	if (prev_req_com1 == DECREMENT)
		temp |= INCREMENT << COM1_SHIFT;

	xgkr->ld_update = temp;
	ld_coe_update(xgkr);

	/* setup the procedure for sending move back to prev req until LP responds to it */
	xgkr->move_back_prev = true;
	xgkr->move_back_cnt = 0;
	xgkr->move_back_lp_status = 0;
	if (prev_req_cop1 == HOLD)
		xgkr->move_back_lp_status |= (COE_UPDATED << COP1_SHIFT);
	if (prev_req_coz == HOLD)
		xgkr->move_back_lp_status |= (COE_UPDATED << COZ_SHIFT);
	if (prev_req_com1 == HOLD)
		xgkr->move_back_lp_status |= (COE_UPDATED << COM1_SHIFT);
}

static void process_bad_state(struct xgkr_params *xgkr)
{
	bool lp_still_init, lp_still_preset;

	/* LP status still at Init/Preset
	 * if now LP status is Init/Preset
	 * OR now LP status is No update AND the last nonzero LP status was Init/Preset */
	if ((xgkr->lp_status & ALL_COE_MASK) == (COE_UPDATED << COP1_SHIFT | COE_UPDATED << COZ_SHIFT | COE_UPDATED << COM1_SHIFT)) {
		lp_still_init = true;
	} else {
		lp_still_init = (xgkr->lp_status & ALL_COE_MASK) == 0 &&
				((xgkr->lp_last_nonzero_status & ALL_COE_MASK) == (COE_UPDATED << COP1_SHIFT | COE_UPDATED << COZ_SHIFT | COE_UPDATED << COM1_SHIFT));
	}
	if ((xgkr->lp_status & ALL_COE_MASK) == (COE_MAX << COP1_SHIFT | COE_MAX << COZ_SHIFT | COE_MAX << COM1_SHIFT) ||
		(xgkr->lp_status & ALL_COE_MASK) == (COE_UPDATED << COP1_SHIFT | COE_UPDATED << COZ_SHIFT | COE_UPDATED << COM1_SHIFT)) {
		lp_still_preset = true;
	} else {
		lp_still_preset = (xgkr->lp_status & ALL_COE_MASK) == 0 &&
				((xgkr->lp_last_nonzero_status & ALL_COE_MASK) == (COE_MAX << COP1_SHIFT | COE_MAX << COZ_SHIFT | COE_MAX << COM1_SHIFT) ||
				 (xgkr->lp_last_nonzero_status & ALL_COE_MASK) == (COE_UPDATED << COP1_SHIFT | COE_UPDATED << COZ_SHIFT | COE_UPDATED << COM1_SHIFT));
	}

	dbg_log_lane(xgkr, "process_bad_state: lp_still_init = %s / lp_still_preset = %s",
			lp_still_init ? "true" : "false", lp_still_preset ? "true" : "false");

	if (lp_still_init) {
		/* Try Request Preset */
		xgkr->ld_update = PRESET_MASK;
		ld_coe_update(xgkr);
	} else if (lp_still_preset) {
		/* LT ERROR */

		//set lt_error flag to prevent reaching training state = TRAINED and resume training in case of LT error
		xgkr->lt_error = true;
		dev_err(&xgkr->phydev->mdio.dev, "LT Error: CDR_LOCK is zero on Preset! \n");
		dbg_log_lane(xgkr, "LT Error: CDR_LOCK is zero on Preset!");

		//TODO: Start link again ? or just issue Error ?
		/**
			Need recommendations for how to handle errors.
			Maybe suggest changing TECR0 to new INITIALIZE state and restart?
			Should only allow if NXP talking to NXP?
		 */
	} else {
		/* Move back to previous C-, C0, C+ and HOLD */
		move_back_to_prev(xgkr);
	}
}

static bool is_cdr_lock(struct xgkr_params *xgkr, bool retry)
{
	int i;
	if (xgkr->srds->is_cdr_lock(xgkr->reg_base))
		return true;

	/* CDR_LOCK = 0: Statistics are invalid */

	if (retry) {
		/* Try RX_RESET (Allow for 3 tries) */
		for (i = 0; i < 3; i++) {
			dbg_log_lane(xgkr, "is_cdr_lock: CDR_LOCK = 0: reset Rx lane and retry: %d", i + 1);
			xgkr->srds->reset_lane(xgkr->reg_base, LANE_RX);

			udelay(50);

			if (xgkr->srds->is_cdr_lock(xgkr->reg_base)) {
				dbg_log_lane(xgkr, "is_cdr_lock: cdr_lock recovered: exit with CDR_LOCK = 1");
				return true;
			}
		}
	}

	dbg_log_lane(xgkr, "is_cdr_lock: exit with CDR_LOCK = 0");
	return false;
}

#ifdef DEBUG_LOG_SNAPSHOTS
static void print_bin_snapshots(struct xgkr_params *xgkr, enum bin_type bin_type)
{
	s16 *bin_snapshot = NULL;
	char *str;

	/* select Bin snapshots */
	switch (bin_type) {
	case BIN_1:
		bin_snapshot = xgkr->bin1_snapshot;
		str = "BIN_1";
		break;
	case BIN_2:
		bin_snapshot = xgkr->bin2_snapshot;
		str = "BIN_2";
		break;
	case BIN_3:
		bin_snapshot = xgkr->bin3_snapshot;
		str = "BIN_3";
		break;
	case BIN_OFFSET:
		bin_snapshot = xgkr->bin_offset_snapshot;
		str = "BIN_OFFSET";
		break;
	case BIN_M1:
		bin_snapshot = xgkr->binM1_snapshot;
		str = "BIN_M1";
		break;
	case BIN_LONG:
		bin_snapshot = xgkr->binLong_snapshot;
		str = "BIN_LONG";
		break;
	default:
		/* invalid bin type */
		return;
	}
	if (bin_snapshot == NULL)
		return;

	dbg_log_lane(xgkr, "snapshot %s = %d, %d, %d, %d, %d, %d, %d, %d, %d, %d",
			str,
			bin_snapshot[0], bin_snapshot[1], bin_snapshot[2], bin_snapshot[3], bin_snapshot[4],
			bin_snapshot[5], bin_snapshot[6], bin_snapshot[7], bin_snapshot[8], bin_snapshot[9]);
}

static void print_gain_snapshots(struct xgkr_params *xgkr, char *str, u8 *snapshot)
{
	dbg_log_lane(xgkr, "snapshot %s = 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x",
			str,
			snapshot[0], snapshot[1], snapshot[2], snapshot[3], snapshot[4],
			snapshot[5], snapshot[6], snapshot[7], snapshot[8], snapshot[9]);
}
#endif //DEBUG_LOG_SNAPSHOTS

static bool try_collect_bin_snapshots(struct xgkr_params *xgkr, enum bin_type bin_type)
{
	int snp_size;
	s16 *bin_snapshot = NULL;

	/* collect Bin snapshots */
	switch (bin_type) {
	case BIN_1:
		bin_snapshot = xgkr->bin1_snapshot;
		break;
	case BIN_2:
		bin_snapshot = xgkr->bin2_snapshot;
		break;
	case BIN_3:
		bin_snapshot = xgkr->bin3_snapshot;
		break;
	case BIN_OFFSET:
		bin_snapshot = xgkr->bin_offset_snapshot;
		break;
	case BIN_M1:
		bin_snapshot = xgkr->binM1_snapshot;
		break;
	case BIN_LONG:
		bin_snapshot = xgkr->binLong_snapshot;
		break;
	default:
		/* invalid bin type */
		return false;
	}
	if (bin_snapshot == NULL)
		return false;

	snp_size = xgkr->srds->collect_bin_snapshots(bin_type, xgkr->reg_base, bin_snapshot);
	/* Check if snapshots collection failed: Timeout occurred */
	if (snp_size < BIN_SNAPSHOT_NUM)
		return false;

	/* if CDR_LOCK = 0: Statistics are invalid */
	if (!is_cdr_lock(xgkr, true)) {
		process_bad_state(xgkr);
		return false;
	}

	return true;
}

static char* get_bin_state_name(enum bin_state state)
{
	switch (state) {
	case BIN_INVALID:
		return "BIN_INVALID";
		break;
	case BIN_EARLY:
		return "BIN_EARLY";
		break;
	case BIN_TOGGLE:
		return "BIN_TOGGLE";
		break;
	case BIN_LATE:
		return "BIN_LATE";
		break;
	}
	return "BIN_INVALID";
}

static bool is_rx_happy(struct xgkr_params *xgkr)
{
	int i;
	bool rx_happy;
	s16 snapshot;
	u8 min_snp, max_snp;
	enum bin_state bin1_snapshot_state;
	enum bin_state bin2_snapshot_state;
	enum bin_state bin3_snapshot_state;
#ifdef ENABLE_HAPPY_COND_4_ON_SLIDE_4
	bool bin1_ok, bin2_ok;
	bool gaink2_1_ok, gaink2_2_ok, gaink3_1_ok, gaink3_2_ok;
	u8 midrange_low_gaink = xgkr->srds->get_midrange_low_gaink();
	u8 midrange_high_gaink = xgkr->srds->get_midrange_high_gaink();
	u8 gain_snp;
#endif //ENABLE_HAPPY_COND_4_ON_SLIDE_4
#ifdef ENABLE_LESS_HAPPY_COND_2
	bool rx_happy_21, rx_happy_22;
	u8 full_gaink = xgkr->srds->get_full_gaink2();
#endif //ENABLE_LESS_HAPPY_COND_2
#ifdef ENABLE_EVEN_LESS_HAPPY_COND_3
	bool rx_happy_31, rx_happy_32;
	bool is_ok;
#endif //ENABLE_EVEN_LESS_HAPPY_COND_3
#ifdef ENABLE_SEEMINGLY_HAPPY_COND_4
	bool rx_happy_41;
#endif //ENABLE_SEEMINGLY_HAPPY_COND_4

	/* Save statistics */
	//TODO: stat
	//xgkr->stats.median_gaink2 = ; - median_gaink2 MUST be removed from stats, and others must be introduced

	/*
	 * Checking Bins/Gains after LP has updated its TX
	 */

	/* CDR_LOCK must be 1 */
	if (!is_cdr_lock(xgkr, true)) {
		dbg_log_lane(xgkr, "is_rx_happy: Rx NOT happy: cond 1: CDR_LOCK must be 1");
		return false;
	}

	/*
	Offset Bin must NOT be 10 of the same value
	10G Lynx (T or LS): when LNmTCSR1.CDR_SEL = 0x3, LNmTCSR1.EQ_SNPBIN_DATA= Offset Bin
	28G Lynx (LX): when LNmTCSR1.CDR_SEL = 0x4, LNmRECR4.EQ_SNPBIN_DATA= Offset Bin
	*/
	rx_happy = false;
	snapshot = xgkr->bin_offset_snapshot[0];
	for (i = 0; i < BIN_SNAPSHOT_NUM; i++) {
		if (snapshot != xgkr->bin_offset_snapshot[i]) {
			rx_happy = true;
			break;
		}
	}
	if (!rx_happy) {
		dbg_log_lane(xgkr, "is_rx_happy: Rx NOT happy: cond 2: Offset Bin must NOT be 10 of the same value");
		return false;
	}

	/*
	Offset status must dither (+/-2) around MidRange value
	10G Lynx (T or LS): LNmRECR1.OSETSTAT = Offset Status
	MaxNeg = 0x0, MaxPos = 0x3F, MidRange: 0x10 - 0x2F
	28G Lynx (LX): LNmRECR4.OSETSTAT = Offset Status
	MaxNeg = 0x0, MaxPos = 0x3F, MidRange: 0x10 - 0x2F
	What we want to see is that the Offset has settled to a value somewhere between
	0x10 and 0x2F and that the series of snapshot values are +/-2 of the settled value.
	*/
	rx_happy = true;
	min_snp = xgkr->osestat_snapshot[0];
	max_snp = xgkr->osestat_snapshot[0];
	for (i = 0; i < BIN_SNAPSHOT_NUM; i++) {
		if (xgkr->osestat_snapshot[i] < OSESTAT_MIDRANGE_LOW ||
			xgkr->osestat_snapshot[i] > OSESTAT_MIDRANGE_HIGH) {
			rx_happy = false;
			break;
		}
		if (xgkr->osestat_snapshot[i] < min_snp)
			min_snp = xgkr->osestat_snapshot[i];
		if (xgkr->osestat_snapshot[i] > max_snp)
			max_snp = xgkr->osestat_snapshot[i];
	}
	if (max_snp - min_snp > 4)
		rx_happy = false;
	if (!rx_happy) {
		dbg_log_lane(xgkr, "is_rx_happy: Rx NOT happy: cond 3: Offset status must dither (+/-2) around MidRange value");
		return false;
	}

#ifdef ENABLE_HAPPY_COND_4_ON_SLIDE_4
	/*
	Bin1, Bin2, Bin3 must NOT be 10 of the same value, while GainK2/Gaink3 mid-range and not moving.

	//Thecla rewrite condition by email:
	Bin3 must NOT be 10 of the same value
	AND
		(Bin1 must NOT be 10 of the same value AND GainK2 is mid-range and not moving.
		OR
	 	Bin2 must NOT be 10 of the same value AND GainK3 is mid-range and not moving.)

	10G Lynx (T or LS):
	when LNmTCSR1.CDR_SEL = 0x0, LNmTCSR1.EQ_SNPBIN_DATA= Bin1
	GainK2 Status = LNmRECR1.GK2STAT, Min=0x0, Max=0xF, MidRange: 0x3 - 0xC
	when LNmTCSR1.CDR_SEL = 0x1, LNmTCSR1.EQ_SNPBIN_DATA= Bin2
	GainK3 Status = LNmRECR1.GK3STAT, , Min=0x0, Max=0xF, MidRange: 0x3 - 0xC
	when LNmTCSR1.CDR_SEL = 0x2, LNmTCSR1.EQ_SNPBIN_DATA= Bin3
	28G Lynx (LX):
	when LNmTCSR1.CDR_SEL = 0x0, LNmTCSR1.EQ_SNPBIN_DATA= Bin1
	GainK2 Status = LNmRECR1.GK2STAT, Min=0x0, Max=0x1F, MidRange: 0x7 - 0x17
	when LNmTCSR1.CDR_SEL = 0x1, LNmTCSR1.EQ_SNPBIN_DATA= Bin2
	GainK3 Status = LNmRECR1.GK3STAT, , Min=0x0, Max=0x1F, MidRange: 0x7 - 0x17
	when LNmTCSR1.CDR_SEL = 0x2, LNmTCSR1.EQ_SNPBIN_DATA= Bin3
	*/

	/* Bin3 must NOT be 10 of the same value */
	rx_happy = false;
	snapshot = xgkr->bin3_snapshot[0];
	for (i = 0; i < BIN_SNAPSHOT_NUM; i++) {
		if (snapshot != xgkr->bin3_snapshot[i]) {
			rx_happy = true;
			break;
		}
	}
	if (!rx_happy) {
		dbg_log_lane(xgkr, "is_rx_happy: Rx NOT happy: cond 6: Bin3 must NOT be 10 of the same value");
		return false;
	}

	/* Bin1 must NOT be 10 of the same value */
	bin1_ok = false;
	snapshot = xgkr->bin1_snapshot[0];
	for (i = 0; i < BIN_SNAPSHOT_NUM; i++) {
		if (snapshot != xgkr->bin1_snapshot[i]) {
			bin1_ok = true;
			break;
		}
	}
	/* Bin2 must NOT be 10 of the same value */
	bin2_ok = false;
	snapshot = xgkr->bin2_snapshot[0];
	for (i = 0; i < BIN_SNAPSHOT_NUM; i++) {
		if (snapshot != xgkr->bin2_snapshot[i]) {
			bin2_ok = true;
			break;
		}
	}

	//according to latest Thecla email: RE: BaseKR Training / Sent: Mon April 1, 5:47 PM
	/*
	Bin3 must NOT be 10 of the same value
	AND
	(
	Bin1 must NOT be 10 of the same value AND (GainK2 is mid-range AND NOT be 10 of the same value )
	OR
	Bin2 must NOT be 10 of the same value AND (GainK3 is mid-range AND NOT be 10 of the same value)
	)
	*/
	/* GainK2 is mid-range AND NOT be 10 of the same value */
	gaink2_1_ok = true;
	for (i = 0; i < BIN_SNAPSHOT_NUM; i++) {
		if (xgkr->gaink2_snapshot[i] < midrange_low_gaink ||
			xgkr->gaink2_snapshot[i] > midrange_high_gaink) {
			gaink2_1_ok = false;
			break;
		}
	}
	gaink2_2_ok = false;
	gain_snp = xgkr->gaink2_snapshot[0];
	for (i = 0; i < BIN_SNAPSHOT_NUM; i++) {
		/* gaink2 must NOT be 10 of the same value */
		if (gain_snp != xgkr->gaink2_snapshot[i]) {
			gaink2_2_ok = true;
			break;
		}
	}
	/* GainK3 is mid-range AND NOT be 10 of the same value */
	gaink3_1_ok = true;
	for (i = 0; i < BIN_SNAPSHOT_NUM; i++) {
		if (xgkr->gaink3_snapshot[i] < midrange_low_gaink ||
			xgkr->gaink3_snapshot[i] > midrange_high_gaink) {
			gaink3_1_ok = false;
			break;
		}
	}
	gaink3_2_ok = false;
	gain_snp = xgkr->gaink3_snapshot[0];
	for (i = 0; i < BIN_SNAPSHOT_NUM; i++) {
		/* gaink3 must NOT be 10 of the same value */
		if (gain_snp != xgkr->gaink3_snapshot[i]) {
			gaink3_2_ok = true;
			break;
		}
	}

	//according to latest Thecla email: Sent: Mon April 1, 5:47 PM
	/*
	Bin1 must NOT be 10 of the same value AND (GainK2 is mid-range AND NOT be 10 of the same value )
	OR
	Bin2 must NOT be 10 of the same value AND (GainK3 is mid-range AND NOT be 10 of the same value)
	*/
	rx_happy = ((bin1_ok && gaink2_1_ok && gaink2_2_ok) || (bin2_ok && gaink3_1_ok && gaink3_2_ok));
	if (!rx_happy) {
		dbg_log_lane(xgkr, "is_rx_happy: Rx NOT happy: cond 7 OR cond 8 ( bin1_ok = %d / gaink2_1_ok = %d / gaink2_2_ok = %d )",
				bin1_ok ? 1:0, gaink2_1_ok ? 1:0, gaink2_2_ok ? 1:0);
		return false;
	}
#endif //ENABLE_HAPPY_COND_4_ON_SLIDE_4

	/*
	The RX is happy if:
	 Bin1, Bin2, and Bin3 are toggling as defined on slide 0
	 Proceed to BinLong/BinM1 modules
	*/
	bin1_snapshot_state = xgkr->srds->get_bin_snapshots_state(xgkr->bin1_snapshot);
	bin2_snapshot_state = xgkr->srds->get_bin_snapshots_state(xgkr->bin2_snapshot);
	bin3_snapshot_state = xgkr->srds->get_bin_snapshots_state(xgkr->bin3_snapshot);

	rx_happy = (bin1_snapshot_state == BIN_TOGGLE &&
				bin2_snapshot_state == BIN_TOGGLE &&
				bin3_snapshot_state == BIN_TOGGLE);

	dbg_log_lane(xgkr, "is_rx_happy: Bin1 is %s", get_bin_state_name(bin1_snapshot_state));
	dbg_log_lane(xgkr, "is_rx_happy: Bin2 is %s", get_bin_state_name(bin2_snapshot_state));
	dbg_log_lane(xgkr, "is_rx_happy: Bin3 is %s", get_bin_state_name(bin3_snapshot_state));

	/* If Happy proceed to BinLong/BinM1 */
	if (rx_happy) {
		dbg_log_lane(xgkr, "is_rx_happy: exit with Rx is Happy, proceed to BinLong/BinM1");
		return true;
	}

#ifdef ENABLE_LESS_HAPPY_COND_2
	/*
	The RX is less happy if:
	 Bin1 is toggling,
	 Bin2 is pegged Early, Gaink3 stuck at 0x0 and Bin3 is pegged Late or
	 Bin2 is pegged Late, GainK3 stuck at 0xF (0x1F in 28G) and Bin3 is pegged Early
	 Put in parameter to select if this is a Happy Condition or not.
	 If Happy proceed to BinLong/BinM1
	*/
	rx_happy_21 = false;
	rx_happy_22 = false;
	if (bin1_snapshot_state == BIN_TOGGLE) {
		if (bin2_snapshot_state == BIN_EARLY && bin3_snapshot_state == BIN_LATE) {

			/* check if Gaink3 is stuck at 0x0 */
			rx_happy_21 = true;
			for (i = 0; i < BIN_SNAPSHOT_NUM; i++) {
				if (xgkr->gaink3_snapshot[i] != 0) {
					rx_happy_21 = false;
					break;
				}
			}
		}
		if (bin2_snapshot_state == BIN_LATE && bin3_snapshot_state == BIN_EARLY) {

			/* check if Gaink3 is stuck at full_gaink */
			rx_happy_22 = true;
			for (i = 0; i < BIN_SNAPSHOT_NUM; i++) {
				if (xgkr->gaink3_snapshot[i] != full_gaink) {
					rx_happy_22 = false;
					break;
				}
			}
		}
	}

	/* If RX is less Happy then proceed to BinLong/BinM1 */
	if (rx_happy_21 || rx_happy_22) {
		dbg_log_lane(xgkr, "is_rx_happy: exit with RX is less Happy 2, proceed to BinLong/BinM1");
		return true;
	}
#endif //ENABLE_LESS_HAPPY_COND_2

#ifdef ENABLE_EVEN_LESS_HAPPY_COND_3
	/*
	The RX is even less happy if:
	Bin1 is pegged Early, GainK2 stuck at 0x0 and Bin2 is pegged Late, GainK3 stuck at 0xF (0x1F in 28G) or
	Bin1 is pegged Late, GainK2 stuck at 0xF (0x1F in 28G) and Bin2 is pegged Early, GainK3 stuck at 0x0
	Put in parameter to select if this is a Happy Condition or not.
	If Happy proceed to BinLong/BinM1
	*/
	rx_happy_31 = false;
	rx_happy_32 = false;
	if (bin1_snapshot_state == BIN_EARLY && bin2_snapshot_state == BIN_LATE) {

		/* check if Gaink2 is stuck at 0x0 */
		is_ok = true;
		for (i = 0; i < BIN_SNAPSHOT_NUM; i++) {
			if (xgkr->gaink2_snapshot[i] != 0) {
				is_ok = false;
				break;
			}
		}
		if (is_ok) {
			/* check if Gaink3 is stuck at full_gaink */
			is_ok = true;
			for (i = 0; i < BIN_SNAPSHOT_NUM; i++) {
				if (xgkr->gaink3_snapshot[i] != full_gaink) {
					is_ok = false;
					break;
				}
			}
			if (is_ok)
				rx_happy_31 = true;
		}
	}
	if (bin1_snapshot_state == BIN_LATE && bin2_snapshot_state == BIN_EARLY) {

		/* check if Gaink2 is stuck at full_gaink */
		is_ok = true;
		for (i = 0; i < BIN_SNAPSHOT_NUM; i++) {
			if (xgkr->gaink2_snapshot[i] != full_gaink) {
				is_ok = false;
				break;
			}
		}
		if (is_ok) {
			/* check if Gaink3 is stuck at 0x0 */
			is_ok = true;
			for (i = 0; i < BIN_SNAPSHOT_NUM; i++) {
				if (xgkr->gaink3_snapshot[i] != 0) {
					is_ok = false;
					break;
				}
			}
			if (is_ok)
				rx_happy_32 = true;
		}
	}

	/* If Happy proceed to BinLong/BinM1 */
	if (rx_happy_31 || rx_happy_32) {
		dbg_log_lane(xgkr, "is_rx_happy: exit with RX is even less Happy 3, proceed to BinLong/BinM1");
		return true;
	}
#endif //ENABLE_EVEN_LESS_HAPPY_COND_3

#ifdef ENABLE_SEEMINGLY_HAPPY_COND_4
	/*
	The RX is ‘seemingly happy’ if:
	Bin1 always late for all 10 snapshots, GainK2 pegged at 0xF (0x1F in 28G) AND Bin2 and Bin3 are Toggling
	Put in parameter to select if this is a ‘Happy’ Condition or not.
	If ‘Happy’ proceed to BinLong/BinM1
	 */
	rx_happy_41 = false;
	if (bin1_snapshot_state == BIN_LATE && bin2_snapshot_state == BIN_TOGGLE && bin3_snapshot_state == BIN_TOGGLE) {
		/* check if Gaink2 is pegged at full_gaink */
		rx_happy_41 = true;
		for (i = 0; i < BIN_SNAPSHOT_NUM; i++) {
			if (xgkr->gaink2_snapshot[i] != full_gaink) {
				rx_happy_41 = false;
				break;
			}
		}
	}

	if (rx_happy_41) {
		dbg_log_lane(xgkr, "is_rx_happy: exit with RX is Seemingly Happy 4, proceed to BinLong/BinM1");
		return true;
	}
#endif //ENABLE_SEEMINGLY_HAPPY_COND_4

	dbg_log_lane(xgkr, "is_rx_happy: exit with Rx NOT happy: No happy condition met");
	return false;
}

/**
 * Train Remote Tx
 * This is the main routine for the KR Algorithm
 *
 */
static void train_remote_tx(struct xgkr_params *xgkr)
{
	struct tx_condition *tx_c = &xgkr->tx_c;
	//u32 median_gaink2;
	u32 prev_req_init, prev_req_preset;
	u32 prev_req_cop1, prev_req_coz, prev_req_com1;
	u32 status_cop1, status_coz, status_com1;
	unsigned long lp_resp_time;
	int snp_size;

	//dbg_log_lane(xgkr, "train_remote_tx");

	/* Start a new training step */
new_step:

	/* Store statistics for current step */
	xgkr->stats.training_cycles_remote_tx++;

	/* Store current state as previous state */
	xgkr->prev_ld_update = xgkr->ld_update;
	if ((xgkr->prev_ld_update & ALL_COE_MASK) != 0)
		xgkr->prev_ld_last_nonhold_update = xgkr->prev_ld_update;

	prev_req_init = xgkr->prev_ld_update & INIT_MASK;
	prev_req_preset = xgkr->prev_ld_update & PRESET_MASK;
	prev_req_cop1 = (xgkr->prev_ld_update & COP1_MASK) >> COP1_SHIFT;
	prev_req_coz = (xgkr->prev_ld_update & COZ_MASK) >> COZ_SHIFT;
	prev_req_com1 = (xgkr->prev_ld_update & COM1_MASK) >> COM1_SHIFT;

	/* Training Done condition */
	if (tx_c->bin_m1_stop && tx_c->bin_long_stop)
		tx_c->done_training = true;

	/* Check if Training is Done */
	if (tx_c->done_training) {

		tx_c->tx_complete = true;
		xgkr->ld_status |= RX_READY_MASK;
		ld_coe_status(xgkr);

		/* tell LP we are ready */
		xgkr_phy_write_mmd(xgkr, lt_MDIO_MMD,
			      lt_KR_PMD_STATUS, RX_STAT);

		dbg_log_lane(xgkr, "train_remote_tx: Training is Done");

		return;
	}

	/* Read LP Status */
	xgkr->lp_status = xgkr_phy_read_mmd(xgkr, lt_MDIO_MMD, lt_KR_LP_STATUS);

	if ((xgkr->lp_status & ALL_COE_MASK) != 0)
		xgkr->lp_last_nonzero_status = xgkr->lp_status;

	status_cop1 = (xgkr->lp_status & COP1_MASK) >> COP1_SHIFT;
	status_coz = (xgkr->lp_status & COZ_MASK) >> COZ_SHIFT;
	status_com1 = (xgkr->lp_status & COM1_MASK) >> COM1_SHIFT;

	/* tracing point */
	trace_xgkr_coe_status(xgkr, xgkr->lp_status, false);

	/* IEEE802.3-2008, 72.6.10.2.3.2
	 * we send initialize to the other side to ensure default settings
	 * for the LP. Naturally, we should do this only once.
	 */
	if (!tx_c->sent_init) {

		/* All status MUST be NOTUPDATED for INIT to be executed
		 * otherwise send HOLD first */
		if (status_cop1 == COE_NOTUPDATED && status_coz == COE_NOTUPDATED && status_com1 == COE_NOTUPDATED) {
			tx_c->sent_init = true;
			xgkr->ld_update = INIT_MASK;
			xgkr->req_ld_update_init_count = 1;
			xgkr->init_handshake_time = jiffies_to_msecs(jiffies);
			dbg_log_lane(xgkr, "train_remote_tx: sending ld_update = INIT");
		} else {
			//send HOLD before sending subsequent Init requests (not the very first Init sent)
			xgkr->ld_update = 0;
		}

		ld_coe_update(xgkr);
		return;
	}
	/* continue to sent init request until LP responds to init */
	if (prev_req_init) {
		if (xgkr->lp_status == 0) {
			//nothing to do here for now...
			//perhaps the partner board LP has not yet started
			//so continue to send INIT requests
			//this will happen in the next if condition anyway...
		}
		/* 72.6.10.2.3.2 Initialize
		 * The initialize control shall only be initially sent when all coefficient status fields indicate not_updated,
		 * and will then continue to be sent until no coefficient status field indicates not_updated.
		 *
		 */
		if (status_cop1 == COE_NOTUPDATED || status_coz == COE_NOTUPDATED || status_com1 == COE_NOTUPDATED) {
			xgkr->ld_update = INIT_MASK;
			ld_coe_update(xgkr);
			if (xgkr->req_ld_update_init_count == 1)
				dbg_log_lane(xgkr, "train_remote_tx: continue sending ld_update = INIT until LP responds to init: lp_status = 0x%08x", xgkr->lp_status);
			xgkr->req_ld_update_init_count++;
			return;
		} else {
			/* IEEE802.3-2008, 72.6.10.2.3.2
			 * We may clear INITIALIZE when no coefficients show NOT UPDATED.
			 */
			xgkr->ld_update &= ~INIT_MASK;
			lp_resp_time = jiffies_to_msecs(jiffies) - xgkr->init_handshake_time;
			if (!xgkr->first_recv_init) {
				/* Init handshake not done yet, but will be soon */
				xgkr->req_ld_update_init_count = 1;
				lp_resp_time = 0;
			}
			dbg_log_lane(xgkr, "train_remote_tx: Init Handshake: LP responded to INIT after %d ms and %d requests / lp_status = 0x%08x",
					lp_resp_time, xgkr->req_ld_update_init_count, xgkr->lp_status);

#if 0
			/* We measure training duration from remote_tx Init Handshake: shortest training time */
			xgkr->stats.lt_start = jiffies_to_msecs(jiffies);
#endif

			ld_coe_update(xgkr);
			return;
		}
	}

	/* 72.6.10.2.3.1 Preset
	 * The preset control shall only be initially sent when all coefficient status fields indicate not_updated,
	 * and will then continue to be sent until the status for all coefficients indicates updated or maximum
	 *
	 */
	/* IEEE802.3-2008, 72.6.10.2.3.1
	 * We may clear PRESET when all coefficients show UPDATED or MAX.
	 */
	//check if previous request was preset
	if (prev_req_preset) {
		if ((status_cop1 == COE_UPDATED || status_cop1 == COE_MAX) &&
		    (status_coz == COE_UPDATED || status_coz == COE_MAX) &&
		    (status_com1 == COE_UPDATED || status_com1 == COE_MAX)) {
			xgkr->ld_update &= ~PRESET_MASK;
		}
		else
		{
			/* All status MUST be NOTUPDATED for INIT to be executed
			 * otherwise send HOLD first */
			if (status_cop1 == COE_NOTUPDATED && status_coz == COE_NOTUPDATED && status_com1 == COE_NOTUPDATED) {
				xgkr->ld_update = PRESET_MASK;
			} else {
				//send HOLD before sending subsequent Preset requests
				xgkr->ld_update = 0;
			}
			ld_coe_update(xgkr);
			return;
		}
	}

	/* IEEE802.3-2008, 72.6.10.2.3.3
	 * We only request coefficient updates when no PRESET/INITIALIZE is
	 * pending. We also only request coefficient updates when the
	 * corresponding status is NOT UPDATED and nothing is pending.
	 */
	if (xgkr->ld_update & (PRESET_MASK | INIT_MASK)) {
		return;
	}

	/* continue to move back to previous request until LP responds to it
	 * Move back to previous C-, C0, C+ and HOLD
	 */
	if (xgkr->move_back_prev) {
		/* can exit from here only with: DONE Training */
		if (xgkr->move_back_cnt == TIMEOUT_MOVE_BACK_PREV) {
			tx_c->done_training = true;
			goto new_step;
		}
		xgkr->move_back_cnt++;

		if (status_cop1 == COE_UPDATED)
			xgkr->move_back_lp_status |= (COE_UPDATED << COP1_SHIFT);
		if (status_coz == COE_UPDATED)
			xgkr->move_back_lp_status |= (COE_UPDATED << COZ_SHIFT);
		if (status_com1 == COE_UPDATED)
			xgkr->move_back_lp_status |= (COE_UPDATED << COM1_SHIFT);

		if ((xgkr->move_back_lp_status & ALL_COE_MASK) == LP_STATUS_ALL_COE_UPDATED) {
			tx_c->done_training = true;
			goto new_step;
		}

		/* Move back to previous C-, C0, C+ */
		xgkr->ld_update = xgkr->prev_ld_update;
		ld_coe_update(xgkr);
		return;
	}

	/* 72.6.10.2.5 Coefficient update process
	 * Once the updated, maximum, or minimum state is reported it continues to be reported
	 * until a hold request is received, after which the status reverts to not_updated.
	 */

	/* IEEE802.3-2008, 72.6.10.2.3.3
	 * We set coefficient requests to HOLD when we get the information
	 * about any updates On clearing our prior response, we also update
	 * our internal status.
	 */

	/* send a Hold if want to send another INC same as previous and received status: NOTUPDATED
	 * 1. Continue to send prev REQ until receive status UPDATED
	 * 2. Continue to send HOLD until receive status NOTUPDATED
	 */

	/* 3. LP can remain stuck ~42 ms in reset Rx lane: so we should wait around ~50 ms and only after that issue Timeout error message */

	switch (prev_req_cop1)
	{
	case HOLD:
		//previous request was: HOLD
		if (status_cop1 == COE_NOTUPDATED) {
			/* All good: proceed to BinLong/BinM1 */
		} else {
			/* Continue to send the same request: (2.)
			 * Continue to send HOLD until receive status NOTUPDATED */
			if (xgkr->repeat_request_count >= TIMEOUT_REPEAT_REQUEST) {
				dev_err(&xgkr->phydev->mdio.dev, "REQ Timeout: Repeating HOLD C(+1) request without LP response timeout ! \n");
				dbg_log_lane(xgkr, "train_remote_tx: Repeating request without LP response reached timeout: %d times request resent: HOLD C(+1) !",
						xgkr->repeat_request_count);
				/* just continue: proceed again to BinLong/BinM1 */
			} else {
				/* Allow LP some time to respond and repeat request */
				msleep(2);
				/* Allow LP more time to respond, as the last chance, on the last time before issuing timeout error: (3.) */
				if (xgkr->repeat_request_count == TIMEOUT_REPEAT_REQUEST - 1) {
					dbg_log_lane(xgkr, "train_remote_tx: Repeating request without LP response: %d times request resent: HOLD C(+1) / waiting for LP before timeout...",
											xgkr->repeat_request_count);
					msleep(30);
				}
				xgkr->repeat_request_count++;
				ld_coe_update(xgkr);
				return;
			}
		}
		break;
	case INCREMENT:
	case DECREMENT:
		//previous request was: INC/DEC
		if (status_cop1 == COE_NOTUPDATED) {
			/* Continue to send the same request: (1.)
			 * Continue to send previous REQ until receive status UPDATED */
			if (xgkr->repeat_request_count >= TIMEOUT_REPEAT_REQUEST) {
				dev_err(&xgkr->phydev->mdio.dev, "REQ Timeout: Repeating C(+1) request without LP response timeout ! \n");
				dbg_log_lane(xgkr, "train_remote_tx: Repeating request without LP response reached timeout: %d times request resent: %s C(+1) !",
						xgkr->repeat_request_count, (prev_req_cop1 == INCREMENT) ? "INC" : "DEC");
				/* just continue: proceed again to BinLong/BinM1 */
			} else {
				/* Allow LP some time to respond and repeat request */
				msleep(2);
				/* Allow LP more time to respond, as the last chance, on the last time before issuing timeout error: (3.) */
				if (xgkr->repeat_request_count == TIMEOUT_REPEAT_REQUEST - 1) {
					dbg_log_lane(xgkr, "train_remote_tx: Repeating request without LP response: %d times request resent: %s C(+1) / waiting for LP before timeout...",
							xgkr->repeat_request_count, (prev_req_cop1 == INCREMENT) ? "INC" : "DEC");
					msleep(30);
				}
				xgkr->repeat_request_count++;
				ld_coe_update(xgkr);
				return;
			}
		} else {
			/* Sent HOLD (as in orig ver) because LP responded to this REQ */
			xgkr->ld_update &= ~COP1_MASK;
		}
		break;
	default:
		//previous request was: RESERVED: do nothing
		break;
	}

	switch (prev_req_coz)
	{
	case HOLD:
		//previous request was: HOLD
		if (status_coz == COE_NOTUPDATED) {
			/* All good: proceed to BinLong/BinM1 */
		} else {
			/* Continue to send the same request: (2.)
			 * Continue to send HOLD until receive status NOTUPDATED */
			if (xgkr->repeat_request_count >= TIMEOUT_REPEAT_REQUEST) {
				dev_err(&xgkr->phydev->mdio.dev, "REQ Timeout: Repeating HOLD C(0) request without LP response timeout ! \n");
				dbg_log_lane(xgkr, "train_remote_tx: Repeating request without LP response reached timeout: %d times request resent: HOLD C(0) !",
						xgkr->repeat_request_count);
				/* just continue: proceed again to BinLong/BinM1 */
			} else {
				/* Allow LP some time to respond and repeat request */
				msleep(2);
				/* Allow LP more time to respond, as the last chance, on the last time before issuing timeout error: (3.) */
				if (xgkr->repeat_request_count == TIMEOUT_REPEAT_REQUEST - 1) {
					dbg_log_lane(xgkr, "train_remote_tx: Repeating request without LP response: %d times request resent: HOLD C(0) / waiting for LP before timeout...",
							xgkr->repeat_request_count);
					msleep(30);
				}
				xgkr->repeat_request_count++;
				ld_coe_update(xgkr);
				return;
			}
		}
		break;
	case INCREMENT:
	case DECREMENT:
		//previous request was: INC/DEC
		if (status_coz == COE_NOTUPDATED) {
			/* Continue to send the same request: (1.)
			 * Continue to send previous REQ until receive status UPDATED */
			if (xgkr->repeat_request_count >= TIMEOUT_REPEAT_REQUEST) {
				dev_err(&xgkr->phydev->mdio.dev, "REQ Timeout: Repeating C(0) request without LP response timeout ! \n");
				dbg_log_lane(xgkr, "train_remote_tx: Repeating request without LP response reached timeout: %d times request resent: %s C(0) !",
						xgkr->repeat_request_count, (prev_req_coz == INCREMENT) ? "INC" : "DEC");
				/* just continue: proceed again to BinLong/BinM1 */
			} else {
				/* Allow LP some time to respond and repeat request */
				msleep(2);
				/* Allow LP more time to respond, as the last chance, on the last time before issuing timeout error: (3.) */
				if (xgkr->repeat_request_count == TIMEOUT_REPEAT_REQUEST - 1) {
					dbg_log_lane(xgkr, "train_remote_tx: Repeating request without LP response: %d times request resent: %s C(0) / waiting for LP before timeout...",
							xgkr->repeat_request_count, (prev_req_coz == INCREMENT) ? "INC" : "DEC");
					msleep(30);
				}
				xgkr->repeat_request_count++;
				ld_coe_update(xgkr);
				return;
			}
		} else {
			/* Sent HOLD (as in orig ver) because LP responded to this REQ */
			xgkr->ld_update &= ~COZ_MASK;
		}
		break;
	default:
		//previous request was: RESERVED: do nothing
		break;
	}

	switch (prev_req_com1)
	{
	case HOLD:
		//previous request was: HOLD
		if (status_com1 == COE_NOTUPDATED) {
			/* All good: proceed to BinLong/BinM1 */
		} else {
			/* Continue to send the same request: (2.)
			 * Continue to send HOLD until receive status NOTUPDATED */
			if (xgkr->repeat_request_count >= TIMEOUT_REPEAT_REQUEST) {
				dev_err(&xgkr->phydev->mdio.dev, "REQ Timeout: Repeating HOLD C(-1) request without LP response timeout ! \n");
				dbg_log_lane(xgkr, "train_remote_tx: Repeating request without LP response reached timeout: %d times request resent: HOLD C(-1) !",
						xgkr->repeat_request_count);
				/* just continue: proceed again to BinLong/BinM1 */
			} else {
				/* Allow LP some time to respond and repeat request */
				msleep(2);
				/* Allow LP more time to respond, as the last chance, on the last time before issuing timeout error: (3.) */
				if (xgkr->repeat_request_count == TIMEOUT_REPEAT_REQUEST - 1) {
					dbg_log_lane(xgkr, "train_remote_tx: Repeating request without LP response: %d times request resent: HOLD C(-1) / waiting for LP before timeout...",
							xgkr->repeat_request_count);
					msleep(30);
				}
				xgkr->repeat_request_count++;
				ld_coe_update(xgkr);
				return;
			}
		}
		break;
	case INCREMENT:
	case DECREMENT:
		//previous request was: INC/DEC
		if (status_com1 == COE_NOTUPDATED) {
			/* Continue to send the same request: (1.)
			 * Continue to send previous REQ until receive status UPDATED */
			if (xgkr->repeat_request_count >= TIMEOUT_REPEAT_REQUEST) {
				dev_err(&xgkr->phydev->mdio.dev, "REQ Timeout: Repeating C(-1) request without LP response timeout ! \n");
				dbg_log_lane(xgkr, "train_remote_tx: Repeating request without LP response reached timeout: %d times request resent: %s C(-1) !",
						xgkr->repeat_request_count, (prev_req_com1 == INCREMENT) ? "INC" : "DEC");
				/* just continue: proceed again to BinLong/BinM1 */
			} else {
				/* Allow LP some time to respond and repeat request */
				msleep(2);
				/* Allow LP more time to respond, as the last chance, on the last time before issuing timeout error: (3.) */
				if (xgkr->repeat_request_count == TIMEOUT_REPEAT_REQUEST - 1) {
					dbg_log_lane(xgkr, "train_remote_tx: Repeating request without LP response: %d times request resent: %s C(-1) / waiting for LP before timeout...",
							xgkr->repeat_request_count, (prev_req_com1 == INCREMENT) ? "INC" : "DEC");
					msleep(30);
				}
				xgkr->repeat_request_count++;
				ld_coe_update(xgkr);
				return;
			}
		} else {
			/* Sent HOLD (as in orig ver) because LP responded to this REQ */
			xgkr->ld_update &= ~COM1_MASK;
		}
		break;
	default:
		//previous request was: RESERVED: do nothing
		break;
	}

	/* Reset repeat request counter:
	 * must be after prev_req verifications above */
	xgkr->repeat_request_count = 0;

	if (xgkr->prev_ld_update != xgkr->ld_update) {
		ld_coe_update(xgkr);
		/* Redo these status checks and updates until we have no more
		 * changes, to speed up the overall process.
		 */
		return;
	}

	/* Do nothing if we have pending request.
	 */
	if (prev_req_cop1 || prev_req_coz || prev_req_com1) {
		return;
	}
	else if (xgkr->lp_status & ALL_COE_MASK) {
		/* No pending request but LP status was not reverted to
		 * not updated.
		 */
		return;
	}

	/* Initialize status for the current step */
	xgkr->lt_error = false;

	/* if CDR_LOCK = 0: Statistics are invalid */
	if (!is_cdr_lock(xgkr, true)) {
		process_bad_state(xgkr);
		return;
	}

	/* collect Bin snapshots */
	if (!try_collect_bin_snapshots(xgkr, BIN_1))
		return;
	if (!try_collect_bin_snapshots(xgkr, BIN_2))
		return;
	if (!try_collect_bin_snapshots(xgkr, BIN_3))
		return;
	if (!try_collect_bin_snapshots(xgkr, BIN_OFFSET))
		return;
	//TODO: optimization: Read BinM1/BIN_LONG only if it is needed for this step
	if (!try_collect_bin_snapshots(xgkr, BIN_M1))
		return;
	if (!try_collect_bin_snapshots(xgkr, BIN_LONG))
		return;

	/* collect Gains */
	snp_size = xgkr->srds->collect_gains(xgkr->reg_base, xgkr->gaink2_snapshot, xgkr->gaink3_snapshot, xgkr->osestat_snapshot);
	/* Check if snapshots collection failed: Timeout occurred */
	if (snp_size < BIN_SNAPSHOT_NUM)
		return;

	/* if CDR_LOCK = 0: Statistics are invalid */
	if (!is_cdr_lock(xgkr, true)) {
		process_bad_state(xgkr);
		return;
	}

#ifdef DEBUG_LOG_SNAPSHOTS
	//Debug print of snapshots:
	print_bin_snapshots(xgkr, BIN_1);
	print_bin_snapshots(xgkr, BIN_2);
	print_bin_snapshots(xgkr, BIN_3);
	print_bin_snapshots(xgkr, BIN_OFFSET);
	print_bin_snapshots(xgkr, BIN_M1);
	print_bin_snapshots(xgkr, BIN_LONG);

	print_gain_snapshots(xgkr, "gaink2", xgkr->gaink2_snapshot);
	print_gain_snapshots(xgkr, "gaink3", xgkr->gaink3_snapshot);
	print_gain_snapshots(xgkr, "osestat", xgkr->osestat_snapshot);
#endif //DEBUG_LOG_SNAPSHOTS

	/* tracing points */
	trace_xgkr_bin_snapshots(xgkr, "BIN_1", xgkr->bin1_snapshot);
	trace_xgkr_bin_snapshots(xgkr, "BIN_2", xgkr->bin2_snapshot);
	trace_xgkr_bin_snapshots(xgkr, "BIN_3", xgkr->bin3_snapshot);
	trace_xgkr_bin_snapshots(xgkr, "BIN_OFFSET", xgkr->bin_offset_snapshot);
	trace_xgkr_bin_snapshots(xgkr, "BIN_M1", xgkr->binM1_snapshot);
	trace_xgkr_bin_snapshots(xgkr, "BIN_LONG", xgkr->binLong_snapshot);

	/* tracing points */
	trace_xgkr_gain_snapshots(xgkr, "gaink2", xgkr->gaink2_snapshot);
	trace_xgkr_gain_snapshots(xgkr, "gaink3", xgkr->gaink3_snapshot);
	trace_xgkr_gain_snapshots(xgkr, "osestat", xgkr->osestat_snapshot);

	/* Check Bins and Gains */
	if (!is_rx_happy(xgkr)) {
		dbg_log_lane(xgkr, "train_remote_tx: is_rx_happy = false");
		process_bad_state(xgkr);
		return;
	}
	//dbg_log_lane(xgkr, "train_remote_tx: is_rx_happy = true");

	/* Move to BinLong/BinM1 modules */

	/* Store current state as previous state */
	xgkr->prev_bin_m1_state = xgkr->bin_m1_state;
	xgkr->prev_bin_long_state = xgkr->bin_long_state;

	xgkr->bin_m1_state = xgkr->srds->get_bin_snapshots_state(xgkr->binM1_snapshot);
	if (xgkr->bin_m1_state == BIN_INVALID) {
		//TODO: invalid state: should never happen
		return;
	}

	xgkr->bin_long_state = xgkr->srds->get_bin_snapshots_state(xgkr->binLong_snapshot);
	if (xgkr->bin_long_state == BIN_INVALID) {
		//TODO: invalid state: should never happen
		return;
	}


#ifdef BIN_MODULES_ORDER_BINLONG_BINM1
	/* Bin Modules order:  We try to finish BinLong before we do BinM1 (as used by the Old algorithm) */
	if (!tx_c->bin_long_stop) {

		process_BinLong(xgkr);

		if (is_ld_coe_update(xgkr)) {
			/* Some C Inc/Dec request was sent */
			tx_c->long_min_max_cnt = 0;
		} else {
			/* All C are in Hold */
			tx_c->long_min_max_cnt++;
			if (tx_c->long_min_max_cnt >= TIMEOUT_LONG)
				tx_c->bin_long_stop = true;
		}
		return;
	}

	/* Start with BinM1 module, decide on movement of preq, ask for movement */
	if (!tx_c->bin_m1_stop) {

		process_BinM1(xgkr);

		if (is_ld_coe_update(xgkr)) {
			/* Some C Inc/Dec request was sent */
			tx_c->m1_min_max_cnt = 0;
		} else {
			/* All C are in Hold */
			tx_c->m1_min_max_cnt++;
			if (tx_c->m1_min_max_cnt >= TIMEOUT_M1)
				tx_c->bin_m1_stop = true;
		}
		return;
	}
#else //BIN_MODULES_ORDER_BINLONG_BINM1
	/* Bin Modules order:  BinM1 before BinLong */
	/* Start with BinM1 module, decide on movement of preq, ask for movement */
	if (!tx_c->bin_m1_stop) {

		process_BinM1(xgkr);

		if (is_ld_coe_update(xgkr)) {
			/* Some C Inc/Dec request was sent */
			tx_c->m1_min_max_cnt = 0;
		} else {
			/* All C are in Hold */
			tx_c->m1_min_max_cnt++;
			if (tx_c->m1_min_max_cnt >= TIMEOUT_M1)
				tx_c->bin_m1_stop = true;
		}
#if 0
		//can be done this way ?
		if (xgkr->prev_ld_update != xgkr->ld_update) {
			ld_coe_update(xgkr);
			/* Redo these status checks and updates until we have no more
			 * changes, to speed up the overall process.
			 */
			goto new_step;
		}
#endif
		return;
	}

	/* Once BinM1 is happy then move onto tuning BinLong */
	if (!tx_c->bin_long_stop) {

		process_BinLong(xgkr);

		if (is_ld_coe_update(xgkr)) {
			/* Some C Inc/Dec request was sent */
			tx_c->long_min_max_cnt = 0;
		} else {
			/* All C are in Hold */
			tx_c->long_min_max_cnt++;
			if (tx_c->long_min_max_cnt >= TIMEOUT_LONG)
				tx_c->bin_long_stop = true;
		}
		return;
	}

#endif //#if BIN_MODULES_ORDER_BINLONG_BINM1

	/* All C are in Hold and both Bins are stopped - So the Training is done */
	if (tx_c->bin_m1_stop && tx_c->bin_long_stop) {
		tx_c->done_training = true;
		goto new_step;
	}
}

//
//            NEW ALGORITHM Training for Remote Tx
//
//------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------
//
//            Training for Local Tx
//

static void initialize(struct xgkr_params *xgkr)
{
	dbg_log_lane(xgkr, "initialize:");
	reset_tecr(xgkr);

	xgkr->ld_status &= ~ALL_COE_MASK;
	xgkr->ld_status |= COE_UPDATED << COP1_SHIFT |
			   COE_UPDATED << COZ_SHIFT |
			   COE_UPDATED << COM1_SHIFT;

	ld_coe_status(xgkr);

	xgkr->stats.ld_init_count++;
}

static void preset(struct xgkr_params *xgkr)
{
	dbg_log_lane(xgkr, "preset:");

	/* Preset as defined by: IEEE 802.3, sub-clause 72.6.10.2.3.1
	 * These are all MAX values from the IEEE802.3 perspective.
	 */
	xgkr->ratio_pst1q = POST_COE_MAX;
	xgkr->adpt_eq = ZERO_COE_MAX;
	xgkr->ratio_preq = PRE_COE_MAX;

	tune_tecr(xgkr, true);

	xgkr->ld_status &= ~ALL_COE_MASK;
	xgkr->ld_status |= COE_MAX << COP1_SHIFT |
			   COE_MAX << COZ_SHIFT |
			   COE_MAX << COM1_SHIFT;

	ld_coe_status(xgkr);

	xgkr->stats.ld_preset_count++;
}

/* Coefficient values have hardware restrictions */
static int is_ld_valid(struct xgkr_params *xgkr, u32 *ld_coe)
{
	u32 ratio_pst1q = ld_coe[0];
	u32 adpt_eq = ld_coe[1];
	u32 ratio_preq = ld_coe[2];

	dbg_log_lane(xgkr, "is_ld_valid: ratio_preq = %d, adpt_eq = %d, ratio_pst1q = %d", ratio_preq, adpt_eq, ratio_pst1q);

	/*
	Basic HW restrictions:
	General function of the equalization as described in
	section 4.5.1.5.3 of the 10G Lynx Creation Guide.
	This general function is used for PCI-EXP.
	 */
	if (adpt_eq > ZERO_COE_MAX)
		return 0;

	if (adpt_eq < ZERO_COE_MIN)
		return 0;

	if (ratio_preq > PRE_COE_MIN)
		return 0;

	if (ratio_pst1q > POST_COE_MIN)
		return 0;

	if ((ratio_preq + ratio_pst1q + 20) > adpt_eq)
		return 0;

	/*
	 Additional HW restrictions:
	 Section 5.3.1 10GBaseKR Transmit Adaptive Equalization Control
	 additional restrictions set down by the 802.3 specification Clause 72,
	 specifically 72.7.1.11 Transmitter output waveform requirements

	 Maintaining the following relationships limit the transmit equalization
	 to reasonable levels compliant with the 10GBaseKR specification

	 These restrictions are visually documented in trac ticket #900 'KR Training' comment 16:
	 http://trac.nxp.com/lx2160/ticket/900#comment:16

	 1. 6'd26 <= lnx_(m)_tx_ratio_preq[3:0] + lnx_(m)_tx_adpt_eq[5:0] + lnx_(m)_tx_ratio_post1q[4:0] <= 6'd48
	 2. 4'b0000 <= lnx_(m)_tx_ratio_preq[3:0] <= 4'b1000
	 3. 5'b0_0000 <= lnx_(m)_tx_ratio_post1q[4:0] <= 5'b1_0000
	 4. 6'b01_1010 <= lnx_(m)_tx_adpt_eq[5:0] <= 6'b11_0000
	 5. lnx_(m)_tx_ratio_post1q[4:0] >= lnx_(m)_tx_ratio_preq[3:0]
	 6.
	 	 ( lnx_(m)_tx_adpt_eq[5:0] + lnx_(m)_tx_ratio_preq[3:0] + lnx_(m)_tx_ratio_post1q[4:0] ) /
	 	 ( lnx_(m)_tx_adpt_eq[5:0] - lnx_(m)_tx_ratio_preq[3:0] - lnx_(m)_tx_ratio_post1q[4:0] ) < 4.25
	 */
	/*
	 1. 6'd26 <= lnx_(m)_tx_ratio_preq[3:0] + lnx_(m)_tx_adpt_eq[5:0] + lnx_(m)_tx_ratio_post1q[4:0] <= 6'd48
	 */
	if ((ratio_preq + ratio_pst1q + adpt_eq) < 26)
		return 0;
	if ((ratio_preq + ratio_pst1q + adpt_eq) > 48)
		return 0;
	/*
	 2. 4'b0000 <= lnx_(m)_tx_ratio_preq[3:0] <= 4'b1000
	 */
	if (ratio_preq > 0x8)
		return 0;
	/*
	 3. 5'b0_0000 <= lnx_(m)_tx_ratio_post1q[4:0] <= 5'b1_0000
	 */
	if (ratio_pst1q > 0x10)
		return 0;
	/*
	 4. 6'b01_1010 <= lnx_(m)_tx_adpt_eq[5:0] <= 6'b11_0000
	 */
	if (adpt_eq < 0x1A)
		return 0;
	if (adpt_eq > 0x30)
		return 0;
	/*
	 5. lnx_(m)_tx_ratio_post1q[4:0] >= lnx_(m)_tx_ratio_preq[3:0]
	 */
	if (ratio_pst1q < ratio_preq)
		return 0;
	/*
	 6.
	 ( lnx_(m)_tx_adpt_eq[5:0] + lnx_(m)_tx_ratio_preq[3:0] + lnx_(m)_tx_ratio_post1q[4:0] ) /
	 ( lnx_(m)_tx_adpt_eq[5:0] - lnx_(m)_tx_ratio_preq[3:0] - lnx_(m)_tx_ratio_post1q[4:0] ) < 4.25 = 17/4
	 */
	if (((ratio_pst1q + adpt_eq + ratio_preq) * 4) >=
	    ((adpt_eq - ratio_pst1q - ratio_preq) * 17))
		return 0;

	return 1;
}

#ifdef ALLOWED_VALUES_TABLE
static int is_value_allowed(const u32 *val_table, u32 val)
{
	/* consider ALL values and don't skip any */
	return 1;
/*
	int i;

	for (i = 0;; i++) {
		if (*(val_table + i) == VAL_INVALID)
			return 0;
		if (*(val_table + i) == val)
			return 1;
	}
	return 0;
*/
}
#endif //ALLOWED_VALUES_TABLE

static char* get_field_name(enum coe_field field)
{
	switch (field) {
	case COE_COP1:
		return "C(+1)";
		break;
	case COE_COZ:
		return "C(0)";
		break;
	case COE_COM:
		return "C(-1)";
		break;
	}
	return "INVALID";
}

static enum coe_update inc_dec(struct xgkr_params *xgkr, enum coe_field field, int request)
{
	u32 ld_limit[3], ld_coe[3], step[3];

	ld_coe[0] = xgkr->ratio_pst1q;
	ld_coe[1] = xgkr->adpt_eq;
	ld_coe[2] = xgkr->ratio_preq;

	step[0] = -1;
	step[1] = +1;
	step[2] = -1;

	/* 72.6.10.2.5 Coefficient update process
	 * Upon execution of a received increment or decrement request, the status is reported as updated, maximum, or minimum.
	 */

	switch (request) {
	case INCREMENT:
		ld_limit[0] = POST_COE_MAX;
		ld_limit[1] = ZERO_COE_MAX;
		ld_limit[2] = PRE_COE_MAX;
		if (ld_coe[field] != ld_limit[field]) {
			ld_coe[field] += step[field];
			xgkr->stats.inc_coe_count[field]++;

			dbg_log_lane(xgkr, "inc_dec: INC performed on %s", get_field_name(field));
		}
		else {
			dbg_log_lane(xgkr, "inc_dec: INC failed: COE_MAX limit reached on %s", get_field_name(field));
			/* MAX */
			return COE_MAX;
		}
		break;
	case DECREMENT:
		ld_limit[0] = POST_COE_MIN;
		ld_limit[1] = ZERO_COE_MIN;
		ld_limit[2] = PRE_COE_MIN;
		if (ld_coe[field] != ld_limit[field]) {
			ld_coe[field] -= step[field];
			xgkr->stats.dec_coe_count[field]++;

			dbg_log_lane(xgkr, "inc_dec: DEC performed on %s", get_field_name(field));
		}
		else {
			dbg_log_lane(xgkr, "inc_dec: DEC failed: COE_MIN limit reached on %s", get_field_name(field));
			/* MIN */
			return COE_MIN;
		}
		break;
	default:
		break;
	}

	if (is_ld_valid(xgkr, ld_coe)) {
		/* accept new ld */
		xgkr->ratio_pst1q = ld_coe[0];
		xgkr->adpt_eq = ld_coe[1];
		xgkr->ratio_preq = ld_coe[2];

#ifdef ALLOWED_VALUES_TABLE
		/* only some values for preq and pst1q can be used.
		 * for preq: 0x0, 0x1, 0x3, 0x5, 0x7, 0x9, 0xb, 0xc.
		 * for pst1q: 0x0, 0x1, 0x3, 0x5, 0x7, 0x9, 0xb, 0xd, 0xf, 0x10.
		 */
		if (!is_value_allowed((const u32 *)&preq_table, ld_coe[2])) {
			dbg_log_lane(xgkr, "inc_dec: COE_UPDATED: skipped preq invalid value = %d", ld_coe[2]);
			/* UPDATED */
			return COE_UPDATED;
		}

		if (!is_value_allowed((const u32 *)&pst1q_table, ld_coe[0])) {
			dbg_log_lane(xgkr, "inc_dec: COE_UPDATED: skipped pst1q invalid value = %d", ld_coe[0]);
			/* UPDATED */
			return COE_UPDATED;
		}
#endif //ALLOWED_VALUES_TABLE

		dbg_log_lane(xgkr, "inc_dec: tuning tecr for updating %s to valid value = %d", get_field_name(field), ld_coe[field]);
		tune_tecr(xgkr, false);

	} else {
		if (request == DECREMENT) {
			dbg_log_lane(xgkr, "inc_dec: COE_MIN: HW restriction on %s", get_field_name(field));
			/* MIN */
			return COE_MIN;
		}
		if (request == INCREMENT) {
			dbg_log_lane(xgkr, "inc_dec: COE_MAX: HW restriction on %s", get_field_name(field));
			/* MAX */
			return COE_MAX;
		}
	}

	/* UPDATED */
	return COE_UPDATED;
}

static void min_max_updated(struct xgkr_params *xgkr, enum coe_field field, enum coe_update cs)
{
	u32 mask, val;
	u32 ld_cs = cs;

	if (cs == COE_INV)
		return;

	switch (field) {
	case COE_COP1:
		mask = COP1_MASK;
		val = ld_cs << COP1_SHIFT;
		break;
	case COE_COZ:
		mask = COZ_MASK;
		val = ld_cs << COZ_SHIFT;
		break;
	case COE_COM:
		mask = COM1_MASK;
		val = ld_cs << COM1_SHIFT;
		break;
	default:
		return;
	}

	xgkr->ld_status &= ~mask;
	xgkr->ld_status |= val;

	dbg_log_lane(xgkr, "min_max_updated: ld_status update on field = %s / ld_cs = %d / xgkr->ld_status = 0x%08x", get_field_name(field), ld_cs, xgkr->ld_status);
}

static void check_request(struct xgkr_params *xgkr, int request)
{
	int cop1_req, coz_req, com_req;
	int old_status;
	enum coe_update cu = COE_INV;

	cop1_req = (request & COP1_MASK) >> COP1_SHIFT;
	coz_req = (request & COZ_MASK) >> COZ_SHIFT;
	com_req = (request & COM1_MASK) >> COM1_SHIFT;

	/* IEEE802.3-2008, 72.6.10.2.5
	 * Ensure we only act on INCREMENT/DECREMENT when we are in NOT UPDATED
	 *
	 * 72.6.10.2.5 Coefficient update process
	 * An increment or decrement request will only be acted upon when the state of the tap is not_updated.
	 */
	old_status = xgkr->ld_status;

	if (cop1_req && !(xgkr->ld_status & COP1_MASK)) {
		dbg_log_lane(xgkr, "check_request: C+ INC/DEC recv request: 0x%08x / xgkr->ld_status = 0x%08x", request, xgkr->ld_status);
		cu = inc_dec(xgkr, COE_COP1, cop1_req);
		min_max_updated(xgkr, COE_COP1, cu);
	}

	if (coz_req && !(xgkr->ld_status & COZ_MASK)) {
		dbg_log_lane(xgkr, "check_request: C0 INC/DEC recv request: 0x%08x / xgkr->ld_status = 0x%08x", request, xgkr->ld_status);
		cu = inc_dec(xgkr, COE_COZ, coz_req);
		min_max_updated(xgkr, COE_COZ, cu);
	}

	if (com_req && !(xgkr->ld_status & COM1_MASK)) {
		dbg_log_lane(xgkr, "check_request: C- INC/DEC recv request: 0x%08x / xgkr->ld_status = 0x%08x", request, xgkr->ld_status);
		cu = inc_dec(xgkr, COE_COM, com_req);
		min_max_updated(xgkr, COE_COM, cu);
	}

	if (cu == COE_UPDATED)
		xgkr->stats.coe_updates_from_lp++;

	if (old_status != xgkr->ld_status)
		ld_coe_status(xgkr);
}

static void train_local_tx(struct xgkr_params *xgkr)
{
	int request, old_ld_status;

	xgkr->stats.training_cycles_local_tx++;

	/* get request from LP */
	request = xgkr_phy_read_mmd(xgkr, lt_MDIO_MMD, lt_KR_LP_CU) & LD_ALL_MASK;

	/* tracing point */
	trace_xgkr_coe_update(xgkr, request, false);

	old_ld_status = xgkr->ld_status;

	/* IEEE802.3-2008, 72.6.10.2.5
	 * Ensure we always go to NOT UDPATED for status reporting in
	 * response to HOLD requests.
	 * IEEE802.3-2008, 72.6.10.2.3.1/2
	 * ... but only if PRESET/INITIALIZE are not active to ensure
	 * we keep status until they are released.
	 *
	 * 72.6.10.2.5 Coefficient update process
	 * Once the updated, maximum, or minimum state is reported it continues to be reported
	 * until a hold request is received, after which the status reverts to not_updated.
	 */
	if (!(request & (PRESET_MASK | INIT_MASK))) {
		/* Reset status on HOLD request */
		if (!(request & COP1_MASK)) {
			xgkr->ld_status &= ~COP1_MASK;
			//dbg_log_lane(xgkr, "train_local_tx: clean xgkr->ld_status on C+ / xgkr->ld_status = 0x%08x", xgkr->ld_status);
		}

		if (!(request & COZ_MASK)) {
			xgkr->ld_status &= ~COZ_MASK;
			//dbg_log_lane(xgkr, "train_local_tx: clean xgkr->ld_status on C0 / xgkr->ld_status = 0x%08x", xgkr->ld_status);
		}

		if (!(request & COM1_MASK)) {
			xgkr->ld_status &= ~COM1_MASK;
			//dbg_log_lane(xgkr, "train_local_tx: clean xgkr->ld_status on C- / xgkr->ld_status = 0x%08x", xgkr->ld_status);
		}

		ld_coe_status(xgkr);
	}

	/* IEEE802.3-2008, 72.6.10.2.3.1/2
	 * only act on PRESET/INITIALIZE if all status is NOT UPDATED.
	 */
	if (request & (PRESET_MASK | INIT_MASK)) {
		if (!(xgkr->ld_status & ALL_COE_MASK)) {
			if (request & PRESET_MASK)
				preset(xgkr);

			if (request & INIT_MASK) {
				if (!xgkr->first_recv_init) {
					xgkr->first_recv_init = true;
					/* Init requests must be counted from initial handshake */
					xgkr->req_ld_update_init_count = 1;
					xgkr->init_handshake_time = jiffies_to_msecs(jiffies);
					dbg_log_lane(xgkr, "train_local_tx: Init Handshake: first INIT received from LP");

					/* We measure training duration from initial handshake on INIT: intermediate training time */
					xgkr->stats.lt_start = jiffies_to_msecs(jiffies);
				}
				initialize(xgkr);
			}
		} else {
			/**
			 * Inform the partner about current ld status
			 * which should be: ALL UPDATED for INIT  and  ALL MAX for PRESET
			 */
			ld_coe_status(xgkr);
		}
	}

	/* LP Coefficient are not in HOLD */
	if (request & ALL_COE_MASK)
		check_request(xgkr, request & ALL_COE_MASK);
}

//
//            Training for Local Tx
//
//------------------------------------------------------------------------------------------------------

static void xgkr_start_train_step(struct xgkr_params *xgkr)
{
	struct phy_device *phydev = xgkr->phydev;
	struct xgkr_phy_data *xgkr_inst = phydev->priv;
	struct tx_condition *tx_c = &xgkr->tx_c;
	int val = 0, i, j;
	int lt_state;
	unsigned long dead_line;
	int lp_rx_ready, tx_training_complete;
	u32 lt_timeout = 500;
	bool bTimeout;

	/* check if training algorithm is disabled on this lane */
	if (xgkr->training_disabled)
		return;

#if 0
	/* We measure training duration from the first step
	 * (once the link was determined to be KR by Backplane Ethernet status register: BP_stat_KR_negotiated)
	 * but also includes the time delay between boards: longest training time */
	/* training duration should be measured from INIT handshake for shorter training time
	 * to remove the time delay between boards */
	if (xgkr->stats.training_steps == 0)
		xgkr->stats.lt_start = jiffies_to_msecs(jiffies);
#endif

	xgkr->stats.training_steps++;

	start_lt(xgkr);
	
	if (xgkr_inst->bp_mode == PHY_BACKPLANE_40GBASE_KR) {
		lt_timeout = 2000;
	}

	for (i = 0; i < 2;) {
		
		dead_line = jiffies + msecs_to_jiffies(lt_timeout);
		bTimeout = true;
		while (time_before(jiffies, dead_line)) {

			val = xgkr_phy_read_mmd(xgkr, lt_MDIO_MMD,
					   lt_KR_PMD_STATUS);

			if (val & TRAIN_FAIL) {
				/* LT failed already, reset lane to avoid
				 * it run into hanging, then start LT again.
				 */
				xgkr->stats.training_failed_count++;
				if (xgkr_inst->bp_mode == PHY_BACKPLANE_40GBASE_KR) {
					/* Reset only the Master Lane */
					if (xgkr->idx == MASTER_LANE) {
						//dbg_log_lane(xgkr, "xgkr_start_train_step: reset MASTER lane");
						xgkr->srds->reset_lane(xgkr->reg_base, LANE_RX_TX);
					}
				} else {
					//dbg_log_lane(xgkr, "xgkr_start_train_step: reset Rx/Tx lane");
					xgkr->srds->reset_lane(xgkr->reg_base, LANE_RX_TX);
				}
				
				start_lt(xgkr);
			} else if ((val & PMD_STATUS_SUP_STAT) &&
				   (val & PMD_STATUS_FRAME_LOCK)) {
				bTimeout = false;
				break;
			}

			usleep_range(100, 500);
		}//while
		if (bTimeout)
			xgkr->stats.training_timeouts++;

		if (!((val & PMD_STATUS_FRAME_LOCK) &&
		      (val & PMD_STATUS_SUP_STAT))) {
			i++;
			continue;
		}

		/* init process */
		lp_rx_ready = false;
		tx_training_complete = false;

		/* the LT should be finished in 500ms, failed or OK. */
		dead_line = jiffies + msecs_to_jiffies(lt_timeout);
		bTimeout = true;
		while (time_before(jiffies, dead_line)) {
			/* check if the LT is already failed */

			lt_state = xgkr_phy_read_mmd(xgkr, lt_MDIO_MMD,
						lt_KR_PMD_STATUS);

			if (lt_state & TRAIN_FAIL) {
				xgkr->stats.training_failed_count++;
				if (xgkr_inst->bp_mode == PHY_BACKPLANE_40GBASE_KR) {
					/* Reset only the Master Lane */
					if (xgkr->idx == MASTER_LANE) {
						//dbg_log_lane(xgkr, "xgkr_start_train_step: reset MASTER lane");
						xgkr->srds->reset_lane(xgkr->reg_base, LANE_RX_TX);
					}
				} else {
					//dbg_log_lane(xgkr, "xgkr_start_train_step: reset Rx/Tx lane");
					xgkr->srds->reset_lane(xgkr->reg_base, LANE_RX_TX);
				}
				
				bTimeout = false;
				break;
			}

			lp_rx_ready = check_rx(xgkr);
			tx_training_complete = tx_c->tx_complete;

			if (lp_rx_ready) {
				/* LP receiver is ready */
				/* As soon as the LP shows ready, no need to do any more updates. */
				xgkr->ld_status &= ~ALL_COE_MASK;
				ld_coe_status(xgkr);
			} else {
				train_local_tx(xgkr);
			}

			if (!tx_training_complete) {
				train_remote_tx(xgkr);
				if (xgkr->lt_error) {
					bTimeout = false;
					break;
				}
			}

			if (lp_rx_ready && tx_training_complete) {
				bTimeout = false;
				break;
			}

			usleep_range(100, 500);
		}//while
		if (bTimeout)
			xgkr->stats.training_timeouts++;

		i++;
		/* check if LT Error occurred */
		if (xgkr->lt_error) {
			dbg_log_lane(xgkr, "xgkr_start_train_step: LT Error");
			//xgkr->stats.training_lt_error_count++;
			//TODO: should init here or what to do ??
			init_xgkr(xgkr, false);
			continue;
		} else
		/* check LT result */
		if (is_link_training_fail(xgkr)) {
			dbg_log_lane(xgkr, "xgkr_start_train_step: link_training_failed");
			xgkr->stats.training_failed_count++;
			//TODO: should init here or what to do ??
			//should reset the bit : RX_READY_MASK ??
			//should init xgkr with reset = true ?
			init_xgkr(xgkr, false);
			continue;
		} else {
			stop_lt(xgkr);
			xgkr->state = TRAINED;
			xgkr->stats.lt_duration = jiffies_to_msecs(jiffies) - xgkr->stats.lt_start;
			xgkr->stats.tuned_tecr0 = xgkr->srds->read_tecr0(xgkr->reg_base);
			
			dbg_log_lane(xgkr, "xgkr_start_train_step: 10GBase-KR link TRAINED at TECR0: 0x%08x", xgkr->stats.tuned_tecr0);
			dbg_log_lane(xgkr, "xgkr_start_train_step: Training duration: %d ms, (Tx equalization: RATIO_PREQ = 0x%x, RATIO_PST1Q = 0x%x, ADPT_EQ = 0x%x)",
					xgkr->stats.lt_duration, xgkr->tuned_ratio_preq, xgkr->tuned_ratio_pst1q, xgkr->tuned_adpt_eq);

			switch (xgkr_inst->bp_mode)
			{
			case PHY_BACKPLANE_10GBASE_KR:
				if (phydev->attached_dev == NULL)
					dev_info(&phydev->mdio.dev, "10GBase-KR link trained (Tx equalization: RATIO_PREQ = 0x%x, RATIO_PST1Q = 0x%x, ADPT_EQ = 0x%x)\n",
							xgkr->tuned_ratio_preq, xgkr->tuned_ratio_pst1q, xgkr->tuned_adpt_eq);
				else
					dev_info(&phydev->mdio.dev, "%s %s: 10GBase-KR link trained (Tx equalization: RATIO_PREQ = 0x%x, RATIO_PST1Q = 0x%x, ADPT_EQ = 0x%x)\n",
							dev_driver_string(phydev->attached_dev->dev.parent), 
							dev_name(phydev->attached_dev->dev.parent),
							xgkr->tuned_ratio_preq, xgkr->tuned_ratio_pst1q, xgkr->tuned_adpt_eq);
				break;
				
			case PHY_BACKPLANE_40GBASE_KR:
				if (xgkr->idx == xgkr_inst->phy_lanes - 1) {
					if (phydev->attached_dev == NULL)
						dev_info(&phydev->mdio.dev, "40GBase-KR link trained at lanes Tx equalization:\n");
					else
						dev_info(&phydev->mdio.dev, "%s %s: 40GBase-KR link trained at lanes Tx equalization:\n",
								dev_driver_string(phydev->attached_dev->dev.parent), 
								dev_name(phydev->attached_dev->dev.parent));

					for (j = 0; j < xgkr_inst->phy_lanes; j++) {
						if (phydev->attached_dev == NULL)
							dev_info(&phydev->mdio.dev, "40GBase-KR Lane %d: RATIO_PREQ = 0x%x, RATIO_PST1Q = 0x%x, ADPT_EQ = 0x%x\n",
									j, xgkr_inst->xgkr[j].tuned_ratio_preq, xgkr_inst->xgkr[j].tuned_ratio_pst1q, xgkr_inst->xgkr[j].tuned_adpt_eq);
						else
							dev_info(&phydev->mdio.dev, "%s %s: 40GBase-KR Lane %d: RATIO_PREQ = 0x%x, RATIO_PST1Q = 0x%x, ADPT_EQ = 0x%x\n",
									dev_driver_string(phydev->attached_dev->dev.parent),
									dev_name(phydev->attached_dev->dev.parent),
									j, xgkr_inst->xgkr[j].tuned_ratio_preq, xgkr_inst->xgkr[j].tuned_ratio_pst1q, xgkr_inst->xgkr[j].tuned_adpt_eq);
					}
				}
				break;
			}

			break;
		}
	}
}

static void xgkr_request_restart_an(struct xgkr_params *xgkr)
{
	struct phy_device *phydev = xgkr->phydev;
	struct xgkr_phy_data *xgkr_inst = phydev->priv;
	int i;

	if (time_before(jiffies, xgkr->rt_time))
		return;
	
	xgkr_inst->stats.aneg_restarted_count++;

	dbg_log_lane(xgkr, "xgkr_request_restart_an:");

	switch (xgkr_inst->bp_mode)
	{
	case PHY_BACKPLANE_1000BASE_KX:
		dev_err(&phydev->mdio.dev, "Wrong call path for 1000Base-KX \n");
		break;

	case PHY_BACKPLANE_10GBASE_KR:
		init_xgkr(xgkr, true);
		// Reset the lane to recover from link down
		xgkr->srds->reset_lane(xgkr->reg_base, LANE_RX_TX);
		reset_lt(xgkr);
		start_xgkr_an(xgkr);
		break;

	case PHY_BACKPLANE_40GBASE_KR:
//TODO: New algorithm doesn't work on 40G !
		for (i = 0; i < xgkr_inst->phy_lanes; i++) {

			init_xgkr(&xgkr_inst->xgkr[i], true);
			// Reset the lane to recover from link down
			xgkr_inst->xgkr[i].srds->reset_lane(xgkr_inst->xgkr[i].reg_base, LANE_RX_TX);
			reset_lt(&xgkr_inst->xgkr[i]);
		}
		//Start AN only for Master Lane
		start_xgkr_an(&xgkr_inst->xgkr[MASTER_LANE]);
		break;
	}
	
	xgkr->rt_time = jiffies + msecs_to_jiffies(XGKR_DENY_RT_INTERVAL);
}

static void xgkr_state_machine(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct xgkr_params *xgkr = container_of(dwork,
						  struct xgkr_params, xgkr_wk);
	struct phy_device *phydev = xgkr->phydev;
	struct xgkr_phy_data *xgkr_inst = phydev->priv;
	int an_state;
	int i;
	bool start_train = false;
	unsigned long xgkr_timeout = XGKR_TIMEOUT_1;

#ifdef DEBUG_MONITORING
	if (xgkr->state == DETECTING_LP)
	{
		dbg_log_lane(xgkr, "xgkr_state_machine: enter_cycle, xgkr = 0x%08x", xgkr);
	}

	if (!xgkr_inst->aneg_done)
	{
		an_state = phy_read_mmd(phydev, MDIO_MMD_AN, g_an_STAT);
		dbg_log_lane(xgkr, "xgkr_state_machine: Monitoring: AN_status_aneg_complete (bit_5) = %d", (an_state & AN_COMPLETE_MASK) ? 1 : 0);

		an_state = xgkr_phy_read_mmd(xgkr, MDIO_MMD_AN, g_an_BP_STAT);
		dbg_log_lane(xgkr, "xgkr_state_machine: Monitoring: BP Eth stat negotiated 10GBase-KR = %d", (an_state & KR_AN_MASK_10G) ? 1 : 0);

		dbg_log_lane(xgkr, "xgkr_state_machine: Monitoring: BP Eth stat negotiated BP AN ability = %d", (an_state & 0x1) ? 1 : 0);
		dbg_log_lane(xgkr, "xgkr_state_machine: Monitoring: BP Eth stat negotiated 1000Base-KX = %d", (an_state & 0x2) ? 1 : 0);
		dbg_log_lane(xgkr, "xgkr_state_machine: Monitoring: BP Eth stat negotiated 10GBase-KX4 = %d", (an_state & 0x4) ? 1 : 0);
		dbg_log_lane(xgkr, "xgkr_state_machine: Monitoring: BP Eth stat negotiated BASE-R FEC = %d", (an_state & 0x10) ? 1 : 0);
		dbg_log_lane(xgkr, "xgkr_state_machine: Monitoring: BP Eth stat negotiated 40GBase-KR4 = %d", (an_state & 0x20) ? 1 : 0);
		dbg_log_lane(xgkr, "xgkr_state_machine: Monitoring: BP Eth stat negotiated 40GBase-CR4 = %d", (an_state & 0x40) ? 1 : 0);
		dbg_log_lane(xgkr, "xgkr_state_machine: Monitoring: BP Eth stat negotiated 100GBase-CR10 = %d", (an_state & 0x100) ? 1 : 0);

		dbg_log_lane(xgkr, "xgkr_state_machine: Monitoring: aneg_done = %d", (xgkr_inst->aneg_done) ? 1 : 0);
		dbg_log_lane(xgkr, "xgkr_state_machine: Monitoring: an_link_up = %d", (is_an_link_up(phydev)) ? 1 : 0);
		dbg_log_lane(xgkr, "xgkr_state_machine: Monitoring: pcs_link_up = %d", (is_pcs_link_up(phydev)) ? 1 : 0);
#ifdef DEBUG_GBASE_R_LINK_UP
		dbg_log_lane(xgkr, "xgkr_state_machine: Monitoring: 10gbase_r_pcs_link_up = %d", (is_10gbase_r_pcs_link_up(phydev)) ? 1 : 0);
#endif

		//AN Advertisement
		an_state = phy_read_mmd(phydev, MDIO_MMD_AN, 16);
		dbg_log_lane(xgkr, "xgkr_state_machine: Monitoring: AN_Adv Next_page = %d", (an_state & 0x8000) ? 1 : 0);
		an_state = phy_read_mmd(phydev, MDIO_MMD_AN, 17);
		dbg_log_lane(xgkr, "xgkr_state_machine: Monitoring: AN_Adv Technology = %d", (an_state & 0x0080) ? 1 : 0);
	}
#endif //DEBUG_MONITORING

	/**
	 * The link training occurs after auto-negotiation has determined the link to be a Base-R link.
	 * This is indicated by asserting the corresponding technology bit within the BP_ETH_STATUS
	 * register. Note that this occurs before auto-negotiation can declare auto-negotiation complete,
	 * as this requires the PCS to report a valid link.
	 *
	 * signaled by register: Backplane Ethernet Status Register (g_an_BP_STAT)
	 *   	bit KR_AN_MASK_10G for 10GBase-KR
	 *   	bit KR_AN_MASK_40G for 40GBase-KR
	 */

#if 0
	//This condition is not really needed because we must wait for bit: KR_AN_MASK_10G
	if (!xgkr_inst->aneg_config) {
		start_xgkr_state_machine(&xgkr->xgkr_wk, XGKR_TIMEOUT_1);
		return;
	}
	//Do NOT wait for auto-negotiation to complete
	//if (!xgkr_inst->aneg_done)
#endif

#ifdef WAIT_AN_status_aneg_complete
	/**
	 * TODO: not sure if we should also wait for this bit AN_status_aneg_complete (bit_5)
	 * that signals BP_ETH_STATUS is valid now...
	 * - don't use this condition because: BP_stat_KR_negotiated is raised to 1 before AN_status_aneg_complete (bit_5)
	 */
	an_state = phy_read_mmd(phydev, MDIO_MMD_AN, g_an_STAT);
	if ((an_state & AN_COMPLETE_MASK) == 0)
	{
		start_xgkr_state_machine(&xgkr->xgkr_wk, XGKR_TIMEOUT_1);
		return;
	}
	/**
	 * BP_ETH_STATUS valid now (once the bit above was raised to 1)
	 */
#endif

	mutex_lock(&phydev->lock);
	switch (xgkr->state) {
	case DETECTING_LP:

		switch (xgkr_inst->bp_mode)
		{
		case PHY_BACKPLANE_1000BASE_KX:
			dev_err(&phydev->mdio.dev, "Wrong call path for 1000Base-KX \n");
			break;

		case PHY_BACKPLANE_10GBASE_KR:
			an_state = xgkr_phy_read_mmd(xgkr, MDIO_MMD_AN, g_an_BP_STAT);
			if (an_state & KR_AN_MASK_10G) {
				//AN acquired: Train the lane
				xgkr->an_acquired = true;
				xgkr->an_wait_count = 0;
				start_train = true;

#ifdef DEBUG_MONITORING
				//start_train monitoring
				dbg_log_lane(xgkr, "xgkr_state_machine: start_train: aneg_done = %d", (xgkr_inst->aneg_done) ? 1 : 0);

				an_state = phy_read_mmd(phydev, MDIO_MMD_AN, g_an_STAT);
				dbg_log_lane(xgkr, "xgkr_state_machine: start_train: AN_status_aneg_complete (bit_5) = %d", (an_state & AN_COMPLETE_MASK) ? 1 : 0);

				an_state = xgkr_phy_read_mmd(xgkr, MDIO_MMD_AN, g_an_BP_STAT);
				dbg_log_lane(xgkr, "xgkr_state_machine: Monitoring: BP Eth stat negotiated 10GBase-KR = %d", (an_state & KR_AN_MASK_10G) ? 1 : 0);

				dbg_log_lane(xgkr, "xgkr_state_machine: Monitoring: BP Eth stat negotiated BP AN ability = %d", (an_state & 0x1) ? 1 : 0);
				dbg_log_lane(xgkr, "xgkr_state_machine: Monitoring: BP Eth stat negotiated 1000Base-KX = %d", (an_state & 0x2) ? 1 : 0);
				dbg_log_lane(xgkr, "xgkr_state_machine: Monitoring: BP Eth stat negotiated 10GBase-KX4 = %d", (an_state & 0x4) ? 1 : 0);
				dbg_log_lane(xgkr, "xgkr_state_machine: Monitoring: BP Eth stat negotiated BASE-R FEC = %d", (an_state & 0x10) ? 1 : 0);
				dbg_log_lane(xgkr, "xgkr_state_machine: Monitoring: BP Eth stat negotiated 40GBase-KR4 = %d", (an_state & 0x20) ? 1 : 0);
				dbg_log_lane(xgkr, "xgkr_state_machine: Monitoring: BP Eth stat negotiated 40GBase-CR4 = %d", (an_state & 0x40) ? 1 : 0);
				dbg_log_lane(xgkr, "xgkr_state_machine: Monitoring: BP Eth stat negotiated 100GBase-CR10 = %d", (an_state & 0x100) ? 1 : 0);

				//AN Advertisement
				an_state = phy_read_mmd(phydev, MDIO_MMD_AN, 16);
				dbg_log_lane(xgkr, "xgkr_state_machine: start_train: AN_Adv Next_page = %d", (an_state & 0x8000) ? 1 : 0);
				an_state = phy_read_mmd(phydev, MDIO_MMD_AN, 17);
				dbg_log_lane(xgkr, "xgkr_state_machine: start_train: AN_Adv Technology = %d", (an_state & 0x0080) ? 1 : 0);
#endif //DEBUG_MONITORING

			} else {
				//AN lost or not yet acquired
				if (xgkr->an_acquired) {
					//AN acquired first time but now was lost
					if (!is_link_up(phydev)) {
						//Link is down: restart training
						xgkr->an_wait_count = 0;
						xgkr_request_restart_an(xgkr);
					} else {
						//Link is up: wait few iterations for AN to be acquired
						if (xgkr->an_wait_count >= XGKR_AN_WAIT_ITERATIONS) {
							xgkr->an_wait_count = 0;
							xgkr_request_restart_an(xgkr);
						} else {
							xgkr->an_wait_count++;
						}
					}
				} else {
					/**
					 * AN was not yet acquired first time
					 * DO nothing, just wait AN to be acquired first time
					 */
				}
			}
			break;

		case PHY_BACKPLANE_40GBASE_KR:
			//Check AN state only on Master Lane
			an_state = xgkr_phy_read_mmd(&xgkr_inst->xgkr[MASTER_LANE], MDIO_MMD_AN, g_an_BP_STAT);
			if (an_state & KR_AN_MASK_40G) {
				//AN acquired: Train all lanes in order starting with Master Lane
				xgkr->an_acquired = true;
				xgkr->an_wait_count = 0;
				if (xgkr->idx == MASTER_LANE) {
					start_train = true;
				}
				else if (xgkr_inst->xgkr[xgkr->idx - 1].state == TRAINED) {
					start_train = true;
				}
			} else {
				//AN lost or not yet acquired
				if (xgkr->an_acquired) {
					//AN acquired first time but now was lost

					if (!is_link_up(phydev)) {
						//Link is down: restart training
						xgkr->an_wait_count = 0;
						if (xgkr->idx == MASTER_LANE)
							xgkr_request_restart_an(xgkr);
						else if (xgkr_inst->xgkr[xgkr->idx - 1].state == TRAINED)
							xgkr_request_restart_an(xgkr);
					} else {
						//Link is up: wait few iterations for AN to be acquired
						if (xgkr->an_wait_count >= XGKR_AN_WAIT_ITERATIONS) {
							xgkr->an_wait_count = 0;
							if (xgkr->idx == MASTER_LANE)
								xgkr_request_restart_an(xgkr);
							else if (xgkr_inst->xgkr[xgkr->idx - 1].state == TRAINED)
								xgkr_request_restart_an(xgkr);
						} else {
							xgkr->an_wait_count++;
						}
					}
				} else {
					/**
					 * AN was not yet acquired first time
					 * DO nothing, just wait AN to be acquired first time
					 */
				}

			}
			break;
		}
		break;

	case TRAINED:
		xgkr_timeout = XGKR_TIMEOUT_2;
		if (!is_link_up(phydev)) {
			xgkr_timeout = XGKR_TIMEOUT_1;
			switch (xgkr_inst->bp_mode)
			{
			case PHY_BACKPLANE_1000BASE_KX:
				dev_err(&phydev->mdio.dev, "Wrong call path for 1000Base-KX \n");
				break;

			case PHY_BACKPLANE_10GBASE_KR:
				dev_info(&phydev->mdio.dev, "Detect hotplug, restart training\n");
				/* initializations on Detect hotplug / restart: they must not be part of init_xgkr */
				xgkr_inst->xgkr[SINGLE_LANE].first_recv_init = false;
				xgkr_request_restart_an(xgkr);
				break;

			case PHY_BACKPLANE_40GBASE_KR:
				if (xgkr->idx == MASTER_LANE) {
					//check if all lanes are trained only on Master Lane
					if (are_all_lanes_trained(xgkr_inst)) {
						dev_info(&phydev->mdio.dev, "Detect hotplug, restart training\n");
						for (i = 0; i < xgkr_inst->phy_lanes; i++) {
							/* initializations on Detect hotplug / restart: they must not be part of init_xgkr */
							xgkr_inst->xgkr[i].first_recv_init = false;
						}
						xgkr_request_restart_an(xgkr);
					}
				}
				break;
			}
		}
		break;
	}

	if (start_train) {
		xgkr_start_train_step(xgkr);
	}

	mutex_unlock(&phydev->lock);
	start_xgkr_state_machine(&xgkr->xgkr_wk, xgkr_timeout);
}

//------------------------------------------------------------------------------------------------------
//
//            Driver callback functions
//

static int fsl_backplane_probe(struct phy_device *phydev)
{
	struct xgkr_phy_data *xgkr_inst;
	struct device_node *phy_node, *lane_node;
	struct resource res_lane;
	struct serdes_access *srds = NULL;
	int serdes_type = SERDES_INVAL;
	const char *st;
	const char *bm;
	int comp_no, ret, i, phy_lanes;
	int bp_mode;
	u32 lane_base_addr[MAX_PHY_LANES_NO], lane_memmap_size;

	phy_node = phydev->mdio.dev.of_node;
	if (!phy_node) {
		dev_err(&phydev->mdio.dev, "No associated device tree node\n");
		return -EINVAL;
	}

	dbg_log_phy(phydev, "fsl_backplane_probe: Running Training Algorithm %s", TRAIN_ALGORITHM_VERSION);

	bp_mode = of_property_read_string(phy_node, "backplane-mode", &bm);
	if (bp_mode < 0)
		return -EINVAL;

	phy_lanes = 1;
	if (!strcasecmp(bm, "1000base-kx")) {
		bp_mode = PHY_BACKPLANE_1000BASE_KX;
	} else if (!strcasecmp(bm, "10gbase-kr")) {
		bp_mode = PHY_BACKPLANE_10GBASE_KR;
	} else if (!strcasecmp(bm, "40gbase-kr")) {
		bp_mode = PHY_BACKPLANE_40GBASE_KR;
		phy_lanes = 4;
	} else {
		dev_err(&phydev->mdio.dev, "Unknown backplane-mode\n");
		return -EINVAL;
	}

	lane_node = of_parse_phandle(phy_node, "fsl,lane-handle", 0);
	if (!lane_node) {
		dev_err(&phydev->mdio.dev, "parse fsl,lane-handle failed\n");
		return -EINVAL;
	}

	comp_no = of_property_count_strings(lane_node, "compatible");
	for (i = 0; i < comp_no; i++) {
		ret = of_property_read_string_index(lane_node, "compatible", i, &st);
		if (ret == 0) {
			if (!strcasecmp(st, "fsl,serdes-10g")) {
				serdes_type = SERDES_10G;
				break;
			} else if (!strcasecmp(st, "fsl,serdes-28g")) {
				serdes_type = SERDES_28G;
				break;
			}
		}
	}
	if (serdes_type == SERDES_INVAL) {
		dev_err(&phydev->mdio.dev, "Unknown serdes-type\n");
		return -EINVAL;
	}

	ret = of_address_to_resource(lane_node, 0, &res_lane);
	if (ret) {
		dev_err(&phydev->mdio.dev, "could not obtain memory map\n");
		return ret;
	}

	of_node_put(lane_node);
	ret = of_property_read_u32_array(phy_node, "fsl,lane-reg",
					 (u32 *)lane_base_addr, phy_lanes);
	if (ret) {
		dev_err(&phydev->mdio.dev, "could not get fsl,lane-reg\n");
		return -EINVAL;
	}

	switch (serdes_type)
	{
	case SERDES_10G:
		setup_an_lt_ls();
		srds = setup_serdes_access_10g();
		break;

	case SERDES_28G:
		setup_an_lt_lx();
		srds = setup_serdes_access_28g();
		break;

	default:
		dev_err(&phydev->mdio.dev, "Unsupported serdes-type\n");
		return -EINVAL;
	}

	if (!srds) {
		dev_err(&phydev->mdio.dev, "Unsupported serdes-type\n");
		return -EINVAL;
	}

	srds->serdes_type = serdes_type;
	srds->is_little_endian = of_property_read_bool(lane_node, "little-endian");

	if (srds->is_little_endian) {
		srds->ioread32 = le_ioread32;
		srds->iowrite32 = le_iowrite32;
	} else {
		srds->ioread32 = be_ioread32;
		srds->iowrite32 = be_iowrite32;
	}

	backplane_dbg_init();

	xgkr_inst = devm_kzalloc(&phydev->mdio.dev,
				 sizeof(*xgkr_inst), GFP_KERNEL);
	if (!xgkr_inst)
		return -ENOMEM;

	phydev->priv = xgkr_inst;

	xgkr_inst->phy_lanes = phy_lanes;
	xgkr_inst->bp_mode = bp_mode;
	mutex_init(&xgkr_inst->phy_lock);

	lane_memmap_size = srds->get_lane_memmap_size();
	
	for (i = 0; i < phy_lanes; i++) {
		xgkr_inst->xgkr[i].idx = i;
		xgkr_inst->xgkr[i].phydev = phydev;
		xgkr_inst->xgkr[i].srds = srds;
		xgkr_inst->xgkr[i].reg_base = devm_ioremap_nocache(&phydev->mdio.dev,
						    res_lane.start + lane_base_addr[i],
						    lane_memmap_size);
		if (!xgkr_inst->xgkr[i].reg_base) {
			dev_err(&phydev->mdio.dev, "ioremap_nocache failed\n");
			return -ENOMEM;
		}
		setup_default_tecr(&xgkr_inst->xgkr[i]);
		xgkr_inst->xgkr[i].rt_time = jiffies + msecs_to_jiffies(XGKR_DENY_RT_INTERVAL);

		dbg_log_phy(phydev, "fsl_backplane_probe: phydev = 0x%08x, lane = %d, lane_addr = 0x%08x \n",
				phydev, i, res_lane.start + lane_base_addr[i]);
	}

	switch (bp_mode)
	{
	case PHY_BACKPLANE_1000BASE_KX:
		phydev->speed = SPEED_1000;
		/* configure the lane for 1000BASE-KX */
		srds->lane_set_1gkx(xgkr_inst->xgkr[SINGLE_LANE].reg_base);
		break;

	case PHY_BACKPLANE_10GBASE_KR:
		phydev->speed = SPEED_10000;
		INIT_DELAYED_WORK(&xgkr_inst->xgkr[SINGLE_LANE].xgkr_wk, xgkr_state_machine);
		break;

	case PHY_BACKPLANE_40GBASE_KR:
		phydev->speed = SPEED_40000;
		for (i = 0; i < phy_lanes; i++)
			INIT_DELAYED_WORK(&xgkr_inst->xgkr[i].xgkr_wk, xgkr_state_machine);
		break;
	}

	return 0;
}

static int fsl_backplane_aneg_done(struct phy_device *phydev)
{
	struct xgkr_phy_data *xgkr_inst = phydev->priv;
	int i;

	if (!phydev->mdio.dev.of_node) {
		dev_err(&phydev->mdio.dev, "No associated device tree node\n");
		return -EINVAL;
	}
	
	dbg_log_phy(phydev, "fsl_backplane_aneg_done:");

	xgkr_inst->aneg_done = true;

	phydev->state = PHY_RUNNING;

	/* add work on workqueue once more just in case the state machine has not started
	 * it happened before and the link remained hanged,
	 * so we must be sure the state machine started at this point
	 * */
	if (xgkr_inst->bp_mode == PHY_BACKPLANE_10GBASE_KR ||
		xgkr_inst->bp_mode == PHY_BACKPLANE_40GBASE_KR) {
		for (i = 0; i < xgkr_inst->phy_lanes; i++) {
			start_xgkr_state_machine(&xgkr_inst->xgkr[i].xgkr_wk, XGKR_TIMEOUT_1);
		}
	}

	return 1;
}

static int fsl_backplane_config_aneg(struct phy_device *phydev)
{
	struct xgkr_phy_data *xgkr_inst = phydev->priv;
	int i;

	if (!phydev->mdio.dev.of_node) {
		dev_err(&phydev->mdio.dev, "No associated device tree node\n");
		return -EINVAL;
	}

	dbg_log_phy(phydev, "fsl_backplane_config_aneg: Running Training Algorithm %s", TRAIN_ALGORITHM_VERSION);

#ifdef BIN_MODULES_ORDER_BINLONG_BINM1
	dbg_log_phy(phydev, "Bin Modules order:  BinLong before BinM1");
#else
	dbg_log_phy(phydev, "Bin Modules order:  BinM1 before BinLong");
#endif

#ifdef ENABLE_HAPPY_COND_4_ON_SLIDE_4
	dbg_log_phy(phydev, "Rx 4th Happy condition on slide 4 is enabled");
#else
	dbg_log_phy(phydev, "Rx 4th Happy condition on slide 4 is disabled");
#endif //ENABLE_HAPPY_COND_4_ON_SLIDE_4

#ifdef ENABLE_LESS_HAPPY_COND_2
	dbg_log_phy(phydev, "Rx Less Happy condition is enabled");
#else
	dbg_log_phy(phydev, "Rx Less Happy condition is disabled");
#endif //ENABLE_LESS_HAPPY_COND_2

#ifdef ENABLE_EVEN_LESS_HAPPY_COND_3
	dbg_log_phy(phydev, "Rx Even Less Happy condition is enabled");
#else
	dbg_log_phy(phydev, "Rx Even Less Happy condition is disabled");
#endif //ENABLE_EVEN_LESS_HAPPY_COND_3

#ifdef ENABLE_SEEMINGLY_HAPPY_COND_4
	dbg_log_phy(phydev, "Rx Seemingly Happy condition is enabled");
#else
	dbg_log_phy(phydev, "Rx Seemingly Happy condition is disabled");
#endif //ENABLE_SEEMINGLY_HAPPY_COND_4

	switch (xgkr_inst->bp_mode)
	{
	case PHY_BACKPLANE_1000BASE_KX:
		phydev->speed = SPEED_1000;
		phydev->supported |= SUPPORTED_1000baseKX_Full;
		start_1gkx_an(phydev);
		break;

	case PHY_BACKPLANE_10GBASE_KR:

		//required for T2080 only: TECR0 is not correctly read on probe
		setup_default_tecr(&xgkr_inst->xgkr[SINGLE_LANE]);

		dbg_log_phy(phydev, "fsl_backplane_config_aneg: initial TECR0: 0x%08x",
				xgkr_inst->xgkr[SINGLE_LANE].stats.init_tecr0);
		dbg_log_phy(phydev, "fsl_backplane_config_aneg: starting with: RATIO_PREQ = 0x%x, RATIO_PST1Q = 0x%x, ADPT_EQ = 0x%x",
				xgkr_inst->xgkr[SINGLE_LANE].def_ratio_preq, xgkr_inst->xgkr[SINGLE_LANE].def_ratio_pst1q, xgkr_inst->xgkr[SINGLE_LANE].def_adpt_eq);

		phydev->speed = SPEED_10000;
		phydev->supported |= SUPPORTED_10000baseKR_Full;
		if (lanes_trained_count(xgkr_inst) == 0) {
			init_xgkr(&xgkr_inst->xgkr[SINGLE_LANE], true);
			reset_lt(&xgkr_inst->xgkr[SINGLE_LANE]);
			start_xgkr_an(&xgkr_inst->xgkr[SINGLE_LANE]);
			/* start state machine*/
			start_xgkr_state_machine(&xgkr_inst->xgkr[SINGLE_LANE].xgkr_wk, XGKR_TIMEOUT_1);
		}
		break;

	case PHY_BACKPLANE_40GBASE_KR:
		phydev->speed = SPEED_40000;
		phydev->supported |= SUPPORTED_40000baseKR4_Full;
		if (lanes_trained_count(xgkr_inst) == 0) {
			for (i = 0; i < xgkr_inst->phy_lanes; i++) {
				init_xgkr(&xgkr_inst->xgkr[i], true);
				reset_lt(&xgkr_inst->xgkr[i]);
			}
			//Start AN only for Master Lane
			start_xgkr_an(&xgkr_inst->xgkr[MASTER_LANE]);
			/* start state machine*/
			for (i = 0; i < xgkr_inst->phy_lanes; i++) {
				start_xgkr_state_machine(&xgkr_inst->xgkr[i].xgkr_wk, XGKR_TIMEOUT_1);
			}
		}
		break;
	}

	phydev->advertising = phydev->supported;
	phydev->duplex = 1;

	xgkr_inst->force_retrained = 0;

	phydev->state = PHY_AN;

	backplane_dbg_add(phydev);

	xgkr_inst->aneg_config = true;

	return 0;
}

static int fsl_backplane_suspend(struct phy_device *phydev)
{
	struct xgkr_phy_data *xgkr_inst = phydev->priv;
	int i;

	if (!phydev->mdio.dev.of_node) {
		dev_err(&phydev->mdio.dev, "No associated device tree node\n");
		return -EINVAL;
	}

	dbg_log_phy(phydev, "fsl_backplane_suspend:");

	if (xgkr_inst->aneg_config && !xgkr_inst->phy_suspended) {

		if (xgkr_inst->bp_mode == PHY_BACKPLANE_10GBASE_KR ||
			xgkr_inst->bp_mode == PHY_BACKPLANE_40GBASE_KR) {
			for (i = 0; i < xgkr_inst->phy_lanes; i++)
				cancel_delayed_work_sync(&xgkr_inst->xgkr[i].xgkr_wk);
		}
		xgkr_inst->phy_suspended = true;
	}

	return 0;
}

static int fsl_backplane_resume(struct phy_device *phydev)
{
	struct xgkr_phy_data *xgkr_inst = phydev->priv;
	int i;

	if (!phydev->mdio.dev.of_node) {
		dev_err(&phydev->mdio.dev, "No associated device tree node\n");
		return -EINVAL;
	}

	dbg_log_phy(phydev, "fsl_backplane_resume:");

	if (xgkr_inst->aneg_config && xgkr_inst->phy_suspended) {

		if (xgkr_inst->bp_mode == PHY_BACKPLANE_10GBASE_KR ||
			xgkr_inst->bp_mode == PHY_BACKPLANE_40GBASE_KR) {
			for (i = 0; i < xgkr_inst->phy_lanes; i++) {
				init_xgkr(&xgkr_inst->xgkr[i], true);
				start_xgkr_state_machine(&xgkr_inst->xgkr[i].xgkr_wk, XGKR_TIMEOUT_1);
			}
		}
		xgkr_inst->phy_suspended = false;
	}

	return 0;
}

static int fsl_backplane_read_status(struct phy_device *phydev)
{
	struct xgkr_phy_data *xgkr_inst = phydev->priv;

	if (!phydev->mdio.dev.of_node) {
		dev_err(&phydev->mdio.dev, "No associated device tree node\n");
		return -EINVAL;
	}

	/* Linkup method proposal for training stability:
	 * Don't raise linkup until all lanes are trained
	 * in order to prevent interface sending packets that may
	 * interfere with the training packets
	 */
	if (is_link_up(phydev))
		phydev->link = are_all_lanes_trained(xgkr_inst);
	else
		phydev->link = 0;

	xgkr_inst->stats.lp_detected = phydev->link;

	return 0;
}

static int fsl_backplane_match_phy_device(struct phy_device *phydev)
{
	struct device_node *phy_node, *lane_node;
	const char *st;
	int comp_no, i, ret;
	int serdes_type = SERDES_INVAL;
	const int num_ids = ARRAY_SIZE(phydev->c45_ids.device_ids);

	if (!phydev->mdio.dev.of_node) {
		return 0;
	}

	//	 WORKAROUND:
	// Required for LX2 devices
	// where PHY ID cannot be verified in PCS
	// because PCS Device Identifier Upper and Lower registers are hidden
	// and always return 0 when they are read:
	// 2  02 	Device_ID0  RO 		Bits 15:0 	0
	// val = phy_read_mmd(phydev, MDIO_MMD_PCS, 0x2);
	// 3  03 	Device_ID1  RO 		Bits 31:16 	0
	// val = phy_read_mmd(phydev, MDIO_MMD_PCS, 0x3);
	//
	// To be removed: After the issue will be fixed on LX2 devices

	if (!phydev->is_c45)
		return 0;

	phy_node = phydev->mdio.dev.of_node;

	lane_node = of_parse_phandle(phy_node, "fsl,lane-handle", 0);
	if (!lane_node) {
		dev_err(&phydev->mdio.dev, "parse fsl,lane-handle failed\n");
		return 0;
	}

	comp_no = of_property_count_strings(lane_node, "compatible");
	for (i = 0; i < comp_no; i++) {
		ret = of_property_read_string_index(lane_node, "compatible", i, &st);
		if (ret == 0) {
			if (!strcasecmp(st, "fsl,serdes-10g")) {
				serdes_type = SERDES_10G;
				break;
			} else if (!strcasecmp(st, "fsl,serdes-28g")) {
				serdes_type = SERDES_28G;
				break;
			}
		}
	}
	if (serdes_type == SERDES_INVAL) {
		dev_err(&phydev->mdio.dev, "Unknown serdes-type\n");
		return 0;
	}

	if (serdes_type == SERDES_10G) {
		//On LS devices we must find the c45 device with correct PHY ID
		//Implementation similar with the one existent in phy_device: @function: phy_bus_match
		for (i = 1; i < num_ids; i++) {
			if (!(phydev->c45_ids.devices_in_package & (1 << i)))
				continue;

			if ((PCS_PHY_DEVICE_ID & PCS_PHY_DEVICE_ID_MASK) ==
				(phydev->c45_ids.device_ids[i] & PCS_PHY_DEVICE_ID_MASK))
			{
				return 1;
			}
		}
		return 0;
	}

	//On LX devices we cannot verify PHY ID
	//so we are happy only with preliminary verifications already made: mdio.dev.of_node and is_c45
	//because we already filtered other undesired devices: non clause 45

	return 1;
}

static int fsl_backplane_get_sset_count(struct phy_device *phydev)
{
	struct xgkr_phy_data *xgkr_inst = phydev->priv;
	int count = 0;

	if (!phydev->mdio.dev.of_node) {
		return 0;
	}
	if (!phydev->is_c45)
		return 0;

	switch (xgkr_inst->bp_mode)
	{
	case PHY_BACKPLANE_1000BASE_KX:
		dev_err(&phydev->mdio.dev, "Wrong call path for 1000Base-KX \n");
		break;

	case PHY_BACKPLANE_10GBASE_KR:
		count = ARRAY_SIZE(xgkr_phy_stats_strings) +
			ARRAY_SIZE(xgkr_lane_stats_strings);
		break;

	case PHY_BACKPLANE_40GBASE_KR:
		count = ARRAY_SIZE(xgkr_phy_stats_strings) +
			xgkr_inst->phy_lanes * ARRAY_SIZE(xgkr_lane_stats_strings);
		break;
	}
	return count;
}

static void fsl_backplane_get_strings(struct phy_device *phydev, u8 *data)
{
	struct xgkr_phy_data *xgkr_inst = phydev->priv;
	u8 *lane_data;
	int i, ln;

	if (!phydev->mdio.dev.of_node) {
		return;
	}
	if (!phydev->is_c45)
		return;

	switch (xgkr_inst->bp_mode)
	{
	case PHY_BACKPLANE_1000BASE_KX:
		dev_err(&phydev->mdio.dev, "Wrong call path for 1000Base-KX \n");
		break;

	case PHY_BACKPLANE_10GBASE_KR:
		memcpy(data, xgkr_phy_stats_strings, sizeof(xgkr_phy_stats_strings));
		lane_data = data + sizeof(xgkr_phy_stats_strings);
		memcpy(lane_data, xgkr_lane_stats_strings, sizeof(xgkr_lane_stats_strings));
		break;

	case PHY_BACKPLANE_40GBASE_KR:
		memcpy(data, xgkr_phy_stats_strings, sizeof(xgkr_phy_stats_strings));
		lane_data = data + sizeof(xgkr_phy_stats_strings);

		for (ln = 0; ln < xgkr_inst->phy_lanes; ln++) {
			for (i = 0; i < XGKR_LANE_STATS_COUNT; i++)
				sprintf(crt_lane_stats_strings[i], "Ln%d %s", ln, xgkr_lane_stats_strings[i]);

			memcpy(lane_data, crt_lane_stats_strings, sizeof(crt_lane_stats_strings));
			lane_data += sizeof(xgkr_lane_stats_strings);
		}
		break;
	}
}

static void fsl_backplane_get_stats(struct phy_device *phydev,
			  struct ethtool_stats *stats, u64 *data)
{
	struct xgkr_phy_data *xgkr_inst = phydev->priv;
	struct xgkr_params *xgkr;
	int ln;

	if (!phydev->mdio.dev.of_node) {
		return;
	}
	if (!phydev->is_c45)
		return;

	switch (xgkr_inst->bp_mode)
	{
	case PHY_BACKPLANE_1000BASE_KX:
		dev_err(&phydev->mdio.dev, "Wrong call path for 1000Base-KX \n");
		break;

	case PHY_BACKPLANE_10GBASE_KR:
		report_phy_stats(phydev, data);

		xgkr = &xgkr_inst->xgkr[SINGLE_LANE];
		report_lane_stats(xgkr, data, XGKR_PHY_STATS_COUNT);
		break;

	case PHY_BACKPLANE_40GBASE_KR:
		report_phy_stats(phydev, data);

		for (ln = 0; ln < xgkr_inst->phy_lanes; ln++) {
			xgkr = &xgkr_inst->xgkr[ln];
			report_lane_stats(xgkr, data, XGKR_PHY_STATS_COUNT + ln*XGKR_LANE_STATS_COUNT);
		}
		break;
	}
}

static struct phy_driver fsl_backplane_driver[] = {
	{
	.phy_id		= PCS_PHY_DEVICE_ID,
	.name		= "Freescale Backplane",
	.phy_id_mask	= PCS_PHY_DEVICE_ID_MASK,
	.features	= SUPPORTED_Backplane | SUPPORTED_Autoneg |
			  SUPPORTED_MII,
	.probe          = fsl_backplane_probe,
	.aneg_done      = fsl_backplane_aneg_done,
	.config_aneg	= fsl_backplane_config_aneg,
	.read_status	= fsl_backplane_read_status,
	.suspend	= fsl_backplane_suspend,
	.resume		= fsl_backplane_resume,
	.match_phy_device = fsl_backplane_match_phy_device,
	.get_sset_count = fsl_backplane_get_sset_count,
	.get_strings    = fsl_backplane_get_strings,
	.get_stats      = fsl_backplane_get_stats,
	},
};

module_phy_driver(fsl_backplane_driver);

static struct mdio_device_id __maybe_unused freescale_tbl[] = {
	{ PCS_PHY_DEVICE_ID, PCS_PHY_DEVICE_ID_MASK },
	{ }
};

MODULE_DEVICE_TABLE(mdio, freescale_tbl);

MODULE_DESCRIPTION("Freescale Backplane driver");
MODULE_AUTHOR("Shaohui Xie <Shaohui.Xie@freescale.com>");
MODULE_LICENSE("GPL v2");
