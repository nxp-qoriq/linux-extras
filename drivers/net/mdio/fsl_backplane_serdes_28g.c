// SPDX-License-Identifier: GPL-2.0+
/*
 *  DPAA Backplane driver for SerDes 28G.
 *
 * Copyright 2018-2019, 2021 NXP
 */

#include <linux/io.h>
#include <linux/delay.h>
#include <linux/sched.h>

#include "fsl_backplane.h"

#define BIN_1_SEL					0x00000000
#define BIN_2_SEL					0x00001000
#define BIN_3_SEL					0x00002000
#define BIN_4_SEL					0x00003000
#define BIN_OFFSET_SEL				0x00004000
#define BIN_BLW_SEL					0x00008000
#define BIN_DATA_AVG_SEL			0x00009000
#define BIN_M1_SEL					0x0000c000
#define BIN_LONG_SEL				0x0000d000
#define CDR_SEL_MASK				0x0000f000

#define BIN_SNP_AV_THR_LOW			-150
#define BIN_SNP_AV_THR_HIGH			150

#define RATIO_PREQ_SHIFT			16
#define RATIO_PST1Q_SHIFT			8
#define AMP_RED_SHIFT				0
#define ADPT_EQ_SHIFT				24

#define RATIO_PREQ_MASK				0x000f0000
#define RATIO_PST1Q_MASK			0x00001f00
#define ADPT_EQ_MASK				0x3f000000
#define AMP_RED_MASK				0x0000003f

#define TECR0_INIT					0x20808000

#define RESET_REQ_MASK				0x80000000

#define TCSR0_SD_STAT_OBS_EN_MASK	0x80000000
#define RECR3_SNP_START_MASK		0x80000000
#define RECR3_SNP_DONE_MASK			0x40000000

#define RECR4_SNP_DATA_MASK			0x000001ff
#define RECR4_SNP_DATA_SHIFT		0
#define RECR4_EQ_SNPBIN_SIGN_MASK	0x100

#define RECR3_GAINK2_MASK			0x1f000000
#define RECR3_GAINK2_SHIFT			24

#define RECR3_GAINK3_MASK			0x001f0000
#define RECR3_GAINK3_SHIFT			16

#define RECR3_GAINK4_MASK			0x0000001f
#define RECR3_GAINK4_SHIFT			0

#define RECR4_EQ_OFFSET_MASK		0x003f0000
#define RECR4_EQ_OFFSET_SHIFT		16

#define RRSTCTL_CDR_LOCK_MASK		0x00001000

/* Required only for 1000BASE KX */
#define GCR1_REIDL_TH_MASK			0x00700000
#define GCR1_REIDL_EX_SEL_MASK		0x000c0000
#define GCR1_REIDL_ET_MAS_MASK		0x04000000

struct per_lane_ctrl_status {
	u32 gcr0;	/* 0x.000 - General Control Register 0 */
	u32 resv1;	/* 0x.004 - Reserved */
	u32 resv2;	/* 0x.008 - Reserved */
	u32 resv3;	/* 0x.00C - Reserved */
	u32 resv4;	/* 0x.010 - Reserved */
	u32 resv5;	/* 0x.014 - Reserved */
	u32 resv6;	/* 0x.018 - Reserved */
	u32 resv7;	/* 0x.01C - Reserved */
	u32 trstctl;	/* 0x.020 - TX Reset Control Register */
	u32 tgcr0;	/* 0x.024 - TX General Control Register 0 */
	u32 tgcr1;	/* 0x.028 - TX General Control Register 1 */
	u32 tgcr2;	/* 0x.02C - TX General Control Register 2 */
	u32 tecr0;	/* 0x.030 - Transmit Equalization Control Register 0 */
	u32 tecr1;	/* 0x.034 - Transmit Equalization Control Register 1 */
	u32 resv8;	/* 0x.038 - Reserved */
	u32 resv9;	/* 0x.03C - Reserved */
	u32 rrstctl;	/* 0x.040 - RX Reset Control Register */
	u32 rgcr0;	/* 0x.044 - RX General Control Register 0 */
	u32 rxgcr1; 	/* 0x.048 - RX General Control Register 1 */
	u32 resv10;	/* 0x.04C - Reserved */
	u32 recr0;	/* 0x.050 - RX Equalization Register 0 */
	u32 recr1;	/* 0x.054 - RX Equalization Register 1 */
	u32 recr2;	/* 0x.058 - RX Equalization Register 2 */
	u32 recr3;	/* 0x.05C - RX Equalization Register 3 */
	u32 recr4;	/* 0x.060 - RX Equalization Register 4 */
	u32 resv11;	/* 0x.064 - Reserved */
	u32 rccr0;	/* 0x.068 - RX Calibration Register 0 */
	u32 rccr1;	/* 0x.06C - RX Calibration Register 1 */
	u32 rcpcr0;	/* 0x.070 - RX Clock Path Register 0 */
	u32 rsccr0;	/* 0x.074 - RX Sampler Calibration Control Register 0 */
	u32 rsccr1;	/* 0x.078 - RX Sampler Calibration Control Register 1 */
	u32 resv12;	/* 0x.07C - Reserved */
	u32 ttlcr0;	/* 0x.080 - Transition Tracking Loop Register 0 */
	u32 ttlcr1;	/* 0x.084 - Transition Tracking Loop Register 1 */
	u32 ttlcr2;	/* 0x.088 - Transition Tracking Loop Register 2 */
	u32 ttlcr3;	/* 0x.08C - Transition Tracking Loop Register 3 */
	u32 resv13;	/* 0x.090 - Reserved */
	u32 resv14;	/* 0x.094 - Reserved */
	u32 resv15;	/* 0x.098 - Reserved */
	u32 resv16;	/* 0x.09C - Reserved */
	u32 tcsr0;	/* 0x.0A0 - Test Control/Status Register 0 */
	u32 tcsr1;	/* 0x.0A4 - Test Control/Status Register 1 */
	u32 tcsr2;	/* 0x.0A8 - Test Control/Status Register 2 */
	u32 tcsr3;	/* 0x.0AC - Test Control/Status Register 3 */
	u32 tcsr4;	/* 0x.0B0 - Test Control/Status Register 4 */
	u32 resv17;	/* 0x.0B4 - Reserved */
	u32 resv18;	/* 0x.0B8 - Reserved */
	u32 resv19;	/* 0x.0BC - Reserved */
	u32 rxcb0;	/* 0x.0C0 - RX Control Block Register 0 */
	u32 rxcb1;	/* 0x.0C4 - RX Control Block Register 1 */
	u32 resv20;	/* 0x.0C8 - Reserved */
	u32 resv21;	/* 0x.0CC - Reserved */
	u32 rxss0;	/* 0x.0D0 - RX Speed Switch Register 0 */
	u32 rxss1;	/* 0x.0D4 - RX Speed Switch Register 1 */
	u32 rxss2;	/* 0x.0D8 - RX Speed Switch Register 2 */
	u32 resv22;	/* 0x.0DC - Reserved */
	u32 txcb0;	/* 0x.0E0 - TX Control Block Register 0 */
	u32 txcb1;	/* 0x.0E4 - TX Control Block Register 1 */
	u32 resv23;	/* 0x.0E8 - Reserved */
	u32 resv24;	/* 0x.0EC - Reserved */
	u32 txss0;	/* 0x.0F0 - TX Speed Switch Register 0 */
	u32 txss1;	/* 0x.0F4 - TX Speed Switch Register 1 */
	u32 txss2;	/* 0x.0F8 - TX Speed Switch Register 2 */
	u32 resv25;	/* 0x.0FC - Reserved */
};

static struct serdes_access srds;

static u32 get_lane_memmap_size(void)
{
	return 0x100;
}

static int get_lane_id(u32 lane_addr)
{
	int i;
	for (i = 0; i < srds.lanes_no; i++) {
		if (srds.lanes_offsets[i] == lane_addr)
			return i;
	}
	return -1;
}

static void reset_lane(void *reg, enum lane_type ln_type)
{
	struct per_lane_ctrl_status *reg_base = reg;
	u32 val;
	unsigned long timeout;

	/* reset Tx lane: send reset request */
	if (ln_type & LANE_TX) {
		srds.iowrite32(srds.ioread32(&reg_base->trstctl) | RESET_REQ_MASK,
				&reg_base->trstctl);
		udelay(1);
		timeout = 10;
		while (timeout--) {
			val = srds.ioread32(&reg_base->trstctl);
			if (!(val & RESET_REQ_MASK))
				break;
			usleep_range(5, 20);
		}
	}
	
	/* reset Rx lane: send reset request */
	if (ln_type & LANE_RX) {
		srds.iowrite32(srds.ioread32(&reg_base->rrstctl) | RESET_REQ_MASK,
				&reg_base->rrstctl);
		udelay(1);
		timeout = 10;
		while (timeout--) {
			val = srds.ioread32(&reg_base->rrstctl);
			if (!(val & RESET_REQ_MASK))
				break;
			usleep_range(5, 20);
		}
	}
	
	/* wait for a while after reset */
	if (ln_type != LANE_INVALID) {
		timeout = jiffies + 10;
		while (time_before(jiffies, timeout)) {
			schedule();
			usleep_range(5, 20);
		}
	}
}

static u32 read_tecr0(void *reg)
{
	struct per_lane_ctrl_status *reg_base = reg;

	return srds.ioread32(&reg_base->tecr0);
}

static u32 read_tecr1(void *reg)
{
	struct per_lane_ctrl_status *reg_base = reg;

	return srds.ioread32(&reg_base->tecr1);
}

static void read_tecr_params(void *reg, struct tecr_params *params)
{
	struct per_lane_ctrl_status *reg_base = reg;
	u32 val;

	val = srds.ioread32(&reg_base->tecr0);
	params->ratio_preq = (val & RATIO_PREQ_MASK) >> RATIO_PREQ_SHIFT;
	params->ratio_pst1q = (val & RATIO_PST1Q_MASK) >> RATIO_PST1Q_SHIFT;
	params->amp_red = (val & AMP_RED_MASK) >> AMP_RED_SHIFT;

	val = srds.ioread32(&reg_base->tecr1);
	params->adpt_eq = (val & ADPT_EQ_MASK) >> ADPT_EQ_SHIFT;
}

static void tune_tecr(void *reg, struct tecr_params *params, bool reset)
{
	struct per_lane_ctrl_status *reg_base = reg;
	u32 val;

	if (reset) {
		/* reset lanes */
		reset_lane(reg, LANE_RX_TX);
	}
	
	val = TECR0_INIT |
		params->ratio_preq << RATIO_PREQ_SHIFT |
		params->ratio_pst1q << RATIO_PST1Q_SHIFT |
		params->amp_red << AMP_RED_SHIFT;
	srds.iowrite32(val, &reg_base->tecr0);

	val = params->adpt_eq << ADPT_EQ_SHIFT;
	srds.iowrite32(val, &reg_base->tecr1);
	
	udelay(1);
}

static void set_amp_red(void *reg, u32 amp_red)
{
	struct per_lane_ctrl_status *reg_base = reg;
	u32 val;

	val = srds.ioread32(&reg_base->tecr0);

	val |= (amp_red << AMP_RED_SHIFT);

	srds.iowrite32(val, &reg_base->tecr0);
	udelay(1);
}

static void lane_set_1gkx(void *reg)
{
	struct per_lane_ctrl_status *reg_base = reg;
	u32 val;

	/* reset lanes */
	reset_lane(reg, LANE_RX_TX);

	/* set gcr1 for 1GKX */
	val = srds.ioread32(&reg_base->rxgcr1);
	val &= ~(GCR1_REIDL_TH_MASK | GCR1_REIDL_EX_SEL_MASK |
		 GCR1_REIDL_ET_MAS_MASK);
	srds.iowrite32(val, &reg_base->rxgcr1);
	udelay(1);

	/* set tecr0 for 1GKX */
	val = srds.ioread32(&reg_base->tecr0);
	val &= ~AMP_RED_MASK;
	srds.iowrite32(val, &reg_base->tecr0);
	udelay(1);
}

static u8 get_full_gaink2(void)
{
	return 0x1F;
}

static u8 get_midrange_low_gaink(void)
{
	return 0x1;
}

static u8 get_midrange_high_gaink(void)
{
	return 0x1E;
}

static int get_median_gaink2(void *reg)
{
	int gaink2_snap_shot[BIN_SNAPSHOT_NUM];
	u32 rx_eq_snp;
	struct per_lane_ctrl_status *reg_base = reg;
	int timeout;
	int i, j, tmp, pos;

	/* Enable observation of SerDes status on all status registers */
	srds.iowrite32(srds.ioread32(&reg_base->tcsr0) | TCSR0_SD_STAT_OBS_EN_MASK,
		    &reg_base->tcsr0);

	for (i = 0; i < BIN_SNAPSHOT_NUM; i++) {
		/* wait RECR3_SNP_DONE_MASK has cleared */
		timeout = 100;
		while (srds.ioread32(&reg_base->recr3) &
				RECR3_SNP_DONE_MASK) {
			udelay(1);
			timeout--;
			if (timeout == 0)
				break;
		}

		/* start snap shot */
		srds.iowrite32((srds.ioread32(&reg_base->recr3) |
			    RECR3_SNP_START_MASK),
			    &reg_base->recr3);

		/* wait for SNP done */
		timeout = 100;
		while (!(srds.ioread32(&reg_base->recr3) &
				RECR3_SNP_DONE_MASK)) {
			udelay(1);
			timeout--;
			if (timeout == 0)
				break;
		}

		/* read and save the snap shot */
		rx_eq_snp = srds.ioread32(&reg_base->recr3);
		gaink2_snap_shot[i] = (rx_eq_snp & RECR3_GAINK2_MASK) >>
					RECR3_GAINK2_SHIFT;

		/* terminate the snap shot by setting GCR1[REQ_CTL_SNP] */
		srds.iowrite32((srds.ioread32(&reg_base->recr3) &
			    ~RECR3_SNP_START_MASK),
			    &reg_base->recr3);
	}

	/* get median of all collected snapshots */
	for (i = 0; i < BIN_SNAPSHOT_NUM - 1; i++) {
		tmp = gaink2_snap_shot[i];
		pos = i;
		for (j = i + 1; j < BIN_SNAPSHOT_NUM; j++) {
			if (gaink2_snap_shot[j] < tmp) {
				tmp = gaink2_snap_shot[j];
				pos = j;
			}
		}

		gaink2_snap_shot[pos] = gaink2_snap_shot[i];
		gaink2_snap_shot[i] = tmp;
	}

	return gaink2_snap_shot[2];
}

#if 0
static bool is_bin_early(enum bin_type bin_sel, void *reg)
{
	bool early = false;
	int bin_snap_shot[BIN_SNAPSHOT_NUM];
	int i, negative_count = 0;
	struct per_lane_ctrl_status *reg_base = reg;
	int timeout;

	/* Enable observation of SerDes status on all status registers */
	srds.iowrite32(srds.ioread32(&reg_base->tcsr0) | TCSR0_SD_STAT_OBS_EN_MASK,
		    &reg_base->tcsr0);

	for (i = 0; i < BIN_SNAPSHOT_NUM; i++) {
		/* wait RECR3_SNP_DONE_MASK has cleared */
		timeout = 100;
		while ((srds.ioread32(&reg_base->recr3) & RECR3_SNP_DONE_MASK)) {
			udelay(1);
			timeout--;
			if (timeout == 0)
				break;
		}

		/* set TCSR1[CDR_SEL] to BinM1/BinLong */
		if (bin_sel == BIN_M1) {
			srds.iowrite32((srds.ioread32(&reg_base->recr4) &
				    ~CDR_SEL_MASK) | BIN_M1_SEL,
				    &reg_base->recr4);
		} else {
			srds.iowrite32((srds.ioread32(&reg_base->recr4) &
				    ~CDR_SEL_MASK) | BIN_LONG_SEL,
				    &reg_base->recr4);
		}

		/* start snap shot */
		srds.iowrite32(srds.ioread32(&reg_base->recr3) | RECR3_SNP_START_MASK,
			    &reg_base->recr3);

		/* wait for SNP done */
		timeout = 100;
		while (!(srds.ioread32(&reg_base->recr3) & RECR3_SNP_DONE_MASK)) {
			udelay(1);
			timeout--;
			if (timeout == 0)
				break;
		}

		/* read and save the snap shot */
		bin_snap_shot[i] = (srds.ioread32(&reg_base->recr4) &
				RECR4_SNP_DATA_MASK) >> RECR4_SNP_DATA_SHIFT;
		if (bin_snap_shot[i] & RECR4_EQ_SNPBIN_SIGN_MASK)
			negative_count++;

		/* terminate the snap shot by setting GCR1[REQ_CTL_SNP] */
		srds.iowrite32(srds.ioread32(&reg_base->recr3) & ~RECR3_SNP_START_MASK,
			    &reg_base->recr3);
	}

	if (((bin_sel == BIN_M1) && (negative_count > BIN_M1_THRESHOLD)) ||
	    ((bin_sel == BIN_LONG && (negative_count > BIN_LONG_THRESHOLD)))) {
		early = true;
	}

	return early;
}
#endif

static int collect_gains(void *reg, u8 *gaink2, u8 *gaink3, u8 *eq_offset)
{
	u32 recr3, recr4;
	struct per_lane_ctrl_status *reg_base = reg;
	int timeout;
	int i;

	/* Enable observation of SerDes status on all status registers */
	srds.iowrite32(srds.ioread32(&reg_base->tcsr0) | TCSR0_SD_STAT_OBS_EN_MASK,
		    &reg_base->tcsr0);

	for (i = 0; i < BIN_SNAPSHOT_NUM; i++) {
		/* wait RECR3_SNP_DONE_MASK has cleared */
		timeout = 100;
		while (srds.ioread32(&reg_base->recr3) &
				RECR3_SNP_DONE_MASK) {
			udelay(1);
			timeout--;
			if (timeout == 0)
				break;
		}

		/* start snap shot */
		srds.iowrite32((srds.ioread32(&reg_base->recr3) |
			    RECR3_SNP_START_MASK),
			    &reg_base->recr3);

		/* wait for SNP done */
		timeout = 100;
		while (!(srds.ioread32(&reg_base->recr3) &
				RECR3_SNP_DONE_MASK)) {
			udelay(1);
			timeout--;
			if (timeout == 0)
				break;
		}

		/* read and save the snap shot */
		recr3 = srds.ioread32(&reg_base->recr3);
		recr4 = srds.ioread32(&reg_base->recr4);

		gaink2[i] = (u8)((recr3 & RECR3_GAINK2_MASK) >>
				RECR3_GAINK2_SHIFT);
		gaink3[i] = (u8)((recr3 & RECR3_GAINK3_MASK) >>
				RECR3_GAINK3_SHIFT);
		//gaink4[i] = (u8)((recr3 & RECR3_GAINK4_MASK) >>
		//		RECR3_GAINK4_SHIFT);
		eq_offset[i] = (u8)((recr4 & RECR4_EQ_OFFSET_MASK) >>
				RECR4_EQ_OFFSET_SHIFT);

		/* terminate the snap shot by setting GCR1[REQ_CTL_SNP] */
		srds.iowrite32((srds.ioread32(&reg_base->recr3) &
			    ~RECR3_SNP_START_MASK),
			    &reg_base->recr3);
	}
	return i;
}

static int collect_bin_snapshots(enum bin_type bin_type, void *reg, s16 *bin_snapshots)
{
	int bin_snapshot;
	u32 bin_sel;
	int i, timeout;
	struct per_lane_ctrl_status *reg_base = reg;

	/* calculate RECR4[EQ_BIN_DATA_SEL] */
	switch (bin_type) {
	case BIN_1:
		bin_sel = BIN_1_SEL;
		break;
	case BIN_2:
		bin_sel = BIN_2_SEL;
		break;
	case BIN_3:
		bin_sel = BIN_3_SEL;
		break;
	case BIN_4:
		bin_sel = BIN_4_SEL;
		break;
	case BIN_OFFSET:
		bin_sel = BIN_OFFSET_SEL;
		break;
	case BIN_M1:
		bin_sel = BIN_M1_SEL;
		break;
	case BIN_LONG:
		bin_sel = BIN_LONG_SEL;
		break;
	default:
		/* invalid bin type */
		return 0;
	}

	/* Enable observation of SerDes status on all status registers */
	srds.iowrite32(srds.ioread32(&reg_base->tcsr0) | TCSR0_SD_STAT_OBS_EN_MASK,
		    &reg_base->tcsr0);

	for (i = 0; i < BIN_SNAPSHOT_NUM; i++) {
		/* wait RECR3_SNP_DONE_MASK has cleared */
		timeout = 100;
		while ((srds.ioread32(&reg_base->recr3) & RECR3_SNP_DONE_MASK)) {
			udelay(1);
			timeout--;
			if (timeout == 0)
				break;
		}

		/* set RECR4[EQ_BIN_DATA_SEL] */
		srds.iowrite32((srds.ioread32(&reg_base->recr4) &
			    ~CDR_SEL_MASK) | bin_sel,
			    &reg_base->recr4);

		/* start snap shot */
		srds.iowrite32(srds.ioread32(&reg_base->recr3) | RECR3_SNP_START_MASK,
			    &reg_base->recr3);

		/* wait for SNP done */
		timeout = 100;
		while (!(srds.ioread32(&reg_base->recr3) & RECR3_SNP_DONE_MASK)) {
			udelay(1);
			timeout--;
			if (timeout == 0)
				break;
		}

		/* read and save the snap shot: 2's complement 9bit long value (-256 to 255) */
		bin_snapshot = (srds.ioread32(&reg_base->recr4) &
				RECR4_SNP_DATA_MASK) >> RECR4_SNP_DATA_SHIFT;
		if (bin_snapshot & RECR4_EQ_SNPBIN_SIGN_MASK) {
			//2's complement 10bit long negative number
			bin_snapshot &= ~RECR4_EQ_SNPBIN_SIGN_MASK;
			bin_snapshot -= 256;
		}

		/* save collected Bin snapshot */
		bin_snapshots[i] = (s16)bin_snapshot;

		/* terminate the snap shot by setting GCR1[REQ_CTL_SNP] */
		srds.iowrite32(srds.ioread32(&reg_base->recr3) & ~RECR3_SNP_START_MASK,
			    &reg_base->recr3);
	}
	return i;
}

static enum bin_state get_bin_snapshots_state(s16 *bin_snapshots)
{
	int i;
	s16 snapshot_average, snapshot_sum = 0;

	for (i = 0; i < BIN_SNAPSHOT_NUM; i++) {
		snapshot_sum += bin_snapshots[i];
	}
	snapshot_average = (s16)(snapshot_sum / BIN_SNAPSHOT_NUM);

	if (snapshot_average >= -256 && snapshot_average < BIN_SNP_AV_THR_LOW)
		return BIN_EARLY;
	else if (snapshot_average >= BIN_SNP_AV_THR_LOW && snapshot_average < BIN_SNP_AV_THR_HIGH)
		return BIN_TOGGLE;
	else if (snapshot_average >= BIN_SNP_AV_THR_HIGH && snapshot_average <= 255)
		return BIN_LATE;

	return BIN_INVALID;
}

static bool is_cdr_lock(void *reg)
{
	struct per_lane_ctrl_status *reg_base = reg;

	if (srds.ioread32(&reg_base->rrstctl) & RRSTCTL_CDR_LOCK_MASK)
		return true;

	return false;
}

struct serdes_access* setup_serdes_access_28g(void)
{
	memset(&srds, 0, sizeof(srds));
	srds.get_lane_memmap_size = get_lane_memmap_size;
	srds.get_lane_id = get_lane_id;
	srds.tune_tecr = tune_tecr;
	srds.set_amp_red = set_amp_red;
	srds.read_tecr0 = read_tecr0;
	srds.read_tecr1 = read_tecr1;
	srds.read_tecr_params = read_tecr_params;
	srds.reset_lane = reset_lane;
	srds.lane_set_1gkx = lane_set_1gkx;
	srds.get_full_gaink2 = get_full_gaink2;
	srds.get_midrange_low_gaink = get_midrange_low_gaink;
	srds.get_midrange_high_gaink = get_midrange_high_gaink;
	srds.get_median_gaink2 = get_median_gaink2;
	//srds.is_bin_early = is_bin_early;
	srds.collect_gains = collect_gains;
	srds.collect_bin_snapshots = collect_bin_snapshots;
	srds.get_bin_snapshots_state = get_bin_snapshots_state;
	srds.is_cdr_lock = is_cdr_lock;

	return &srds;
}
