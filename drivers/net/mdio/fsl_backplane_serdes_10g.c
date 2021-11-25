// SPDX-License-Identifier: GPL-2.0+
/*
 *  DPAA Backplane driver for SerDes 10G.
 *
 * Copyright 2018-2019, 2021 NXP
 */

#include <linux/io.h>
#include <linux/delay.h>

#include "fsl_backplane.h"

#define BIN_1_SEL					0x00000000
#define BIN_2_SEL					0x00010000
#define BIN_3_SEL					0x00020000
#define BIN_OFFSET_SEL				0x00030000
#define BIN_BLW_SEL					0x00040000
#define BIN_DATA_AVG_SEL			0x00050000
#define BIN_M1_SEL					0x00060000
#define BIN_LONG_SEL				0x00070000
#define CDR_SEL_MASK				0x00070000

#define BIN_SNP_AV_THR_LOW			-150
#define BIN_SNP_AV_THR_HIGH			150

#define RATIO_PREQ_SHIFT			22
#define RATIO_PST1Q_SHIFT			16
#define ADPT_EQ_SHIFT				8
#define AMP_RED_SHIFT				0

#define RATIO_PREQ_MASK				0x03c00000
#define RATIO_PST1Q_MASK			0x001f0000
#define ADPT_EQ_MASK				0x00003f00
#define AMP_RED_MASK				0x0000003f

#define TECR0_INIT					0x24200000

#define GCR0_RESET_MASK				0x00600000
#define GCR0_TRST_MASK				0x00200000
#define GCR0_RRST_MASK				0x00400000

#define GCR1_SNP_START_MASK			0x00000040
#define GCR1_CTL_SNP_START_MASK		0x00002000

#define RECR1_CTL_SNP_DONE_MASK		0x00000002
#define RECR1_SNP_DONE_MASK			0x00000004
#define TCSR1_SNP_DATA_MASK			0x00007fc0
#define TCSR1_SNP_DATA_SHIFT		6
#define TCSR1_EQ_SNPBIN_SIGN_MASK	0x100

#define TCSR3_CDR_LCK_MASK			0x08000000

#define RECR1_GAINK2_MASK			0x0f000000
#define RECR1_GAINK2_SHIFT			24

#define RECR1_GAINK3_MASK			0x000f0000
#define RECR1_GAINK3_SHIFT			16

/* the algorithm should only be looking at offset_stat[5:0]
 * [6] is only used at higher bit rates to adjust the overall range of the internal offset DAC */
#define RECR1_EQ_OFFSET_MASK		0x00001f80
#define RECR1_EQ_OFFSET_SHIFT		7

#define RECR1_HAPPSTAT_MASK			0x00000008
#define RECR1_HAPPSTAT_SHIFT		3

/* Required only for 1000BASE KX */
#define GCR1_REIDL_TH_MASK			0x00700000
#define GCR1_REIDL_EX_SEL_MASK		0x000c0000
#define GCR1_REIDL_ET_MAS_MASK		0x00004000

struct per_lane_ctrl_status {
	u32 gcr0;	/* 0x.000 - General Control Register 0 */
	u32 gcr1;	/* 0x.004 - General Control Register 1 */
	u32 gcr2;	/* 0x.008 - General Control Register 2 */
	u32 resv1;	/* 0x.00C - Reserved */
	u32 recr0;	/* 0x.010 - Receive Equalization Control Register 0 */
	u32 recr1;	/* 0x.014 - Receive Equalization Control Register 1 */
	u32 tecr0;	/* 0x.018 - Transmit Equalization Control Register 0 */
	u32 resv2;	/* 0x.01C - Reserved */
	u32 tlcr0;	/* 0x.020 - TTL Control Register 0 */
	u32 tlcr1;	/* 0x.024 - TTL Control Register 1 */
	u32 tlcr2;	/* 0x.028 - TTL Control Register 2 */
	u32 tlcr3;	/* 0x.02C - TTL Control Register 3 */
	u32 tcsr0;	/* 0x.030 - Test Control/Status Register 0 */
	u32 tcsr1;	/* 0x.034 - Test Control/Status Register 1 */
	u32 tcsr2;	/* 0x.038 - Test Control/Status Register 2 */
	u32 tcsr3;	/* 0x.03C - Test Control/Status Register 3 */
};

static struct serdes_access srds;

static u32 get_lane_memmap_size(void)
{
	return 0x40;
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

	/* reset Tx lane: send reset request */
	if (ln_type & LANE_TX) {
		srds.iowrite32(srds.ioread32(&reg_base->gcr0) & ~GCR0_TRST_MASK,
			    &reg_base->gcr0);
	}
	/* reset Rx lane: send reset request */
	if (ln_type & LANE_RX) {
		srds.iowrite32(srds.ioread32(&reg_base->gcr0) & ~GCR0_RRST_MASK,
			    &reg_base->gcr0);
	}
	/* unreset the lane */
	if (ln_type != LANE_INVALID) {
		udelay(1);
		srds.iowrite32(srds.ioread32(&reg_base->gcr0) | GCR0_RESET_MASK,
				&reg_base->gcr0);
		udelay(1);
	}
}

static u32 read_tecr0(void *reg)
{
	struct per_lane_ctrl_status *reg_base = reg;

	return srds.ioread32(&reg_base->tecr0);
}

static u32 read_tecr1(void *reg)
{
	return 0;
}

static void read_tecr_params(void *reg, struct tecr_params *params)
{
	struct per_lane_ctrl_status *reg_base = reg;
	u32 val;

	val = srds.ioread32(&reg_base->tecr0);

	params->ratio_preq = (val & RATIO_PREQ_MASK) >> RATIO_PREQ_SHIFT;
	params->ratio_pst1q = (val & RATIO_PST1Q_MASK) >> RATIO_PST1Q_SHIFT;
	params->adpt_eq = (val & ADPT_EQ_MASK) >> ADPT_EQ_SHIFT;
	params->amp_red = (val & AMP_RED_MASK) >> AMP_RED_SHIFT;
}

static void tune_tecr(void *reg, struct tecr_params *params, bool reset)
{
	struct per_lane_ctrl_status *reg_base = reg;
	u32 val;

	val = TECR0_INIT |
		params->adpt_eq << ADPT_EQ_SHIFT |
		params->ratio_preq << RATIO_PREQ_SHIFT |
		params->ratio_pst1q << RATIO_PST1Q_SHIFT |
		params->amp_red << AMP_RED_SHIFT;

	if (reset) {
		/* reset the lane */
		srds.iowrite32(srds.ioread32(&reg_base->gcr0) & ~GCR0_RESET_MASK,
				&reg_base->gcr0);
		udelay(1);
	}
	
	srds.iowrite32(val, &reg_base->tecr0);
	udelay(1);
	
	if (reset) {
		/* unreset the lane */
		srds.iowrite32(srds.ioread32(&reg_base->gcr0) | GCR0_RESET_MASK,
				&reg_base->gcr0);
		udelay(1);
	}
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

	/* reset the lane */
	srds.iowrite32(srds.ioread32(&reg_base->gcr0) & ~GCR0_RESET_MASK,
		    &reg_base->gcr0);
	udelay(1);

	/* set gcr1 for 1GKX */
	val = srds.ioread32(&reg_base->gcr1);
	val &= ~(GCR1_REIDL_TH_MASK | GCR1_REIDL_EX_SEL_MASK |
		 GCR1_REIDL_ET_MAS_MASK);
	srds.iowrite32(val, &reg_base->gcr1);
	udelay(1);

	/* set tecr0 for 1GKX */
	val = srds.ioread32(&reg_base->tecr0);
	val &= ~AMP_RED_MASK;
	srds.iowrite32(val, &reg_base->tecr0);
	udelay(1);

	/* unreset the lane */
	srds.iowrite32(srds.ioread32(&reg_base->gcr0) | GCR0_RESET_MASK,
		    &reg_base->gcr0);
	udelay(1);
}

static u8 get_full_gaink2(void)
{
	return 0xF;
}

static u8 get_midrange_low_gaink(void)
{
	return 0x1;
}

static u8 get_midrange_high_gaink(void)
{
	return 0xE;
}

static int get_median_gaink2(void *reg)
{
	int gaink2_snap_shot[BIN_SNAPSHOT_NUM];
	u32 rx_eq_snp;
	struct per_lane_ctrl_status *reg_base = reg;
	int timeout;
	int i, j, tmp, pos;

	for (i = 0; i < BIN_SNAPSHOT_NUM; i++) {
		/* wait RECR1_CTL_SNP_DONE_MASK has cleared */
		timeout = 100;
		while (srds.ioread32(&reg_base->recr1) &
		       RECR1_CTL_SNP_DONE_MASK) {
			udelay(1);
			timeout--;
			if (timeout == 0)
				break;
		}

		/* start snap shot */
		srds.iowrite32((srds.ioread32(&reg_base->gcr1) |
			    GCR1_CTL_SNP_START_MASK),
			    &reg_base->gcr1);

		/* wait for SNP done */
		timeout = 100;
		while (!(srds.ioread32(&reg_base->recr1) &
		       RECR1_CTL_SNP_DONE_MASK)) {
			udelay(1);
			timeout--;
			if (timeout == 0)
				break;
		}

		/* read and save the snap shot */
		rx_eq_snp = srds.ioread32(&reg_base->recr1);
		gaink2_snap_shot[i] = (rx_eq_snp & RECR1_GAINK2_MASK) >>
					RECR1_GAINK2_SHIFT;

		/* terminate the snap shot by setting GCR1[REQ_CTL_SNP] */
		srds.iowrite32((srds.ioread32(&reg_base->gcr1) &
			    ~GCR1_CTL_SNP_START_MASK),
			    &reg_base->gcr1);
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

	for (i = 0; i < BIN_SNAPSHOT_NUM; i++) {
		/* wait RECR1_SNP_DONE_MASK has cleared */
		timeout = 100;
		while ((srds.ioread32(&reg_base->recr1) & RECR1_SNP_DONE_MASK)) {
			udelay(1);
			timeout--;
			if (timeout == 0)
				break;
		}

		/* set TCSR1[CDR_SEL] to BinM1/BinLong */
		if (bin_sel == BIN_M1) {
			srds.iowrite32((srds.ioread32(&reg_base->tcsr1) &
				    ~CDR_SEL_MASK) | BIN_M1_SEL,
				    &reg_base->tcsr1);
		} else {
			srds.iowrite32((srds.ioread32(&reg_base->tcsr1) &
				    ~CDR_SEL_MASK) | BIN_LONG_SEL,
				    &reg_base->tcsr1);
		}

		/* start snap shot */
		srds.iowrite32(srds.ioread32(&reg_base->gcr1) | GCR1_SNP_START_MASK,
			    &reg_base->gcr1);

		/* wait for SNP done */
		timeout = 100;
		while (!(srds.ioread32(&reg_base->recr1) & RECR1_SNP_DONE_MASK)) {
			udelay(1);
			timeout--;
			if (timeout == 0)
				break;
		}

		/* read and save the snap shot */
		bin_snap_shot[i] = (srds.ioread32(&reg_base->tcsr1) &
				TCSR1_SNP_DATA_MASK) >> TCSR1_SNP_DATA_SHIFT;
		if (bin_snap_shot[i] & TCSR1_EQ_SNPBIN_SIGN_MASK)
			negative_count++;

		/* terminate the snap shot by setting GCR1[REQ_CTL_SNP] */
		srds.iowrite32(srds.ioread32(&reg_base->gcr1) & ~GCR1_SNP_START_MASK,
			    &reg_base->gcr1);
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
	u32 rx_eq_snp;
	struct per_lane_ctrl_status *reg_base = reg;
	int timeout;
	int i;

	for (i = 0; i < BIN_SNAPSHOT_NUM; i++) {
		/* wait RECR1_CTL_SNP_DONE_MASK has cleared */
		timeout = 100;
		while (srds.ioread32(&reg_base->recr1) &
		       RECR1_CTL_SNP_DONE_MASK) {
			udelay(1);
			timeout--;
			if (timeout == 0)
				break;
		}

		/* start snap shot */
		srds.iowrite32((srds.ioread32(&reg_base->gcr1) |
			    GCR1_CTL_SNP_START_MASK),
			    &reg_base->gcr1);

		/* wait for SNP done */
		timeout = 100;
		while (!(srds.ioread32(&reg_base->recr1) &
		       RECR1_CTL_SNP_DONE_MASK)) {
			udelay(1);
			timeout--;
			if (timeout == 0)
				break;
		}

		/* read and save the snap shot */
		rx_eq_snp = srds.ioread32(&reg_base->recr1);
		gaink2[i] = (u8)((rx_eq_snp & RECR1_GAINK2_MASK) >>
					RECR1_GAINK2_SHIFT);
		gaink3[i] = (u8)((rx_eq_snp & RECR1_GAINK3_MASK) >>
					RECR1_GAINK3_SHIFT);
		eq_offset[i] = (u8)((rx_eq_snp & RECR1_EQ_OFFSET_MASK) >>
					RECR1_EQ_OFFSET_SHIFT);
		//happy_stat[i] = (u8)((rx_eq_snp & RECR1_HAPPSTAT_MASK) >>
		//		RECR1_HAPPSTAT_SHIFT);

		/* terminate the snap shot by setting GCR1[REQ_CTL_SNP] */
		srds.iowrite32((srds.ioread32(&reg_base->gcr1) &
			    ~GCR1_CTL_SNP_START_MASK),
			    &reg_base->gcr1);
	}
	return i;
}

static int collect_bin_snapshots(enum bin_type bin_type, void *reg, s16 *bin_snapshots)
{
	int bin_snapshot;
	u32 bin_sel;
	int i, timeout;
	struct per_lane_ctrl_status *reg_base = reg;

	/* calculate TCSR1[CDR_SEL] */
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

	for (i = 0; i < BIN_SNAPSHOT_NUM; i++) {
		/* wait RECR1_SNP_DONE_MASK has cleared */
		timeout = 100;
		while ((srds.ioread32(&reg_base->recr1) & RECR1_SNP_DONE_MASK)) {
			udelay(1);
			timeout--;
			if (timeout == 0)
				break;
		}

		/* set TCSR1[CDR_SEL] */
		srds.iowrite32((srds.ioread32(&reg_base->tcsr1) &
			    ~CDR_SEL_MASK) | bin_sel,
			    &reg_base->tcsr1);

		/* start snap shot */
		srds.iowrite32(srds.ioread32(&reg_base->gcr1) | GCR1_SNP_START_MASK,
			    &reg_base->gcr1);

		/* wait for SNP done */
		timeout = 100;
		while (!(srds.ioread32(&reg_base->recr1) & RECR1_SNP_DONE_MASK)) {
			udelay(1);
			timeout--;
			if (timeout == 0)
				break;
		}

		/* read and save the snap shot: 2's complement 9bit long value (-256 to 255) */
		bin_snapshot = (srds.ioread32(&reg_base->tcsr1) &
				TCSR1_SNP_DATA_MASK) >> TCSR1_SNP_DATA_SHIFT;
		if (bin_snapshot & TCSR1_EQ_SNPBIN_SIGN_MASK) {
			//2's complement 9bit long negative number
			bin_snapshot &= ~TCSR1_EQ_SNPBIN_SIGN_MASK;
			bin_snapshot -= 256;
		}

		/* save collected Bin snapshot */
		bin_snapshots[i] = (s16)bin_snapshot;

		/* terminate the snap shot by setting GCR1[REQ_CTL_SNP] */
		srds.iowrite32(srds.ioread32(&reg_base->gcr1) & ~GCR1_SNP_START_MASK,
			    &reg_base->gcr1);
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

	if (srds.ioread32(&reg_base->tcsr3) & TCSR3_CDR_LCK_MASK)
		return true;

	return false;
}

struct serdes_access* setup_serdes_access_10g(void)
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

