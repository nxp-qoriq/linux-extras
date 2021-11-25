/* SPDX-License-Identifier: GPL-2.0+ */
/*
 *  DPAA Backplane driver.
 *
 * Copyright 2018-2019, 2021 NXP
 */

#ifndef FSL_BACKPLANE_H
#define FSL_BACKPLANE_H

#include <linux/phy.h>
#include <linux/mutex.h>

#include "fsl_backplane_debugfs.h"

#define FSL_BACKPLANE_DRIVER_NAME	"fsl_backplane"

/* Collect data from 10 snapshots  (increase from 5) */
#define BIN_SNAPSHOT_NUM			10
#define BIN_M1_THRESHOLD			3
#define BIN_LONG_THRESHOLD			2

#define MAX_PHY_LANES_NO			4
#define MAX_LANES_NO				8

enum bin_type {
	BIN_1,
	BIN_2,
	BIN_3,
	BIN_4,
	BIN_OFFSET,
	BIN_BLW,
	BIN_DATA_AVG,
	BIN_M1,
	BIN_LONG
};

enum bin_state {
	BIN_INVALID,
	BIN_EARLY,
	BIN_TOGGLE,
	BIN_LATE
};

enum train_state {
	DETECTING_LP,
	TRAINED,
};

enum lane_type {
	LANE_INVALID,
	LANE_RX,
	LANE_TX,
	LANE_RX_TX
};

struct tecr_params {
	u32 ratio_preq;
	u32 ratio_pst1q;
	u32 adpt_eq;
	u32 amp_red;
};

struct serdes_access {
	int serdes_type;
	bool is_little_endian;
	int lanes_no;
	u32 lanes_offsets[MAX_LANES_NO];
	u32 (*ioread32)(u32 *reg);
	void (*iowrite32)(u32 value, u32 *reg);
	u32 (*get_lane_memmap_size)(void);
	int (*get_lane_id)(u32 lane_addr);
	void (*tune_tecr)(void *reg, struct tecr_params *params, bool reset);
	void (*set_amp_red)(void *reg, u32 amp_red);
	u32 (*read_tecr0)(void *reg);
	u32 (*read_tecr1)(void *reg);
	void (*read_tecr_params)(void *reg, struct tecr_params *params);
	void (*reset_lane)(void *reg, enum lane_type ln_type);
	void (*lane_set_1gkx)(void *reg);
	u8 (*get_full_gaink2)(void);
	u8 (*get_midrange_low_gaink)(void);
	u8 (*get_midrange_high_gaink)(void);
	int (*get_median_gaink2)(void *reg);
	int (*collect_gains)(void *reg, u8 *gaink2, u8 *gaink3, u8 *eq_offset);
	//bool (*is_bin_early)(enum bin_type bin_sel, void *reg);
	int (*collect_bin_snapshots)(enum bin_type bin_type, void *reg, s16 *bin_snapshots);
	enum bin_state (*get_bin_snapshots_state)(s16 *bin_snapshots);
	bool (*is_cdr_lock)(void *reg);
};

struct train_status {
	bool bin_m1_stop;
	bool bin_long_stop;
	bool done_training;
	bool remote_tx_complete;
	bool remote_tx_running;
	bool sent_init;
	int lp_rx_ready;
	bool local_tx_running;
	int m1_min_max_cnt;
	int long_min_max_cnt;
};

struct xgkr_stats {
	u32 init_tecr0;
	u32 init_tecr1;
	u32 tuned_tecr0;
	u32 tuned_tecr1;
	u32 training_steps;
	u32 training_started_count;
	u32 training_failed_count;
	u32 training_timeouts;
	u32 training_cycles_remote_tx; /* total cycles for all training steps */
	u32 training_cycles_local_tx;  /* total cycles for all training steps */
	u32 lt_start; 	 /* training start time */
	u32 lt_finish; 	 /* training finish time */
	u32 lt_duration; /* total duration for all steps (in msec) */
	u32 coe_updates_to_lp;
	u32 coe_updates_from_lp;
	u32 inc_coe_count[3];
	u32 dec_coe_count[3];
	u32 ld_preset_count;
	u32 ld_init_count;
};

struct xgkr_phy_stats {
	u32 lp_detected;
	u32 aneg_restarted_count;
	u32 last_status_pcs_link_up;
	u32 pcs_link_lost_count;
	u32 last_status_an_link_up;
	u32 an_link_lost_count;
	u32 lt_start; 	 /* training start time */
	u32 lt_finish; 	 /* training finish time */
	u32 lt_duration; /* total training duration (in msec) */
};

struct xgkr_params {
	void *reg_base;		/* lane memory map: registers base address */
	int lane_id;		/* lane id: 0 = Lane H, 1 = Lane G, ... 7 = Lane A */
	int idx;			/* lane relative index inside a multi-lane PHY */
	struct phy_device *phydev;
	struct serdes_access *srds;
	struct train_status trst;
	struct delayed_work xgkr_wk;
	enum train_state state;
	//----------------------------------
	// New algorithm
	enum bin_state bin_m1_state;
	enum bin_state bin_long_state;
	enum bin_state prev_bin_m1_state;
	enum bin_state prev_bin_long_state;
	u32 prev_ld_update;
	u32 prev_ld_last_nonhold_update;
	u32 prev_alg_ld_update;
	u32 lp_status;
	u32 lp_last_nonzero_status;
	bool lt_error;
	bool move_back_prev;
	int move_back_cnt;
	u32 move_back_lp_status;
	u32 req_ld_update_init_count;
	u32 repeat_request_count;
	unsigned long init_handshake_time;
	bool first_recv_init;
	bool an_acquired;
	//----------------------------------
	struct mutex lane_lock;
	int an_wait_count;
	unsigned long rt_time;
	u32 ld_update;
	u32 ld_status;
	u32 ratio_preq;
	u32 ratio_pst1q;
	u32 adpt_eq;
	u32 def_ratio_preq;
	u32 def_ratio_pst1q;
	u32 def_adpt_eq;
	u32 def_amp_red;
	u32 tuned_ratio_preq;
	u32 tuned_ratio_pst1q;
	u32 tuned_adpt_eq;
	/* debugfs setup */
	u32 set_ratio_preq;
	u32 set_ratio_pst1q;
	u32 set_adpt_eq;
	u32 set_amp_red;
	u8 set_applied;
	u8 training_disabled;
	u8 local_tx_apply_req_disabled;
	u8 remote_tx_req_update_disabled;
	u8 hw_restrictions_disabled;

	/* Bin snapshots */
	s16 bin1_snapshot[BIN_SNAPSHOT_NUM];
	s16 bin2_snapshot[BIN_SNAPSHOT_NUM];
	s16 bin3_snapshot[BIN_SNAPSHOT_NUM];
	s16 bin_offset_snapshot[BIN_SNAPSHOT_NUM];
	s16 binM1_snapshot[BIN_SNAPSHOT_NUM];
	s16 binLong_snapshot[BIN_SNAPSHOT_NUM];

	/* Gain snapshots */
	u8 gaink2_snapshot[BIN_SNAPSHOT_NUM];
	u8 gaink3_snapshot[BIN_SNAPSHOT_NUM];
	u8 osestat_snapshot[BIN_SNAPSHOT_NUM];

	/* Lane Statistics */
	struct xgkr_stats stats;

	/* Lane Trace */
	char base_operation[80];

#ifdef CONFIG_FSL_BACKPLANE_DEBUGFS
	struct xgkr_debugfs dbg;
#endif /* CONFIG_FSL_BACKPLANE_DEBUGFS */

};

struct xgkr_phy_data {
	int bp_mode;
	u32 phy_lanes;
	struct mutex phydev_lock;
	struct mutex phy_trained_lock;
	bool aneg_config;
	bool aneg_done;
	bool phy_suspended;
	struct xgkr_params xgkr[MAX_PHY_LANES_NO];

	/* debugfs setup */
	u8 force_retrained;

	/* Phy Statistics */
	struct xgkr_phy_stats stats;

	/* Phy Trace */
	char base_operation[80];

#ifdef CONFIG_FSL_BACKPLANE_DEBUGFS
	struct xgkr_phy_debugfs dbg;
#endif /* CONFIG_FSL_BACKPLANE_DEBUGFS */
};

/* Serdes access API */
struct serdes_access* setup_serdes_access_10g(void);
struct serdes_access* setup_serdes_access_28g(void);

/* Backplane API (used by debugfs) */
void force_kr_setup(struct xgkr_params *xgkr);
void force_amp_red(struct xgkr_params *xgkr);
void force_restart_training(struct xgkr_phy_data *xgkr_inst);

#endif //FSL_BACKPLANE_H
