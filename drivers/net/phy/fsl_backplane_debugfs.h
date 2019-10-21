/* SPDX-License-Identifier: GPL-2.0+ */
/*
 *  DPAA Backplane debugfs
 *
 * Copyright 2019 NXP
 */

#ifndef FSL_BACKPLANE_DEBUGFS_H
#define FSL_BACKPLANE_DEBUGFS_H

#include <linux/dcache.h>

struct xgkr_debugfs {
	struct dentry *dir;
	struct dentry *train_params;
	struct dentry *tuned_params;
	struct dentry *set_preq;
	struct dentry *set_pstq;
	struct dentry *set_adpteq;
	struct dentry *set_apply;
	struct dentry *set_ampred;
	struct dentry *cfg;
};

struct xgkr_phy_debugfs {
	struct dentry *dir;
	struct dentry *cmd;
};

#ifdef CONFIG_FSL_BACKPLANE_DEBUGFS
void backplane_dbg_init(void);
void backplane_dbg_exit(void);
void backplane_dbg_add(struct phy_device *phydev);
void backplane_dbg_remove(struct phy_device *phydev);
#else
static inline void backplane_dbg_init(void) {}
static inline void backplane_dbg_exit(void) {}
static inline void backplane_dbg_add(void *phydev) {}
static inline void backplane_dbg_remove(void *phydev) {}
#endif /* CONFIG_FSL_BACKPLANE_DEBUGFS */

#endif /* FSL_BACKPLANE_DEBUGFS_H */
