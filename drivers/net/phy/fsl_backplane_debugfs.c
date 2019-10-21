// SPDX-License-Identifier: GPL-2.0+
/*
 *  DPAA Backplane debugfs
 *
 * Copyright 2019 NXP
 */

#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/netdevice.h>

#include "fsl_backplane.h"

#ifdef CONFIG_FSL_BACKPLANE_DEBUGFS

#define FSL_BACKPLANE_DBG_ROOT "fsl_backplane"

static struct dentry *fsl_backplane_dbg_root = NULL;

/*
documentation at: https://www.tldp.org/LDP/lkmpg/2.4/html/c577.htm
*/

//---------------------------------------------------------------------------------
// show current params
//
static int xgkr_dbg_train_params_show(struct seq_file *file, void *offset)
{
	struct xgkr_params *xgkr = (struct xgkr_params *)file->private;

	//seq_printf(file, "Training parameters for %s lane %d:\n", dev_name(xgkr->phydev->attached_dev->dev.parent), idx);
	seq_printf(file, "%16s%16s%16s\n",
		   "ratio_preq", "ratio_pst1q", "adpt_eq");

	seq_printf(file, "%16u%16u%16u\n",
			xgkr->ratio_preq,
			xgkr->ratio_pst1q,
			xgkr->adpt_eq);

	return 0;
}

static int xgkr_dbg_train_params_open(struct inode *inode, struct file *file)
{
	int err;
	struct xgkr_params *xgkr = (struct xgkr_params *)inode->i_private;

	err = single_open(file, xgkr_dbg_train_params_show, xgkr);
	if (err < 0)
		netdev_err(xgkr->phydev->attached_dev, "single_open() failed\n");

	return err;
}

static const struct file_operations xgkr_dbg_train_params_ops = {
	.open = xgkr_dbg_train_params_open,
	.read	= seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

//---------------------------------------------------------------------------------
// show tuned params
//
static int xgkr_dbg_tuned_params_show(struct seq_file *file, void *offset)
{
	struct xgkr_params *xgkr = (struct xgkr_params *)file->private;

	//seq_printf(file, "Tuned parameters for %s lane %d:\n", dev_name(xgkr->phydev->attached_dev->dev.parent), idx);
	seq_printf(file, "%16s%16s%16s\n",
		   "tuned_preq", "tuned_pst1q", "tuned_adpt_eq");

	seq_printf(file, "%16u%16u%16u\n",
			xgkr->tuned_ratio_preq,
			xgkr->tuned_ratio_pst1q,
			xgkr->tuned_adpt_eq);

	return 0;
}

static int xgkr_dbg_tuned_params_open(struct inode *inode, struct file *file)
{
	int err;
	struct xgkr_params *xgkr = (struct xgkr_params *)inode->i_private;

	err = single_open(file, xgkr_dbg_tuned_params_show, xgkr);
	if (err < 0)
		netdev_err(xgkr->phydev->attached_dev, "single_open() failed\n");

	return err;
}

static const struct file_operations xgkr_dbg_tuned_params_ops = {
	.open = xgkr_dbg_tuned_params_open,
	.read	= seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

//---------------------------------------------------------------------------------
// setup preq
//
static int xgkr_dbg_set_preq_show(struct seq_file *file, void *offset)
{
	struct xgkr_params *xgkr = (struct xgkr_params *)file->private;

	seq_printf(file, "%u", xgkr->set_ratio_preq);

	return 0;
}

static int xgkr_dbg_set_preq_open(struct inode *inode, struct file *file)
{
	int err;
	struct xgkr_params *xgkr = (struct xgkr_params *)inode->i_private;

	err = single_open(file, xgkr_dbg_set_preq_show, xgkr);
	if (err < 0)
		netdev_err(xgkr->phydev->attached_dev, "single_open() failed\n");

	return err;
}

static ssize_t xgkr_dbg_write_preq(struct file *filp, const char __user *buf,
			 size_t count, loff_t *pos)
{
	struct xgkr_params *xgkr = (struct xgkr_params *)filp->f_inode->i_private;
	char lbuf[10] = { 0 };
	int err;
	long val;

	if (copy_from_user(lbuf, buf, count))
		return -EFAULT;

	err = kstrtol((const char *)lbuf, 0, &val);
	if (err) {
		netdev_err(xgkr->phydev->attached_dev, "Invalid parameter\n");
		return err;
	}

	xgkr->set_ratio_preq = (u32)val;
	xgkr->set_applied = 0;

	return count;
}

static const struct file_operations xgkr_dbg_set_preq_ops = {
	.owner = THIS_MODULE,
	.open = xgkr_dbg_set_preq_open,
	.read	= seq_read,
	.write	= xgkr_dbg_write_preq,
	.llseek = seq_lseek,
	.release = single_release,
};

//---------------------------------------------------------------------------------
// setup pstq
//
static int xgkr_dbg_set_pstq_show(struct seq_file *file, void *offset)
{
	struct xgkr_params *xgkr = (struct xgkr_params *)file->private;

	seq_printf(file, "%u", xgkr->set_ratio_pst1q);

	return 0;
}

static int xgkr_dbg_set_pstq_open(struct inode *inode, struct file *file)
{
	int err;
	struct xgkr_params *xgkr = (struct xgkr_params *)inode->i_private;

	err = single_open(file, xgkr_dbg_set_pstq_show, xgkr);
	if (err < 0)
		netdev_err(xgkr->phydev->attached_dev, "single_open() failed\n");

	return err;
}

static ssize_t xgkr_dbg_write_pstq(struct file *filp, const char __user *buf,
			 size_t count, loff_t *pos)
{
	struct xgkr_params *xgkr = (struct xgkr_params *)filp->f_inode->i_private;
	char lbuf[10] = { 0 };
	int err;
	long val;

	if (copy_from_user(lbuf, buf, count))
		return -EFAULT;

	err = kstrtol((const char *)lbuf, 0, &val);
	if (err) {
		netdev_err(xgkr->phydev->attached_dev, "Invalid parameter\n");
		return err;
	}

	xgkr->set_ratio_pst1q = (u32)val;
	xgkr->set_applied = 0;

	return count;
}

static const struct file_operations xgkr_dbg_set_pstq_ops = {
	.owner = THIS_MODULE,
	.open = xgkr_dbg_set_pstq_open,
	.read	= seq_read,
	.write	= xgkr_dbg_write_pstq,
	.llseek = seq_lseek,
	.release = single_release,
};

//---------------------------------------------------------------------------------
// setup adpteq
//
static int xgkr_dbg_set_adpteq_show(struct seq_file *file, void *offset)
{
	struct xgkr_params *xgkr = (struct xgkr_params *)file->private;

	seq_printf(file, "%u", xgkr->set_adpt_eq);

	return 0;
}

static int xgkr_dbg_set_adpteq_open(struct inode *inode, struct file *file)
{
	int err;
	struct xgkr_params *xgkr = (struct xgkr_params *)inode->i_private;

	err = single_open(file, xgkr_dbg_set_adpteq_show, xgkr);
	if (err < 0)
		netdev_err(xgkr->phydev->attached_dev, "single_open() failed\n");

	return err;
}

static ssize_t xgkr_dbg_write_adpteq(struct file *filp, const char __user *buf,
			 size_t count, loff_t *pos)
{
	struct xgkr_params *xgkr = (struct xgkr_params *)filp->f_inode->i_private;
	char lbuf[10] = { 0 };
	int err;
	long val;

	if (copy_from_user(lbuf, buf, count))
		return -EFAULT;

	err = kstrtol((const char *)lbuf, 0, &val);
	if (err) {
		netdev_err(xgkr->phydev->attached_dev, "Invalid parameter\n");
		return err;
	}

	xgkr->set_adpt_eq = (u32)val;
	xgkr->set_applied = 0;

	return count;
}

static const struct file_operations xgkr_dbg_set_adpteq_ops = {
	.owner = THIS_MODULE,
	.open = xgkr_dbg_set_adpteq_open,
	.read	= seq_read,
	.write	= xgkr_dbg_write_adpteq,
	.llseek = seq_lseek,
	.release = single_release,
};

//---------------------------------------------------------------------------------
// setup apply
//
static int xgkr_dbg_set_apply_show(struct seq_file *file, void *offset)
{
	struct xgkr_params *xgkr = (struct xgkr_params *)file->private;

	seq_printf(file, "%u", xgkr->set_applied);

	return 0;
}

static int xgkr_dbg_set_apply_open(struct inode *inode, struct file *file)
{
	int err;
	struct xgkr_params *xgkr = (struct xgkr_params *)inode->i_private;

	err = single_open(file, xgkr_dbg_set_apply_show, xgkr);
	if (err < 0)
		netdev_err(xgkr->phydev->attached_dev, "single_open() failed\n");

	return err;
}

static ssize_t xgkr_dbg_write_apply(struct file *filp, const char __user *buf,
			 size_t count, loff_t *pos)
{
	struct xgkr_params *xgkr = (struct xgkr_params *)filp->f_inode->i_private;
	char lbuf[10] = { 0 };
	int err;
	long val;

	if (copy_from_user(lbuf, buf, count))
		return -EFAULT;

	//if (!strcmp(lbuf, "go"))
	//	printk("xgkr_dbg_write_apply: go \n");

	err = kstrtol((const char *)lbuf, 0, &val);
	if (err) {
		netdev_err(xgkr->phydev->attached_dev, "Invalid parameter\n");
		return err;
	}

	if (val == 1) {
		force_kr_setup(xgkr);
		xgkr->set_applied = 1;

		dev_info(&xgkr->phydev->mdio.dev, "Forced KR setup applied\n");
	}

	return count;
}

static const struct file_operations xgkr_dbg_set_apply_ops = {
	.owner = THIS_MODULE,
	.open = xgkr_dbg_set_apply_open,
	.read	= seq_read,
	.write	= xgkr_dbg_write_apply,
	.llseek = seq_lseek,
	.release = single_release,
};

//---------------------------------------------------------------------------------
// setup ampred
//
static int xgkr_dbg_set_ampred_show(struct seq_file *file, void *offset)
{
	struct xgkr_params *xgkr = (struct xgkr_params *)file->private;

	seq_printf(file, "%u", xgkr->set_amp_red);

	return 0;
}

static int xgkr_dbg_set_ampred_open(struct inode *inode, struct file *file)
{
	int err;
	struct xgkr_params *xgkr = (struct xgkr_params *)inode->i_private;

	err = single_open(file, xgkr_dbg_set_ampred_show, xgkr);
	if (err < 0)
		netdev_err(xgkr->phydev->attached_dev, "single_open() failed\n");

	return err;
}

static ssize_t xgkr_dbg_write_ampred(struct file *filp, const char __user *buf,
			 size_t count, loff_t *pos)
{
	struct xgkr_params *xgkr = (struct xgkr_params *)filp->f_inode->i_private;
	char lbuf[10] = { 0 };
	int err;
	long val;

	if (copy_from_user(lbuf, buf, count))
		return -EFAULT;

	err = kstrtol((const char *)lbuf, 0, &val);
	if (err) {
		netdev_err(xgkr->phydev->attached_dev, "Invalid parameter\n");
		return err;
	}

	xgkr->set_amp_red = (u32)val;

	force_amp_red(xgkr);

	dev_info(&xgkr->phydev->mdio.dev, "Forced amp_red applied\n");

	return count;
}

static const struct file_operations xgkr_dbg_set_ampred_ops = {
	.owner = THIS_MODULE,
	.open = xgkr_dbg_set_ampred_open,
	.read	= seq_read,
	.write	= xgkr_dbg_write_ampred,
	.llseek = seq_lseek,
	.release = single_release,
};

//---------------------------------------------------------------------------------
// cfg
//
static int xgkr_dbg_cfg_show(struct seq_file *file, void *offset)
{
	struct xgkr_params *xgkr = (struct xgkr_params *)file->private;

	seq_printf(file, "training_disabled = %u\n", xgkr->training_disabled);

	return 0;
}

static int xgkr_dbg_cfg_open(struct inode *inode, struct file *file)
{
	int err;
	struct xgkr_params *xgkr = (struct xgkr_params *)inode->i_private;

	err = single_open(file, xgkr_dbg_cfg_show, xgkr);
	if (err < 0)
		netdev_err(xgkr->phydev->attached_dev, "single_open() failed\n");

	return err;
}

static ssize_t xgkr_dbg_write_cfg(struct file *filp, const char __user *buf,
			 size_t count, loff_t *pos)
{
	struct xgkr_params *xgkr = (struct xgkr_params *)filp->f_inode->i_private;
	char lbuf[20] = { 0 };

	if (copy_from_user(lbuf, buf, count))
		return -EFAULT;

	//remove newline terminator
	lbuf[strlen(lbuf) - 1] = 0;

	if (!strcmp(lbuf, "train_en")) {

		xgkr->training_disabled = 0;

		dev_info(&xgkr->phydev->mdio.dev, "Enabled training algorithm\n");
	} else
		if (!strcmp(lbuf, "train_dis")) {

		xgkr->training_disabled = 1;

		dev_info(&xgkr->phydev->mdio.dev, "Disabled training algorithm\n");
	}

	return count;
}

static const struct file_operations xgkr_dbg_cfg_ops = {
	.owner = THIS_MODULE,
	.open = xgkr_dbg_cfg_open,
	.read	= seq_read,
	.write	= xgkr_dbg_write_cfg,
	.llseek = seq_lseek,
	.release = single_release,
};

//---------------------------------------------------------------------------------
// phy cmd
//
static int xgkr_dbg_phy_cmd_show(struct seq_file *file, void *offset)
{
	struct xgkr_phy_data *xgkr_inst = (struct xgkr_phy_data *)file->private;

	seq_printf(file, "retrain = %u\n", xgkr_inst->force_retrained);

	return 0;
}

static int xgkr_dbg_phy_cmd_open(struct inode *inode, struct file *file)
{
	int err;
	struct xgkr_phy_data *xgkr_inst = (struct xgkr_phy_data *)inode->i_private;

	err = single_open(file, xgkr_dbg_phy_cmd_show, xgkr_inst);
	if (err < 0)
		netdev_err(xgkr_inst->xgkr[0].phydev->attached_dev, "single_open() failed\n");

	return err;
}

static ssize_t xgkr_dbg_write_phy_cmd(struct file *filp, const char __user *buf,
			 size_t count, loff_t *pos)
{
	struct xgkr_phy_data *xgkr_inst = (struct xgkr_phy_data *)filp->f_inode->i_private;
	char lbuf[20] = { 0 };

	if (copy_from_user(lbuf, buf, count))
		return -EFAULT;

	//remove newline terminator
	lbuf[strlen(lbuf) - 1] = 0;

	if (!strcmp(lbuf, "retrain")) {

		force_restart_training(xgkr_inst);
		xgkr_inst->force_retrained = 1;

		dev_info(&xgkr_inst->xgkr[0].phydev->mdio.dev, "Forced restart KR training\n");
	}

	return count;
}

static const struct file_operations xgkr_dbg_phy_cmd_ops = {
	.owner = THIS_MODULE,
	.open = xgkr_dbg_phy_cmd_open,
	.read	= seq_read,
	.write	= xgkr_dbg_write_phy_cmd,
	.llseek = seq_lseek,
	.release = single_release,
};

//---------------------------------------------------------------------------------
// common debugfs functions
//

#define LANE_NAME	20

void backplane_dbg_add(struct phy_device *phydev)
{
	struct xgkr_phy_data *xgkr_inst = phydev->priv;
	struct dentry *phydev_dbg_root = xgkr_inst->dbg.dir;
	u8 i;
	char szlane[LANE_NAME];
	void *priv;

	if (!fsl_backplane_dbg_root)
		return;

	if (!phydev->attached_dev) {
		pr_err("%s: backplane_dbg_add failed: no phydev->attached_dev \n", FSL_BACKPLANE_DRIVER_NAME);
		return;
	}

	if (phydev_dbg_root)
		return;

	/* Create a directory for the interface */
										//dev_name(phydev->attached_dev->dev.parent)
										//netdev_name(phydev->attached_dev)
	phydev_dbg_root = debugfs_create_dir(dev_name(phydev->attached_dev->dev.parent),
					   fsl_backplane_dbg_root);
	if (!phydev_dbg_root) {
		netdev_err(phydev->attached_dev, "debugfs_create_dir() failed\n");
		return;
	}

	xgkr_inst->dbg.dir = phydev_dbg_root;

	/* phy cmd file */
	xgkr_inst->dbg.cmd = debugfs_create_file("cmd", 0777,
			phydev_dbg_root, xgkr_inst,
			&xgkr_dbg_phy_cmd_ops);
	if (!xgkr_inst->dbg.cmd) {
		netdev_err(phydev->attached_dev, "debugfs_create_file() failed\n");
		debugfs_remove(xgkr_inst->dbg.cmd);
	}


	for (i = 0; i < xgkr_inst->phy_lanes; i++) {

		/* Create a directory for the lane */
		snprintf(szlane, LANE_NAME - 1, "lane%d", i);
		xgkr_inst->xgkr[i].dbg.dir = debugfs_create_dir(szlane,
				phydev_dbg_root);

		if (!xgkr_inst->xgkr[i].dbg.dir) {
			netdev_err(phydev->attached_dev, "debugfs_create_dir() failed\n");
			continue;
		}

		priv = &xgkr_inst->xgkr[i];

		/* current training params file */
		xgkr_inst->xgkr[i].dbg.train_params = debugfs_create_file("train_params", 0777,
				xgkr_inst->xgkr[i].dbg.dir, priv,
				&xgkr_dbg_train_params_ops);
		if (!xgkr_inst->xgkr[i].dbg.train_params) {
			netdev_err(phydev->attached_dev, "debugfs_create_file() failed\n");
			debugfs_remove(xgkr_inst->xgkr[i].dbg.train_params);
		}

		/* tuned params file */
		xgkr_inst->xgkr[i].dbg.tuned_params = debugfs_create_file("tuned_params", 0777,
				xgkr_inst->xgkr[i].dbg.dir, priv,
				&xgkr_dbg_tuned_params_ops);
		if (!xgkr_inst->xgkr[i].dbg.tuned_params) {
			netdev_err(phydev->attached_dev, "debugfs_create_file() failed\n");
			debugfs_remove(xgkr_inst->xgkr[i].dbg.tuned_params);
		}

		/* set preq file */
		xgkr_inst->xgkr[i].dbg.set_preq = debugfs_create_file("set_preq", 0777,
				xgkr_inst->xgkr[i].dbg.dir, priv,
				&xgkr_dbg_set_preq_ops);
		if (!xgkr_inst->xgkr[i].dbg.set_preq) {
			netdev_err(phydev->attached_dev, "debugfs_create_file() failed\n");
			debugfs_remove(xgkr_inst->xgkr[i].dbg.set_preq);
		}

		/* set pstq file */
		xgkr_inst->xgkr[i].dbg.set_pstq = debugfs_create_file("set_pstq", 0777,
				xgkr_inst->xgkr[i].dbg.dir, priv,
				&xgkr_dbg_set_pstq_ops);
		if (!xgkr_inst->xgkr[i].dbg.set_pstq) {
			netdev_err(phydev->attached_dev, "debugfs_create_file() failed\n");
			debugfs_remove(xgkr_inst->xgkr[i].dbg.set_pstq);
		}

		/* set adpteq file */
		xgkr_inst->xgkr[i].dbg.set_adpteq = debugfs_create_file("set_adpteq", 0777,
				xgkr_inst->xgkr[i].dbg.dir, priv,
				&xgkr_dbg_set_adpteq_ops);
		if (!xgkr_inst->xgkr[i].dbg.set_adpteq) {
			netdev_err(phydev->attached_dev, "debugfs_create_file() failed\n");
			debugfs_remove(xgkr_inst->xgkr[i].dbg.set_adpteq);
		}

		/* set apply file */
		xgkr_inst->xgkr[i].dbg.set_apply = debugfs_create_file("set_apply", 0777,
				xgkr_inst->xgkr[i].dbg.dir, priv,
				&xgkr_dbg_set_apply_ops);
		if (!xgkr_inst->xgkr[i].dbg.set_apply) {
			netdev_err(phydev->attached_dev, "debugfs_create_file() failed\n");
			debugfs_remove(xgkr_inst->xgkr[i].dbg.set_apply);
		}

		/* set ampred file */
		xgkr_inst->xgkr[i].dbg.set_ampred = debugfs_create_file("set_ampred", 0777,
				xgkr_inst->xgkr[i].dbg.dir, priv,
				&xgkr_dbg_set_ampred_ops);
		if (!xgkr_inst->xgkr[i].dbg.set_ampred) {
			netdev_err(phydev->attached_dev, "debugfs_create_file() failed\n");
			debugfs_remove(xgkr_inst->xgkr[i].dbg.set_ampred);
		}

		/* cfg file */
		xgkr_inst->xgkr[i].dbg.cfg = debugfs_create_file("cfg", 0777,
				xgkr_inst->xgkr[i].dbg.dir, priv,
				&xgkr_dbg_cfg_ops);
		if (!xgkr_inst->xgkr[i].dbg.cfg) {
			netdev_err(phydev->attached_dev, "debugfs_create_file() failed\n");
			debugfs_remove(xgkr_inst->xgkr[i].dbg.cfg);
		}
	}
}

void backplane_dbg_remove(struct phy_device *phydev)
{
	struct xgkr_phy_data *xgkr_inst = phydev->priv;
	u8 i;

	for (i = 0; i < xgkr_inst->phy_lanes; i++) {

		debugfs_remove(xgkr_inst->xgkr[i].dbg.train_params);
		debugfs_remove(xgkr_inst->xgkr[i].dbg.tuned_params);
		debugfs_remove(xgkr_inst->xgkr[i].dbg.set_preq);
		debugfs_remove(xgkr_inst->xgkr[i].dbg.set_pstq);
		debugfs_remove(xgkr_inst->xgkr[i].dbg.set_adpteq);
		debugfs_remove(xgkr_inst->xgkr[i].dbg.set_apply);
		debugfs_remove(xgkr_inst->xgkr[i].dbg.set_ampred);
		debugfs_remove(xgkr_inst->xgkr[i].dbg.cfg);
	}

	debugfs_remove(xgkr_inst->dbg.cmd);
}

void backplane_dbg_init(void)
{
	if (fsl_backplane_dbg_root)
		return;

	fsl_backplane_dbg_root = debugfs_create_dir(FSL_BACKPLANE_DBG_ROOT, NULL);
	if (!fsl_backplane_dbg_root) {
		pr_err("%s: debugfs create failed\n", FSL_BACKPLANE_DRIVER_NAME);
		return;
	}

	pr_info("%s: debugfs created\n", FSL_BACKPLANE_DRIVER_NAME);
}

void __exit backplane_dbg_exit(void)
{
	debugfs_remove(fsl_backplane_dbg_root);
	pr_info("%s: debugfs removed\n", FSL_BACKPLANE_DRIVER_NAME);
}
#endif /* CONFIG_FSL_BACKPLANE_DEBUGFS */
