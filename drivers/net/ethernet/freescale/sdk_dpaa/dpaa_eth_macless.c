/* Copyright 2008-2013 Freescale Semiconductor Inc.
 * Copyright 2020 NXP
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *	 notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *	 notice, this list of conditions and the following disclaimer in the
 *	 documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *	 names of its contributors may be used to endorse or promote products
 *	 derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifdef CONFIG_FSL_DPAA_ETH_DEBUG
#define pr_fmt(fmt) \
	KBUILD_MODNAME ": %s:%hu:%s() " fmt, \
	KBUILD_BASENAME".c", __LINE__, __func__
#else
#define pr_fmt(fmt) \
	KBUILD_MODNAME ": " fmt
#endif

#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_net.h>
#include <linux/etherdevice.h>
#include <linux/kthread.h>
#include <linux/percpu.h>
#include <linux/highmem.h>
#include <linux/fsl_qman.h>
#include "dpaa_eth.h"
#include "dpaa_eth_common.h"
#include "dpaa_eth_base.h"
#include "lnxwrp_fsl_fman.h" /* fm_get_rx_extra_headroom(), fm_get_max_frm() */
#include "mac.h"
#include <net/rtnetlink.h>

#define DPA_MACLESS_NAPI_WEIGHT		64

/* For MAC-based interfaces, we compute the tx needed headroom from the
 * associated Tx port's buffer layout settings.
 * For MACless interfaces just use a default value.
 */
#define DPA_DEFAULT_TX_HEADROOM	64

#define DPA_DESCRIPTION "FSL DPAA MACless Ethernet driver"

MODULE_LICENSE("Dual BSD/GPL");

MODULE_DESCRIPTION(DPA_DESCRIPTION);

/* This has to work in tandem with the DPA_CS_THRESHOLD_xxx values. */
static uint16_t macless_tx_timeout = 1000;
module_param(macless_tx_timeout, ushort, S_IRUGO);
MODULE_PARM_DESC(macless_tx_timeout, "The MACless Tx timeout in ms");

/* forward declarations */
static int __cold dpa_macless_start(struct net_device *net_dev);
static int __cold dpa_macless_stop(struct net_device *net_dev);
static int __cold dpa_macless_set_address(struct net_device *net_dev,
					  void *addr);
static void __cold dpa_macless_set_rx_mode(struct net_device *net_dev);

static int dpaa_eth_macless_probe(struct platform_device *_of_dev);
static netdev_features_t
dpa_macless_fix_features(struct net_device *dev, netdev_features_t features);

struct sk_buff *__hot contig_fd_to_skb(const struct dpa_priv_s *priv,
	const struct qm_fd *fd, int *use_gro);

struct sk_buff *__hot sg_fd_to_skb(const struct dpa_priv_s *priv,
			       const struct qm_fd *fd, int *use_gro,
			       int *count_ptr);

void priv_ern(struct qman_portal	*portal,
		       struct qman_fq		*fq,
		       const struct qm_mr_entry	*msg);

/* forward declarations */
static enum qman_cb_dqrr_result __hot
macless_rx_dqrr(struct qman_portal *portal, struct qman_fq *fq,
		const struct qm_dqrr_entry *dq);
static void macless_ern(struct qman_portal	*portal,
		       struct qman_fq		*fq,
		       const struct qm_mr_entry	*msg);

static int __hot dpa_peer_tx(struct sk_buff *skb, struct net_device *net_dev)
{
	struct dpa_priv_s	*priv;
	const int queue_mapping = dpa_get_queue_mapping(skb);
	struct qman_fq *egress_fq, *conf_fq;

#ifdef CONFIG_FSL_DPAA_HOOKS
	/* If there is a Tx hook, run it. */
	if (dpaa_eth_hooks.tx &&
		dpaa_eth_hooks.tx(skb, net_dev) == DPAA_ETH_STOLEN)
		/* won't update any Tx stats */
		return NETDEV_TX_OK;
#endif

	priv = netdev_priv(net_dev);
#ifdef CONFIG_FSL_DPAA_CEETM
	if (priv->ceetm_en)
		return ceetm_tx(skb, net_dev);
#endif

	egress_fq = priv->egress_fqs[queue_mapping];
	conf_fq = priv->conf_fqs[queue_mapping];

	return dpa_tx_extended(skb, net_dev, egress_fq, conf_fq);
}

static enum qman_cb_dqrr_result __hot
macless_rx_dqrr(struct qman_portal *portal, struct qman_fq *fq,
		const struct qm_dqrr_entry *dq)
{
	struct net_device		*net_dev;
	struct dpa_priv_s		*priv;
	struct dpa_percpu_priv_s	*percpu_priv;
	int                             *count_ptr;
	const struct qm_fd *fd = &dq->fd;
	struct dpa_bp *dpa_bp;
	struct sk_buff *skb;
	dma_addr_t addr;
	int use_gro;

	net_dev = ((struct dpa_fq *)fq)->net_dev;
	priv = netdev_priv(net_dev);
	dpa_bp = dpa_bpid2pool(fd->bpid);
	BUG_ON(!dpa_bp);

	//printk("from BPID %d\n", fd->bpid);

	use_gro = net_dev->features & NETIF_F_GRO;

	addr = qm_fd_addr(fd);


	percpu_priv = raw_cpu_ptr(priv->percpu_priv);
	count_ptr = raw_cpu_ptr(dpa_bp->percpu_count);

	if (unlikely(dpaa_eth_napi_schedule(percpu_priv, portal)))
			return qman_cb_dqrr_stop;


	if (unlikely(dpaa_eth_refill_bpools(dpa_bp, count_ptr))) {
		/* Unable to refill the buffer pool due to insufficient
		 * system memory. Just release the frame back into the pool,
		 * otherwise we'll soon end up with an empty buffer pool.
		 */
		dpa_fd_release(net_dev, &dq->fd);

	/* this label is confusing*/
	goto out;
	}

	if (unlikely(fd->status & FM_FD_STAT_RX_ERRORS) != 0) {
		if (netif_msg_hw(priv) && net_ratelimit())
			netdev_warn(net_dev, "FD status = 0x%08x\n",
					fd->status & FM_FD_STAT_RX_ERRORS);

		percpu_priv->stats.rx_errors++;
		dpa_fd_release(net_dev, fd);
		goto out;
	}
	/* prefetch the first 64 bytes of the frame or the SGT start */
	dma_unmap_single(dpa_bp->dev, addr, dpa_bp->size, DMA_BIDIRECTIONAL);
	prefetch(phys_to_virt(addr) + dpa_fd_offset(fd));

	/* The only FD types that we may receive are contig and S/G */
	DPA_BUG_ON((fd->format != qm_fd_contig) && (fd->format != qm_fd_sg));

	if (likely(fd->format == qm_fd_contig)) {
#ifdef CONFIG_FSL_DPAA_HOOKS
		/* Execute the Rx processing hook, if it exists. */
		if (dpaa_eth_hooks.rx_default &&
			dpaa_eth_hooks.rx_default((void *)fd, net_dev,
					fqid) == DPAA_ETH_STOLEN) {
			/* won't count the rx bytes in */
			return;
		}
#endif
		skb = contig_fd_to_skb(priv, fd, &use_gro);
	} else {
		skb = sg_fd_to_skb(priv, fd, &use_gro, count_ptr);
		percpu_priv->rx_sg++;
	}

	(*count_ptr)--;
	skb->protocol = eth_type_trans(skb, net_dev);

	if (use_gro) {
		gro_result_t gro_result;
		const struct qman_portal_config *pc =
					qman_p_get_portal_config(portal);
		struct dpa_napi_portal *np = &percpu_priv->np[pc->index];

		np->p = portal;
		gro_result = napi_gro_receive(&np->napi, skb);
		/* If frame is dropped by the stack, rx_dropped counter is
		 * incremented automatically, so no need for us to update it
		 */
		if (unlikely(gro_result == GRO_DROP))
			goto out;

	} else if (unlikely(netif_receive_skb(skb) == NET_RX_DROP))
		goto out;

	percpu_priv->stats.rx_packets++;
	percpu_priv->stats.rx_bytes += dpa_fd_length(fd);

out:
	return qman_cb_dqrr_consume;
}

static void macless_ern(struct qman_portal	*portal,
		       struct qman_fq		*fq,
		       const struct qm_mr_entry	*msg)
{
	struct net_device *net_dev;
	const struct dpa_priv_s	*priv;
	struct dpa_percpu_priv_s *percpu_priv;
	struct dpa_fq *dpa_fq = (struct dpa_fq *)fq;

	net_dev = dpa_fq->net_dev;
	priv = netdev_priv(net_dev);
	percpu_priv = raw_cpu_ptr(priv->percpu_priv);

	dpa_fd_release(net_dev, &msg->ern.fd);

	percpu_priv->stats.tx_dropped++;
	percpu_priv->stats.tx_fifo_errors++;
	count_ern(percpu_priv, msg);
}

struct dpa_fq_cbs_t macless_fq_cbs = {
	.rx_defq = { .cb = { .dqrr = macless_rx_dqrr } },
	.tx_defq = { .cb = { .dqrr = priv_tx_conf_default_dqrr } },
	.rx_errq = { .cb = { .dqrr = macless_rx_dqrr } },
	.tx_errq = { .cb = { .dqrr = priv_tx_conf_error_dqrr } },
	.egress_ern = { .cb = { .ern = macless_ern } }
};

static const struct net_device_ops dpa_macless_ops = {
	.ndo_open = dpa_macless_start,
	.ndo_start_xmit = dpa_peer_tx,
	.ndo_stop = dpa_macless_stop,
	.ndo_tx_timeout = dpa_timeout,
	.ndo_get_stats64 = dpa_get_stats64,
	.ndo_set_mac_address = dpa_macless_set_address,
	.ndo_set_rx_mode = dpa_macless_set_rx_mode,
	.ndo_validate_addr = eth_validate_addr,
#ifdef CONFIG_FSL_DPAA_ETH_USE_NDO_SELECT_QUEUE
	.ndo_select_queue = dpa_select_queue,
#endif
	.ndo_init = dpa_ndo_init,
	.ndo_set_features = dpa_set_features,
	.ndo_fix_features = dpa_macless_fix_features,
};

static const struct of_device_id dpa_macless_match[] = {
	{
		.compatible	= "fsl,dpa-ethernet-macless"
	},
	{}
};
MODULE_DEVICE_TABLE(of, dpa_macless_match);

static struct platform_driver dpa_macless_driver = {
	.driver = {
		.name		= KBUILD_MODNAME "-macless",
		.of_match_table	= dpa_macless_match,
		.owner		= THIS_MODULE,
	},
	.probe		= dpaa_eth_macless_probe,
	.remove		= dpa_remove
};

static const char macless_frame_queues[][25] = {
	[RX] = "fsl,qman-frame-queues-rx",
	[TX] = "fsl,qman-frame-queues-tx"
};

static void dpaa_macless_napi_enable(struct dpa_priv_s *priv)
{
	struct dpa_percpu_priv_s *percpu_priv;
	int i, j;

	for_each_possible_cpu(i) {
		percpu_priv = per_cpu_ptr(priv->percpu_priv, i);

		for (j = 0; j < qman_portal_max; j++)
			napi_enable(&percpu_priv->np[j].napi);
	}
}

static void dpaa_macless_napi_disable(struct dpa_priv_s *priv)
{
	struct dpa_percpu_priv_s *percpu_priv;
	int i, j;

	for_each_possible_cpu(i) {
		percpu_priv = per_cpu_ptr(priv->percpu_priv, i);

		for (j = 0; j < qman_portal_max; j++)
			napi_disable(&percpu_priv->np[j].napi);
	}
}

static int __cold dpa_macless_start(struct net_device *net_dev)
{
	struct dpa_percpu_priv_s	*percpu_priv;
	int                             *count_ptr;
	struct dpa_bp *dpa_bp;
	struct dpa_priv_s *priv = netdev_priv(net_dev);
	struct proxy_device *proxy_dev = (struct proxy_device *)priv->peer;

	dpaa_macless_napi_enable(priv);
	netif_tx_start_all_queues(net_dev);

	if (proxy_dev)
		dpa_proxy_start(net_dev);

	dpa_bp = priv->dpa_bp;
	/*make sure to refill the buffer pool if interface is brought down and then up again. Being a macless */
	percpu_priv = raw_cpu_ptr(priv->percpu_priv);
	count_ptr = raw_cpu_ptr(dpa_bp->percpu_count);


	/*TODO -refill the buffer pool when no buffers are in the pool. This can happen for the macless scenario */


	return 0;
}

static int __cold dpa_macless_stop(struct net_device *net_dev)
{
	struct dpa_priv_s *priv = netdev_priv(net_dev);
	struct proxy_device *proxy_dev = (struct proxy_device *)priv->peer;

	netif_tx_stop_all_queues(net_dev);
	dpaa_macless_napi_disable(priv);

	if (proxy_dev)
		dpa_proxy_stop(proxy_dev, net_dev);

	return 0;
}

static int dpa_macless_set_address(struct net_device *net_dev, void *addr)
{
	const struct dpa_priv_s *priv = netdev_priv(net_dev);
	struct proxy_device *proxy_dev = (struct proxy_device *)priv->peer;
	int			 _errno;

	_errno = eth_mac_addr(net_dev, addr);
	if (_errno < 0) {
		if (netif_msg_drv(priv))
			netdev_err(net_dev, "eth_mac_addr() = %d\n", _errno);
		return _errno;
	}

	if (proxy_dev) {
		_errno = dpa_proxy_set_mac_address(proxy_dev, net_dev);
		if (_errno < 0) {
			if (netif_msg_drv(priv))
				netdev_err(net_dev, "proxy_set_mac_address() = %d\n",
						_errno);
			return _errno;
		}
	}

	return 0;
}

static void __cold dpa_macless_set_rx_mode(struct net_device *net_dev)
{
	const struct dpa_priv_s	*priv = netdev_priv(net_dev);
	struct proxy_device *proxy_dev = (struct proxy_device *)priv->peer;

	if (proxy_dev)
		dpa_proxy_set_rx_mode(proxy_dev, net_dev);
}

static netdev_features_t
dpa_macless_fix_features(struct net_device *dev, netdev_features_t features)
{
	netdev_features_t unsupported_features = 0;

	/* In theory we should never be requested to enable features that
	 * we didn't set in netdev->features and netdev->hw_features at probe
	 * time, but double check just to be on the safe side.
	 */
	unsupported_features |= NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM;
	/* We don't support enabling Rx csum through ethtool yet */
	unsupported_features |= NETIF_F_RXCSUM;

	features &= ~unsupported_features;

	return features;
}

static int dpa_macless_netdev_init(struct device_node *dpa_node,
				struct net_device *net_dev)
{
	struct dpa_priv_s *priv = netdev_priv(net_dev);
	struct proxy_device *proxy_dev = (struct proxy_device *)priv->peer;
	struct device *dev = net_dev->dev.parent;
	const uint8_t *mac_addr;

	net_dev->netdev_ops = &dpa_macless_ops;

	if (proxy_dev) {
		struct mac_device *mac_dev = proxy_dev->mac_dev;
		net_dev->mem_start = mac_dev->res->start;
		net_dev->mem_end = mac_dev->res->end;

		return dpa_netdev_init(net_dev, mac_dev->addr,
				macless_tx_timeout);
	} else {
		/* Get the MAC address from device tree */
		mac_addr = of_get_mac_address(dpa_node);

		if (mac_addr == NULL) {
			if (netif_msg_probe(priv))
				dev_err(dev, "No MAC address found!\n");
			return -EINVAL;
		}

		return dpa_netdev_init(net_dev, mac_addr,
				macless_tx_timeout);
	}
}

static int dpa_tx_conf_fq_macless_setup(struct net_device *macless_net_dev, struct list_head *list)
{

	struct dpa_fq	*dpa_fq;
	uint32_t loop = 0, conf_cnt = 0;
	int errno = 0;
	struct dpa_priv_s *priv = NULL;
	struct dpa_priv_s *macless_priv;


	macless_priv = netdev_priv(macless_net_dev);
	if (!dpa_fq_alloc(macless_net_dev->dev.parent, 0x0,
			((loop) ? loop : DPAA_ETH_TX_QUEUES), list, FQ_TYPE_TX_CONF_MQ)) {
		dev_err(macless_net_dev->dev.parent, "_dpa_fq_alloc() failed\n");
					return -ENOMEM;
	}

	list_for_each_entry(dpa_fq, &macless_priv->dpa_fq_list, list) {
		if (dpa_fq->fq_type == FQ_TYPE_TX_CONF_MQ) {
					macless_priv->conf_fqs[conf_cnt++] = &dpa_fq->fq_base;
					dpa_fq->fq_base.cb.dqrr = priv_tx_conf_default_dqrr;
					dpa_fq->net_dev = macless_priv->net_dev;
					dpa_fq->flags = QMAN_FQ_FLAG_NO_ENQUEUE;
					dpa_fq->channel = (priv) ? priv->channel : macless_priv->channel;
					errno = dpa_fq_init(dpa_fq, false);
					if (errno)
						break;

		}
	}

	return errno;
}

/* Probing of FQs for MACless ports */
static int dpa_fq_probe_macless(struct device *dev, struct list_head *list,
				enum port_type ptype, struct net_device *peer)
{
	struct device_node *np = dev->of_node;
	struct fqid_cell *fqids;
	const void *fqids_off = NULL;
	int num_ranges;
	int i, lenp;

	fqids_off = of_get_property(np, macless_frame_queues[ptype], &lenp);

	if (fqids_off == NULL) {
		dev_err(dev, "Need FQ definition in dts for MACless devices\n");
		return -EINVAL;
	}

	num_ranges = lenp / sizeof(*fqids);
	fqids = devm_kzalloc(dev, sizeof(*fqids) * num_ranges,
					GFP_KERNEL);
	if (fqids == NULL) {
		dev_err(dev, "Cannot allocate memory for frame queues\n");
		return -ENOMEM;
	}

	/* convert to CPU endianess */
	for (i = 0; i < num_ranges; i++) {
		fqids[i].start = be32_to_cpup(fqids_off +
						i * sizeof(*fqids));
		fqids[i].count = be32_to_cpup(fqids_off +
						i * sizeof(*fqids) + sizeof(__be32));
	}

	/*overwrite the start TX fq of macless with the one from peer. all the range will be thus the range from peer*/
	if (peer && ptype == TX) {
		struct dpa_fq	*dpa_fq;
		struct dpa_priv_s *priv = netdev_priv(peer);

		list_for_each_entry(dpa_fq, &priv->dpa_fq_list, list) {
			if (dpa_fq->fq_type == FQ_TYPE_TX) {
				break;
			}
		}
		fqids[0].start = dpa_fq->fqid;
	}

	/* All ranges defined in the device tree are used as Rx/Tx queues */
	for (i = 0; i < num_ranges; i++) {
		if (!dpa_fq_alloc(dev, fqids[i].start,
				  fqids[i].count, list,
				  ptype == RX ? FQ_TYPE_RX_PCD : FQ_TYPE_TX)) {
			dev_err(dev, "_dpa_fq_alloc() failed\n");
			return -ENOMEM;
		}
	}

	return 0;
}


	static struct proxy_device *
dpa_macless_proxy_probe(struct platform_device *_of_dev)
{
	struct device		*dev;
	const phandle		*proxy_prop;
	struct proxy_device	*proxy_dev;
	struct device_node	*proxy_node;
	struct platform_device  *proxy_pdev;
	int lenp;

	dev = &_of_dev->dev;

	proxy_prop = of_get_property(dev->of_node, "proxy", &lenp);
	if (!proxy_prop)
		return NULL;

	proxy_node = of_find_node_by_phandle(*proxy_prop);
	if (!proxy_node) {
		dev_err(dev, "Cannot find proxy node\n");
		return NULL;
	}

	proxy_pdev = of_find_device_by_node(proxy_node);
	if (!proxy_pdev) {
		of_node_put(proxy_node);
		dev_err(dev, "Cannot find device represented by proxy node\n");
		return NULL;
	}

	proxy_dev = dev_get_drvdata(&proxy_pdev->dev);

	of_node_put(proxy_node);

	return proxy_dev;
}

static int dpa_macless_napi_add(struct net_device *net_dev)
{
	struct dpa_priv_s *priv = netdev_priv(net_dev);
	struct dpa_percpu_priv_s *percpu_priv;
	int i, cpu;

	for_each_possible_cpu(cpu) {
		percpu_priv = per_cpu_ptr(priv->percpu_priv, cpu);

		percpu_priv->np = devm_kzalloc(net_dev->dev.parent,
			qman_portal_max * sizeof(struct dpa_napi_portal),
			GFP_KERNEL);

		if (unlikely(percpu_priv->np == NULL)) {
			dev_err(net_dev->dev.parent, "devm_kzalloc() failed\n");
			return -ENOMEM;
		}

		for (i = 0; i < qman_portal_max; i++)
			netif_napi_add(net_dev, &percpu_priv->np[i].napi,
					dpaa_eth_poll, DPA_MACLESS_NAPI_WEIGHT);
	}

	return 0;
}


void dpa_macless_napi_del(struct net_device *net_dev)
{
	struct dpa_priv_s *priv = netdev_priv(net_dev);
	struct dpa_percpu_priv_s *percpu_priv;
	int i, cpu;

	for_each_possible_cpu(cpu) {
		percpu_priv = per_cpu_ptr(priv->percpu_priv, cpu);

		if (percpu_priv->np) {
			for (i = 0; i < qman_portal_max; i++)
				netif_napi_del(&percpu_priv->np[i].napi);

			devm_kfree(net_dev->dev.parent, percpu_priv->np);
		}
	}
}

static int dpaa_eth_macless_probe(struct platform_device *_of_dev)
{
	int err = 0, i, channel;
	struct device *dev;
	struct device_node *dpa_node;
	struct dpa_bp *dpa_bp;
	size_t count;
	struct net_device *net_dev = NULL;
	struct dpa_priv_s *priv = NULL;
	struct dpa_percpu_priv_s *percpu_priv;
	static struct proxy_device *proxy_dev;
	struct dpa_buffer_layout_s *buf_layout = NULL;
	static u8 macless_idx;

	dev = &_of_dev->dev;

	dpa_node = dev->of_node;

	if (!of_device_is_available(dpa_node))
		return -ENODEV;

	/* Get the buffer pools assigned to this interface */
	dpa_bp = dpa_bp_probe(_of_dev, &count);
	if (IS_ERR(dpa_bp))
		return PTR_ERR(dpa_bp);

	for (i = 0; i < count; i++)
		dpa_bp[i].seed_cb = dpa_bp_macless_port_seed;

	proxy_dev = dpa_macless_proxy_probe(_of_dev);


	/* Allocate this early, so we can store relevant information in
	 * the private area (needed by 1588 code in dpa_mac_probe)
	 */
	net_dev = alloc_etherdev_mq(sizeof(*priv), DPAA_ETH_TX_QUEUES);
	if (!net_dev) {
		dev_err(dev, "alloc_etherdev_mq() failed\n");
		return -ENOMEM;
	}

	/* Do this here, so we can be verbose early */
	SET_NETDEV_DEV(net_dev, dev);
	dev_set_drvdata(dev, net_dev);

	priv = netdev_priv(net_dev);
	priv->net_dev = net_dev;
	sprintf(priv->if_type, "macless%d", macless_idx++);

	priv->msg_enable = netif_msg_init(advanced_debug, -1);

	priv->peer = NULL;
	priv->mac_dev = NULL;

	buf_layout = devm_kzalloc(dev, 2 * sizeof(*buf_layout),
				  GFP_KERNEL);
	buf_layout[RX].priv_data_size = (uint16_t)DPA_RX_PRIV_DATA_SIZE;
	buf_layout[RX].parse_results = true;
	buf_layout[RX].hash_results = true;

	buf_layout[RX].manip_extra_space = 64;

	buf_layout[RX].data_align = DPA_FD_DATA_ALIGNMENT;

	/* Tx */
	buf_layout[TX].priv_data_size = DPA_TX_PRIV_DATA_SIZE;
	buf_layout[TX].parse_results = true;
	buf_layout[TX].hash_results = true;
	priv->buf_layout = buf_layout;

	if (proxy_dev) {
		/* This is a temporary solution for the need of
		 * having main driver upstreamability: adjust_link
		 * is a general function that should work for both
		 * private driver and macless driver with MAC device
		 * control capabilities even if the last will not be
		 * upstreamable.
		 * TODO: find a convenient solution (wrapper over
		 * main priv structure, etc.)
		 */
		priv->mac_dev = proxy_dev->mac_dev;

		/* control over proxy's mac device */
		priv->peer = (void *)proxy_dev;
	}

	INIT_LIST_HEAD(&priv->dpa_fq_list);

	err = dpa_fq_probe_macless(dev, &priv->dpa_fq_list, RX, NULL);
	if (!err)
		err = dpa_fq_probe_macless(dev, &priv->dpa_fq_list,
					   TX, NULL);
	if (err < 0)
		goto fq_probe_failed;

	/* bp init */
	priv->bp_count = count;
	err = dpa_bp_create(net_dev, dpa_bp, count);
	if (err < 0)
		goto bp_create_failed;

	channel = dpa_get_channel();

	if (channel < 0) {
		err = channel;
		goto get_channel_failed;
	}

	priv->channel = (uint16_t)channel;
	dpaa_eth_add_channel(priv->channel);

	/* changed ern callback in case of congestion */
	macless_fq_cbs.egress_ern.cb.ern = priv_ern;
	dpa_fq_setup(priv, &macless_fq_cbs, NULL);

	/* Add the FQs to the interface, and make them active */
	/* For MAC-less devices we only get here for RX frame queues
	 * initialization, which are the TX queues of the other
	 * partition.
	 * It is safe to rely on one partition to set the FQ taildrop
	 * threshold for the TX queues of the other partition
	 * because the ERN notifications will be received by the
	 * partition doing qman_enqueue.
	 */
	err = dpa_fqs_init(dev,  &priv->dpa_fq_list, true);
	if (err < 0)
		goto fq_alloc_failed;

	err = dpa_tx_conf_fq_macless_setup(net_dev,&priv->dpa_fq_list);
	if (err < 0)
		goto alloc_tx_conf_fq_failed;

	priv->tx_headroom = DPA_DEFAULT_TX_HEADROOM;

	priv->percpu_priv = devm_alloc_percpu(dev, *priv->percpu_priv);

	if (priv->percpu_priv == NULL) {
		dev_err(dev, "devm_alloc_percpu() failed\n");
		err = -ENOMEM;
		goto alloc_percpu_failed;
	}
	for_each_possible_cpu(i) {
		percpu_priv = per_cpu_ptr(priv->percpu_priv, i);
		memset(percpu_priv, 0, sizeof(*percpu_priv));
	}

	/* Initialize NAPI */
	err = dpa_macless_napi_add(net_dev);
	if (err < 0)
		goto napi_add_failed;

	err = dpa_macless_netdev_init(dpa_node, net_dev);
	if (err < 0)
		goto netdev_init_failed;

	dpaa_eth_sysfs_init(&net_dev->dev);

	pr_info("fsl_dpa_macless: Probed %s interface as %s\n",
			priv->if_type, net_dev->name);

	return 0;

napi_add_failed:
dpa_macless_napi_del(net_dev);
netdev_init_failed:
alloc_tx_conf_fq_failed:
alloc_percpu_failed:
fq_alloc_failed:
	if (net_dev)
		dpa_fq_free(dev, &priv->dpa_fq_list);
get_channel_failed:
	if (net_dev)
		dpa_bp_free(priv);
bp_create_failed:
fq_probe_failed:
	dev_set_drvdata(dev, NULL);
	if (net_dev)
		free_netdev(net_dev);

	return err;
}

static int __init __cold dpa_macless_load(void)
{
	int	 _errno;

	pr_info(DPA_DESCRIPTION "\n");

	/* Initialize dpaa_eth mirror values */
	dpa_rx_extra_headroom = fm_get_rx_extra_headroom();
	dpa_max_frm = fm_get_max_frm();

	_errno = platform_driver_register(&dpa_macless_driver);
	if (unlikely(_errno < 0)) {
		pr_err(KBUILD_MODNAME
			": %s:%hu:%s(): platform_driver_register() = %d\n",
			KBUILD_BASENAME".c", __LINE__, __func__, _errno);
	}

	pr_debug(KBUILD_MODNAME ": %s:%s() ->\n",
		KBUILD_BASENAME".c", __func__);

	return _errno;
}
module_init(dpa_macless_load);

static void __exit __cold dpa_macless_unload(void)
{
	platform_driver_unregister(&dpa_macless_driver);

	pr_debug(KBUILD_MODNAME ": %s:%s() ->\n",
		KBUILD_BASENAME".c", __func__);
}
module_exit(dpa_macless_unload);
