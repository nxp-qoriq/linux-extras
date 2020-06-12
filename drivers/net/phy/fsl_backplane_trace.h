/* SPDX-License-Identifier: GPL-2.0+ */
/*
 *  DPAA Backplane trace
 *
 * Copyright 2019 NXP
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM	fsl_backplane

#if !defined(_FSL_BACKPLANE_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define _FSL_BACKPLANE_TRACE_H

#include <linux/phy.h>
#include <linux/tracepoint.h>

TRACE_EVENT(xgkr_debug_log,
	    /* Trace function prototype */
	    TP_PROTO(char* phy, char* prf, char* msg),
	    /* Repeat argument list here */
	    TP_ARGS(phy, prf, msg),
	    /* A structure containing the relevant information we want
	     * to record. Declare name and type for each normal element,
	     * name, type and size for arrays. Use __string for variable
	     * length strings.
	     */
	    TP_STRUCT__entry(
	    		__string(phy_name, phy)
	    		__string(prefix, prf)
	    		__string(message, msg)
	    ),
	    /* The function that assigns values to the above declared
	     * fields
	     */
	    TP_fast_assign(
				__assign_str(phy_name, phy);
				__assign_str(prefix, prf);
				__assign_str(message, msg);
	    ),
	    /* This is what gets printed when the trace event is
	     * triggered.
	     */
	    TP_printk("%s: %s: %s", __get_str(phy_name), __get_str(prefix), __get_str(message))
);

#define TR_COE_UPDATE_FMT "%s, lane%d, %s update, INIT %u, PRESET %u, C(-1) %s, C(0) %s, C(+1) %s"

#define COE_UPDATE_STRING(upd) \
	((upd == HOLD) ? "HOLD" : \
	 (upd == INCREMENT) ? "INC" : \
	 (upd == DECREMENT) ? "DEC" : "RESV")

TRACE_EVENT(xgkr_coe_update,
	    /* Trace function prototype */
	    TP_PROTO(struct xgkr_params *xgkr, u32 coe_update, bool send),
	    /* Repeat argument list here */
	    TP_ARGS(xgkr, coe_update, send),
	    /* Structure containing the relevant information */
	    TP_STRUCT__entry(
			     __field(u8, send)
			     __field(u8, preset)
			     __field(u8, init)
			     __field(u8, cop1_upd)
			     __field(u8, coz_upd)
			     __field(u8, com1_upd)
			     __field(u8, lane_index)
			     __string(phy_name, dev_name(xgkr->phydev->attached_dev->dev.parent))
	    ),
	    /* Assign values to the above declared fields */
	    TP_fast_assign(
			   __entry->send = send;
			   __entry->preset = ((coe_update & PRESET_MASK) != 0);
			   __entry->init = ((coe_update & INIT_MASK) != 0);
			   __entry->cop1_upd = ((coe_update & COP1_MASK) >> COP1_SHIFT);
			   __entry->coz_upd = ((coe_update & COZ_MASK) >> COZ_SHIFT);
			   __entry->com1_upd = ((coe_update & COM1_MASK) >> COM1_SHIFT);
			   __entry->lane_index = xgkr->idx;
			   __assign_str(phy_name, dev_name(xgkr->phydev->attached_dev->dev.parent));
	    ),
	    /* Print event trace info */
	    TP_printk(TR_COE_UPDATE_FMT,
		      __get_str(phy_name),
		      __entry->lane_index,
		      __entry->send ? "send" : "recv",
		      __entry->init,
		      __entry->preset,
		      COE_UPDATE_STRING(__entry->com1_upd),
		      COE_UPDATE_STRING(__entry->coz_upd),
		      COE_UPDATE_STRING(__entry->cop1_upd))
);

#define TR_COE_STATUS_FMT "%s, lane%d, %s status, RX_RDY %u, C(-1) %s, C(0) %s, C(+1) %s"

#define COE_STATUS_STRING(upd) \
	((upd == COE_NOTUPDATED) ? "NOT_UPDATED" : \
	 (upd == COE_UPDATED) ? "UPDATED" : \
	 (upd == COE_MIN) ? "MIN" : \
	 (upd == COE_MAX) ? "MAX" : "INVALID")

TRACE_EVENT(xgkr_coe_status,
	    /* Trace function prototype */
	    TP_PROTO(struct xgkr_params *xgkr, u32 coe_status, bool local),
	    /* Repeat argument list here */
	    TP_ARGS(xgkr, coe_status, local),
	    /* Structure containing the relevant information */
	    TP_STRUCT__entry(
			     __field(u8, local)
			     __field(u8, rx_rdy)
			     __field(u8, cop1_status)
			     __field(u8, coz_status)
			     __field(u8, com1_status)
			     __field(u8, lane_index)
			     __string(name, dev_name(xgkr->phydev->attached_dev->dev.parent))
	    ),
	    /* Assign values to the above declared fields */
	    TP_fast_assign(
			   __entry->local = local;
			   __entry->rx_rdy = ((coe_status & RX_READY_MASK) != 0);
			   __entry->cop1_status = ((coe_status & COP1_MASK) >> COP1_SHIFT);
			   __entry->coz_status = ((coe_status & COZ_MASK) >> COZ_SHIFT);
			   __entry->com1_status = ((coe_status & COM1_MASK) >> COM1_SHIFT);
			   __entry->lane_index = xgkr->idx;
			   __assign_str(name, dev_name(xgkr->phydev->attached_dev->dev.parent));
	    ),
	    /* Print event trace info */
	    TP_printk(TR_COE_STATUS_FMT,
		      __get_str(name),
		      __entry->lane_index,
		      __entry->local ? "LD" : "LP",
		      __entry->rx_rdy,
		      COE_STATUS_STRING(__entry->com1_status),
		      COE_STATUS_STRING(__entry->coz_status),
		      COE_STATUS_STRING(__entry->cop1_status))
);

#define TR_BIN_SNAPSHOT_FMT "%s, lane%d, %s: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d"

TRACE_EVENT(xgkr_bin_snapshots,
	    /* Trace function prototype */
	    TP_PROTO(struct xgkr_params *xgkr, char* bin_name, s16 *snapshot),
	    /* Repeat argument list here */
	    TP_ARGS(xgkr, bin_name, snapshot),
	    /* Structure containing the relevant information */
	    TP_STRUCT__entry(
			     __string(bin_str, bin_name)
			     __field(u8, lane_index)
			     __field(s16, snapshot_0)
			     __field(s16, snapshot_1)
			     __field(s16, snapshot_2)
			     __field(s16, snapshot_3)
			     __field(s16, snapshot_4)
			     __field(s16, snapshot_5)
			     __field(s16, snapshot_6)
			     __field(s16, snapshot_7)
			     __field(s16, snapshot_8)
			     __field(s16, snapshot_9)
			     __string(phy_name, dev_name(xgkr->phydev->attached_dev->dev.parent))
	    ),
	    /* Assign values to the above declared fields */
	    TP_fast_assign(
			   __assign_str(bin_str, bin_name);
			   __entry->lane_index = xgkr->idx;
			   __entry->snapshot_0 = snapshot[0];
			   __entry->snapshot_1 = snapshot[1];
			   __entry->snapshot_2 = snapshot[2];
			   __entry->snapshot_3 = snapshot[3];
			   __entry->snapshot_4 = snapshot[4];
			   __entry->snapshot_5 = snapshot[5];
			   __entry->snapshot_6 = snapshot[6];
			   __entry->snapshot_7 = snapshot[7];
			   __entry->snapshot_8 = snapshot[8];
			   __entry->snapshot_9 = snapshot[9];
			   __assign_str(phy_name, dev_name(xgkr->phydev->attached_dev->dev.parent));
	    ),
	    /* Print event trace info */
	    TP_printk(TR_BIN_SNAPSHOT_FMT,
			      __get_str(phy_name),
			      __entry->lane_index,
			      __get_str(bin_str),
			      __entry->snapshot_0,
			      __entry->snapshot_1,
			      __entry->snapshot_2,
			      __entry->snapshot_3,
			      __entry->snapshot_4,
			      __entry->snapshot_5,
			      __entry->snapshot_6,
			      __entry->snapshot_7,
			      __entry->snapshot_8,
			      __entry->snapshot_9
		      )
);

#define TR_GAIN_SNAPSHOT_FMT "%s, lane%d, %s: 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x"

TRACE_EVENT(xgkr_gain_snapshots,
	    /* Trace function prototype */
	    TP_PROTO(struct xgkr_params *xgkr, char* bin_name, u8 *snapshot),
	    /* Repeat argument list here */
	    TP_ARGS(xgkr, bin_name, snapshot),
	    /* Structure containing the relevant information */
	    TP_STRUCT__entry(
			     __string(bin_str, bin_name)
			     __field(u8, lane_index)
			     __field(u8, snapshot_0)
			     __field(u8, snapshot_1)
			     __field(u8, snapshot_2)
			     __field(u8, snapshot_3)
			     __field(u8, snapshot_4)
			     __field(u8, snapshot_5)
			     __field(u8, snapshot_6)
			     __field(u8, snapshot_7)
			     __field(u8, snapshot_8)
			     __field(u8, snapshot_9)
			     __string(phy_name, dev_name(xgkr->phydev->attached_dev->dev.parent))
	    ),
	    /* Assign values to the above declared fields */
	    TP_fast_assign(
			   __assign_str(bin_str, bin_name);
			   __entry->lane_index = xgkr->idx;
			   __entry->snapshot_0 = snapshot[0];
			   __entry->snapshot_1 = snapshot[1];
			   __entry->snapshot_2 = snapshot[2];
			   __entry->snapshot_3 = snapshot[3];
			   __entry->snapshot_4 = snapshot[4];
			   __entry->snapshot_5 = snapshot[5];
			   __entry->snapshot_6 = snapshot[6];
			   __entry->snapshot_7 = snapshot[7];
			   __entry->snapshot_8 = snapshot[8];
			   __entry->snapshot_9 = snapshot[9];
			   __assign_str(phy_name, dev_name(xgkr->phydev->attached_dev->dev.parent));
	    ),
	    /* Print event trace info */
	    TP_printk(TR_GAIN_SNAPSHOT_FMT,
			      __get_str(phy_name),
			      __entry->lane_index,
			      __get_str(bin_str),
			      __entry->snapshot_0,
			      __entry->snapshot_1,
			      __entry->snapshot_2,
			      __entry->snapshot_3,
			      __entry->snapshot_4,
			      __entry->snapshot_5,
			      __entry->snapshot_6,
			      __entry->snapshot_7,
			      __entry->snapshot_8,
			      __entry->snapshot_9
		      )
);

#endif /* _FSL_BACKPLANE_TRACE_H */

/* This must be outside ifdef _FSL_BACKPLANE_TRACE_H */
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE	fsl_backplane_trace
#include <trace/define_trace.h>
