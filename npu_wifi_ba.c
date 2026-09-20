/*
 * AN75XX NPU firmware - block ack reorder engine (kite)
 *
 * The kite chips hand the NPU frames in the order they arrive, so the
 * NPU holds the reorder window. Each WCID has 8 TIDs, each TID a
 * 28-byte entry, and each entry a list of 36-byte MPDU nodes.
 *
 * The eagle chips reorder in the WiFi chip, so none of this runs there.
 */

#include "npu_internal.h"
#include "npu_wifi.h"


/* ================================================================
 * Reorder node management
 *
 * Two pools: pri (2000 nodes) and sec (5000 nodes), 36 bytes each.
 * Free-list managed via circular index buffers with mutex.
 * ================================================================ */

#define REORDER_NODE_SIZE      36

void reorder_node_free(u16 node_idx, u8 node_type, u32 band)
{
	hw_mutex_lock(reorder_free_mutex);

	if (node_type != 0) {
		*(u16 *)(reorder_pri_idx_pool + 2 * reorder_pri_ridx) = node_idx;
		reorder_pri_ridx++;
	} else {
		*(u16 *)(reorder_sec_idx_pool + 2 * reorder_sec_ridx) = node_idx;
		reorder_sec_ridx++;
	}

	if (wifi_debug_flags & 4) {
		u32 base;

		if (band == 1)
			base = counter_base_5g;
		else if (band != 0)
			base = counter_base_tri;
		else
			base = counter_base_2g;
		(*(u32 *)(base + 212))++;
	}

	if (reorder_pri_ridx == REORDER_PRI_POOL_SIZE)
		reorder_pri_ridx = 0;
	if (reorder_sec_ridx == REORDER_SEC_POOL_SIZE)
		reorder_sec_ridx = 0;

	hw_mutex_unlock(reorder_free_mutex);
}

u32 reorder_node_alloc(u32 band, u32 *pool_type, u16 *idx_out)
{
	u16 widx, next;

	hw_mutex_lock(reorder_alloc_mutex);

	/* try pri pool */
	widx = reorder_pri_widx;
	next = (widx == REORDER_PRI_POOL_SIZE - 1) ? 0 : widx + 1;
	if (reorder_pri_ridx != next) {
		reorder_pri_widx++;
		*idx_out = *(u16 *)(reorder_pri_idx_pool + 2 * widx);
		if (reorder_pri_widx == REORDER_PRI_POOL_SIZE)
			reorder_pri_widx = 0;

		if (wifi_debug_flags & 4) {
			u32 base = (band == 1) ? counter_base_5g :
				   (band != 0) ? counter_base_tri :
				   counter_base_2g;
			(*(u32 *)(base + 220))++;
		}

		hw_mutex_unlock(reorder_alloc_mutex);
		return reorder_pri_node_base +
		       REORDER_NODE_SIZE * (*idx_out);
	}

	/* try sec pool */
	widx = reorder_sec_widx;
	next = (widx == REORDER_SEC_POOL_SIZE - 1) ? 0 : widx + 1;
	if (reorder_sec_ridx != next) {
		*pool_type = 0;
		reorder_sec_widx = widx + 1;
		*idx_out = *(u16 *)(reorder_sec_idx_pool + 2 * widx);
		if (reorder_sec_widx == REORDER_SEC_POOL_SIZE)
			reorder_sec_widx = 0;

		if (wifi_debug_flags & 4) {
			u32 base = (band == 1) ? counter_base_5g :
				   (band != 0) ? counter_base_tri :
				   counter_base_2g;
			(*(u32 *)(base + 224))++;
		}

		hw_mutex_unlock(reorder_alloc_mutex);
		return reorder_sec_node_base +
		       REORDER_NODE_SIZE * (*idx_out);
	}

	/* both pools exhausted */
	if (wifi_debug_flags & 4) {
		u32 base = (band == 1) ? counter_base_5g :
			   (band != 0) ? counter_base_tri :
			   counter_base_2g;
		(*(u32 *)(base + 216))++;
	}

	hw_mutex_unlock(reorder_alloc_mutex);
	return 0;
}

/* ================================================================
 * BME send (buffer move engine enqueue)
 * ================================================================ */

static int bme_send_pkt(u32 buf_id, u32 pkt_size, u32 phys_addr, u32 band)
{
	u32 desc_base = tdma_bme_dscp_base_addr;
	u32 idx, desc_addr;
	u32 hw_idx;

	if (desc_base == 0)
		return -1;

	hw_idx = REG32(BME_BASE + 0x008);
	idx = tdma_bme_dscp_idx;

	if (((idx + 1) & 0x1FF) == hw_idx)
		return -1;

	desc_addr = desc_base + idx * 8;
	*(volatile u32 *)desc_addr = phys_addr;
	*(volatile u32 *)(desc_addr + 4) =
		(pkt_size & 0x3FFF) | ((band & 3) << 14) | (buf_id << 16);

	tdma_bme_dscp_idx = (idx + 1) & 0x1FF;
	REG32(BME_BASE + 0x004) = tdma_bme_dscp_idx;

	return 0;
}

/* ================================================================
 * Packet enqueue to bridge/host
 *
 * Routes packets through fast-path (dispatch table), direct forward
 * (pkt_forward), or BME DMA depending on configuration.
 * ================================================================ */

int pkt_enqueue_bridge(u32 buf_id, u32 pkt_len, u32 amsdu,
			      u32 band, u32 fwd_type)
{
	int ret;

	/* fast path: dispatch table configured */
	if (fwd_dispatch_table[band * 16 + fwd_type] != 0) {
		ret = pkt_forward(buf_id & 0xFFFF, pkt_len, 0, 0,
				  band, 2, pkt_len, -2, 0);
		if (ret != 0)
			buf_id_free(0, 0, buf_id);
		return ret;
	}

	/* pipeline mode: check if band pipeline is ready */
	if (wifi_debug_flags & 1) {
		if (band == 1 && pipeline_5g_ready == 1)
			goto direct_fwd;
		if (band == 0 && pipeline_2g_ready == 1)
			goto direct_fwd;
	}

	if (bme_path_enable == 0) {
direct_fwd:
		ret = pkt_forward(buf_id & 0xFFFF, pkt_len, 0, 0,
				  band, 2, pkt_len, 0, 0);
		if (ret != 0)
			buf_id_free(0, band, buf_id);
		return ret;
	}

	/* BME path: DMA packet to host */
	{
		u32 pkt_size = (pkt_len - amsdu) & 0xFFFF;
		u32 adj_size = (pkt_size - 2) & 0xFFFF;
		u32 phys_addr;

		/* byte stats */
		if (wifi_debug_flags & 4) {
			u32 *cnt;

			if (band == 1)
				cnt = (u32 *)(counter_base_5g + 100);
			else if (band == 0)
				cnt = (u32 *)(counter_base_2g + 100);
			else
				cnt = (u32 *)(counter_base_tri + 100);

			if (cnt) {
				u32 lo = cnt[0];

				cnt[0] = lo + adj_size;
				cnt[1] += (lo + adj_size < lo);
			}
		}

		/* packet count stats */
		if (wifi_debug_flags & 4) {
			u32 *cnt;

			if (band == 1)
				cnt = (u32 *)(counter_base_5g + 84);
			else if (band == 0)
				cnt = (u32 *)(counter_base_2g + 84);
			else
				cnt = (u32 *)(counter_base_tri + 84);
			(*cnt)++;
		}

		/* size classification stats */
		if (((pkt_size - 506) & 0xFFFF) <= 0x10) {
			if (wifi_debug_flags & 4) {
				u32 base = (band == 1) ? counter_base_5g :
					   (band != 0) ? counter_base_tri :
					   counter_base_2g;
				(*(u32 *)(base + 92))++;
			}
		} else if (((pkt_size - 86) & 0xFFFF) <= 8) {
			if (wifi_debug_flags & 4) {
				u32 base = (band == 1) ? counter_base_5g :
					   (band != 0) ? counter_base_tri :
					   counter_base_2g;
				(*(u32 *)(base + 88))++;
			}
		} else if (((pkt_size - 1512) & 0xFFFF) <= 0x10) {
			if (wifi_debug_flags & 4) {
				u32 base = (band == 1) ? counter_base_5g :
					   (band != 0) ? counter_base_tri :
					   counter_base_2g;
				(*(u32 *)(base + 96))++;
			}
		}

		phys_addr = (buf_id << 12) +
			    ((wifi_buf_id_base & 0x3FFFFFFF) | 0x40000000) +
			    2 + amsdu;
		ret = bme_send_pkt(buf_id, adj_size, phys_addr, band);
		if (ret != 0)
			buf_id_free(0, band, buf_id);
		return ret;
	}
}

/* ================================================================
 * BA (Block Ack) entry management
 *
 * Each WCID has 8 TIDs x 28-byte BA entries. MPDU nodes (36 bytes)
 * form a linked list per entry; each MPDU can have MSDU sub-nodes.
 *
 * Node offsets: 0=next, 4=msdu_head, 8=msdu_tail, 12=msdu_cnt,
 *   16=amsdu, 19=fwd_type, 20=buf_id, 22=node_idx, 24=seq,
 *   26=data_len, 28=class, 29=node_type
 *
 * Entry offsets: 0=mpdu_head, 4=mpdu_tail, 8=count, 12=state,
 *   16=timeout(u64), 18=expected_seq, 20=ref_sn, 22=ba_state,
 *   23=flag, 24=active, 25=band
 * ================================================================ */

void ba_flush_entry(u32 *entry)
{
	u8 band = *(u8 *)((u32)entry + 25);
	u32 *mpdu, *msdu;

	if (band | wifi_dbdc_mode)
		hw_mutex_lock(ba_mutex_5g);
	else
		hw_mutex_lock(ba_mutex_2g);

	while (entry[0] != 0) {
		mpdu = (u32 *)entry[0];

		*(u16 *)((u32)entry + 8) -= 1;
		entry[0] = mpdu[0];
		if (mpdu[0] == 0) {
			mpdu[0] = 0;
			entry[1] = 0;
		}

		pkt_enqueue_bridge(
			*(u16 *)((u32)mpdu + 20),
			*(u16 *)((u32)mpdu + 26),
			*(u8 *)((u32)mpdu + 16),
			(u32)band,
			*(u8 *)((u32)mpdu + 19));

		while (mpdu[1] != 0) {
			msdu = (u32 *)mpdu[1];

			*(u16 *)((u32)mpdu + 12) -= 1;
			mpdu[1] = msdu[0];
			if (msdu[0] == 0) {
				msdu[0] = 0;
				mpdu[2] = 0;
			}

			pkt_enqueue_bridge(
				*(u16 *)((u32)msdu + 20),
				*(u16 *)((u32)msdu + 26),
				*(u8 *)((u32)msdu + 16),
				(u32)band,
				*(u8 *)((u32)mpdu + 19));

			reorder_node_free(
				*(u16 *)((u32)msdu + 22),
				*(u8 *)((u32)msdu + 29),
				(u32)band);
		}

		*(u16 *)((u32)entry + 18) = *(u16 *)((u32)mpdu + 24);

		reorder_node_free(
			*(u16 *)((u32)mpdu + 22),
			*(u8 *)((u32)mpdu + 29),
			(u32)band);
	}

	entry[3] = 0;

	if (band | wifi_dbdc_mode)
		hw_mutex_unlock(ba_mutex_5g);
	else
		hw_mutex_unlock(ba_mutex_2g);
}

void ba_indicate_le_seq(u32 *entry, u32 seq)
{
	u8 band = *(u8 *)((u32)entry + 25);
	u32 *mpdu;
	u16 mpdu_seq;

	if (band | wifi_dbdc_mode)
		hw_mutex_lock(ba_mutex_5g);
	else
		hw_mutex_lock(ba_mutex_2g);

	mpdu = (u32 *)entry[0];
	if (entry[0] == 0)
		goto unlock;

	do {
		mpdu_seq = *(u16 *)((u32)mpdu + 24);

		if (mpdu_seq != (u16)seq &&
		    (((u16)mpdu_seq - (u16)seq) & 0x800) == 0)
			break;

		*(u16 *)((u32)entry + 8) -= 1;
		entry[0] = mpdu[0];
		if (mpdu[0] == 0) {
			mpdu[0] = 0;
			entry[1] = 0;
		}

		{
			s16 bid = *(s16 *)((u32)mpdu + 20);

			if (bid > 5600 ||
			    *(u16 *)((u32)mpdu + 26) == 0 ||
			    *(u8 *)((u32)mpdu + 28) > 3)
				npu_printf("[%d,%s]!![%s]ERROR: bufid is over: %d, or pkt len is mpdu_blk->datalength=%d\n",
					   638,
					   "ba_indicate_reordering_mpdus_le_seq",
					   (band == 1) ? "5G" : "2.4G",
					   (u32)bid,
					   (u32)*(u16 *)((u32)mpdu + 26));
		}

		if (wifi_debug_flags & 4) {
			u32 base = (band == 1) ? counter_base_5g :
				   (band != 0) ? counter_base_tri :
				   counter_base_2g;
			(*(u32 *)(base + 68))++;
		}

		while (mpdu[1] != 0) {
			u32 *msdu = (u32 *)mpdu[1];

			*(u16 *)((u32)mpdu + 12) -= 1;
			mpdu[1] = msdu[0];
			if (msdu[0] == 0) {
				msdu[0] = 0;
				mpdu[2] = 0;
			}

			{
				s16 bid = *(s16 *)((u32)msdu + 20);

				if (bid > 5600 ||
				    *(u16 *)((u32)msdu + 26) == 0 ||
				    *(u8 *)((u32)msdu + 28) > 3)
					npu_printf("[%d,%s]!!ERROR:[%s] bufid is over: %d, or pkt len is msdu_blk->datalength=%d\n",
						   652,
						   "ba_indicate_reordering_mpdus_le_seq",
						   (band == 1) ? "5G" : "2.4G",
						   (u32)bid,
						   (u32)*(u16 *)((u32)msdu + 26));
			}

			if (wifi_debug_flags & 4) {
				u32 base = (band == 1) ? counter_base_5g :
					   (band != 0) ? counter_base_tri :
					   counter_base_2g;
				(*(u32 *)(base + 68))++;
			}

			reorder_node_free(
				*(u16 *)((u32)msdu + 22),
				*(u8 *)((u32)msdu + 29),
				(u32)band);
		}

		reorder_node_free(
			*(u16 *)((u32)mpdu + 22),
			*(u8 *)((u32)mpdu + 29),
			(u32)band);

		mpdu = (u32 *)entry[0];
	} while (entry[0] != 0);

unlock:
	if (band != 0 || wifi_dbdc_mode)
		hw_mutex_unlock(ba_mutex_5g);
	else
		hw_mutex_unlock(ba_mutex_2g);
}

u32 ba_seq_scan(u32 *entry, u32 seq)
{
	u8 band = *(u8 *)((u32)entry + 25);
	u32 *mpdu;
	u32 last_seq;

	if (band | wifi_dbdc_mode)
		hw_mutex_lock(ba_mutex_5g);
	else
		hw_mutex_lock(ba_mutex_2g);

	mpdu = (u32 *)entry[0];
	last_seq = 0xFFFF;

	if (entry[0] == 0)
		goto unlock;

	if (*(u16 *)((u32)mpdu + 24) != ((seq + 1) & 0xFFF)) {
		last_seq = 0xFFFF;
		goto unlock;
	}

	do {
		*(u16 *)((u32)entry + 8) -= 1;
		entry[0] = mpdu[0];
		if (mpdu[0] == 0) {
			mpdu[0] = 0;
			entry[1] = 0;
		}

		if (wifi_debug_flags & 4) {
			u32 base = (band == 1) ? counter_base_5g :
				   (band != 0) ? counter_base_tri :
				   counter_base_2g;
			(*(u32 *)(base + 68))++;
		}

		last_seq = *(u16 *)((u32)mpdu + 24);

		while (mpdu[1] != 0) {
			u32 *msdu = (u32 *)mpdu[1];

			*(u16 *)((u32)mpdu + 12) -= 1;
			mpdu[1] = msdu[0];
			if (msdu[0] == 0) {
				msdu[0] = 0;
				mpdu[2] = 0;
			}

			if (wifi_debug_flags & 4) {
				u32 base = (band == 1) ? counter_base_5g :
					   (band != 0) ? counter_base_tri :
					   counter_base_2g;
				(*(u32 *)(base + 68))++;
			}

			reorder_node_free(
				*(u16 *)((u32)msdu + 22),
				*(u8 *)((u32)msdu + 29),
				(u32)band);
		}

		reorder_node_free(
			*(u16 *)((u32)mpdu + 22),
			*(u8 *)((u32)mpdu + 29),
			(u32)band);

		mpdu = (u32 *)entry[0];
	} while (entry[0] != 0 &&
		 *(u16 *)((u32)mpdu + 24) == ((last_seq + 1) & 0xFFF));

unlock:
	if (band != 0 || wifi_dbdc_mode)
		hw_mutex_unlock(ba_mutex_5g);
	else
		hw_mutex_unlock(ba_mutex_2g);

	return last_seq;
}

u32 ba_state_update(u32 sn, u32 check_type, u32 entry_addr)
{
	u16 ref_sn = *(u16 *)(entry_addr + 20);
	u8 state_adj = *(u8 *)(entry_addr + 22) - 2;

	if (ref_sn == (u16)sn) {
		if (state_adj > 1 || check_type != 3)
			return 0;
	} else if (state_adj > 1) {
		return 0;
	}

	if (wifi_debug_flags & 4) {
		u8 band = *(u8 *)(entry_addr + 25);
		u32 base = (band == 1) ? counter_base_5g :
			   (band != 0) ? counter_base_tri :
			   counter_base_2g;
		(*(u32 *)(base + 196))++;
	}

	{
		u8 flag = *(u8 *)(entry_addr + 23);

		*(u32 *)(entry_addr + 12) = 0;
		if (flag != 0)
			return 1;
	}

	*(u16 *)(entry_addr + 18) = ref_sn;
	return 1;
}

/* ================================================================
 * WiFi packet header / statistics helpers
 * ================================================================ */

static int wifi_pkt_hdr_len(u32 buf_id, u32 band)
{
	u32 desc = (buf_id << 12) +
		   ((wifi_buf_id_base & 0x3FFFFFFF) | 0x40000000);
	u32 flags;
	int len;

	if (*(u32 *)(desc + 8) & 0x32000000) {
		if (wifi_debug_flags & 4) {
			u32 base = (band == 1) ? counter_base_5g :
				   (band != 0) ? counter_base_tri :
				   counter_base_2g;
			(*(u32 *)(base + 76))++;
		}
		return -1;
	}

	flags = *(u32 *)(desc + 4);
	if (!(flags & 0x4000))
		return -1;

	len = (flags & 0x800) ? 56 : 40;
	if (flags & 0x1000)
		len = (flags & 0x800) ? 64 : 48;
	if (flags & 0x2000)
		len += 8;
	if (flags & 0x8000)
		len += 72;

	return len;
}

static void wifi_update_stats(u32 dir, u32 port, u8 state, u32 type,
			      u32 tid, u32 byte_cnt)
{
	if (port > 15) {
		if (type != 1 && type != 15)
			goto tid_stats;
		if (dir != 1)
			goto global_2g;
		goto global_5g;
	}

	if (dir != 1) {
		u32 idx = port * 2;
		u32 lo = stats_bytes_2g[idx];

		stats_bytes_2g[idx] = lo + byte_cnt;
		stats_bytes_2g[idx + 1] += (lo + byte_cnt < lo);

		lo = stats_pkts_2g[idx];
		stats_pkts_2g[idx] = lo + 1;
		stats_pkts_2g[idx + 1] += (lo + 1 < lo);

		wifi_port_state_2g[port] = state;

		if (type != 1 && type != 15)
			goto tid_stats;
global_2g:
		wifi_global_bytes_hi +=
			(wifi_global_bytes_lo + byte_cnt < wifi_global_bytes_lo);
		wifi_global_bytes_lo += byte_cnt;
		wifi_global_pkts_hi +=
			(wifi_global_pkts_lo + 1 < wifi_global_pkts_lo);
		wifi_global_pkts_lo++;
	} else {
		u32 idx = port * 2;
		u32 lo = stats_bytes_5g[idx];

		stats_bytes_5g[idx] = lo + byte_cnt;
		stats_bytes_5g[idx + 1] += (lo + byte_cnt < lo);

		lo = stats_pkts_5g[idx];
		stats_pkts_5g[idx] = lo + 1;
		stats_pkts_5g[idx + 1] += (lo + 1 < lo);

		wifi_port_state_5g[port] = state;

		if (type != 1 && type != 15)
			goto tid_stats;
global_5g:
		wifi_global_bytes_5g_hi +=
			(wifi_global_bytes_5g_lo + byte_cnt <
			 wifi_global_bytes_5g_lo);
		wifi_global_bytes_5g_lo += byte_cnt;
		wifi_global_pkts_5g_hi +=
			(wifi_global_pkts_5g_lo + 1 <
			 wifi_global_pkts_5g_lo);
		wifi_global_pkts_5g_lo++;
	}

tid_stats:
	if ((s8)tid >= 0) {
		u32 off = (dir * 128 + tid) * 2;
		u32 lo;

		lo = wifi_tid_pkt_cnt[off];
		wifi_tid_pkt_cnt[off] = lo + 1;
		wifi_tid_pkt_cnt[off + 1] += (lo + 1 < lo);

		lo = wifi_tid_byte_cnt[off];
		wifi_tid_byte_cnt[off] = lo + byte_cnt;
		wifi_tid_byte_cnt[off + 1] += (lo + byte_cnt < lo);
	}
}

u32 ba_scan_entries(u32 band)
{
	u32 wcid, tid;
	u32 max_wcid = (wifi_dbdc_mode == 0) ? 150 : 300;
	u32 entry_addr = 0;

	for (wcid = 1; wcid <= max_wcid; wcid++) {
		u32 base_off = (wcid - 1) * 224;

		for (tid = 0; tid < 8; tid++) {
			if (wifi_dbdc_mode == 0) {
				if (band != 0)
					entry_addr = ba_table_a + base_off +
						     tid * 28;
				else
					entry_addr = ba_table_b + base_off +
						     tid * 28;
			} else {
				if (wcid > 150)
					entry_addr = ba_table_b +
						     (wcid - 151) * 224 +
						     tid * 28;
				else
					entry_addr = ba_table_a + base_off +
						     tid * 28;
				if (*(volatile u8 *)(entry_addr + 25) !=
				    (u8)band)
					continue;
			}

			if (*(volatile u8 *)(entry_addr + 24) != 4)
				continue;
			if (*(volatile u16 *)(entry_addr + 8) == 0)
				continue;
		}
	}
	return entry_addr;
}

void ba_flush_all(u32 band)
{
	u32 wcid, tid;
	u32 max_wcid = (wifi_dbdc_mode == 0) ? 150 : 300;
	u32 entry_addr;

	for (wcid = 1; wcid <= max_wcid; wcid++) {
		u32 base_off = (wcid - 1) * 224;

		for (tid = 0; tid < 8; tid++) {
			if (wifi_dbdc_mode == 0) {
				if (band != 0)
					entry_addr = ba_table_a + base_off +
						     tid * 28;
				else
					entry_addr = ba_table_b + base_off +
						     tid * 28;
			} else {
				if (wcid > 150)
					entry_addr = ba_table_b +
						     (wcid - 151) * 224 +
						     tid * 28;
				else
					entry_addr = ba_table_a + base_off +
						     tid * 28;
				if (*(u8 *)(entry_addr + 25) != (u8)band)
					continue;
			}

			if (*(u8 *)(entry_addr + 24) != 4)
				continue;

			ba_flush_entry((u32 *)entry_addr);

			if (wifi_debug_flags & 4) {
				u32 base = (*(u8 *)(entry_addr + 25) == 1) ?
					   counter_base_5g :
					   (*(u8 *)(entry_addr + 25) != 0) ?
					   counter_base_tri : counter_base_2g;
				(*(u32 *)(base + 200))++;
			}
		}
	}
}
