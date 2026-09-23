/*
 * AN75XX NPU firmware - WiFi receive path (kite)
 *
 * The classifier decides what the reorder window does with a frame. The
 * multi-descriptor handler joins a frame that spans several ring
 * entries. The two process functions drain one band each, and the two
 * loops below them are what cores 1 and 5 run.
 *
 * The bridge state and the two functions that initialize it live here
 * too, because this file is the only reader of that state.
 */

#include "npu_internal.h"
#include "npu_wifi.h"

#ifdef WIFI_KITE

/* ================================================================
 * WiFi packet classifier
 *
 * Main BA reorder window handler. Parameters come packed in
 * stack variables from the caller (wifi_rx_process).
 * ================================================================ */

static int wifi_pkt_classify(u32 buf_id, u32 pkt_len, u32 iface)
{
	u32 band = iface - 6;
	u32 *entry;
	u32 sn, last_sn, win_size;
	u32 pool_type = 1;
	u16 node_idx = 0xFFFF;
	u32 wcid, tid;
	u8 amsdu_cnt;

	/* extract fields from descriptor */
	u32 desc_addr = (buf_id << 12) +
			((wifi_buf_id_base & 0x3FFFFFFF) | 0x40000000);
	u16 pkt_buf_id = *(u16 *)(desc_addr);
	u16 orig_len = *(u16 *)(desc_addr + 2);
	u16 seq_num;
	u8 fwd_type;

	wcid = *(u8 *)(desc_addr + 0x1E);
	tid = *(u8 *)(desc_addr + 0x1D);
	amsdu_cnt = *(u8 *)(desc_addr + 0x1F);
	fwd_type = *(u8 *)(desc_addr + 0x1C);

	if (wifi_classifier_bypass != 0) {
		if (buf_id != (u32)-1)
			pkt_enqueue_bridge(buf_id & 0xFFFF, pkt_len,
					   buf_id & 0xFFFF, iface,
					   fwd_type);
		return -1;
	}

	if (buf_id == (u32)-1)
		return buf_id;

	if (wcid == 0) {
		buf_id_free(0, band, pkt_buf_id);
		if (wifi_debug_flags & 4) {
			u32 base = (band == 1) ? counter_base_5g :
				   (band != 0) ? counter_base_tri :
				   counter_base_2g;
			(*(u32 *)(base + 232))++;
		}
		return -1;
	}

	/* locate BA entry: 28 bytes per TID, 8 TIDs per WCID */
	if (wifi_dbdc_mode != 0) {
		if (wcid > 150)
			entry = (u32 *)(ba_table_b +
					28 * (8 * (wcid - 151) + tid));
		else
			entry = (u32 *)(ba_table_a +
					28 * (8 * (wcid - 1) + tid));
	} else {
		u32 off = 28 * (8 * (wcid - 1) + tid);

		if (iface == 6)
			entry = (u32 *)(ba_table_b + off);
		else
			entry = (u32 *)(ba_table_a + off);
	}

	/* state 0: flush and return */
	if (*(u8 *)((u32)entry + 24) == 0) {
		ba_flush_entry(entry);
		return buf_id;
	}

	/* state 3: re-init to state 4 */
	if (*(u8 *)((u32)entry + 24) == 3) {
		ba_flush_entry(entry);
		*(u16 *)((u32)entry + 18) = seq_num - 1;
		*(u16 *)((u32)entry + 20) = seq_num;
		*(u8 *)((u32)entry + 24) = 4;
		*(u8 *)((u32)entry + 22) = amsdu_cnt;
	}

	*(u8 *)((u32)entry + 25) = (u8)band;

	/* A-MSDU aggregation tracking */
	if (amsdu_cnt != 0 && (wifi_debug_flags & 4)) {
		u32 base = (band == 1) ? counter_base_5g :
			   (band != 0) ? counter_base_tri :
			   counter_base_2g;
		(*(u32 *)(base + 72))++;
	}

	/* read SN from descriptor if hardware SN extraction available */
	if (wifi_debug_flags != 0 ||
	    (*(u32 *)(desc_addr + 4) & 0x4000) == 0) {
		sn = seq_num;
	} else {
		sn = *(u16 *)(desc_addr + 0x20) >> 4;
	}

	ba_state_update(sn, amsdu_cnt, (u32)entry);

	last_sn = *(u16 *)((u32)entry + 18);
	win_size = *(u16 *)((u32)entry + 16);

	/* exact next sequence → fast path forward */
	if (((last_sn + 1) & 0xFFF) == sn)
		goto forward_pkt;

	/* duplicate */
	if (last_sn == sn) {
		if (wifi_debug_flags & 4) {
			u32 base = (band == 1) ? counter_base_5g :
				   (band != 0) ? counter_base_tri :
				   counter_base_2g;
			(*(u32 *)(base + 56))++;
		}
		if (wifi_debug_flags & 1)
			pkt_enqueue_bridge(pkt_buf_id, orig_len,
					   pkt_buf_id, band, fwd_type);
		else
			buf_id_free(0, band, pkt_buf_id);

		if (amsdu_cnt <= 1)
			entry[3] = 0;
		*(u8 *)((u32)entry + 23) = 1;
		*(u8 *)((u32)entry + 22) = amsdu_cnt;
		*(u16 *)((u32)entry + 20) = sn;
		return -1;
	}

	/* backward (sn behind last_sn in 12-bit space) */
	if (((sn - last_sn) & 0x800) != 0) {
		if (amsdu_cnt <= 1)
			entry[3] = 0;
		*(u8 *)((u32)entry + 22) = amsdu_cnt;
		*(u8 *)((u32)entry + 23) = 2;
		*(u16 *)((u32)entry + 20) = sn;

		if (win_size < last_sn - sn ||
		    (((s32)(((win_size >> 1) + sn + 1) % 4095 -
		      last_sn) & 0x800) != 0) ||
		    (win_size < sn - last_sn &&
		     sn + win_size < 4096)) {
			pkt_enqueue_bridge(pkt_buf_id, orig_len,
					   pkt_buf_id, band, fwd_type);
			*(u16 *)((u32)entry + 18) = sn;
			return -1;
		}

		if (wifi_debug_flags & 1) {
			pkt_enqueue_bridge(pkt_buf_id, orig_len,
					   pkt_buf_id, band, fwd_type);
			return -1;
		}

		buf_id_free(0, band, pkt_buf_id);

		if (wifi_debug_flags & 4) {
			u32 base = (band == 1) ? counter_base_5g :
				   (band != 0) ? counter_base_tri :
				   counter_base_2g;
			(*(u32 *)(base + 60))++;
		}
		return -1;
	}

	/* forward jump: out of window ahead */
	if (((sn - ((last_sn + win_size + 1) & 0xFFF)) & 0x800) != 0)
		goto reorder_insert;

	/* way ahead: flush old entries and re-sync */
	{
		u32 flush_sn = sn - win_size;
		s32 adj = flush_sn + 1;

		if (adj < 0)
			adj = flush_sn + 4097;
		flush_sn = (adj - 1) & 0xFFF;

		ba_indicate_le_seq(entry, flush_sn);
		*(u16 *)((u32)entry + 18) = flush_sn;

		{
			u32 scan_result = ba_seq_scan(entry, flush_sn);

			if (scan_result == 0xFFFF)
				scan_result = *(u16 *)((u32)entry + 18);
			else
				*(u16 *)((u32)entry + 18) = scan_result;
		}

		*(u8 *)((u32)entry + 23) = 4;

		if (sn == ((*(u16 *)((u32)entry + 18) + 1) & 0xFFF))
			goto forward_pkt;
		if (sn == *(u16 *)((u32)entry + 18))
			goto forward_pkt; /* duplicate after flush */

		win_size = *(u16 *)((u32)entry + 16);
		last_sn = *(u16 *)((u32)entry + 18);
		if (((sn - last_sn) & 0x800) != 0)
			goto forward_pkt; /* behind after flush */
		if (((sn - ((last_sn + win_size + 1) & 0xFFF)) & 0x800) != 0)
			goto reorder_insert;

		/* still way ahead after flush → give up, forward directly */
		goto forward_pkt;
	}

reorder_insert:
	if (wifi_debug_flags & 4) {
		u32 base = (band == 1) ? counter_base_5g :
			   (band != 0) ? counter_base_tri :
			   counter_base_2g;
		(*(u32 *)(base + 64))++;
	}

	{
		u32 node = reorder_node_alloc(band, &pool_type, &node_idx);

		if (node == 0) {
			if (wifi_debug_flags & 4) {
				u32 base = (band == 1) ? counter_base_5g :
					   (band != 0) ? counter_base_tri :
					   counter_base_2g;
				(*(u32 *)(base + 80))++;
			}
			return -1;
		}

		/* populate reorder node (36 bytes) */
		*(u8 *)(node + 16) = (u8)buf_id;
		*(u16 *)(node + 20) = pkt_buf_id;
		*(u16 *)(node + 24) = sn;
		*(u16 *)(node + 22) = node_idx;
		*(u16 *)(node + 26) = orig_len;
		*(u8 *)(node + 28) = amsdu_cnt;
		*(u8 *)(node + 17) = (u8)tid;
		*(u8 *)(node + 29) = (u8)pool_type;
		*(u8 *)(node + 18) = (u8)wcid;
		*(u8 *)(node + 19) = fwd_type;
		*(u32 *)node = 0;
		*(u32 *)(node + 4) = 0;
		*(u32 *)(node + 8) = 0;
		*(u32 *)(node + 32) = timer_slow_tick;

		/* mutex for linked-list insertion */
		if (*(u8 *)((u32)entry + 25) | wifi_dbdc_mode)
			hw_mutex_lock(ba_mutex_5g);
		else
			hw_mutex_lock(ba_mutex_2g);

		/* insert into MPDU chain at entry[3] (sub-chain head) */
		if (entry[3] != 0) {
			u32 sub = entry[3];
			u32 *tail = (u32 *)*(u32 *)(sub + 8);

			++(*(u16 *)(sub + 12));
			*(u32 *)node = 0;
			if (tail != (u32 *)0)
				*tail = node;
			else
				*(u32 *)(sub + 4) = node;
			*(u32 *)(sub + 8) = node;
		} else {
			/* insert sorted by sequence number */
			u32 *prev = entry;
			u32 *cur = (u32 *)*entry;
			u32 cur_seq;

			if (cur != (u32 *)0) {
				cur_seq = *(u16 *)((u32)cur + 24);
				if (((cur_seq - sn) & 0x800) != 0) {
					while (1) {
						prev = cur;
						cur = (u32 *)*cur;
						if (cur == (u32 *)0)
							break;
						cur_seq = *(u16 *)
							  ((u32)cur + 24);
						if (((cur_seq - sn) &
						     0x800) == 0)
							goto check_dup;
					}
				} else {
check_dup:
					if (sn == cur_seq) {
						reorder_node_free(
							node_idx,
							(u8)pool_type,
							band);
						buf_id_free(0, band,
							    pkt_buf_id);
						if (wifi_debug_flags & 4) {
							u32 b = (band == 1) ?
								counter_base_5g :
								(band != 0) ?
								counter_base_tri :
								counter_base_2g;
							(*(u32 *)(b + 208))++;
						}
						goto unlock_done;
					}
				}
			}

			{
				u16 cnt = *(u16 *)((u32)entry + 8);

				*(u32 *)node = (u32)cur;
				*prev = node;
				*(u16 *)((u32)entry + 8) = cnt + 1;
				if (amsdu_cnt <= 1)
					entry[3] = 0;
				else
					entry[3] = node;
			}
		}

unlock_done:
		{
			u32 cnt = *(u16 *)((u32)entry + 8);

			if (*(u16 *)((u32)entry + 16) < cnt &&
			    (wifi_debug_flags & 4)) {
				u32 base = (band == 1) ? counter_base_5g :
					   (band != 0) ? counter_base_tri :
					   counter_base_2g;
				(*(u32 *)(base + 228))++;
			}
		}

		if ((*(u8 *)((u32)entry + 25) | wifi_dbdc_mode) == 0)
			hw_mutex_unlock(ba_mutex_2g);
		else
			hw_mutex_unlock(ba_mutex_5g);

		*(u8 *)((u32)entry + 22) = amsdu_cnt;
		*(u16 *)((u32)entry + 20) = sn;
		if (amsdu_cnt <= 1)
			entry[3] = 0;
		*(u8 *)((u32)entry + 23) = 3;
		return -1;
	}

forward_pkt:
	if (wifi_debug_flags & 4) {
		u32 base = (band == 1) ? counter_base_5g :
			   (band != 0) ? counter_base_tri :
			   counter_base_2g;
		(*(u32 *)(base + 52))++;
	}

	pkt_enqueue_bridge(pkt_buf_id, orig_len, pkt_buf_id, band, fwd_type);

	if (amsdu_cnt <= 1) {
		*(u16 *)((u32)entry + 18) = sn;
		{
			u32 scan_result = ba_seq_scan(entry, sn);

			if (scan_result != 0xFFFF)
				*(u16 *)((u32)entry + 18) = scan_result;
		}
		*(u8 *)((u32)entry + 22) = amsdu_cnt;
		*(u16 *)((u32)entry + 20) = sn;
		if (amsdu_cnt <= 1)
			entry[3] = 0;
	} else {
		*(u8 *)((u32)entry + 22) = amsdu_cnt;
		*(u16 *)((u32)entry + 20) = sn;
	}

	*(u8 *)((u32)entry + 23) = 0;
	return -1;
}

/* ================================================================
 * WiFi single-packet forward wrapper
 * ================================================================ */

static int wifi_pkt_forward_single(u32 buf_id, u16 pkt_len, u8 amsdu)
{
	u32 desc = ((wifi_buf_id_base & 0x3FFFFFFF) | 0x40000000) +
		   (buf_id << 12);

	return pkt_forward(buf_id,
			   *(u16 *)desc,
			   pkt_len,
			   amsdu,
			   (*(u8 *)(desc + 0xD) >= 0x24) ? 1 : 0,
			   2,
			   *(u16 *)desc,
			   0, 1);
}

/* ================================================================
 * WiFi multi-descriptor scatter/gather handler
 *
 * Handles packets spanning multiple descriptors in the RX ring.
 * Three size classes: jumbo (>=0x4000), large (>0xDAC), normal.
 * ================================================================ */

static u32 wifi_multi_desc_handler(u32 band, u32 start_idx, u32 size,
				   u32 count)
{
	u32 idx = start_idx;
	u32 ring_base = (band != 0) ? wifi_rxd_ring_5g : wifi_rxd_ring_2g;
	u16 *bufid_tbl = wifi_rxd_bufid_tbl;
	u32 desc_addr;
	u32 i;

	if (wifi_debug_flags & 4) {
		u32 base = (band == 1) ? counter_base_5g :
			   (band != 0) ? counter_base_tri :
			   counter_base_2g;
		(*(u32 *)(base + 116))++;
	}

	if (band == 0)
		bufid_tbl = (u16 *)wifi_rxd_idx_2g;

	/* jumbo: size >= 0x4000 */
	if (size >= 0x4000) {
		if (wifi_debug_flags & 4) {
			u32 base = (band == 1) ? counter_base_5g :
				   (band != 0) ? counter_base_tri :
				   counter_base_2g;
			(*(u32 *)(base + 132))++;
		}
		if (count != 0) {
			while (count) {
				desc_addr = ring_base + idx * 16;
				count = (u8)(count - 1);
				*(u16 *)(desc_addr + 6) =
					(*(u16 *)(desc_addr + 6) & 0x4000) |
					0xDAC;
				idx = (idx + 1 > 0x5FF) ? 0 : idx + 1;
			}
		}
		goto update_idx;
	}

	/* large: size > 0xDAC, needs scatter across pages */
	if (size > 0xDAC) {
		if (wifi_debug_flags & 4) {
			u32 base = (band == 1) ? counter_base_5g :
				   (band != 0) ? counter_base_tri :
				   counter_base_2g;
			(*(u32 *)(base + 120))++;
		}
		if (count != 0) {
			u32 frag_count = count << 29 >> 24;
			u8 frag_idx = 0;
			u32 remaining = size;

			while (1) {
				u32 new_buf_id = buf_id_alloc_hw(0, band);

				if (new_buf_id == (u32)-1) {
					if (wifi_debug_flags & 4) {
						u32 base =
							(band == 1) ?
							counter_base_5g :
							(band != 0) ?
							counter_base_tri :
							counter_base_2g;
						(*(u32 *)(base + 132))++;
					}
					while (count) {
						desc_addr = ring_base +
							    idx * 16;
						count = (u8)(count - 1);
						*(u16 *)(desc_addr + 6) =
							(*(u16 *)(desc_addr +
								  6) &
							 0x4000) | 0xDAC;
						idx = (idx + 1 > 0x5FF) ?
						      0 : idx + 1;
					}
					goto update_idx;
				}

				frag_idx++;
				desc_addr = ring_base + idx * 16;
				{
					u16 old_bufid =
						*(s16 *)(bufid_tbl +
							 (u32)idx);
					u32 frag_len =
						*(u16 *)(desc_addr + 6) &
						0x3FFF;
					u8 flags = ((*(u32 *)(desc_addr +
							      4) >> 29) &
						    2) |
						   (4 * frag_idx) |
						   (frag_count & 0xFE);
					u16 retry = wifi_retry_limit;

					if (pkt_forward(old_bufid, remaining,
							0, 0, band, flags,
							frag_len, 0, 0) != 0) {
						if (retry == 0)
							goto scatter_fail;
						do {
							retry--;
							if (pkt_forward(
								old_bufid,
								remaining,
								0, 0, band,
								flags,
								frag_len,
								0, 0) == 0)
								break;
						} while (retry != 0);
						if (retry == 0)
							goto scatter_fail;
					}

					{
						u16 old_done =
							*(u16 *)(desc_addr +
								 6) & 0x4000;
						u32 phy =
							((wifi_buf_id_base &
							  0x3FFFFFFF) |
							 0x40000000) +
							(new_buf_id << 12);

						*(s16 *)(bufid_tbl +
							 (u32)idx) =
							new_buf_id;
						*(u32 *)desc_addr = phy;
						*(u16 *)(desc_addr + 6) =
							old_done | 0xDAC;
					}

					count = (u8)(count - 1);
					idx = (idx + 1 > 0x5FF) ? 0 :
					      idx + 1;
					if (count == 0)
						goto update_idx;
				}
				continue;
scatter_fail:
				{
					u32 d = idx * 16;

					if (band != 0)
						d += wifi_rxd_ring_5g;
					else
						d += wifi_rxd_ring_2g;

					*(u16 *)(d + 6) =
						(*(u16 *)(d + 6) & 0x4000) |
						0xDAC;
					count = (u8)(count - 1);
					idx = (idx + 1 > 0x5FF) ? 0 :
					      idx + 1;

					if (wifi_debug_flags & 4) {
						u32 base =
							(band == 1) ?
							counter_base_5g :
							(band != 0) ?
							counter_base_tri :
							counter_base_2g;
						(*(u32 *)(base + 124))++;
					}

					if (count == 0)
						break;
				}
			}
		}
		goto update_idx;
	}

	/* normal: single-page allocation */
	if (wifi_debug_flags & 4) {
		u32 base = (band == 1) ? counter_base_5g :
			   (band != 0) ? counter_base_tri :
			   counter_base_2g;
		(*(u32 *)(base + 128))++;
	}

	{
		u32 new_buf_id = buf_id_alloc_hw(0, band);

		if (new_buf_id == (u32)-1) {
			if (wifi_debug_flags & 4) {
				u32 base = (band == 1) ? counter_base_5g :
					   (band != 0) ? counter_base_tri :
					   counter_base_2g;
				(*(u32 *)base)++;
			}
			while (count) {
				count = (u8)(count - 1);
				desc_addr = ring_base + idx * 16;
				if ((u8)(count - 1) == 0xFF)
					break;
				*(u16 *)(desc_addr + 6) =
					(*(u16 *)(desc_addr + 6) & 0x4000) |
					0xDAC;
				idx = (idx + 1 > 0x5FF) ? 0 : idx + 1;
			}
			goto update_idx;
		}

		if (count != 0) {
			u32 coalesce_off = 0;
			u32 dst = ((wifi_buf_id_base & 0x3FFFFFFF) |
				   0x40000000) + (new_buf_id << 12);

			for (i = 0; i < count; i++) {
				u32 d = ring_base + idx * 16;
				u16 old_id = *(s16 *)(bufid_tbl + (u32)idx);
				u32 src = ((wifi_buf_id_base & 0x3FFFFFFF) |
					   0x40000000) +
					  ((u32)old_id << 12);
				u16 frag_len = *(u16 *)(d + 6) & 0x3FFF;

				npu_memcpy((void *)(dst + coalesce_off),
					   (void *)src, frag_len);

				*(u16 *)(d + 6) =
					(*(u16 *)(d + 6) & 0x4000) | 0xDAC;
				count = (u8)(count - 1);
				coalesce_off += frag_len;
				idx = (idx + 1 > 0x5FF) ? 0 : idx + 1;
			}
		}

		if (pkt_forward(new_buf_id, size, 0, 0, band, 2, size,
				0, 0) != 0)
			buf_id_free(0, band, (u16)new_buf_id);
	}

update_idx:
	if (band != 0)
		wifi_rxd_idx_5g = idx;
	else
		wifi_rxd_idx_2g = idx;
	return idx;
}

/* WiFi bridge state */
static u8 wifi_bridge_enabled;
static u16 wifi_bridge_ch_count;
static u8 wifi_bridge_report;
static u8 wifi_bridge_active;
static u32 wifi_tx_pending;
static u32 wifi_rx_pending;
static u8 wifi_batch_count;
static u8 wifi_mode_flags;

/* WiFi TX/RX ring state */
u32 wifi_rx_ring_base_2g;
u32 wifi_tx_ring_base_5g;
static u32 wifi_rx_ridx_2g;
static u32 wifi_tx_ridx_5g;
static u32 wifi_rx_last_tick;
static u32 wifi_tx_last_tick;
static u32 wifi_rx_desc_base;
static u32 wifi_tx_desc_base;
static u16 *wifi_rx_bufid_table;
static u16 *wifi_tx_bufid_table;

/* WiFi pipeline queue state (core-to-core handoff) */
static u16 wifi_pipeline_widx;
u32 wifi_pipeline_base;

#define WIFI_RING_SIZE     1536
#define WIFI_RING_MASK     0x5FF
#define WIFI_DESC_SIZE     16
#define WIFI_DDONE_BIT     (1u << 31)
#define WIFI_LS_BIT        (1u << 30)
#define WIFI_DESC_LEN_MASK 0x3FFF0000
#define WIFI_DESC_LEN_SHIFT 16
#define WIFI_PKT_MAX       3500

/* WiFi state table bases (per-WCID) */
static u32 wifi_wcid_base_2g;
static u32 wifi_wcid_base_5g;

/* core 0 bridge init; the rings come up per band
 * from SET_WAIT_DESC */
void wifi_bridge_init(void)
{
#ifdef WIFI_KITE
	u32 i;

	wifi_no_ba_test = 0;
	wifi_band_cap = 1;
	wifi_retry_limit = 3;

	for (i = 0; i < 16; i++) {
		wifi_wait_state_5g[i] = 0;
		stats_bytes_5g[i * 2] = 0;
		stats_bytes_5g[i * 2 + 1] = 0;
		stats_pkts_5g[i * 2] = 0;
		stats_pkts_5g[i * 2 + 1] = 0;
		wifi_port_band_5g[i] = 0xFF;
	}
	for (i = 0; i < 16; i++) {
		wifi_wait_state_2g[i] = 0;
		stats_bytes_2g[i * 2] = 0;
		stats_bytes_2g[i * 2 + 1] = 0;
		stats_pkts_2g[i * 2] = 0;
		stats_pkts_2g[i * 2 + 1] = 0;
		wifi_port_band_2g[i] = 0xFF;
	}

	wifi_pcie_desc_alloc();
	wifi_wait_band_2g = 0;
	wifi_wait_band_5g = 0;
	wifi_ba_node_init();
	counter_init(2);
	wifi_queue_mutex_init();
	rxd_2g_init_done = 0;
	rxd_5g_init_done = 0;
#endif
}

/* periodic housekeeping: scan WCID tables */
static void wifi_periodic_check(u32 band)
{
	if (wifi_classifier_bypass == 0)
		ba_scan_entries(band);
}

static void wifi_flush_stale(u32 band)
{
	ba_flush_all(band);
}

/* WiFi RX processing (2.4G band) */
static void wifi_rx_process(void)
{
#ifdef HAS_WIFI
	u32 ridx = wifi_rx_ridx_2g;
	u32 desc_base = wifi_rx_ring_base_2g;
	u32 desc_addr;
	u32 pkt_len = 0;
	u32 desc_cnt = 0;
	u32 desc_w1;
	u32 new_buf_id;
	u16 old_buf_id;
	u32 next_ridx;

	if ((wifi_debug_flags & 4) && counter_base_2g)
		(*(u32 *)(counter_base_2g + 4))++;

	/* periodic housekeeping */
	if (wifi_bridge_report == 0 &&
	    (timer_slow_tick - wifi_rx_last_tick) > 9) {
		wifi_periodic_check(0);
		wifi_rx_last_tick = timer_slow_tick;
	}

	while (1) {
		desc_cnt++;
		desc_addr = desc_base + ridx * WIFI_DESC_SIZE;
		desc_w1 = *(volatile u32 *)(desc_addr + 4);

		if (!(desc_w1 & WIFI_DDONE_BIT))
			return;

		if ((wifi_debug_flags & 4) && counter_base_2g) {
			(*(u32 *)(counter_base_2g + 8))++;
			desc_w1 = *(volatile u32 *)(desc_addr + 4);
		}

		pkt_len += (desc_w1 >> WIFI_DESC_LEN_SHIFT) & 0x3FFF;

		if (desc_w1 & WIFI_LS_BIT)
			break;

		ridx = (ridx + 1 > WIFI_RING_MASK) ? 0 : ridx + 1;
	}

	if (desc_cnt != 1) {
		wifi_multi_desc_handler(0, wifi_rxd_idx_2g, pkt_len, (u8)desc_cnt);
		return;
	}

	/* allocate replacement buffer ID */
	new_buf_id = buf_id_alloc_hw(0, 0);
	next_ridx = ridx + 1;

	if (new_buf_id != (u32)-1) {
		old_buf_id = wifi_rx_bufid_table[ridx];
		wifi_rx_bufid_table[ridx] = (u16)new_buf_id;

		/* update descriptor with new buffer physical address */
		*(volatile u32 *)desc_addr =
			(((new_buf_id << 12) + wifi_buf_id_base) & 0x3FFFFFFF) | 0x80000000;
		*(volatile u16 *)(desc_addr + 6) =
			(*(volatile u16 *)(desc_addr + 6) & 0x4000) | 0xDAC;

		wifi_rx_ridx_2g = (next_ridx > WIFI_RING_MASK) ? 0 : next_ridx;
	} else {
		/* no buffer available */
		wifi_flush_stale(0);
		*(volatile u16 *)(desc_addr + 6) =
			(*(volatile u16 *)(desc_addr + 6) & 0x4000) | 0xDAC;
		wifi_rx_ridx_2g = (next_ridx > WIFI_RING_MASK) ? 0 : next_ridx;
		old_buf_id = (u16)-1;
		if ((wifi_debug_flags & 4) && counter_base_2g)
			(*(u32 *)(counter_base_2g + 0x10))++;
	}

	wifi_batch_count++;
	if (old_buf_id != (u16)-1) {
		int ret;

		if ((wifi_debug_flags & 4) && counter_base_2g)
			(*(u32 *)(counter_base_2g + 0x0C))++;

		ret = wifi_pkt_classify(old_buf_id, pkt_len & 0xFFFF, 0);
		if (ret != -1) {
			if (pkt_forward(old_buf_id, pkt_len & 0xFFFF, 0, 0,
					0, 2, pkt_len & 0xFFFF, ret, 0) != 0) {
				buf_id_free(0, 0, old_buf_id);
				if ((wifi_debug_flags & 4) && counter_base_2g)
					(*(u32 *)(counter_base_2g + 0x20))++;
			}
		}
	}

	/* batch counter wrap */
	if ((s8)wifi_batch_count < 0) {
		if (wifi_rx_desc_base)
			*(u32 *)(wifi_rx_desc_base + 8) = WIFI_RING_SIZE - 1;
		wifi_batch_count = 0;
	}
#endif
}

/* WiFi TX processing (5G band) */
static void wifi_tx_process(void)
{
#ifdef HAS_WIFI
	u32 ridx = wifi_tx_ridx_5g;
	u32 desc_base = wifi_tx_ring_base_5g;
	u32 desc_addr;
	u32 pkt_len = 0;
	u32 desc_cnt = 0;
	u32 desc_w1;
	u32 new_buf_id;
	u16 old_buf_id;
	u32 next_ridx;

	if ((wifi_debug_flags & 4) && counter_base_5g)
		(*(u32 *)(counter_base_5g + 4))++;

	if (wifi_bridge_report == 0 &&
	    (timer_slow_tick - wifi_tx_last_tick) > 9) {
		wifi_periodic_check(1);
		wifi_tx_last_tick = timer_slow_tick;
	}

	while (1) {
		desc_cnt++;
		desc_addr = desc_base + ridx * WIFI_DESC_SIZE;
		desc_w1 = *(volatile u32 *)(desc_addr + 4);

		if (!(desc_w1 & WIFI_DDONE_BIT))
			return;

		if ((wifi_debug_flags & 4) && counter_base_5g) {
			(*(u32 *)(counter_base_5g + 8))++;
			desc_w1 = *(volatile u32 *)(desc_addr + 4);
		}

		pkt_len += (desc_w1 >> WIFI_DESC_LEN_SHIFT) & 0x3FFF;

		if (desc_w1 & WIFI_LS_BIT)
			break;

		ridx = (ridx + 1 > WIFI_RING_MASK) ? 0 : ridx + 1;
	}

	/* packet size classification counters */
	if (pkt_len >= 504 && pkt_len <= 520) {
		if ((wifi_debug_flags & 4) && counter_base_5g)
			(*(u32 *)(counter_base_5g + 0xF0))++;
	} else if (pkt_len >= 1510 && pkt_len <= 1526) {
		if ((wifi_debug_flags & 4) && counter_base_5g)
			(*(u32 *)(counter_base_5g + 0xF4))++;
	} else {
		if ((wifi_debug_flags & 4) && counter_base_5g)
			(*(u32 *)(counter_base_5g + 0xF8))++;
	}

	if (desc_cnt != 1) {
		u32 result = wifi_multi_desc_handler(1, wifi_rxd_idx_5g,
						     pkt_len, (u8)desc_cnt);
		if (result != 0)
			*(u32 *)(wifi_rx_desc_base + 8) = result - 1;
		else
			*(u32 *)(wifi_rx_desc_base + 8) = WIFI_RING_SIZE - 1;
		return;
	}

	/* single descriptor: allocate new buffer */
	new_buf_id = buf_id_alloc_hw(0, 1);

	if (new_buf_id == (u32)-1) {
		wifi_flush_stale(1);
		*(volatile u16 *)(desc_addr + 6) =
			(*(volatile u16 *)(desc_addr + 6) & 0x4000) | 0xDAC;
		next_ridx = ridx + 1;
		wifi_tx_ridx_5g = (next_ridx > WIFI_RING_MASK) ? 0 : next_ridx;
		if ((wifi_debug_flags & 4) && counter_base_5g)
			(*(u32 *)(counter_base_5g + 0x10))++;
		return;
	}

	old_buf_id = wifi_tx_bufid_table[ridx];
	*(volatile u32 *)desc_addr =
		(((new_buf_id << 12) + wifi_buf_id_base) & 0x3FFFFFFF) | 0x80000000;
	wifi_tx_bufid_table[ridx] = (u16)new_buf_id;
	*(volatile u16 *)(desc_addr + 6) =
		(*(volatile u16 *)(desc_addr + 6) & 0x4000) | 0xDAC;
	next_ridx = ridx + 1;
	wifi_tx_ridx_5g = (next_ridx > WIFI_RING_MASK) ? 0 : next_ridx;

	if (old_buf_id == (u16)-1)
		return;

	/* pipeline mode: enqueue for core1 */
	if (wifi_debug_flags & 1) {
		u32 *slot = (u32 *)(wifi_pipeline_base + wifi_pipeline_widx * 8);

		if (*slot != (u32)-1) {
			/* pipeline slot full, forward directly */
			if ((wifi_debug_flags & 4) && counter_base_5g)
				(*(u32 *)counter_base_5g)++;
			if (pkt_forward(old_buf_id, pkt_len & 0xFFFF, 0, 0,
					1, 2, pkt_len & 0xFFFF, 0, 0) != 0) {
				buf_id_free(0, 1, old_buf_id);
				if ((wifi_debug_flags & 4) && counter_base_5g)
					(*(u32 *)(counter_base_5g + 0x20))++;
			}
			return;
		}

		*slot = old_buf_id;
		*(u16 *)(slot + 1) = pkt_len & 0xFFFF;
		wifi_pipeline_widx = (wifi_pipeline_widx + 1 == 3200) ?
			0 : wifi_pipeline_widx + 1;
		return;
	}

	/* non-pipeline: classify then forward */
	{
		int ret = wifi_pkt_classify(old_buf_id,
					    pkt_len & 0xFFFF, 1);

		if (ret != -1) {
			if (pkt_forward(old_buf_id, pkt_len & 0xFFFF, 0, 0,
					1, 2, pkt_len & 0xFFFF, ret,
					0) != 0) {
				buf_id_free(0, 1, old_buf_id);
				if ((wifi_debug_flags & 4) && counter_base_5g)
					(*(u32 *)(counter_base_5g + 0x20))++;
			}
		}
	}
#endif
}

/* WiFi bridge main loop: runs on core1, dispatches RX/TX */
void __attribute__((noreturn)) wifi_bridge_loop(void)
{
#ifdef HAS_WIFI
	while (1) {
		if (wifi_tx_pending != 0)
			goto do_tx;

		while (wifi_rx_pending != 0 && wifi_bridge_enabled) {
			wifi_rx_process();
			if (wifi_tx_pending != 0) {
do_tx:
				wifi_tx_process();
			}
		}
	}
#else
	while (1)
		;
#endif
}

static int npu_wifi_tx_kick_out_wrapper(void)
{
	npu_printf("%s() error! func not support\n",
		   "npu_wifi_tx_kick_out_wrapper");
	return 0;
}

static int npu_tdma_2_wifi_fast_path_wrapper(void)
{
	npu_printf("%s() error! func not support\n",
		   "npu_tdma_2_wifi_fast_path_wrapper");
	return 0;
}

/* WiFi pipeline 5G worker: runs on core1, dequeues from pipeline ring */
void __attribute__((noreturn)) wifi_pipeline_worker(void)
{
#ifdef HAS_WIFI
	u16 ridx = 0;

	npu_printf("[NPU1]  %s...\n", "npu_offload_5G_2");

	while (wifi_tx_pending == 0)
		;

	while (1) {
		u32 *slot = (u32 *)(wifi_pipeline_base + ridx * 8);
		u32 buf_id = *slot;
		u16 pkt_len;
		int ret;

		if (buf_id == (u32)-1)
			continue;

		pkt_len = *(u16 *)(wifi_pipeline_base + ridx * 8 + 4);
		ridx++;
		if (ridx == 3200)
			ridx = 0;

		*slot = (u32)-1;
		*(u16 *)(slot + 1) = 0;

		ret = wifi_pkt_classify(buf_id & 0xFFFF, pkt_len, 1);
		if (ret != -1 &&
		    pkt_forward(buf_id & 0xFFFF, pkt_len, 0, 0,
				1, 1, pkt_len, ret, 0) != 0) {
			buf_id_return(buf_id);
			if ((wifi_debug_flags & 4) && counter_base_5g)
				(*(u32 *)(counter_base_5g + 0x20))++;
		}
	}
#else
	while (1)
		;
#endif
}

#else /* !WIFI_KITE */

/* AN7581 core 5: nothing to do without a kite chip */
void __attribute__((noreturn)) wifi_bridge_loop(void)
{
	while (1)
		;
}

#endif /* WIFI_KITE */
