/*
 * AN75XX NPU firmware - block ack reorder engine (kite)
 * Kite chips deliver frames unordered; the NPU keeps each BA window.
 */

#include "npu_internal.h"
#include "npu_wifi.h"

#ifdef WIFI_KITE

/* ================================================================
 * Reorder nodes: 2000 in SRAM, 5000 in the funcId 7 DRAM block
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
	wifi_cnt_inc(band, 212);

	if (reorder_pri_ridx == REORDER_PRI_POOL_SIZE)
		reorder_pri_ridx = 0;
	if (reorder_sec_ridx == REORDER_SEC_POOL_SIZE)
		reorder_sec_ridx = 0;

	hw_mutex_unlock(reorder_free_mutex);
}

/* a node from the primary pool, else the secondary */
u32 reorder_node_alloc(u32 band, u32 *pool_type, u16 *idx_out)
{
	u16 widx, next;

	hw_mutex_lock(reorder_alloc_mutex);

	widx = reorder_pri_widx;
	next = (widx == REORDER_PRI_POOL_SIZE - 1) ? 0 : widx + 1;
	if (reorder_pri_ridx != next) {
		reorder_pri_widx++;
		*idx_out = *(u16 *)(reorder_pri_idx_pool + 2 * widx);
		if (reorder_pri_widx == REORDER_PRI_POOL_SIZE)
			reorder_pri_widx = 0;
		wifi_cnt_inc(band, 220);
		hw_mutex_unlock(reorder_alloc_mutex);
		return ba_node_pool_base + REORDER_NODE_SIZE * (*idx_out);
	}

	widx = reorder_sec_widx;
	next = (widx == REORDER_SEC_POOL_SIZE - 1) ? 0 : widx + 1;
	if (reorder_sec_ridx != next) {
		*pool_type = 0;
		reorder_sec_widx = widx + 1;
		*idx_out = *(u16 *)(reorder_sec_idx_pool + 2 * widx);
		if (reorder_sec_widx == REORDER_SEC_POOL_SIZE)
			reorder_sec_widx = 0;
		wifi_cnt_inc(band, 224);
		hw_mutex_unlock(reorder_alloc_mutex);
		return wifi_dram_ba_node_addr +
		       REORDER_NODE_SIZE * (*idx_out);
	}

	wifi_cnt_inc(band, 216);
	hw_mutex_unlock(reorder_alloc_mutex);
	return 0;
}

/* ================================================================
 * Frame out of the reorder engine
 * ================================================================ */

/* host for a rate-limited BSS, a waiting band or no offload; else the
 * wire over TDMA from hdr, where the 802.3 frame starts */
int pkt_enqueue_bridge(u32 buf_id, u32 pkt_len, u32 hdr,
		       u32 band, u32 bss)
{
	u32 size, tx_len;
	int ret;

	if (ratelimit_table[band * 16 + bss] != 0) {
		ret = pkt_forward((s16)buf_id, pkt_len, 0, 0, band, 2,
				  pkt_len, -2, 0);
		/* back to the buffer manager */
		if (ret != 0)
			buf_id_free(0, band, buf_id);
		return ret;
	}

	if ((wifi_force_to_cpu &&
	     ((band == 1 && wifi_wait_band_5g == 1) ||
	      (band == 0 && wifi_wait_band_2g == 1))) ||
	    wifi_band_cap == 0) {
		ret = pkt_forward((s16)buf_id, pkt_len, 0, 0, band, 2,
				  pkt_len, 0, 0);
		if (ret != 0)
			buf_id_free(0, band, buf_id);
		return ret;
	}

	size = (pkt_len - hdr) & 0xFFFF;
	tx_len = (size - 2) & 0xFFFF;

	if ((wifi_debug_flags & 4) && band <= 1)
		wifi_u64_add((u32 *)((band ? counter_base_5g :
				      counter_base_2g) + 100), tx_len);
	wifi_cnt_inc(band, 84);

	if (size - 506 <= 16)
		wifi_cnt_inc(band, 92);
	else if (size - 86 <= 8)
		wifi_cnt_inc(band, 88);
	else if (size - 1512 <= 16)
		wifi_cnt_inc(band, 96);

	ret = tdma_tx_submit(buf_id, tx_len, wifi_pkt_va(buf_id) + 2 + hdr,
			     band);
	if (ret != 0)
		buf_id_free(0, band, buf_id);
	return ret;
}

/* ================================================================
 * BA entries: 28 bytes per TID, 8 TIDs per WCID; nodes are 36 bytes
 * ================================================================ */

/* node: 0 next, 4/8 msdu head/tail, 12 msdus, 16 hdr, 17 tid,
 * 18 wcid, 19 bss, 20 buf, 22 idx, 24 sn, 26 len,
 * 28 amsdu, 29 pool, 32 tick */
/* entry: 0/4 head/tail, 8 count, 12 msdu chain, 16 win, 18 last sn,
 * 20 ref sn, 22 amsdu, 23 flag, 24 state, 25 band */

static void ba_lock(u32 *entry)
{
	if (*(u8 *)((u32)entry + 25) | wifi_dbdc_mode)
		hw_mutex_lock(ba_mutex_5g);
	else
		hw_mutex_lock(ba_mutex_2g);
}

static void ba_unlock(u32 *entry)
{
	if (*(u8 *)((u32)entry + 25) | wifi_dbdc_mode)
		hw_mutex_unlock(ba_mutex_5g);
	else
		hw_mutex_unlock(ba_mutex_2g);
}

static void ba_check_node(u32 *node, u32 band, int msdu)
{
	s16 bid = *(s16 *)((u32)node + 20);
	u16 len = *(u16 *)((u32)node + 26);

	if (bid <= 5600 && len != 0 && *(u8 *)((u32)node + 28) <= 3)
		return;
	if (msdu)
		npu_printf("[%d,%s]!!ERROR:[%s] bufid is over: %d, or pkt len is msdu_blk->datalength=%d\n",
			   652, "ba_indicate_reordering_mpdus_le_seq",
			   (band == 1) ? "5G" : "2.4G", (s32)bid, (u32)len);
	else
		npu_printf("[%d,%s]!![%s]ERROR: bufid is over: %d, or pkt len is mpdu_blk->datalength=%d\n",
			   638, "ba_indicate_reordering_mpdus_le_seq",
			   (band == 1) ? "5G" : "2.4G", (s32)bid, (u32)len);
}

/* take the head MPDU off the entry */
static u32 *ba_pop_mpdu(u32 *entry)
{
	u32 *mpdu = (u32 *)entry[0];

	*(u16 *)((u32)entry + 8) -= 1;
	entry[0] = mpdu[0];
	if (mpdu[0] == 0) {
		mpdu[0] = 0;
		entry[1] = 0;
	}
	return mpdu;
}

/* send an MPDU and its MSDUs on, then free their nodes; mode 0 is
 * a flush, 1 an in-order release, 2 one that checks the nodes */
static void ba_release_mpdu(u32 *mpdu, u32 band, int mode)
{
	u32 *msdu;

	if (mode == 2)
		ba_check_node(mpdu, band, 0);
	pkt_enqueue_bridge(*(u16 *)((u32)mpdu + 20),
			   *(u16 *)((u32)mpdu + 26),
			   *(u8 *)((u32)mpdu + 16), band,
			   *(u8 *)((u32)mpdu + 19));
	if (mode)
		wifi_cnt_inc(band, 68);

	while (mpdu[1] != 0) {
		msdu = (u32 *)mpdu[1];
		*(u16 *)((u32)mpdu + 12) -= 1;
		mpdu[1] = msdu[0];
		if (msdu[0] == 0) {
			msdu[0] = 0;
			mpdu[2] = 0;
		}
		if (mode == 2)
			ba_check_node(msdu, band, 1);
		pkt_enqueue_bridge(*(u16 *)((u32)msdu + 20),
				   *(u16 *)((u32)msdu + 26),
				   *(u8 *)((u32)msdu + 16), band,
				   *(u8 *)((u32)mpdu + 19));
		if (mode)
			wifi_cnt_inc(band, 68);
		reorder_node_free(*(u16 *)((u32)msdu + 22),
				  *(u8 *)((u32)msdu + 29), band);
	}

	reorder_node_free(*(u16 *)((u32)mpdu + 22),
			  *(u8 *)((u32)mpdu + 29), band);
}

/* release every held frame */
void ba_flush_entry(u32 *entry)
{
	u8 band = *(u8 *)((u32)entry + 25);
	u32 *mpdu;

	ba_lock(entry);
	while (entry[0] != 0) {
		mpdu = ba_pop_mpdu(entry);
		*(u16 *)((u32)entry + 18) = *(u16 *)((u32)mpdu + 24);
		ba_release_mpdu(mpdu, band, 0);
	}
	entry[3] = 0;
	ba_unlock(entry);
}

/* release the held frames up to and including seq */
void ba_indicate_le_seq(u32 *entry, u32 seq)
{
	u8 band = *(u8 *)((u32)entry + 25);
	u16 sn;

	ba_lock(entry);
	while (entry[0] != 0) {
		sn = *(u16 *)(entry[0] + 24);
		if (sn != (u16)seq && ((u16)(sn - seq) & 0x800) == 0)
			break;
		ba_release_mpdu(ba_pop_mpdu(entry), band, 2);
	}
	ba_unlock(entry);
}

/* release the frames that follow seq without a gap, return the last
 * one released or 0xFFFF */
u32 ba_seq_scan(u32 *entry, u32 seq)
{
	u8 band = *(u8 *)((u32)entry + 25);
	u32 last = 0xFFFF;
	u32 *mpdu;

	ba_lock(entry);
	if (entry[0] != 0 &&
	    *(u16 *)(entry[0] + 24) == ((seq + 1) & 0xFFF)) {
		do {
			mpdu = ba_pop_mpdu(entry);
			last = *(u16 *)((u32)mpdu + 24);
			ba_release_mpdu(mpdu, band, 1);
		} while (entry[0] != 0 &&
			 *(u16 *)(entry[0] + 24) == ((last + 1) & 0xFFF));
	}
	ba_unlock(entry);
	return last;
}

/* a retransmit of the last A-MSDU restarts the window */
u32 ba_state_update(u32 sn, u32 check_type, u32 entry_addr)
{
	u16 ref_sn = *(u16 *)(entry_addr + 20);
	u8 state_adj = *(u8 *)(entry_addr + 22) - 2;
	u32 r;

	if (ref_sn == (u16)sn) {
		if (state_adj > 1 || check_type != 3)
			return 0;
	} else if (state_adj > 1) {
		return 0;
	}

	wifi_cnt_inc(*(u8 *)(entry_addr + 25), 196);

	*(u32 *)(entry_addr + 12) = 0;
	if (*(u8 *)(entry_addr + 23) != 0)
		return 1;

	*(u16 *)(entry_addr + 18) = ref_sn;
	r = ba_seq_scan((u32 *)entry_addr, ref_sn);
	if (r != 0xFFFF)
		*(u16 *)(entry_addr + 18) = r;
	return 1;
}

/* release the frames held longer than flushone_timeout */
static void ba_timeout_entry(u32 *entry, u32 now)
{
	u16 sn, r;

	while (entry[0] != 0 &&
	       now - *(u32 *)(entry[0] + 32) > wifi_flushone_timeout) {
		sn = *(u16 *)(entry[0] + 24);
		ba_indicate_le_seq(entry, sn);
		*(u16 *)((u32)entry + 18) = sn;
		r = ba_seq_scan(entry, sn);
		if (r != 0xFFFF)
			*(u16 *)((u32)entry + 18) = r;
		wifi_cnt_inc(*(u8 *)((u32)entry + 25), 200);
	}
}

/* the active entry of (wcid, tid) the band owns, or NULL */
static u32 *ba_band_entry(u32 band, u32 wcid, u32 tid)
{
	u32 off = (wcid - 1) * 224 + tid * 28;
	u32 e;

	if (wifi_dbdc_mode == 0) {
		e = (band != 0 ? ba_table_b : ba_table_a) + off;
		if (e == 0 || *(u8 *)(e + 24) != 4)
			return NULL;
		return (u32 *)e;
	}
	if (wcid > 150)
		e = ba_table_a + off - 150 * 224;
	else
		e = ba_table_b + off;
	if (*(u8 *)(e + 25) != band || *(u8 *)(e + 24) != 4)
		return NULL;
	return (u32 *)e;
}

/* every 10 ticks from the rx loop */
void ba_timeout_scan(u32 band)
{
	u32 now = timer_slow_tick;
	u32 max = wifi_dbdc_mode ? 300 : 150;
	u32 wcid, tid;
	u32 *e;

	for (wcid = 1; wcid <= max; wcid++) {
		for (tid = 0; tid < 8; tid++) {
			e = ba_band_entry(band, wcid, tid);
			if (e != NULL && *(u16 *)((u32)e + 8) != 0)
				ba_timeout_entry(e, now);
		}
	}
}

/* no rx buffer left: release everything the band holds */
void ba_flush_all(u32 band)
{
	u32 max = wifi_dbdc_mode ? 300 : 150;
	u32 wcid, tid;
	u32 *e;

	for (wcid = 1; wcid <= max; wcid++) {
		for (tid = 0; tid < 8; tid++) {
			e = ba_band_entry(band, wcid, tid);
			if (e == NULL)
				continue;
			ba_flush_entry(e);
			wifi_cnt_inc(*(u8 *)((u32)e + 25), 200);
		}
	}
}

#endif /* WIFI_KITE */
