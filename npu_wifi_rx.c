/*
 * AN75XX NPU firmware - WiFi receive path (kite)
 * Cores 1 and 2 drain the rings, reorder per BA window, queue for core 3.
 */

#include "npu_internal.h"
#include "npu_wifi.h"

#ifdef WIFI_KITE

#define KITE_RX_RING_LAST	1535
#define KITE_RX_BUF_LEN		0xDAC
#define KITE_PIPE_ENTRIES	3200

/* the host changes these at run time */
#define kite_fast_flag()	(*(volatile u8 *)&wifi_debug_flags)
#define kite_driver_model()	(*(volatile u8 *)&wifi_driver_model)

/* DBDC drivers run both bands on core 1 */
#define kite_dbdc_model(m)	((u8)((m) - 1) <= 1)

/* the fields of a received frame, laid out as a reorder node */
struct kite_node {
	u32 next;
	u32 msdu_head;
	u32 msdu_tail;
	u16 msdu_cnt;
	u16 pad0;
	u8 hdr;
	u8 tid;
	u8 wcid;
	u8 bss;
	u16 buf_id;
	u16 idx;
	u16 sn;
	u16 len;
	u8 amsdu;
	u8 pool;
	u16 pad1;
	u32 tick;
};

u32 wifi_pipeline_base;
#if MAX_CORE_NUM > 2
static u16 wifi_pipeline_widx;
#endif
static u8 rxd_2g_kick;

/* ================================================================
 * Rx statistics
 * ================================================================ */

/* per-BSS, per-WCID and apcli counters */
static void kite_rx_stats(u32 dir, u32 port, u8 state, u32 type,
			  u32 wcid, u32 bytes)
{
	if (port <= 15) {
		if (dir != 1) {
			wifi_u64_add(&stats_bytes_2g[port * 2], bytes);
			wifi_u64_add(&stats_pkts_2g[port * 2], 1);
			wifi_port_band_2g[port] = state;
		} else {
			wifi_u64_add(&stats_bytes_5g[port * 2], bytes);
			wifi_u64_add(&stats_pkts_5g[port * 2], 1);
			wifi_port_band_5g[port] = state;
		}
	}
	if (type == 1 || type == 15) {
		if (dir != 1) {
			wifi_u64_add(apcli_byte_count_2g, bytes);
			wifi_u64_add(apcli_count_2g, 1);
		} else {
			wifi_u64_add(apcli_byte_count_5g, bytes);
			wifi_u64_add(apcli_count_5g, 1);
		}
	}
	if ((s8)wcid >= 0 && dir <= 1) {
		wifi_u64_add(&npu_rx_pkts_entry[dir][wcid * 2], 1);
		wifi_u64_add(&npu_rx_bytes_entry[dir][wcid * 2], bytes);
	}
}

/* ================================================================
 * Rx descriptor parser
 * ================================================================ */

/* -1: untranslated, errored or non-QoS data frames go to the host */
static int kite_rx_parse(u32 buf_id, struct kite_node *n, u32 len, u32 band)
{
	u32 buf = wifi_pkt_va(buf_id);
	u32 w1 = *(u32 *)(buf + 4);
	u32 w2 = *(u32 *)(buf + 8);
	u32 raw = w2 & 0x3F;
	u32 bss = ((w2 & 0x30) ? raw - 16 : raw) & 0xFF;
	u32 wcid = w1 & 0xFF;
	u32 hdr, pay, fc;

	if (!(w1 & 0x4000)) {
		pay = (len - 18 - 24) & 0xFFFF;
		wifi_cnt_inc(band, (w2 & 0x32000000) ? 76 : 84);
		kite_rx_stats(band, bss, raw, raw, wcid, pay);
		return -1;
	}

	hdr = 40;
	if (w1 & 0x800)
		hdr += 16;
	if (w1 & 0x1000)
		hdr += 8;
	if (w1 & 0x2000)
		hdr += 8;
	if (w1 & 0x8000)
		hdr += 72;
	pay = (len - 2 - hdr) & 0xFFFF;

	if (w2 & 0x32000000) {
		wifi_cnt_inc(band, 76);
		kite_rx_stats(band, bss, raw, raw, wcid, pay);
		return -1;
	}

	/* 802.11 frame control: data needs QoS and must not be 0xC0 */
	fc = *(u16 *)(buf + 24);
	if ((fc & 8) && (!(fc & 0x80) || (fc & 0xC0) == 0xC0)) {
		kite_rx_stats(band, bss, raw, raw, wcid, pay);
		return -1;
	}

	n->tid = (w2 >> 16) & 0xF;
	n->wcid = wcid;
	n->sn = *(u16 *)(buf + 32) >> 4;
	n->buf_id = buf_id;
	n->amsdu = *(u32 *)(buf + 16) & 3;
	n->len = len;
	n->bss = bss;
	n->hdr = hdr;

	wifi_cnt_inc(band, 84);
	kite_rx_stats(band, bss, raw, raw, wcid, pay);
	if (band == 1)
		*(u32 *)(wcid_counter_base_5g + wcid * 4) += pay;
	else
		*(u32 *)(wcid_counter_base_2g + wcid * 4) += pay;
	return 0;
}

/* offset of the 802.3 frame for the no-BA test mode */
static s32 kite_rx_hdr_len(u32 buf_id, u32 band)
{
	u32 buf = wifi_pkt_va(buf_id);
	u32 w1 = *(u32 *)(buf + 4);
	u32 len;

	if (*(u32 *)(buf + 8) & 0x32000000) {
		wifi_cnt_inc(band, 76);
		return -1;
	}
	if (!(w1 & 0x4000))
		return -1;

	len = (w1 & 0x800) ? 56 : 40;
	if (w1 & 0x1000)
		len += 8;
	if (w1 & 0x2000)
		len += 8;
	if (w1 & 0x8000)
		len += 72;
	return len;
}

/* ================================================================
 * BA reorder window classifier
 * ================================================================ */

/* -1 when the window took the frame, else the caller sends it up */
static s32 kite_classify(s32 buf_id, u32 len, u32 band)
{
	struct kite_node pn;
	u32 *entry, e, node;
	u32 sn, last, win, cnt;
	u32 pool = 1;
	u16 idx = 0xFFFF;
	s32 hdr, fs;

	if (wifi_no_ba_test != 0) {
		hdr = kite_rx_hdr_len(buf_id, band);
		if (hdr == -1)
			return buf_id;
		/* the BSS is unknown here, pass 0 */
		pkt_enqueue_bridge((u16)buf_id, len & 0xFFFF, hdr, band, 0);
		return -1;
	}

	if (kite_rx_parse(buf_id, &pn, len, band) == -1)
		return buf_id;

	if (pn.wcid == 0) {
		buf_id_free(0, band, pn.buf_id);
		wifi_cnt_inc(band, 232);
		return -1;
	}

	if (wifi_dbdc_mode != 0) {
		if (pn.wcid > 150)
			e = ba_table_a + 28 * (8 * (pn.wcid - 151) + pn.tid);
		else
			e = ba_table_b + 28 * (8 * (pn.wcid - 1) + pn.tid);
	} else {
		e = (band != 0 ? ba_table_b : ba_table_a) +
		    28 * (8 * (pn.wcid - 1) + pn.tid);
	}
	entry = (u32 *)e;

	if (*(u8 *)(e + 24) == 0) {
		ba_flush_entry(entry);
		return buf_id;
	}
	if (*(u8 *)(e + 24) == 3) {
		/* first frame after ADDBA: start the window here */
		ba_flush_entry(entry);
		*(u16 *)(e + 18) = (pn.sn - 1) & 0xFFF;
		*(u16 *)(e + 20) = pn.sn;
		*(u8 *)(e + 24) = 4;
		*(u8 *)(e + 22) = pn.amsdu;
	}
	*(u8 *)(e + 25) = band;

	if (pn.amsdu != 0)
		wifi_cnt_inc(band, 72);

	if (kite_fast_flag() == 0 &&
	    (*(u32 *)(wifi_pkt_va(buf_id) + 4) & 0x4000))
		pn.sn = *(u16 *)(wifi_pkt_va(buf_id) + 32) >> 4;
	sn = pn.sn;

	ba_state_update(sn, pn.amsdu, e);
	last = *(u16 *)(e + 18);

	while (1) {
		win = *(u16 *)(e + 16);

		if (((last + 1) & 0xFFF) == sn)
			goto in_order;

		if (last == sn) {
			/* duplicate */
			wifi_cnt_inc(band, 56);
			if (kite_fast_flag() & 1)
				pkt_enqueue_bridge(pn.buf_id, pn.len, pn.hdr,
						   band, pn.bss);
			else
				buf_id_free(0, band, pn.buf_id);
			if (pn.amsdu <= 1)
				entry[3] = 0;
			*(u8 *)(e + 23) = 1;
			*(u8 *)(e + 22) = pn.amsdu;
			*(u16 *)(e + 20) = sn;
			return -1;
		}

		if ((sn - last) & 0x800) {
			/* behind the window */
			if (pn.amsdu <= 1)
				entry[3] = 0;
			*(u8 *)(e + 22) = pn.amsdu;
			*(u8 *)(e + 23) = 2;
			*(u16 *)(e + 20) = sn;
			if ((s32)win < (s32)(last - sn) ||
			    ((((s32)((win >> 1) + sn + 1) % 4095) - (s32)last) &
			     0x800) ||
			    ((s32)win < (s32)(sn - last) &&
			     (s32)(sn + win) < 4096)) {
				/* too far back: the peer restarted */
				pkt_enqueue_bridge(pn.buf_id, pn.len, pn.hdr,
						   band, pn.bss);
				*(u16 *)(e + 18) = sn;
				return -1;
			}
			if (kite_fast_flag() & 1) {
				pkt_enqueue_bridge(pn.buf_id, pn.len, pn.hdr,
						   band, pn.bss);
				return -1;
			}
			buf_id_free(0, band, pn.buf_id);
			wifi_cnt_inc(band, 60);
			return -1;
		}

		if ((sn - ((last + win + 1) & 0xFFF)) & 0x800)
			break;

		/* past the window: slide it so sn is its last slot */
		fs = (s32)(sn - win) + 1;
		if (fs < 0)
			fs = (s32)(sn - win) + 0x1001;
		fs = (fs - 1) & 0xFFF;
		ba_indicate_le_seq(entry, fs);
		*(u16 *)(e + 18) = fs;
		last = ba_seq_scan(entry, fs);
		if (last == 0xFFFF)
			last = *(u16 *)(e + 18);
		else
			*(u16 *)(e + 18) = last;
		*(u8 *)(e + 23) = 4;
	}

	/* inside the window: hold it in SN order */
	wifi_cnt_inc(band, 64);
	node = reorder_node_alloc(band, &pool, &idx);
	if (node == 0) {
		/* drop the frame, keep its buffer */
		wifi_cnt_inc(band, 80);
		return -1;
	}

	*(u8 *)(node + 16) = pn.hdr;
	*(u16 *)(node + 20) = pn.buf_id;
	*(u16 *)(node + 24) = sn;
	*(u16 *)(node + 22) = idx;
	*(u16 *)(node + 26) = pn.len;
	*(u8 *)(node + 17) = pn.tid;
	*(u8 *)(node + 28) = pn.amsdu;
	*(u8 *)(node + 18) = pn.wcid;
	*(u8 *)(node + 29) = pool;
	*(u32 *)node = 0;
	*(u8 *)(node + 19) = pn.bss;
	*(u32 *)(node + 4) = 0;
	*(u32 *)(node + 8) = 0;
	*(u32 *)(node + 32) = KITE_TICK;

	if (*(u8 *)(e + 25) | wifi_dbdc_mode)
		hw_mutex_lock(ba_mutex_5g);
	else
		hw_mutex_lock(ba_mutex_2g);

	if (entry[3] != 0) {
		/* the next MSDU of the A-MSDU being held */
		u32 sub = entry[3];
		u32 tail = *(u32 *)(sub + 8);

		*(u16 *)(sub + 12) += 1;
		*(u32 *)node = 0;
		if (tail != 0)
			*(u32 *)tail = node;
		else
			*(u32 *)(sub + 4) = node;
		*(u32 *)(sub + 8) = node;
	} else {
		u32 *prev = entry;
		u32 cur = entry[0];

		while (cur != 0 && ((*(u16 *)(cur + 24) - sn) & 0x800)) {
			prev = (u32 *)cur;
			cur = *(u32 *)cur;
		}
		if (cur != 0 && *(u16 *)(cur + 24) == sn) {
			/* already held */
			reorder_node_free(idx, pool, *(u8 *)(e + 25));
			buf_id_free(0, *(u8 *)(e + 25), pn.buf_id);
			wifi_cnt_inc(*(u8 *)(e + 25), 208);
		} else {
			*(u32 *)node = cur;
			*prev = node;
			*(u16 *)(e + 8) += 1;
			entry[3] = (pn.amsdu <= 1) ? 0 : node;
		}
	}
	cnt = *(u16 *)(e + 8);

	if (*(u16 *)(e + 16) < cnt)
		wifi_cnt_inc(*(u8 *)(e + 25), 228);

	if (*(u8 *)(e + 25) | wifi_dbdc_mode)
		hw_mutex_unlock(ba_mutex_5g);
	else
		hw_mutex_unlock(ba_mutex_2g);

	*(u8 *)(e + 22) = pn.amsdu;
	*(u16 *)(e + 20) = sn;
	if (pn.amsdu <= 1)
		entry[3] = 0;
	*(u8 *)(e + 23) = 3;
	return -1;

in_order:
	wifi_cnt_inc(band, 52);
	pkt_enqueue_bridge(pn.buf_id, pn.len, pn.hdr, *(u8 *)(e + 25),
			   pn.bss);
	if (pn.amsdu <= 1) {
		*(u16 *)(e + 18) = sn;
		last = ba_seq_scan(entry, sn);
		if (last != 0xFFFF)
			*(u16 *)(e + 18) = last;
		entry[3] = 0;
	}
	*(u8 *)(e + 22) = pn.amsdu;
	*(u16 *)(e + 20) = sn;
	*(u8 *)(e + 23) = 0;
	return -1;
}

/* ================================================================
 * Frames spanning several rx descriptors
 * ================================================================ */

/* give n descriptors back to the chip unchanged */
static u32 kite_rx_skip(u32 ring, u32 idx, u32 n, u32 band, u32 off)
{
	u32 desc;

	while (n--) {
		desc = ring + idx * 16;
		*(u16 *)(desc + 6) = (*(u16 *)(desc + 6) & 0x4000) |
				     KITE_RX_BUF_LEN;
		idx = (idx >= KITE_RX_RING_LAST) ? 0 : idx + 1;
		if (off)
			wifi_cnt_inc(band, off);
	}
	return idx;
}

/* up to 0xDAC copied into one buffer, up to 0x4000 sent in segments,
 * longer dropped; returns the next index */
static u32 kite_rx_multi(u32 band, u32 idx, u32 len, u32 count)
{
	u32 ring = band ? rxd_base_5g : rxd_base_2g;
	s16 *tbl = band ? (s16 *)rxd_5g_bufid_table : (s16 *)rxd_2g_bufid_base;
	u32 n = count & 0xFF;
	u32 desc, w1, frag, off, dst, retry;
	u8 flags, seg;
	s32 id;

	wifi_cnt_inc(band, 116);

	if (len >= 0x4000) {
		wifi_cnt_inc(band, 132);
		idx = kite_rx_skip(ring, idx, n, band, 0);
		goto out;
	}

	if (len > KITE_RX_BUF_LEN) {
		wifi_cnt_inc(band, 120);
		seg = 0;
		while (n != 0) {
			id = buf_id_alloc_hw(0, band);
			if (id == -1) {
				wifi_cnt_inc(band, 132);
				idx = kite_rx_skip(ring, idx, n, band, 0);
				goto out;
			}
			seg++;
			desc = ring + idx * 16;
			w1 = *(u32 *)(desc + 4);
			frag = (w1 >> 16) & 0x3FFF;
			/* bit 1 last segment, 4:2 segment, 7:5 count */
			flags = ((w1 >> 29) & 2) | (seg << 2) |
				(((count & 7) << 5) & 0xFE);
			if (pkt_forward(tbl[idx], len & 0xFFFF, 0, 0, band,
					flags, frag, 0, 0) != 0) {
				for (retry = wifi_retry_limit & 0xFF; retry;
				     retry--)
					if (pkt_forward(tbl[idx], len & 0xFFFF,
							0, 0, band, flags,
							frag, 0, 0) == 0)
						break;
				if (retry == 0) {
					/* core 3 is stuck: drop the rest */
					idx = kite_rx_skip(ring, idx, n, band,
							   124);
					goto out;
				}
			}
			tbl[idx] = id;
			*(u32 *)desc = wifi_pkt_va(id);
			*(u16 *)(desc + 6) = (*(u16 *)(desc + 6) & 0x4000) |
					     KITE_RX_BUF_LEN;
			n--;
			idx = (idx >= KITE_RX_RING_LAST) ? 0 : idx + 1;
		}
		goto out;
	}

	wifi_cnt_inc(band, 128);
	id = buf_id_alloc_hw(0, band);
	if (id == -1) {
		wifi_cnt_inc(band, 0);
		idx = kite_rx_skip(ring, idx, n, band, 0);
		goto out;
	}

	dst = wifi_pkt_va(id);
	off = 0;
	while (n != 0) {
		desc = ring + idx * 16;
		frag = *(u16 *)(desc + 6) & 0x3FFF;
		npu_memcpy((void *)(dst + off),
			   (void *)wifi_pkt_va(tbl[idx]), frag);
		*(u16 *)(desc + 6) = (*(u16 *)(desc + 6) & 0x4000) |
				     KITE_RX_BUF_LEN;
		n--;
		off += frag;
		idx = (idx >= KITE_RX_RING_LAST) ? 0 : idx + 1;
	}
	if (pkt_forward(id, len & 0xFFFF, 0, 0, band, 2, len & 0xFFFF,
			0, 0) != 0)
		buf_id_free(0, band, (u16)id);

out:
	if (band != 0)
		rxd_5g_cpu_idx = idx;
	else
		rxd_2g_cpu_idx = idx;
	return idx;
}

/* ================================================================
 * Rx rings
 * ================================================================ */

/* walk the descriptors of one frame; 0 when it is not complete */
static u32 kite_rx_frame(u32 ring, u32 *ridx, u32 *len, u32 band)
{
	u32 idx = *ridx, n = 0, desc, w1;

	*len = 0;
	while (1) {
		n++;
		desc = ring + idx * 16;
		w1 = *(volatile u32 *)(desc + 4);
		if (!(w1 & 0x80000000))
			return 0;
		wifi_cnt_inc(band, 8);
		w1 = *(volatile u32 *)(desc + 4);
		*len += (w1 >> 16) & 0x3FFF;
		if (w1 & 0x40000000)
			break;
		idx = (idx >= KITE_RX_RING_LAST) ? 0 : idx + 1;
	}
	*ridx = idx;
	return n;
}

/* 2.4G ring: one frame per call */
static void kite_rx_2g(void)
{
	u32 ridx = rxd_2g_cpu_idx;
	u32 ring = rxd_base_2g;
	u32 len, n, desc, next, done;
	s32 id, old = -1, r;

	wifi_cnt_inc(0, 4);
	if (wifi_no_ba_test == 0 &&
	    KITE_TICK - rxd_2g_flush_tick > 9) {
		ba_timeout_scan(0);
		rxd_2g_flush_tick = KITE_TICK;
	}

	n = kite_rx_frame(ring, &ridx, &len, 0);
	if (n == 0)
		return;

	if (n != 1) {
		next = kite_rx_multi(0, rxd_2g_cpu_idx, len, n);
	} else {
		desc = ring + ridx * 16;
		next = (ridx >= KITE_RX_RING_LAST) ? 0 : ridx + 1;
		id = buf_id_alloc_hw(0, 0);
		if (id != -1) {
			old = ((s16 *)rxd_2g_bufid_base)[ridx];
			((u16 *)rxd_2g_bufid_base)[ridx] = id;
			*(u32 *)desc = wifi_pkt_dma(id);
		} else {
			ba_flush_all(0);
		}
		*(u16 *)(desc + 6) = (*(u16 *)(desc + 6) & 0x4000) |
				     KITE_RX_BUF_LEN;
		rxd_2g_cpu_idx = next;
		if (id == -1)
			wifi_cnt_inc(0, 0x10);
	}
	done = next ? next - 1 : KITE_RX_RING_LAST;

	/* give the chip the consumed descriptors every 128 frames */
	if ((s8)++rxd_2g_kick < 0) {
		*(u32 *)(pcie_base_2g + 8) = done;
		rxd_2g_kick = 0;
	}

	if (old == -1)
		return;
	wifi_cnt_inc(0, 0xC);
	r = kite_classify(old, len, 0);
	if (r == -1)
		return;
	if (pkt_forward(old, len & 0xFFFF, 0, 0, 0, 2, len & 0xFFFF,
			r, 0) != 0) {
		buf_id_free(0, 0, (u16)old);
		wifi_cnt_inc(0, 0x20);
	}
}

/* 5G ring: one frame per call */
static void kite_rx_5g(void)
{
	u32 ridx = rxd_5g_cpu_idx;
	u32 ring = rxd_base_5g;
	u32 len, n, desc, next;
	s32 id, old = -1, r;
#if MAX_CORE_NUM > 2
	volatile u32 *slot;
#endif

	wifi_cnt_inc(1, 4);
	if (wifi_no_ba_test == 0 &&
	    KITE_TICK - rxd_5g_flush_tick > 9) {
		ba_timeout_scan(1);
		rxd_5g_flush_tick = KITE_TICK;
	}

	n = kite_rx_frame(ring, &ridx, &len, 1);
	if (n == 0)
		return;

	if (len - 504 <= 16)
		wifi_cnt_inc(1, 0xF0);
	else if (len - 1510 <= 16)
		wifi_cnt_inc(1, 0xF4);
	else
		wifi_cnt_inc(1, 0xF8);

	if (n != 1) {
		next = kite_rx_multi(1, rxd_5g_cpu_idx, len, n);
		*(u32 *)(pcie_base_5g + 8) = next ? next - 1 :
					     KITE_RX_RING_LAST;
		return;
	}

	desc = ring + ridx * 16;
	next = (ridx >= KITE_RX_RING_LAST) ? 0 : ridx + 1;
	id = buf_id_alloc_hw(0, 1);
	if (id == -1) {
		ba_flush_all(1);
		*(u16 *)(desc + 6) = (*(u16 *)(desc + 6) & 0x4000) |
				     KITE_RX_BUF_LEN;
		rxd_5g_cpu_idx = next;
		wifi_cnt_inc(1, 0x10);
	} else {
		old = (s16)rxd_5g_bufid_table[ridx];
		rxd_5g_bufid_table[ridx] = id;
		*(u32 *)desc = wifi_pkt_dma(id);
		*(u16 *)(desc + 6) = (*(u16 *)(desc + 6) & 0x4000) |
				     KITE_RX_BUF_LEN;
		rxd_5g_cpu_idx = next;
	}

	/* give the chip the consumed descriptors every 128 frames */
	if (next != 0 && ((next - 1) & 127) == 0)
		*(u32 *)(pcie_base_5g + 8) = next - 1;

	if (old == -1)
		return;

	/* no classifier core on AN7552: always classify here */
	if (MAX_CORE_NUM <= 2 || !(kite_fast_flag() & 1)) {
		r = kite_classify(old, len, 1);
		if (r == -1)
			return;
		if (pkt_forward(old, len & 0xFFFF, 0, 0, 1, 2, len & 0xFFFF,
				r, 0) != 0) {
			buf_id_free(0, 1, (u16)old);
			wifi_cnt_inc(1, 0x20);
		}
		return;
	}

#if MAX_CORE_NUM > 2
	/* pipeline mode: core 2 classifies */
	slot = (volatile u32 *)(wifi_pipeline_base + wifi_pipeline_widx * 8);
	if (*slot != (u32)-1) {
		npu_printf("%s(1) Enter PushIndex1 %d\n",
			   "pipeline1_pkt_enqueue", wifi_pipeline_widx);
		wifi_cnt_inc(1, 0);
		if (pkt_forward(old, len & 0xFFFF, 0, 0, 1, 2, len & 0xFFFF,
				0, 0) != 0) {
			buf_id_free(0, 1, (u16)old);
			wifi_cnt_inc(1, 0x20);
		}
		return;
	}
	*(volatile u16 *)(slot + 1) = len;
	*slot = old;
	wifi_pipeline_widx = (wifi_pipeline_widx + 1 == KITE_PIPE_ENTRIES) ?
			     0 : wifi_pipeline_widx + 1;
#endif
}

/* ================================================================
 * Per-core loops
 * ================================================================ */

/* core 1 */
void __attribute__((noreturn)) kite_core1_loop(void)
{
	npu_dbg_loop(NDBG_TAG('K', 'R', 'X', '1'));
	while (1) {
		npu_dbg_poll();
		if (rxd_5g_init_done != 0)
			goto rx_5g;
#if MAX_CORE_NUM > 2
		while (rxd_2g_init_done != 0 &&
		       kite_dbdc_model(kite_driver_model())) {
#else
		while (rxd_2g_init_done != 0) {
#endif
			kite_rx_2g();
			if (rxd_5g_init_done != 0) {
rx_5g:
				kite_rx_5g();
			}
		}
	}
}

#if MAX_CORE_NUM > 2
/* 2.4G on core 2 while the driver is not DBDC */
static void kite_core2_rx_2g(void)
{
	u8 model = kite_driver_model();

	do {
		if (rxd_2g_init_done != 0 && model == 0) {
			kite_rx_2g();
			model = kite_driver_model();
			if (kite_dbdc_model(model))
				return;
		} else if (kite_dbdc_model(model)) {
			return;
		}
	} while (!(kite_fast_flag() & 1));
}

/* pipeline mode: classify the 5G frames core 1 queued */
static void __attribute__((noreturn)) kite_pipeline_worker(void)
{
	volatile u32 *slot;
	u16 ridx = 0;
	s32 id, r;
	u16 len;

	npu_printf("[NPU1]  %s...\n", "npu_offload_5G_2");
	while (rxd_5g_init_done == 0)
		npu_dbg_poll();

	npu_dbg_loop(NDBG_TAG('K', 'P', 'I', 'P'));
	while (1) {
		npu_dbg_poll();
		slot = (volatile u32 *)(wifi_pipeline_base + ridx * 8);
		id = (s32)*slot;
		if (id == -1)
			continue;
		len = *(volatile u16 *)(slot + 1);
		ridx = (ridx + 1 == KITE_PIPE_ENTRIES) ? 0 : ridx + 1;
		*slot = (u32)-1;
		*(volatile u16 *)(slot + 1) = 0;

		r = kite_classify(id, len, 1);
		/* back to the buffer manager */
		if (r != -1 &&
		    pkt_forward((s16)id, len, 0, 0, 1, 1, len, r, 0) != 0) {
			buf_id_free(0, 1, (u16)id);
			wifi_cnt_inc(1, 0x20);
		}
	}
}

/* core 2, after the timer ISR */
void __attribute__((noreturn)) kite_core2_loop(void)
{
	npu_dbg_loop(NDBG_TAG('K', 'R', 'X', '2'));
	while (!(kite_fast_flag() & 1)) {
		npu_dbg_poll();
		kite_core2_rx_2g();
	}
	kite_pipeline_worker();
}
#endif

/* core 0 bridge init; the rings come up per band
 * from SET_WAIT_DESC */
void wifi_bridge_init(void)
{
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
}

#endif /* WIFI_KITE */

/* AN7581 core 5 has nothing to do */
void __attribute__((noreturn)) wifi_bridge_loop(void)
{
	npu_dbg_idle();
}
