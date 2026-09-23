/*
 * AN75XX NPU firmware - packet forwarding rings (kite)
 *
 * piNode and rxNode hand a frame from the core that received it to
 * core 3, which gives it to the host. A frame that fits one buffer takes
 * the 16-byte piNode ring; a segment of a longer frame takes the 12-byte
 * rxNode ring, where core 3 puts the segments back together.
 */

#include "npu_internal.h"
#include "npu_wifi.h"


/* per-band per-queue stats: 16 queues x u64 */
u32 stats_bytes_2g[32];
u32 stats_pkts_2g[32];
u32 stats_bytes_5g[32];
u32 stats_pkts_5g[32];

#ifdef WIFI_KITE

#define PINODE_RING_SIZE_2G  256
#define PINODE_RING_SIZE_5G  512
#define RXNODE_RING_SIZE     128

/* ring bases (SRAM types 3/2 and 15/14) and indices */
static u32 pinode_base_2g;
static u32 pinode_base_5g;
static u32 rxnode_base_2g;
static u32 rxnode_base_5g;
static u16 pinode_widx_2g;
static u16 pinode_widx_5g;
static u16 pinode_ridx_2g;
static u16 pinode_ridx_5g;
static u16 rxnode_widx_2g;
static u16 rxnode_widx_5g;
static u16 rxnode_ridx_2g;
static u16 rxnode_ridx_5g;

/* pkt_queue_init, from npu_init */
void wifi_pkt_queue_init(u32 band)
{
	u32 i, count, node;

	if (band != 0) {
		npu_printf("[NPU]%s  %s...\n", "5G", "pkt_queue_init");
		if (band != 1)
			return;
		pinode_widx_5g = 0;
		pinode_ridx_5g = 0;
		pinode_base_5g = sram_buf_alloc(2);
		rxnode_widx_5g = 0;
		rxnode_ridx_5g = 0;
		rxnode_base_5g = sram_buf_alloc(14);
		count = PINODE_RING_SIZE_5G;
	} else {
		npu_printf("[NPU]%s  %s...\n", "2.4", "pkt_queue_init");
		pinode_widx_2g = 0;
		pinode_ridx_2g = 0;
		pinode_base_2g = sram_buf_alloc(3);
		rxnode_widx_2g = 0;
		rxnode_ridx_2g = 0;
		rxnode_base_2g = sram_buf_alloc(15);
		count = PINODE_RING_SIZE_2G;
	}

	for (i = 0; i < count; i++) {
		node = (band ? pinode_base_5g : pinode_base_2g) + i * 16;
		*(u32 *)node = 0xFFFFFFFF;
		*(u32 *)(node + 4) = 0;
		*(u16 *)(node + 8) = 0;
		*(u8 *)(node + 11) = 0;
		*(u8 *)(node + 10) = 0;
	}

	for (i = 0; i < RXNODE_RING_SIZE; i++) {
		node = (band ? rxnode_base_5g : rxnode_base_2g) + i * 12;
		*(u32 *)node = 0xFFFFFFFF;
		*(u32 *)(node + 4) = 0;
		*(u8 *)(node + 8) = 0;
	}
}

/* ================================================================
 * Packet forwarding engine
 *
 * Queues a frame for core 3. Returns 1 when the ring is full.
 * ================================================================ */

int pkt_forward(u32 buf_id, u32 pkt_len, s16 wcid, u8 amsdu,
		u32 band, u8 fwd_type, u32 orig_len,
		int classify_result, u8 tunnel)
{
	u32 node, widx, info, q;

	if (orig_len != pkt_len) {
		/* one segment of a longer frame */
		if (band != 0) {
			widx = rxnode_widx_5g;
			node = rxnode_base_5g + widx * 12;
		} else {
			widx = rxnode_widx_2g;
			node = rxnode_base_2g + widx * 12;
		}
		if (*(u8 *)(node + 8) & 1)
			return 1;

		*(u16 *)(node + 4) = (u16)pkt_len;
		*(u16 *)(node + 6) = (u16)orig_len;
		*(u32 *)node = buf_id;
		*(u8 *)(node + 8) = fwd_type | 1;
		wifi_cnt_inc(band, 148);

		widx = (widx + 1 != RXNODE_RING_SIZE) ? widx + 1 : 0;
		if (band != 0)
			rxnode_widx_5g = widx;
		else
			rxnode_widx_2g = widx;
	} else {
		/* a whole frame; cores 1 and 2 both fill this ring */
		hw_mutex_lock(queue_mutex_2g);
		if (band != 0) {
			widx = pinode_widx_5g;
			node = pinode_base_5g + widx * 16;
		} else {
			widx = pinode_widx_2g;
			node = pinode_base_2g + widx * 16;
		}
		if (*(u8 *)(node + 10) & 1) {
			hw_mutex_unlock(queue_mutex_2g);
			wifi_cnt_inc(band, 28);
			return 1;
		}

		*(u8 *)(node + 12) = (classify_result == -2);
		*(u16 *)(node + 6) = (u16)pkt_len;
		*(u16 *)(node + 8) = (u16)orig_len;
		*(u32 *)node = buf_id;
		*(u16 *)(node + 4) = wcid;
		*(u8 *)(node + 11) = amsdu;
		*(u8 *)(node + 10) = fwd_type | 1;
		wifi_cnt_inc(band, 24);

		widx++;
		if (band != 0)
			pinode_widx_5g = (widx != PINODE_RING_SIZE_5G) ? widx : 0;
		else
			pinode_widx_2g = (widx != PINODE_RING_SIZE_2G) ? widx : 0;
		hw_mutex_unlock(queue_mutex_2g);
	}

	/* per-queue byte/packet stats; errored frames do not count */
	info = *(u32 *)(wifi_pkt_va(buf_id) + 0x88);
	if (info & 0x32000000)
		return 0;
	q = info & 0x3F;
	if (info & 0x30)
		q = (u8)(q - 16);
	if (q > 15 || tunnel != 0)
		return 0;

	if (band == 1) {
		wifi_u64_add(&stats_bytes_5g[q * 2], (orig_len - 98) & 0xFFFF);
		wifi_u64_add(&stats_pkts_5g[q * 2], 1);
	} else {
		wifi_u64_add(&stats_bytes_2g[q * 2], (orig_len - 98) & 0xFFFF);
		wifi_u64_add(&stats_pkts_2g[q * 2], 1);
	}
	return 0;
}

/* a frame the BME returned: hand it to the host */
int pkt_forward_bme(s32 buf_id, u32 wcid, u32 info)
{
	u32 buf = wifi_pkt_va(buf_id);
	u16 len = *(u16 *)buf;

	return pkt_forward(buf_id, len, (s16)wcid, (u8)info,
			   *(u8 *)(buf + 13) >= 36, 2, len, 0, 1);
}

/* ================================================================
 * Host ring drains, core 3
 *
 * pinode_drain: one whole frame per call
 * rxnode_drain: the segments of one frame
 * ================================================================ */

void pinode_drain(u32 band)
{
	u16 ridx;
	u32 *entry;
	u32 ring_size;
	s32 buf_id;
	int ret;

	if (band != 0) {
		ridx = pinode_ridx_5g;
		entry = (u32 *)(pinode_base_5g + (u32)ridx * 16);
		ring_size = PINODE_RING_SIZE_5G;
	} else {
		ridx = pinode_ridx_2g;
		entry = (u32 *)(pinode_base_2g + (u32)ridx * 16);
		ring_size = PINODE_RING_SIZE_2G;
	}

	if (!(*(u8 *)((u32)entry + 10) & 1)) {
		wifi_cnt_inc(band == 1, 20);
		return;
	}

	wifi_cnt_inc(band == 1, 36);

	buf_id = (s32)entry[0];
	if (buf_id < 0) {
		wifi_cnt_inc(band == 1, 40);
	} else if (*(u16 *)((u32)entry + 6) == 0) {
		wifi_cnt_inc(band == 1, 44);
	} else {
		ret = host_ring_submit(wifi_pkt_dma(buf_id),
				       *(u16 *)((u32)entry + 6),
				       band,
				       *(u16 *)((u32)entry + 4),
				       *(u8 *)((u32)entry + 11),
				       *(u8 *)((u32)entry + 10) >> 2,
				       *(u16 *)((u32)entry + 8),
				       (*(u8 *)((u32)entry + 10) >> 1) & 1,
				       *(u8 *)((u32)entry + 12));
		wifi_cnt_inc(band == 1, ret ? 108 : 48);
		buf_id_free(0, band, *(u16 *)entry);
	}

	entry[0] = (u32)-1;
	entry[1] = 0;
	*(u16 *)((u32)entry + 8) = 0;
	*(u8 *)((u32)entry + 11) = 0;
	*(u8 *)((u32)entry + 10) = 0;

	ridx = ((ridx + 1) != ring_size) ? ridx + 1 : 0;
	if (band != 0)
		pinode_ridx_5g = ridx;
	else
		pinode_ridx_2g = ridx;
	wifi_cnt_inc(band == 1, 20);
}

static int rxnode_submit(u32 node, u32 band)
{
	return host_ring_submit(wifi_pkt_dma(*(u32 *)node),
				*(u16 *)(node + 4), band, 0, 0,
				*(u8 *)(node + 8) >> 2,
				*(u16 *)(node + 6),
				(*(u8 *)(node + 8) & 2) != 0, 0);
}

void rxnode_drain(u32 band)
{
	u16 ridx, scan;
	u32 base, node, flags, total, seg, next;
	u32 tries = 0, count = 0, submit = 0;
	u32 cb = (band == 1);
	u32 retry;

	if (band == 0) {
		ridx = rxnode_ridx_2g;
		base = rxnode_base_2g;
	} else {
		ridx = rxnode_ridx_5g;
		base = rxnode_base_5g;
	}

	if (!(*(u8 *)(base + (u32)ridx * 12 + 8) & 1)) {
		wifi_cnt_inc(cb, 152);
		return;
	}

	/* find how many segments make the frame at ridx */
	scan = ridx;
	while (1) {
		wifi_cnt_inc(cb, 156);
		node = base + (u32)scan * 12;
		if (!(*(u8 *)(node + 8) & 1)) {
			if (++tries > wifi_retry_limit) {
				wifi_cnt_inc(cb, 184);
				goto process;
			}
			continue;
		}

		wifi_cnt_inc(cb, 160);
		flags = *(u8 *)(node + 8);
		total = flags >> 5;
		seg = (flags >> 2) & 7;
		next = (count + 1) & 0xFFFF;

		if (flags & 2) {
			/* last segment: submit only a complete frame */
			wifi_cnt_inc(cb, 168);
			if (total == next && seg == next) {
				wifi_cnt_inc(cb, 164);
				submit = 1;
			} else if (total == next) {
				wifi_cnt_inc(cb, 172);
			} else if (total == seg) {
				wifi_cnt_inc(cb, 176);
			} else {
				wifi_cnt_inc(cb, 172);
				wifi_cnt_inc(cb, 176);
			}
			count = next;
			goto process;
		}

		if (next >= total) {
			wifi_cnt_inc(cb, 180);
			goto process;
		}
		count = next;
		scan = ((scan + 1) != RXNODE_RING_SIZE) ? scan + 1 : 0;
		if (tries > wifi_retry_limit) {
			wifi_cnt_inc(cb, 184);
			goto process;
		}
	}

process:
	/* hand the frame to the host, or drop its segments */
	while (count != 0) {
		node = base + (u32)ridx * 12;
		if (submit) {
			if (rxnode_submit(node, band) != 0) {
				wifi_cnt_inc(cb, 136);
				for (retry = wifi_retry_limit; retry; retry--) {
					if (rxnode_submit(node, band) == 0) {
						wifi_cnt_inc(cb, 188);
						break;
					}
				}
			} else {
				wifi_cnt_inc(cb, 0xBC);
			}
		}

		ridx = ((ridx + 1) != RXNODE_RING_SIZE) ? ridx + 1 : 0;
		buf_id_free(0, band, *(u32 *)node);
		count--;
		*(u32 *)node = (u32)-1;
		*(u32 *)(node + 4) = 0;
		*(u8 *)(node + 8) = 0;
		wifi_cnt_inc(cb, 192);
	}

	if (band != 0)
		rxnode_ridx_5g = ridx;
	else
		rxnode_ridx_2g = ridx;
}

#endif /* WIFI_KITE */
