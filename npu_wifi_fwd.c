/*
 * AN75XX NPU firmware - packet forwarding rings (kite)
 *
 * piNode and rxNode hand a frame from the core that received it to the
 * core that gives it to the host. A frame whose length the NPU changed
 * takes the 16-byte piNode path, one it did not takes the 12-byte
 * rxNode path.
 */

#include "npu_internal.h"
#include "npu_wifi.h"


/* ================================================================
 * Packet node access (piNode / rxNode descriptors)
 *
 * piNode: 16-byte descriptor for packet info
 * rxNode: 12-byte descriptor for RX ring state
 * ================================================================ */

static u32 pinode_base_2g;
static u32 pinode_base_5g;
static u32 rxnode_base_2g;
static u32 rxnode_base_5g;

static u32 get_pinode(u32 idx, u32 band)
{
	if (band != 0)
		return pinode_base_5g + idx * 16;
	return pinode_base_2g + idx * 16;
}

static u32 get_rxnode(u32 idx, u32 band)
{
	if (band != 0)
		return rxnode_base_5g + idx * 12;
	return rxnode_base_2g + idx * 12;
}

/* ================================================================
 * Packet forwarding engine
 *
 * Enqueues packets to piNode/rxNode rings for core-to-core handoff.
 * Per-band byte/packet statistics updated inline.
 * ================================================================ */

/* per-band ring write indices */
u16 rxnode_widx_2g;
u16 rxnode_widx_5g;
u16 pinode_widx_2g;
u16 pinode_widx_5g;

/* per-band ring read indices (drain side, core3) */
static u16 rxnode_ridx_2g;
static u16 rxnode_ridx_5g;
static u16 pinode_ridx_2g;
static u16 pinode_ridx_5g;
static u16 rxnode_retry_limit;

/* per-band per-queue stats: 16 queues x {u64 bytes, u64 pkts} */
u32 stats_bytes_2g[32];
u32 stats_pkts_2g[32];
u32 stats_bytes_5g[32];
u32 stats_pkts_5g[32];

static u32 fwd_mutex[2];

int pkt_forward(u32 buf_id, u32 pkt_len, s16 wcid, u8 amsdu,
		       u32 band, u8 fwd_type, u32 orig_len,
		       int classify_result, u8 tunnel)
{
	u32 node_base, widx, *node;
	u32 pkt_info_addr;
	u32 queue_id;

	if (orig_len != pkt_len) {
		/* different original length: use piNode path */
		hw_mutex_lock(fwd_mutex);

		if (band != 0) {
			widx = pinode_widx_5g;
			node_base = pinode_base_5g + widx * 16;
		} else {
			widx = pinode_widx_2g;
			node_base = pinode_base_2g + widx * 16;
		}

		node = (u32 *)node_base;
		if (node[2] & 1) {
			/* ring full */
			hw_mutex_unlock(fwd_mutex);
			return 1;
		}

		*(u8 *)(node_base + 12) = (classify_result == -2);
		*(u16 *)(node_base + 6) = (u16)pkt_len;
		*(u16 *)(node_base + 8) = (u16)orig_len;
		node[0] = buf_id;
		*(u16 *)(node_base + 4) = wcid;
		*(u8 *)(node_base + 11) = amsdu;
		*(u8 *)(node_base + 10) = fwd_type | 1;

		if (band != 0) {
			widx++;
			pinode_widx_5g = (widx != 512) ? widx : 0;
		} else {
			widx++;
			pinode_widx_2g = (widx != 256) ? widx : 0;
		}
		hw_mutex_unlock(fwd_mutex);
	} else {
		/* same length: use rxNode path */
		if (band != 0) {
			widx = rxnode_widx_5g;
			node_base = rxnode_base_5g + widx * 12;
		} else {
			widx = rxnode_widx_2g;
			node_base = rxnode_base_2g + widx * 12;
		}

		if (*(u8 *)(node_base + 8) & 1)
			return 1;

		*(u16 *)(node_base + 4) = (u16)pkt_len;
		*(u16 *)(node_base + 6) = (u16)orig_len;
		*(u32 *)node_base = buf_id;
		*(u8 *)(node_base + 8) = fwd_type | 1;

		widx++;
		if (band != 0)
			rxnode_widx_5g = (widx & 0xFFFF) != 128 ? widx : 0;
		else
			rxnode_widx_2g = (widx & 0xFFFF) != 128 ? widx : 0;
	}

	/* per-queue byte/packet stats */
	pkt_info_addr = (sram_buf_pad[0] & 0x3FFFFFFF) | 0x40000000;
	pkt_info_addr += buf_id << 12;
	queue_id = REG32(pkt_info_addr + 0x88) & 0x3F;
	if (queue_id & 0x30)
		queue_id = (u8)(queue_id - 16);

	if (queue_id <= 15 && tunnel == 0) {
		u32 adj_len = (orig_len - 98) & 0xFFFF;

		if (band == 1) {
			stats_bytes_5g[queue_id * 2] += adj_len;
			stats_pkts_5g[queue_id * 2]++;
		} else {
			stats_bytes_2g[queue_id * 2] += adj_len;
			stats_pkts_2g[queue_id * 2]++;
		}
	}

	return 0;
}

/* ================================================================
 * Host ring drain functions (kite only)
 *
 * pinode_drain: reads 16B piNode entries, submits to host ring
 * rxnode_drain: reads 12B rxNode entries, group/scatter logic
 * Both run on core3 in the drain loop.
 * ================================================================ */

#ifdef WIFI_KITE

#define PINODE_RING_SIZE_2G  256
#define PINODE_RING_SIZE_5G  512
#define RXNODE_RING_SIZE     128

void pinode_drain(u32 band)
{
	u16 ridx;
	u32 *entry;
	u32 ring_size;
	s32 buf_id;

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
		if (!(wifi_debug_flags & 4))
			return;
		goto drain_stat;
	}

	if (wifi_debug_flags & 4) {
		if (band == 1)
			(*(u32 *)(counter_base_5g + 36))++;
		else
			(*(u32 *)(counter_base_2g + 36))++;
	}

	buf_id = (s32)entry[0];
	if (buf_id >= 0) {
		if (*(u16 *)((u32)entry + 6) == 0) {
			if (wifi_debug_flags & 4) {
				if (band == 1)
					(*(u32 *)(counter_base_5g + 44))++;
				else
					(*(u32 *)(counter_base_2g + 44))++;
			}
			goto clear;
		}

		{
			u32 buf_addr = ((((u32)buf_id << 12) +
					wifi_buf_id_base) & 0x3FFFFFFF) |
				       0x80000000;
			int ret = host_ring_submit(
				buf_addr,
				*(u16 *)((u32)entry + 6),
				band,
				*(u16 *)((u32)entry + 4),
				*(u8 *)((u32)entry + 11),
				*(u8 *)((u32)entry + 10) >> 2,
				*(u16 *)((u32)entry + 8),
				(*(u8 *)((u32)entry + 10) >> 1) & 1,
				*(u8 *)((u32)entry + 12));

			if (ret != 0) {
				if (wifi_debug_flags & 4) {
					if (band == 1)
						(*(u32 *)(counter_base_5g + 108))++;
					else
						(*(u32 *)(counter_base_2g + 108))++;
				}
			} else {
				if (wifi_debug_flags & 4) {
					if (band == 1)
						(*(u32 *)(counter_base_5g + 48))++;
					else
						(*(u32 *)(counter_base_2g + 48))++;
				}
			}
		}
		buf_id_return(*(u16 *)entry);
	} else {
		if (wifi_debug_flags & 4) {
			if (band == 1)
				(*(u32 *)(counter_base_5g + 40))++;
			else
				(*(u32 *)(counter_base_2g + 40))++;
		}
	}

clear:
	entry[0] = (u32)-1;
	entry[1] = 0;
	*(u16 *)((u32)entry + 8) = 0;
	*(u8 *)((u32)entry + 11) = 0;
	*(u8 *)((u32)entry + 10) = 0;

	ridx = ((ridx + 1) != ring_size) ? ridx + 1 : 0;

	if (band == 0) {
		pinode_ridx_2g = ridx;
		if (!(wifi_debug_flags & 4))
			return;
		(*(u32 *)(counter_base_2g + 20))++;
		return;
	}
	pinode_ridx_5g = ridx;
	if (!(wifi_debug_flags & 4))
		return;
drain_stat:
	if (band == 1)
		(*(u32 *)(counter_base_5g + 20))++;
	else
		(*(u32 *)(counter_base_2g + 20))++;
}

void rxnode_drain(u32 band)
{
	u16 ridx;
	u32 base;
	u32 retry = 0;
	u32 group_count = 0;
	u32 scan_idx;
	u32 submit;
	u32 entry_addr;
	u8 flags;

	if (band == 0) {
		ridx = rxnode_ridx_2g;
		base = rxnode_base_2g;
	} else {
		ridx = rxnode_ridx_5g;
		base = rxnode_base_5g;
	}

	if (!(*(u8 *)(base + (u32)ridx * 12 + 8) & 1)) {
		if (wifi_debug_flags & 4) {
			if (band == 1)
				(*(u32 *)(counter_base_5g + 152))++;
			else
				(*(u32 *)(counter_base_2g + 152))++;
		}
		return;
	}

	scan_idx = ridx;

	while (1) {
		if (wifi_debug_flags & 4) {
			if (band == 1)
				(*(u32 *)(counter_base_5g + 156))++;
			else
				(*(u32 *)(counter_base_2g + 156))++;
		}

		entry_addr = base + scan_idx * 12;
		if (!(*(u8 *)(entry_addr + 8) & 1)) {
			retry++;
			if (retry > rxnode_retry_limit) {
				if (wifi_debug_flags & 4) {
					if (band == 1)
						(*(u32 *)(counter_base_5g + 184))++;
					else
						(*(u32 *)(counter_base_2g + 184))++;
				}
				goto finish;
			}
			continue;
		}

		if (wifi_debug_flags & 4) {
			if (band == 1)
				(*(u32 *)(counter_base_5g + 160))++;
			else
				(*(u32 *)(counter_base_2g + 160))++;
		}

		flags = *(u8 *)(entry_addr + 8);
		{
			u32 group_flag = flags & 2;
			u32 total_count = (u32)flags >> 5;
			u32 group_index = ((u32)flags >> 2) & 7;
			u32 expected = (group_count + 1) & 0xFFFF;

			if (group_flag) {
				if (wifi_debug_flags & 4) {
					if (band == 1)
						(*(u32 *)(counter_base_5g + 168))++;
					else
						(*(u32 *)(counter_base_2g + 168))++;
				}

				if (total_count == expected) {
					if (group_index != expected) {
						submit = 0;
						if (wifi_debug_flags & 4) {
							if (band == 1)
								(*(u32 *)(counter_base_5g + 172))++;
							else
								(*(u32 *)(counter_base_2g + 172))++;
						}
						group_count = expected;
					} else {
						if (wifi_debug_flags & 4) {
							if (band == 1)
								(*(u32 *)(counter_base_5g + 164))++;
							else
								(*(u32 *)(counter_base_2g + 164))++;
						}
						submit = 1;
						group_count = expected;
					}
				} else {
					if (total_count == group_index ||
					    !(wifi_debug_flags & 4)) {
						submit = 0;
						if (wifi_debug_flags & 4) {
							if (band == 1)
								(*(u32 *)(counter_base_5g + 176))++;
							else
								(*(u32 *)(counter_base_2g + 176))++;
						}
						group_count = expected;
					} else {
						if (wifi_debug_flags & 4) {
							if (band == 1)
								(*(u32 *)(counter_base_5g + 172))++;
							else
								(*(u32 *)(counter_base_2g + 172))++;
						}
						submit = 0;
						if (total_count != expected) {
							if (wifi_debug_flags & 4) {
								if (band == 1)
									(*(u32 *)(counter_base_5g + 176))++;
								else
									(*(u32 *)(counter_base_2g + 176))++;
							}
						}
						group_count = expected;
					}
				}
				goto process;
			}

			if (expected >= total_count)
				break;

			group_count = expected;
			scan_idx = ((scan_idx + 1) != RXNODE_RING_SIZE) ?
				   scan_idx + 1 : 0;

			if (retry > rxnode_retry_limit) {
				if (wifi_debug_flags & 4) {
					if (band == 1)
						(*(u32 *)(counter_base_5g + 184))++;
					else
						(*(u32 *)(counter_base_2g + 184))++;
				}
				goto finish;
			}
			continue;
		}
	}

	if (wifi_debug_flags & 4) {
		if (band == 1)
			(*(u32 *)(counter_base_5g + 180))++;
		else
			(*(u32 *)(counter_base_2g + 180))++;
	}

finish:
	submit = 0;
	if (group_count == 0)
		goto done;

process:
	{
		u32 proc_addr = base + ridx * 12;
		s32 buf_id = *(s32 *)proc_addr;

		while (1) {
			if (submit) {
				u32 buf_a = (((buf_id << 12) +
					     wifi_buf_id_base) &
					    0x3FFFFFFF) | 0x80000000;
				int ret = host_ring_submit(
					buf_a,
					*(u16 *)(proc_addr + 4),
					band, 0, 0,
					(u32)*(u8 *)(proc_addr + 8) >> 2,
					*(u16 *)(proc_addr + 6),
					(*(u8 *)(proc_addr + 8) & 2) != 0,
					0);

				if (ret != 0) {
					if (wifi_debug_flags & 4) {
						if (band == 1)
							(*(u32 *)(counter_base_5g + 136))++;
						else
							(*(u32 *)(counter_base_2g + 136))++;
					}

					u32 retries = rxnode_retry_limit;
					while (retries) {
						buf_id = *(s32 *)proc_addr;
						retries--;
						buf_a = (((buf_id << 12) +
							 wifi_buf_id_base) &
							0x3FFFFFFF) |
							0x80000000;
						ret = host_ring_submit(
							buf_a,
							*(u16 *)(proc_addr + 4),
							band, 0, 0,
							(u32)*(u8 *)(proc_addr + 8) >> 2,
							*(u16 *)(proc_addr + 6),
							(*(u8 *)(proc_addr + 8) & 2) != 0,
							0);
						if (ret == 0) {
							buf_id = *(s32 *)proc_addr;
							if (wifi_debug_flags & 4) {
								if (band == 1)
									(*(u32 *)(counter_base_5g + 188))++;
								else
									(*(u32 *)(counter_base_2g + 188))++;
							}
							goto drop_entry;
						}
					}
				} else {
					buf_id = *(s32 *)proc_addr;
					if (wifi_debug_flags & 4) {
						if (band == 1)
							(*(u32 *)(counter_base_5g + 0xBC))++;
						else
							(*(u32 *)(counter_base_2g + 0xBC))++;
					}
				}
			}

drop_entry:
			ridx = ((ridx + 1) != RXNODE_RING_SIZE) ?
			       ridx + 1 : 0;
			buf_id_return((u16)buf_id);
			group_count--;
			*(u32 *)proc_addr = (u32)-1;
			*(u32 *)(proc_addr + 4) = 0;
			*(u8 *)(proc_addr + 8) = 0;

			if (wifi_debug_flags & 4) {
				if (band == 1)
					(*(u32 *)(counter_base_5g + 192))++;
				else
					(*(u32 *)(counter_base_2g + 192))++;
			}

			if (group_count == 0)
				goto done;

			proc_addr = base + ridx * 12;
			buf_id = *(s32 *)proc_addr;
		}
	}

done:
	if (band != 0)
		rxnode_ridx_5g = ridx;
	else
		rxnode_ridx_2g = ridx;
}

#endif /* WIFI_KITE */
