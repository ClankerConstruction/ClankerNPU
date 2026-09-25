/*
 * AN75XX NPU firmware - host adaptor rings
 *
 * The DMA rings between the NPU and the ARM host. The out rings carry
 * received frames to the host. The in rings carry frames the host wants
 * transmitted, and exist only where HAS_NPU_WIFI_TX does.
 */

#include "npu_internal.h"
#include "npu_wifi.h"


/* ================================================================
 * Host adaptor (DMA interface to host ARM)
 * ================================================================ */

#define HOSTADPT_TX_DMA_PTR   0x1EC0D180
#define HOSTADPT_RX_DMA_PTR   0x1EC0D190

static u32 hostadpt_tx_ring_base;
volatile u8 hostadpt_tx_ring_ready;
static u32 hostadpt_rx_ring_base;

#ifdef HAS_NPU_WIFI_TX
/* host -> NPU tx rings: 208-byte entries, bit0 of word0 is the own bit */
#define HOSTADPT_IN_ENTRY     208
u32 hostadpt_in_base[2];
u32 hostadpt_in_size[2];
u32 hostadpt_in_ridx[2];
#endif

int hostadpt_init(void)
{
#ifdef HAS_NPU_WIFI_TX
	while (REG32(HOSTADPT_IN_BASE_PTR(1)) == 0)
		;
	hostadpt_in_base[1] = (REG32(HOSTADPT_IN_BASE_PTR(1)) & 0x3FFFFFFF) |
			      NPU_ADDR_MASK;
	hostadpt_in_base[0] = (REG32(HOSTADPT_IN_BASE_PTR(0)) & 0x3FFFFFFF) |
			      NPU_ADDR_MASK;
	hostadpt_in_size[0] = REG32(HOSTADPT_IN_MAX_CNT(0));
	hostadpt_in_size[1] = REG32(HOSTADPT_IN_MAX_CNT(1));
#endif
	/* wait for host to configure RX DMA pointer */
	while (REG32(HOSTADPT_RX_DMA_PTR) == 0)
		;

	hostadpt_tx_ring_base = (REG32(HOSTADPT_TX_DMA_PTR) & 0x3FFFFFFF) | 0x40000000;
	hostadpt_rx_ring_base = (REG32(HOSTADPT_RX_DMA_PTR) & 0x3FFFFFFF) | 0x40000000;
	hostadpt_tx_ring_ready = 1;
	return 0;
}

#ifdef HAS_WIFI

#if defined(WIFI_EAGLE) || defined(HAS_NPU_WIFI_TX)
#define HOSTADPT_BUFFER_LEN  1792
#else
#define HOSTADPT_BUFFER_LEN  3500
#endif
#define HOSTADPT_RING_SIZE   512

static void host_ring_write_desc(u32 desc_addr, u32 buf_addr, u16 pkt_len,
				 u16 wcid, u8 amsdu, u8 fwd_type,
				 u16 orig_len, u8 is_last, u32 info)
{
	/* word 0 carries the ready bit: store it last */
	volatile u32 *d = (volatile u32 *)desc_addr;
	u32 dma_len;

	if (pkt_len <= HOSTADPT_BUFFER_LEN) {
		dma_len = pkt_len;
		if ((wifi_debug_flags & 4) && counter_base_tri)
			(*(u32 *)(counter_base_tri + 0x34))++;
	} else {
		dma_len = orig_len;
		if (dma_len > HOSTADPT_BUFFER_LEN)
			dma_len = HOSTADPT_BUFFER_LEN;
		if ((wifi_debug_flags & 4) && counter_base_tri)
			(*(u32 *)(counter_base_tri + 0x30))++;
	}

	bridge_dma_copy(3, buf_addr, d[3], dma_len);

	d[2] = info;
	d[1] = (wcid & 0xFFFF) | ((amsdu & 0x1F) << 16) |
	       ((fwd_type & 0x3F) << 26);
	d[0] = 1 | ((orig_len & 0x3FFF) << 1) | ((pkt_len & 0x3FFF) << 15) |
	       ((is_last & 1) << 29);

	if ((wifi_debug_flags & 4) && counter_base_tri)
		(*(u32 *)(counter_base_tri + 0x38))++;
}

int host_ring_submit(u32 buf_addr, u16 pkt_len, u32 band,
			    u16 wcid, u8 amsdu, u8 fwd_type,
			    u16 orig_len, u8 is_last, u32 info)
{
	u32 idx, check_idx, ring_base, desc_addr;

	if (wifi_debug_flags & 4) {
		u32 *ctr;
		if (band == 1)
			ctr = (u32 *)(counter_base_5g + 140);
		else if (band == 0)
			ctr = (u32 *)(counter_base_2g + 140);
		else
			ctr = (u32 *)(counter_base_tri + 140);
		(*ctr)++;
	}

	if (band == 0) {
		idx = REG32(HOSTADPT_RX_DMA_IDX(0));
		ring_base = hostadpt_tx_ring_base;

		check_idx = idx + 2;
		if (check_idx > 511)
			check_idx = idx - 510;

		if (*(u8 *)(ring_base + check_idx * 24) & 1) {
			if (wifi_debug_flags & 4)
				(*(u32 *)(counter_base_2g + 0xEC))++;
			if (idx != 0)
				REG32(HOSTADPT_RX_DMA_IDX(0)) = idx - 1;
			else
				REG32(HOSTADPT_RX_DMA_IDX(0)) = 511;
			REG32(HOSTADPT_RX_DMA_IDX(0)) = idx;
			return -1;
		}

		desc_addr = ring_base + idx * 24;
		host_ring_write_desc(desc_addr, buf_addr, pkt_len, wcid,
				     amsdu, fwd_type, orig_len, is_last, info);
		idx++;
		REG32(HOSTADPT_RX_DMA_IDX(0)) = (idx < HOSTADPT_RING_SIZE) ? idx : 0;
		return 0;

	} else if (band == 1) {
		idx = REG32(HOSTADPT_RX_DMA_IDX(1));
		ring_base = hostadpt_rx_ring_base;

		check_idx = idx + 2;
		if (check_idx > 511)
			check_idx = idx - 510;

		if (*(u8 *)(ring_base + check_idx * 24) & 1) {
			if (wifi_debug_flags & 4)
				(*(u32 *)(counter_base_5g + 0xEC))++;
			if (idx != 0)
				REG32(HOSTADPT_RX_DMA_IDX(1)) = idx - 1;
			else
				REG32(HOSTADPT_RX_DMA_IDX(1)) = 511;
			REG32(HOSTADPT_RX_DMA_IDX(1)) = idx;
			return -1;
		}

		desc_addr = ring_base + idx * 24;
		host_ring_write_desc(desc_addr, buf_addr, pkt_len, wcid,
				     amsdu, fwd_type, orig_len, is_last, info);
		idx++;
		REG32(HOSTADPT_RX_DMA_IDX(1)) = (idx < HOSTADPT_RING_SIZE) ? idx : 0;
		return 0;

	} else {
		npu_printf("Ring Index error!\n");
		return -1;
	}
}

#ifdef HAS_ASYNC_COPY
/* bridge_dma_copy on channel 3, split at the wait */
static inline void host_out_copy(u32 src, u32 dst, u32 len)
{
	REG32(DMA_COPY_SRC(3)) = src;
	REG32(DMA_COPY_DST(3)) = dst;
	REG32(DMA_COPY_CTRL(3)) = (len << 16) | 0x23;
}

static inline void host_out_wait(void)
{
	while (!(REG32(DMA_COPY_STATUS) & 8))
		;
	REG32(DMA_COPY_STATUS) = 8;
}

NPU_HOT u32 host_out_idx(void)
{
	return REG32(HOSTADPT_RX_DMA_IDX(0));
}

/* host_ring_submit for ring 0 in two steps, one copy in flight. -1:
 * full ring or counters on, prev untouched; use host_ring_submit. */
NPU_HOT int host_out_start(struct host_out *o, struct host_out *prev,
			   u32 idx, u32 buf_addr, u16 pkt_len, u16 wcid,
			   u8 amsdu, u8 fwd_type, u16 orig_len, u8 is_last,
			   u32 info)
{
	u32 ring = hostadpt_tx_ring_base;
	u32 check_idx = (idx + 2 > 511) ? idx - 510 : idx + 2;
	volatile u32 *d = (volatile u32 *)(ring + idx * 24);
	u32 dma_len = pkt_len;

	if ((wifi_debug_flags & 4) ||
	    (*(volatile u8 *)(ring + check_idx * 24) & 1))
		return -1;
	if (dma_len > HOSTADPT_BUFFER_LEN)
		dma_len = orig_len < HOSTADPT_BUFFER_LEN ? orig_len :
							   HOSTADPT_BUFFER_LEN;
	o->desc = (u32)d;
	o->dst = d[3];
	if (prev)
		host_out_finish(prev);
	host_out_copy(buf_addr, o->dst, dma_len);

	o->next = (idx + 1 < HOSTADPT_RING_SIZE) ? idx + 1 : 0;
	o->w2 = info;
	o->w1 = (wcid & 0xFFFF) | ((amsdu & 0x1F) << 16) |
		((fwd_type & 0x3F) << 26);
	o->w0 = 1 | ((orig_len & 0x3FFF) << 1) | ((pkt_len & 0x3FFF) << 15) |
		((is_last & 1) << 29);
	return 0;
}

/* word 0 carries the ready bit: store it last */
NPU_HOT void host_out_finish(struct host_out *o)
{
	volatile u32 *d = (volatile u32 *)o->desc;

	host_out_wait();
	d[2] = o->w2;
	d[1] = o->w1;
	d[0] = o->w0;
	REG32(HOSTADPT_RX_DMA_IDX(0)) = o->next;
}
#endif

#endif /* HAS_WIFI */
