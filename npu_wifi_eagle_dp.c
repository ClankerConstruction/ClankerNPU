/*
 * AN75XX NPU firmware - eagle datapath
 *
 * The loops the cores run once the host has handed over every ring.
 * Core 1 reads the rxdmad ring, core 2 fills the WiFi tx ring, core 3
 * works both host adaptor rings and the tx done ring, and core 4 keeps
 * the rx rings stocked. On AN7552 core 0 does the work of cores 3 and 4.
 */

#include "npu_internal.h"
#include "npu_wifi.h"


#ifdef WIFI_EAGLE

/* ================================================================
 * Eagle datapath
 *
 * WiFi -> host   core 1 reads the rxdmad ring, chains the segments of a
 *                frame and queues it; core 3 moves the queue into the
 *                host adaptor out ring.
 * host -> WiFi   core 3 drains the host adaptor in ring into a staging
 *                ring; core 2 copies the staged TXD into the WiFi tx
 *                ring.
 * buffers        core 4 refills the two rx rings from the rx id pool,
 *                core 3 recycles tx tokens off the tx done ring.
 * ================================================================ */

/* TDMA back-pressure toward the NPU. Only the AN7581 die wires the pause
 * CRs; elsewhere one register gates the whole path. */
void eagle_tdma_flow_ctrl(int on)
{
	if ((REG32(CHIP_ID_REG) >> 16) != 15) {
		if (on)
			REG32(TDMA_FC_CFG2) = 3;
		else
			npu_printf("%s() not support\n",
				   "npu_disable_tdma_flow_control");
		return;
	}
	REG32(TDMA_FC_CFG0) = on ? 0x80048004 : 0;
	REG32(AN7552_FC_REG) = on ? 0xEB00EA : 0x610060;
}

static void eagle_delay(u32 loops)
{
	volatile u32 i;

	for (i = 0; i < loops; i++)
		;
}

/* LAN -> WiFi. Once HWNAT binds a flow the PPE sends its frames through
 * the TDMA rx ring instead of the host. Descriptor words 4-7 carry the
 * WiFi info: band in bit 25 and wcid in bits 24:14 of word 4, bss in
 * bits 30:24 of word 6. */

/* publish a band's tx ring cpu index */
static void eagle_tx_ring_publish(u32 band)
{
	REG32(eagle_tx_ring_pcie_base[band] + 8) = eagle_tx_ring_cpu_idx[band];
}

/* one frame into the WiFi tx ring, cpu index not published.
 * Only the TXP words are written; TXD words 0-7 of
 * the fast path TXD space stay zero. */
static int eagle_tx_ring_fill(u32 buf, u16 len, u16 token, u32 info,
			      u8 *band_out)
{
	u32 w0 = REG32(info), w2 = REG32(info + 8);
	u32 band = (w0 >> 25) & 1;
	u32 retry = 1000, cpu, desc, txd, phys, i;

	while (1) {
		cpu = eagle_tx_ring_cpu_idx[band];
		desc = eagle_tx_ring_desc[band] + 16 * cpu;
		*band_out = (u8)band;
		if ((s32)REG32(desc + 4) < 0 || retry == 0 || eagle_stopping)
			break;
		dbg.lanwait++;
		if (retry == 1000 || retry == 1)
			npu_printf(band ? "fband1 cpuindex = %d, dmaindex = %d" :
					  "fband0 cpuindex = %d, dmaindex = %d",
				   cpu, REG32(eagle_tx_ring_pcie_base[band] + 0xC));
		eagle_delay(10000);
		retry--;
	}

	/* keep five slots between us and the chip */
	while ((s32)REG32(eagle_tx_ring_desc[band] +
			  (((cpu + 5) << 4) & 0x7FF0) + 4) >= 0 &&
	       eagle_stopping == 0)
		eagle_delay(100000);

	if (eagle_rxdmad_on_core2)
		eagle_delay(440);

	/* own zeroed TXD space, not the one host TXDs are copied into */
	txd = eagle_tx_buf_space_pg[band] + (cpu << 8);
	REG32(txd + 32) = ((u32)token << 16) | 0x80;
	REG32(txd + 36) = (((w0 >> 14) & 0x7FF) << 8) | ((w2 >> 24) & 0x7F) |
			  0x1000000;
	REG32(txd + 40) = buf;
	REG32(txd + 64) = len;

	phys = (txd & 0x3FFFFFFF) | 0x80000000;
	for (i = 5; ; ) {
		REG32(desc) = phys;
		REG32(desc + 8) = buf;
		REG32(desc + 4) = EAGLE_TX_DESC_CTRL;
		if ((s32)REG32(desc + 4) >= 0)
			break;
		if (--i == 0)
			return -1;
		eagle_delay(10000);
	}

	eagle_tx_ring_cpu_idx[band] = (cpu + 1) & EAGLE_TX_RING_MASK;
	dbg.lan[band]++;
	return 0;
}

/* drain up to budget frames of TDMA rx ring 'ring' into the WiFi tx
 * ring. Each consumed slot gets a fresh tx token. */
static int eagle_tdma_to_wifi(u32 ring, u32 budget)
{
	u32 base = tdma_rx_dscp_base[ring];
	u32 kick = TDMA_RX_BASE_PTR(ring) + 8;
	u32 ridx = tdma_rx_ridx[ring];
	u32 batch = 0, fail = 0, stop, d, w1, buf, tok;
	int pushed = 0;
	s32 ntok;
	u8 band = 0;

	if (budget > 127)
		budget = 128;

	do {
		d = base + TDMA_RX_DESC_SIZE * ridx;
		w1 = REG32(d + 4);
		if ((s32)w1 >= 0)
			break;

		buf = REG32(d + 8);
		stop = 0;
		ntok = tx_token_alloc();
		if (ntok == -1) {
			/* no spare buffer: drop the frame, rearm the slot */
			tdma_rx_alloc_fail++;
			dbg.lanfail++;
			fail++;
			REG32(d + 4) = 0x800;
			stop = 1;
		} else {
			tok = (buf - npu_tx_pkt_buf_addr) >> 11;
			REG32(d + 8) = ((((u32)ntok << 11) + npu_tx_pkt_buf_addr) &
					0x3FFFFFFF) | 0x80000000;
			REG32(d + 4) = 0x800;
			if (eagle_tx_ring_fill(buf, w1 & 0xFFFF, (u16)tok,
					       d + 16, &band) == -1) {
				tx_token_free((u16)tok);
				dbg.lanfail++;
				fail++;
				stop = 1;
			}
		}

		if (++batch == 8) {
			REG32(kick) = ridx;
			eagle_tx_ring_publish(band);
			pushed = 1;
			batch = 0;
			fail = 0;
		}
		ridx = (ridx > TDMA_RX_RING_DESCS - 2) ? 0 : ridx + 1;
		if (stop)
			break;
	} while (--budget);

	if (batch) {
		REG32(kick) = ridx ? ridx - 1 : TDMA_RX_RING_DESCS - 1;
		pushed = 1;
		if (fail != batch)
			eagle_tx_ring_publish(band);
	}
	tdma_rx_ridx[ring] = ridx;
	return pushed;
}

/* Where frames are supposed to appear. With the stats print bit set,
 * core 3 prints them every two seconds. Fields are in npu_wifi.h. */
struct eagle_dbg dbg;

#define EAGLE_DBG_CYCLES  (2u * 720u * 1000u * 1000u)	/* ~2s at 720MHz */

static u32 eagle_dma_idx(u32 band)
{
	if (eagle_tx_ring_pcie_base[band] == 0)
		return 0xFFFF;
	return REG32(eagle_tx_ring_pcie_base[band] + 0xC) & 0xFFFF;
}

void eagle_dbg_print(void)
{
	npu_printf("[NPU]tx in=%d/%d stg=%d/%d full=%d/%d psh=%d/%d\n",
		   dbg.in[0], dbg.in[1], dbg.stage[0], dbg.stage[1],
		   dbg.nostage[0], dbg.nostage[1], dbg.push[0], dbg.push[1]);
	npu_printf("[NPU]ring cpu=%d/%d dma=%d/%d done=%d rf=%d/%d\n",
		   eagle_tx_ring_cpu_idx[0], eagle_tx_ring_cpu_idx[1],
		   eagle_dma_idx(0), eagle_dma_idx(1), dbg.txdone,
		   dbg.refill[0], dbg.refill[1]);
	npu_printf("[NPU]rx rxd=%d q=%d fast=%d seg=%d drop=%d\n",
		   dbg.rxd, dbg.rxq, dbg.rxfast, dbg.rxseg, dbg.rxdrop);
	npu_printf("[NPU]rxo out=%d full=%d host=%d/%d state=%d/%d/%d\n",
		   dbg.rxout, dbg.rxoutfail,
		   REG32(HOSTADPT_RX_CPU_IDX(0)) & 0xFFFF,
		   REG32(HOSTADPT_RX_CPU_IDX(1)) & 0xFFFF,
		   eagle_rx_en, eagle_tx_en, eagle_txq_state);
	npu_printf("[NPU]lan push=%d/%d fail=%d wait=%d ridx=%d/%d\n",
		   dbg.lan[0], dbg.lan[1], dbg.lanfail, dbg.lanwait,
		   tdma_rx_ridx[0], tdma_rx_ridx[1]);
	npu_printf("[NPU]wire sw=%d hw=%d cfg=%x glb=%x\n",
		   tdma_tx_sw_idx[0], REG32(TDMA_TX_RING0_DMA_IDX) & 0xFFFF,
		   REG32(TDMA_TX_RING0_CFG), REG32(TDMA_GLB_CFG));
}

static void eagle_dbg_tick(void)
{
	static u32 last;
	u32 now;

	if (!(ndbg->print_mask & (1u << NDBG_STATS)))
		return;
	now = (u32)csr_read(mcycle);
	if ((u32)(now - last) < EAGLE_DBG_CYCLES)
		return;
	last = now;
	eagle_dbg_print();
}

static NPU_INLINE u32 eagle_buf_uncached(u32 buf_id)
{
	return ((eagle_pkt_buf_addr & 0x3FFFFFFF) | 0x40000000) +
	       (buf_id << EAGLE_PKT_BUF_SHIFT);
}

static u32 eagle_buf_phys(u32 buf_id)
{
	return ((((buf_id << EAGLE_PKT_BUF_SHIFT) + eagle_pkt_buf_addr) &
		 0x3FFFFFFF) | 0x80000000) + EAGLE_PKT_HEADROOM;
}

/* ---- packet queues ---- */

/* Queue one packet for the host adaptor. A frame that arrived whole goes
 * to the per-band queue; one spread over several rx buffers goes to the
 * multi-segment queue, which keeps the segments together. */
#ifdef HAS_HOT_TEXT
/* a whole frame into the band 1 queue, the one only the rxdmad hart
 * fills. Mutex 10 still guards dbg.rxq, which core 0 also counts. */
static NPU_INLINE int eagle_txq1_put(u32 buf_id, u16 len)
{
	u32 idx, e;
	int ret = 1;

	hw_mutex_take(10);
	npu_barrier();
	idx = eagle_txq_widx[1];
	e = eagle_txq_base[1] + EAGLE_Q_ENTRY * idx;
	if ((*(volatile u8 *)(e + 10) & 1) == 0) {
		*(volatile u32 *)e = buf_id;
		*(volatile u16 *)(e + 4) = 0;
		*(volatile u16 *)(e + 6) = len;
		*(volatile u16 *)(e + 8) = len;
		*(volatile u8 *)(e + 11) = 0;
		*(volatile u8 *)(e + 10) = 2 | 1;
		idx++;
		eagle_txq_widx[1] = (idx == EAGLE_TXQ_ENTRIES) ? 0 : idx;
		ret = 0;
		dbg.rxq++;
	}
	npu_barrier();
	hw_mutex_give(10);
	return ret;
}
#endif

static int eagle_pkt_enqueue(u32 buf_id, u16 seg_len, u16 wcid, u8 info,
			     u32 dst, u8 flags, u16 pkt_len)
{
	u32 band = (dst == 0) ? 0 : 1;
	u32 idx, e;
	int ret = 1;

	hw_mutex_lock(eagle_txq_mutex);

	if (pkt_len == seg_len) {
		idx = eagle_txq_widx[band];
		e = eagle_txq_base[band] + EAGLE_Q_ENTRY * idx;
		if (*(volatile u8 *)(e + 10) & 1)
			goto out;
		*(volatile u32 *)e = buf_id;
		*(volatile u16 *)(e + 4) = wcid;
		*(volatile u16 *)(e + 6) = seg_len;
		*(volatile u16 *)(e + 8) = pkt_len;
		*(volatile u8 *)(e + 11) = info;
		*(volatile u8 *)(e + 10) = flags | 1;
		idx++;
		eagle_txq_widx[band] = (idx == EAGLE_TXQ_ENTRIES) ? 0 : idx;
	} else {
		idx = eagle_mseg_widx[band];
		e = eagle_mseg_base[band] + EAGLE_Q_ENTRY * idx;
		if (*(volatile u8 *)(e + 8) & 1)
			goto out;
		*(volatile u32 *)e = buf_id;
		*(volatile u16 *)(e + 4) = seg_len;
		*(volatile u16 *)(e + 6) = pkt_len;
		*(volatile u8 *)(e + 8) = flags | 1;
		idx++;
		eagle_mseg_widx[band] = (idx == EAGLE_MSEG_ENTRIES) ? 0 : idx;
	}
	ret = 0;
	dbg.rxq++;
out:
	hw_mutex_unlock(eagle_txq_mutex);
	return ret;
}

void eagle_queue_init(u32 band)
{
	u32 i, e;

	if (band == 2) {
		npu_printf("%s() special handling: band is changed to BAND1\n",
			   "npu_eagle_rx_init");
		band = 1;
	}

	eagle_txq_widx[band] = 0;
	eagle_txq_ridx[band] = 0;
	eagle_mseg_widx[band] = 0;
	eagle_mseg_ridx[band] = 0;

	if (band == 0) {
		eagle_txq_base[0] = sram_buf_alloc(2);
		eagle_mseg_base[0] = sram_buf_alloc(14);
		eagle_rx_ring_desc_base[0] = eagle_ring_desc_base(1);
	} else {
		eagle_txq_base[1] = sram_buf_alloc(3);
		eagle_mseg_base[1] = sram_buf_alloc(15);
		eagle_rx_ring_desc_base[1] = eagle_ring_desc_base(2);
	}

	for (i = 0; i < EAGLE_TXQ_ENTRIES; i++) {
		e = eagle_txq_base[band] + EAGLE_Q_ENTRY * i;
		*(volatile u32 *)e = 0xFFFFFFFF;
		*(volatile u32 *)(e + 4) = 0;
		*(volatile u16 *)(e + 8) = 0;
		*(volatile u8 *)(e + 10) = 0;
		*(volatile u8 *)(e + 11) = 0;
	}
	for (i = 0; i < EAGLE_MSEG_ENTRIES; i++) {
		e = eagle_mseg_base[band] + EAGLE_Q_ENTRY * i;
		*(volatile u32 *)e = 0xFFFFFFFF;
		*(volatile u32 *)(e + 4) = 0;
		*(volatile u8 *)(e + 8) = 0;
	}
}

/* PPE_WIFI_BUF_ID (PLIC 95): the PPE returns WiFi rx buffers it did
 * not forward. Bit 30 set: just free the buffer. Clear: the flow is not
 * bound, so the frame goes to the host. */
void ppe_wifi_bufid_isr(int src)
{
	u32 n = REG32(PPE_WIFI_BUF_CNT) & 0xFFFF, i = 0, v, info, hdr;
	u16 id;

	(void)src;
	if (n == 0)
		return;
	v = REG32(PPE_WIFI_BUF_ID);
	while ((s32)v < 0) {
		id = v & 0xFFFF;
		if (v & 0x40000000) {
			buf_id_return(id);
		} else {
			info = REG32(PPE_WIFI_BUF_INFO);
			hdr = REG32(eagle_buf_uncached(id));
			if (eagle_pkt_enqueue(id, (hdr >> 3) & 0x3FFF,
					      info & PPE_WIFI_BUF_WCID,
					      (info >> 16) & 31,
					      0, 2, (hdr >> 3) & 0x3FFF) != 0)
				buf_id_return(id);
		}
		REG32(PPE_WIFI_BUF_ID) = 0x80000000;
		if (++i == n)
			break;
		v = REG32(PPE_WIFI_BUF_ID);
	}
}

/* ---- WiFi -> host ---- */

#if defined(AN7552)
/* Drains share hart 0 with the PPE ISR, which also frees under mutex
 * 13; re-acquiring it stalls the bus. Mask PLIC 95 around
 * the free. */
static void eagle_drain_free(u16 buf_id)
{
	plic_disable(95);
	buf_id_return(buf_id);
	plic_enable(95);
}
#else
#define eagle_drain_free(id) buf_id_return(id)
#endif

/* Hand one queued packet to the host adaptor out ring and give the rx
 * buffer id back either way: the host copy already took the data. */
static void eagle_txq_drain(u32 band)
{
	u32 e = eagle_txq_base[band] + EAGLE_Q_ENTRY * eagle_txq_ridx[band];
	u32 buf_id, idx;
	u8 flags;
	u16 seg_len;

	if ((*(volatile u8 *)(e + 10) & 1) == 0)
		return;

	buf_id = *(volatile u32 *)e;
	seg_len = *(volatile u16 *)(e + 6);
	flags = *(volatile u8 *)(e + 10);

	if ((s32)buf_id >= 0 && seg_len != 0) {
		dbg.rxout++;
		if (host_ring_submit(eagle_buf_phys(buf_id), seg_len, 0,
				 *(volatile u16 *)(e + 4),
				 *(volatile u8 *)(e + 11),
				 flags >> 2,
				 *(volatile u16 *)(e + 8),
				 (flags >> 1) & 1,
				 *(volatile u32 *)eagle_buf_uncached(buf_id)) != 0)
			dbg.rxoutfail++;
		eagle_drain_free((u16)buf_id);
	}

	*(volatile u32 *)e = 0xFFFFFFFF;
	*(volatile u32 *)(e + 4) = 0;
	*(volatile u16 *)(e + 8) = 0;
	*(volatile u8 *)(e + 11) = 0;
	*(volatile u8 *)(e + 10) = 0;

	idx = eagle_txq_ridx[band] + 1;
	eagle_txq_ridx[band] = (idx == EAGLE_TXQ_ENTRIES) ? 0 : idx;
}

/* Drain one frame's worth of segments out of the multi-segment queue.
 * The segments of a frame are contiguous; the last one carries bit 1 and
 * every one of them carries the count in bits 7:5. */
static void eagle_mseg_drain(u32 band)
{
	u32 idx = eagle_mseg_ridx[band];
	u32 base = eagle_mseg_base[band];
	u32 e = base + EAGLE_Q_ENTRY * idx;
	u32 waited = 0, segs = 0, count, this_idx, buf_id;
	u8 flags;
	int ok;

	if ((*(volatile u8 *)(e + 8) & 1) == 0)
		return;

	/* collect a whole frame first, the host adaptor needs it in order */
	while (1) {
		e = base + EAGLE_Q_ENTRY * idx;
		if ((*(volatile u8 *)(e + 8) & 1) == 0) {
			if (++waited > eagle_mseg_retry)
				return;
			eagle_delay(10000);
			continue;
		}
		flags = *(volatile u8 *)(e + 8);
		count = flags >> 5;
		this_idx = (flags >> 2) & 7;
		segs++;
		if (flags & 2) {
			ok = (count == segs && this_idx == segs - 1);
			break;
		}
		if (segs >= count) {
			ok = 0;
			break;
		}
		idx = (idx + 1 == EAGLE_MSEG_ENTRIES) ? 0 : idx + 1;
		if (waited > eagle_mseg_retry)
			return;
	}

	idx = eagle_mseg_ridx[band];
	while (segs != 0) {
		e = base + EAGLE_Q_ENTRY * idx;
		buf_id = *(volatile u32 *)e;
		if (ok) {
			dbg.rxout++;
			host_ring_submit(eagle_buf_phys(buf_id),
					 *(volatile u16 *)(e + 4), 0, 0, 0,
					 *(volatile u8 *)(e + 8) >> 2,
					 *(volatile u16 *)(e + 6),
					 (*(volatile u8 *)(e + 8) & 2) != 0,
					 *(volatile u32 *)eagle_buf_uncached(buf_id));
		}
		eagle_drain_free((u16)buf_id);
		*(volatile u32 *)e = 0xFFFFFFFF;
		*(volatile u32 *)(e + 4) = 0;
		*(volatile u8 *)(e + 8) = 0;
		idx = (idx + 1 == EAGLE_MSEG_ENTRIES) ? 0 : idx + 1;
		segs--;
	}
	eagle_mseg_ridx[band] = idx;
}

/* One rxdmad descriptor. Returns 1 while the ring is empty. */
static int eagle_rxdmad_handle(u8 *chaining)
{
	u32 d = eagle_ind_cmd_desc_base + 16 * eagle_rxdmad_ridx;
	u32 dw1, dw2, info, buf, next, err = 0;
	u32 head, seg_len, i, count;
	s16 buf_id;

	if (REG32(d + 12) >> 28 != eagle_rxdmad_gen)
		return 1;
	dbg.rxd++;
	if (eagle_rxdmad_on_core2)
		eagle_delay(280);

	if (eagle_rxdmad_ridx == EAGLE_RX_RING_MAX_IDX)
		eagle_rxdmad_gen = (eagle_rxdmad_gen + 1) & 0xF;

	dw1 = REG32(d + 4);
	dw2 = REG32(d + 8);
	next = eagle_rxdmad_ridx + 1;
	if (next >= 1536)
		next = 0;

	buf_id = (s16)(dw2 >> 16);
	buf = eagle_buf_uncached((u16)buf_id);

	info = (((dw1 >> 16) & 0x3FFF) << 3) | (((dw1 >> 30) & 1) ^ 1);
	if ((dw1 & 0x1800) == 0x800)
		info |= 2 | ((dw1 & 0x7F) << 17);
	if (dw1 & 0x2000) {
		info |= 0x2000000;
		err = 1;
	}
	if ((dw2 & 0xF000) == 0x1000)
		err |= 4;
	else if ((dw2 & 0xF000) == 0x2000)
		err |= 2;
	info = (info & 0x0FFFFFFF) | ((dw2 >> 12) << 28);

	/* the sampled frames were all 128-byte null data. Take the next few
	 * that actually carry something instead. */
	if (NDBG_PRINTING(NDBG_WIFI) && ((dw1 >> 16) & 0x3FFF) > 200 && dbg.rxbig < 4) {
		dbg.rxbig++;
		npu_printf("[NPU]rxdsc n=%d dw1=%x dw2=%x info=%x sdl=%d dst=%d\n",
			   dbg.rxd, dw1, dw2, info, (dw1 >> 16) & 0x3FFF,
			   (dw1 >> 11) & 3);
		npu_hexdump("rxpkt", buf + EAGLE_PKT_HEADROOM, 208);
	}

	if ((info & 1) == 0 && *chaining == 0) {
		/* a whole frame in one buffer */
		seg_len = (info >> 3) & 0x3FFF;

		if (err & 3) {
			u32 rxd = buf + EAGLE_PKT_HEADROOM;
			u32 wcid = REG32(rxd + 4) & 0xFFFFF;
			u32 tid = (REG32(rxd + 16) >> 3) & 0xF;

			if (eagle_icv_err_table != 0 &&
			    (REG32(eagle_icv_err_table + 4 * wcid) &
			     (1u << tid)) != 0) {
				err = 0;
				dw2 &= ~0x80u;
			}
		}
		REG32(buf) = info;

		if (err == 0 && wifi_force_to_cpu == 0 && (dw2 & 0x80) == 0) {
			/* dst_sel 1: the chip reordered the frame and gives its
			 * ethernet header offset; send it to the wired side. The
			 * PPE returns unbound flows via ppe_wifi_bufid_isr. */
			if (info & 2) {
				u32 off = (info >> 16) & 0xFE;

				dbg.rxfast++;
				if (tdma_tx_submit((u16)buf_id,
						   (seg_len - off) & 0xFFFF,
						   buf + EAGLE_PKT_HEADROOM + off,
						   0) != 0) {
					dbg.rxdrop++;
					buf_id_return((u16)buf_id);
				}
			} else if (eagle_pkt_enqueue((u16)buf_id, seg_len, 0, 0,
						     1, 2, seg_len) != 0) {
				dbg.rxdrop++;
				buf_id_return((u16)buf_id);
				npu_printf("enq slow path faill\n");
			}
		} else if (eagle_pkt_enqueue((u16)buf_id, seg_len, 0, 0, 1, 2,
					     seg_len) != 0) {
			dbg.rxdrop++;
			buf_id_return((u16)buf_id);
		}
		goto done;
	}

	if ((info & 1) == 0 && eagle_rxdmad_abort != 0) {
		buf_id_return((u16)buf_id);
		*chaining = 0;
		eagle_rxdmad_abort = 0;
		goto done;
	}

	if (*chaining == 0) {
		eagle_rxdmad_abort = 0;
		eagle_rxdmad_segs = 0;
		eagle_rxdmad_seglen = 0;
		*chaining = 1;
		for (i = 0; i < EAGLE_MSEG_MAX; i++) {
			eagle_seg_bufid[i] = 0;
			eagle_seg_len[i] = 0;
		}
		REG32(buf) = info;
	}
	if (eagle_rxdmad_abort != 0) {
		buf_id_return((u16)buf_id);
		goto done;
	}
#if defined(AN7552)
	/* give core 0 1 ms to refill its descriptor */
	if (eagle_sync[buf_id] == 0)
		delay_ms(1);
#endif

	seg_len = (info >> 3) & 0x3FFF;
	dbg.rxseg++;
	eagle_seg_len[eagle_rxdmad_segs] = (u16)seg_len;
	eagle_seg_bufid[eagle_rxdmad_segs] = (u16)buf_id;
	eagle_rxdmad_seglen += seg_len;
	eagle_rxdmad_segs++;

	if (info & 1) {
		if (eagle_rxdmad_segs > EAGLE_MSEG_MAX - 1) {
			npu_printf("[NPU] too many big pkt, abort \n");
			eagle_rxdmad_abort = 1;
			for (i = 0; i < EAGLE_MSEG_MAX; i++)
				buf_id_return((u16)eagle_seg_bufid[i]);
		}
		goto done;
	}

	count = eagle_rxdmad_segs;
	if (count > EAGLE_MSEG_MAX)
		count = EAGLE_MSEG_MAX;
	head = eagle_buf_uncached(eagle_seg_bufid[0]);
	REG32(head) = (REG32(head) & 0xFFFE0007) | (8 * eagle_rxdmad_seglen);

	for (i = 0; i < count; i++) {
		u8 flags = (u8)(((i == count - 1) << 1) | (count << 5) |
				(i << 2));

		if (eagle_pkt_enqueue(eagle_seg_bufid[i],
				      eagle_seg_len[i], 0, 0, 1, flags,
				      (u16)eagle_rxdmad_seglen) != 0)
			buf_id_return((u16)eagle_seg_bufid[i]);
	}
	*chaining = 0;
done:
	eagle_rxdmad_ridx = next;
	/* hand the ring back to the chip every 128 descriptors */
	if (++eagle_rxdmad_kick < 0) {
		u32 cidx = next ? next - 1 : EAGLE_RX_RING_MAX_IDX;

		if (eagle_emi_cidx_valid == 1)
			REG32(eagle_emi_cidx) = cidx;
		else
			REG32(eagle_ind_cmd_pcie_base + 8) = cidx;
		eagle_rxdmad_kick = 0;
	}
	return 0;
}

/* ---- host -> WiFi ---- */

#ifdef HAS_NPU_WIFI_TX
/* Copy one host tx frame into an NPU tx buffer and stage its TXD. */
static int eagle_tx_stage(u32 band, u32 *in)
{
	u32 idx = eagle_stage_widx[band];
	u32 e = eagle_stage_base[band] + EAGLE_STAGE_ENTRY * idx;
	s32 token;
	u16 len;
	u32 buf;

	if (*(volatile u8 *)(e + 12) == 1)
		return -1;

	len = (u16)((in[0] << 1) >> 19);
	if (len > EAGLE_TX_BUF_BYTES)
		len = EAGLE_TX_BUF_BYTES;
	*(volatile u16 *)(e + 10) = len;

	token = tx_token_alloc();
	*(volatile u16 *)(e + 8) = (u16)token;
	if (token == -1)
		return -1;

	if (len != 0) {
		buf = ((((u32)token << EAGLE_PKT_BUF_SHIFT) +
			npu_tx_pkt_buf_addr) & 0x3FFFFFFF) | 0x80000000;
		*(volatile u32 *)(e + 4) = buf;
		bridge_dma_copy(0, in[1], buf, len);
		bridge_dma_copy(0, (((u32)in + 16) & 0x3FFFFFFF) | 0x80000000,
				*(volatile u32 *)e, EAGLE_TXD_BYTES);
	}
	*(volatile u8 *)(e + 12) = 1;

	idx++;
	eagle_stage_widx[band] = (idx == EAGLE_STAGE_ENTRIES) ? 0 : idx;
	return 0;
}

/* Take everything the host put in the in ring. */
static int eagle_hostadpt_drain(u32 band)
{
	u32 budget = EAGLE_HOSTADPT_BUDGET;
	u32 idx = hostadpt_in_ridx[band];
	u32 e = hostadpt_in_base[band] + HOSTADPT_IN_ENTRY * idx;
	int moved = 0;

	if (hostadpt_in_size[band] == 0)
		return 0;

	while (*(volatile u32 *)e & 1) {
		if (NDBG_PRINTING(NDBG_WIFI) && eagle_in_first[band] == 0) {
			eagle_in_first[band] = 1;
			npu_printf("[NPU]in%d base=%x size=%d w0=%x pkt=%x skb=%x\n",
				   band, hostadpt_in_base[band],
				   hostadpt_in_size[band], ((u32 *)e)[0],
				   ((u32 *)e)[1], ((u32 *)e)[2]);
			npu_hexdump("txd", e + 16, EAGLE_TXD_BYTES);
			npu_hexdump("pkt",
				    (((u32 *)e)[1] & 0x3FFFFFFF) | NPU_ADDR_MASK,
				    32);
		}
		dbg.in[band]++;
		if (eagle_tx_stage(band, (u32 *)e) < 0) {
			dbg.nostage[band]++;
			return moved;
		}
		dbg.stage[band]++;
		idx++;
		if (idx == hostadpt_in_size[band])
			idx = 0;
		hostadpt_in_ridx[band] = idx;
		REG32(HOSTADPT_IN_CPU_IDX(band)) = idx;
		e = hostadpt_in_base[band] + HOSTADPT_IN_ENTRY * idx;
		moved = 1;
		if (--budget == 0)
			return moved;
	}
	return moved;
}

#endif /* HAS_NPU_WIFI_TX */

/* Move one staged frame into the WiFi tx ring. The TXD the host built
 * goes into the ring's own 256-byte slot; bit 27 of DW7 picks which of
 * the two token layouts that TXD wants. */
static int eagle_tx_ring_push(u32 band)
{
	u32 idx = eagle_stage_ridx[band];
	u32 e = eagle_stage_base[band] + EAGLE_STAGE_ENTRY * idx;
	u32 cpu, desc, txd, slot, next, wait, buf;
	int pushed = 0;

	if (*(volatile u8 *)(e + 12) != 1)
		return 0;

	if ((s16)*(volatile u16 *)(e + 8) >= 0 &&
	    *(volatile u16 *)(e + 10) != 0) {
		cpu = eagle_tx_ring_cpu_idx[band];
		desc = eagle_tx_ring_desc[band] + 16 * cpu;
		txd = (cpu << 8) + eagle_txd_space[band];

		if (NDBG_PRINTING(NDBG_WIFI) && eagle_tx_first_push[band] == 0) {
			eagle_tx_first_push[band] = 1;
			npu_printf("[NPU]tx%d stage=%x/%d tok=%x len=%d ring=%x cpu=%d dw1=%x txd=%x pcie=%x\n",
				   band, eagle_stage_base[band], idx,
				   *(volatile u16 *)(e + 8),
				   *(volatile u16 *)(e + 10),
				   eagle_tx_ring_desc[band], cpu,
				   REG32(desc + 4), txd,
				   eagle_tx_ring_pcie_base[band]);
		}

		for (wait = 1000; wait != 0 && eagle_stopping == 0; wait--) {
			if ((s32)REG32(desc + 4) < 0)
				break;
			if (wait == 1000 || wait == 1)
				npu_printf(band ? "sband1 cpuindex = %d, dmaindex = %d" :
						  "sband0 cpuindex = %d, dmaindex = %d", cpu,
					   REG32(eagle_tx_ring_pcie_base[band] + 0xC));
		}

		next = (cpu + 1) & EAGLE_TX_RING_MASK;
		while ((s32)REG32(eagle_tx_ring_desc[band] + 16 * next + 4) >= 0 &&
		       eagle_stopping == 0)
			eagle_delay(10000);

		slot = (txd & 0x3FFFFFFF) | 0x80000000;
		bridge_dma_copy(1, (*(volatile u32 *)e & 0x3FFFFFFF) | 0x80000000,
				slot, EAGLE_TXD_BYTES);

		buf = *(volatile u32 *)(e + 4);
		if (REG32(txd + 28) & (1u << 27)) {
			REG32(txd + 32) = ((buf - npu_tx_pkt_buf_addr) >>
					   EAGLE_PKT_BUF_SHIFT) | 0x8000;
			REG32(txd + 40) = buf;
		} else {
			REG32(txd + 40) = buf;
			*(volatile u16 *)(txd + 34) =
				(u16)((buf - npu_tx_pkt_buf_addr) >>
				      EAGLE_PKT_BUF_SHIFT);
			*(volatile u16 *)(txd + 64) = *(volatile u16 *)(e + 10);
		}

		for (wait = 5; wait != 0; wait--) {
			REG32(desc) = slot;
			REG32(desc + 4) = EAGLE_TX_DESC_CTRL;
			if ((s32)REG32(desc + 4) >= 0)
				break;
			eagle_delay(10000);
		}
		REG32(desc + 8) = buf;
		REG32(desc + 12) = 0;

		eagle_tx_ring_cpu_idx[band] = (u16)next;
		REG32(eagle_tx_ring_pcie_base[band] + 8) = next;
		dbg.push[band]++;
		pushed = 1;

		if (NDBG_PRINTING(NDBG_WIFI) && eagle_tx_first_push[band] == 1) {
			eagle_tx_first_push[band] = 2;
			npu_printf("[NPU]tx%d sent d0=%x d1=%x d2=%x cidx=%d dma=%d\n",
				   band, REG32(desc), REG32(desc + 4),
				   REG32(desc + 8), next,
				   REG32(eagle_tx_ring_pcie_base[band] + 0xC));
		}
	}

	*(volatile u32 *)(e + 4) = 0;
	*(volatile u32 *)(e + 8) = 0xFFFF;
	*(volatile u8 *)(e + 12) = 0;

	idx++;
	eagle_stage_ridx[band] = (idx == EAGLE_STAGE_ENTRIES) ? 0 : idx;
	return pushed;
}

/* ---- tx done ring ---- */

/* The WiFi chip reports finished frames here. A report frees the tokens
 * it lists; anything else is a stray buffer that only needs recycling. */
static int eagle_txdone_poll(void)
{
	u32 d, dw1, buf, hdr, i, n, cnt;
	u16 *ids = (u16 *)eagle_txdone_id_base;
	u32 idx = eagle_txdone_ridx;
	s32 buf_id;
	int any = 0;

	if (ids == NULL || eagle_rx_txdone_desc_base == 0)
		return 0;

	while (1) {
		d = eagle_rx_txdone_desc_base + 16 * idx;
		dw1 = REG32(d + 4);
		if ((s32)dw1 >= 0)
			break;

		buf = eagle_buf_uncached(ids[idx]);
		hdr = REG32(buf + EAGLE_PKT_HEADROOM);

		if ((hdr >> 27) == 6 || (hdr >> 27) == 24) {
			/* token report: 15-bit ids, 0x7FFF is the terminator */
			u32 left = (hdr & 0xFFFF);

			left = (left > 12) ? left - 12 : 0;
			cnt = (hdr >> 16) & 0xFF;
			n = 0;
			for (i = 0; n < cnt && left >= 4; i++, left -= 4) {
				u32 w = REG32(buf + EAGLE_PKT_HEADROOM + 12 + 4 * i);
				u32 lo = w & 0x7FFF;
				u32 hi = (w >> 15) & 0x7FFF;

				if ((s32)w < 0 || (w & 0x40000000))
					continue;
				if (lo != 0x7FFF) {
					n++;
					if (lo <= 0x33FF)
						tx_token_free((u16)lo);
				}
				if (hi != 0x7FFF) {
					n++;
					if (hi <= 0x33FF)
						tx_token_free((u16)hi);
				}
			}
		} else {
			/* any other event belongs to the host: pass the
			 * buffer up and give the slot a fresh one */
			buf_id = buf_id_alloc_ring();
			if (buf_id == -1) {
				npu_printf("txdone alloc buffid fail\n");
			} else {
				u16 old = ids[idx];
				u16 len = (dw1 >> 16) & 0x3FFF;

				ids[idx] = (u16)buf_id;
				REG32(eagle_buf_uncached(old)) =
					(8 * len) | 0x8000000;
				if (eagle_pkt_enqueue(old, len, 0, 0, 0, 2,
						      len) != 0)
					buf_id_return(old);
			}
		}

		REG32(d) = eagle_buf_phys(ids[idx]);
		REG32(d + 4) = EAGLE_RX_DESC_CTRL;

		dbg.txdone++;
		idx = (idx + 1 < eagle_txdone_ring_cnt) ? idx + 1 : 0;
		eagle_txdone_ridx = idx;
		if (++eagle_txdone_kick > 15) {
			REG32(eagle_txdone_pcie_base + 8) =
				(idx == 0) ? eagle_txdone_ring_cnt - 1 : idx - 1;
			eagle_txdone_kick = 0;
		}
		any = 1;
	}
	return any;
}

/* ---- rx ring refill ---- */

/* Put a fresh buffer under one rx descriptor. */
static int eagle_rx_ring_refill(u32 band, u32 desc, u32 *idx)
{
	s32 buf_id;
	u32 i = *idx;

#if defined(AN7552)
	/* the id leaving the descriptor is now safe */
	eagle_sync[(s16)(REG32(desc + 8) >> 16)] = 1;
#endif
	buf_id = buf_id_alloc_ring();
	if (buf_id == -1)
		return 1;
#if defined(AN7552)
	eagle_sync[buf_id] = 0;
#endif

	dbg.refill[band]++;
	eagle_rx_ring_bufid[band][i] = (u16)buf_id;
	REG32(desc) = eagle_buf_phys((u32)buf_id);
	REG32(desc + 8) = (u32)buf_id << 16;
	REG32(desc + 4) = EAGLE_RX_DESC_CTRL;

	*idx = (i + 1 < eagle_rx_ring_size[band]) ? i + 1 : 0;
	/* publish the refilled slots every 128 buffers */
	if (++eagle_rx_ring_kick[band] < 0) {
		REG32(eagle_rx_ring_pcie_base[band] + 8) = i;
		eagle_rx_ring_kick[band] = 0;
	}
	return 0;
}

static void eagle_rx_ring_sweep(u32 band)
{
	u32 idx = eagle_rx_ring_ridx[band];
	u32 base = eagle_rx_ring_desc_base[band];
	u32 budget = EAGLE_REFILL_BUDGET;

	while (budget--) {
		u32 desc = base + 16 * idx;

		if ((s32)REG32(desc + 4) >= 0)
			break;
		if (eagle_rx_ring_refill(band, desc, &idx) != 0)
			break;
	}
	eagle_rx_ring_ridx[band] = idx;
}

#ifdef HAS_HOT_TEXT
/* The common descriptor: a whole frame, dst_sel 1, no error, nothing
 * sending it to the host, WiFi print off. 0 done, 1 ring empty, 2 not
 * this kind: eagle_rxdmad_handle takes it. */
static NPU_INLINE int eagle_rxdmad_fast(void)
{
	u32 ridx = eagle_rxdmad_ridx;
	u32 d = eagle_ind_cmd_desc_base + 16 * ridx;
	u32 dw1, dw2, info, buf, off, next, len, host;
	u16 buf_id;

	if (REG32(d + 12) >> 28 != eagle_rxdmad_gen)
		return 1;
	dw1 = REG32(d + 4);
	dw2 = REG32(d + 8);
	if ((dw1 & 0x40002000) != 0x40000000 ||
	    (dw2 & 0xF000) == 0x1000 || (dw2 & 0xF000) == 0x2000 ||
	    NDBG_PRINTING(NDBG_WIFI))
		return 2;

	dbg.rxd++;
	if (ridx == EAGLE_RX_RING_MAX_IDX)
		eagle_rxdmad_gen = (eagle_rxdmad_gen + 1) & 0xF;
	next = ridx + 1;
	if (next >= 1536)
		next = 0;

	buf_id = dw2 >> 16;
	buf = eagle_buf_uncached(buf_id);
	len = (dw1 >> 16) & 0x3FFF;
	info = (len << 3) | ((dw2 >> 12) << 28);
	host = (dw2 & 0x80) | *(volatile u8 *)&wifi_force_to_cpu;
	if ((dw1 & 0x1800) == 0x800) {
		REG32(buf) = info | 2 | ((dw1 & 0x7F) << 17);
		if (host == 0) {
			off = (dw1 & 0x7F) << 1;
			dbg.rxfast++;
			if (tdma_tx_submit(buf_id, (len - off) & 0xFFFF,
					   buf + EAGLE_PKT_HEADROOM + off,
					   0) != 0) {
				dbg.rxdrop++;
				buf_id_return(buf_id);
			}
			goto done;
		}
	} else {
		REG32(buf) = info;
	}
	if (eagle_txq1_put(buf_id, len) != 0) {
		dbg.rxdrop++;
		buf_id_return(buf_id);
		if ((dw1 & 0x1800) != 0x800 && host == 0)
			npu_printf("enq slow path faill\n");
	}
done:
	eagle_rxdmad_ridx = next;
	if (++eagle_rxdmad_kick < 0) {
		u32 cidx = next ? next - 1 : EAGLE_RX_RING_MAX_IDX;

		if (eagle_emi_cidx_valid == 1)
			REG32(eagle_emi_cidx) = cidx;
		else
			REG32(eagle_ind_cmd_pcie_base + 8) = cidx;
		eagle_rxdmad_kick = 0;
	}
	return 0;
}

/* 0 done, 1 ring empty */
static NPU_INLINE int eagle_rxdmad_next(u8 *chaining)
{
	int r = *chaining ? 2 : eagle_rxdmad_fast();

	return r == 2 ? eagle_rxdmad_handle(chaining) : r;
}
#else
#define eagle_rxdmad_next(chaining)	eagle_rxdmad_handle(chaining)
#endif

/* ---- cores ---- */

/* core 1: the rxdmad ring */
NPU_HOT void eagle_rxdmad_loop(void)
{
	u8 chaining = 0;
	int empty;

	while (eagle_tx_en == 0 || eagle_rx_en == 0)
		;
	npu_printf("start %s\n", "kite_handle_rxdmad_c_ring");

	if (eagle_rxdmad_on_core2)
		return;

	eagle_rx_busy = 1;
	npu_dbg_loop(NDBG_TAG('E', 'R', 'X', 'D'));
	while (1) {
		npu_dbg_poll();
		while (eagle_tx_en == 0) {
			npu_dbg_poll();
			eagle_rx_busy = 0;
			eagle_delay(10000);
		}
		eagle_rx_busy = 1;
		NPU_PROF(NP_ERXD, dbg.rxd,
			 empty = eagle_rxdmad_next(&chaining));
		if (empty != 0)
			eagle_delay(500);
	}
}

#ifdef EAGLE_NO_TX_PUSH
/* NPUTX=0: stage host frames but never hand them to the WiFi tx ring */
#define eagle_tx_ring_push(band) (0)
#endif

/* core 2: staged frames into the WiFi tx ring, paced by the ring's own
 * dma index so the chip is never overrun */
void __attribute__((noreturn)) eagle_tx_fast_path(void)
{
	u8 chaining = 0;
	u16 dma[2] = { 0, 0 };
	u32 band, cpu, free;
	int empty;

	while (eagle_init_done == 0 || eagle_fastpath_en == 0 ||
	       eagle_txq_state != 3)
		;
	npu_printf("start %s\n", "eagle_band0_band1_tx_fast_path");

	if (eagle_rxdmad_on_core2) {
		while (eagle_tx_en == 0 || eagle_rx_en == 0)
			;
		npu_printf("start %s\n", "eagle_band0_band1_tx_fast_path");
		eagle_rx_busy = 1;
	}

	npu_dbg_loop(NDBG_TAG('E', 'T', 'X', 'F'));
	while (1) {
		npu_dbg_poll();
		while (eagle_txq_state != 3) {
			npu_dbg_poll();
			if (eagle_rxdmad_on_core2 && eagle_tx_en == 0)
				eagle_rx_busy = 0;
			eagle_rx_stopped = 1;
			eagle_delay(2000);
			dma[0] = 0;
			dma[1] = 0;
		}
		eagle_rx_stopped = 0;

		if (eagle_rxdmad_on_core2) {
			eagle_rx_busy = 1;
			NPU_PROF(NP_ERXD, dbg.rxd,
				 empty = eagle_rxdmad_handle(&chaining));
			if (empty != 0)
				eagle_delay(500);
		}

		for (band = 1; band != (u32)-1; band--) {
			cpu = eagle_tx_ring_cpu_idx[band];
			free = (dma[band] - cpu - 1) & EAGLE_TX_RING_MASK;
			if (free <= EAGLE_TX_RING_ROOM) {
				eagle_delay(5000);
				dma[band] = (u16)REG32(eagle_tx_ring_pcie_base[band] + 0xC);
				continue;
			}
			while (free > EAGLE_TX_RING_ROOM) {
				free--;
				NPU_PROF(NP_ETXP, dbg.push[band],
					 eagle_tx_ring_push(band));
				if (free == EAGLE_TX_RING_ROOM)
					break;
				NPU_PROF(NP_ELAN, dbg.lan[0] + dbg.lan[1],
					 eagle_tdma_to_wifi(band, free - 6));
				if (free <= 129)
					break;
			}
			eagle_delay(1000);
			dma[band] = (u16)REG32(eagle_tx_ring_pcie_base[band] + 0xC);
		}
	}
}

/* core 3: the host adaptor in both directions plus the tx done ring */
void __attribute__((noreturn)) eagle_core3_loop(void)
{
	u32 started = 0;
	int done;
#ifdef AN758X
	u32 t300 = timer_raw_tick;
#endif

	npu_dbg_loop(NDBG_TAG('E', 'C', '3', 'L'));
	while (1) {
		npu_dbg_poll();
#ifdef AN758X
		if (timer_raw_tick < t300)
			t300 = timer_raw_tick;
		if (timer_raw_tick - t300 > 300) {
			xpon_license_check();
			t300 = timer_raw_tick;
		}
#endif
		if (started == 0 && eagle_init_done != 0 &&
		    eagle_rro_state == 3 && eagle_tx_en != 0) {
			npu_printf("start %s\n", "npu_core3_main_loop_handle");
			started = 1;
		}
		if (started != 0 && eagle_rro_state != 3) {
			eagle_delay(2000);
			started = 0;
		}
		eagle_dbg_tick();
		if (started == 0)
			continue;

#ifdef HAS_NPU_WIFI_TX
		NPU_PROF(NP_EHIN, dbg.in[0] + dbg.in[1],
			 eagle_hostadpt_drain(0); eagle_hostadpt_drain(1));
#endif
		if (eagle_rx_ring_init_done[0] != 0 &&
		    eagle_rx_ring_init_done[1] != 0 &&
		    hostadpt_tx_ring_ready == 1)
			NPU_PROF(NP_EHOUT, dbg.rxout,
				 eagle_txq_drain(0); eagle_txq_drain(1);
				 eagle_mseg_drain(1));
		NPU_PROF(NP_ETXD, dbg.txdone, done = eagle_txdone_poll());
		if (done == 0)
			eagle_delay(10);
	}
}

#if defined(AN7552)
/* AN7552 has two cores: core 0 carries the queue drain and the refill
 * that the larger parts give to cores 3 and 4. */
void __attribute__((noreturn)) eagle_core0_loop(void)
{
	/* start once init, rx, tx and both rings are up */
	npu_dbg_loop(NDBG_TAG('E', 'C', '0', 'L'));
	while (eagle_init_done == 0 || eagle_rx_en == 0 || eagle_tx_en == 0 ||
	       eagle_rx_ring_init_done[0] == 0 || eagle_rx_ring_init_done[1] == 0)
		npu_dbg_poll();
	eagle_delay(10000);
	npu_printf("start %s\n", "npu_start_kite_rro_v31_refill_ring_donebitmode");

	while (1) {
		npu_dbg_poll();
		while (eagle_rx_en == 0) {
			npu_dbg_poll();
			eagle_delay(1000);
		}

		if (hostadpt_tx_ring_ready == 1)
			NPU_PROF(NP_EHOUT, dbg.rxout,
				 eagle_txq_drain(0); eagle_txq_drain(1);
				 eagle_mseg_drain(1));
		NPU_PROF(NP_ERFL, dbg.refill[0] + dbg.refill[1],
			 eagle_rx_ring_sweep(0); eagle_rx_ring_sweep(1));
		eagle_delay(100);
	}
}
#endif

/* core 4: keep both rx rings stocked */
void __attribute__((noreturn)) eagle_rx_refill_loop(void)
{
	npu_dbg_loop(NDBG_TAG('E', 'R', 'F', 'L'));
	while (eagle_init_done == 0 || eagle_rx_en == 0 || eagle_tx_en == 0 ||
	       eagle_rx_ring_init_done[0] == 0 || eagle_rx_ring_init_done[1] == 0)
		npu_dbg_poll();
	eagle_delay(10000);
	npu_printf("start\n");

	while (1) {
		npu_dbg_poll();
		while (eagle_rx_en == 0) {
			npu_dbg_poll();
			eagle_delay(10000);
		}

		NPU_PROF(NP_ERFL, dbg.refill[0], eagle_rx_ring_sweep(0));
		eagle_delay(100);
		NPU_PROF(NP_ERFL, dbg.refill[1], eagle_rx_ring_sweep(1));
		eagle_delay(100);
	}
}

#endif /* WIFI_EAGLE */
