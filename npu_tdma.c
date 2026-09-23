/*
 * AN75XX NPU firmware - TDMA rings, buffer manager and DMA copy
 *
 * The TDMA engine carries frames between the NPU and the wired side.
 * The rings exist only where HAS_BME does, which is AN7552 and AN7583
 * with a WiFi chip. AN7581 has no TDMA WiFi path.
 */

#include "npu_internal.h"
#include "npu_wifi.h"


/* ================================================================
 * DMA copy engine (0x1FB30000)
 *
 * Channel-based hardware copy engine. Each channel has a 16-byte
 * register set: src, dst, ctrl. Channel 3 used for host ring DMA.
 * ================================================================ */

#ifdef HAS_WIFI
void bridge_dma_copy(u32 channel, u32 src, u32 dst, u32 len)
{
	u32 mask = 1u << channel;

	REG32(DMA_COPY_SRC(channel)) = src;
	REG32(DMA_COPY_DST(channel)) = dst;
	REG32(DMA_COPY_CTRL(channel)) = (len << 16) | 0x23;

	while (!(REG32(DMA_COPY_STATUS) & mask))
		;
	REG32(DMA_COPY_STATUS) = mask;
}
#endif

/* ================================================================
 * TDMA / Buffer Manager initialization
 * ================================================================ */

/* Buffer manager state (software free-list path) */
static u32 buf_mgr_alloc_cfg;
static u32 buf_mgr_alloc_idx;
static u32 buf_mgr_free_cfg;
static u32 buf_mgr_free_idx;
static u32 buf_mgr_id_base;
static u16 buf_mgr_alloc_widx;
static u16 buf_mgr_free_widx;

/* TDMA descriptor ring state */
static u32 tdma_tx_ring0_base;
static u32 tdma_tx_ring1_base;
static u32 tdma_tx_ring0_cnt;
static u32 tdma_tx_ring1_cnt;

/* TDMA TX ring per-band state */
u32 tdma_tx_sw_idx[8];

/* BME descriptor state */
u32 tdma_bme_dscp_base_addr;

/* TDMA init: zero 512KB SRAM, clear alloc table */
void tdma_init(void)
{
	sram_buf_init();
}

#ifdef HAS_BME
#ifdef WIFI_KITE
#define BME_RING_ENTRIES	512

static u32 bme_ridx;

/* PLIC 32: frames the PPE did not bind come back on the BME ring and
 * go to the host */
static void bme_done_isr(int src)
{
	u32 hw = REG32(BME_BASE + 0x004);
	u32 st = REG32(BME_BASE + 0x02C);
	u32 n, *d;

	(void)src;
	n = (hw >= bme_ridx) ? hw - bme_ridx : hw + BME_RING_ENTRIES - bme_ridx;
	REG32(BME_BASE + 0x02C) = st;

	d = (u32 *)(tdma_bme_dscp_base_addr + bme_ridx * 8);
	while (n-- != 0 && (d[0] & 0x80000000)) {
		if (d[0] & 0x40000000) {
			npu_printf("Error: the HW free bind bufid was enabled.\n");
			wifi_cnt_inc(2, 4);
		} else {
			/* w0 15:0 buffer id, w1 15:0 wcid, 20:16 info */
			if (pkt_forward_bme((s16)d[0], (u16)d[1],
					    (d[1] >> 16) & 0x1F) != 0)
				buf_id_free(0, 0, (u16)d[0]);
			wifi_cnt_inc(2, 8);
		}
		d[0] = 0;
		d[1] = 0;
		bme_ridx = bme_ridx + 1 == BME_RING_ENTRIES ? 0 : bme_ridx + 1;
		d = (u32 *)(tdma_bme_dscp_base_addr + bme_ridx * 8);
	}
	REG32(BME_BASE + 0x008) = bme_ridx;
}

/* BME init: buffer move engine descriptor ring */
static void tdma_bme_init(void)
{
	u32 *desc;
	u32 i;

	tdma_bme_dscp_base_addr = sram_buf_alloc(134);
	REG32(BME_BASE + 0x00C) = tdma_bme_dscp_base_addr;
	npu_printf("tdma_bme_dscp =%x\n", tdma_bme_dscp_base_addr);

	/* zero 512 8-byte descriptors (4KB) */
	desc = (u32 *)tdma_bme_dscp_base_addr;
	for (i = 0; i < 512; i++) {
		desc[i * 2] = 0;
		desc[i * 2 + 1] = 0;
	}

	REG32(BME_BASE + 0x000) = 511;

	/* buffer info: 8 units, 8 bytes */
	REG32(BME_BASE + 0x010) = (REG32(BME_BASE + 0x010) & ~0xF0u) | 0x80;
	npu_printf("set_bufid_info_util_size = %x/%x\n", 8,
		   REG32(BME_BASE + 0x010));
	REG32(BME_BASE + 0x010) = (REG32(BME_BASE + 0x010) & ~0xFu) | 8;
	npu_printf("set_bufid_info_byte = %x/%x\n", 8,
		   REG32(BME_BASE + 0x010));

	bme_ridx = 0;
	REG32(BME_BASE + 0x008) = 0;

	/* register BME done ISR on PLIC source 32 */
	plic_register_isr(INTR_BME_DONE, bme_done_isr);
	npu_printf("bme plic register done, INTR_BUFID_MOVE_ENGINE(%d)\n", INTR_BME_DONE);

	/* configure BME: interrupt mode + SRAM mode, ID threshold */
	REG32(BME_CSR_CTRL) = (REG32(BME_CSR_CTRL) & 0xFFFF0000) | 0x10;
	npu_printf("BME intMode + sarm mode, id thld=%x\n", REG32(BME_CSR_CTRL));

	/* timeout */
	REG32(BME_CSR_MAX_INDEX) = 1000;
	npu_printf("BME timeout setting%x = %x\n", BME_CSR_MAX_INDEX,
		   REG32(BME_CSR_MAX_INDEX));

	/* global config: enable with size=8 */
	{
		u32 cfg = REG32(BME_CSR_BASE_ADDR);

		npu_printf("[%s] defult: BME_CSR_GLB_CFG(%x)=%x\n",
			   "tdma_bme_init", BME_CSR_BASE_ADDR, cfg);
		npu_printf(" csr_mng_en enable");
		cfg = (cfg & 0xFFE0) | 0x170007;
		REG32(BME_CSR_BASE_ADDR) = cfg;
		npu_printf("[%s] configed: BME_CSR_GLB_CFG(%x), bme size=%d\n",
			   "tdma_bme_init", cfg, 8);
	}
}

/* TDMA BME init: hardware buffer-ID allocator path */
#endif /* WIFI_KITE */

/* PLIC 33 (25 on AN7552): acknowledge the BMGR status */
static void bmgr_isr(int src)
{
	(void)src;
	REG32(BMGR_STATUS) = REG32(BMGR_STATUS);
	if (counter_base_tri)
		(*(u32 *)(counter_base_tri + 60))++;
}

#if defined(AN7552)
#define BMGR_READY	8
#else
#define BMGR_READY	1
#endif

/* hand the rx buffer ids to the hardware allocator */
void tdma_bmgr_init(void)
{
	u32 st;

	REG32(BMGR_BUF_ID_BASE) = sram_buf_alloc(138);
	REG32(BMGR_BASE + 0x004) = 0;
	REG32(BMGR_BASE + 0x008) = BUFID_POOL_ENTRIES;
	REG32(BMGR_BASE + 0x00C) = 2;
	REG32(BMGR_BASE + 0x010) = 2;
	REG32(BMGR_CFG) = 15;
	REG32(BMGR_BASE + 0x030) = 7;
	REG32(BMGR_INIT) = 1;
	st = REG32(BMGR_STATUS);
	while (!(REG32(BMGR_STATUS) & BMGR_READY))
		;
	REG32(BMGR_STATUS) = st;
	plic_register_isr(INTR_BMGR, bmgr_isr);
}

/* wait until the FE took every TDMA tx descriptor */
void tdma_tx_wait_idle(void)
{
	u32 ring;

	for (ring = 0; ring < 2; ring++)
		while (tdma_tx_sw_idx[ring] !=
		       REG32(TDMA_TX_RING0_DMA_IDX + 16 * ring))
			delay_ms(10);
}

/* after a WiFi stop: wait for the PPE to return every buffer, then
 * rebuild the buffer id allocator */
void tdma_bmgr_reinit(void)
{
	u32 pass, v;

	delay_ms(100);
	for (pass = 0; pass < 2; pass++) {
		while (REG32(0x1FB50FE4) & 0xFFFF)
			delay_ms(100);
		delay_ms(100);
	}

	if (tdma_bmgr_mode == 0) {
		rx_bufid_pool_reset();
		return;
	}

	npu_printf("bmgr reinit %d\n", 677);
	plic_disable(INTR_BMGR);
	do {
		REG32(BMGR_CFG) = 0;
		v = REG32(BMGR_RESET);
		REG32(BMGR_RESET) = ~v;
		REG32(BMGR_RESET) = v;
		REG32(BMGR_CFG) = 15;
		REG32(BMGR_INIT) = 1;
		while ((v = REG32(BMGR_STATUS)) == 0)
			;
		if (!(v & 8))
			npu_printf("bmgr init fail, vale:%d retry\n", v);
	} while (!(v & 8));
	REG32(BMGR_STATUS) = REG32(BMGR_STATUS);
	npu_printf("bmgr reinit %d\n", 704);
	plic_enable(INTR_BMGR);
}
#endif /* HAS_BME */

#ifdef HAS_WIFI
#ifdef HAS_BME
/* TDMA TX init: configure TX descriptor rings */
void tdma_tx_init(void)
{
	u32 ring_base;
	u32 *desc;
	u32 i;

	/* allocate ring 0 */
	ring_base = sram_buf_alloc(132);
	REG32(TDMA_TX_RING0_BASE) = ring_base & 0x1FFFFFFF;
	npu_printf("%s L%d dscpBaseAddr=%x reg:%x\n",
		   "tdma_tx_init", 975, ring_base, ring_base & 0x1FFFFFFF);

	REG32(TDMA_TX_RING0_CFG) = (REG32(TDMA_TX_RING0_CFG) & 0xFFF8FFFF) | 0x50000;
	REG32(TDMA_TX_RING0_CFG) = (REG32(TDMA_TX_RING0_CFG) & 0xFFFFF000) | 0x400;

	/* ring 1 at ring_base + 0x2000 */
	REG32(TDMA_TX_RING1_BASE) = (ring_base + 0x2000) & 0x1FFFFFFF;
	npu_printf("%s L%d dscpBaseAddr=%x reg:%x\n",
		   "tdma_tx_init", 975, ring_base, ring_base & 0x1FFFFFFF);

	REG32(TDMA_TX_RING1_CFG) = (REG32(TDMA_TX_RING1_CFG) & 0xFFF8FFFF) | 0x50000;
	REG32(TDMA_TX_RING1_CFG) = (REG32(TDMA_TX_RING1_CFG) & 0xFFFFF000) | 0x400;

	/* init ring 0 descriptors: 1024 entries x 8 bytes */
	tdma_tx_ring0_base = ring_base;
	desc = (u32 *)ring_base;
	for (i = 0; i < 0x2000; i += 8) {
		desc[i / 4] = (desc[i / 4] & 0x3FFFC000) | 0xC0000800;
	}

	REG32(TDMA_TX_RING0_IDX) = 0;
	tdma_tx_ring0_cnt = 1023;

	/* interrupt config for ring 0 */
	REG32(TDMA_INT_CFG1) = 16843009;
	REG32(TDMA_INT_CFG0) = 1;
	npu_printf("%s L%d :: ring:%d %x=%x %x=%x\n",
		   "tdma_set_tx_ring_to_int", 109, 0,
		   TDMA_INT_CFG1, REG32(TDMA_INT_CFG1),
		   TDMA_INT_CFG0, REG32(TDMA_INT_CFG0));

	/* init ring 1 descriptors */
	tdma_tx_ring1_base = ring_base + 0x2000;
	desc = (u32 *)(ring_base + 0x2000);
	for (i = 0; i < 0x2000; i += 8) {
		desc[i / 4] = (desc[i / 4] & 0x3FFFC000) | 0xC0000800;
	}

	REG32(TDMA_TX_RING1_IDX) = 0;
	REG32(TDMA_INT_CFG1) = 33686018;
	REG32(TDMA_INT_CFG0) = 17;
	tdma_tx_ring1_cnt = 1023;
	npu_printf("%s L%d :: ring:%d %x=%x %x=%x\n",
		   "tdma_set_tx_ring_to_int", 109, 1,
		   TDMA_INT_CFG1, REG32(TDMA_INT_CFG1),
		   TDMA_INT_CFG0, REG32(TDMA_INT_CFG0));

	/* global TDMA config */
#if defined(AN7552) && defined(WIFI_EAGLE)
	/* three ORs, no FC_CFG0/1 */
	REG32(TDMA_GLB_CFG) |= 1;
	REG32(TDMA_GLB_CFG) |= 0x40;
	REG32(TDMA_GLB_CFG) |= 0x30;
#else
	REG32(TDMA_GLB_CFG) = (REG32(TDMA_GLB_CFG) & 0xFF8FFFFE) | 0x400001;
	REG32(TDMA_GLB_CFG) |= 0x40;
	REG32(TDMA_GLB_CFG) |= 0x30;
	REG32(TDMA_GLB_CFG) = (REG32(TDMA_GLB_CFG) & 0xFF7FFFFF) | 0x800000;

	/* flow control */
	REG32(TDMA_FC_CFG0) |= 0x40004000;
	REG32(TDMA_FC_CFG1) |= 0x40004000;

	REG32(TDMA_GLB_CFG) = (REG32(TDMA_GLB_CFG) & 0xFFFFC7FF) | 0x3000;
#endif
	npu_printf("%s L%d INTR_TDMA_0 = %d INTR_PPE_WIFI_BUF_ID = %d\n",
		   "tdma_tx_init", 1038, 184, 95);
#ifdef WIFI_EAGLE
	plic_register_isr(95, ppe_wifi_bufid_isr);
#endif

	/* AN7552-specific flow control */
	if (REG32(CHIP_ID_REG) >> 16 == 15) {
		npu_printf("set AN7552 flow ctrl\n");
		REG32(TDMA_FC_CFG0) = 0x80048004;
		REG32(AN7552_FC_REG) = 15401194;
	} else {
		REG32(TDMA_FC_CFG2) = 3;
	}

	/* WiFi buffer config: the field layout differs per WiFi chip, not
	 * per SoC. AN7581 never reaches here - it has no TDMA TX path. */
#if defined(WIFI_EAGLE)
	REG32(TDMA_WIFI_BUF_CFG) =
		(REG32(TDMA_WIFI_BUF_CFG) & 0xFFB300FF) | 0x190100;
#else
	REG32(TDMA_WIFI_BUF_CFG) =
		(REG32(TDMA_WIFI_BUF_CFG) & 0xFFF300FF) | 0x590100;
#endif
	npu_printf("[%s] PPE_WIFI_BUF_CFG=%x, value=%x\n",
		   "tdma_tx_init", TDMA_WIFI_BUF_CFG, REG32(TDMA_WIFI_BUF_CFG));

#ifdef HAS_BME
#ifdef WIFI_KITE
	tdma_bme_init();
#endif
#endif
}

#ifdef HAS_BME
/* TDMA RX init: two 1024-entry rings of 32-byte descriptors, each
 * pointing at a 2KB slot of the host tx packet buffer */
void tdma_rx_init(void)
{
	u32 base, phys, ring, i, desc;
	s32 buf_id;

	npu_printf("%s\n", "tdma_rx_init");

	REG32(TDMA_GLB_CFG) &= ~0x80000u;
	REG32(TDMA_RX_CFG) |= 0x80000000u;

	base = sram_buf_alloc(133);
	phys = base & 0x1FFFFFFF;

	for (ring = 0; ring < TDMA_RX_RINGS; ring++) {
		REG32(TDMA_RX_BASE_PTR(ring)) =
			(base + ring * TDMA_RX_RING_STRIDE) & 0x1FFFFFFF;
		REG32(TDMA_RX_BASE_PTR(ring) + 4) =
			(REG32(TDMA_RX_BASE_PTR(ring) + 4) & 0xFFFFF000) |
			TDMA_RX_RING_DESCS;
		REG32(TDMA_RX_BASE_PTR(ring) + 4) &= 0x8000FFFFu;
	}

	/* the host publishes the tx packet buffer over the mailbox */
	while (npu_tx_pkt_buf_addr == 0)
		delay_ms(100);

	for (ring = 0; ring < TDMA_RX_RINGS; ring++) {
		tdma_rx_dscp_base[ring] = base + ring * TDMA_RX_RING_STRIDE;

		for (i = 0; i < TDMA_RX_RING_DESCS; i++) {
			buf_id = tx_token_alloc();
			if (buf_id == -1) {
				tdma_rx_alloc_fail++;
				npu_printf("%s skbufid=%d index=%d maclloc failed\n",
					   "tdma_rx_init", -1, i);
				continue;
			}
			desc = tdma_rx_dscp_base[ring] + TDMA_RX_DESC_SIZE * i;
			REG32(desc + 8) = ((((u32)buf_id << 11) +
					    npu_tx_pkt_buf_addr) &
					   0x3FFFFFFF) | 0x80000000;
			REG32(desc + 4) = (REG32(desc + 4) & 0x7FFF0000) | 0x800;
			tdma_rx_desc_count++;
		}

		REG32(TDMA_RX_BASE_PTR(ring) + 8) = TDMA_RX_RING_DESCS - 1;
		REG32(TDMA_RX_BASE_PTR(ring) + 12) = 0;
	}

	REG32(TDMA_GLB_CFG) = (REG32(TDMA_GLB_CFG) & 0xFFF8FFFB) | 0x40004;

	npu_printf("%s L%d tdma rx ring dscpBaseAddr_uncache=%x reg:%x tdma used pkt_buf_phy_addr:%x\n",
		   "tdma_rx_init", 1570, base, phys, npu_tx_pkt_buf_addr);
}
#endif /* HAS_BME */
#endif /* HAS_BME */

static u32 *tdma_stats_base(u32 band)
{
	if (band == 0)
		return (u32 *)counter_base_2g;
	if (band == 1)
		return (u32 *)counter_base_5g;
	return (u32 *)counter_base_tri;
}

/* WiFi -> wired. Eight-byte descriptors, 1024 to a ring: word 0 carries
 * the frame length in bits 12:0 and the buffer token in 28:14, word 1
 * the buffer itself. */
#ifdef WIFI_KITE
static u32 tdma_tx_mutex[2];
#endif

int __attribute__((noinline)) tdma_tx_submit(u32 token, u32 pkt_len,
						    u32 buf_addr, u32 band)
{
	u32 base = (band != 0) ? tdma_tx_ring1_base : tdma_tx_ring0_base;
	volatile u32 *hw_idx_reg = (volatile u32 *)(TDMA_TX_RING0_DMA_IDX +
						    16 * band);
	u32 sw_idx = tdma_tx_sw_idx[band];
	u32 retries = 5;
	u32 hw_idx, free_slots, next, w0;
	u32 *desc;

	if (base == 0)
		return -1;

#ifdef WIFI_KITE
	/* pipeline mode: cores 1 and 2 both send */
	u32 locked = wifi_debug_flags & 1;

	if (locked) {
		hw_mutex_lock(tdma_tx_mutex);
		sw_idx = tdma_tx_sw_idx[band];
	}
#endif
	while (1) {
		hw_idx = *hw_idx_reg;
		free_slots = (sw_idx < hw_idx) ? (hw_idx - sw_idx - 1)
					       : (1023 - sw_idx + hw_idx);
		if (free_slots > 4)
			break;
		{ volatile u32 i; for (i = 0; i < 300; i++) ; }
		if ((wifi_debug_flags & 4))
			(*(tdma_stats_base(band) + 63))++;
		if (--retries == 0) {
			if ((wifi_debug_flags & 4))
				(*(tdma_stats_base(band) + 64))++;
#ifdef WIFI_KITE
			/* release the mutex while the ring stays full */
			if (locked)
				hw_mutex_unlock(tdma_tx_mutex);
#endif
			return -1;
		}
	}

	desc = (u32 *)(base + 8 * sw_idx);
	w0 = (desc[0] & 0x3FFFFFFF) | 0x40000000;
	w0 = (w0 & 0xC0001FFF) | (token << 14);

	if (pkt_len < 60) {
		npu_memset((void *)(buf_addr + pkt_len), 0, 60 - pkt_len);
		pkt_len = 60;
	}

	next = (sw_idx > 0x3FE) ? 0 : sw_idx + 1;

	if ((wifi_debug_flags & 4))
		(*(tdma_stats_base(band) + 79))++;

	desc[1] = (buf_addr & 0x3FFFFFFF) | 0x80000000;
	tdma_tx_sw_idx[band] = next;
	desc[0] = (w0 & 0xFFFFE000) | pkt_len;
#if defined(AN7552) && defined(WIFI_EAGLE)
	/* short wait for the id's rx refill */
	if (eagle_sync[token] == 0) {
		volatile u32 i;

		for (i = 0; i < 1000; i++)
			;
	}
#endif
	REG32(TDMA_TX_RING0_IDX + 16 * band) = next;
#ifdef WIFI_KITE
	if (locked)
		hw_mutex_unlock(tdma_tx_mutex);
#endif
	return 0;
}
#endif /* HAS_WIFI */

/* Software buffer manager init (non-TDMA path) */
void buf_mgr_init(void)
{
	u16 *pool;
	u32 i;

	buf_mgr_alloc_cfg = 12;
	buf_mgr_alloc_idx = 0;
	buf_mgr_free_cfg = 13;
	buf_mgr_free_idx = 0;

	pool = (u16 *)sram_buf_alloc(140);
	buf_mgr_id_base = (u32)pool;

	for (i = 0; i < 5600; i++)
		pool[i] = (u16)i;

	buf_mgr_alloc_widx = 0;
	buf_mgr_free_widx = 0;
}
