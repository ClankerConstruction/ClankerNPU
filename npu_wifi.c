#include "npu_internal.h"

/* ================================================================
 * Debug counter ISR
 * ================================================================ */

void dbg_cnt_isr(int src)
{
	u32 intr_cores = REG32(NPU_DBG_CNT_INTR_CORE);

	npu_printf("(%s) intSrc:%d, intr_cores:0x%x\n",
		   "dbg_cnt_isr", src, intr_cores);

	if (intr_cores == 0) {
		npu_printf("Error(%s): intr_cores==0 (DBG_CNT_INTR_CORE:0x%x)\n",
			   "dbg_cnt_isr", intr_cores);
		return;
	}

	for (int i = 0; i < (int)MAX_CORE_NUM; i++) {
		if (intr_cores & (1u << i)) {
			npu_printf("-> Core%d's intr_addr:%d\n",
				   i, REG32(NPU_DBG_CNT_BASE + i * 4));
		}
	}
}

/* ================================================================
 * Buffer ID management
 *
 * Two paths: hardware BME uses custom CSRs 0xBC8-0xBEA,
 * software path uses a sequential free-list.
 * band=0 → 2.4G, band=1 → 5G, band=2 → 6G (future)
 * ================================================================ */

#define BUF_ID_INVALID   0xFFFF
#define BUF_ID_CSR_BASE  0xBC8

/* hardware buffer ID allocator via custom CSR */
static u32 buf_id_alloc_hw(u32 band, u32 dir)
{
	u32 csr_addr;
	u32 val;

	/* CSR matrix: band 0..2 x dir 0..2, base 0xBC8 */
	if (band == 0) {
		csr_addr = BUF_ID_CSR_BASE + dir;
	} else if (band == 1) {
		csr_addr = BUF_ID_CSR_BASE + 0x10 + dir;
	} else if (band == 2) {
		csr_addr = BUF_ID_CSR_BASE + 0x20 + dir;
	} else {
		return (u32)-1;
	}

	__asm__ volatile("fence" ::: "memory");
	val = csr_read(mhartid); /* placeholder - actual CSR read uses csr_addr */
	(void)csr_addr;
	return (val << 16) >> 16;
}

/* buffer ID return via MMIO */
static void buf_id_free(u32 result_type, u32 band, u32 buf_id)
{
	u32 base;

	if (result_type == 0) {
		/* write to buffer ID base table */
		((volatile u32 *)(sram_buf_pad[0]))[512 * band + 67 + band] = buf_id;
	} else {
		if (result_type == 1)
			base = BMGR_BASE;
		else
			base = 0x1EC05800;
		REG32(base + 4 * (band * 512 + 67 + band)) = buf_id;
	}
}

/* ================================================================
 * Counter / statistics infrastructure
 * ================================================================ */

static u32 counter_base_2g;
static u32 counter_base_5g;
static u32 counter_base_tri;
static u32 wcid_counter_base_2g;
static u32 wcid_counter_base_5g;

/* buf_id recycle ring (5600-entry ring of u16 buffer indices) */
static u32 bufid_enq_mutex[2];
static u16 bufid_widx;
static u32 bufid_ring_base;
static u32 bufid_enq_count;
static u32 bufid_deq_mutex[2];
static u16 bufid_ridx;
static u32 bufid_deq_count;

static void __attribute__((noinline)) bufid_ring_mutex_init(void)
{
	bufid_enq_mutex[0] = 29;
	bufid_enq_mutex[1] = 0;
	bufid_deq_mutex[0] = 28;
	bufid_deq_mutex[1] = 0;
}

static void buf_id_return(u16 buf_id)
{
	hw_mutex_lock(bufid_enq_mutex);
	*(u16 *)(bufid_ring_base + 2 * (u32)bufid_widx) = buf_id;
	bufid_enq_count++;
	if ((wifi_debug_flags & 4) && counter_base_tri)
		(*(u32 *)(counter_base_tri + 0x18))++;
	bufid_widx = (bufid_widx + 1 == 5600) ? 0 : bufid_widx + 1;
	hw_mutex_unlock(bufid_enq_mutex);
}

static s32 buf_id_alloc_ring(void)
{
	u16 next;
	s32 id;

	hw_mutex_lock(bufid_deq_mutex);
	next = (bufid_ridx + 1 == 5600) ? 0 : bufid_ridx + 1;
	if (bufid_widx == next) {
		if ((wifi_debug_flags & 4) && counter_base_tri)
			(*(u32 *)(counter_base_tri + 0x20))++;
		hw_mutex_unlock(bufid_deq_mutex);
		return -1;
	}
	bufid_deq_count++;
	if ((wifi_debug_flags & 4) && counter_base_tri)
		(*(u32 *)(counter_base_tri + 0x1C))++;
	id = *(s16 *)(bufid_ring_base + 2 * (u32)bufid_ridx);
	bufid_ridx = next;
	hw_mutex_unlock(bufid_deq_mutex);
	return id;
}

void bufid_pool_init(void)
{
	u16 *ring;
	u32 i;

	bufid_deq_mutex[0] = 28;
	bufid_deq_mutex[1] = 0;
	bufid_enq_mutex[0] = 29;
	bufid_enq_mutex[1] = 0;
	ring = (u16 *)sram_buf_alloc(138);
	bufid_ring_base = (u32)ring;
	for (i = 0; i < 5600; i++)
		ring[i] = (u16)i;
	bufid_ridx = 0;
	bufid_widx = 0;
}

static void counter_init(u32 band)
{
	u32 *base;
	u32 count;

	npu_printf("%s:%d\n", "counter_init", band);

	if (band == 1) {
		counter_base_5g = sram_buf_alloc(9);
		base = (u32 *)counter_base_5g;
		count = 250;
	} else if (band == 0) {
		counter_base_2g = sram_buf_alloc(10);
		base = (u32 *)counter_base_2g;
		count = 250;
	} else {
		counter_base_tri = sram_buf_alloc(11);
		base = (u32 *)counter_base_tri;
		count = 28;
	}

	npu_memset(base, 0, count * 4);
}

static void wcid_counter_init(u32 band)
{
	u32 *base;

	npu_printf("%s:%d\n", "wcid_counter_init", band);

	if (band == 1) {
		wcid_counter_base_5g = sram_buf_alloc(19);
		base = (u32 *)wcid_counter_base_5g;
	} else if (band == 0) {
		wcid_counter_base_2g = sram_buf_alloc(20);
		base = (u32 *)wcid_counter_base_2g;
	} else {
		return;
	}

	npu_memset(base, 0, 256 * 4);
}


/* ================================================================
 * DMA copy engine (0x1FB30000)
 *
 * Channel-based hardware copy engine. Each channel has a 16-byte
 * register set: src, dst, ctrl. Channel 3 used for host ring DMA.
 * ================================================================ */

#ifdef WIFI_KITE
static void bridge_dma_copy(u32 channel, u32 src, u32 dst, u32 len)
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

/* TDMA HW registers */
#define TDMA_TX_RING0_BASE    0x1FB50800
#define TDMA_TX_RING0_CFG     0x1FB50804
#define TDMA_TX_RING1_BASE    0x1FB50810
#define TDMA_TX_RING1_CFG     0x1FB50814
#define TDMA_TX_RING0_IDX     0x1FB50808
#define TDMA_TX_RING1_IDX     0x1FB50818
#define TDMA_INT_CFG0         0x1FB50A28
#define TDMA_INT_CFG1         0x1FB50A2C
#define TDMA_GLB_CFG          0x1FB50A04
#define TDMA_FC_CFG0          0x1FB521F0
#define TDMA_FC_CFG1          0x1FB521F4
#define TDMA_FC_CFG2          0x1FB52230
#define TDMA_WIFI_BUF_CFG     0x1FB50FE8
#define AN7552_FC_REG         0x1FB501BC

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
static u32 tdma_tx_ring_base[4];
static u32 tdma_tx_sw_idx[8];

/* BME descriptor state */
static u32 tdma_bme_dscp_base_addr;

/* TDMA init: zero 512KB SRAM, clear alloc table */
void tdma_init(void)
{
	sram_buf_init();
}

#ifdef HAS_BME
/* BME ISR handler */
static void bme_done_isr(int src)
{
	(void)src;
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
	REG32(BME_BASE + 0x008) = 0;

	/* register BME done ISR on PLIC source 32 */
	plic_register_isr(INTR_BME_DONE, bme_done_isr);
	npu_printf("bme plic register done, INTR_BUFID_MOVE_ENGINE(%d)\n", INTR_BME_DONE);

	/* configure BME: interrupt mode + SRAM mode, ID threshold */
	REG32(BME_CSR_CTRL) = (REG32(BME_CSR_CTRL) & 0xFFFF0000) | 0x10;
	npu_printf("BME intMode + sarm mode, id thld=%x\n", REG32(BME_CSR_CTRL));

	/* timeout */
	REG32(BME_CSR_MAX_INDEX) = 1000;
	npu_printf("BME timeout setting%x = %x\n", BME_CSR_MAX_INDEX, 1000);

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
void tdma_bmgr_init(void)
{
	u32 buf_base;

	npu_printf("do tdma_bmgr_init\n");

	buf_base = sram_buf_alloc(138);

	REG32(BMGR_BUF_ID_BASE) = buf_base;
	REG32(BMGR_BASE + 0x004) = 0;
	REG32(BMGR_BASE + 0x008) = 5600;
	REG32(BMGR_BASE + 0x00C) = 2;
	REG32(BMGR_BASE + 0x010) = 2;
	REG32(BMGR_BASE + 0x028) = 15;
	REG32(BMGR_BASE + 0x030) = 7;
	REG32(BMGR_INIT) = 1;

#if defined(AN7552)
	while (!(REG32(BMGR_BASE + 0x02C) & 8))
#else
	while (!(REG32(BMGR_BASE + 0x02C) & 1))
#endif
		;

	plic_enable_wrapper(INTR_BMGR);
}
#endif /* HAS_BME */

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
		   (u32)&REG32(TDMA_INT_CFG1) - 1492, 16843009,
		   (u32)&REG32(TDMA_INT_CFG0) - 1496, 1);

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
		   TDMA_INT_CFG1, 33686018,
		   TDMA_INT_CFG0, 17);

	/* global TDMA config */
	REG32(TDMA_GLB_CFG) = (REG32(TDMA_GLB_CFG) & 0xFF8FFFFE) | 0x400001;
	REG32(TDMA_GLB_CFG) |= 0x40;
	REG32(TDMA_GLB_CFG) |= 0x30;
	REG32(TDMA_GLB_CFG) = (REG32(TDMA_GLB_CFG) & 0xFF7FFFFF) | 0x800000;

	/* flow control */
	REG32(TDMA_FC_CFG0) |= 0x40004000;
	REG32(TDMA_FC_CFG1) |= 0x40004000;

	REG32(TDMA_GLB_CFG) = (REG32(TDMA_GLB_CFG) & 0xFFFFC7FF) | 0x3000;
	npu_printf("%s L%d INTR_TDMA_0 = %d INTR_PPE_WIFI_BUF_ID = %d\n",
		   "tdma_tx_init", 1038, 184, 95);

	/* AN7552-specific flow control */
	if (REG32(CHIP_ID_REG) >> 16 == 15) {
		npu_printf("set AN7552 flow ctrl\n");
		REG32(TDMA_FC_CFG0) = 0x80004004;
		REG32(AN7552_FC_REG) = 15401194;
	} else {
		REG32(TDMA_FC_CFG2) = 3;
	}

	/* WiFi buffer config */
#if defined(AN7581)
	REG32(TDMA_WIFI_BUF_CFG) = (REG32(TDMA_WIFI_BUF_CFG) & 0xFFE200FF) | 0x190100;
#else
	REG32(TDMA_WIFI_BUF_CFG) = (REG32(TDMA_WIFI_BUF_CFG) & 0xFFA200FF) | 0x590100;
#endif
	npu_printf("[%s] PPE_WIFI_BUF_CFG=%x, value=%x\n",
		   "tdma_tx_init", TDMA_WIFI_BUF_CFG, REG32(TDMA_WIFI_BUF_CFG));

#ifdef HAS_BME
	tdma_bme_init();
#endif
}

static u32 *tdma_stats_base(u32 band)
{
	if (band == 0)
		return (u32 *)counter_base_2g;
	if (band == 1)
		return (u32 *)counter_base_5g;
	return (u32 *)counter_base_tri;
}

static int __attribute__((noinline)) tdma_tx_submit(u32 port, u32 pkt_len,
						    u32 buf_addr, u32 band)
{
	u32 sw_idx = tdma_tx_sw_idx[band + 3];
	volatile u32 *hw_idx_reg = (volatile u32 *)(0x1FB5080C + 16 * band);
	u32 retries = 5;
	u32 hw_idx, free_slots;
	u32 *desc;
	u32 next;

	while (1) {
		hw_idx = *hw_idx_reg;
		if (sw_idx < hw_idx)
			free_slots = hw_idx - sw_idx - 1;
		else
			free_slots = 1023 - sw_idx + hw_idx;
		if (free_slots > 9)
			break;
		{ volatile u32 i; for (i = 0; i < 300; i++) ; }
		if ((wifi_debug_flags & 4))
			(*(tdma_stats_base(band) + 63))++;
		if (--retries == 0) {
			if ((wifi_debug_flags & 4))
				(*(tdma_stats_base(band) + 64))++;
			return -1;
		}
	}

	desc = (u32 *)(tdma_tx_ring_base[band] + 32 * sw_idx);
	if ((s32)desc[1] >= 0) {
		if ((wifi_debug_flags & 4))
			(*(tdma_stats_base(band) + 63))++;
		npu_printf("tdma tx (%d) full. cpu %d desc word %x\n",
			   band, sw_idx, desc[1]);
		return -1;
	}

	if ((wifi_debug_flags & 4))
		(*(tdma_stats_base(band) + 79))++;

	if (pkt_len < 60) {
		npu_memset((void *)(buf_addr + pkt_len), 0, 60 - pkt_len);
		pkt_len = 60;
	}

	next = (sw_idx <= 0x3FE) ? sw_idx + 1 : 0;
	desc[4] = (port << 14) | 0x80000000;
	desc[2] = (buf_addr & 0x3FFFFFFF) | 0x80000000;
	desc[1] = pkt_len;
	REG32(0x1FB50808 + 16 * band) = next;
	tdma_tx_sw_idx[band + 3] = next;
	return 0;
}
#endif /* HAS_BME */

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

/* ================================================================
 * Host adaptor (DMA interface to host ARM)
 * ================================================================ */

#define HOSTADPT_TX_DMA_PTR   0x1EC0D180
#define HOSTADPT_RX_DMA_PTR   0x1EC0D190

static u32 hostadpt_tx_ring_base;
static u8  hostadpt_tx_ring_ready;
static u32 hostadpt_rx_ring_base;

int hostadpt_init(void)
{
	/* wait for host to configure RX DMA pointer */
	while (REG32(HOSTADPT_RX_DMA_PTR) == 0)
		;

	hostadpt_tx_ring_base = (REG32(HOSTADPT_TX_DMA_PTR) & 0x3FFFFFFF) | 0x40000000;
	hostadpt_tx_ring_ready = 1;
	hostadpt_rx_ring_base = (REG32(HOSTADPT_RX_DMA_PTR) & 0x3FFFFFFF) | 0x40000000;
	return 0;
}

#ifdef WIFI_KITE

#define HOSTADPT_BUFFER_LEN  3500
#define HOSTADPT_RING_SIZE   512

static void host_ring_write_desc(u32 desc_addr, u32 buf_addr, u16 pkt_len,
				 u16 wcid, u8 amsdu, u8 fwd_type,
				 u16 orig_len, u8 is_last, u8 classify_result)
{
	u32 dma_len;
	u32 pkt_addr;

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

	pkt_addr = *(u8 *)(desc_addr + 12) |
		   ((u32)*(u8 *)(desc_addr + 13) << 8) |
		   ((u32)*(u8 *)(desc_addr + 14) << 16) |
		   ((u32)*(u8 *)(desc_addr + 15) << 24);

	bridge_dma_copy(3, buf_addr, pkt_addr, dma_len);

	*(u32 *)(desc_addr + 8) = classify_result;
	*(u32 *)(desc_addr + 4) = (wcid & 0xFFFF) |
				   ((amsdu & 0x1F) << 16) |
				   ((fwd_type & 0x3F) << 26);
	*(u32 *)desc_addr = 1 |
			    ((orig_len & 0x3FFF) << 1) |
			    ((pkt_len & 0x3FFF) << 15) |
			    ((is_last & 1) << 29);

	if ((wifi_debug_flags & 4) && counter_base_tri)
		(*(u32 *)(counter_base_tri + 0x38))++;
}

static int host_ring_submit(u32 buf_addr, u16 pkt_len, u32 band,
			    u16 wcid, u8 amsdu, u8 fwd_type,
			    u16 orig_len, u8 is_last, u8 classify_result)
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
				     amsdu, fwd_type, orig_len, is_last,
				     classify_result);
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
				     amsdu, fwd_type, orig_len, is_last,
				     classify_result);
		idx++;
		REG32(HOSTADPT_RX_DMA_IDX(1)) = (idx < HOSTADPT_RING_SIZE) ? idx : 0;
		return 0;

	} else {
		npu_printf("Ring Index error!\n");
		return -1;
	}
}

#endif /* WIFI_KITE */

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
static u16 rxnode_widx_2g;
static u16 rxnode_widx_5g;
static u16 pinode_widx_2g;
static u16 pinode_widx_5g;

/* per-band ring read indices (drain side, core3) */
static u16 rxnode_ridx_2g;
static u16 rxnode_ridx_5g;
static u16 pinode_ridx_2g;
static u16 pinode_ridx_5g;
static u16 rxnode_retry_limit;

/* per-band per-queue stats: 16 queues x {u64 bytes, u64 pkts} */
static u32 stats_bytes_2g[32];
static u32 stats_pkts_2g[32];
static u32 stats_bytes_5g[32];
static u32 stats_pkts_5g[32];

static u32 fwd_mutex[2];

static int pkt_forward(u32 buf_id, u32 pkt_len, s16 wcid, u8 amsdu,
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

static void pinode_drain(u32 band)
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

static void rxnode_drain(u32 band)
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

/* ================================================================
 * Reorder node management
 *
 * Two pools: pri (2000 nodes) and sec (5000 nodes), 36 bytes each.
 * Free-list managed via circular index buffers with mutex.
 * ================================================================ */

#define REORDER_PRI_POOL_SIZE  2000
#define REORDER_SEC_POOL_SIZE  5000
#define REORDER_NODE_SIZE      36

static void reorder_node_free(u16 node_idx, u8 node_type, u32 band)
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

static u32 reorder_node_alloc(u32 band, u32 *pool_type, u16 *idx_out)
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

static int pkt_enqueue_bridge(u32 buf_id, u32 pkt_len, u32 amsdu,
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

static void ba_flush_entry(u32 *entry)
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

static void ba_indicate_le_seq(u32 *entry, u32 seq)
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

static u32 ba_seq_scan(u32 *entry, u32 seq)
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

static u32 ba_state_update(u32 sn, u32 check_type, u32 entry_addr)
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

static u32 ba_scan_entries(u32 band)
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

static void ba_flush_all(u32 band)
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

/* ================================================================
 * WiFi RXD/TXD initialization
 * ================================================================ */

static void __attribute__((noinline)) npu_set_pcie_base(u32 addr, u32 band)
{
	npu_printf("[NPU] %s, addr=%lx, band_idx=%d \n",
		   "npu_set_pcie_base", addr, band);

	if (band != 0) {
		pcie_base_5g = addr;
		*(u32 *)(addr + 8) = 1535;
		npu_printf("[%s] 5G pcieBase %lx mem_base %lx\n",
			   "npu_set_pcie_base", addr, pcie_base_5g);
	} else {
		pcie_base_2g = addr;
		*(u32 *)(addr + 8) = 1535;
		npu_printf("[%s] 2.4G pcieBase %lx mem_base %lx\n",
			   "npu_set_pcie_base", addr, pcie_base_2g);
	}
}

static int wifi_init_rxd_5g(u32 ring_size, u32 band)
{
	u32 i;
	u32 desc_addr;
	u32 new_buf_id;
	u16 old_val;

	npu_printf("[NPU] Enter %s, band_idx=%d, rx_ring_size=%lu\n",
		   "npu_npu_init_rxd_5G", band, ring_size);

	if (ring_size - 1 > 0x5FF)
		npu_printf("ERROR! rx_ring_size = %lu \n", ring_size);

	if (rxd_5g_init_done != 0) {
		npu_printf("5G init second: do np_skb free\n");
		rxd_5g_init_done = 0;
		if (ring_size != 0) {
			for (i = 0; i < ring_size; i++)
				buf_id_free(0, band,
					    (u32)rxd_5g_bufid_table[i]);
		}
	}

	if (ring_size == 0) {
		rxd_5g_cpu_idx = 0;
		rxd_5g_mirror = wifi_base_cfg_val;
		rxd_5g_init_done = 1;
		return 0;
	}

	for (i = 0; i < ring_size; i++) {
		desc_addr = rxd_base_5g + i * 16;
		new_buf_id = buf_id_alloc_hw(0, band);

		if (new_buf_id == (u32)-1) {
			npu_printf("[%s] rxd init: alloc buffid fail!\n",
				   (band != 0) ? "5G" : "2.4G");
			return 1;
		}

		rxd_5g_bufid_table[i] = (u16)new_buf_id;
		old_val = *(u16 *)(desc_addr + 6);
		*(u32 *)desc_addr =
			(((new_buf_id << 12) + wifi_buf_id_base) &
			 0x3FFFFFFF) | 0x80000000;
		*(u16 *)(desc_addr + 6) = (old_val & 0x4000) | 0xDAC;
	}

	rxd_5g_cpu_idx = 0;
	rxd_5g_mirror = wifi_base_cfg_val;
	rxd_5g_init_done = 1;
	return 0;
}

static int wifi_init_rxd_2g(u32 ring_size, u32 band)
{
	u32 i;
	u32 desc_addr;
	u32 new_buf_id;
	u16 old_val;

	npu_printf("[NPU] Enter(2.4G) %s, band_idx=%d, rx_ring_size=%lu\n",
		   "npu_npu_init_rxd", band, ring_size);

	if (ring_size - 1 > 0x5FF)
		npu_printf("ERROR! rx_ring_size = %lu\n", ring_size);

	if (rxd_2g_init_done != 0 && rxd_2g_bufid_base != 0) {
		rxd_2g_init_done = 0;
		if (ring_size != 0) {
			for (i = 0; i < ring_size; i++)
				buf_id_free(0, band,
					    (u32)*(u16 *)(rxd_2g_bufid_base +
							  2 * i));
		}
	}

	if (ring_size == 0) {
		rxd_2g_cpu_idx = 0;
		rxd_2g_mirror = wifi_base_cfg_val;
		rxd_2g_init_done = 1;
		return 0;
	}

	for (i = 0; i < ring_size; i++) {
		desc_addr = rxd_base_2g + i * 16;
		new_buf_id = buf_id_alloc_hw(0, band);

		if (new_buf_id == (u32)-1) {
			npu_printf("!!!!!!!!!!!! [%s] rxd init: alloc buffid fail!\n",
				   (band != 0) ? "5G" : "2.4G");
			return 1;
		}

		*(u16 *)(rxd_2g_bufid_base + 2 * i) = (u16)new_buf_id;
		old_val = *(u16 *)(desc_addr + 6);
		*(u32 *)desc_addr =
			(((new_buf_id << 12) + wifi_buf_id_base) &
			 0x3FFFFFFF) | 0x80000000;
		*(u16 *)(desc_addr + 6) = (old_val & 0x4000) | 0xDAC;
	}

	rxd_2g_cpu_idx = 0;
	rxd_2g_mirror = wifi_base_cfg_val;
	rxd_2g_init_done = 1;
	return 0;
}

static void __attribute__((noinline)) wifi_reset_ba_entry(u32 dir, u32 wcid)
{
	u32 i, entry_addr;

	if (wcid == 0) {
		npu_printf("ERROR!!!!! npu_reset_ba_entry() [%s]wcid == 0\n",
			   (dir != 1) ? "2.4G" : "5G");
		return;
	}

	for (i = 0; i < 8; i++) {
		if (wifi_dbdc_mode != 0) {
			if (wcid > 150)
				entry_addr = ba_table_a +
					     224 * (wcid - 151) + i * 28;
			else
				entry_addr = ba_table_b +
					     224 * (wcid - 1) + i * 28;
		} else {
			if (dir != 0)
				entry_addr = ba_table_b +
					     224 * (wcid - 1) + i * 28;
			else
				entry_addr = ba_table_a +
					     224 * (wcid - 1) + i * 28;
		}

		ba_flush_entry((u32 *)entry_addr);

		if (dir != 0 || wifi_dbdc_mode)
			hw_mutex_lock(ba_mutex_5g);
		else
			hw_mutex_lock(ba_mutex_2g);

		*(u8 *)(entry_addr + 25) = (u8)dir;
		*(u32 *)(entry_addr + 16) = 8;
		*(u32 *)(entry_addr + 20) = 0;
		*(u8 *)(entry_addr + 24) = 0;

		if (dir != 0 || wifi_dbdc_mode)
			hw_mutex_unlock(ba_mutex_5g);
		else
			hw_mutex_unlock(ba_mutex_2g);
	}
}

/* ================================================================
 * WiFi subsystem
 * ================================================================ */

#ifdef WIFI_KITE
static int wifi_get_chip_name(u32 idx, char *name)
{
	if (idx >= 5)
		return -1;
	npu_memcpy(name, wifi_chip_names[idx], 8);
	return 0;
}
#endif

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
static u32 wifi_rx_ring_base_2g;
static u32 wifi_tx_ring_base_5g;
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
static u32 wifi_pipeline_base;

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
	new_buf_id = buf_id_alloc_hw(1, 0);

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

/* WiFi queue mutex init: assigns HW mutex indices to queue/BA mutexes */
static void wifi_queue_mutex_init(void)
{
	npu_printf("[NPU] %s...\n", "queue_mutex_init");
	queue_mutex_2g[0] = 10;
	queue_mutex_2g[1] = 0;
	queue_mutex_5g[0] = 11;
	queue_mutex_5g[1] = 0;
	ba_mutex_5g[0] = 5;
	ba_mutex_5g[1] = 0;
	ba_mutex_2g[0] = 6;
	ba_mutex_2g[1] = 0;
}

/* WiFi pkt queue init: allocate and zero per-band packet queues */
static void wifi_pkt_queue_init(u32 band)
{
	u32 i, count, base, rx_base;

	if (band != 0) {
		npu_printf("[NPU]%s  %s...\n", "5G", "pkt_queue_init");
		if (band == 1) {
			pkt_queue_widx_5g = 0;
			pinode_widx_5g = 0;
			pkt_queue_base_5g = sram_buf_alloc(2);
			pkt_queue_rx_widx_2g = 0;
			pkt_queue_rx_ridx_2g = 0;
			pkt_queue_rx_base_5g = sram_buf_alloc(14);
			count = 512;
		} else {
			count = 256;
		}
	} else {
		npu_printf("[NPU]%s  %s...\n", "2.4", "pkt_queue_init");
		pinode_widx_2g = 0;
		pkt_queue_widx_2g = 0;
		pkt_queue_base_2g = sram_buf_alloc(3);
		rxnode_widx_2g = 0;
		rxnode_widx_5g = 0;
		pkt_queue_rx_base_2g = sram_buf_alloc(15);
		count = 256;
	}

	for (i = 0; i < count; i++) {
		if (band != 0)
			base = pkt_queue_base_5g + i * 16;
		else
			base = pkt_queue_base_2g + i * 16;
		*(u32 *)base = 0xFFFFFFFF;
		*(u32 *)(base + 4) = 0;
		*(u16 *)(base + 8) = 0;
		*(u8 *)(base + 11) = 0;
		*(u8 *)(base + 10) = 0;
	}

	for (i = 0; i < 128; i++) {
		if (band != 0)
			rx_base = pkt_queue_rx_base_5g + i * 12;
		else
			rx_base = pkt_queue_rx_base_2g + i * 12;
		*(u32 *)rx_base = 0xFFFFFFFF;
		*(u32 *)(rx_base + 4) = 0;
		*(u8 *)(rx_base + 8) = 0;
	}
}

/* WiFi BA node init: allocate reorder node pools */
static void wifi_ba_node_init(void)
{
	u16 *pool;
	u32 i;

	npu_printf("[NPU] %s...\n", "ba_node_init");
	ba_node_pool_base = sram_buf_alloc(4);

	npu_printf("%s\n", "baNode_array_init");
	reorder_alloc_mutex[0] = 8;
	reorder_alloc_mutex[1] = 0;
	reorder_free_mutex[0] = 9;
	reorder_free_mutex[1] = 0;

	pool = (u16 *)sram_buf_alloc(12);
	reorder_pri_idx_pool = (u32)pool;
	for (i = 0; i < REORDER_PRI_POOL_SIZE; i++)
		pool[i] = (u16)i;
	reorder_pri_widx = 0;
	reorder_pri_ridx = 0;

	pool = (u16 *)sram_buf_alloc(13);
	reorder_sec_idx_pool = (u32)pool;
	for (i = 0; i < REORDER_SEC_POOL_SIZE; i++)
		pool[i] = (u16)i;
	reorder_sec_widx = 0;
	reorder_sec_ridx = 0;
}

/* WiFi PCIe desc alloc: allocate PCIe descriptor ring buffer */
void wifi_pcie_desc_alloc(void)
{
	wifi_pcie_desc_base = sram_buf_alloc(1);
	npu_printf("PCIE_TOTAL_DESC_BASE=%x\n", wifi_pcie_desc_base);
}

/* WiFi state init: clear per-band ring state and byte/pkt stats */
static void wifi_state_init(void)
{
#ifdef HAS_WIFI
	u32 i;

	wifi_bridge_report = 0;
	wifi_bridge_enabled = 1;
	wifi_retry_limit = 3;

	/* clear 5G band stats */
	for (i = 0; i < 16; i++) {
		stats_bytes_5g[i * 2] = 0;
		stats_bytes_5g[i * 2 + 1] = 0;
		stats_pkts_5g[i * 2] = 0;
		stats_pkts_5g[i * 2 + 1] = 0;
	}

	/* clear 2.4G band stats */
	for (i = 0; i < 16; i++) {
		stats_bytes_2g[i * 2] = 0;
		stats_bytes_2g[i * 2 + 1] = 0;
		stats_pkts_2g[i * 2] = 0;
		stats_pkts_2g[i * 2 + 1] = 0;
	}

	/* allocate tri-band counters */
	counter_init(2);

	wifi_rx_pending = 0;
	wifi_tx_pending = 0;
#endif
}

/* WiFi bridge init: state init and ring setup */
static void wifi_bridge_init(void)
{
#ifdef HAS_WIFI
	wifi_queue_mutex_init();
	bufid_ring_mutex_init();
	wifi_pkt_queue_init(0);
	wifi_pkt_queue_init(1);
	wifi_ba_node_init();

	wifi_state_init();

	/* allocate per-band counter and WCID buffers */
	counter_init(0);
	counter_init(1);
	wcid_counter_init(0);
	wcid_counter_init(1);

	wifi_bridge_active = 0;
	wifi_batch_count = 0;
	wifi_pipeline_widx = 0;

#endif
}

/* WiFi PCIe BAR descriptor offset: type 1=base, type 2=base+0x6020 */
static u32 wifi_pcie_desc_offset(u32 base, u32 type)
{
	if (type == 2)
		return base + 0x6020;
	if (type != 1) {
		npu_printf("not support type[wificase:%d,wifisubcase:%d]\n",
			   1, type);
		return 0;
	}
	return base;
}

/* WiFi get DBDC mode from driver model */
static u8 wifi_get_dbdc_mode(u8 model)
{
	if (model == 1 || model == 2) {
		npu_printf("DriverModel(%d) %s Support DBDC\n", model, "");
		return 1;
	}
	npu_printf("DriverModel(%d) %s Support DBDC\n", model, "Not");
	return 0;
}

/* WiFi get band capability from chip variant registers */
static u8 wifi_get_band_cap(u8 model)
{
	u32 chip_rev = REG32(CHIP_ID_REG) >> 16;
	u32 variant;

	if (model != 0) {
		if (chip_rev == 15) {
			variant = (REG32(CHIP_VARIANT_REG) & 0xF) |
				  ((REG32(CHIP_VARIANT_REG) >> 3) & 0x10);
			if (variant == 1) {
				npu_printf("Chip id(%x) does not support "
					   "NPU Wifi Offload!!!\n",
					   REG32(CHIP_VARIANT_REG));
				return 0;
			}
		}
		npu_printf("Support NPU Wifi Offload\n");
		return 1;
	}
	if (chip_rev == 12) {
		variant = (REG32(CHIP_VARIANT_REG) & 0xF) |
			  ((REG32(CHIP_VARIANT_REG) >> 3) & 0x10);
		if (variant == 0) {
			npu_printf("Chip id(%x) does not support "
				   "NPU Wifi Offload!!!\n",
				   REG32(CHIP_VARIANT_REG));
			return 0;
		}
	}
	npu_printf("Support NPU Wifi Offload\n");
	return 1;
}

/* WiFi NPU init: main WiFi subsystem init called from mailbox */
static void __attribute__((noinline)) wifi_npu_init(u32 dbdc)
{
#ifdef HAS_WIFI
	u32 desc_type1, desc_type2;
	u32 bar_5g, bar_2g;

	npu_printf("[NPU] %s...\n", "npu_init");
	npu_printf("=======================\n");
#ifdef WIFI_KITE
	npu_printf("NPU Version: %s_NPU_%s\n",
		   wifi_chip_names[wifi_driver_model], NPU_INIT_VERSION);
#else
	npu_printf("NPU init Version: %s\n", NPU_INIT_VERSION);
#endif
	npu_printf("=======================\n");

	wifi_band_cap = wifi_get_band_cap(wifi_driver_model);
	wifi_dbdc_mode = wifi_get_dbdc_mode(wifi_driver_model);

	desc_type2 = wifi_pcie_desc_offset(wifi_pcie_desc_base, 2);
	desc_type1 = wifi_pcie_desc_offset(wifi_pcie_desc_base, 1);
	bar_5g = desc_type2 + 0x6020;
	bar_2g = desc_type1 + 0x6020;

	switch (wifi_pcie_port_type) {
	case 1:
		REG32(0x1FA90038) = desc_type1 & 0x1FFFFFFF;
		REG32(0x1FA9003C) = bar_5g & 0x1FFFFFFF;
		break;
	case 0:
		REG32(PCIE0_MAC_BASE + 0x8030) = desc_type1 & 0x1FFFFFFF;
		REG32(PCIE0_MAC_BASE + 0x8034) = bar_5g & 0x1FFFFFFF;
		if (dbdc != 0)
			goto alloc_5g;
		goto alloc_2g;
	case 2:
		if (dbdc != 0) {
			REG32(0x1FA90038) = desc_type1 & 0x1FFFFFFF;
			REG32(0x1FA9003C) = bar_2g & 0x1FFFFFFF;
			goto alloc_5g;
		}
		REG32(PCIE0_MAC_BASE + 0x8030) = desc_type2 & 0x1FFFFFFF;
		REG32(PCIE0_MAC_BASE + 0x8034) = bar_5g & 0x1FFFFFFF;
		goto alloc_2g;
	case 3:
		if (dbdc == 0) {
			REG32(0x1FA90038) = desc_type2 & 0x1FFFFFFF;
			REG32(0x1FA9003C) = bar_5g & 0x1FFFFFFF;
			goto alloc_2g;
		}
		REG32(PCIE0_MAC_BASE + 0x8030) = desc_type1 & 0x1FFFFFFF;
		REG32(PCIE0_MAC_BASE + 0x8034) = bar_2g & 0x1FFFFFFF;
		goto alloc_5g;
	default:
		break;
	}
	if (dbdc != 0)
		goto alloc_5g;

alloc_2g:
	ba_table_a = sram_buf_alloc(8);
	pinode_widx_5g = 0;
	pkt_queue_rx_base_2g = sram_buf_alloc(6);
	wifi_pipeline_queue_2g =
		wifi_pcie_desc_offset(wifi_pcie_desc_base, 2);
	goto pipeline_init;

alloc_5g:
	ba_table_b = sram_buf_alloc(7);
	pinode_widx_2g = 0;
	wifi_pipeline_queue_5g =
		wifi_pcie_desc_offset(wifi_pcie_desc_base, 1);

pipeline_init:
	npu_printf("[NPU]  %s...\n", "pipeline_pkt_queue_init");
	{
		u32 *p;
		u32 end;

		wifi_pipeline_base = sram_buf_alloc(21);
		p = (u32 *)wifi_pipeline_base;
		end = wifi_pipeline_base + 25600;
		while ((u32)p < end) {
			*p = 0xFFFFFFFF;
			*(u16 *)(p + 1) = 0;
			p = (u32 *)((u8 *)p + 8);
		}
	}

	counter_init(dbdc);
	wcid_counter_init(dbdc);
#endif
}

/* WiFi mailbox setters: simple parameter setters called from host.
 * noinline: blob has these as separate callees, not inlined into handlers. */
static void __attribute__((noinline)) npu_set_retry_limit(u32 val)
{
	wifi_retry_limit = (u16)val;
	npu_printf("enq_error_retry_times = %d !!!\n", val);
}

static void __attribute__((noinline)) npu_set_pcie_port_type(u32 val)
{
	wifi_pcie_port_type = (u8)val;
	npu_printf("PCIe_Port_Type = %d !!!\n", val);
}

static void __attribute__((noinline)) npu_set_band_enable(u32 band)
{
	if (band > 1) {
		npu_printf("[ERROR] band_idx is wrong value %d !!!\n", band);
		return;
	}
	*((u8 *)&pipeline_5g_ready + band) = 1;
}

static void __attribute__((noinline)) npu_set_force_to_cpu(u8 val)
{
	wifi_force_to_cpu = val;
	npu_printf("isForceToCpu=%s\n", val ? "true" : "false");
	if (wifi_force_to_cpu > 1)
		npu_printf("[ERROR] isForceToCpu is wrong value !!!\n");
}

static void __attribute__((noinline)) npu_set_flushall_timeout(u32 val)
{
	wifi_flushall_timeout = (u16)val;
	npu_printf("flushall_timeout=%d\n", val);
}

static void __attribute__((noinline)) npu_set_flushone_timeout(u32 val)
{
	wifi_flushone_timeout = (u16)val;
	npu_printf("flushone_timeout=%d\n", val);
}

static void __attribute__((noinline)) npu_set_no_ba_test(u8 val)
{
	wifi_no_ba_test = val;
	npu_printf("isforTestNoBA=%s\n", val ? "true" : "false");
	if (wifi_no_ba_test > 1)
		npu_printf("[ERROR] isforTestNoBA is wrong value !!!\n");
}

static void __attribute__((noinline)) npu_set_fast_flag(u8 val)
{
	wifi_debug_flags = val;
	npu_printf("npu_wifi_fast_flag = %d !!!\n", val);
}

static void __attribute__((noinline)) npu_set_pkt_buf_addr(u32 val)
{
	wifi_pkt_buf_addr = val;
	npu_printf("pkt_buf_addr=%lx\n", val);
}

static void __attribute__((noinline)) npu_set_dram_ba_node_addr(u32 val)
{
	wifi_dram_ba_node_addr = (val & 0x3FFFFFFF) | NPU_ADDR_MASK;
	npu_printf("dramBaNodeAddr=%x\n", val);
}

static void __attribute__((noinline)) npu_set_driver_model(u32 val)
{
	wifi_driver_model = (u8)val;
	npu_printf("driverModel=%d\n", val);
}

static void __attribute__((noinline)) npu_set_band0_on_cpu(u32 val)
{
	wifi_band0_on_cpu = (u8)val;
	npu_printf("npu_band0_on_cpu_support=%s\n",
		   val ? "true" : "false");
}

static void __attribute__((noinline)) npu_set_bar_info(u32 band, u32 packed)
{
	u32 tid = packed & 7;
	u32 ssn = (packed << 8) >> 19;
	u32 wcid = (u8)(packed >> 3);
	u32 entry_addr;

	if (wcid == 0) {
		if ((wifi_debug_flags & 4) != 0) {
			u32 *cnt;

			if (band == 1)
				cnt = (u32 *)(counter_base_5g + 268);
			else if (band != 0)
				cnt = (u32 *)(counter_base_tri + 268);
			else
				cnt = (u32 *)(counter_base_2g + 268);
			(*cnt)++;
		}
		npu_printf("ERROR!!!!! npu_set_bar_info() [%s]wcid == 0\n",
			   (band != 1) ? "2.4G" : "5G");
		return;
	}

	if (wifi_dbdc_mode != 0) {
		if (wcid == 0) {
			npu_printf("ERROR!!!!! [GET_BA_ENTRY_DBDC_]wcid == 0\n");
			return;
		}
		if (wcid > 150)
			entry_addr = ba_table_a +
				     28 * (8 * (wcid - 151) + tid);
		else
			entry_addr = ba_table_b +
				     28 * (8 * (wcid - 1) + tid);
	} else {
		u32 off = 28 * (8 * (wcid - 1) + tid);

		if (band != 0)
			entry_addr = ba_table_b + off;
		else
			entry_addr = ba_table_a + off;
	}

	ba_state_update(ssn & 0xFFFF, 3, entry_addr);

	if ((*(u16 *)(entry_addr + 18) - ssn) & 0x8000) {
		u16 new_ssn = (ssn != 0) ? ssn - 1 : 4095;

		ba_indicate_le_seq((u32 *)entry_addr, new_ssn);

		if (band == 0) {
			if (wifi_dbdc_mode != 0)
				hw_mutex_lock(ba_mutex_5g);
			else
				hw_mutex_lock(ba_mutex_2g);
			*(u16 *)(entry_addr + 18) = new_ssn;
			if (wifi_dbdc_mode != 0)
				hw_mutex_unlock(ba_mutex_5g);
			else
				hw_mutex_unlock(ba_mutex_2g);
		} else {
			hw_mutex_lock(ba_mutex_5g);
			*(u16 *)(entry_addr + 18) = new_ssn;
			hw_mutex_unlock(ba_mutex_5g);
		}

		{
			u16 scan_result = ba_seq_scan(
				(u32 *)entry_addr,
				*(u16 *)(entry_addr + 18));

			if (scan_result != 0xFFFF) {
				if (band == 0) {
					if (wifi_dbdc_mode != 0)
						hw_mutex_lock(ba_mutex_5g);
					else
						hw_mutex_lock(ba_mutex_2g);
				} else {
					hw_mutex_lock(ba_mutex_5g);
				}
				*(u16 *)(entry_addr + 18) = scan_result;
				if (band == 0) {
					if (wifi_dbdc_mode != 0)
						hw_mutex_unlock(ba_mutex_5g);
					else
						hw_mutex_unlock(ba_mutex_2g);
				} else {
					hw_mutex_unlock(ba_mutex_5g);
				}
			}
		}
	}
}

static void __attribute__((noinline)) npu_set_ba_entry(u32 band, u32 packed)
{
	u32 tid = packed & 7;
	u32 win_size = packed >> 20;
	u32 ssn = (packed >> 11) & 0x1FF;
	u32 wcid = (u8)(packed >> 3);
	u32 entry_addr;

	if (wcid == 0) {
		npu_printf("ERROR!!!!! npu_set_ba_entry() [%s]wcid == 0\n",
			   (band != 1) ? "2.4G" : "5G");
		return;
	}

	if (band != 0) {
		if (wifi_dbdc_mode != 0) {
			if (wcid == 0) {
				npu_printf("ERROR!!!!! "
					   "[GET_BA_ENTRY_DBDC_]wcid == 0\n");
				entry_addr = 0;
			} else if (wcid > 150) {
				entry_addr = ba_table_a +
					     28 * (8 * (wcid - 151) + tid);
			} else {
				entry_addr = ba_table_b +
					     28 * (8 * (wcid - 1) + tid);
			}
		} else {
			entry_addr = ba_table_b +
				     28 * (8 * (wcid - 1) + tid);
		}

		ba_flush_entry((u32 *)entry_addr);
		hw_mutex_lock(ba_mutex_5g);

		if (win_size == 0) {
			*(u8 *)(entry_addr + 25) = (u8)band;
			*(u32 *)(entry_addr + 16) = 8;
			*(u32 *)(entry_addr + 20) = 0;
			*(u8 *)(entry_addr + 24) = 0;
		} else {
			*(u16 *)(entry_addr + 16) = (u16)win_size;
			*(u16 *)(entry_addr + 18) = (u16)ssn;
			*(u8 *)(entry_addr + 25) = (u8)band;
			*(u8 *)(entry_addr + 24) = 3;
		}

		hw_mutex_unlock(ba_mutex_5g);
	} else {
		if (wifi_dbdc_mode != 0) {
			if (wcid == 0) {
				npu_printf("ERROR!!!!! "
					   "[GET_BA_ENTRY_DBDC_]wcid == 0\n");
				entry_addr = 0;
			} else if (wcid > 150) {
				entry_addr = ba_table_a +
					     28 * (8 * (wcid - 151) + tid);
			} else {
				entry_addr = ba_table_b +
					     28 * (8 * (wcid - 1) + tid);
			}
		} else {
			entry_addr = ba_table_a +
				     28 * (8 * (wcid - 1) + tid);
		}

		ba_flush_entry((u32 *)entry_addr);

		if (wifi_dbdc_mode != 0)
			hw_mutex_lock(ba_mutex_5g);
		else
			hw_mutex_lock(ba_mutex_2g);

		if (win_size == 0) {
			*(u32 *)(entry_addr + 16) = 8;
			*(u32 *)(entry_addr + 20) = 0;
			*(u16 *)(entry_addr + 24) = 0;
		} else {
			*(u16 *)(entry_addr + 16) = (u16)win_size;
			*(u16 *)(entry_addr + 18) = (u16)ssn;
			*(u16 *)(entry_addr + 24) = 3;
		}

		if (wifi_dbdc_mode != 0)
			hw_mutex_unlock(ba_mutex_5g);
		else
			hw_mutex_unlock(ba_mutex_2g);
	}
}

static void __attribute__((noinline)) npu_set_wait_state(u32 port, u8 state)
{
	u32 i;

	if (port <= 15) {
		wifi_wait_state_2g[port] = state;
		wifi_band0_on_cpu = state;
	} else {
		wifi_wait_state_5g[port - 16] = state;
		wifi_force_to_cpu = state;
	}

	for (i = 0; i < 16; i++) {
		if (wifi_wait_state_2g[i] != 0 ||
		    wifi_wait_state_5g[i] != 0) {
			wifi_force_to_cpu = 1;
			return;
		}
	}
	wifi_force_to_cpu = 0;
	npu_printf("set wait!  not Force to CPU \n");
}

static void __attribute__((noinline)) npu_set_rxd_init(u32 ring_size, u32 band)
{
	if (band == 1)
		wifi_init_rxd_5g(ring_size, 1);
	else
		wifi_init_rxd_2g(ring_size, 0);
}

/* WiFi get pipeline queue base for a given band */
static u32 npu_get_pipeline_queue(u32 band)
{
	if (band == 1)
		return wifi_pipeline_queue_5g & 0x1FFFFFFF;
	return wifi_pipeline_queue_2g & 0x1FFFFFFF;
}

/* ================================================================
 * WiFi mailbox command wrappers (kite path)
 *
 * Kite (MT7916/MT7996) host-side mailbox command handlers. Eagle uses
 * the same wrapper shape but a different helper behind every command -
 * see the eagle table in the blob, where funcId 0 reaches
 * npu_set_pcie_base_eagle rather than npu_set_pcie_base. None of the
 * eagle helpers are reconstructed yet, so an eagle build leaves both
 * dispatch tables empty and every command returns without acting.
 * ================================================================ */

#ifdef WIFI_KITE

static void npu_mbox_txrx_ring_size_get(u32 dir, u32 band, u32 *size)
{
	npu_printf("%s L%d not support on 791X\n",
		   "npu_mbox_txrx_ring_ring_size_get_wrapper", 3995);
	(void)dir; (void)band; (void)size;
}

static u32 npu_mbox_txrx_ring_dma_addr_get(u32 dir, u32 band)
{
	npu_printf("%s L%d not support on 791X\n",
		   "npu_mbox_txrx_ring_dma_addr_get_wrapper", 4001);
	return 0;
}

static void npu_mbox_pcie_swap_set(u32 val)
{
	npu_printf("%s L%d not support on 791X\n",
		   "npu_mbox_pcie_swap_set_wrapper", 4007);
	(void)val;
}

static void npu_mbox_stop_set(u32 band)
{
	npu_printf("%s L%d not support on 791X\n",
		   "npu_mbox_stop_set_wrapper", 4013);
	(void)band;
}

static void npu_mbox_rx_hw_cfg_set(u32 band, u32 a, u32 b)
{
	npu_printf("%s L%d not support on 791X\n",
		   "npu_mbox_rx_hw_cfg_set_wrapper", 4019);
	(void)band; (void)a; (void)b;
}

static void npu_mbox_set_debug_flag(u32 band, u32 flag)
{
	npu_printf("%s L%d not support on 791X\n",
		   "npu_mbox_set_debug_flag_wrapper", 4025);
	(void)band; (void)flag;
}

static void npu_mbox_set_txrx_reg_addr(u32 band, u32 a, u32 b, u32 c, u32 d)
{
	npu_printf("%s L%d not support on 791X\n",
		   "npu_mbox_set_wait_inode_txrx_reg_addr_wrapper", 4032);
	(void)band; (void)a; (void)b; (void)c; (void)d;
}

static u32 npu_mbox_get_rxdesc_base(u32 band)
{
	if (band == 1)
		return wifi_tx_ring_base_5g & 0x1FFFFFFF;
	else if (wifi_rx_ring_base_2g != 0)
		return wifi_rx_ring_base_2g & 0x1FFFFFFF;
	npu_printf("%s: not support\n", "wifi_mail_get_wait_rxdesc_base");
	return 1111;
}

static u32 wcid_counter_base_get(u32 band)
{
	return sram_buf_alloc((band != 1) + 19) & 0x1FFFFFFF;
}

static u32 counter_base_get(u32 band)
{
	if (band == 1)
		return sram_buf_alloc(9) & 0x1FFFFFFF;
	return sram_buf_alloc((band != 0) + 10) & 0x1FFFFFFF;
}

static void npu_mbox_get_counter(u32 port, u64 *bytes_2g, u64 *pkts_2g,
				 u64 *bytes_5g, u64 *pkts_5g,
				 u8 *omac_2g, u8 *omac_5g)
{
	if (port > 15) {
		npu_printf("[ERROR]%s() invalid input value:%d \n",
			   "npu_mbox_get_wait_counter_wrapper", port);
		*bytes_2g = 0;
		*pkts_2g = 0;
		*bytes_5g = 0;
		*pkts_5g = 0;
		*omac_2g = 0;
		*omac_5g = 0;
		return;
	}

	*bytes_2g = ((u64 *)stats_bytes_2g)[port];
	*pkts_2g = ((u64 *)stats_pkts_2g)[port];
	*bytes_5g = ((u64 *)stats_bytes_5g)[port];
	*pkts_5g = ((u64 *)stats_pkts_5g)[port];
	*omac_2g = wifi_port_band_2g[port];
	*omac_5g = wifi_port_band_5g[port];
}

static void npu_mbox_get_counter_apcli(u32 *apcli_out, u32 *rx_bytes_dst,
					u32 *rx_pkts_dst)
{
	u32 i;

	apcli_out[0] = apcli_count_2g[0];
	apcli_out[1] = apcli_count_2g[1];
	apcli_out[2] = apcli_count_5g[0];
	apcli_out[3] = apcli_count_5g[1];
	apcli_out[4] = apcli_byte_count_2g[0];
	apcli_out[5] = apcli_byte_count_2g[1];
	apcli_out[6] = apcli_byte_count_5g[0];
	apcli_out[7] = apcli_byte_count_5g[1];

	for (i = 0; i < 128; i++) {
		/* band 0 */
		rx_bytes_dst[i * 2] = npu_rx_bytes_entry[0][i * 2];
		rx_bytes_dst[i * 2 + 1] = npu_rx_bytes_entry[0][i * 2 + 1];
		rx_pkts_dst[i * 2] = npu_rx_pkts_entry[0][i * 2];
		rx_pkts_dst[i * 2 + 1] = npu_rx_pkts_entry[0][i * 2 + 1];
		/* band 1 */
		rx_bytes_dst[256 + i * 2] = npu_rx_bytes_entry[1][i * 2];
		rx_bytes_dst[256 + i * 2 + 1] = npu_rx_bytes_entry[1][i * 2 + 1];
		rx_pkts_dst[256 + i * 2] = npu_rx_pkts_entry[1][i * 2];
		rx_pkts_dst[256 + i * 2 + 1] = npu_rx_pkts_entry[1][i * 2 + 1];
	}
}

static void wifi_print_stats_5g(void)
{
	u32 i;

	for (i = 0; i < 16; i++) {
		npu_printf("ReceivedPktCount5G[%d]=%lu\n",
			   i, stats_pkts_5g[i * 2], stats_pkts_5g[i * 2 + 1]);
		npu_printf("ReceivedByteCount5G[%d]=%llu(%lluMiB)\n",
			   i, stats_bytes_5g[i * 2], stats_bytes_5g[i * 2 + 1],
			   (stats_bytes_5g[i * 2 + 1] << 12) |
			   (stats_bytes_5g[i * 2] >> 20),
			   stats_bytes_5g[i * 2 + 1] >> 20);
		npu_printf("omacIdx5G[%d]=%u\n", i, wifi_port_band_5g[i]);
	}
}

static void wifi_print_stats_2g(void)
{
	u32 i;

	for (i = 0; i < 16; i++) {
		npu_printf("ReceivedPktCount2G[%d]=%lu\n",
			   i, stats_pkts_2g[i * 2], stats_pkts_2g[i * 2 + 1]);
		npu_printf("ReceivedByteCount2G[%d]=%llu(%lluMiB)\n",
			   i, stats_bytes_2g[i * 2], stats_bytes_2g[i * 2 + 1],
			   (stats_bytes_2g[i * 2 + 1] << 12) |
			   (stats_bytes_2g[i * 2] >> 20),
			   stats_bytes_2g[i * 2 + 1] >> 20);
		npu_printf("omacIdx2G[%d]=%u\n", i, wifi_port_band_2g[i]);
	}
}

static void npu_mbox_get_npu_info(void)
{
	npu_printf("%s L%d do nothing\n",
		   "npu_mbox_get_wait_npu_info_wrapper", 3794);
}

/* WiFi mail handler callbacks (registered in dispatch table) */
int wifi_mail_set_debug_flag(u32 *msg)
{
	u32 flag = msg[2];
	u32 band = msg[0] & 0xF;

	npu_mbox_set_debug_flag(band, flag);
	return 1;
}

int wifi_mail_set_wait_inode_cfg(u32 *msg)
{
	u32 a = ((u8 *)msg)[8];
	u32 b = ((u8 *)msg)[9];
	u32 band = msg[0] & 0xF;

	npu_mbox_rx_hw_cfg_set(band, a, b);
	return 1;
}

int wifi_mail_set_wait_inode_stop(u32 *msg)
{
	u32 band = msg[0] & 0xF;

	npu_mbox_stop_set(band);
	return 1;
}

int wifi_mail_set_pcie_swap(u32 *msg)
{
	u32 val = msg[2];

	npu_mbox_pcie_swap_set(val);
	return 1;
}

/* SET_WAIT handlers [0]-[23], [29]-[30] — indexed by SDK enum */

int wifi_mail_set_pcie_addr(u32 *msg)
{
	npu_set_pcie_base(msg[2], msg[0] & 0xF);
	return 1;
}

int wifi_mail_set_desc(u32 *msg)
{
	npu_set_rxd_init(msg[2], msg[0] & 0xF);
	return 1;
}

int wifi_mail_set_init_done(u32 *msg)
{
	(void)msg;
	return 1;
}

int wifi_mail_set_tran_to_cpu(u32 *msg)
{
	npu_printf("%s() interfaceID = %u \n",
		   "wifi_mail_set_wait_tran2cpu", msg[0] & 0xF);
	npu_set_wait_state(msg[0] & 0xF, (u8)msg[2]);
	return 1;
}

int wifi_mail_set_ba_win_size(u32 *msg)
{
	npu_set_ba_entry(msg[0] & 0xF, msg[2]);
	return 1;
}

int wifi_mail_set_driver_model_cmd(u32 *msg)
{
	npu_set_driver_model((u8)msg[2]);
	return 1;
}

int wifi_mail_set_del_sta(u32 *msg)
{
	wifi_reset_ba_entry(msg[0] & 0xF, msg[2]);
	return 1;
}

int wifi_mail_set_dram_ba_node(u32 *msg)
{
	npu_set_dram_ba_node_addr(msg[2]);
	return 1;
}

int wifi_mail_set_pkt_buf(u32 *msg)
{
	npu_set_pkt_buf_addr(msg[2]);
	return 1;
}

int wifi_mail_set_test_noba(u32 *msg)
{
	npu_set_no_ba_test((u8)msg[2]);
	return 1;
}

int wifi_mail_set_flushone(u32 *msg)
{
	npu_set_flushone_timeout((u16)msg[2]);
	return 1;
}

int wifi_mail_set_flushall(u32 *msg)
{
	npu_set_flushall_timeout((u16)msg[2]);
	return 1;
}

int wifi_mail_set_force_cpu(u32 *msg)
{
	npu_set_force_to_cpu((u8)msg[2]);
	return 1;
}

int wifi_mail_set_pcie_state(u32 *msg)
{
	npu_set_band_enable(msg[0] & 0xF);
	return 1;
}

int wifi_mail_set_port_type(u32 *msg)
{
	npu_set_pcie_port_type((u8)msg[2]);
	return 1;
}

int wifi_mail_set_retry(u32 *msg)
{
	npu_set_retry_limit((u16)msg[2]);
	return 1;
}

int wifi_mail_set_bar_info_cmd(u32 *msg)
{
	npu_set_bar_info(msg[0] & 0xF, msg[2]);
	return 1;
}

int wifi_mail_set_fast_flag_cmd(u32 *msg)
{
	npu_set_fast_flag((u8)msg[2]);
	return 1;
}

int wifi_mail_set_band0_cpu(u32 *msg)
{
	npu_set_band0_on_cpu((u8)msg[2]);
	return 1;
}

int wifi_mail_set_tx_ring_pcie(u32 *msg)
{
	(void)msg;
	return 1;
}

int wifi_mail_set_tx_desc_hw(u32 *msg)
{
	npu_printf("%s: [band_idx=%d] desc phy addr=%lx \n",
		   "wifi_mail_set_wait_tx_ring_desc_phy_addr",
		   msg[0] & 0xF, msg[2]);
	return 1;
}

int wifi_mail_set_tx_buf_hw(u32 *msg)
{
	(void)msg;
	return 1;
}

int wifi_mail_set_rx_txdone_hw(u32 *msg)
{
	(void)msg;
	return 1;
}

int wifi_mail_set_tx_pkt_buf(u32 *msg)
{
	(void)msg;
	return 1;
}

int wifi_mail_set_txrx_reg(u32 *msg)
{
	npu_mbox_set_txrx_reg_addr(msg[0] & 0xF, msg[2], msg[3],
				   msg[4], msg[5]);
	return 1;
}

int wifi_mail_set_ratelimit(u32 *msg)
{
	npu_printf("%s:%d band_idx=%d bssid_idx=%d ctrl=%d !!!\n",
		   "wifi_mail_set_wait_ratelimit_ctrl", 460,
		   msg[2], msg[3], msg[4]);
	ratelimit_table[msg[2] * 16 + msg[3]] = msg[4];
	return 1;
}

int wifi_mail_set_arht_chip_info(u32 *msg)
{
	u32 i;

	if (msg[9] == 0xFFFFFFFF) {
		npu_printf("%s get phy tx gpio error\n",
			   "wifi_mail_set_wait_arht_chip_info");
		return 0;
	}
	arht_phy_tx_gpio = msg[9];
	for (i = 0; i < 6; i++)
		arht_chip_info[i] = msg[2 + i];
	arht_chip_info_valid = 1;
	return 1;
}

int wifi_mail_get_dma_addr(u32 *msg)
{
	u32 dir = msg[2];
	u32 band = msg[0] & 0xF;
	u32 addr;

	addr = npu_mbox_txrx_ring_dma_addr_get(dir, band);
	msg[2] = addr;
	npu_printf("get dma addr. band=%d dir=%d addr=%x\n", band, dir, addr);
	return 1;
}

int wifi_mail_get_ring_size(u32 *msg)
{
	u32 dir = msg[2];
	u32 band = msg[0] & 0xF;
	u32 size = 0;

	npu_mbox_txrx_ring_size_get(dir, band, &size);
	msg[2] = size;
	npu_printf("%s get wait size =%d\n",
		   "wifi_mail_get_wait_ring_size", size);
	return 1;
}

int wifi_mail_get_rxdesc_base(u32 *msg)
{
	u32 band = msg[0] & 0xF;
	u32 base;

	base = npu_mbox_get_rxdesc_base(band);
	if (base != 0)
		msg[2] = base;
	else {
		npu_printf("%s: not support\n", "wifi_mail_get_wait_rxdesc_base");
		msg[2] = 1111;
	}
	return 1;
}

/* GET_WAIT handlers — indexed by SDK WIFI_MAIL_Get_Wait_Func_t in
 * get_wait_func_table[10].  Each receives the DMA-translated msg pointer. */

int wifi_mail_get_npu_info(u32 *msg)
{
	msg[2] = 0;
	npu_mbox_get_npu_info();
	return 1;
}

int wifi_mail_get_last_rate(u32 *msg)
{
	msg[2] = 222;
	msg[3] = 3333;
	return 1;
}

int wifi_mail_get_counter(u32 *msg)
{
	u32 *v2 = &msg[4];
	u8 *v3 = (u8 *)&msg[132];
	u64 bytes_2g, pkts_2g, bytes_5g, pkts_5g;
	u8 omac_2g, omac_5g;
	u32 apcli[8];
	u32 i;

	msg[2] = 0;
	for (i = 0; i < 16; i++) {
		npu_mbox_get_counter(i, &bytes_2g, &pkts_2g,
				     &bytes_5g, &pkts_5g,
				     &omac_2g, &omac_5g);
		v2[0] = (u32)pkts_2g;
		v2[1] = (u32)(pkts_2g >> 32);
		v2[32] = (u32)pkts_5g;
		v2[33] = (u32)(pkts_5g >> 32);
		v2[64] = (u32)bytes_2g;
		v2[65] = (u32)(bytes_2g >> 32);
		v2[96] = (u32)bytes_5g;
		v2[97] = (u32)(bytes_5g >> 32);
		v3[0] = omac_5g;
		v3[16] = omac_2g;
		v2 += 2;
		v3++;
	}

	npu_mbox_get_counter_apcli(apcli, &msg[660], &msg[148]);

	msg[140] = apcli[0];
	msg[141] = apcli[1];
	msg[142] = apcli[2];
	msg[143] = apcli[3];
	msg[144] = apcli[4];
	msg[145] = apcli[5];
	msg[146] = apcli[6];
	msg[147] = apcli[7];

	return 1;
}

int wifi_mail_get_dbg_counter(u32 *msg)
{
	msg[2] = counter_base_get(msg[0] & 0xF);
	return 1;
}

int wifi_mail_get_wcid_dbg_counter(u32 *msg)
{
	msg[2] = wcid_counter_base_get(msg[0] & 0xF);
	return 1;
}

int wifi_mail_get_mdc_lock(u32 *msg)
{
	(void)msg;
	return 1;
}

int wifi_mail_get_dump_mapping(u32 *msg)
{
	sram_buf_dump();
	msg[2] = 0;
	return 1;
}


#if defined(HAS_TR471) && defined(WIFI_KITE)
int kite_wifi_config(u32 base, u32 cnt)
{
	u32 *msg = (u32 *)((base & 0x3FFFFFFF) | NPU_ADDR_MASK);
	u32 func_type = msg[0];
	u32 sz;

	(void)cnt;
	switch (func_type) {
	case 1:
		kite_wifi_cfg[0] = msg[1];
		sz = msg[2];
		if (sz > 1450)
			sz = 1450;
		kite_wifi_cfg[1] = sz;
		kite_wifi_cfg[2] = msg[3];
		sz = msg[4];
		if (sz > 1450)
			sz = 1450;
		kite_wifi_cfg[3] = sz;
		kite_wifi_cfg[4] = msg[5];
		break;
	case 5:
		npu_printf("FUNC_TYPE_START_TEST\n");
		kite_wifi_cfg[9] = msg[2];
		kite_wifi_cfg[10] = msg[1];
		kite_wifi_cfg[8] = 1;
		kite_test_active = 1;
		break;
	case 6:
		npu_memset((void *)kite_wifi_cfg, 0, sizeof(kite_wifi_cfg));
		kite_test_active = 0;
		break;
	case 10:
		npu_printf("FUNC_TYPE_SET_BUFF_ADDR\n");
		break;
	default:
		break;
	}
	return 0;
}
#endif /* HAS_TR471 && WIFI_KITE */

#endif /* WIFI_KITE */

#ifdef WIFI_EAGLE

/* ================================================================
 * WiFi mailbox command wrappers (eagle path)
 *
 * MT7991/MT7992/MT7993. Same wrapper shape as kite, one helper per
 * command, but every helper differs: the host carries a ring index in
 * interfaceID rather than a band, and the rings are the eagle RRO,
 * MSDU page and indirect-command rings.
 * ================================================================ */

/* ring index, carried in interfaceID */
#define EAGLE_RING_RX0          0
#define EAGLE_RING_RX1          1
#define EAGLE_RING_MSDU_PG0     5
#define EAGLE_RING_MSDU_PG1     6
#define EAGLE_RING_IND_CMD0     8
#define EAGLE_RING_IND_CMD1     9
#define EAGLE_RING_TXDONE0      10
#define EAGLE_RING_TXDONE1      11
#define EAGLE_RING_ALL          15

#define EAGLE_RX_RING_MAX_IDX   1535
#define EAGLE_TX_BUF_SLOT       256
#define EAGLE_TX_BUF_SPACE_SIZE 0x80000
#define EAGLE_TXDONE_RING_BYTES 0x2000

/* host addresses below 0xC0000000 are outside the window the NPU can reach */
static int eagle_addr_in_range(u32 addr, const char *who)
{
	if (addr < 0xC0000000u)
		return 1;
	npu_printf("********** ERROR ***************\n");
	npu_printf("%s() ERROR !!! out of available range:%x \n", who, addr);
	return 0;
}

static void npu_set_pcie_base_eagle(u32 addr, u32 ring)
{
	eagle_addr_in_range(addr, "npu_set_pcie_base_eagle");

	switch (ring) {
	case EAGLE_RING_RX0:
		eagle_rx_ring_pcie_base[0] = addr;
		break;
	case EAGLE_RING_RX1:
		eagle_rx_ring_pcie_base[1] = addr;
		break;
	case EAGLE_RING_MSDU_PG0:
		eagle_msdu_pg_pcie_base = addr;
		break;
	case EAGLE_RING_IND_CMD0:
	case EAGLE_RING_IND_CMD1:
		eagle_ind_cmd_pcie_base = addr;
		break;
	case EAGLE_RING_TXDONE0:
		eagle_txdone_pcie_base = addr;
		break;
	case EAGLE_RING_ALL:
		/* every base is in, publish the cpu index of each ring */
		REG32(eagle_rx_ring_pcie_base[0] + 8) = EAGLE_RX_RING_MAX_IDX;
		REG32(eagle_rx_ring_pcie_base[1] + 8) = EAGLE_RX_RING_MAX_IDX;
		REG32(eagle_ind_cmd_pcie_base + 8) = EAGLE_RX_RING_MAX_IDX;
		REG32(eagle_txdone_pcie_base + 8) = eagle_txdone_ring_cnt - 1;
		npu_printf("[NPU] set RRO ring cpu idx \n");
		break;
	default:
		npu_printf("%s() wrong input value !!!!!\n",
			   "npu_set_pcie_base_eagle");
		break;
	}
}

static void npu_set_pcie_base_for_tx_ring_eagle(u32 addr, u32 ring)
{
	eagle_addr_in_range(addr, "npu_set_pcie_base_for_tx_ring_eagle");

	switch (ring) {
	case 0:
		eagle_tx_ring_pcie_base[0] = addr;
		eagle_tx_ring_cpu_idx = 0;
		eagle_tx_ring_dma_idx = 0;
		break;
	case 1:
		eagle_tx_ring_pcie_base[1] = addr;
		eagle_tx_ring_cpu_idx = 0;
		eagle_tx_ring_dma_idx = 0;
		break;
	case 3:
		eagle_tx_ring_pcie_base_r3 = addr;
		break;
	default:
		npu_printf("%s() wrong input value!!!\n",
			   "npu_set_pcie_base_for_tx_ring_eagle");
		break;
	}
}

static void npu_set_rx_ring_for_tx_done_phy_base_eagle(u32 addr, u32 ring)
{
	eagle_addr_in_range(addr, "npu_set_rx_ring_for_tx_done_phy_base_eagle");

	if (ring == 0) {
		eagle_rx_txdone_desc_base = (addr & 0x3FFFFFFF) | NPU_ADDR_MASK;
		return;
	}
	if (ring != EAGLE_RING_MSDU_PG0) {
		npu_printf("%s() wrong input value!!!\n",
			   "npu_set_rx_ring_for_tx_done_phy_base_eagle");
		return;
	}
	npu_printf("%s()[NPU] set 2G msdu desc addr, MSDU_PG_RING_DESC_BASE:%x\n",
		   "npu_set_rx_ring_for_tx_done_phy_base_eagle",
		   eagle_msdu_pg_desc_base);
	eagle_msdu_pg_desc_base = (addr & 0x3FFFFFFF) | NPU_ADDR_MASK;
}

/* descriptor base of one eagle ring inside the shared PCIe descriptor
 * block, by the ring's own id. Offsets are the blob's own table. */
static u32 eagle_ring_desc_base(u32 ring_id)
{
	static const u32 off[6] = {
		0x00000, 0x140A0, 0x06020, 0x1A0C0, 0x0E020, 0x0E0A0
	};

	if (ring_id - 1 > 5)
		return 0;
	return wifi_pcie_desc_base + off[ring_id - 1];
}

/* every 256-byte slot of the tx buffer space carries a 144-byte header */
static void eagle_tx_buf_space_clear(u32 base)
{
	u32 p, end = base + EAGLE_TX_BUF_SPACE_SIZE;

	for (p = base; p != end; p += EAGLE_TX_BUF_SLOT)
		npu_memset((void *)p, 0, 144);
}

/* tx done ring: one 16-byte descriptor per 256-byte tx packet buffer */
static void eagle_txdone_ring_init(u32 band)
{
	u32 buf, desc, i;

	/* the host sets the tx packet buffer last; wait for it */
	while (eagle_tx_pkt_buf_addr == 0)
		delay_ms(100);

	if (band != 0) {
		buf = eagle_tx_buf_space_r11;
		eagle_txdone_cpu_idx[1] = 0;
		eagle_txdone_dma_idx[1] = 0;
		eagle_txdone_desc_base[1] = sram_buf_alloc(17);
		desc = eagle_txdone_desc_base[1];
	} else {
		buf = eagle_tx_buf_space_r10;
		eagle_txdone_cpu_idx[0] = 0;
		eagle_txdone_dma_idx[0] = 0;
		eagle_txdone_desc_base[0] = sram_buf_alloc(16);
		desc = eagle_txdone_desc_base[0];
	}

	for (i = 0; i < EAGLE_TXDONE_RING_BYTES; i += 16) {
		REG32(desc + i) = (buf & 0x3FFFFFFF) | 0x80000000;
		REG32(desc + i + 4) = 0;
		REG32(desc + i + 8) = 0;
		*(volatile u8 *)(desc + i + 12) = 0;
		buf += EAGLE_TX_BUF_SLOT;
	}
}

static void npu_set_tx_ring_buf_space_phy_base_eagle(u32 addr, u32 ring)
{
	u32 base = (addr & 0x3FFFFFFF) | NPU_ADDR_MASK;

	eagle_addr_in_range(addr, "npu_set_tx_ring_buf_space_phy_base_eagle");

	switch (ring) {
	case 0:
		eagle_tx_buf_space[0] = base;
		break;
	case 1:
		eagle_tx_buf_space[1] = base;
		break;
	case EAGLE_RING_MSDU_PG0:
		eagle_tx_buf_space_pg[0] = base;
		eagle_tx_buf_space_clear(base);
		break;
	case EAGLE_RING_MSDU_PG1:
		eagle_tx_buf_space_pg[1] = base;
		eagle_tx_buf_space_clear(base);
		break;
	case EAGLE_RING_TXDONE0:
		eagle_tx_buf_space_r10 = base;
		eagle_txdone_ring_init(0);
		break;
	case EAGLE_RING_TXDONE1:
		eagle_tx_buf_space_r11 = base;
		eagle_txdone_ring_init(1);
		break;
	default:
		npu_printf("[error]%s wrong band index:%d !!!!!\n",
			   "npu_set_tx_ring_buf_space_phy_base_eagle", ring);
		break;
	}
}

/* del_sta: the host packs wcid in bits[10:0] and tid in bits[14:11] */
static void eagle_icv_err_mbox_handle(u32 action, u32 arg)
{
	u32 wcid = arg & 0x7FF;
	u32 tid = (arg >> 11) & 0xF;
	u32 i;

	if (wcid > 0x402 || tid > 8) {
		npu_printf("%s() invalid wcid:%d or tid:%d \n",
			   "_icv_err_mbox_handle", wcid, tid);
		return;
	}
	if (eagle_icv_err_table == 0)
		return;

	switch (action) {
	case 0:
		REG32(eagle_icv_err_table + 4 * wcid) |= 1u << tid;
		break;
	case 1:
		REG32(eagle_icv_err_table + 4 * wcid) &= ~(1u << tid);
		break;
	case 2:
		npu_printf("wcid[%04d]:%08x \n", wcid,
			   REG32(eagle_icv_err_table + 4 * wcid));
		break;
	case 3:
		for (i = 0; i < 1026; i++)
			REG32(eagle_icv_err_table + 4 * i) = 0;
		break;
	default:
		npu_printf("%s() wrong msg action:%d %x \n",
			   "_icv_err_mbox_handle", action, arg);
		break;
	}
}

/* rx ring: one 16-byte descriptor per allocated packet buffer */
static int eagle_rx_ring_init(u32 ring_size, u32 band)
{
	u32 desc_base, buf_id, desc, i;

	if (ring_size - 1 > EAGLE_RX_RING_MAX_IDX)
		npu_printf("ERROR! rx_ring_size = %d\n", ring_size);

	eagle_rx_ring_size[band] = (u16)ring_size;
	desc_base = eagle_rx_ring_desc_base[band];

	for (i = 0; i < ring_size; i++) {
		buf_id = buf_id_alloc_hw(0, band);
		if (buf_id == (u32)-1) {
			npu_printf("[%s]rx ring init: alloc buffid fail!\n",
				   band ? "BAND1" : "BAND0");
			return 1;
		}
		eagle_rx_ring_bufid[band][i] = (u16)buf_id;
		desc = desc_base + 16 * i;
		REG32(desc) = ((((buf_id << 11) + eagle_pkt_buf_addr) &
				0x3FFFFFFF) | 0x80000000) + 192;
		REG32(desc + 8) = buf_id << 16;
		REG32(desc + 12) = 0;
		REG32(desc + 4) = 0x07000000;
	}

	eagle_rx_ring_cpu_idx[band] = 0;
	eagle_rx_ring_init_done[band] = 1;
	return 0;
}

static void npu_mbox_init_rxd_wrapper(u32 ring_size, u32 ring)
{
	switch (ring) {
	case EAGLE_RING_RX0:
		eagle_rx_ring_desc_base[0] = eagle_ring_desc_base(1);
		eagle_rx_ring_init(ring_size, 0);
		break;
	case EAGLE_RING_RX1:
		eagle_rx_ring_desc_base[1] = eagle_ring_desc_base(3);
		eagle_rx_ring_init(ring_size, 1);
		break;
	case EAGLE_RING_MSDU_PG0:
		eagle_msdu_pg_desc_base = eagle_ring_desc_base(5);
		break;
	case EAGLE_RING_MSDU_PG1:
		npu_printf("%s() wrong input val:%d\n",
			   "npu_rro_msdu_pg_ring_desc_addr", 1);
		break;
	case EAGLE_RING_IND_CMD0:
	case EAGLE_RING_IND_CMD1:
		eagle_ind_cmd_desc_base = eagle_ring_desc_base(6);
		break;
	case EAGLE_RING_TXDONE0:
		break;
	case EAGLE_RING_TXDONE1:
		npu_printf("[NPU] ignore tx done ring1 currently because of no use\n");
		break;
	default:
		npu_printf("%s() wrong input val:%d \n",
			   "npu_mbox_init_rxd_wrapper", ring);
		break;
	}
}

/* ---- set_wait table ---- */

int eagle_mail_set_pcie_addr(u32 *msg)
{
	npu_set_pcie_base_eagle(msg[2], msg[0] & 0xF);
	return 1;
}

int eagle_mail_set_desc(u32 *msg)
{
	npu_mbox_init_rxd_wrapper(msg[2], msg[0] & 0xF);
	return 1;
}

int eagle_mail_set_init_done(u32 *msg)
{
	(void)msg;
	return 1;
}

int eagle_mail_set_tran_to_cpu(u32 *msg)
{
	npu_printf("%s() interfaceID = %u \n", "wifi_mail_set_wait_tran2cpu",
		   msg[0] & 0xF);
	return 1;
}

int eagle_mail_set_ba_win_size(u32 *msg)
{
	(void)msg;
	npu_printf("%s() return due to no BA \n",
		   "npu_mbox_set_wait_ba_win_size_wrapper");
	return 1;
}

int eagle_mail_set_driver_model(u32 *msg)
{
	eagle_rro_mode = (u8)msg[2];
	npu_printf("glb_rro_mode=%d\n", eagle_rro_mode);
	return 1;
}

int eagle_mail_set_del_sta(u32 *msg)
{
	eagle_icv_err_mbox_handle(msg[0] & 0xF, msg[2]);
	return 1;
}

int eagle_mail_set_dram_ba_node(u32 *msg)
{
	eagle_addr_in_range(msg[2], "npu_mbox_set_wait_dram_ba_node_addr_wrapper");
	eagle_dram_ba_node_addr = msg[2];
	return 1;
}

int eagle_mail_set_pkt_buf(u32 *msg)
{
	eagle_addr_in_range(msg[2], "npu_mbox_set_wait_pkt_buf_addr_wrapper");
	eagle_pkt_buf_addr = msg[2];
	npu_printf("pkt_buf_addr=%x\n", eagle_pkt_buf_addr);
	return 1;
}

int eagle_mail_set_test_noba(u32 *msg)
{
	eagle_test_noba = (u8)msg[2];
	npu_printf("isforTestNoBA=%s\n", eagle_test_noba ? "true" : "false");
	if (eagle_test_noba > 1)
		npu_printf("[ERROR] isforTestNoBA is wrong value !!!\n");
	return 1;
}

int eagle_mail_set_flushone(u32 *msg)
{
	(void)msg;
	npu_printf("%s() not support\n",
		   "npu_mbox_set_wait_flushone_timeout_wrapper");
	return 1;
}

int eagle_mail_set_flushall(u32 *msg)
{
	(void)msg;
	npu_printf("%s() not support\n",
		   "npu_mbox_set_wait_flushall_timeout_wrapper");
	return 1;
}

int eagle_mail_set_force_cpu(u32 *msg)
{
	wifi_force_to_cpu = (u8)msg[2];
	npu_printf("isForceToCpu=%s\n", wifi_force_to_cpu ? "true" : "false");
	if (wifi_force_to_cpu > 1)
		npu_printf("[ERROR] isForceToCpu is wrong value !!!\n");
	return 1;
}

int eagle_mail_set_pcie_state(u32 *msg)
{
	u32 band = msg[0] & 0xF;

	if (band > 1) {
		npu_printf("[ERROR] band_idx is wrong value %d !!!\n", band);
		return 1;
	}
	eagle_pcie_state[band] = 1;
	return 1;
}

int eagle_mail_set_port_type(u32 *msg)
{
	eagle_pcie_port_type = (u8)msg[2];
	/* the blob also republishes the per-port PCIe windows at 0x1FA90038
	 * and 0x1FC28030 from SRAM addresses; that path is not reconstructed */
	return 1;
}

int eagle_mail_set_retry(u32 *msg)
{
	eagle_retry_times = (u16)msg[2];
	npu_printf("enq_error_retry_times = %d !!!\n", eagle_retry_times);
	return 1;
}

int eagle_mail_set_bar_info(u32 *msg)
{
	(void)msg;
	npu_printf("%s() return due to no BA \n",
		   "npu_mbox_set_wait_bar_info_wrapper");
	return 1;
}

int eagle_mail_set_fast_flag(u32 *msg)
{
	(void)msg;
	npu_printf("%s() not support\n", "npu_mbox_set_wait_fast_flag_wrapper");
	return 1;
}

int eagle_mail_set_band0_cpu(u32 *msg)
{
	(void)msg;
	npu_printf("%s L%d not support on kite\n",
		   "wifi_mail_set_wait_npu_band0_on_cpu_wrapper", 10342);
	return 1;
}

int eagle_mail_set_tx_ring_pcie(u32 *msg)
{
	npu_set_pcie_base_for_tx_ring_eagle(msg[2], msg[0] & 0xF);
	return 1;
}

int eagle_mail_set_tx_desc_hw(u32 *msg)
{
	npu_printf("%s: [band_idx=%d] desc phy addr=%lx \n",
		   "wifi_mail_set_wait_tx_ring_desc_phy_addr",
		   msg[0] & 0xF, msg[2]);
	return 1;
}

int eagle_mail_set_tx_buf_hw(u32 *msg)
{
	npu_set_tx_ring_buf_space_phy_base_eagle(msg[2], msg[0] & 0xF);
	return 1;
}

int eagle_mail_set_rx_txdone_hw(u32 *msg)
{
	npu_set_rx_ring_for_tx_done_phy_base_eagle(msg[2], msg[0] & 0xF);
	return 1;
}

int eagle_mail_set_tx_pkt_buf(u32 *msg)
{
	eagle_addr_in_range(msg[2],
			    "npu_mbox_set_wait_tx_pkt_buf_addr_wrapper");
	eagle_tx_pkt_buf_addr = msg[2];
	return 1;
}

int eagle_mail_set_txrx_reg(u32 *msg)
{
	(void)msg;
	return 1;
}

int eagle_mail_set_debug_flag(u32 *msg)
{
	npu_printf("set band:%d debugflag=%d\n", msg[0] & 0xF, msg[2]);
	return 1;
}

int eagle_mail_set_inode_cfg(u32 *msg)
{
	(void)msg;
	return 1;
}

int eagle_mail_set_inode_stop(u32 *msg)
{
	npu_printf("%s L%d set. band:%d\n",
		   "wifi_mail_set_wait_inode_stop_action", 306, msg[0] & 0xF);
	return 1;
}

int eagle_mail_set_pcie_swap(u32 *msg)
{
	npu_printf("%s L%d set %d\n", "wifi_mail_set_wait_inode_pcie_swap",
		   322, msg[2]);
	return 1;
}

int eagle_mail_set_ratelimit(u32 *msg)
{
	npu_printf("%s:%d band_idx=%d bssid_idx=%d ctrl=%d !!!\n",
		   "wifi_mail_set_wait_ratelimit_ctrl", 460,
		   msg[2], msg[3], msg[4]);
	return 1;
}

int eagle_mail_set_arht_chip_info(u32 *msg)
{
	u32 i;

	eagle_phy_tx_gpio = msg[9];
	for (i = 0; i < 6; i++)
		eagle_chip_info[i] = msg[2 + i];
	return 1;
}

/* ---- get_wait table ---- */

int eagle_mail_get_npu_info(u32 *msg)
{
	msg[2] = 0;
	return 1;
}

int eagle_mail_get_last_rate(u32 *msg)
{
	msg[2] = 222;
	msg[3] = 3333;
	return 1;
}

int eagle_mail_get_counter(u32 *msg)
{
	npu_memset(&msg[2], 0, 40);
	return 1;
}

int eagle_mail_get_dbg_counter(u32 *msg)
{
	msg[2] = 0;
	return 1;
}

int eagle_mail_get_rxdesc_base(u32 *msg)
{
	u32 band = msg[0] & 0xF;

	if (band > 1)
		return 1;
	msg[2] = eagle_rx_ring_pcie_base[band];
	return 1;
}

int eagle_mail_get_wcid_dbg_counter(u32 *msg)
{
	msg[2] = 0;
	return 1;
}

int eagle_mail_get_dma_addr(u32 *msg)
{
	msg[2] = 0;
	return 1;
}

int eagle_mail_get_ring_size(u32 *msg)
{
	msg[2] = 0;
	npu_printf("%s() not support\n", "wifi_mail_get_wait_ring_size");
	return 1;
}

int eagle_mail_get_mdc_lock(u32 *msg)
{
	(void)msg;
	return 1;
}

int eagle_mail_get_dump_mapping(u32 *msg)
{
	msg[2] = 0;
	return 1;
}

int eagle_mail_set_event(u32 base, u32 cnt)
{
	u32 *msg = (u32 *)((base & 0x3FFFFFFF) | NPU_ADDR_MASK);
	u32 evt = msg[0];

	(void)cnt;
	if (evt < 4 && eagle_event_table[evt])
		return eagle_event_table[evt](base, cnt);
	return 0;
}

int eagle_wifi_config(u32 base, u32 cnt)
{
	u32 *msg = (u32 *)((base & 0x3FFFFFFF) | NPU_ADDR_MASK);
	u32 func_type = msg[0];
	u32 sz;

	(void)cnt;
	switch (func_type) {
	case 1:
		eagle_rro_cfg[0] = msg[1];
		sz = msg[2];
		if (sz > 1450)
			sz = 1450;
		eagle_rro_cfg[1] = sz;
		eagle_rro_cfg[2] = msg[3];
		sz = msg[4];
		if (sz > 1450)
			sz = 1450;
		eagle_rro_cfg[3] = sz;
		eagle_rro_cfg[4] = msg[5];
		break;
	case 5:
		npu_printf("FUNC_TYPE_START_TEST\n");
		eagle_rro_cfg[9] = msg[2];
		eagle_rro_cfg[10] = msg[1];
		eagle_rro_cfg[8] = 1;
		eagle_rro_active = 1;
		break;
	case 6:
		npu_memset((void *)eagle_rro_cfg, 0, sizeof(eagle_rro_cfg));
		eagle_rro_active = 0;
		break;
	case 10:
		npu_printf("FUNC_TYPE_SET_BUFF_ADDR\n");
		break;
	default:
		break;
	}
	return 1;
}

#endif /* WIFI_EAGLE */

#ifdef HAS_WIFI
/* WiFi funcType dispatch (callback[0] — MFUNC_WIFI)
 *
 * Blob dispatches all four funcTypes via function pointer tables in .data:
 *   SET_WAIT(1): set_wait_func_table[31] indexed by funcId (up to 30)
 *   SET_NO_WAIT(2): single fn ptr, only funcId=0
 *   GET_WAIT(3): get_wait_func_table[10] indexed by funcId (up to 9)
 *   GET_NO_WAIT(4): single fn ptr, only funcId=0
 *
 * All handlers receive the DMA-translated msg pointer. */
int wifi_mail_dispatch(u32 base, u32 cnt)
{
	u32 *msg = (u32 *)((base & 0x3FFFFFFF) | NPU_ADDR_MASK);
	u32 func_type = (msg[0] >> 4) & 0xF;
	u32 func_id;

	switch (func_type) {
	case 1: /* SET_WAIT */
		return wifi_mail_set_wait(base, cnt);
	case 2: /* SET_NO_WAIT */
#ifdef WIFI_KITE
		return wifi_mail_set_event(base, cnt);
#elif defined(WIFI_EAGLE)
		return eagle_mail_set_event(base, cnt);
#else
		return 0;
#endif
	case 3: /* GET_WAIT */
		func_id = msg[1];
		if (func_id > 9) {
			npu_printf("Error: exceed max num!interfaceid=%u "
				   "wifi_mail_data->funcType=%u "
				   "wifi_mail_data->funcId=%u\n",
				   msg[0] & 0xF, func_type, func_id);
			return 1;
		}
		if (get_wait_func_table[func_id])
			return get_wait_func_table[func_id](msg);
		return 1;
	case 4: /* GET_NO_WAIT */
		if (msg[1] != 0) {
			npu_printf("Error: exceed max num!interfaceid=%u "
				   "wifi_mail_data->funcType=%u "
				   "wifi_mail_data->funcId=%u\n",
				   msg[0] & 0xF, func_type, msg[1]);
		}
		return 1;
	default:
		npu_printf("not support unknow funcType\n");
		return 1;
	}
}

/* WiFi mail set_wait handler: dispatches sub-commands from host */
int __attribute__((noinline)) wifi_mail_set_wait(u32 base, u32 cnt)
{
	u32 *msg = (u32 *)((base & 0x3FFFFFFF) | NPU_ADDR_MASK);
	u32 func_id = msg[1];

	(void)cnt;
	if (func_id > 30) {
		npu_printf("Error: exceed max num!interfaceid=%u "
			   "wifi_mail_data->funcType=%u "
			   "wifi_mail_data->funcId=%u\n",
			   msg[0] & 0xF, 1, func_id);
		return 1;
	}
	if (set_wait_func_table[func_id])
		return set_wait_func_table[func_id](msg);
	return 1;
}

/* WiFi mail set_event handler */
int __attribute__((noinline)) wifi_mail_set_event(u32 base, u32 cnt)
{
	u32 *msg = (u32 *)((base & 0x3FFFFFFF) | NPU_ADDR_MASK);
	u32 cmd = msg[1];

	switch (cmd) {
	case 0:
		wifi_npu_init(msg[0] & 0xF);
		break;
	default:
		npu_printf("set_event: unknown cmd %d\n", cmd);
		break;
	}
	return 1;
}
#endif /* HAS_WIFI */

/* Core0 WiFi init wrapper */
void core0_wifi_init_wrapper(void)
{
#ifdef HAS_WIFI
	int result;

#ifdef HAS_BME
	tdma_tx_init();
#endif
	wifi_bridge_init();
	npu_printf("%s finish\n", "core0_wifi_init_wrapper");

	result = hostadpt_init();
	if (result != 0)
		npu_printf("Error: there is something wrong with hostadpt\n");
#endif
}

/* Core3 WiFi init wrapper - drain loop (kite) or TX/RX poll (eagle) */
void core3_wifi_init_wrapper(void)
{
#ifdef HAS_WIFI
#ifdef WIFI_KITE
	do {
		if (rxd_5g_init_done != 0) {
			if (hostadpt_tx_ring_ready == 1)
				rxnode_drain(1);
			if (hostadpt_tx_ring_ready == 1)
				pinode_drain(1);
		}
		if (rxd_2g_init_done != 0) {
			if (hostadpt_tx_ring_ready == 1)
				rxnode_drain(0);
			if (hostadpt_tx_ring_ready == 1)
				pinode_drain(0);
		}
	} while (!(wifi_debug_flags & 2));
#else
	while (!(wifi_bridge_active & 2)) {
		if (wifi_tx_pending != 0)
			wifi_tx_process();
		if (wifi_rx_pending != 0)
			wifi_rx_process();
	}
#endif
	npu_printf("%s finish\n", "core3_wifi_init_wrapper");
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

