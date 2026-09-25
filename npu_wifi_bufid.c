/*
 * AN75XX NPU firmware - buffer ids, tx tokens and counters
 *
 * Two rings over two packet buffers. An rx buffer id covers 2 KB of the
 * WiFi packet buffer and comes back from whoever finishes with the
 * frame. A tx token covers 2 KB of the NPU tx packet buffer and comes
 * back from the WiFi tx done ring.
 *
 * The counter blocks are the debug statistics the host reads over
 * WIFI_MAIL_API_GET_WAIT_DBG_COUNTER.
 */

#include "npu_internal.h"
#include "npu_wifi.h"


/* ================================================================
 * Buffer ID management
 *
 * The buffer manager hands out ids through CSR 0xBC8 + 16 * type + band
 * (AN7552: an MMIO load) and takes them back through a per-band MMIO
 * register.
 * band=0 → 2.4G, band=1 → 5G, band=2 → 6G (future)
 * ================================================================ */

#define bufid_csr(n) ({ \
	u32 __v; \
	__asm__ volatile("fence\n\tcsrr %0, " #n : "=r"(__v) :: "memory"); \
	(s32)(s16)__v; })

/* one id from the buffer manager, -1 when it has none */
NPU_HOT s32 buf_id_alloc_hw(u32 type, u32 band)
{
#ifdef AN7552
	/* no id CSRs: a load pops the id, beside the free register */
	u32 base = type ? 0x1EC0A000 : BMGR_BASE;

	return (s16)REG32(base + band * 0x800 + (64 + band) * 4);
#endif
	switch (type * 4 + band) {
	case 0:
		return bufid_csr(0xBC8);
	case 1:
		return bufid_csr(0xBC9);
	case 2:
		return bufid_csr(0xBCA);
	case 4:
		return bufid_csr(0xBD8);
	case 5:
		return bufid_csr(0xBD9);
	case 6:
		return bufid_csr(0xBDA);
	case 8:
		return bufid_csr(0xBE8);
	case 9:
		return bufid_csr(0xBE9);
	case 10:
		return bufid_csr(0xBEA);
	default:
		return -1;
	}
}

/* give an id back to the buffer manager */
NPU_HOT void buf_id_free(u32 type, u32 band, u32 buf_id)
{
	u32 base;

	if (type == 0)
		base = BMGR_BASE;
#ifdef AN7552
	else
		base = 0x1EC0A000;
#else
	else if (type == 1)
		base = 0x1EC0A000;
	else
		base = 0x1EC07000;
#endif
	REG32(base + band * 0x800 + (67 + band) * 4) = buf_id;
}

/* ================================================================
 * Counter / statistics infrastructure
 * ================================================================ */

u32 counter_base_2g;
u32 counter_base_5g;
u32 counter_base_tri;
u32 wcid_counter_base_2g;
u32 wcid_counter_base_5g;

#if defined(AN7552) && defined(WIFI_KITE)
void wifi_cnt_add(u32 band, u32 off)
{
	u32 base = (band == 1) ? counter_base_5g :
		   (band != 0) ? counter_base_tri : counter_base_2g;

	if (base)
		(*(u32 *)(base + off))++;
}
#endif

/* tx buffer id rings.
 *
 * bufid_pool  12288 ids, the full buffer id space
 * tx_free_ring 13312 slots, the ids free for transmit
 * tx_buf_state 13312 bytes of per-id state, 3 means the id is parked in
 *              a TDMA rx descriptor and must not be handed out
 * tdma_rx_ids  the ids recovered from the rx rings, up to 2048
 */
#define BUFID_POOL_LAST       (BUFID_POOL_ENTRIES - 1)
#define TX_FREE_RING_ENTRIES  13312
#define TX_FREE_RING_LAST     (TX_FREE_RING_ENTRIES - 1)
#define TDMA_RX_ID_ENTRIES    2048
#define TX_BUF_STATE_IN_RX    3

static u32 bufid_pool_base;
static u32 tx_buf_state_base;
static u32 tdma_rx_ids_base;

/* rx buffer id ring: 12288 ids over the WiFi packet buffer */
static u32 rx_bufid_alloc_mutex[2];
static u32 rx_bufid_free_mutex[2];
/* ring indices: written by one hart under the mutex, read by others */
static volatile u16 rx_bufid_ridx;
static volatile u16 rx_bufid_widx;
static u32 rx_bufid_alloc_count;

/* tx token ring: 13312 tokens over the NPU tx packet buffer */
static u32 bufid_ring_base;
static u32 tx_token_alloc_mutex[2];
static u32 tx_token_free_mutex[2];
static volatile u16 bufid_ridx;
static volatile u16 bufid_widx;
static u32 bufid_deq_count;
static u32 bufid_enq_count;

/* several harts free: read the index only under the mutex */
void buf_id_return(u16 buf_id)
{
	u16 widx;

	hw_mutex_lock(rx_bufid_free_mutex);
	widx = rx_bufid_widx;
	*(volatile u16 *)(bufid_pool_base + 2 * (u32)widx) = buf_id;
	if ((wifi_debug_flags & 4) && counter_base_tri)
		(*(u32 *)(counter_base_tri + 0x18))++;
	rx_bufid_widx = (widx == BUFID_POOL_LAST) ? 0 : widx + 1;
	hw_mutex_unlock(rx_bufid_free_mutex);
}

s32 buf_id_alloc_ring(void)
{
	u16 next;
	s32 id;

	hw_mutex_lock(rx_bufid_alloc_mutex);
	next = (rx_bufid_ridx == BUFID_POOL_LAST) ? 0 : rx_bufid_ridx + 1;
	if (rx_bufid_widx == next) {
		if ((wifi_debug_flags & 4) && counter_base_tri)
			(*(u32 *)(counter_base_tri + 0x20))++;
		hw_mutex_unlock(rx_bufid_alloc_mutex);
		return -1;
	}
	rx_bufid_alloc_count++;
	if ((wifi_debug_flags & 4) && counter_base_tri)
		(*(u32 *)(counter_base_tri + 0x1C))++;
	id = *(s16 *)(bufid_pool_base + 2 * (u32)rx_bufid_ridx);
	rx_bufid_ridx = next;
	hw_mutex_unlock(rx_bufid_alloc_mutex);
	return id;
}

void tx_token_free(u16 token)
{
	hw_mutex_lock(tx_token_free_mutex);
	*(volatile u16 *)(bufid_ring_base + 2 * (u32)bufid_widx) = token;
	bufid_enq_count++;
	if ((wifi_debug_flags & 4) && counter_base_tri)
		(*(u32 *)(counter_base_tri + 0x4C))++;
	bufid_widx = (bufid_widx == TX_FREE_RING_LAST) ? 0 : bufid_widx + 1;
	hw_mutex_unlock(tx_token_free_mutex);
}

s32 tx_token_alloc(void)
{
	u16 next;
	s32 id;

	hw_mutex_lock(tx_token_alloc_mutex);
	next = (bufid_ridx == TX_FREE_RING_LAST) ? 0 : bufid_ridx + 1;
	if (bufid_widx == next) {
		if ((wifi_debug_flags & 4) && counter_base_tri)
			(*(u32 *)(counter_base_tri + 0x54))++;
		hw_mutex_unlock(tx_token_alloc_mutex);
		return -1;
	}
	bufid_deq_count++;
	if ((wifi_debug_flags & 4) && counter_base_tri)
		(*(u32 *)(counter_base_tri + 0x50))++;
	id = *(s16 *)(bufid_ring_base + 2 * (u32)bufid_ridx);
	bufid_ridx = next;
	hw_mutex_unlock(tx_token_alloc_mutex);
	return id;
}

/* back to every id free */
void rx_bufid_pool_reset(void)
{
	u16 *p = (u16 *)bufid_pool_base;
	u32 i;

	hw_mutex_lock(rx_bufid_free_mutex);
	hw_mutex_lock(rx_bufid_alloc_mutex);
	for (i = 0; i < BUFID_POOL_ENTRIES; i++)
		p[i] = (u16)i;
	rx_bufid_ridx = 0;
	rx_bufid_widx = 0;
	hw_mutex_unlock(rx_bufid_free_mutex);
	hw_mutex_unlock(rx_bufid_alloc_mutex);
}

void bufid_pool_init(void)
{
	u16 *p;
	u32 i;

	rx_bufid_alloc_mutex[0] = 12;
	rx_bufid_alloc_mutex[1] = 0;
	rx_bufid_free_mutex[0] = 13;
	rx_bufid_free_mutex[1] = 0;
	tx_token_alloc_mutex[0] = 3;
	tx_token_alloc_mutex[1] = 0;
	tx_token_free_mutex[0] = 4;
	tx_token_free_mutex[1] = 0;

	p = (u16 *)sram_buf_alloc(138);
	bufid_pool_base = (u32)p;
	for (i = 0; i < BUFID_POOL_ENTRIES; i++)
		p[i] = (u16)i;
	rx_bufid_ridx = 0;
	rx_bufid_widx = 0;

#ifndef AN7552
	/* tx tokens: AN7552 has none (no NPU tx) */
	p = (u16 *)sram_buf_alloc(18);
	bufid_ring_base = (u32)p;
	for (i = 0; i < TX_FREE_RING_ENTRIES; i++)
		p[i] = (u16)i;

	tx_buf_state_base = sram_buf_alloc(28);
	npu_memset((void *)tx_buf_state_base, 0, TX_FREE_RING_ENTRIES * 2);

	tdma_rx_ids_base = sram_buf_alloc(29);

	bufid_ridx = 0;
	bufid_widx = 0;
#endif
}

#ifdef HAS_BME
/* Recover the buffer ids parked in the TDMA rx descriptors. Each
 * descriptor keeps the buffer pointer at +8. */
static u32 tdma_rx_collect_bufids(u16 *out, u32 max)
{
	u32 ring, i, n = 0;
	u32 ptr;

	for (ring = 0; ring < TDMA_RX_RINGS; ring++) {
		for (i = 0; i < TDMA_RX_RING_DESCS; i++) {
			ptr = REG32(tdma_rx_dscp_base[ring] +
				    TDMA_RX_DESC_SIZE * i + 8);
			out[n++] = (u16)((ptr - npu_tx_pkt_buf_addr) >> 11);
			if (n > max) {
				npu_printf("prepared array not enough %d\n",
					   max);
				break;
			}
		}
	}
	return n;
}

/* Rebuild the transmit free ring: every buffer id except the ones the
 * TDMA rx descriptors are holding. */
void np_skb_tx_force_reset(void)
{
	u16 *rx_ids = (u16 *)tdma_rx_ids_base;
	u16 *state = (u16 *)tx_buf_state_base;
	u16 *ring = (u16 *)bufid_ring_base;
	u32 in_rx, i, w, id;

	hw_mutex_lock(tx_token_alloc_mutex);
	hw_mutex_lock(tx_token_free_mutex);

	npu_printf("run %s()\n", "np_skb_tx_force_reset");

	for (i = 0; i < TDMA_RX_ID_ENTRIES; i++)
		rx_ids[i] = 0xFFFF;
	in_rx = tdma_rx_collect_bufids(rx_ids, TDMA_RX_ID_ENTRIES);

	npu_printf("%s() enter\n", "np_skb_set_tx_buf_state");
	for (i = 0; i < TX_FREE_RING_ENTRIES; i++)
		state[i] = 0;
	for (i = 0; i < in_rx; i++)
		state[rx_ids[i]] = TX_BUF_STATE_IN_RX;

	/* the ids held by rx go in first, then every id still free */
	id = 0;
	for (w = 0; w < TX_FREE_RING_ENTRIES; w++) {
		if (w < in_rx && rx_ids[w] != 0xFFFF) {
			ring[w] = rx_ids[w];
			continue;
		}
		while (id < TX_FREE_RING_LAST &&
		       state[id] == TX_BUF_STATE_IN_RX)
			id++;
		ring[w] = (u16)id;
		id++;
	}

	bufid_ridx = (u16)in_rx;
	bufid_widx = 0;

	npu_printf("finish run %s() total_buf_in_tdmaRx:%d(%d)\n",
		   "np_skb_tx_force_reset", in_rx, TDMA_RX_ID_ENTRIES);

	hw_mutex_unlock(tx_token_free_mutex);
	hw_mutex_unlock(tx_token_alloc_mutex);
}
#endif /* HAS_BME */

/* per-band counter words: 170 on AN7552 (types 9/10 are 0x2A8) */
#ifdef AN7552
#define BAND_COUNTER_WORDS	170
#else
#define BAND_COUNTER_WORDS	250
#endif

void counter_init(u32 band)
{
	u32 *base;
	u32 count;

	npu_printf("%s:%d\n", "counter_init", band);
	if (band == 1) {
		counter_base_5g = sram_buf_alloc(9);
		base = (u32 *)counter_base_5g;
		count = BAND_COUNTER_WORDS;
	} else if (band == 0) {
		counter_base_2g = sram_buf_alloc(10);
		base = (u32 *)counter_base_2g;
		count = BAND_COUNTER_WORDS;
	} else {
		counter_base_tri = sram_buf_alloc(11);
		base = (u32 *)counter_base_tri;
		count = 28;
	}

	npu_memset(base, 0, count * 4);
}

void wcid_counter_init(u32 band)
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
