/*
 * AN75XX NPU firmware - reconstructed from binary
 *
 * Single-file firmware for Airoha AN7552/AN7581/AN7583 NPU.
 * Compile-time variants via -DAN75xx -DMTxxxx.
 */

#include "npu_config.h"
#include "npu_types.h"
#include "npu_regs.h"

/* ---- forward declarations ---- */

/* trap handler (in assembly section at bottom) */
void trap_vector(void);

/* core main functions */
static void core0_main(void);
static void core1_main(void);
#if MAX_CORE_NUM > 2
static void core2_main(void);
static void core3_main(void);
static void core4_main(void);
static void core5_main(void);
#endif
#if MAX_CORE_NUM > 6
static void core6_main(void);
static void core7_main(void);
#endif

/* subsystem init */
static void plic_init(void);
static void timer_init(int timer, int enable, int period);
static void mailbox_init(void);

/* PLIC */
static void default_isr(int src);
void call_isr_by_src(u32 src);
static void plic_enable(u32 src);
static void plic_disable(u32 src);

/* mailbox */
static void mbox_isr(int src);

/* printf */
int npu_printf(const char *fmt, ...);

/* mutex */
static void hw_mutex_lock(u32 *desc);
static void hw_mutex_unlock(u32 *desc);

/* SRAM buffer management */
static u32 sram_buf_alloc(u32 addr_type);
static void sram_buf_init(void);

/* counter infrastructure */
static void counter_init(u32 band);
static void wcid_counter_init(u32 band);

/* host adaptor */
static int hostadpt_init(void);

/* NPU bridge */
static void npu_bridge_buf_init(void);

/* packet forwarding */
static int pkt_forward(u32 buf_id, u32 pkt_len, s16 wcid, u8 amsdu,
		       u32 band, u8 fwd_type, u32 orig_len,
		       int classify_result, u8 tunnel);

/* reorder node management */
static void reorder_node_free(u16 node_idx, u8 node_type, u32 band);
static u32 reorder_node_alloc(u32 band, u32 *pool_type, u16 *idx_out);

/* BA entry management */
static void ba_flush_entry(u32 *entry);
static void ba_indicate_le_seq(u32 *entry, u32 seq);
static u32 ba_seq_scan(u32 *entry, u32 seq);
static u32 ba_state_update(u32 sn, u32 check_type, u32 entry_addr);
static int pkt_enqueue_bridge(u32 buf_id, u32 pkt_len, u32 amsdu,
			      u32 band, u32 fwd_type);

/* BME send */
static int bme_send_pkt(u32 buf_id, u32 pkt_size, u32 phys_addr, u32 band);

/* WiFi packet classification */
static int wifi_pkt_hdr_len(u32 buf_id, u32 band);
static void wifi_update_stats(u32 dir, u32 port, u8 state, u32 type,
			      u32 tid, u32 byte_cnt);
static u32 ba_scan_entries(u32 band);
static void ba_flush_all(u32 band);
static int wifi_pkt_classify(u32 buf_id, u32 pkt_len, u32 iface);
static int wifi_pkt_forward_single(u32 buf_id, u16 pkt_len, u8 amsdu);
static u32 wifi_multi_desc_handler(u32 band, u32 start_idx, u32 size,
				   u32 count);

/* WiFi init subsystem */
static void wifi_queue_mutex_init(void);
static void wifi_pkt_queue_init(u32 band);
static void wifi_ba_node_init(void);
static void wifi_pcie_desc_alloc(void);
static void wifi_npu_init(u32 dbdc);

/* WiFi RXD init */
static void npu_set_pcie_base(u32 addr, u32 band);
static int wifi_init_rxd_5g(u32 ring_size, u32 band);
static int wifi_init_rxd_2g(u32 ring_size, u32 band);
static void wifi_reset_ba_entry(u32 dir, u32 wcid);

/* WiFi mailbox setters */
static void npu_set_bar_info(u32 band, u32 packed);
static void npu_set_ba_entry(u32 band, u32 packed);

/* WiFi handlers - forward declared for data init */
#ifdef WIFI_KITE
static int wifi_mail_set_wait(u32 base, u32 cnt);
static int wifi_mail_set_event(u32 base, u32 cnt);
static void tdma_tx_init(void);
#endif

/* tunnel handlers */
#ifdef HAS_TUNNEL
static int tunnel_mail_handler(u32 base, u32 cnt);
static int hwnat_mail_dispatch(u32 raw_ptr);
#endif

/* DBA handlers */
#ifdef HAS_DBA
static int dba_mail_handler(u32 base, u32 cnt);
#endif

/* ================================================================
 * Global variables - .data section (initialized)
 *
 * Order must match npu_data.bin layout for binary-correct output.
 * Addresses shown are for AN7583_MT7996 reference variant.
 * ================================================================ */

/* 0x000: system config base */
u32 npu_max_frame_size = 1500;
static u32 __data_pad0[3];

/* 0x010: WiFi HW queue-to-AC mapping (3 bands x 16 entries) */
static u32 wifi_queue_map_b0[16] = {
	18, 19, 20, 52, 53, 54, 55, 21, 24, 25, 26, 27, 28, 29, 30, 31
};
static u32 wifi_queue_map_b1[8] = {
	16, 17, 18, 21, 22, 23, 24, 19
};
static u32 wifi_queue_map_b2[8] = {
	16, 17, 18, 21, 22, 23, 24, 19
};

/* 0x090: timer bit-index table */
static u32 timer_bit_map[16] = {
	0, 1, 2, 5, 6, 7, 8, 3, 0, 1, 2, 5, 6, 7, 8, 3
};

/* 0x0D0: PLIC source-to-register mapping (2 sets of 16) */
static u32 plic_src_reg_map[32] = {
	0x1EC10108, 0x1EC10110, 0x1EC10118, 0x1EC10120,
	0x1EC10128, 0x1EC10130, 0x1EC10138, 0x1EC10140,
	0x1EC10148, 0x1EC10150, 0x1EC10158, 0x1EC10160,
	0x1EC10168, 0x1EC10170, 0x1EC10178, 0x1EC10180,
	0x1EC10108, 0x1EC10110, 0x1EC10118, 0x1EC10120,
	0x1EC10128, 0x1EC10130, 0x1EC10138, 0x1EC10140,
	0x1EC10148, 0x1EC10150, 0x1EC10158, 0x1EC10160,
	0x1EC10168, 0x1EC10170, 0x1EC10178, 0x1EC10180,
};

/* 0x150+: additional PLIC config and WiFi handler pointers
 * (exact layout TBD - placeholder for binary matching) */
static u32 plic_config_ext[16];

/* 0x1C0: mailbox command dispatch table (kite WiFi) */
#ifdef WIFI_KITE
static mbox_handler_t wifi_mbox_handlers[10];
#endif

/* 0x1EC: WiFi chip name table (kite only, 8 bytes per entry) */
#ifdef WIFI_KITE
static const char wifi_chip_names[6][8] = {
	"MT7915A", "MT7915D", "MT7916",
	"MT7902",  "MT7990",  ""
};
#endif

/* 0x220: mailbox wrapper function pointer tables */
static mbox_handler_t mbox_pri_handlers[10];
static mbox_handler_t mbox_ext_handlers[32];

/* 0x2D0: tunnel config structures */
#ifdef HAS_TUNNEL
static u32 tunnel_config_ptrs[4];
static u8 tunnel_config_area[0x500];
#endif

/* 0x7E8: debug/exception handler dispatch */
static u32 exception_handlers[12];
static u32 exception_sentinels[4] = {
	0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0x0000FFFF
};

/* 0x820: UART console descriptors */
static u8 uart_desc[2][32] = {
	{ 0x01, 0x00, 'A','S','C','I','I', 0 },
	{ 0 }
};

/* 0x864: mutex descriptor pair table */
static u32 mutex_initial_state;
static u32 mutex_desc_pairs[512];

/* 0xC70: per-variant config scalars */
static s32 wifi_chip_index = -1;

#ifdef HAS_DBA
static u8 dba_cfg_flag0 = 1;
static u8 dba_cfg_flag1 = 1;
static u8 dba_cfg_flag2;
static u32 dba_alloc_gran = 5;
static u32 dba_tick_count = 10;
static u32 dba_defaults[4] = { 1, 1, 1, 1 };
static u32 dba_band_count = 1;
static u32 dba_bw_unit = 46;
static u32 dba_max_alloc_ids = 128;
static u32 dba_cfg3 = 1;
static u32 dba_max_frame = 1500;
static u32 dba_alloc_mask = 0xFFFF;
#endif

u32 npu_reset_pending = 1;
u32 npu_printf_prefix = 1;

#ifdef HAS_DBA
static u8 dba_cfg4 = 1;
#endif

#ifdef HAS_TUNNEL
static u32 tunnel_max_count = 7;
static u32 tunnel_timeout_ms = 100;
static u8 tunnel_cfg_flag = 1;
#endif

static s32 sentinel_1 = -1;

#ifdef HAS_DBA
static u16 dba_cfg_word0 = 25;
static u16 dba_cfg_word1 = 10;
#endif

static s32 sentinel_2 = -1;

#ifdef HAS_TUNNEL
static u32 tunnel_dispatch_ptr;
#endif

static s32 sentinel_3 = -1;
static u32 config_flags = 0x00020000;

static const char delay_name[] = "__delay";

/* ================================================================
 * Global variables - .bss section (zero-initialized)
 *
 * Order must match BSS layout for correct GP-relative access.
 * ================================================================ */

/* mailbox handler slots */
static mbox_handler_t mbox_core_handlers[6];

/* printf subsystem */
static u8 printf_cfg_flags[4];
static u8 printf_desc[8];
static u8 printf_flag;

/* timer subsystem */
static u32 timer_context[10];
static u32 timer_ref_counts[8];
static u8 srv6_my_ipv6[16];
static u32 timer_isr_context[12];
static isr_fn_t timer_isr_handler0;
static isr_fn_t timer_isr_handler1;
static u32 timer_isr_pad[4];
static isr_fn_t timer_isr_handler2;
static u32 timer_pad2[100];
static isr_fn_t timer_callback;
static u32 timer_pad3;

/* printf buffer */
static char printf_buf[1024];

/* tunnel context */
#ifdef HAS_TUNNEL
static u8 tunnel_ctx[3][1024];
#endif

/* BME/bridge */
static u32 bme_base_addr;
static u32 bme_pad[38];
static u32 bme_config;
static u32 bme_pad2[10];
static u32 bme_desc_count;
static u32 bme_status;
static u32 bme_pad3;
static u32 bme_ring_state[4];
static u32 tdma_bme_dscp_idx;
static u32 tdma_bme_dscp_base;
static u32 npu_bridge_base;

/* PLIC ISR table: 192 function pointers */
static isr_fn_t plic_isr_table[192];

/* PLIC config */
static u32 plic_cfg[3];
static u32 printf_mutex_desc[2];

/* SRAM buffer manager */
static u32 sram_buf_mutex[2];
static u32 sram_buf_pad[4];
static u32 sram_buf_max_use;
static u32 sram_buf_cur_idx;
static u16 sram_buf_entries[200];
static u32 sram_buf_pad2[8];

/* WiFi state */
static u32 wifi_state[256];

#ifdef HAS_TUNNEL
static u8 tunnel_srv6_hdr_len[8];
#endif

/* DBA state */
#ifdef HAS_DBA
static u32 dba_state[128];
static u32 npu_fttr_base;
static u8 dba_band_switch;
static u8 dba_bwmap_switch;
static u32 dba_alloc_state[20];
static u32 dba_timer0_snap;
#endif

/* PLIC threshold per-hart */
static u8 plic_threshold_table[8];
static u32 plic_isr_init_done;
static u32 plic_cfg2;
static u32 plic_state;

/* npu_init sync */
static u32 mib12_snapshot;
static u32 core_sync_flag;

/* WiFi extended state */
static u32 wifi_ext_state[32];

/* Reorder node pools (2000 pri + 5000 sec, 36 bytes each) */
static u32 reorder_pri_node_base;
static u32 reorder_sec_node_base;
static u32 reorder_pri_idx_pool;
static u32 reorder_sec_idx_pool;
static u16 reorder_pri_widx;
static u16 reorder_pri_ridx;
static u16 reorder_sec_widx;
static u16 reorder_sec_ridx;
static u32 reorder_alloc_mutex[2];
static u32 reorder_free_mutex[2];

/* BA (Block Ack) reorder window */
static u32 ba_mutex_5g[2];
static u32 ba_mutex_2g[2];
static u8 wifi_dbdc_mode;
static u32 ba_table_a;
static u32 ba_table_b;

/* Bridge/BME state */
static u8 bme_path_enable;
static u8 pipeline_5g_ready;
static u8 pipeline_2g_ready;
static u32 fwd_dispatch_table[32];

/* PCIe bases */
static u32 pcie_base_5g;
static u32 pcie_base_2g;

/* RXD state */
static u32 rxd_base_5g;
static u32 rxd_5g_init_done;
static u32 rxd_base_2g;
static u32 rxd_2g_bufid_base;
static u32 rxd_2g_init_done;
static u32 rxd_5g_cpu_idx;
static u32 rxd_5g_mirror;
static u32 rxd_2g_cpu_idx;
static u32 rxd_2g_mirror;
static u16 rxd_5g_bufid_table[1536];

/* WiFi misc (declared early for reorder/BA functions) */
static u32 wifi_base_cfg_val;
static u8 wifi_debug_flags;
static u32 wifi_buf_id_base;

/* WiFi per-port state (16 ports) */
static u8 wifi_port_state_2g[16];
static u8 wifi_port_state_5g[16];

/* WiFi global counters (u64 as lo/hi pairs) */
static u32 wifi_global_bytes_lo;
static u32 wifi_global_bytes_hi;
static u32 wifi_global_pkts_lo;
static u32 wifi_global_pkts_hi;
static u32 wifi_global_bytes_5g_lo;
static u32 wifi_global_bytes_5g_hi;
static u32 wifi_global_pkts_5g_lo;
static u32 wifi_global_pkts_5g_hi;

/* WiFi per-TID counters (256 entries × u64 as lo/hi pairs) */
static u32 wifi_tid_pkt_cnt[512];
static u32 wifi_tid_byte_cnt[512];

/* WiFi classifier state */
static u8 wifi_classifier_bypass;

/* WiFi multi-desc state */
static u32 wifi_rxd_ring_2g;
static u32 wifi_rxd_ring_5g;
static u32 wifi_rxd_idx_2g;
static u32 wifi_rxd_idx_5g;
static u16 *wifi_rxd_bufid_tbl;
static u8 wifi_retry_limit;

/* WiFi init state */
static u8 wifi_driver_model;
static u8 wifi_pcie_port_type;
static u8 wifi_band_cap;
static u8 wifi_force_to_cpu;
static u8 wifi_no_ba_test;
static u8 wifi_band0_on_cpu;
static u16 wifi_flushall_timeout;
static u16 wifi_flushone_timeout;
static u32 wifi_pkt_buf_addr;
static u32 wifi_dram_ba_node_addr;
static u32 wifi_pcie_desc_base;

/* WiFi per-band pkt queue state */
static u16 pkt_queue_widx_2g;
static u16 pkt_queue_widx_5g;
static u32 pkt_queue_base_2g;
static u32 pkt_queue_base_5g;
static u16 pkt_queue_rx_widx_2g;
static u16 pkt_queue_rx_ridx_2g;
static u32 pkt_queue_rx_base_2g;
static u32 pkt_queue_rx_base_5g;

/* WiFi per-band queue mutexes */
static u32 queue_mutex_2g[2];
static u32 queue_mutex_5g[2];
static u32 queue_mutex_rx_2g[2];
static u32 queue_mutex_rx_5g[2];

/* BA node management */
static u32 ba_node_pool_base;

/* WiFi per-port wait state (16 ports × 2 bands) */
static u8 wifi_wait_state_2g[16];
static u8 wifi_wait_state_5g[16];

/* WiFi per-port band assignment */
static u8 wifi_port_band_2g[16];
static u8 wifi_port_band_5g[16];

/* WiFi pipeline pkt queue */
static u32 wifi_pipeline_queue_2g;
static u32 wifi_pipeline_queue_5g;

/* ================================================================
 * Utility functions
 * ================================================================ */

void *npu_memset(void *dst, int c, u32 n)
{
	u8 *d = (u8 *)dst;

	while (n--)
		*d++ = (u8)c;
	return dst;
}

void *npu_memcpy(void *dst, const void *src, u32 n)
{
	u8 *d = (u8 *)dst;
	const u8 *s = (const u8 *)src;

	while (n--)
		*d++ = *s++;
	return dst;
}

static u32 npu_strlen(const char *s)
{
	const char *p = s;

	while (*p)
		p++;
	return (u32)(p - s);
}

static char get_core_char(void)
{
	u32 id = get_hartid();

	if (id <= 7)
		return (char)('0' + id);
	return 'X';
}

/* ================================================================
 * Hardware mutex
 * ================================================================ */

static void hw_mutex_lock(u32 *desc)
{
	u32 idx = (desc[0] & HW_MUTEX_IDX_MASK) >> 2;
	u32 hart = get_hartid();
	u32 val = (hart << 8) | 0x40;

	REG32(HW_MUTEX_ACQ(idx)) = val;
	while (!(REG32(HW_MUTEX_STATUS(hart, idx)) & 0x10000))
		REG32(HW_MUTEX_ACQ(idx)) = val;
}

static void hw_mutex_unlock(u32 *desc)
{
	u32 idx = (desc[0] & HW_MUTEX_IDX_MASK) >> 2;
	u32 hart = get_hartid();

	REG32(HW_MUTEX_REL(hart, idx)) = (hart << 8);
}

static void hw_mutex_lock_pri(u32 *desc)
{
	u32 idx = (desc[0] & HW_MUTEX_IDX_MASK) >> 2;
	u32 hart = get_hartid();
	u32 val;

	if (desc[1] != 0)
		val = (hart << 8) | 0x10040;
	else
		val = (hart << 8) | 0x40;

	REG32(HW_MUTEX_ACQ(idx)) = val;
	while (!(REG32(HW_MUTEX_STATUS(hart, idx)) & 0x10000))
		REG32(HW_MUTEX_ACQ(idx)) = val;
}

static void hw_mutex_unlock_pri(u32 *desc)
{
	hw_mutex_unlock(desc);
}

/* ================================================================
 * PLIC (Platform-Level Interrupt Controller)
 * ================================================================ */

static void default_isr(int src)
{
	npu_printf("(%s) implement code to clear intrSrc:%d then execute ISR here\n",
		   "default_isr", src);
}

void call_isr_by_src(u32 src)
{
	get_hartid();
	if (src > 191) {
		npu_printf("%s just return due to intSrc:%d > %d\n",
			   "call_isr_bySrc", src, 191);
		return;
	}
	plic_isr_table[src](src);
	REG32(PLIC_CLAIM_REG) = src + 1;
}

static void plic_set_threshold(void)
{
	u32 hart = get_hartid();

	REG32(PLIC_THRESHOLD_REG) = plic_threshold_table[hart] & 0x1F;
}

static u32 plic_get_priority(u32 src)
{
	if (src > 191) {
		npu_printf("%s just return due to intSrc:%d > %d\n",
			   "PLIC_get_priority_bySrc", src, 191);
		return 16;
	}
	return REG32(PLIC_PRIORITY(src)) & 0x1F;
}

static void plic_set_priority(u32 src, u8 prio)
{
	if (src > 191) {
		npu_printf("%s just return due to intSrc:%d > %d\n",
			   "PLIC_set_priority_bySrc", src, 191);
		return;
	}
	REG32(PLIC_PRIORITY(src)) = prio & 0x1F;
}

static void plic_enable(u32 src)
{
	u32 word_idx, bit;

	if (src > 191) {
		npu_printf("%s just return due to intSrc:%d > %d\n",
			   "PLIC_enable_intSrc_toOneCore", src, 191);
		return;
	}
	word_idx = (src + 1) >> 5;
	bit = 1u << ((src + 1) & 0x1F);
	REG32(PLIC_MASK_REG(word_idx)) &= ~bit;
	REG32(PLIC_ENABLE_REG(word_idx)) |= bit;
}

static void plic_disable(u32 src)
{
	u32 word_idx, bit;

	if (src > 191) {
		npu_printf("%s just return due to intSrc:%d > %d\n",
			   "PLIC_DISABLE_intSrc_toOneCore", src, 191);
		return;
	}
	word_idx = (src + 1) >> 5;
	bit = 1u << ((src + 1) & 0x1F);
	REG32(PLIC_MASK_REG(word_idx)) |= bit;
	REG32(PLIC_ENABLE_REG(word_idx)) &= ~bit;
}

static void plic_enable_wrapper(u32 src)
{
	plic_enable(src);
}

static void plic_init(void)
{
	u32 i;

	plic_set_threshold();

	for (i = 0; i < 192; i++) {
		if (i == 95)
			REG32(PLIC_PRIORITY(95)) = 17;
		else
			REG32(PLIC_PRIORITY(i)) = 16;

		if (get_hartid() == 0 && plic_isr_init_done == 0)
			plic_isr_table[i] = default_isr;
	}

	if (get_hartid() == 0)
		plic_isr_init_done = 1;
}

static void plic_register_isr(u32 src, isr_fn_t handler)
{
	if (src < 192) {
		plic_isr_table[src] = handler;
		plic_enable(src);
	}
}

/* ================================================================
 * Timer
 * ================================================================ */

static u32 timer_get_bit(u32 src)
{
	u32 idx;

	if (src >= 24 && src <= 31)
		idx = src - 24 + 8;
	else
		idx = src - 16;

	if (idx < 16)
		return timer_bit_map[idx];
	return 0;
}

static void timer_isr(int src)
{
	u32 bit = timer_get_bit((u32)src);
	u32 base;

	if (src >= 24 && src <= 31)
		base = NPU_TIMER1_BASE;
	else
		base = NPU_TIMER0_BASE;

	/* clear timer interrupt */
	REG32(base) |= (1u << bit);
	REG32(base) &= ~(1u << bit);
}

static void timer_init(int timer, int enable, int period)
{
	u32 base = NPU_TIMER0_BASE;
	u32 clk_mhz = 25;
	u32 reload;

	if (!enable) {
		REG32(base) &= ~(1u << timer);
		return;
	}

	reload = (u32)((u64)1000 * period * clk_mhz / 100);
	REG32(base + 0x10 + timer * 4) = reload;
	REG32(base) |= (1u << timer);
}

static void delay_1ms(u32 ms)
{
	volatile u32 i, j;

	for (i = 0; i < ms; i++)
		for (j = 0; j < 25000; j++)
			;
}

/* ================================================================
 * vsprintf (minimal implementation)
 * ================================================================ */

static int npu_vsprintf(char *buf, const char *fmt, u32 *args)
{
	char *p = buf;
	const char *f = fmt;
	int arg_idx = 0;
	char tmp[12];
	int i, len;

	while (*f) {
		if (*f != '%') {
			*p++ = *f++;
			continue;
		}
		f++;

		/* flags */
		int pad_zero = 0, left = 0, width = 0, is_long = 0;

		if (*f == '-') { left = 1; f++; }
		if (*f == '0') { pad_zero = 1; f++; }
		while (*f >= '0' && *f <= '9')
			width = width * 10 + (*f++ - '0');
		if (*f == 'l') { is_long = 1; f++; }
		if (*f == 'l') { f++; } /* ignore ll */

		switch (*f) {
		case 'd': {
			s32 v = (s32)args[arg_idx++];
			int neg = 0;
			u32 uv;

			if (v < 0) { neg = 1; uv = (u32)(-v); }
			else uv = (u32)v;

			len = 0;
			do { tmp[len++] = '0' + (uv % 10); uv /= 10; } while (uv);
			if (neg) tmp[len++] = '-';
			for (i = 0; i < width - len; i++)
				*p++ = pad_zero ? '0' : ' ';
			for (i = len - 1; i >= 0; i--)
				*p++ = tmp[i];
			break;
		}
		case 'u': {
			u32 v = args[arg_idx++];

			len = 0;
			do { tmp[len++] = '0' + (v % 10); v /= 10; } while (v);
			for (i = 0; i < width - len; i++)
				*p++ = pad_zero ? '0' : ' ';
			for (i = len - 1; i >= 0; i--)
				*p++ = tmp[i];
			break;
		}
		case 'x':
		case 'X': {
			u32 v = args[arg_idx++];
			const char *hex = (*f == 'X') ?
				"0123456789ABCDEF" : "0123456789abcdef";

			len = 0;
			do { tmp[len++] = hex[v & 0xF]; v >>= 4; } while (v);
			for (i = 0; i < width - len; i++)
				*p++ = pad_zero ? '0' : ' ';
			for (i = len - 1; i >= 0; i--)
				*p++ = tmp[i];
			break;
		}
		case 's': {
			const char *s = (const char *)args[arg_idx++];

			if (!s) s = "(null)";
			while (*s)
				*p++ = *s++;
			break;
		}
		case 'c':
			*p++ = (char)args[arg_idx++];
			break;
		case 'p': {
			u32 v = args[arg_idx++];
			const char *hex = "0123456789abcdef";

			*p++ = '0'; *p++ = 'x';
			for (i = 7; i >= 0; i--)
				*p++ = hex[(v >> (i * 4)) & 0xF];
			break;
		}
		case '%':
			*p++ = '%';
			break;
		default:
			*p++ = '%';
			*p++ = *f;
			break;
		}
		f++;
	}
	*p = '\0';
	return (int)(p - buf);
}

/* ================================================================
 * UART / Printf
 * ================================================================ */

static void uart_putc(char c)
{
	while (!(REG32(UART_TX_STATUS) & UART_TX_READY_BIT))
		;
	REG32(UART_TX_DATA) = (u32)c;
}

static void uart_puts_raw(const char *s)
{
	while (*s) {
		if (*s == '\n') {
			uart_putc('\r');
		}
		uart_putc(*s++);
	}
}

static void printf_enable(int enable)
{
	npu_printf_prefix = (u32)enable;
}

int npu_printf(const char *fmt, ...)
{
	int len;
	u32 *args;

	/* check if printing is suppressed by host */
	if (REG32(NPU_MIB(21)) != 0)
		return 0;

	irq_disable();
	hw_mutex_lock_pri(printf_mutex_desc);

	/* format into buffer */
	args = (u32 *)&fmt + 1;
	len = npu_vsprintf(printf_buf, fmt, args);

	/* print core prefix if enabled */
	if (npu_printf_prefix) {
		uart_putc('[');
		uart_putc('C');
		uart_putc(get_core_char());
		uart_putc(']');
	}

	/* output with LF→CRLF conversion */
	uart_puts_raw(printf_buf);

	hw_mutex_unlock_pri(printf_mutex_desc);
	irq_enable();

	return len;
}

/* boot printf (uses boot UART at 0x1EC10000) */
static int boot_printf(const char *fmt, ...)
{
	int len;
	u32 *args;
	char *p;

	hw_mutex_lock_pri(printf_mutex_desc);

	args = (u32 *)&fmt + 1;
	len = npu_vsprintf(printf_buf, fmt, args);

	/* output core prefix */
	while (!(REG32(BOOT_UART_STATUS) & UART_TX_READY_BIT)) ;
	REG32(BOOT_UART_TX) = '[';
	while (!(REG32(BOOT_UART_STATUS) & UART_TX_READY_BIT)) ;
	REG32(BOOT_UART_TX) = 'C';
	while (!(REG32(BOOT_UART_STATUS) & UART_TX_READY_BIT)) ;
	REG32(BOOT_UART_TX) = get_core_char();
	while (!(REG32(BOOT_UART_STATUS) & UART_TX_READY_BIT)) ;
	REG32(BOOT_UART_TX) = ']';

	/* output buffer */
	p = printf_buf;
	while (*p) {
		if (*p == '\n') {
			while (!(REG32(BOOT_UART_STATUS) & UART_TX_READY_BIT)) ;
			REG32(BOOT_UART_TX) = '\r';
		}
		while (!(REG32(BOOT_UART_STATUS) & UART_TX_READY_BIT)) ;
		REG32(BOOT_UART_TX) = *p++;
	}

	hw_mutex_unlock_pri(printf_mutex_desc);
	return len;
}

/* ================================================================
 * Mailbox communication
 *
 * 80 bytes per core in dispatch table:
 *   [0..63]  = raw data DWORDs (16 slots, indexed by func_id)
 *   [32..63] = raw data WORDs (overlaps with above)
 *   [48..79] = callback function pointers (8 slots)
 * ================================================================ */

/* mailbox dispatch table: 6 cores x 80 bytes */
static u8 mbox_dispatch[MAX_CORE_NUM][80];

/* mailbox handler slots registered during init */
static mbox_handler_t mbox_wifi_handler;
static mbox_handler_t mbox_wifi_handler2;
static mbox_handler_t mbox_notify_handler;
static mbox_handler_t mbox_cfg_handler;

static void mbox_isr(int src)
{
	u32 mbox_idx = (u32)src - 8;
	u32 rptr, base_ptr, max_cnt, func_idx;
	mbox_handler_t handler;

	if ((u8)mbox_idx != mbox_idx) {
		npu_printf("Error: core_id:%d != mbox_idx:%d\n",
			   mbox_idx, (u8)mbox_idx);
		return;
	}

	/* clear interrupt (write-1-to-clear) */
	REG32(MBOX_INT_STS) = (1u << mbox_idx);

	/* verify clear */
	if ((REG32(MBOX_INT_STS) >> mbox_idx) & 1) {
		npu_printf("Error(%s): cleaning mbox intr isn't done (mbox_status:0x%x, mbox_idx:%d)\n",
			   "mBox_isr", REG32(MBOX_INT_STS), (u8)mbox_idx);
		return;
	}

	/* read queue registers */
	base_ptr = REG32(MBQ_BASE_PTR(mbox_idx));
	max_cnt = REG32(MBQ_MAX_CNT(mbox_idx));
	rptr = REG32(MBQ_RPTR(mbox_idx));

	/* set response status if not already set */
	if (!(rptr & 1)) {
		rptr = (rptr & 0xFFFFFF00u) | (rptr & 0xE1) | 0x6;
		REG32(MBQ_RPTR(mbox_idx)) = rptr;
	}

	func_idx = (rptr >> 11) & 0xF;

	if (rptr & 0x20) {
		/* raw data path: store base_ptr and max_cnt */
		u32 *data = (u32 *)&mbox_dispatch[mbox_idx][0];
		u16 *cnt = (u16 *)&mbox_dispatch[mbox_idx][32];

		data[func_idx] = base_ptr;
		cnt[func_idx] = (u16)max_cnt;
	} else {
		/* callback path: dispatch to handler */
		u32 *callbacks = (u32 *)&mbox_dispatch[mbox_idx][48];

		handler = (mbox_handler_t)(void *)callbacks[func_idx];
		if (handler) {
			u32 ret = (u32)handler(base_ptr, max_cnt);
			rptr = (rptr & 0xFFFFFFE3u) | ((ret & 7) << 2);
		}

		/* signal completion */
		if (rptr & 1)
			REG32(MBQ_RPTR(mbox_idx)) = (rptr & ~2u) | 2;
	}
}

static void mailbox_init(void)
{
	u32 i;
	u32 *callbacks;

	/* enable per-core mailbox interrupts */
	for (i = 0; i < MAX_CORE_NUM; i++) {
		REG32(NPU_MBOX_BASE + 0x008 + i * 4) = (1u << i);
		plic_register_isr(8 + i, mbox_isr);
	}

	REG32(MBOX_INT_MASK0) = 256;
	plic_cfg[0] = 14;
	plic_state = 0;

	/* zero all dispatch tables */
	for (i = 0; i < MAX_CORE_NUM; i++)
		npu_memset(mbox_dispatch[i], 0, 80);

	/* register default handlers into core 0's callback slots */
	callbacks = (u32 *)&mbox_dispatch[0][48];
	callbacks[0] = (u32)(void *)mbox_wifi_handler;
	callbacks[1] = (u32)(void *)mbox_wifi_handler2;
	callbacks[5] = (u32)(void *)mbox_cfg_handler;

	/* register handler in last core's callback slot */
	callbacks = (u32 *)&mbox_dispatch[MAX_CORE_NUM - 1][48];
	callbacks[3] = (u32)(void *)mbox_notify_handler;
}

/* notify host via mailbox queue 8 */
static int mbox_notify_host(u32 base_ptr, u32 max_cnt, u32 func_id)
{
	u32 timeout = 30;
	u32 rptr;

	hw_mutex_lock_pri(printf_mutex_desc);

	REG32(MBQ_BASE_PTR(8)) = base_ptr;
	REG32(MBQ_MAX_CNT(8)) = max_cnt;
	REG32(MBQ_RPTR(8)) = (func_id << 11) | 1;
	REG32(MBQ_WPTR(8)) = 1;

	while (timeout--) {
		rptr = REG32(MBQ_RPTR(8));
		if (rptr & 2)
			break;
	}

	hw_mutex_unlock_pri(printf_mutex_desc);
	return (rptr & 2) ? 0 : -1;
}

/* ================================================================
 * Chip ID and module detection
 * ================================================================ */

static u32 chip_id;
static u32 chip_rev;

static void chip_id_query(void)
{
	chip_id = REG32(CHIP_ID_REG);
	chip_rev = REG32(CHIP_VARIANT_REG);

	npu_printf("chip_id=0x%x, chip_rev=0x%x\n", chip_id, chip_rev);
}

/* USB power down */
static void usb_powerdown(void)
{
	/* USB PHY power control */
	REG32(0x1FAC0700) |= (1u << 4);
	REG32(0x1FAE0700) |= (1u << 4);
}

/* Reboot */
static void npu_reboot(void)
{
	npu_printf("REBOOTING...\n");
	REG32(0x1FB00040) = 0x80000001;
}

/* ================================================================
 * SRAM buffer management
 *
 * 512KB SRAM at 0x3E800000, bump-allocated with alignment.
 * Alloc table: 100 entries of {u16 addr_type, u16 pad, u32 base}.
 * Size lookup via per-type descriptors in .rodata.
 * ================================================================ */

#define SRAM_BASE         0x3E800000
#define SRAM_END          0x3E87FFFE
#define SRAM_SIZE         0x80000
#define SRAM_MAX_ENTRIES  100
#define SRAM_ERROR_ADDR   0x3E880000

static u32 sram_alloc_offset;
static u32 sram_alloc_count;
static u32 sram_alloc_calls;
static u16 sram_alloc_table[SRAM_MAX_ENTRIES * 4];

/* SRAM region type descriptors: {u16 addr_type, u8 align_class, u8 pad, u32 size} */
struct sram_region_desc {
	u16 addr_type;
	u8 align;
	u8 pad;
	u32 size;
};

static u32 sram_buf_alloc_impl(u16 addr_type, u32 size_class)
{
	u32 i;
	u16 *entry;
	u8 *base;
	u32 offset;

	hw_mutex_lock(sram_buf_mutex);

	if (sram_alloc_offset >= SRAM_SIZE && sram_alloc_count > 99) {
		hw_mutex_unlock(sram_buf_mutex);
		npu_printf("sram is over the max!!current para:AddrType=%d,idx=%d,tmp_restore_index=%d\n",
			   addr_type, sram_alloc_offset, sram_alloc_count);
		return SRAM_ERROR_ADDR;
	}

	/* check if already allocated */
	if (sram_alloc_count > 0) {
		for (i = 0; i < sram_alloc_count; i++) {
			if (sram_alloc_table[i * 4] == addr_type) {
				u32 existing = *(u32 *)&sram_alloc_table[i * 4 + 2];

				npu_printf("already exist!!AddrType=%d,npu_init_sram_addr=%x\n",
					   addr_type, existing);
				hw_mutex_unlock(sram_buf_mutex);
				return existing;
			}
		}
	}

	/* bump-allocate with alignment */
	offset = sram_alloc_offset;
	base = (u8 *)SRAM_BASE;
	if (offset != 0) {
		base += offset;
		/* align to 16 or 32 bytes depending on type */
		if ((u32)base & 0x1F)
			base = (u8 *)(((u32)base + 0x1F) & ~0x1F);
		if ((u32)base > SRAM_END) {
			npu_printf("alloc fail!!current para:AddrType=%d,idx=%x,tmp_restore_index=%d,npu_init_sram_addr=%x\n",
				   addr_type, sram_alloc_offset, sram_alloc_count, (u32)base);
			hw_mutex_unlock(sram_buf_mutex);
			return SRAM_ERROR_ADDR;
		}
	} else {
		base = (u8 *)SRAM_BASE;
	}

	/* record entry */
	entry = &sram_alloc_table[sram_alloc_count * 4];
	entry[0] = addr_type;
	*(u32 *)(entry + 2) = (u32)base;
	sram_alloc_offset = (u32)base - SRAM_BASE + size_class;
	sram_alloc_count++;

	hw_mutex_unlock(sram_buf_mutex);
	return (u32)base;
}

static u32 sram_buf_alloc(u32 addr_type)
{
	u32 result;

	if (addr_type > 256)
		return 0;
	result = sram_buf_alloc_impl((u16)addr_type, addr_type);
	if (result == SRAM_ERROR_ADDR)
		return 0;
	return result;
}

static void sram_buf_init(void)
{
	npu_memset((void *)SRAM_BASE, 0, SRAM_SIZE);
	npu_memset(sram_alloc_table, 0, sizeof(sram_alloc_table));
	sram_buf_mutex[0] = 2;
	sram_buf_mutex[1] = 0;
	sram_alloc_offset = 0;
	sram_alloc_count = 0;
}

static void sram_buf_dump(void)
{
	u32 i;
	u16 *entry = sram_alloc_table;

	npu_printf("base:%x,total size:%x,current max use size:%x\n",
		   SRAM_BASE, SRAM_SIZE, sram_alloc_offset);
	for (i = 0; i < SRAM_MAX_ENTRIES; i++) {
		u32 addr = *(u32 *)(entry + 2);

		if (addr == 0)
			break;
		npu_printf("Idx=%d,AddrType=%d,baseaddr=%x\n",
			   i, (u32)entry[0], addr);
		entry += 4;
	}
}

/* ================================================================
 * Debug counter ISR
 * ================================================================ */

static void dbg_cnt_isr(int src)
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

/* buf_id recycle ring (return freed IDs to alloc pool) */
static u32 bufid_free_mutex[2];
static u16 bufid_free_widx;
static u32 bufid_free_base;
static u32 bufid_free_count;

static void bufid_ring_mutex_init(void)
{
	bufid_free_mutex[0] = 13;
	bufid_free_mutex[1] = 0;
}

static void buf_id_return(u16 buf_id)
{
	hw_mutex_lock(bufid_free_mutex);
	*(u16 *)(bufid_free_base + 2 * bufid_free_widx) = buf_id;
	bufid_free_count++;
	if ((wifi_debug_flags & 4) && counter_base_tri)
		(*(u32 *)(counter_base_tri + 0x18))++;
	bufid_free_widx = (bufid_free_widx + 1 == 5600) ?
		0 : bufid_free_widx + 1;
	hw_mutex_unlock(bufid_free_mutex);
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
 * NPU bridge (DMA channels between NPU and host)
 * ================================================================ */

#define NPU_BRIDGE_REG_BASE    0x1EC12000
#define BRIDGE_CH_CTRL(ch)     (NPU_BRIDGE_REG_BASE + 0x010 + (ch) * 0x10)
#define BRIDGE_CH_STATUS(ch)   (NPU_BRIDGE_REG_BASE + 0x050 + (ch) * 0x20)
#define BRIDGE_PKT_BUF_BASE_REG 0x1EC12010
#define BRIDGE_PKT_BUF_CFG    0x1EC12018

static u32 npu_bridge_pkt_base;
static u32 bridge_tx_count[4];

static u32 npu_bridge_addr(void)
{
	return npu_bridge_base + 0x10000;
}

static void npu_bridge_buf_init(void)
{
	u32 i;
	u32 *ch_status;

	npu_bridge_base = sram_buf_alloc(129);
	npu_printf("npuBridgeBase=%x\n", npu_bridge_base);
	npu_bridge_pkt_base = npu_bridge_base;
	npu_printf("%s: NPU_BRIDGE_BASE_PACKET_BUFFER(0x%x)=0x%x, NPU_BRIDGE_PACKET_BUF_SIZE:0x%x\n",
		   "npu_bridge_buf_init",
		   (u32)&npu_bridge_pkt_base,
		   npu_bridge_base,
		   0x10000);

	REG32(BRIDGE_PKT_BUF_BASE_REG) = 264192;
	REG32(BRIDGE_PKT_BUF_CFG) = 1;

	delay_1ms(10);

	for (i = 0; i < 4; i++) {
		ch_status = (u32 *)(0x1EC12210 + i * 16);
		if (*ch_status & 1)
			npu_printf("npu bridge channel-%d buf init sucess\n", i);
		else
			npu_printf("npu bridge channel-%d buf init fail\n", i);
		*ch_status = 1;
	}
}

static int npu_bridge_ingress(u32 ch, u32 *pkt_ptr, u32 *desc_ptr)
{
	u32 cnt;

	cnt = bridge_tx_count[ch];
	if (cnt == 0) {
		cnt = (u8)REG32(0x1EC12050 + ch * 4);
		bridge_tx_count[ch] = cnt;
		if (cnt == 0)
			return -1;
	}

	*desc_ptr = REG32(0x1EC12080 + ch * 16) | 0x20000000;
	*pkt_ptr = *(volatile u32 *)(*desc_ptr) + 32;
	bridge_tx_count[ch] = cnt - 1;
	return 0;
}

static int npu_bridge_egress(u32 ch, u32 w0, u32 w1, u32 w2,
			     u32 w3, u32 w4, u32 w5, u32 w6)
{
	u32 base;

	if ((REG32(0x1EC12050 + ch * 4) & 0xFF00) == 0) {
		npu_printf("npu bridge egress fail, channel-%d, epkt_info_w0=%x, epkt_info_w1=%x, epkt_info_w2=0x%x \n",
			   ch, w0, w1, w2);
		return -1;
	}

	base = 0x1EC12100 + ch * 32;
	REG32(base + 0x04) = w1;
	REG32(base + 0x08) = w2;
	REG32(base + 0x0C) = w3;
	REG32(base + 0x10) = w4;
	REG32(base + 0x14) = w5;
	REG32(base + 0x18) = w6;
	REG32(base + 0x00) = w0;
	return 0;
}

static int npu_bridge_send(u32 ch, u32 w0, u32 fwd, u32 len, u32 flags)
{
	if (flags != 0)
		return 0;
	return npu_bridge_egress(ch, w0, len << 16,
				 (ch & 7) | 0xC0000000 | ((fwd != 0) << 24),
				 0, 0, 0, 0);
}

static int npu_bridge_send_direct(u32 ch, u32 data, u32 len)
{
	return npu_bridge_egress(ch, data, len << 16,
				 (ch & 7) | 0xC2000000,
				 0, 0, 0, 0);
}

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

/* BME descriptor state */
static u32 tdma_bme_dscp_base_addr;

/* TDMA init: zero 512KB SRAM, clear alloc table */
static void tdma_init(void)
{
	sram_buf_init();
}

#ifdef WIFI_KITE
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
	REG32(0x1EC0B80C) = tdma_bme_dscp_base_addr;
	npu_printf("tdma_bme_dscp =%x\n", tdma_bme_dscp_base_addr);

	/* zero 512 8-byte descriptors (4KB) */
	desc = (u32 *)tdma_bme_dscp_base_addr;
	for (i = 0; i < 512; i++) {
		desc[i * 2] = 0;
		desc[i * 2 + 1] = 0;
	}

	REG32(0x1EC0B800) = 511;
	REG32(0x1EC0B808) = 0;

	/* register BME done ISR on PLIC source 32 */
	plic_register_isr(32, bme_done_isr);
	npu_printf("bme plic register done, INTR_BUFID_MOVE_ENGINE(%d)\n", 32);

	/* configure BME: interrupt mode + SRAM mode, ID threshold */
	REG32(0x1EC0B824) = (REG32(0x1EC0B824) & 0xFFFF0000) | 0x10;
	npu_printf("BME intMode + sarm mode, id thld=%x\n", REG32(0x1EC0B824));

	/* timeout */
	REG32(0x1EC0B818) = 1000;
	npu_printf("BME timeout setting%x = %x\n", 0x1EC0B818, 1000);

	/* global config: enable with size=8 */
	{
		u32 cfg = REG32(0x1EC0B814);

		npu_printf("[%s] defult: BME_CSR_GLB_CFG(%x)=%x\n",
			   "tdma_bme_init", 0x1EC0B814, cfg);
		npu_printf(" csr_mng_en enable");
		cfg = (cfg & 0xFFE0) | 0x170007;
		REG32(0x1EC0B814) = cfg;
		npu_printf("[%s] configed: BME_CSR_GLB_CFG(%x), bme size=%d\n",
			   "tdma_bme_init", cfg, 8);
	}
}

/* TDMA BME init: hardware buffer-ID allocator path */
static void tdma_bmgr_init(void)
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

	while (!(REG32(BMGR_BASE + 0x02C) & 1))
		;

	plic_enable_wrapper(33);
}

/* TDMA TX init: configure TX descriptor rings */
static void tdma_tx_init(void)
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
	REG32(TDMA_WIFI_BUF_CFG) = (REG32(TDMA_WIFI_BUF_CFG) & 0xFFA200FF) | 0x590100;
	npu_printf("[%s] PPE_WIFI_BUF_CFG=%x, value=%x\n",
		   "tdma_tx_init", TDMA_WIFI_BUF_CFG, REG32(TDMA_WIFI_BUF_CFG));

	/* init BME */
	tdma_bme_init();
}
#endif /* WIFI_KITE */

/* Software buffer manager init (non-TDMA path) */
static void buf_mgr_init(void)
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

static int hostadpt_init(void)
{
	/* wait for host to configure RX DMA pointer */
	while (REG32(HOSTADPT_RX_DMA_PTR) == 0)
		;

	hostadpt_tx_ring_base = (REG32(HOSTADPT_TX_DMA_PTR) & 0x3FFFFFFF) | 0x40000000;
	hostadpt_tx_ring_ready = 1;
	hostadpt_rx_ring_base = (REG32(HOSTADPT_RX_DMA_PTR) & 0x3FFFFFFF) | 0x40000000;
	return 0;
}

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

	hw_idx = REG32(0x1EC0B808);
	idx = tdma_bme_dscp_idx;

	if (((idx + 1) & 0x1FF) == hw_idx)
		return -1;

	desc_addr = desc_base + idx * 8;
	*(volatile u32 *)desc_addr = phys_addr;
	*(volatile u32 *)(desc_addr + 4) =
		(pkt_size & 0x3FFF) | ((band & 3) << 14) | (buf_id << 16);

	tdma_bme_dscp_idx = (idx + 1) & 0x1FF;
	REG32(0x1EC0B804) = tdma_bme_dscp_idx;

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
			    (wifi_buf_id_base & 0x3FFFFFFF | 0x40000000) +
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
		   (wifi_buf_id_base & 0x3FFFFFFF | 0x40000000);
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
			(wifi_buf_id_base & 0x3FFFFFFF | 0x40000000);
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
		*(u32 *)(node + 32) = counter_base_2g;

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
	u32 desc = (wifi_buf_id_base & 0x3FFFFFFF | 0x40000000) +
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
							(wifi_buf_id_base &
							 0x3FFFFFFF |
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
			u32 dst = (wifi_buf_id_base & 0x3FFFFFFF |
				   0x40000000) + (new_buf_id << 12);

			for (i = 0; i < count; i++) {
				u32 d = ring_base + idx * 16;
				u16 old_id = *(s16 *)(bufid_tbl + (u32)idx);
				u32 src = (wifi_buf_id_base & 0x3FFFFFFF |
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

static void npu_set_pcie_base(u32 addr, u32 band)
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
			((new_buf_id << 12) + wifi_buf_id_base) &
			0x3FFFFFFF | 0x80000000;
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
			((new_buf_id << 12) + wifi_buf_id_base) &
			0x3FFFFFFF | 0x80000000;
		*(u16 *)(desc_addr + 6) = (old_val & 0x4000) | 0xDAC;
	}

	rxd_2g_cpu_idx = 0;
	rxd_2g_mirror = wifi_base_cfg_val;
	rxd_2g_init_done = 1;
	return 0;
}

static void wifi_reset_ba_entry(u32 dir, u32 wcid)
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
static u32 wifi_tick_count;

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
	    (wifi_tick_count - wifi_rx_last_tick) > 9) {
		wifi_periodic_check(0);
		wifi_rx_last_tick = wifi_tick_count;
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
			((new_buf_id << 12) + wifi_buf_id_base) & 0x3FFFFFFF | 0x80000000;
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
	    (wifi_tick_count - wifi_tx_last_tick) > 9) {
		wifi_periodic_check(1);
		wifi_tx_last_tick = wifi_tick_count;
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
		((new_buf_id << 12) + wifi_buf_id_base) & 0x3FFFFFFF | 0x80000000;
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
static void wifi_pcie_desc_alloc(void)
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

	mbox_wifi_handler = wifi_mail_set_wait;
	mbox_wifi_handler2 = wifi_mail_set_event;
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
static void wifi_npu_init(u32 dbdc)
{
#ifdef HAS_WIFI
	u32 desc_type1, desc_type2;
	u32 bar_5g, bar_2g;

	npu_printf("[NPU] %s...\n", "npu_init");
	npu_printf("=======================\n");
	npu_printf("NPU Version: %s_NPU_%s\n",
		   wifi_chip_names[wifi_driver_model], NPU_INIT_VERSION);
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

/* WiFi mailbox setters: simple parameter setters called from host */
static void npu_set_retry_limit(u32 val)
{
	wifi_retry_limit = (u16)val;
	npu_printf("enq_error_retry_times = %d !!!\n", val);
}

static void npu_set_pcie_port_type(u32 val)
{
	wifi_pcie_port_type = (u8)val;
	npu_printf("PCIe_Port_Type = %d !!!\n", val);
}

static void npu_set_band_enable(u32 band)
{
	if (band > 1) {
		npu_printf("[ERROR] band_idx is wrong value %d !!!\n", band);
		return;
	}
	*((u8 *)&pipeline_5g_ready + band) = 1;
}

static void npu_set_force_to_cpu(u8 val)
{
	wifi_force_to_cpu = val;
	npu_printf("isForceToCpu=%s\n", val ? "true" : "false");
	if (wifi_force_to_cpu > 1)
		npu_printf("[ERROR] isForceToCpu is wrong value !!!\n");
}

static void npu_set_flushall_timeout(u32 val)
{
	wifi_flushall_timeout = (u16)val;
	npu_printf("flushall_timeout=%d\n", val);
}

static void npu_set_flushone_timeout(u32 val)
{
	wifi_flushone_timeout = (u16)val;
	npu_printf("flushone_timeout=%d\n", val);
}

static void npu_set_no_ba_test(u8 val)
{
	wifi_no_ba_test = val;
	npu_printf("isforTestNoBA=%s\n", val ? "true" : "false");
	if (wifi_no_ba_test > 1)
		npu_printf("[ERROR] isforTestNoBA is wrong value !!!\n");
}

static void npu_set_fast_flag(u8 val)
{
	wifi_debug_flags = val;
	npu_printf("npu_wifi_fast_flag = %d !!!\n", val);
}

static void npu_set_pkt_buf_addr(u32 val)
{
	wifi_pkt_buf_addr = val;
	npu_printf("pkt_buf_addr=%lx\n", val);
}

static void npu_set_dram_ba_node_addr(u32 val)
{
	wifi_dram_ba_node_addr = (val & 0x3FFFFFFF) | NPU_ADDR_MASK;
	npu_printf("dramBaNodeAddr=%x\n", val);
}

static void npu_set_driver_model(u32 val)
{
	wifi_driver_model = (u8)val;
	npu_printf("driverModel=%d\n", val);
}

static void npu_set_band0_on_cpu(u32 val)
{
	wifi_band0_on_cpu = (u8)val;
	npu_printf("npu_band0_on_cpu_support=%s\n",
		   val ? "true" : "false");
}

/* npu_set_bar_info: update BA entry window start for a given WCID/TID */
static void npu_set_bar_info(u32 band, u32 packed)
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

/* npu_set_ba_entry: configure BA reorder window for a WCID/TID */
static void npu_set_ba_entry(u32 band, u32 packed)
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

/* npu_set_wait_state: set per-port wait state and update force_to_cpu */
static void npu_set_wait_state(u32 port, u8 state)
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

/* WiFi RXD init thunk: dispatches to band-specific init */
static void npu_set_rxd_init(u32 band, u32 ring_size)
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
 * These are the host-side mailbox command handlers. On the kite
 * (MT7916/MT7996) path, most return "not support on 791X".
 * The eagle path has full implementations.
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

static u32 npu_mbox_get_rxdesc_base(u32 band)
{
	if (band == 1)
		return wifi_tx_ring_base_5g & 0x1FFFFFFF;
	else if (wifi_rx_ring_base_2g != 0)
		return wifi_rx_ring_base_2g & 0x1FFFFFFF;
	npu_printf("%s: not support\n", "wifi_mail_get_wait_rxdesc_base");
	return 1111;
}

static void npu_mbox_get_npu_info(void)
{
	npu_printf("%s L%d do nothing\n",
		   "npu_mbox_get_wait_npu_info_wrapper", 3794);
}

/* WiFi mail handler callbacks (registered in dispatch table) */
static int wifi_mail_set_debug_flag(u32 *msg)
{
	u32 flag = msg[2];
	u32 band = msg[0] & 0xF;

	npu_mbox_set_debug_flag(band, flag);
	return 1;
}

static int wifi_mail_set_wait_inode_cfg(u32 *msg)
{
	u32 a = ((u8 *)msg)[8];
	u32 b = ((u8 *)msg)[9];
	u32 band = msg[0] & 0xF;

	npu_mbox_rx_hw_cfg_set(band, a, b);
	return 1;
}

static int wifi_mail_set_wait_inode_stop(u32 *msg)
{
	u32 band = msg[0] & 0xF;

	npu_mbox_stop_set(band);
	return 1;
}

static int wifi_mail_set_pcie_swap(u32 *msg)
{
	u32 val = msg[2];

	npu_mbox_pcie_swap_set(val);
	return 1;
}

static int wifi_mail_get_dma_addr(u32 *msg)
{
	u32 dir = msg[2];
	u32 band = msg[0] & 0xF;
	u32 addr;

	addr = npu_mbox_txrx_ring_dma_addr_get(dir, band);
	msg[2] = addr;
	npu_printf("get dma addr. band=%d dir=%d addr=%x\n", band, dir, addr);
	return 1;
}

static int wifi_mail_get_ring_size(u32 *msg)
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

static int wifi_mail_get_rxdesc_base(u32 *msg)
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

#endif /* WIFI_KITE */

/* WiFi mailbox command dispatcher: lookup and call handler by cmd index */
static int wifi_mbox_cmd_dispatch(u32 *msg)
{
#ifdef WIFI_KITE
	u32 cmd_idx = *msg;

	if (cmd_idx < 10 && wifi_mbox_handlers[cmd_idx])
		return wifi_mbox_handlers[cmd_idx]((u32)msg, 0);
#endif
	return 0;
}

/* WiFi mail set_wait handler: dispatches sub-commands from host */
static int wifi_mail_set_wait(u32 base, u32 cnt)
{
	u32 *msg = (u32 *)((base & 0x3FFFFFFF) | NPU_ADDR_MASK);
	u32 cmd = msg[1];
	u32 band = msg[0] & 0xF;
	u32 val = msg[2];

	switch (cmd) {
	case 0:
		npu_set_driver_model(val);
		break;
	case 1:
		npu_set_pcie_port_type(val);
		break;
	case 2:
		npu_set_retry_limit(val);
		break;
	case 3:
		npu_set_force_to_cpu((u8)val);
		break;
	case 4:
		npu_set_flushall_timeout(val);
		break;
	case 5:
		npu_set_flushone_timeout(val);
		break;
	case 6:
		npu_set_no_ba_test((u8)val);
		break;
	case 7:
		npu_set_fast_flag((u8)val);
		break;
	case 8:
		npu_set_pkt_buf_addr(val);
		break;
	case 9:
		npu_set_dram_ba_node_addr(val);
		break;
	case 10:
		npu_set_band0_on_cpu(val);
		break;
	case 11:
		npu_set_bar_info(band, val);
		break;
	case 12:
		npu_set_ba_entry(band, val);
		break;
	case 13:
		wifi_reset_ba_entry(band, val);
		break;
	case 14:
		npu_set_pcie_base(val, band);
		break;
	case 15:
		npu_set_rxd_init(band, val);
		break;
	case 16:
		npu_set_band_enable(band);
		break;
	case 17:
		npu_set_wait_state(val, (u8)(msg[3]));
		break;
	default:
		npu_printf("wifi_mail_set_wait: unknown cmd %d\n", cmd);
		break;
	}
	return 1;
}

/* WiFi mail set_event handler */
static int wifi_mail_set_event(u32 base, u32 cnt)
{
	u32 *msg = (u32 *)((base & 0x3FFFFFFF) | NPU_ADDR_MASK);
	u32 cmd = msg[1];

	switch (cmd) {
	case 0:
		wifi_npu_init(msg[0] & 0xF);
		break;
	default:
		wifi_mbox_cmd_dispatch(msg);
		break;
	}
	return 1;
}

/* Core0 WiFi init wrapper */
static void core0_wifi_init_wrapper(void)
{
#ifdef HAS_WIFI
	int result;

#ifdef WIFI_KITE
	tdma_tx_init();
#endif
	wifi_bridge_init();
	npu_printf("%s finish\n", "core0_wifi_init_wrapper");

	result = hostadpt_init();
	if (result != 0)
		npu_printf("Error: there is something wrong with hostadpt\n");
#endif
}

/* Core3 WiFi init wrapper - polls for init completion */
static void core3_wifi_init_wrapper(void)
{
#ifdef HAS_WIFI
	while (!(wifi_bridge_active & 2)) {
		if (wifi_tx_pending != 0)
			wifi_tx_process();
		if (wifi_rx_pending != 0)
			wifi_rx_process();
	}
	npu_printf("%s finish\n", "core3_wifi_init_wrapper");
#endif
}

/* WiFi bridge main loop: runs on core1, dispatches RX/TX */
static void __attribute__((noreturn)) wifi_bridge_loop(void)
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

/* WiFi pipeline 5G worker: runs on core1, dequeues from pipeline ring */
static void __attribute__((noreturn)) wifi_pipeline_worker(void)
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

/* ================================================================
 * Tunnel offload (AN758X)
 * ================================================================ */

#ifdef HAS_TUNNEL

/* PPE register bases for tunnel offload */
#define PPE0_CTRL       0x1FB50E00
#define PPE1_CTRL       0x1FB51E00
#define PPE0_CTRL2      0x1FB50E04
#define PPE1_CTRL2      0x1FB51E04
#define PPE0_MISC       0x1FB50E1C

static void tunnel_ppe_reset(void)
{
	u32 chip_rev = REG32(CHIP_ID_REG) >> 16;

	/* clear PPE0 control bits */
	REG32(PPE0_CTRL) &= ~1u;
	REG32(PPE0_CTRL) &= ~2u;
	REG32(PPE0_CTRL) &= ~0x100u;
	REG32(PPE0_CTRL) &= ~0x200u;
	REG32(PPE0_CTRL) &= ~0x40u;
	REG32(PPE0_CTRL) &= ~0x1000u;
	REG32(PPE0_CTRL) &= ~0x20u;

	/* dual-PPE (AN7581) */
	if (chip_rev == 14) {
		REG32(PPE1_CTRL) &= ~1u;
		REG32(PPE1_CTRL) &= ~2u;
		REG32(PPE1_CTRL) &= ~0x100u;
		REG32(PPE1_CTRL) &= ~0x200u;
		REG32(PPE1_CTRL) &= ~0x40u;
		REG32(PPE1_CTRL) &= ~0x1000u;
		REG32(PPE1_CTRL) &= ~0x20u;
	}

	/* SRv6/MAP-T disable */
	if (chip_rev == 10 || chip_rev == 12 || chip_rev == 14 ||
	    chip_rev == 15 || chip_rev == 16) {
		REG32(PPE0_CTRL) &= ~0x8000u;
		if (chip_rev == 14)
			REG32(PPE1_CTRL) &= ~0x8000u;
	}

	if (chip_rev == 11)
		REG32(PPE0_CTRL) ^= ~REG32(PPE0_CTRL) & 0x8000;

	/* clear remaining bits */
	REG32(PPE0_CTRL) &= ~0x10u;
	REG32(PPE0_CTRL) ^= ~(u8)REG32(PPE0_CTRL) & 8;
	REG32(PPE0_CTRL) ^= ~(u8)REG32(PPE0_CTRL) & 4;

	if (chip_rev == 14) {
		REG32(PPE1_CTRL) &= ~0x10u;
		REG32(PPE1_CTRL) ^= ~(u8)REG32(PPE1_CTRL) & 8;
		REG32(PPE1_CTRL) ^= ~(u8)REG32(PPE1_CTRL) & 4;
	}

	/* VXLAN/GRE disable */
	if (chip_rev == 12 || chip_rev == 14 ||
	    chip_rev == 15 || chip_rev == 16) {
		REG32(PPE0_CTRL) &= ~0x10000u;
		REG32(PPE0_CTRL) &= ~0x20000u;
		if (chip_rev == 14) {
			REG32(PPE1_CTRL) &= ~0x10000u;
			REG32(PPE1_CTRL) &= ~0x20000u;
		}
	}

	/* preserve only bit 16 of ctrl2 */
	REG32(PPE0_CTRL2) &= 0x10000u;
	if (chip_rev == 14)
		REG32(PPE1_CTRL2) = REG32(PPE0_CTRL2) & 0x10000;

	/* clear misc interrupt bits */
	REG32(PPE0_MISC) &= ~0x80u;
	REG32(PPE0_MISC) &= ~0x100u;
	REG32(PPE0_MISC) &= ~0x200u;
	REG32(PPE0_MISC) &= ~0x400u;
	REG32(PPE0_MISC) &= ~0x800u;

	if (chip_rev == 14) {
		REG32(PPE0_MISC) &= ~0x80u;
		REG32(PPE0_MISC) &= ~0x100u;
		REG32(PPE0_MISC) &= ~0x200u;
		REG32(PPE0_MISC) &= ~0x400u;
		REG32(PPE0_MISC) &= ~0x800u;
	}
}

static void tunnel_init(void)
{
	npu_printf("tunnel_init\n");
	npu_memset(tunnel_ctx, 0, sizeof(tunnel_ctx));
}

static void tunnel_process(void)
{
	/* tunnel packet processing: VXLAN/SRv6/MAP-T encap/decap */
}

static int tunnel_mail_handler(u32 base, u32 cnt)
{
	npu_printf("tunnel_mail_handler\n");
	return 1;
}

/* tunnel mailbox sub-handlers */
static int tunnel_mail_store_hdr(u32 base)
{
	u32 idx = *(u8 *)(base + 8);
	u32 bridge_addr = npu_bridge_addr();

	npu_memcpy((void *)(bridge_addr + idx * 128), (void *)(base + 9), 50);
	return 1;
}

static int tunnel_mail_store_srv6(u32 base)
{
	u32 idx = *(u8 *)(base + 8);
	u32 len = *(u8 *)(base + 9);
	u32 bridge_addr = npu_bridge_addr();

	if (idx > 7) {
		npu_printf("invalid idx %d in %s,%d\n",
			   idx, "tunnel_mail_npu_store_srv6_hdr", 107);
		return 1;
	}

	npu_memcpy((void *)(bridge_addr + (idx + 20) * 128),
		   (void *)(base + 10), len);
	tunnel_srv6_hdr_len[idx] = (u8)len;
	return 1;
}

static int tunnel_mail_set_srv6_addr(u32 base)
{
	npu_memcpy(srv6_my_ipv6, (void *)(base + 8), 16);
	npu_printf("set srv6 my ipv6: %02X%02X:%02X%02X:%02X%02X:%02X%02X:%02X%02X:%02X%02X:%02X%02X:%02X%02X\n",
		   srv6_my_ipv6[0], srv6_my_ipv6[1], srv6_my_ipv6[2], srv6_my_ipv6[3],
		   srv6_my_ipv6[4], srv6_my_ipv6[5], srv6_my_ipv6[6], srv6_my_ipv6[7],
		   srv6_my_ipv6[8], srv6_my_ipv6[9], srv6_my_ipv6[10], srv6_my_ipv6[11],
		   srv6_my_ipv6[12], srv6_my_ipv6[13], srv6_my_ipv6[14], srv6_my_ipv6[15]);
	return 1;
}

static int tunnel_mail_l4s_stub(u32 base)
{
	(void)base;
	npu_printf("L4S not support!!!\n");
	return 1;
}

static int tunnel_mail_reset(void)
{
	tunnel_ppe_reset();
	return 1;
}

/* hwnat mail dispatcher: receives raw data from host, dispatches by funcId */
static int hwnat_mail_dispatch(u32 raw_ptr)
{
	u32 addr = (raw_ptr & 0x3FFFFFFF) | 0x40000000;
	u32 func_type = *(volatile u32 *)addr;
	u32 func_id;

	if (func_type != 1) {
		u32 i;

		npu_printf("not support unknow funcType\n");
		for (i = 0; i < 28; i += 4)
			npu_printf("Offset: %08zx, Value: 0x%08x\n",
				   i, *(volatile u32 *)(addr + i));
		return 0;
	}

	func_id = *(volatile u32 *)(addr + 4);
	if (func_id < 1 || func_id > 5) {
		npu_printf("Error: invalid funcId! hwnat_mail_data->funcType=%u hwnat_mail_data->funcId=%u\n",
			   1, func_id);
		return 0;
	}

	/* dispatch through function table at mbox_ext_handlers */
	if (mbox_ext_handlers[func_id + 4] == NULL)
		return 0;

	{
		int result = mbox_ext_handlers[func_id + 4](addr, 0);

		if (result == 0)
			npu_printf("hwnat_mail_set_wait_operation fail !\n");
		return result;
	}
}

#endif /* HAS_TUNNEL */

/* ================================================================
 * DBA - Dynamic Bandwidth Allocation (AN7583 + WiFi)
 * ================================================================ */

#ifdef HAS_DBA

static void dba_init(void)
{
	npu_printf("dba_init\n");
	npu_memset(dba_state, 0, sizeof(dba_state));

	npu_fttr_base = NPU_FTTR_BASE;
	npu_printf("npu_fttr_base=%x\n", npu_fttr_base);

	dba_band_switch = 0;
	dba_bwmap_switch = 0;
	npu_memset(dba_alloc_state, 0, sizeof(dba_alloc_state));
}

static void dba_alloc_process(void)
{
	u32 i;
	u32 base = npu_fttr_base;

	if (base == 0)
		return;

	/* process bandwidth allocation for each T-CONT */
	for (i = 0; i < dba_max_alloc_ids; i++) {
		u32 report = REG32(base + 0x100 + i * 4);

		if (report == 0)
			continue;
		/* update allocation state based on report */
		dba_alloc_state[i % 20] += report;
	}
}

static void dba_main_loop(void)
{
	if (npu_fttr_base == 0) {
		npu_printf("error, npu_fttr_base is not init\n");
		return;
	}

	npu_printf("dba_main_loop start, fttr_base=%x\n", npu_fttr_base);

	while (1) {
		if (dba_band_switch)
			dba_alloc_process();

		/* check for DBA events from FTTR hardware */
		if (REG32(npu_fttr_base + 0x000) & 1) {
			REG32(npu_fttr_base + 0x000) = 1;
			dba_alloc_process();
		}
	}
}

static int dba_mail_handler(u32 base, u32 cnt)
{
	u32 cmd;

	cmd = *(volatile u32 *)((base & 0x3FFFFFFF) | 0x40000000);
	npu_printf("dba_mail_handler cmd=%x\n", cmd);

	switch (cmd) {
	case 0:
		dba_band_switch = 1;
		break;
	case 1:
		dba_band_switch = 0;
		break;
	case 2:
		dba_bwmap_switch = (u8)cnt;
		break;
	default:
		npu_printf("dba unknown cmd %d\n", cmd);
		break;
	}
	return 1;
}

static void dba_timer_handler(int src)
{
	u32 bit = timer_get_bit((u32)src);
	u32 base = NPU_TIMER0_BASE;

	REG32(base) |= (1u << bit);
	REG32(base) &= ~(1u << bit);

	dba_timer0_snap = REG32(NPU_TIMER0_BASE + 0x10);
}

#endif /* HAS_DBA */

/* ================================================================
 * TR-471 test infrastructure (AN7581 + WiFi)
 * ================================================================ */

#ifdef HAS_TR471

static void tr471_main_init(void)
{
	npu_printf("tr471_main_init\n");
}

#endif

/* ================================================================
 * Per-core main functions
 * ================================================================ */

static void core0_main(void)
{
	tdma_init();

#ifdef WIFI_KITE
	tdma_bmgr_init();
#else
	buf_mgr_init();
#endif

	npu_bridge_buf_init();
	core0_wifi_init_wrapper();
	plic_register_isr(59, dbg_cnt_isr);

	npu_printf("%s\n", "core0_main");
}

static void core1_main(void)
{
	npu_printf("%s\n", "core1_main");
#ifdef HAS_WIFI
	if (wifi_debug_flags & 1)
		wifi_pipeline_worker();
	else
		wifi_bridge_loop();
#endif
}

#if MAX_CORE_NUM > 2
static void core2_main(void)
{
	npu_printf("%s\n", "core2_main");
	/* register timer ISR on PLIC source 18 */
	plic_register_isr(18, timer_isr);
}

static void core3_main(void)
{
	npu_printf("%s\n", "core3_main");

	chip_id_query();
	usb_powerdown();
	core3_wifi_init_wrapper();
}

static void core4_main(void)
{
	npu_printf("%s\n", "core4_main");
#ifdef HAS_WIFI
	/* WiFi handler - variant-specific */
#endif
}

static void core5_main(void)
{
#if defined(HAS_DBA)
	npu_printf("%s: start\n", "core5_dba_main");

	dba_init();

	/* register DBA timer ISR on PLIC source 24 */
	plic_register_isr(24, dba_timer_handler);

	/* DBA main processing loop (never returns) */
	dba_main_loop();
#elif defined(AN7581) && defined(HAS_WIFI)
	npu_printf("%s\n", "core5_main");
	wifi_bridge_loop();
#else
	npu_printf("%s\n", "core5_main");
#endif
}
#endif /* MAX_CORE_NUM > 2 */

#if MAX_CORE_NUM > 6
static void core6_main(void)
{
	npu_printf("%s\n", "core6_main");
#ifdef HAS_WIFI
	/* AN7581 core6: WiFi handler */
#endif
}

static void core7_main(void)
{
#ifdef HAS_TUNNEL
	npu_printf("%s for npu tunnel offload\n", "core7_main");

	/* register tunnel PLIC interrupt 15 */
	plic_register_isr(15, mbox_isr);

	tunnel_init();

#ifdef HAS_TR471
	tr471_main_init();
#endif

	/* tunnel processing loop */
	while (1) {
		tunnel_process();
	}
#else
	npu_printf("%s\n", "core7_main");
#endif
}
#endif /* MAX_CORE_NUM > 6 */

/* ================================================================
 * Core dispatch
 * ================================================================ */

void core_dispatch(int a0, int a1)
{
	u32 core = get_hartid();

	(void)a0;
	(void)a1;

	switch (core) {
	case 0:
		core0_main();
		break;
	case 1:
		core1_main();
		break;
#if MAX_CORE_NUM > 2
	case 2:
		core2_main();
		break;
	case 3:
		core3_main();
		break;
	case 4:
		core4_main();
		break;
	case 5:
		core5_main();
		break;
#endif
#if MAX_CORE_NUM > 6
	case 6:
		core6_main();
		break;
	case 7:
		core7_main();
		break;
#endif
	default:
		npu_printf("\nWrong Core ID: %d", core);
		break;
	}
}

/* ================================================================
 * Trap vector (machine-mode exception/interrupt handler)
 * ================================================================ */

void __attribute__((naked, aligned(4))) trap_vector(void)
{
	__asm__ volatile(
		"addi sp, sp, -128\n"
		"sw ra,   0(sp)\n"
		"sw t0,   4(sp)\n"
		"sw t1,   8(sp)\n"
		"sw t2,  12(sp)\n"
		"sw a0,  16(sp)\n"
		"sw a1,  20(sp)\n"
		"sw a2,  24(sp)\n"
		"sw a3,  28(sp)\n"
		"sw a4,  32(sp)\n"
		"sw a5,  36(sp)\n"
		"sw a6,  40(sp)\n"
		"sw a7,  44(sp)\n"
		"sw t3,  48(sp)\n"
		"sw t4,  52(sp)\n"
		"sw t5,  56(sp)\n"
		"sw t6,  60(sp)\n"
		"sw s0,  64(sp)\n"
		"sw s1,  68(sp)\n"
		"sw s2,  72(sp)\n"
		"sw s3,  76(sp)\n"
		"sw s4,  80(sp)\n"
		"sw s5,  84(sp)\n"
		"sw s6,  88(sp)\n"
		"sw s7,  92(sp)\n"
		"sw s8,  96(sp)\n"
		"sw s9, 100(sp)\n"
		"sw s10,104(sp)\n"
		"sw s11,108(sp)\n"
		"csrr a0, mcause\n"
		"bgez a0, 1f\n"
		/* interrupt: claim from PLIC, dispatch ISR */
		"lui a0, 0x0C200\n"
		"lw a0, 4(a0)\n"
		"addi a0, a0, -1\n"
		"call call_isr_by_src\n"
		"j 2f\n"
		"1:\n"
		"call call_isr_by_src\n"
		"2:\n"
		"lw ra,   0(sp)\n"
		"lw t0,   4(sp)\n"
		"lw t1,   8(sp)\n"
		"lw t2,  12(sp)\n"
		"lw a0,  16(sp)\n"
		"lw a1,  20(sp)\n"
		"lw a2,  24(sp)\n"
		"lw a3,  28(sp)\n"
		"lw a4,  32(sp)\n"
		"lw a5,  36(sp)\n"
		"lw a6,  40(sp)\n"
		"lw a7,  44(sp)\n"
		"lw t3,  48(sp)\n"
		"lw t4,  52(sp)\n"
		"lw t5,  56(sp)\n"
		"lw t6,  60(sp)\n"
		"lw s0,  64(sp)\n"
		"lw s1,  68(sp)\n"
		"lw s2,  72(sp)\n"
		"lw s3,  76(sp)\n"
		"lw s4,  80(sp)\n"
		"lw s5,  84(sp)\n"
		"lw s6,  88(sp)\n"
		"lw s7,  92(sp)\n"
		"lw s8,  96(sp)\n"
		"lw s9, 100(sp)\n"
		"lw s10,104(sp)\n"
		"lw s11,108(sp)\n"
		"addi sp, sp, 128\n"
		"mret\n"
	);
}

/* ================================================================
 * NPU initialization (called from crt0 before core_dispatch)
 * ================================================================ */

void npu_init(void)
{
	u32 hart = get_hartid();

	/* hart 0 clears sync flag; others wait */
	if (hart == 0)
		core_sync_flag = 0;
	else if (core_sync_flag == 0) {
		while (core_sync_flag == 0)
			;
	}

	/* init PLIC for this hart */
	plic_init();

	/* configure trap vector and enable machine external interrupts */
	irq_disable();
	csr_write(mtvec, (unsigned long)trap_vector);
	csr_write(mie, 0x800);
	irq_enable();

	/* hart 0 conditional block: first-time initialization */
	if (npu_reset_pending != 0) {
		npu_reset_pending = 0;
		mib12_snapshot = REG32(NPU_MIB12);
		REG32(NPU_SCU_RSTCTRL1) = 0;
		REG32(NPU_THREAD_ENABLE) = 1;
		REG32(NPU_MIB0) = ALL_FF;
		plic_enable_wrapper(22);
		timer_init(0, 1, 10);
		mailbox_init();
	}

	/* hart 0 signals others to proceed */
	if (hart == 0)
		core_sync_flag = 1;

	/* boot signature */
	REG32(NPU_MIB31) = INIT_COMPLETE;
}
