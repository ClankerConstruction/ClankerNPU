/*
 * AN75XX NPU firmware - reconstructed from binary
 *
 * Compile-time variants via -DAN75xx -DMTxxxx.
 */

#include "npu_internal.h"

/* ================================================================
 * Global variables - .data section (initialized)
 *
 * Order must match npu_data.bin layout for binary-correct output.
 * Addresses shown are for AN7583_MT7996 reference variant.
 * ================================================================ */

/* 0x000: system config base */
u32 npu_max_frame_size = 1500;
static u32 __data_pad0[3];

/* 0x010 - 0x14C: five per-timer tables. AN7583 has a second bank of 8
 * timers at 0x1EC10200; AN7581 exposes only 4. */
u32 timer_irq_map[NPU_TIMER_NUM] = {
#if defined(AN7581)
	18, 19, 20, 21
#elif defined(AN7552)
	18, 19, 20, 52, 53, 54, 55, 21
#else
	18, 19, 20, 52, 53, 54, 55, 21, 24, 25, 26, 27, 28, 29, 30, 31
#endif
};
u32 timer_clr_bit[NPU_TIMER_NUM] = {
#if defined(AN7581)
	16, 17, 18, 21
#elif defined(AN7552)
	16, 17, 18, 21, 22, 23, 24, 19
#else
	16, 17, 18, 21, 22, 23, 24, 19, 16, 17, 18, 21, 22, 23, 24, 19
#endif
};
u32 timer_bit_map[NPU_TIMER_NUM] = {
#if defined(AN7581)
	0, 1, 2, 5
#elif defined(AN7552)
	0, 1, 2, 5, 6, 7, 8, 3
#else
	0, 1, 2, 5, 6, 7, 8, 3, 0, 1, 2, 5, 6, 7, 8, 3
#endif
};
u32 timer_counter_reg[NPU_TIMER_NUM] = {
	0x1EC10108, 0x1EC10110, 0x1EC10118, 0x1EC10130,
#if !defined(AN7581)
	0x1EC10140, 0x1EC10120, 0x1EC10128, 0x1EC10154,
#endif
#if !defined(AN7581) && !defined(AN7552)
	0x1EC10208, 0x1EC10210, 0x1EC10218, 0x1EC10230,
	0x1EC10240, 0x1EC10220, 0x1EC10228, 0x1EC10254,
#endif
};
u32 timer_reload_reg[NPU_TIMER_NUM] = {
	0x1EC10104, 0x1EC1010C, 0x1EC10114, 0x1EC1012C,
#if !defined(AN7581)
	0x1EC1013C, 0x1EC1011C, 0x1EC10124, 0x1EC10150,
#endif
#if !defined(AN7581) && !defined(AN7552)
	0x1EC10204, 0x1EC1020C, 0x1EC10214, 0x1EC1022C,
	0x1EC1023C, 0x1EC1021C, 0x1EC10224, 0x1EC10250,
#endif
};
#if !defined(AN7581)
/* timers 3..6 share a control bit with another block; clear it on enable */
u32 timer_pair_bit[4] = { 0x19, 0x1a, 0x1b, 0x1c };
#endif

/* 0x180: GET_WAIT function table (indexed by SDK WIFI_MAIL_Get_Wait_Func_t) */
#ifdef HAS_WIFI
typedef int (*wifi_mail_fn_t)(u32 *msg);
#ifdef WIFI_KITE
wifi_mail_fn_t get_wait_func_table[10] __attribute__((section(".data"))) = {
	wifi_mail_get_npu_info,
	wifi_mail_get_last_rate,
	wifi_mail_get_counter,
	wifi_mail_get_dbg_counter,
	wifi_mail_get_rxdesc_base,
	wifi_mail_get_wcid_dbg_counter,
	wifi_mail_get_dma_addr,
	wifi_mail_get_ring_size,
	wifi_mail_get_mdc_lock,
	wifi_mail_get_dump_mapping,
};
wifi_mail_fn_t set_wait_func_table[31] __attribute__((section(".data"))) = {
	wifi_mail_set_pcie_addr,
	wifi_mail_set_desc,
	wifi_mail_set_init_done,
	wifi_mail_set_tran_to_cpu,
	wifi_mail_set_ba_win_size,
	wifi_mail_set_driver_model_cmd,
	wifi_mail_set_del_sta,
	wifi_mail_set_dram_ba_node,
	wifi_mail_set_pkt_buf,
	wifi_mail_set_test_noba,
	wifi_mail_set_flushone,
	wifi_mail_set_flushall,
	wifi_mail_set_force_cpu,
	wifi_mail_set_pcie_state,
	wifi_mail_set_port_type,
	wifi_mail_set_retry,
	wifi_mail_set_bar_info_cmd,
	wifi_mail_set_fast_flag_cmd,
	wifi_mail_set_band0_cpu,
	wifi_mail_set_tx_ring_pcie,
	wifi_mail_set_tx_desc_hw,
	wifi_mail_set_tx_buf_hw,
	wifi_mail_set_rx_txdone_hw,
	wifi_mail_set_tx_pkt_buf,
	wifi_mail_set_txrx_reg,
	wifi_mail_set_debug_flag,
	wifi_mail_set_wait_inode_cfg,
	wifi_mail_set_wait_inode_stop,
	wifi_mail_set_pcie_swap,
	wifi_mail_set_ratelimit,
	wifi_mail_set_arht_chip_info,
};
#else
wifi_mail_fn_t get_wait_func_table[10] __attribute__((section(".data"))) = {
	eagle_mail_get_npu_info,
	eagle_mail_get_last_rate,
	eagle_mail_get_counter,
	eagle_mail_get_dbg_counter,
	eagle_mail_get_rxdesc_base,
	eagle_mail_get_wcid_dbg_counter,
	eagle_mail_get_dma_addr,
	eagle_mail_get_ring_size,
	eagle_mail_get_mdc_lock,
	eagle_mail_get_dump_mapping,
};
wifi_mail_fn_t set_wait_func_table[31] __attribute__((section(".data"))) = {
	eagle_mail_set_pcie_addr,
	eagle_mail_set_desc,
	eagle_mail_set_init_done,
	eagle_mail_set_tran_to_cpu,
	eagle_mail_set_ba_win_size,
	eagle_mail_set_driver_model,
	eagle_mail_set_del_sta,
	eagle_mail_set_dram_ba_node,
	eagle_mail_set_pkt_buf,
	eagle_mail_set_test_noba,
	eagle_mail_set_flushone,
	eagle_mail_set_flushall,
	eagle_mail_set_force_cpu,
	eagle_mail_set_pcie_state,
	eagle_mail_set_port_type,
	eagle_mail_set_retry,
	eagle_mail_set_bar_info,
	eagle_mail_set_fast_flag,
	eagle_mail_set_band0_cpu,
	eagle_mail_set_tx_ring_pcie,
	eagle_mail_set_tx_desc_hw,
	eagle_mail_set_tx_buf_hw,
	eagle_mail_set_rx_txdone_hw,
	eagle_mail_set_tx_pkt_buf,
	eagle_mail_set_txrx_reg,
	eagle_mail_set_debug_flag,
	eagle_mail_set_inode_cfg,
	eagle_mail_set_inode_stop,
	eagle_mail_set_pcie_swap,
	eagle_mail_set_ratelimit,
	eagle_mail_set_arht_chip_info,
};
#endif
#endif

/* 0x1EC: WiFi chip name table (kite only, 8 bytes per entry) */
#ifdef WIFI_KITE
const char wifi_chip_names[6][8] = {
	"MT7915A", "MT7915D", "MT7916",
	"MT7902",  "MT7990",  ""
};
#endif

/* 0x220: mailbox wrapper function pointer tables */
mbox_handler_t mbox_pri_handlers[10];
mbox_handler_t mbox_ext_handlers[32];

/* HWNAT config from the host */
u8 hwnat_cds;
u8 hwnat_xpon_hal_api_ng;
u8 hwnat_wan_xsi;
u8 hwnat_ct_joyme4;
u8 hwnat_max_packet_2000;
u8 hwnat_ready;
u32 hwnat_ppe_type;
u32 hwnat_wan_mode;
u32 hwnat_ae_wan_sel;

/* 0x2D0: tunnel config structures */
#ifdef HAS_TUNNEL
u32 tunnel_config_ptrs[4];
u8 tunnel_config_area[0x500];
#endif

/* 0x7E8: debug/exception handler dispatch */
u32 exception_handlers[12];
u32 exception_sentinels[4] = {
	0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0x0000FFFF
};

/* 0x820: UART console descriptors */
u8 uart_desc[2][32] = {
	{ 0x01, 0x00, 'A','S','C','I','I', 0 },
	{ 0 }
};

/* 0x864: mutex descriptor pair table */
u32 mutex_initial_state;
u32 mutex_desc_pairs[512];

/* 0xC70: per-variant config scalars */
s32 wifi_chip_index = -1;

#ifdef HAS_DBA
u8 dba_cfg_flag0 = 1;
u8 dba_cfg_flag1 = 1;
u8 dba_cfg_flag2;
u32 dba_alloc_gran = 5;
u32 dba_tick_count = 10;
u32 dba_defaults[4] = { 1, 1, 1, 1 };
u32 dba_band_count = 1;
u32 dba_bw_unit = 46;
u32 dba_max_alloc_ids = 128;
u32 dba_cfg3 = 1;
u32 dba_max_frame = 1500;
u32 dba_alloc_mask = 0xFFFF;
#endif

u32 npu_reset_pending = 1;
u32 tdma_bmgr_mode;
u32 npu_printf_prefix = 1;

#ifdef HAS_DBA
u8 dba_cfg4 = 1;
#endif

#ifdef HAS_TUNNEL
u32 tunnel_max_count = 7;
u32 tunnel_timeout_ms = 100;
u8 tunnel_cfg_flag = 1;
#endif

s32 sentinel_1 = -1;

#ifdef HAS_DBA
u16 dba_cfg_word0 = 25;
u16 dba_cfg_word1 = 10;
#endif

s32 sentinel_2 = -1;

#ifdef HAS_TUNNEL
u32 tunnel_dispatch_ptr;
#endif

s32 sentinel_3 = -1;
u32 config_flags = 0x00020000;

const char delay_name[] = "__delay";


/* ================================================================
 * Global variables - .bss section (zero-initialized)
 *
 * Order must match BSS layout for correct GP-relative access.
 * ================================================================ */

/* mailbox handler slots */
mbox_handler_t mbox_core_handlers[6];

/* printf subsystem */
u8 printf_cfg_flags[4];
u8 printf_desc[8];
u8 printf_flag;

/* timer subsystem */
volatile u32 timer_raw_tick;
volatile u32 timer_slow_tick;
u32 timer_int_count;
u32 timer_prev_ctrl;
u32 timer_clk_mhz;
u32 timer_tod_sec;
u32 timer_tod_usec;
u32 timer_context[10];
u32 timer_ref_counts[8];
u8 srv6_my_ipv6[16];
u32 timer_isr_context[12];
isr_fn_t timer_isr_handler0;
isr_fn_t timer_isr_handler1;
u32 timer_isr_pad[4];
isr_fn_t timer_isr_handler2;
u32 timer_pad2[100];
isr_fn_t timer_callback;
u32 timer_pad3;

/* printf buffer */
char printf_buf[1024];

/* tunnel context */
#ifdef HAS_TUNNEL
u8 tunnel_ctx[3][1024];
#endif

/* BME/bridge */
u32 bme_base_addr;
u32 bme_pad[38];
u32 bme_config;
u32 bme_pad2[10];
u32 bme_desc_count;
u32 bme_status;
u32 bme_pad3;
u32 bme_ring_state[4];
u32 tdma_bme_dscp_idx;
u32 tdma_bme_dscp_base;
u32 npu_bridge_base;

/* PLIC ISR table: 192 function pointers */
isr_fn_t plic_isr_table[192];

/* PLIC config */
u32 printf_mutex_desc[2];
u32 mbox_notify_mutex[2];

/* SRAM buffer manager */
u32 sram_buf_mutex[2];
u32 sram_buf_pad[4];
u32 sram_buf_max_use;
u32 sram_buf_cur_idx;
u16 sram_buf_entries[200];
u32 sram_buf_pad2[8];

/* WiFi state */
u32 wifi_state[256];

/* tunnel funcId dispatch table (callback[1] — all variants) */
#ifdef HAS_TUNNEL
mbox_handler_t tunnel_func_table[10] __attribute__((section(".data"))) = {
	[0] = tunnel_mail_store_hdr,
	[3] = tunnel_mail_store_srv6,
	[4] = tunnel_mail_set_srv6_addr,
	[5] = tunnel_mail_frag_mtu,
	[6] = tunnel_mail_reset,
	[8] = tunnel_mail_l4s_stub,
};
#else
mbox_handler_t tunnel_func_table[10];
#endif

#ifdef HAS_TUNNEL
u8 tunnel_srv6_hdr_len[8];
volatile u32 tunnel_ecn_enabled;
u32 l4s_debug_enable;
u32 l4s_pkt_count;
u32 l4s_log_phase;
u32 l4s_skip_count;
u32 l4s_cached_qthresh;
u32 l4s_qid;
u32 l4s_qlen;
u32 ppe_module_idx;
u8 ppe_module_ver;
u32 gdm_fwd_mode;
u32 vlan_aware_mode;
u32 fragment_mtu[4];
u32 tunnel_pending[8];
u32 tunnel_test_active;
u32 tunnel_test_mode;
u32 tunnel_test_param;
u32 tunnel_offload_ready;
u32 tunnel_ipv6_frag_id;
u32 tunnel_v4_reasm_hdroff;
u32 tunnel_v4_reasm_len;
u32 tunnel_v4_reasm_desc;
u32 tunnel_v6_reasm_hdroff;
u32 tunnel_v6_reasm_len;
u32 tunnel_v6_reasm_desc;
u32 tunnel_srv6_seg_table;
u32 tunnel_encap_mtu;
#endif

/* DBA state */
#ifdef HAS_DBA
u32 dba_state[128];
u32 npu_fttr_base;
u8 dba_band_switch;
u8 dba_bwmap_switch;
u32 dba_alloc_state[20];
u32 dba_timer0_snap;
#endif

/* PLIC threshold per-hart */
u8 plic_threshold_table[8];
volatile u32 plic_isr_init_done;
u32 plic_cfg2;

/* UART debug console */
u32 uart_cmd_idx;
u8 uart_cmd_buf[32];

/* npu_init sync */
volatile u32 core_sync_flag;
u32 sim_mode_flag;

/* WiFi extended state */
u32 wifi_ext_state[32];

/* Reorder node pools (2000 pri + 5000 sec, 36 bytes each) */
u32 reorder_pri_node_base;
u32 reorder_sec_node_base;
u32 reorder_pri_idx_pool;
u32 reorder_sec_idx_pool;
u16 reorder_pri_widx;
u16 reorder_pri_ridx;
u16 reorder_sec_widx;
u16 reorder_sec_ridx;
u32 reorder_alloc_mutex[2];
u32 reorder_free_mutex[2];

/* BA (Block Ack) reorder window */
u32 ba_mutex_5g[2];
u32 ba_mutex_2g[2];
u8 wifi_dbdc_mode;
u32 ba_table_a;
u32 ba_table_b;

/* Bridge/BME state */
u8 bme_path_enable;
u8 pipeline_5g_ready;
u8 pipeline_2g_ready;
u32 fwd_dispatch_table[32];

/* PCIe bases */
u32 pcie_base_5g;
u32 pcie_base_2g;

/* RXD state */
u32 rxd_base_5g;
u32 rxd_5g_init_done;
u32 rxd_base_2g;
u32 rxd_2g_bufid_base;
u32 rxd_2g_init_done;
u32 rxd_5g_cpu_idx;
u32 rxd_5g_mirror;
u32 rxd_2g_cpu_idx;
u32 rxd_2g_mirror;
u16 rxd_5g_bufid_table[1536];

/* WiFi misc (declared early for reorder/BA functions) */
u32 wifi_base_cfg_val;
u8 wifi_debug_flags;
u32 wifi_buf_id_base;

/* WiFi per-port state (16 ports) */
u8 wifi_port_state_2g[16];
u8 wifi_port_state_5g[16];

/* WiFi global counters (u64 as lo/hi pairs) */
u32 wifi_global_bytes_lo;
u32 wifi_global_bytes_hi;
u32 wifi_global_pkts_lo;
u32 wifi_global_pkts_hi;
u32 wifi_global_bytes_5g_lo;
u32 wifi_global_bytes_5g_hi;
u32 wifi_global_pkts_5g_lo;
u32 wifi_global_pkts_5g_hi;

/* WiFi per-TID counters (256 entries × u64 as lo/hi pairs) */
u32 wifi_tid_pkt_cnt[512];
u32 wifi_tid_byte_cnt[512];

/* WiFi classifier state */
u8 wifi_classifier_bypass;

/* WiFi multi-desc state */
u32 wifi_rxd_ring_2g;
u32 wifi_rxd_ring_5g;
u32 wifi_rxd_idx_2g;
u32 wifi_rxd_idx_5g;
u16 *wifi_rxd_bufid_tbl;
u8 wifi_retry_limit;

/* WiFi init state */
u8 wifi_driver_model;
u8 wifi_pcie_port_type;
u8 wifi_band_cap;
u8 wifi_force_to_cpu;
u8 wifi_no_ba_test;
u8 wifi_band0_on_cpu;
u16 wifi_flushall_timeout;
u16 wifi_flushone_timeout;
u32 wifi_pkt_buf_addr;
u32 wifi_dram_ba_node_addr;
u32 wifi_pcie_desc_base;

/* WiFi per-band pkt queue state */
u16 pkt_queue_widx_2g;
u16 pkt_queue_widx_5g;
u32 pkt_queue_base_2g;
u32 pkt_queue_base_5g;
u16 pkt_queue_rx_widx_2g;
u16 pkt_queue_rx_ridx_2g;
u32 pkt_queue_rx_base_2g;
u32 pkt_queue_rx_base_5g;

/* WiFi per-band queue mutexes */
u32 queue_mutex_2g[2];
u32 queue_mutex_5g[2];
u32 queue_mutex_rx_2g[2];
u32 queue_mutex_rx_5g[2];

/* BA node management */
u32 ba_node_pool_base;

/* WiFi per-port wait state (16 ports × 2 bands) */
u8 wifi_wait_state_2g[16];
u8 wifi_wait_state_5g[16];

/* WiFi per-port band assignment */
u8 wifi_port_band_2g[16];
u8 wifi_port_band_5g[16];

/* host tx packet buffer, published over the mailbox */
volatile u32 npu_tx_pkt_buf_addr;
u32 tdma_rx_dscp_base[2];
u32 tdma_rx_desc_count;
u32 tdma_rx_alloc_fail;

/* WiFi pipeline pkt queue */
u32 wifi_pipeline_queue_2g;
u32 wifi_pipeline_queue_5g;

#ifdef WIFI_KITE
u32 ratelimit_table[32];
u32 arht_chip_info[6];
u32 arht_phy_tx_gpio;
u32 arht_chip_info_valid;

/* per-entry rx stats: [band][entry], 128 entries per band, stored as u32 pairs (u64) */
u32 npu_rx_bytes_entry[2][256];
u32 npu_rx_pkts_entry[2][256];

/* apcli aggregate counters (u64 stored as u32 pairs) */
u32 apcli_count_2g[2];
u32 apcli_count_5g[2];
u32 apcli_byte_count_2g[2];
u32 apcli_byte_count_5g[2];
#endif

#ifdef WIFI_EAGLE
volatile u32 eagle_rro_cfg[26];
volatile u32 eagle_rro_active;
mbox_handler_t eagle_event_table[4];

/* Host ring bases, keyed by the ring index the host puts in interfaceID.
 * The blob keeps one global per ring; the values are the same. */
u32 eagle_rx_ring_pcie_base[2];
u32 eagle_msdu_pg_pcie_base;
u32 eagle_ind_cmd_pcie_base;
u32 eagle_txdone_pcie_base;
u16 eagle_txdone_ring_cnt;

u32 eagle_tx_ring_pcie_base[2];
u32 eagle_tx_ring_pcie_base_r3;

u32 eagle_txd_space[2];
u32 eagle_tx_buf_space_pg[2];

u32 eagle_rx_txdone_desc_base;
u32 eagle_msdu_pg_desc_base;

u32 eagle_pkt_buf_addr;

u32 eagle_dram_ba_node_addr;
u32 eagle_icv_err_table;
u16 eagle_retry_times;
u8 eagle_pcie_port_type;
u8 eagle_pcie_state[2];
u8 eagle_rro_mode;
u8 eagle_test_noba;
u32 eagle_txdone_id_base;
u32 eagle_chip_info[6];
u32 eagle_phy_tx_gpio;

u32 eagle_stage_buf0;
u32 eagle_stage_buf1;
u32 eagle_stage_base[2];
u16 eagle_stage_widx[2];
u16 eagle_stage_ridx[2];

u32 eagle_rx_ring_desc_base[2];
u32 eagle_ind_cmd_desc_base;
u16 eagle_rx_ring_size[2];
u16 eagle_rx_ring_cpu_idx[2];
u8 eagle_rx_ring_init_done[2];
u16 eagle_rx_ring_bufid[2][1536];

/* datapath state the host drives over the mailbox */
volatile u32 eagle_rx_en;
volatile u32 eagle_tx_en;
volatile u32 eagle_init_done;
volatile u32 eagle_rx_busy;
volatile u8 eagle_rro_state;
volatile u8 eagle_txq_state;
volatile u8 eagle_stopping;
volatile u8 eagle_rx_stopped;
volatile u8 eagle_fastpath_en;
u8 eagle_rxdmad_on_core2;
u32 eagle_rro_addr_elem[128];
u32 eagle_session_tbl;
u32 eagle_emi_cidx;
u8 eagle_emi_cidx_valid;

/* packet queues between the rxdmad ring and the host adaptor */
u32 eagle_txq_base[2];
u16 eagle_txq_widx[2];
u16 eagle_txq_ridx[2];
u32 eagle_mseg_base[2];
u16 eagle_mseg_widx[2];
u16 eagle_mseg_ridx[2];
u32 eagle_txq_mutex[2];
u16 eagle_mseg_retry;

/* rxdmad ring */
u32 eagle_rxdmad_ridx;
u8 eagle_rxdmad_gen;
u8 eagle_rxdmad_abort;
u8 eagle_rxdmad_segs;
u32 eagle_rxdmad_seglen;
u32 eagle_seg_bufid[7];
u16 eagle_seg_len[7];

/* rx rings */
u32 eagle_rx_ring_ridx[2];

/* wifi tx rings */
u32 eagle_tx_ring_desc[2];
u16 eagle_tx_ring_cpu_idx[2];

/* wifi tx done ring */
u32 eagle_txdone_ridx;
u8 eagle_txdone_kick;
#endif

#if defined(WIFI_KITE) && defined(HAS_TR471)
volatile u32 kite_wifi_cfg[26];
volatile u32 kite_test_active;
#endif


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

u32 npu_strlen(const char *s)
{
	const char *p = s;

	while (*p)
		p++;
	return (u32)(p - s);
}

char get_core_char(void)
{
	u32 id = get_hartid();

	if (id <= 7)
		return (char)('0' + id);
	return 'X';
}

static u16 npu_htons(u16 x)
{
	return (u16)((x >> 8) | (x << 8));
}

static void wfi_idle(void)
{
	__asm__ volatile("wfi");
}


/* ================================================================
 * Hardware mutex
 * ================================================================ */

/* Acquire is a single try: the hardware arbitrates, callers do not spin.
 * Returns 0 when this hart owns the mutex, -1 otherwise. */
int hw_mutex_lock(u32 *desc)
{
	u32 off = (desc[0] * 4) & HW_MUTEX_OFF_MASK;
	u32 hart = get_hartid();
	u32 sts;

	REG32(HW_MUTEX_ACQ(off)) = (hart << 8) | 0x40;
	sts = REG32(HW_MUTEX_STATUS(hart, off));
	if (!(sts & HW_MUTEX_HELD))
		return -1;
	return ((sts >> 8) & 0xFF) == hart ? 0 : -1;
}

int hw_mutex_unlock(u32 *desc)
{
	u32 off = (desc[0] * 4) & HW_MUTEX_OFF_MASK;
	u32 hart = get_hartid();

	REG32(HW_MUTEX_REL(hart, off)) = (hart << 8);
	return 0;
}

int hw_mutex_lock_pri(u32 *desc)
{
	u32 off = (desc[0] * 4) & HW_MUTEX_OFF_MASK;
	u32 hart = get_hartid();
	u32 sts;

	if (desc[1] != 0)
		REG32(HW_MUTEX_PRI_ACQ(off)) = (hart << 8) | 0x10040;
	else
		REG32(HW_MUTEX_ACQ(off)) = (hart << 8) | 0x40;

	sts = REG32(HW_MUTEX_STATUS(hart, off));
	if (!(sts & HW_MUTEX_HELD))
		return -1;
	return ((sts >> 8) & 0xFF) == hart ? 0 : -1;
}

int hw_mutex_unlock_pri(u32 *desc)
{
	return hw_mutex_unlock(desc);
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

void plic_enable(u32 src)
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

void plic_disable(u32 src)
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

void plic_enable_wrapper(u32 src)
{
	plic_enable(src);
}

static void plic_init(void)
{
	u32 i;

	plic_set_threshold();

	for (i = 0; i < 192; i++) {
		plic_disable(i);
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

void plic_register_isr(u32 src, isr_fn_t handler)
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

	for (idx = 0; idx < NPU_TIMER_NUM; idx++) {
		if (timer_irq_map[idx] == src)
			return timer_clr_bit[idx];
	}
	npu_printf("Error: %s can't find intrBit for intSrc:%d\n",
		   "get_tmrIntrBit_byIntSrc", src);
	return 20;
}

static void timer_isr(int src)
{
	u32 bit = timer_get_bit((u32)src);
	u32 base;
	u32 tick;

	if (src >= 24 && src <= 31)
		base = NPU_TIMER1_BASE;
	else
		base = NPU_TIMER0_BASE;

	/* ack: rewrite the control word with only this timer's clear bit set */
	timer_prev_ctrl = REG32(base);
	REG32(base) = (timer_prev_ctrl & 0x1E0001EF) | (1u << bit);
	timer_int_count++;

	tick = timer_raw_tick + 1;
	timer_raw_tick = tick;

	if (tick % 100 == 0)
		timer_slow_tick++;

	if (timer_tod_usec + 100000 > 999999999u) {
		timer_tod_sec++;
		timer_tod_usec -= 999900000;
	} else {
		timer_tod_usec += 100000;
	}
}

void timer_init(int timer, int enable, int period)
{
	u32 base = (timer >= 8) ? NPU_TIMER1_BASE : NPU_TIMER0_BASE;
	u32 pair = 0;
	u32 ctrl;

	timer_int_count = 0;
	if ((u32)timer >= NPU_TIMER_NUM) {
		npu_printf("%s timer_no:%d is wrong, should be smaller than %d\n",
			   "timer_init", timer, NPU_TIMER_NUM);
		return;
	}
#if !defined(AN7581)
	if ((u32)(timer - 3) <= 3)
		pair = timer_pair_bit[timer - 3];
#endif
	if (!enable) {
		REG32(base) &= ~(1u << timer_bit_map[timer]);
		return;
	}

#if defined(AN7583)
	timer_clk_mhz = 50;
	npu_printf("%s: Timer clk is running at %d Mhz\n", "timer_init", 50);
	REG32(timer_reload_reg[timer]) = 50000 * (u32)period;
#else
	timer_clk_mhz = cpu_clock_div4();
	npu_printf("%s: Timer clk is running at %d Mhz\n", "timer_init",
		   timer_clk_mhz);
#if defined(AN7581)
	REG32(timer_reload_reg[timer]) =
		(u32)period * cpu_clock_div4() * 1000 / 100;
#else
	REG32(timer_reload_reg[timer]) = 1000 * (u32)period * cpu_clock_div4();
	plic_enable(timer_irq_map[timer]);
#endif
#endif
	ctrl = REG32(base) | (1u << timer_bit_map[timer]);
	if (pair)
		ctrl &= ~(1u << pair);
	REG32(base) = ctrl;
}

static void delay_1ms(u32 ms)
{
	volatile u32 i, j;

	for (i = 0; i < ms; i++)
		for (j = 0; j < 25000; j++)
			;
}

/* PLL clock: selector picks a frequency, bits[2:0]+1 is the divider */
#define PLL_CFG_REG 0x1FA201FC
#if defined(AN7583)
#define PLL_SEL_SHIFT   9
#define PLL_FREQ_TABLE  { 666, 800, 720, 600 }
#else
#define PLL_SEL_SHIFT   8
#define PLL_FREQ_TABLE  { 800, 750, 720, 600 }
#endif

u32 cpu_clock_get(void)
{
	static const u32 pll_freq[] = PLL_FREQ_TABLE;

	if (sim_mode_flag != 0)
		return 100;
	return pll_freq[(REG32(PLL_CFG_REG) >> PLL_SEL_SHIFT) & 3] /
	       ((REG32(PLL_CFG_REG) & 7) + 1);
}

static u32 cpu_clock_div2(void)
{
	if (sim_mode_flag != 0)
		return 50;
	return cpu_clock_get() >> 1;
}

u32 cpu_clock_div4(void)
{
	if (sim_mode_flag != 0)
		return 25;
	return cpu_clock_get() >> 2;
}

static void watchdog_timer_init(int enable, int period)
{
	if (!enable)
		return;
#if defined(AN7583)
	REG32(NPU_TIMER_WDT_RELOAD) = 50000 * (u32)period;
#else
	REG32(NPU_TIMER_WDT_RELOAD) = 1000 * (u32)period * cpu_clock_div4();
#endif
	REG32(NPU_TIMER0_BASE) &= ~0x20u;
	REG32(NPU_TIMER0_BASE) |= 0x2000020u;
}

#ifdef AN7581
static void multi_bank_timer_init(int enable, int prescale, int period, int bank)
{
	if (!enable) {
		REG32(NPU_TIMER_BANK_CTRL(bank)) &= 0xFDFFFFDFu;
		return;
	}
	REG32(NPU_TIMER_BANK_PRESCALE(bank)) = (prescale != -1 && prescale != 1)
		? 25000 * (u32)prescale : (u32)prescale;
	REG32(NPU_TIMER_BANK_RELOAD(bank)) = 1000 * (u32)period * cpu_clock_div4();
	REG32(NPU_TIMER_BANK_CTRL(bank)) &= ~0x20u;
	REG32(NPU_TIMER_BANK_CTRL(bank)) |= 0x2000020u;
}
#endif

static void cpu_timer_init(int idx, int enable, int period)
{
	u32 bit = 1u << idx;

	if (idx > 2)
		npu_printf("%s cpu_tmr:%d is wrong, should be 0 or 1\n",
			   "cpu_timer_init", idx);
	if (enable) {
#if defined(AN7583)
		REG32(CPU_TIMER_RELOAD(idx)) = 50000 * (u32)period;
#else
		REG32(CPU_TIMER_RELOAD(idx)) = 1000 * (u32)period *
			(sim_mode_flag ? 50u : 200u);
#endif
		REG32(CPU_TIMER_COUNTER(idx)) = 0;
		REG32(NPU_CPU_TIMER_BASE) |= bit;
	} else {
		REG32(NPU_CPU_TIMER_BASE) &= ~bit;
	}
}

void delay_us(u32 us)
{
	u32 target = 800 * us;
	u32 prev = csr_read(mcycle);
	u32 elapsed = 0;

	do {
		u32 cur = csr_read(mcycle);

		if (cur >= prev)
			elapsed += cur - prev;
		else
			elapsed += cur - prev - 1;
		prev = cur;
	} while (elapsed < target);
}

void delay_ms(u32 ms);

static void delay_ms_mcycle(u32 ms)
{
	u32 clk = cpu_clock_get();
	u32 target = 1000 * ms * clk;
	u32 prev = csr_read(mcycle);
	u32 elapsed = 0;

	do {
		u32 cur = csr_read(mcycle);

		if (cur >= prev)
			elapsed += cur - prev;
		else
			elapsed += cur - prev - 1;
		prev = cur;
	} while (elapsed < target);
}

void delay_ms(u32 ms)
{
	delay_ms_mcycle(ms);
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

	/* read queue registers; length and arg are 16-bit */
	base_ptr = REG32(MBQ_CTRL0(mbox_idx));
	max_cnt = REG32(MBQ_CTRL1(mbox_idx)) & 0xFFFF;
	rptr = REG32(MBQ_CTRL3(mbox_idx)) & 0xFFFF;

	/* set response status if not already set */
	if (!(rptr & 1)) {
		rptr = ((rptr & 0xFF00u) | (rptr & 0xE1) | 0x6) & 0xFFFF;
		REG32(MBQ_CTRL3(mbox_idx)) = rptr;
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
			REG32(MBQ_CTRL3(mbox_idx)) = (rptr & ~2u) | 2;
	}
}

static void mailbox_init(void)
{
	u32 i;
	u32 *callbacks;

	/* route mailbox n to core n-1 */
	for (i = 0; i < MAX_CORE_NUM; i++) {
		REG32(MBOX_INT_MASK(i + 1)) = (1u << i);
		plic_register_isr(8 + i, mbox_isr);
	}

	REG32(MBOX_INT_MASK0) = 256;
#if defined(AN7581)
	mbox_notify_mutex[0] = 30;
#else
	mbox_notify_mutex[0] = 14;
#endif
	mbox_notify_mutex[1] = 0;

	/* zero all dispatch tables */
	for (i = 0; i < MAX_CORE_NUM; i++)
		npu_memset(mbox_dispatch[i], 0, 80);

	/* register handlers into core 0's callback slots */
	callbacks = (u32 *)&mbox_dispatch[0][48];
#ifdef HAS_WIFI
	callbacks[0] = (u32)(void *)wifi_mail_dispatch;
#endif
	callbacks[1] = (u32)(void *)tunnel_mail_dispatch;

#ifdef HAS_TR471
#ifdef WIFI_KITE
	callbacks[4] = (u32)(void *)kite_wifi_config;
#elif defined(WIFI_EAGLE)
	callbacks[4] = (u32)(void *)eagle_wifi_config;
#endif
#endif

#ifdef HAS_TUNNEL
	callbacks[5] = (u32)(void *)hwnat_mail_dispatch;
#endif

#ifdef HAS_DBA
	{
		u32 *dba_cb = (u32 *)&mbox_dispatch[5][48];
		dba_cb[3] = (u32)(void *)dba_mail_handler;
	}
#endif
}

/* notify host via mailbox queue 8 */
int mbox_notify_host(u32 core_id, u32 func_id, u32 len)
{
	u32 timeout = 30;
	u32 sts;

	hw_mutex_lock_pri(mbox_notify_mutex);

	REG32(MBQ_CTRL0(MBQ_NOTIFY)) = core_id & 0xF;
	REG32(MBQ_CTRL1(MBQ_NOTIFY)) = len & 0xFFFF;
	REG32(MBQ_CTRL3(MBQ_NOTIFY)) = (func_id & 0xF) << 11;
	REG32(MBQ_CTRL2(MBQ_NOTIFY)) = REG32(MBQ_CTRL2(MBQ_NOTIFY)) + 1;

	sts = REG32(MBQ_CTRL3(MBQ_NOTIFY)) & 0xFFFF;
	while (!(sts & 2) && timeout--) {
		delay_ms_mcycle(1);
		sts = REG32(MBQ_CTRL3(MBQ_NOTIFY)) & 0xFFFF;
	}

	if (!(sts & 2))
		npu_printf("Error(%s): npu notify host timeout (func_id:%d, core_id:%d)\n",
			   "npu_notify_host", func_id, core_id);

	hw_mutex_unlock_pri(mbox_notify_mutex);
	return (sts >> 2) & 7;
}


/* ================================================================
 * Chip ID and module detection
 * ================================================================ */

static u32 chip_id;
static u32 chip_rev;

void chip_id_query(void)
{
	chip_id = REG32(CHIP_ID_REG);
	chip_rev = REG32(CHIP_VARIANT_REG);

	npu_printf("chip_id=0x%x, chip_rev=0x%x\n", chip_id, chip_rev);
}

/* USB power down */
void usb_powerdown(void)
{
	/* USB PHY power control */
	REG32(0x1FAC0700) |= (1u << 4);
	REG32(0x1FAE0700) |= (1u << 4);
}

/* Reboot */
static void npu_reboot(void)
{
	npu_printf("REBOOTING...\n");
	delay_ms_mcycle(10);
	REG32(0x1FB00040) = 0x80000001;
}


/* ================================================================
 * SRAM buffer management
 *
 * SRAM at 0x3E800000, bump-allocated with alignment.
 * Size and max alloc entries vary by SoC.
 * ================================================================ */

#define SRAM_BASE         0x3E800000
#if defined(AN7552)
#define SRAM_SIZE         0x40000
#define SRAM_MAX_ENTRIES  50
#elif defined(AN7581)
#define SRAM_SIZE         0x78000
#define SRAM_MAX_ENTRIES  100
#else /* AN7583 */
#define SRAM_SIZE         0x80000
#define SRAM_MAX_ENTRIES  100
#endif
#define SRAM_END          (SRAM_BASE + SRAM_SIZE - 2)
#define SRAM_ERROR_ADDR   (SRAM_BASE + SRAM_SIZE)

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

	if (sram_alloc_offset >= SRAM_SIZE && sram_alloc_count >= SRAM_MAX_ENTRIES) {
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

/* Reserved size per address type. Two classes: types up to 128 are the
 * WiFi rings and tables, 129 and above the bridge and tunnel buffers. */
struct sram_size_ent {
	u16 addr_type;
	u32 size;
};

static const struct sram_size_ent sram_size_lo[] = {
	{   1, 0x220C0 }, {   2, 0x01818 }, {   3, 0x01818 }, {   9, 0x003E8 },
	{  10, 0x003E8 }, {  11, 0x00078 }, {  14, 0x00600 }, {  15, 0x00600 },
	{  22, 0x01008 }, {  25, 0x00040 }, {  26, 0x00010 }, {  30, 0x00100 },
	{  18, 0x06800 }, {  16, 0x02020 }, {  17, 0x02020 }, {  28, 0x06800 },
	{  29, 0x01000 }, {  23, 0x00800 },
};

static const struct sram_size_ent sram_size_hi[] = {
	{ 138, 0x06000 }, { 129, 0x13FFF }, { 132, 0x04000 }, { 133, 0x10000 },
	{ 134, 0x01000 }, { 136, 0x08010 }, { 130, 0x00004 }, { 137, 0x00004 },
};

/* Types this reconstruction allocates that neither blob table lists, so
 * the numbering at those call sites does not match the blob yet. Sized
 * from the loops that fill them; the rest get a bounded default. */
#define SRAM_DEFAULT_SIZE  0x2000

static const struct sram_size_ent sram_size_ext[] = {
	{  12, 0x01000 },	/* reorder primary index pool, 2000 u16 */
	{  13, 0x02800 },	/* reorder secondary index pool, 5000 u16 */
};

static u32 sram_type_size(u32 addr_type)
{
	const struct sram_size_ent *t;
	u32 n, i;

	if (addr_type <= 128) {
		t = sram_size_lo;
		n = sizeof(sram_size_lo) / sizeof(sram_size_lo[0]);
	} else {
		t = sram_size_hi;
		n = sizeof(sram_size_hi) / sizeof(sram_size_hi[0]);
	}
	for (i = 0; i < n; i++) {
		if (t[i].addr_type == addr_type)
			return t[i].size;
	}

	n = sizeof(sram_size_ext) / sizeof(sram_size_ext[0]);
	for (i = 0; i < n; i++) {
		if (sram_size_ext[i].addr_type == addr_type)
			return sram_size_ext[i].size;
	}

	npu_printf("AddrType=%d is not in the size table, reserving 0x%x\n",
		   addr_type, SRAM_DEFAULT_SIZE);
	return SRAM_DEFAULT_SIZE;
}

u32 sram_buf_alloc(u32 addr_type)
{
	u32 size, result;

	if (addr_type == 0 || addr_type > 255)
		return 0;

	size = sram_type_size(addr_type);
	result = sram_buf_alloc_impl((u16)addr_type, size);
	if (result == SRAM_ERROR_ADDR)
		return 0;
	return result;
}

void sram_buf_init(void)
{
	npu_memset((void *)SRAM_BASE, 0, SRAM_SIZE);
	npu_memset(sram_alloc_table, 0, sizeof(sram_alloc_table));
	sram_buf_mutex[0] = 18;
	sram_buf_mutex[1] = 0;
	sram_alloc_offset = 0;
	sram_alloc_count = 0;
}

void sram_buf_dump(void)
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
 * NPU bridge (DMA channels between NPU and host)
 * ================================================================ */

#define NPU_BRIDGE_REG_BASE    0x1EC12000
#define BRIDGE_CH_CTRL(ch)     (NPU_BRIDGE_REG_BASE + 0x010 + (ch) * 0x10)
#define BRIDGE_CH_STATUS(ch)   (NPU_BRIDGE_REG_BASE + 0x050 + (ch) * 0x20)
#define BRIDGE_PKT_BUF_BASE_REG 0x1EC12010
#define BRIDGE_PKT_BUF_CFG    0x1EC12018

u32 npu_bridge_pkt_base;
static u32 bridge_tx_count[4];

u32 npu_bridge_addr(void)
{
	return npu_bridge_base + 0x10000;
}

void npu_bridge_buf_init(void)
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

	for (i = 0; i < 8; i++) {
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
 * DBA - Dynamic Bandwidth Allocation (AN7583 + WiFi)
 * ================================================================ */

#ifdef HAS_DBA

void dba_init(void)
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

void dba_main_loop(void)
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

int dba_mail_handler(u32 base, u32 cnt)
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

void dba_timer_handler(int src)
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

static u32 tr471_stats[15];

void tr471_main_init(void)
{
	npu_memset(tr471_stats, 0, sizeof(tr471_stats));
	npu_printf("%s init %d done\n", "tr471_main_init", 727);
}

#endif


/* ================================================================
 * Per-core main functions
 * ================================================================ */

static void __attribute__((noinline)) core0_main(void)
{
#ifdef HAS_WIFI
	wifi_pcie_desc_alloc();
#endif
	tdma_init();

#ifdef HAS_BME
	/* the BMGR path is only taken when the host asks for it; AN7583
	 * runs the buffer id pool instead and never touches the BMGR */
	if (tdma_bmgr_mode != 0) {
		npu_printf("do tdma_bmgr_init\n");
		tdma_bmgr_init();
	} else {
		bufid_pool_init();
	}
#elif defined(HAS_WIFI)
	buf_mgr_init();
#endif

	npu_bridge_buf_init();
	core0_wifi_init_wrapper();
	plic_register_isr(59, dbg_cnt_isr);

	npu_printf("%s\n", "core0_main");
#if defined(AN7552) && defined(WIFI_EAGLE)
	eagle_core0_loop();
#endif
}

static void __attribute__((noinline)) core1_main(void)
{
	npu_printf("%s\n", "core1_main");
#if defined(WIFI_EAGLE)
	eagle_rxdmad_loop();
	npu_printf("%s finish\n", "core1_wifi_init_wrapper");
#elif defined(HAS_WIFI)
	if (wifi_debug_flags & 1)
		wifi_pipeline_worker();
	else
		wifi_bridge_loop();
#endif
}

#if MAX_CORE_NUM > 2
static void __attribute__((noinline)) core2_main(void)
{
	npu_printf("%s\n", "core2_main");
	plic_register_isr(18, timer_isr);
#ifdef WIFI_EAGLE
	eagle_tx_fast_path();
#endif
}

static void __attribute__((noinline)) core3_main(void)
{
	npu_printf("%s\n", "core3_main");

	chip_id_query();
	usb_powerdown();
	core3_wifi_init_wrapper();
}

static void core4_wifi_init_wrapper(void)
{
#if defined(WIFI_EAGLE)
	eagle_rx_refill_loop();
#elif defined(HAS_WIFI)
	npu_printf("%s core 4 do nothing\n", "core4_wifi_init_wrapper");
#endif
}

static void __attribute__((noinline)) core4_main(void)
{
	npu_printf("%s\n", "core4_main");
	core4_wifi_init_wrapper();
}

static void __attribute__((noinline)) core5_main(void)
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
static void core6_wifi_init_wrapper(void)
{
#ifdef HAS_WIFI
	npu_printf("%s core 6 do nothing\n", "core6_wifi_init_wrapper");
#endif
}

static void __attribute__((noinline)) core6_main(void)
{
	npu_printf("%s\n", "core6_main");
	core6_wifi_init_wrapper();
}

static void core7_wifi_init_wrapper(void)
{
#ifdef HAS_WIFI
	npu_printf("%s core 7 do nothing\n", "core7_wifi_init_wrapper");
#endif
}

static void __attribute__((noinline)) core7_main(void)
{
#ifdef HAS_TUNNEL
	u32 pkt_len = 0, desc_ptr = 0;

	core7_wifi_init_wrapper();
	npu_printf("%s for npu tunnel offload\n", "core7_main");

	plic_register_isr(15, mbox_isr);
	npu_bridge_buf_init();

#ifdef HAS_TR471
	tr471_main_init();
#endif

	tunnel_init();

	while (1) {
		while (tunnel_ecn_enabled) {
			l4s_ecn_process(1);
			l4s_ecn_process(2);
			tunnel_process();
			if (tunnel_test_active == 0)
				goto dequeue;
		}
		tunnel_process();
		if (tunnel_test_active == 0) {
dequeue:
			if (tunnel_dequeue(7, &pkt_len, &desc_ptr) == 0 &&
			    tunnel_offload_handler(7, pkt_len, (u32 *)desc_ptr) == -1)
				tunnel_pkt_drop(7, pkt_len, desc_ptr);
		}
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

/* Returns the mepc to resume at. Only the machine external interrupt goes to
 * the PLIC; anything else is reported and stepped over. Routing an exception
 * through the ISR table lands in default_isr, whose npu_printf re-enters a
 * printf that already holds the printf mutex, and a held-mutex acquire stalls
 * the NPU bus. */
u32 trap_dispatch(u32 mcause, u32 mepc, u32 *sp, u32 ra)
{
	if (mcause == MCAUSE_MACHINE_EXT_IRQ) {
		call_isr_by_src(REG32(PLIC_CLAIM_REG) - 1);
		return mepc;
	}

	npu_printf("UnKnow Interrupts (mcause:0x%x, RA:0x%x, SP:0x%x, EPC:%x)\n",
		   mcause, ra, (u32)sp, mepc);

	/* step over the faulting instruction; it may be compressed */
	return mepc + (((*(volatile u16 *)mepc) & 3) == 3 ? 4 : 2);
}

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
		"csrr a1, mepc\n"
		"mv a2, sp\n"
		"mv a3, ra\n"
		"call trap_dispatch\n"
		"csrw mepc, a0\n"
		"li t0, 0x1800\n"
		"csrs mstatus, t0\n"
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

/* .data + .bss must fit the global-variable window of the NPU SRAM */
#define GLB_VAR_SRAM_SIZE  0x7800
extern char __bss_end[];

void npu_init(void)
{
	u32 hart = get_hartid();
	u32 mib21;

	/* hart 0 clears sync flag; others wait */
	if (hart != 0)
		delay_ms_mcycle(10);
	else
		core_sync_flag = 0;
	if (hart != 0 && core_sync_flag == 0) {
		do
			delay_ms_mcycle(100);
		while (core_sync_flag == 0);
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
		/* MIB21 gates npu_printf and does not survive the reset pulse */
		mib21 = REG32(NPU_MIB21);
		sim_mode_flag = REG32(NPU_MIB12);
		REG32(NPU_SCU_RSTCTRL1) = NPU_SCU_RST_ALL;
		delay_ms_mcycle(1);
		REG32(NPU_SCU_RSTCTRL1) = 0;
		delay_ms_mcycle(1);
		REG32(NPU_THREAD_ENABLE) = 4;
		REG32(NPU_THREAD_ENABLE) = 1;
		delay_ms_mcycle(1);
		REG32(NPU_MIB0) = ALL_FF;
		boot_uart_init();
		REG32(NPU_MIB21) = mib21;
		npu_printf("core freq at %d MHz\n", cpu_clock_get());
		if ((u32)__bss_end - NPU_SRAM_BASE > GLB_VAR_SRAM_SIZE)
			npu_printf("Error: OVER GLB_VAR_SRAM_SIZE. "
				   "_data:0x%x, __bss_end:0x%x\n",
				   NPU_SRAM_BASE, (u32)__bss_end);
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
