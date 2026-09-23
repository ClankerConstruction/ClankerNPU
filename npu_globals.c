/*
 * AN75XX NPU firmware - shared global variables
 *
 * Every global the firmware shares between files is defined here, in one
 * translation unit. The Makefile compiles this file first so that the
 * order below is the order of the .data section, and therefore the order
 * of npu_data.bin.
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
#ifdef WIFI_KITE
u32 tdma_bmgr_mode = 1;
#else
u32 tdma_bmgr_mode;
#endif
u32 npu_printf_prefix = 1;

#ifdef HAS_DBA
u8 dba_cfg4 = 1;
#endif

#ifdef HAS_TUNNEL
u32 l4s_qid = 7;
u32 l4s_qlen_thresh = 100;
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
	[1] = tunnel_mail_nop,
	[2] = tunnel_mail_vxlan_mtu,
	[3] = tunnel_mail_store_srv6,
	[4] = tunnel_mail_set_srv6_addr,
	[5] = tunnel_mail_frag_mtu,
	[6] = tunnel_mail_bridge_dbg,
	[7] = tunnel_mail_map_info,
	[8] = tunnel_mail_l4s,
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
u32 l4s_mark_count;
u32 l4s_qlen;
u32 ppe_module_idx;
u8 ppe_module_ver;
u32 fragment_mtu[4];
u32 tunnel_pending[8];
u32 tunnel_credit[8];
u32 tunnel_ipv6_frag_id;
u32 tunnel_v4_reasm_hdroff;
u32 tunnel_v4_reasm_len;
u32 tunnel_v4_reasm_desc;
u32 tunnel_v6_reasm_hdroff;
u32 tunnel_v6_reasm_len;
u32 tunnel_v6_reasm_desc;
u32 tunnel_encap_mtu = 1500;
u32 tunnel_map_info_base;
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
volatile u32 rxd_5g_init_done;
u32 rxd_base_2g;
u32 rxd_2g_bufid_base;
volatile u32 rxd_2g_init_done;
u32 rxd_5g_cpu_idx;
u32 rxd_5g_flush_tick;
u32 rxd_2g_cpu_idx;
u32 rxd_2g_flush_tick;
u16 rxd_5g_bufid_table[1536];

/* WiFi misc (declared early for reorder/BA functions) */
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
u16 wifi_retry_limit;

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
u8 wifi_wait_band_2g;
u8 wifi_wait_band_5g;

/* WiFi per-port band assignment */
u8 wifi_port_band_2g[16];
u8 wifi_port_band_5g[16];

/* host tx packet buffer, published over the mailbox */
volatile u32 npu_tx_pkt_buf_addr;
u32 tdma_rx_dscp_base[2];
u32 tdma_rx_desc_count;
u32 tdma_rx_alloc_fail;
u32 tdma_rx_ridx[2];

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

/* Host ring bases, keyed by the ring index the host puts in interfaceID. */
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
volatile u8 eagle_rx_ring_init_done[2];
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
s8 eagle_rxdmad_kick;
u8 eagle_rxdmad_abort;
u8 eagle_rxdmad_segs;
u32 eagle_rxdmad_seglen;
u32 eagle_seg_bufid[7];
u16 eagle_seg_len[7];

/* rx rings */
u32 eagle_rx_ring_ridx[2];
s8 eagle_rx_ring_kick[2];

/* wifi tx rings */
u32 eagle_tx_ring_desc[2];
u16 eagle_tx_ring_cpu_idx[2];

/* wifi tx done ring */
u32 eagle_txdone_ridx;
u8 eagle_tx_first_push[2];
u8 eagle_in_first[2];
u8 eagle_txdone_kick;
#endif

#if defined(WIFI_KITE) && defined(HAS_TR471)
volatile u32 kite_wifi_cfg[26];
volatile u32 kite_test_active;
#endif
