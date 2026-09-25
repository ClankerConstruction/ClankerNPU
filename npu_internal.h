#ifndef NPU_INTERNAL_H
#define NPU_INTERNAL_H

#include "npu_config.h"
#include "npu_types.h"
#include "npu_regs.h"

/* rx buffer ids: 2 KB each over the WiFi packet buffer */
#if defined(WIFI_KITE)
#define BUFID_POOL_ENTRIES    5600
#elif defined(AN7552)
#define BUFID_POOL_ENTRIES    11200
#else
#define BUFID_POOL_ENTRIES    12288
#endif

void xpon_license_check(void);

typedef int (*wifi_mail_fn_t)(u32 *msg);

/* npu_util.c */
void *npu_memset(void *dst, int c, u32 n);
void *npu_memcpy(void *dst, const void *src, u32 n);
u32 npu_strlen(const char *s);
char get_core_char(void);

/* npu_mutex.c */
int hw_mutex_lock(u32 *desc);
int hw_mutex_unlock(u32 *desc);
int hw_mutex_lock_pri(u32 *desc);
int hw_mutex_unlock_pri(u32 *desc);

/* npu_plic.c */
void plic_init(void);
void plic_enable(u32 src);
void plic_disable(u32 src);
void plic_enable_wrapper(u32 src);
void plic_register_isr(u32 src, isr_fn_t handler);
void call_isr_by_src(u32 src);

/* npu_timer.c */
void timer_init(int timer, int enable, int period);
void timer_isr(int src);
u32 timer_get_bit(u32 src);
u32 cpu_clock_get(void);
u32 cpu_clock_div4(void);
void delay_us(u32 us);
void delay_ms(u32 ms);
void delay_1ms(u32 ms);
void delay_ms_mcycle(u32 ms);

/* npu_mbox.c */
void mailbox_init(void);
void mbox_isr(int src);
int mbox_notify_host(u32 core_id, u32 func_id, u32 len);

/* npu_sram.c */
u32 sram_buf_alloc(u32 addr_type);
void sram_buf_init(void);
void sram_buf_dump(void);

/* npu_bridge.c */
void npu_bridge_buf_init(void);
u32 npu_bridge_addr(void);
int npu_bridge_egress(u32 ch, u32 w0, u32 w1, u32 w2,
		      u32 w3, u32 w4, u32 w5, u32 w6);

/* npu_main.c — chip ID */
int mtk_usb_powerdown(u32 port);
s32 chip_cap_query(u32 idx, u32 query);

/* npu_printf.c */
int npu_printf(const char *fmt, ...);
void npu_hexdump(const char *tag, u32 addr, u32 len);
void wifi_pcie_desc_alloc(void);
int boot_printf(const char *fmt, ...);
void uart_rx_isr(int src);
void boot_uart_init(void);

/* npu_wifi.c */
int hostadpt_init(void);
void tdma_init(void);
void dbg_cnt_isr(int src);
#ifdef HAS_BME
void bufid_pool_init(void);
void rx_bufid_pool_reset(void);
void tdma_tx_wait_idle(void);
void tdma_bmgr_reinit(void);
void tdma_bmgr_init(void);
void tdma_tx_init(void);
#endif
void core0_wifi_init_wrapper(void);
void core3_wifi_init_wrapper(void);
#ifdef HAS_WIFI
void buf_mgr_init(void);
int wifi_mail_dispatch(u32 base, u32 cnt);
void wifi_bridge_loop(void);
#ifdef WIFI_KITE
void kite_core1_loop(void);
void kite_core2_loop(void);
#endif
#endif
#ifdef WIFI_KITE
int wifi_mail_get_npu_info(u32 *msg);
int wifi_mail_get_last_rate(u32 *msg);
int wifi_mail_get_counter(u32 *msg);
int wifi_mail_get_dbg_counter(u32 *msg);
int wifi_mail_get_rxdesc_base(u32 *msg);
int wifi_mail_get_wcid_dbg_counter(u32 *msg);
int wifi_mail_get_dma_addr(u32 *msg);
int wifi_mail_get_ring_size(u32 *msg);
int wifi_mail_get_mdc_lock(u32 *msg);
int wifi_mail_get_dump_mapping(u32 *msg);
int wifi_mail_set_pcie_addr(u32 *msg);
int wifi_mail_set_desc(u32 *msg);
int wifi_mail_set_init_done(u32 *msg);
int wifi_mail_set_tran_to_cpu(u32 *msg);
int wifi_mail_set_ba_win_size(u32 *msg);
int wifi_mail_set_driver_model_cmd(u32 *msg);
int wifi_mail_set_del_sta(u32 *msg);
int wifi_mail_set_dram_ba_node(u32 *msg);
int wifi_mail_set_pkt_buf(u32 *msg);
int wifi_mail_set_test_noba(u32 *msg);
int wifi_mail_set_flushone(u32 *msg);
int wifi_mail_set_flushall(u32 *msg);
int wifi_mail_set_force_cpu(u32 *msg);
int wifi_mail_set_pcie_state(u32 *msg);
int wifi_mail_set_port_type(u32 *msg);
int wifi_mail_set_retry(u32 *msg);
int wifi_mail_set_bar_info_cmd(u32 *msg);
int wifi_mail_set_fast_flag_cmd(u32 *msg);
int wifi_mail_set_band0_cpu(u32 *msg);
int wifi_mail_set_tx_ring_pcie(u32 *msg);
int wifi_mail_set_tx_desc_hw(u32 *msg);
int wifi_mail_set_tx_buf_hw(u32 *msg);
int wifi_mail_set_rx_txdone_hw(u32 *msg);
int wifi_mail_set_tx_pkt_buf(u32 *msg);
int wifi_mail_set_txrx_reg(u32 *msg);
int wifi_mail_set_debug_flag(u32 *msg);
int wifi_mail_set_wait_inode_cfg(u32 *msg);
int wifi_mail_set_wait_inode_stop(u32 *msg);
int wifi_mail_set_pcie_swap(u32 *msg);
int wifi_mail_set_ratelimit(u32 *msg);
int wifi_mail_set_arht_chip_info(u32 *msg);
#endif
#ifdef WIFI_KITE
int kite_wifi_config(u32 base, u32 cnt);
#endif
#ifdef WIFI_EAGLE
int eagle_mail_set_pcie_addr(u32 *msg);
int eagle_mail_set_desc(u32 *msg);
int eagle_mail_set_init_done(u32 *msg);
int eagle_mail_set_tran_to_cpu(u32 *msg);
int eagle_mail_set_ba_win_size(u32 *msg);
int eagle_mail_set_driver_model(u32 *msg);
int eagle_mail_set_del_sta(u32 *msg);
int eagle_mail_set_dram_ba_node(u32 *msg);
int eagle_mail_set_pkt_buf(u32 *msg);
int eagle_mail_set_test_noba(u32 *msg);
int eagle_mail_set_flushone(u32 *msg);
int eagle_mail_set_flushall(u32 *msg);
int eagle_mail_set_force_cpu(u32 *msg);
int eagle_mail_set_pcie_state(u32 *msg);
int eagle_mail_set_port_type(u32 *msg);
int eagle_mail_set_retry(u32 *msg);
int eagle_mail_set_bar_info(u32 *msg);
int eagle_mail_set_fast_flag(u32 *msg);
int eagle_mail_set_band0_cpu(u32 *msg);
int eagle_mail_set_tx_ring_pcie(u32 *msg);
int eagle_mail_set_tx_desc_hw(u32 *msg);
int eagle_mail_set_tx_buf_hw(u32 *msg);
int eagle_mail_set_rx_txdone_hw(u32 *msg);
int eagle_mail_set_tx_pkt_buf(u32 *msg);
int eagle_mail_set_txrx_reg(u32 *msg);
int eagle_mail_set_debug_flag(u32 *msg);
int eagle_mail_set_inode_cfg(u32 *msg);
int eagle_mail_set_inode_stop(u32 *msg);
int eagle_mail_set_pcie_swap(u32 *msg);
int eagle_mail_set_ratelimit(u32 *msg);
int eagle_mail_set_arht_chip_info(u32 *msg);
int eagle_mail_get_npu_info(u32 *msg);
int eagle_mail_get_last_rate(u32 *msg);
int eagle_mail_get_counter(u32 *msg);
int eagle_mail_get_dbg_counter(u32 *msg);
int eagle_mail_get_rxdesc_base(u32 *msg);
int eagle_mail_get_wcid_dbg_counter(u32 *msg);
int eagle_mail_get_dma_addr(u32 *msg);
int eagle_mail_get_ring_size(u32 *msg);
int eagle_mail_get_mdc_lock(u32 *msg);
int eagle_mail_get_dump_mapping(u32 *msg);
int eagle_wifi_config(u32 base, u32 cnt);
void eagle_rx_init(void);
void eagle_rxdmad_loop(void);
void eagle_tx_fast_path(void) __attribute__((noreturn));
void eagle_core3_loop(void) __attribute__((noreturn));
void eagle_rx_refill_loop(void) __attribute__((noreturn));
void eagle_core0_loop(void) __attribute__((noreturn));
#endif

/* npu_tunnel.c, npu_ppe.c, npu_l4s.c */
int tunnel_mail_dispatch(u32 base, u32 cnt);
#ifdef HAS_TUNNEL
int tunnel_mail_store_hdr(u32 base, u32 cnt);
int tunnel_mail_store_srv6(u32 base, u32 cnt);
int tunnel_mail_set_srv6_addr(u32 base, u32 cnt);
int tunnel_mail_frag_mtu(u32 base, u32 cnt);
int tunnel_mail_nop(u32 base, u32 cnt);
int tunnel_mail_vxlan_mtu(u32 base, u32 cnt);
int tunnel_mail_bridge_dbg(u32 base, u32 cnt);
int tunnel_mail_map_info(u32 base, u32 cnt);
int tunnel_mail_l4s(u32 base, u32 cnt);
int l4s_set_config(u32 cmd, u32 arg);
void npu_bridge_debug(u32 op);
int hwnat_mail_dispatch(u32 base, u32 cnt);
void tunnel_init(void);
s32 tunnel_dequeue(u32 port, u32 *pkt_len, u32 *desc_ptr);
s32 tunnel_offload_handler(u32 port, u32 pkt_len, u32 *desc);
void tunnel_pkt_drop(u32 port, u32 pkt_len, u32 desc);
void l4s_ecn_process(u32 port);
#endif

/* npu_dba.c */
#ifdef HAS_DBA
int dba_mail_handler(u32 base, u32 cnt);
void core5_dba_main(void) __attribute__((noreturn));
#endif

/* npu_tr471.c */
#ifdef HAS_TR471
void tr471_main_init(void);
#endif

/* shared globals (defined in npu_globals.c) */
extern u32 npu_max_frame_size;
extern u32 npu_reset_pending;
extern u32 tdma_bmgr_mode;
extern u32 npu_printf_prefix;
extern u32 sim_mode_flag;
extern u32 config_flags;

extern u32 printf_mutex_desc[2];
extern char printf_buf[1024];
extern u32 uart_cmd_idx;
extern u8 uart_cmd_buf[32];

extern u32 mbox_notify_mutex[2];

extern u32 timer_irq_map[NPU_TIMER_NUM];
extern u32 timer_clr_bit[NPU_TIMER_NUM];
extern u32 timer_bit_map[NPU_TIMER_NUM];
extern u32 timer_counter_reg[NPU_TIMER_NUM];
extern u32 timer_reload_reg[NPU_TIMER_NUM];
extern volatile u32 timer_raw_tick;
extern volatile u32 timer_slow_tick;
extern u32 timer_int_count;
/* kite ageing clock: AN7552 counts timer ticks, the others the slow tick */
#ifdef AN7552
#define KITE_TICK (*(volatile u32 *)&timer_int_count)
#else
#define KITE_TICK timer_slow_tick
#endif
extern u32 timer_prev_ctrl;
extern u32 timer_clk_mhz;
extern u32 timer_tod_sec;
extern u32 timer_tod_usec;
extern u32 timer_context[10];
#if !defined(AN7581)
extern u32 timer_pair_bit[4];
#endif


#ifdef HAS_WIFI
extern wifi_mail_fn_t get_wait_func_table[];
extern wifi_mail_fn_t set_wait_func_table[];
#endif
#ifdef WIFI_KITE
extern const char wifi_chip_names[6][8];
#endif

extern mbox_handler_t mbox_pri_handlers[10];
extern u8 hwnat_cds;
extern u8 hwnat_xpon_hal_api_ng;
extern u8 hwnat_wan_xsi;
extern u8 hwnat_ct_joyme4;
extern u8 hwnat_max_packet_2000;
extern u8 hwnat_ready;
extern u32 hwnat_ppe_type;
extern u32 hwnat_wan_mode;
extern u32 hwnat_ae_wan_sel;

extern u32 exception_handlers[12];

extern u8 uart_desc[2][32];
extern u32 mutex_initial_state;
extern u32 mutex_desc_pairs[512];

extern s32 wifi_chip_index;
extern const char delay_name[];

extern mbox_handler_t mbox_core_handlers[6];

extern u8 printf_cfg_flags[4];
extern u8 printf_desc[8];
extern u8 printf_flag;

extern u32 timer_ref_counts[8];
extern u8 srv6_my_ipv6[16];
extern u32 timer_isr_context[12];
extern isr_fn_t timer_isr_handler0;
extern isr_fn_t timer_isr_handler1;
extern isr_fn_t timer_isr_handler2;
extern isr_fn_t timer_callback;

extern u32 bme_base_addr;
extern u32 bme_config;
extern u32 bme_desc_count;
extern u32 bme_status;
extern u32 bme_ring_state[4];
extern u32 tdma_bme_dscp_idx;
extern u32 tdma_bme_dscp_base;
extern u32 npu_bridge_base;

extern isr_fn_t plic_isr_table[192];
extern u32 sram_buf_mutex[2];
extern u32 sram_buf_pad[4];
extern u32 sram_buf_max_use;
extern u32 sram_buf_cur_idx;
extern u16 sram_buf_entries[200];

extern u32 wifi_state[256];

extern mbox_handler_t tunnel_func_table[10];
#ifdef HAS_TUNNEL
extern u8 tunnel_srv6_hdr_len[8];
extern volatile u32 tunnel_ecn_enabled;
extern u32 l4s_debug_enable;
extern u32 l4s_pkt_count;
extern u32 l4s_log_phase;
extern u32 l4s_mark_count;
extern u32 l4s_qid;
extern u32 l4s_qlen;
extern u32 ppe_module_idx;
extern u8 ppe_module_ver;
extern u32 fragment_mtu[4];
extern u32 tunnel_pending[8];
extern u32 tunnel_credit[8];
extern u32 tunnel_ipv6_frag_id;
extern u32 tunnel_v4_reasm_hdroff;
extern u32 tunnel_v4_reasm_len;
extern u32 tunnel_v4_reasm_desc;
extern u32 tunnel_v6_reasm_hdroff;
extern u32 tunnel_v6_reasm_len;
extern u32 tunnel_v6_reasm_desc;
extern u32 tunnel_encap_mtu;
extern u32 tunnel_map_info_base;
extern u32 tunnel_config_ptrs[4];
extern u8 tunnel_config_area[0x500];
extern u32 l4s_qlen_thresh;
extern u8 tunnel_cfg_flag;
extern u32 tunnel_dispatch_ptr;
extern u8 tunnel_ctx[3][1024];
#endif

#ifdef HAS_DBA
extern u8 dba_new_en_cur;
extern u8 dba_log_en;
extern u8 dba_single_onu_en;
extern u32 dba_rpt_step;
extern u8 dba_new_en;
extern u32 dba_burst_ovh;
#endif

extern u8 plic_threshold_table[8];
extern volatile u32 plic_isr_init_done;
extern volatile u32 core_sync_flag;

extern u32 wifi_ext_state[32];

extern u32 reorder_pri_idx_pool;
extern u32 reorder_sec_idx_pool;
extern u16 reorder_pri_widx;
extern u16 reorder_pri_ridx;
extern u16 reorder_sec_widx;
extern u16 reorder_sec_ridx;
extern u32 reorder_alloc_mutex[2];
extern u32 reorder_free_mutex[2];

extern u32 ba_mutex_5g[2];
extern u32 ba_mutex_2g[2];
extern u8 wifi_dbdc_mode;
extern u32 ba_table_a;
extern u32 ba_table_b;

extern u8 wifi_pcie_state[2];

extern u32 pcie_base_5g;
extern u32 pcie_base_2g;

extern u32 rxd_base_5g;
extern volatile u32 rxd_5g_init_done;
extern u32 rxd_base_2g;
extern u32 rxd_2g_bufid_base;
extern volatile u32 rxd_2g_init_done;
extern u32 rxd_5g_cpu_idx;
extern u32 rxd_5g_flush_tick;
extern u32 rxd_2g_cpu_idx;
extern u32 rxd_2g_flush_tick;
extern u16 rxd_5g_bufid_table[1536];

extern u8 wifi_debug_flags;
extern u16 wifi_retry_limit;

extern u8 wifi_driver_model;
extern u8 wifi_pcie_port_type;
extern u8 wifi_band_cap;
extern u8 wifi_force_to_cpu;
extern u8 wifi_no_ba_test;
extern u8 wifi_band0_on_cpu;
extern u16 wifi_flushall_timeout;
extern u16 wifi_flushone_timeout;
extern u32 wifi_pkt_buf_addr;
extern u32 wifi_dram_ba_node_addr;
extern u32 wifi_pcie_desc_base;


extern u32 queue_mutex_2g[2];
extern u32 queue_mutex_5g[2];
extern u32 queue_mutex_rx_2g[2];
extern u32 queue_mutex_rx_5g[2];

extern u32 ba_node_pool_base;

extern u8 wifi_wait_state_2g[16];
extern u8 wifi_wait_state_5g[16];
extern u8 wifi_wait_band_2g;
extern u8 wifi_wait_band_5g;
extern u8 wifi_port_band_2g[16];
extern u8 wifi_port_band_5g[16];

extern volatile u32 npu_tx_pkt_buf_addr;
extern u32 tdma_rx_dscp_base[2];
extern u32 tdma_rx_desc_count;
extern u32 tdma_rx_alloc_fail;
extern u32 tdma_rx_ridx[2];
void tdma_rx_init(void);
void np_skb_tx_force_reset(void);

#ifdef WIFI_KITE
extern u32 ratelimit_table[32];
extern u32 arht_chip_info[6];
extern u32 arht_phy_tx_gpio;
extern u32 arht_chip_info_valid;
extern u32 npu_rx_bytes_entry[2][256];
extern u32 npu_rx_pkts_entry[2][256];
extern u32 apcli_count_2g[2];
extern u32 apcli_count_5g[2];
extern u32 apcli_byte_count_2g[2];
extern u32 apcli_byte_count_5g[2];
#endif

#ifdef WIFI_EAGLE
extern u32 eagle_rx_ring_pcie_base[2];
extern u32 eagle_msdu_pg_pcie_base;
extern u32 eagle_ind_cmd_pcie_base;
extern u32 eagle_txdone_pcie_base;
extern u16 eagle_txdone_ring_cnt;
extern u32 eagle_tx_ring_pcie_base[2];
extern u32 eagle_tx_ring_pcie_base_r3;
extern u32 eagle_txd_space[2];
extern u32 eagle_tx_buf_space_pg[2];
extern u32 eagle_rx_txdone_desc_base;
extern u32 eagle_msdu_pg_desc_base;
extern u32 eagle_pkt_buf_addr;

extern u32 eagle_dram_ba_node_addr;
extern u32 eagle_icv_err_table;
extern u16 eagle_retry_times;
extern u8 eagle_pcie_port_type;
extern u8 eagle_pcie_state[2];
extern u8 eagle_rro_mode;
extern u8 eagle_test_noba;
extern u32 eagle_txdone_id_base;
extern u32 eagle_chip_info[6];
extern u32 eagle_phy_tx_gpio;
extern u32 eagle_stage_buf0;
extern u32 eagle_stage_buf1;
extern u32 eagle_stage_base[2];
extern u16 eagle_stage_widx[2];
extern u16 eagle_stage_ridx[2];
extern u32 eagle_rx_ring_desc_base[2];
extern u32 eagle_ind_cmd_desc_base;
extern u16 eagle_rx_ring_size[2];
extern u16 eagle_rx_ring_cpu_idx[2];
extern volatile u8 eagle_rx_ring_init_done[2];
extern u16 eagle_rx_ring_bufid[2][1536];
extern volatile u32 eagle_rx_en;
extern volatile u32 eagle_tx_en;
extern volatile u32 eagle_init_done;
extern volatile u32 eagle_rx_busy;
extern volatile u8 eagle_rro_state;
extern volatile u8 eagle_txq_state;
extern volatile u8 eagle_stopping;
extern volatile u8 eagle_rx_stopped;
extern volatile u8 eagle_fastpath_en;
extern u8 eagle_rxdmad_on_core2;
extern u32 eagle_rro_addr_elem[128];
extern u32 eagle_session_tbl;
extern u32 eagle_emi_cidx;
extern u8 eagle_emi_cidx_valid;
extern u32 eagle_txq_base[2];
extern u16 eagle_txq_widx[2];
extern u16 eagle_txq_ridx[2];
extern u32 eagle_mseg_base[2];
extern u16 eagle_mseg_widx[2];
extern u16 eagle_mseg_ridx[2];
extern u32 eagle_txq_mutex[2];
extern u16 eagle_mseg_retry;
extern u32 eagle_rxdmad_ridx;
extern u8 eagle_rxdmad_gen;
extern s8 eagle_rxdmad_kick;
extern u8 eagle_rxdmad_abort;
extern u8 eagle_rxdmad_segs;
extern u32 eagle_rxdmad_seglen;
extern u32 eagle_seg_bufid[7];
extern u16 eagle_seg_len[7];
extern u32 eagle_rx_ring_ridx[2];
extern s8 eagle_rx_ring_kick[2];
extern u32 eagle_tx_ring_desc[2];
extern u16 eagle_tx_ring_cpu_idx[2];
extern u32 eagle_txdone_ridx;
extern u8 eagle_tx_first_push[2];
extern u8 eagle_in_first[2];
extern u8 eagle_txdone_kick;
extern volatile u32 eagle_rro_cfg[26];
extern volatile u32 eagle_rro_active;
#if defined(AN7552)
extern volatile u8 *eagle_sync;
#endif
#endif

#if defined(WIFI_KITE) && defined(HAS_TR471)
extern volatile u32 kite_wifi_cfg[26];
extern volatile u32 kite_test_active;
#endif

/* ================================================================
 * Field debug block: fixed SRAM window the host reads and writes
 * with sys memrl / memwl at NDBG_BASE & 0x1FFFFFFF. See docs/debug.md.
 * ================================================================ */

#define NDBG_BASE	0x3E906800
#define NDBG_SIZE	0x1000
/* tags and text read in order in a word dump, first char in the MSB */
#define NDBG_TAG(a, b, c, d)	((u32)(a) << 24 | (b) << 16 | (c) << 8 | (d))
#define NDBG_MAGIC	NDBG_TAG('N', 'D', 'B', 'G')
#define NDBG_LAYOUT	1

/* subsystem ids: bit n of trace_mask, print_mask and the status mask */
enum {
	NDBG_MBOX, NDBG_WIFI, NDBG_TDMA, NDBG_TUNNEL, NDBG_L4S,
	NDBG_PPE, NDBG_DBA, NDBG_SRAM, NDBG_TRAP, NDBG_STATS = 15,
};

/* counter slots: plain increments, exact when one hart owns the slot */
enum {
	NC_TUN_PKTS, NC_TUN_VXLAN_ENC, NC_TUN_VXLAN_DEC, NC_TUN_SRV6_ENC,
	NC_TUN_SRV6_END, NC_TUN_MAP, NC_TUN_FRAG, NC_TUN_REASM,
	NC_TUN_DROP, NC_TUN_INVALID, NC_BRIDGE_EGRESS_FAIL,
	NC_L4S_PKTS = 12, NC_L4S_MARKS, NC_L4S_QLEN,
	NC_PPE_MAILS = 16, NC_PPE_LAST,
	NC_DBA_MAILS = 20, NC_DBA_FRAMES,
	NC_TDMA_TX_FULL = 24,
	NC_SRAM_ALLOCS = 32, NC_SRAM_USED,
	NC_MAX = 96,
};

struct ndbg_hart {
	u32 loop;	/* NDBG_TAG of the loop the hart runs */
	u32 beat;	/* bumped on every pass of that loop */
	u32 in_isr;
	u32 trace_wr;
	u32 mails;	/* mailbox calls handled */
	u32 last_mail;	/* slot << 24 | first word low 16 bits << 8 | ret */
	u32 trap_cnt;	/* exceptions taken */
	u32 mcause, mepc, mtval, ra, sp;
};

struct ndbg_trace {
	u32 tick;	/* timer tick */
	u32 id;		/* hart << 28 | subsystem << 20 | event */
	u32 a, b;
};

struct ndbg {
	u32 magic, layout, size, harts;			/* 0x000 */
	u32 version[12];				/* 0x010 */
	u32 trace_mask, print_mask, cmd_hart, cmd;	/* 0x040 */
	u32 arg[3], ret[2], done, status;		/* 0x050 */
	u32 rsv[5];					/* 0x06C */
	struct ndbg_hart hart[8];			/* 0x080 */
	struct { u32 tag, addr; } sym[16];		/* 0x200 */
	u32 cnt[NC_MAX];				/* 0x280 */
	struct ndbg_trace trace[8][16];			/* 0x400 */
	u32 buf[256];					/* 0xC00 */
};

#define ndbg	((volatile struct ndbg *)NDBG_BASE)

void npu_dbg_init(void);
void npu_dbg_service(u32 hart);
void npu_dbg_trace(u32 sub, u32 ev, u32 a, u32 b);
void __attribute__((noreturn)) npu_dbg_idle(void);

/* once per pass of a hart's main loop */
static inline void npu_dbg_poll(void)
{
	u32 h = get_hartid();

	ndbg->hart[h].beat++;
	if (ndbg->cmd != 0 && ndbg->cmd_hart == h)
		npu_dbg_service(h);
}

static inline void npu_dbg_loop(u32 tag)
{
	ndbg->hart[get_hartid()].loop = tag;
}

#define NDBG_TRACE(sub, ev, a, b) do {					\
	if (ndbg->trace_mask & (1u << (sub)))				\
		npu_dbg_trace((sub), (ev), (u32)(a), (u32)(b));		\
} while (0)

#define NDBG_PRINTING(sub)	(ndbg->print_mask & (1u << (sub)))
#define NDBG_CNT(i)		(ndbg->cnt[(i)]++)
#define NDBG_SET(i, v)		(ndbg->cnt[(i)] = (u32)(v))

#endif /* NPU_INTERNAL_H */
