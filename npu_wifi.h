#ifndef NPU_WIFI_H
#define NPU_WIFI_H

/*
 * Declarations shared between the npu_wifi_*.c files, npu_tdma.c and
 * npu_hostadpt.c. Nothing outside the WiFi offload includes this header;
 * what the rest of the firmware calls is in npu_internal.h.
 */

#include "npu_internal.h"

/* ================================================================
 * Hardware
 * ================================================================ */

/* TDMA HW registers */
#define TDMA_TX_RING0_BASE    0x1FB50800
#define TDMA_TX_RING0_CFG     0x1FB50804
#define TDMA_TX_RING1_BASE    0x1FB50810
#define TDMA_TX_RING1_CFG     0x1FB50814
#define TDMA_TX_RING0_IDX     0x1FB50808
#define TDMA_TX_RING1_IDX     0x1FB50818
#define TDMA_TX_RING0_DMA_IDX 0x1FB5080C
#define TDMA_INT_CFG0         0x1FB50A28
#define TDMA_INT_CFG1         0x1FB50A2C
#define TDMA_GLB_CFG          0x1FB50A04
#define TDMA_RX_CFG           0x1FB54710
#define TDMA_FC_CFG0          0x1FB521F0
#define TDMA_FC_CFG1          0x1FB521F4
#define TDMA_FC_CFG2          0x1FB52230
#define TDMA_WIFI_BUF_CFG     0x1FB50FE8
#define AN7552_FC_REG         0x1FB501BC
#define PPE_WIFI_BUF_INFO     0x1FB50FDC	/* wcid 14:0, info 20:16 */
#define PPE_WIFI_BUF_ID       0x1FB50FE0	/* 31 valid, 30 free only, 15:0 id */
#define PPE_WIFI_BUF_CNT      0x1FB50FE4
#ifdef AN7552
#define PPE_WIFI_BUF_WCID     0xFFFF
#else
#define PPE_WIFI_BUF_WCID     0x7FFF
#endif

/* TDMA rx ring geometry */
#define TDMA_RX_RINGS         2
#define TDMA_RX_RING_DESCS    1024
#define TDMA_RX_DESC_SIZE     32
#define TDMA_RX_RING_STRIDE   0x8000

#ifdef HAS_NPU_WIFI_TX
/* host -> NPU tx rings: 208-byte entries, bit0 of word0 is the own bit */
#define HOSTADPT_IN_ENTRY     208
#endif


#define REORDER_PRI_POOL_SIZE  2000
#define REORDER_SEC_POOL_SIZE  5000

/* ================================================================
 * npu_wifi_bufid.c
 * ================================================================ */

extern u32 counter_base_2g;
extern u32 counter_base_5g;
extern u32 counter_base_tri;

s32 buf_id_alloc_hw(u32 type, u32 band);
void buf_id_free(u32 type, u32 band, u32 buf_id);
void buf_id_return(u16 buf_id);
s32 buf_id_alloc_ring(void);
void tx_token_free(u16 token);
s32 tx_token_alloc(void);
void counter_init(u32 band);
void wcid_counter_init(u32 band);

/* ================================================================
 * npu_tdma.c
 * ================================================================ */

extern u32 tdma_tx_sw_idx[8];
extern u32 tdma_bme_dscp_base_addr;

#ifdef HAS_WIFI
void bridge_dma_copy(u32 channel, u32 src, u32 dst, u32 len);
int __attribute__((noinline)) tdma_tx_submit(u32 token, u32 pkt_len,
					     u32 buf_addr, u32 band);
#endif

/* ================================================================
 * npu_hostadpt.c
 * ================================================================ */

extern volatile u8 hostadpt_tx_ring_ready;
#ifdef HAS_NPU_WIFI_TX
extern u32 hostadpt_in_base[2];
extern u32 hostadpt_in_size[2];
extern u32 hostadpt_in_ridx[2];
#endif

#ifdef HAS_WIFI
int host_ring_submit(u32 buf_addr, u16 pkt_len, u32 band,
		     u16 wcid, u8 amsdu, u8 fwd_type,
		     u16 orig_len, u8 is_last, u32 info);
#endif

/* ================================================================
 * npu_wifi_fwd.c
 * ================================================================ */

extern u16 rxnode_widx_2g;
extern u16 rxnode_widx_5g;
extern u16 pinode_widx_2g;
extern u16 pinode_widx_5g;

extern u32 stats_bytes_2g[32];
extern u32 stats_pkts_2g[32];
extern u32 stats_bytes_5g[32];
extern u32 stats_pkts_5g[32];

int pkt_forward(u32 buf_id, u32 pkt_len, s16 wcid, u8 amsdu,
		u32 band, u8 fwd_type, u32 orig_len,
		int classify_result, u8 tunnel);
#ifdef WIFI_KITE
void pinode_drain(u32 band);
void rxnode_drain(u32 band);
#endif

/* ================================================================
 * npu_wifi_ba.c
 * ================================================================ */

void reorder_node_free(u16 node_idx, u8 node_type, u32 band);
u32 reorder_node_alloc(u32 band, u32 *pool_type, u16 *idx_out);
int pkt_enqueue_bridge(u32 buf_id, u32 pkt_len, u32 amsdu,
		       u32 band, u32 fwd_type);
void ba_flush_entry(u32 *entry);
void ba_indicate_le_seq(u32 *entry, u32 seq);
u32 ba_seq_scan(u32 *entry, u32 seq);
u32 ba_state_update(u32 sn, u32 check_type, u32 entry_addr);
u32 ba_scan_entries(u32 band);
void ba_flush_all(u32 band);

/* ================================================================
 * npu_wifi_rx.c
 * ================================================================ */

void wifi_bridge_init(void);

/* The bridge state itself stays file-local. wifi_state_init() and
 * wifi_bridge_init() are its only writers and live beside it. */
extern u32 wifi_rx_ring_base_2g;
extern u32 wifi_tx_ring_base_5g;
extern u32 wifi_pipeline_base;

/* ================================================================
 * npu_wifi_init.c
 * ================================================================ */

void wifi_queue_mutex_init(void);
void wifi_pkt_queue_init(u32 band);
void wifi_ba_node_init(void);
void wifi_queue_mutex_init(void);
void wifi_pkt_queue_init(u32 band);
void wifi_ba_node_init(void);
void __attribute__((noinline)) wifi_npu_init(u32 dbdc);
void __attribute__((noinline)) npu_set_pcie_base(u32 addr, u32 band);
void __attribute__((noinline)) wifi_reset_ba_entry(u32 dir, u32 wcid);
void __attribute__((noinline)) npu_set_retry_limit(u32 val);
void __attribute__((noinline)) npu_set_pcie_port_type(u32 val);
void __attribute__((noinline)) npu_set_band_enable(u32 band);
void __attribute__((noinline)) npu_set_force_to_cpu(u8 val);
void __attribute__((noinline)) npu_set_flushall_timeout(u32 val);
void __attribute__((noinline)) npu_set_flushone_timeout(u32 val);
void __attribute__((noinline)) npu_set_no_ba_test(u8 val);
void __attribute__((noinline)) npu_set_fast_flag(u8 val);
void __attribute__((noinline)) npu_set_pkt_buf_addr(u32 val);
void __attribute__((noinline)) npu_set_dram_ba_node_addr(u32 val);
void __attribute__((noinline)) npu_set_driver_model(u32 val);
void __attribute__((noinline)) npu_set_band0_on_cpu(u32 val);
void __attribute__((noinline)) npu_set_bar_info(u32 band, u32 packed);
void __attribute__((noinline)) npu_set_ba_entry(u32 band, u32 packed);
void __attribute__((noinline)) npu_set_wait_state(u32 port, u8 state);
void __attribute__((noinline)) npu_set_rxd_init(u32 ring_size, u32 band);

/* ================================================================
 * Eagle: npu_wifi_eagle.c and npu_wifi_eagle_dp.c
 * ================================================================ */

#ifdef WIFI_EAGLE

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
#define EAGLE_RX_DESC_CTRL      0x07000100
#define EAGLE_PKT_HEADROOM      192
#define EAGLE_PKT_BUF_SHIFT     11

#define EAGLE_Q_ENTRY		12	/* packet queue entry */
#define EAGLE_TXQ_ENTRIES	512
#define EAGLE_MSEG_ENTRIES	128
#define EAGLE_MSEG_MAX		7	/* segments one frame may span */
#define EAGLE_STAGE_ENTRY	16
#define EAGLE_STAGE_ENTRIES	512
#define EAGLE_TXD_SLOT		256
#define EAGLE_TXD_BYTES		76
#define EAGLE_TX_BUF_BYTES	2048
#define EAGLE_TX_RING_MASK	0x7FF
#define EAGLE_TX_RING_ROOM	5	/* keep this many slots free */
#define EAGLE_TX_RING_ENTRIES	2048
#define EAGLE_REFILL_BUDGET	1536
#define EAGLE_HOSTADPT_BUDGET	256
#define EAGLE_TX_DESC_CTRL	0x4C4048

/* Where frames are supposed to appear. One line every two seconds from
 * core 3 says which stage they stop at. */
struct eagle_dbg {
	u32 in[2];	/* host tx entries taken off the in ring */
	u32 stage[2];	/* frames staged for the WiFi tx ring */
	u32 nostage[2];	/* staging refused: ring full or no token */
	u32 push[2];	/* descriptors handed to the WiFi tx ring */
	u32 rxd;	/* rxdmad descriptors parsed */
	u32 rxq;	/* packets queued for the host */
	u32 rxfast;	/* packets sent straight to the wired side */
	u32 rxdrop;	/* packets dropped: queue full or enqueue refused */
	u32 rxseg;	/* segments of a frame spanning several buffers */
	u32 rxout;	/* packets handed to the host adaptor */
	u32 rxbig;	/* frames carrying a payload, sampled */
	u32 rxoutfail;	/* host adaptor ring full */
	u32 txdone;	/* tx done reports consumed */
	u32 refill[2];	/* rx ring descriptors refilled */
	u32 lan[2];	/* TDMA rx frames pushed to the WiFi tx ring */
	u32 lanfail;	/* TDMA rx frames dropped: no token or ring stuck */
	u32 lanwait;	/* waits for a free WiFi tx slot */
};
extern struct eagle_dbg dbg;

u32 eagle_ring_desc_base(u32 ring_id);
void eagle_queue_init(u32 band);
void eagle_tdma_flow_ctrl(int on);
void ppe_wifi_bufid_isr(int src);
void eagle_msdu_pg_pool_init(void);

#endif /* WIFI_EAGLE */

#endif /* NPU_WIFI_H */
