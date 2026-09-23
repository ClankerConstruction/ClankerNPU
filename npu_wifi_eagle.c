/*
 * AN75XX NPU firmware - eagle mailbox handlers and ring setup
 *
 * MT7991, MT7992 and MT7993. Same handler shape as kite, different
 * helper behind every command: interfaceID is a ring index, not a band,
 * and the rings are the RRO, MSDU page and indirect command rings.
 */

#include "npu_internal.h"
#include "npu_wifi.h"


#ifdef WIFI_EAGLE

/* ================================================================
 * WiFi mailbox command wrappers (eagle path)
 *
 * MT7991/MT7992/MT7993. Same wrapper shape as kite, one helper per
 * command, but every helper differs: the host carries a ring index in
 * interfaceID rather than a band, and the rings are the eagle RRO,
 * MSDU page and indirect-command rings.
 * ================================================================ */

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
		eagle_tx_ring_cpu_idx[0] = 0;
		eagle_tx_ring_cpu_idx[1] = 0;
		break;
	case 1:
		eagle_tx_ring_pcie_base[1] = addr;
		eagle_tx_ring_cpu_idx[0] = 0;
		eagle_tx_ring_cpu_idx[1] = 0;
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
 * block, by the ring's own id. */
u32 eagle_ring_desc_base(u32 ring_id)
{
	static const u32 off[6] = {
		0x00000, 0x140A0, 0x06020, 0x1A0C0, 0x0E020, 0x0E0A0
	};

	if (ring_id - 1 > 5)
		return 0;
	return wifi_pcie_desc_base + off[ring_id - 1];
}

/* Set up before the host starts handing over rings. */
void eagle_rx_init(void)
{
	u32 *ind;

	eagle_test_noba = 1;
	eagle_mseg_retry = 3;
	counter_init(2);
	counter_init(0);
	counter_init(1);
	eagle_txq_mutex[0] = 10;
	eagle_txq_mutex[1] = 0;
	eagle_rxdmad_on_core2 = 0;
	/* type 30 is reserved and never read; keeps the SRAM layout */
	sram_buf_alloc(30);
	eagle_rx_ring_init_done[1] = 0;
	eagle_rx_ring_init_done[0] = 0;
	/* indirect command state, 254 = idle */
	ind = (u32 *)sram_buf_alloc(25);
	ind[0] = 254;
	ind[4] = 254;
	ind[8] = 254;
	eagle_icv_err_table = sram_buf_alloc(22);
}

/* ---- mailbox: npu_mbox_set_wait_inode_txrx_reg_addr ---- */

static void eagle_rro_elem_reset(u32 table, u32 entries)
{
	u32 p;

	for (p = table; p != table + 8 * entries; p += 8)
		*(volatile u8 *)(p + 7) = 0xFF;
}

static void eagle_inode_txrx_reg(u32 id, u32 arg, u32 addr)
{
	u32 base, i, p;

	switch (id) {
	case 0:				/* rro address element table */
		base = (addr & 0x3FFFFFFF) | NPU_ADDR_MASK;
		if (arg < 128) {
			eagle_rro_addr_elem[arg] = base;
			npu_memset((void *)base, 0, 0x10000);
			eagle_rro_elem_reset(base, 0x2000);
		}
		break;
	case 1:				/* particular session table */
		base = (addr & 0x3FFFFFFF) | NPU_ADDR_MASK;
		eagle_session_tbl = base;
		eagle_rro_elem_reset(base, 0x400);
		break;
	case 2:				/* rro is up: run the rx path */
		eagle_tdma_flow_ctrl(1);
		eagle_rx_en = 1;
		eagle_tx_en = 1;
		eagle_rro_state = 3;
		eagle_rx_stopped = 0;
		if (eagle_icv_err_table != 0)
			npu_memset((void *)eagle_icv_err_table, 0, 4104);
		break;
	case 3:				/* re-arm one session's elements */
		base = (arg == 1) ? eagle_session_tbl :
				    eagle_rro_addr_elem[(arg >> 3) & 127];
		if (base == 0)
			break;
		for (i = 0; i < 1024; i++) {
			p = (arg == 1) ? base + 8 * i :
					 base + 8 * (((arg & 7) << 10) + i);
			*(volatile u8 *)(p + 7) = 0xFF;
		}
		break;
	case 4:				/* stop the rx path */
		eagle_stopping = 1;
		eagle_tdma_flow_ctrl(0);
		eagle_rx_en = 0;
		eagle_tx_en = 0;
		eagle_txq_state = 0;
		eagle_rro_state = 0;
		eagle_fastpath_en = 0;
		break;
	case 5:				/* emi cpu index address */
		eagle_emi_cidx = (addr & 0x3FFFFFFF) | NPU_ADDR_MASK;
		eagle_emi_cidx_valid = 1;
		break;
	case 6:				/* restart after a stop */
#ifdef HAS_BME
		tdma_tx_wait_idle();
		tdma_bmgr_reinit();
#endif
		eagle_msdu_pg_pool_init();
		counter_init(2);
		counter_init(0);
		counter_init(1);
		break;
	case 7:				/* tx path is up */
		eagle_stopping = 0;
		eagle_tx_en = 1;
		eagle_txq_state = 3;
		eagle_rro_state = 3;
		break;
	default:
		npu_printf("%s() case %d doesn't support \n",
			   "npu_mbox_set_wait_inode_txrx_reg_addr_wrapper", id);
		break;
	}
}

/* every 256-byte slot of the tx buffer space carries a 144-byte header */
static void eagle_tx_buf_space_clear(u32 base)
{
	u32 p, end = base + EAGLE_TX_BUF_SPACE_SIZE;

	for (p = base; p != end; p += EAGLE_TX_BUF_SLOT)
		npu_memset((void *)p, 0, 144);
}

/* tx staging ring: one 16-byte entry per 256-byte tx packet buffer. The
 * host adaptor drain fills an entry, the tx fast path moves it into the
 * WiFi tx ring. */
static void eagle_tx_stage_init(u32 band)
{
	u32 buf, desc, i;

	/* the host sets the tx packet buffer last; wait for it */
	while (npu_tx_pkt_buf_addr == 0)
		delay_ms(100);

	if (band != 0) {
		buf = eagle_stage_buf1;
		eagle_stage_widx[1] = 0;
		eagle_stage_ridx[1] = 0;
		eagle_stage_base[1] = sram_buf_alloc(17);
		desc = eagle_stage_base[1];
	} else {
		buf = eagle_stage_buf0;
		eagle_stage_widx[0] = 0;
		eagle_stage_ridx[0] = 0;
		eagle_stage_base[0] = sram_buf_alloc(16);
		desc = eagle_stage_base[0];
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
		eagle_txd_space[0] = base;
		break;
	case 1:
		eagle_txd_space[1] = base;
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
		eagle_stage_buf0 = base;
		eagle_tx_stage_init(0);
		break;
	case EAGLE_RING_TXDONE1:
		eagle_stage_buf1 = base;
		eagle_tx_stage_init(1);
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

	if (ring_size - 1 > EAGLE_RX_RING_MAX_IDX) {
		npu_printf("ERROR! rx_ring_size = %d\n", ring_size);
		ring_size = EAGLE_RX_RING_MAX_IDX + 1;
	}

	eagle_rx_ring_size[band] = (u16)ring_size;
	desc_base = eagle_rx_ring_desc_base[band];

	for (i = 0; i < ring_size; i++) {
		buf_id = (u32)buf_id_alloc_ring();
		if (buf_id == (u32)-1) {
			npu_printf("[%s]rx ring init: alloc buffid fail!\n",
				   band ? "BAND1" : "BAND0");
			return 1;
		}
		dbg.refill[band]++;
	eagle_rx_ring_bufid[band][i] = (u16)buf_id;
		desc = desc_base + 16 * i;
		REG32(desc) = ((((buf_id << 11) + eagle_pkt_buf_addr) &
				0x3FFFFFFF) | 0x80000000) + 192;
		REG32(desc + 8) = buf_id << 16;
		REG32(desc + 12) = 0;
		REG32(desc + 4) = EAGLE_RX_DESC_CTRL;
	}

	eagle_rx_ring_cpu_idx[band] = 0;
	eagle_rx_ring_init_done[band] = 1;
	return 0;
}

/* The WiFi chip stamps each descriptor it writes with a 4-bit
 * generation. Starting every descriptor on a generation the NPU never
 * expects is what makes a fresh ring read as empty. */
static void eagle_ind_cmd_ring_init(u32 ring_size, u32 wide)
{
	u32 i;

	if (ring_size - 1 > EAGLE_RX_RING_MAX_IDX) {
		npu_printf("ERROR! rx_ring_size = %d\n", ring_size);
		ring_size = EAGLE_RX_RING_MAX_IDX + 1;
	}
	for (i = 0; i < ring_size; i++) {
		if (wide)
			REG32(eagle_ind_cmd_desc_base + 16 * i + 12) |= 0xF0000000;
		else
			REG32(eagle_ind_cmd_desc_base + 8 * i + 4) |= 0xE0000000;
	}
	eagle_rxdmad_ridx = 0;
	eagle_rxdmad_gen = 0;
}

/* RRO rx ring 10: one 16-byte descriptor per buffer, same shape as the
 * rx rings but over the tx-done descriptor base */
static void eagle_txdone_ring_fill(u32 ring_size)
{
	u16 *ids;
	u32 i, desc;
	s32 buf_id;

	if (ring_size - 1 > 511)
		npu_printf("ERROR! %s() rx_ring_size = %d\n",
			   "npu_init_txdone_ring0_eagle", ring_size);

	eagle_txdone_ring_cnt = (u16)ring_size;
	eagle_txdone_id_base = sram_buf_alloc(23);
	ids = (u16 *)eagle_txdone_id_base;

	for (i = 0; i < ring_size; i++) {
		buf_id = buf_id_alloc_ring();
		if (buf_id == -1) {
			npu_printf("[BAND0] txdone init: alloc buffid fail!\n");
			return;
		}
		desc = eagle_rx_txdone_desc_base + 16 * i;
		REG32(desc + 4) = 0;
		REG32(desc) = (((((u32)buf_id << 11) + eagle_pkt_buf_addr) &
				0x3FFFFFFF) | 0x80000000) + 192;
		REG32(desc + 8) = 0;
		REG32(desc + 12) = 0;
		REG32(desc + 4) = EAGLE_RX_DESC_CTRL;
		ids[i] = (u16)buf_id;
	}
}

/* MSDU page ids: an 8-slot free ring */
static u16 *msdu_pg_pool;
static u16 msdu_pg_ridx, msdu_pg_widx;
static u16 msdu_pg_ids[8];
static u32 msdu_pg_ring_ridx;
static u32 msdu_pg_ring_ready;

void eagle_msdu_pg_pool_init(void)
{
	u32 i;

	msdu_pg_pool = (u16 *)sram_buf_alloc(26);
	for (i = 0; i < 8; i++)
		msdu_pg_pool[i] = (u16)i;
	msdu_pg_ridx = 0;
	msdu_pg_widx = 0;
}

static s32 msdu_pg_id_alloc(void)
{
	u16 next = (msdu_pg_ridx + 1 == 8) ? 0 : msdu_pg_ridx + 1;
	s32 id;

	if (msdu_pg_widx == next)
		return -1;
	id = (s16)msdu_pg_pool[msdu_pg_ridx];
	msdu_pg_ridx = next;
	return id;
}

/* npu_rro_msdu_pg_ring_init_eagle */
static int eagle_msdu_pg_ring_init(u32 ring_size, u32 again)
{
	u32 i, desc;
	s32 id;

	if (ring_size - 1 > 7)
		npu_printf("ERROR! rx_ring_size = %d\n", ring_size);

	for (i = 0; i < ring_size; i++) {
		id = msdu_pg_id_alloc();
		if (id == -1) {
			npu_printf("[%s]rx ring init: alloc buffid fail!\n",
				   "npu_rro_msdu_pg_ring_init_eagle");
			return 1;
		}
		msdu_pg_ids[i] = (u16)id;
		desc = eagle_msdu_pg_desc_base + 16 * i;
		REG32(desc + 4) = 0;
		REG32(desc) = ((((u32)id << 7) + eagle_dram_ba_node_addr) &
			       0x3FFFFFFF) | 0x80000000;
		REG32(desc + 8) = (u32)id << 16;
		REG32(desc + 12) = 0;
		REG32(desc + 4) = 0x00800100;
	}

	if (!again) {
		msdu_pg_ring_ridx = 0;
		msdu_pg_ring_ready = 1;
		npu_printf("reset msdu pg ring\n");
	}
	return 0;
}

static void npu_mbox_init_rxd_wrapper(u32 ring_size, u32 ring)
{
#ifdef NPU_MAIL_TRACE
	npu_printf("[NPU]rxd init ring=%d size=%d\n", ring, ring_size);
#endif
	switch (ring) {
	case EAGLE_RING_RX0:
		eagle_queue_init(0);
		eagle_rx_ring_init(ring_size, 0);
		break;
	case EAGLE_RING_RX1:
		eagle_queue_init(1);
		eagle_rx_ring_init(ring_size, 1);
		break;
	case EAGLE_RING_MSDU_PG0:
	case EAGLE_RING_MSDU_PG1:
		if (ring == EAGLE_RING_MSDU_PG1)
			npu_printf("%s() wrong input val:%d\n",
				   "npu_rro_msdu_pg_ring_desc_addr", 1);
		else
			eagle_msdu_pg_desc_base = eagle_ring_desc_base(5);
		eagle_msdu_pg_ring_init(ring_size, ring == EAGLE_RING_MSDU_PG1);
		break;
	case EAGLE_RING_IND_CMD0:
		eagle_ind_cmd_desc_base = eagle_ring_desc_base(6);
		eagle_ind_cmd_ring_init(ring_size, 0);
		break;
	case EAGLE_RING_IND_CMD1:
		eagle_ind_cmd_desc_base = eagle_ring_desc_base(6);
		eagle_ind_cmd_ring_init(ring_size, 1);
		break;
	case EAGLE_RING_TXDONE0:
		eagle_txdone_ring_fill(ring_size);
#ifdef HAS_BME
		/* AN7581 has no TDMA rx ring to reclaim buffers from */
		np_skb_tx_force_reset();
#endif
		eagle_txdone_ridx = 0;
		eagle_fastpath_en = 1;
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

/* on eagle this is the MSDU page pool, 128 bytes per page */
int eagle_mail_set_dram_ba_node(u32 *msg)
{
	eagle_addr_in_range(msg[2],
			    "npu_mbox_set_wait_dram_ba_reordering_node_addr_wrapper");
	eagle_dram_ba_node_addr = msg[2];
	npu_printf("got MSDU PG packet pool address:%x\n", msg[2]);
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

/* Open the PCIe port's window onto the descriptor block. Until this runs
 * the WiFi chip cannot fetch a tx ring descriptor or write an rx one, so
 * its dma index never moves however many descriptors the NPU queues.
 *
 * The window is a base and an end. Port type 0 and 1 give one port the
 * whole block; 2 and 3 split it, band 0 taking everything up to rx ring
 * 1 and band 1 the rest. */
static void eagle_pcie_window_publish(u32 band)
{
	u32 rx1 = eagle_ring_desc_base(2);
	u32 rx1_end = rx1 + 0xE020;
	u32 rx0 = eagle_ring_desc_base(1);
	u32 rx0_end = rx0 + 0x140A0;

	switch (eagle_pcie_port_type) {
	case 0:
		REG32(PCIE1_WIN_BASE) = rx0 & 0x1FFFFFFF;
		REG32(PCIE1_WIN_END) = rx1_end & 0x1FFFFFFF;
		break;
	case 1:
		REG32(PCIE0_WIN_BASE) = rx0 & 0x1FFFFFFF;
		REG32(PCIE0_WIN_END) = rx1_end & 0x1FFFFFFF;
		break;
	case 2:
		if (band != 0) {
			REG32(PCIE0_WIN_BASE) = rx1 & 0x1FFFFFFF;
			REG32(PCIE0_WIN_END) = rx1_end & 0x1FFFFFFF;
		} else {
			REG32(PCIE1_WIN_BASE) = rx0 & 0x1FFFFFFF;
			REG32(PCIE1_WIN_END) = rx0_end & 0x1FFFFFFF;
		}
		break;
	case 3:
		if (band != 0) {
			REG32(PCIE1_WIN_BASE) = rx1 & 0x1FFFFFFF;
			REG32(PCIE1_WIN_END) = rx1_end & 0x1FFFFFFF;
		} else {
			REG32(PCIE0_WIN_BASE) = rx0 & 0x1FFFFFFF;
			REG32(PCIE0_WIN_END) = rx0_end & 0x1FFFFFFF;
		}
		break;
	default:
		break;
	}
}

int eagle_mail_set_port_type(u32 *msg)
{
	eagle_pcie_port_type = (u8)msg[2];
	eagle_pcie_window_publish(0);
	eagle_pcie_window_publish(1);
	npu_printf("[NPU]pcie port type=%d win0=%x/%x win1=%x/%x\n",
		   eagle_pcie_port_type, REG32(PCIE0_WIN_BASE),
		   REG32(PCIE0_WIN_END), REG32(PCIE1_WIN_BASE),
		   REG32(PCIE1_WIN_END));
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
	npu_tx_pkt_buf_addr = msg[2];
	return 1;
}

int eagle_mail_set_txrx_reg(u32 *msg)
{
	eagle_inode_txrx_reg(msg[0] & 0xF, msg[2], msg[3]);
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

/* The host reads the NPU's own tx ring cursor from here, and polls id 3
 * to see whether the rx path has come to rest. */
int eagle_mail_get_npu_info(u32 *msg)
{
	u32 id = msg[0] & 0xF;

	switch (id) {
	case 0:
		msg[2] = eagle_tx_ring_cpu_idx[0];
		break;
	case 1:
	case 2:
		msg[2] = eagle_tx_ring_cpu_idx[1];
		break;
	case 3:
		msg[2] = (eagle_rx_stopped == 0) | eagle_rx_busy;
		break;
	default:
		npu_printf("[ERROR]%s() wrong input value %d\n",
			   "npu_mbox_get_wait_npu_info_wrapper", id);
		msg[2] = 0;
		break;
	}
	return 1;
}

int eagle_mail_get_last_rate(u32 *msg)
{
	msg[2] = 222;
	msg[3] = 3333;
	return 1;
}

/* 16 per-entry counter slots, none kept on eagle: zero the block,
 * one complaint per slot */
int eagle_mail_get_counter(u32 *msg)
{
	u32 i;

	for (i = 0; i < 16; i++)
		npu_printf("[ERROR]%s() not support in NPU \n",
			   "npu_mbox_get_wait_counter_wrapper");
	npu_memset((u8 *)msg + 16, 0, 576);
	return 1;
}

/* where the band's counter block sits in SRAM */
int eagle_mail_get_dbg_counter(u32 *msg)
{
	u32 band = msg[0] & 0xF;

	msg[2] = sram_buf_alloc(band == 0 ? 10 : band == 1 ? 9 : 11) &
		 0x1FFFFFFF;
	return 1;
}

/* Answers with a physical address: the host points the WiFi hardware at
 * it. Asking for a tx ring (5, 6) also arms that ring - every descriptor
 * starts owned by the NPU. */
int eagle_mail_get_rxdesc_base(u32 *msg)
{
	u32 ring = msg[0] & 0xF;
	u32 base, i;

	switch (ring) {
	case EAGLE_RING_RX0:
	case EAGLE_RING_RX1:
		msg[2] = eagle_rx_ring_desc_base[ring] & 0x1FFFFFFF;
		break;
	case 5:
	case 6:
		base = eagle_tx_ring_desc[ring - 5];
		if (base == 0) {
			npu_printf("[NPU][ERROR] wrong tx ring desc pase !!  band_idx=%d \n",
				   ring - 5);
			msg[2] = 0;
			break;
		}
		for (i = 0; i < EAGLE_TX_RING_ENTRIES; i++)
			REG32(base + 16 * i + 4) = 0x80000000;
		msg[2] = base & 0x1FFFFFFF;
		break;
	case EAGLE_RING_IND_CMD0:
	case EAGLE_RING_IND_CMD1:
		msg[2] = eagle_ind_cmd_desc_base & 0x1FFFFFFF;
		break;

	case EAGLE_RING_TXDONE0:
		msg[2] = eagle_msdu_pg_desc_base & 0x1FFFFFFF;
		break;
	default:
		npu_printf("[ERROR]%s() wrong input value %d\n",
			   "npu_mbox_get_wait_rxdesc_base_wrapper", ring);
		msg[2] = 0;
		break;
	}
#ifdef NPU_MAIL_TRACE
	npu_printf("[NPU]rxdesc base ring=%d -> %x\n", ring, msg[2]);
#endif
	return 1;
}

int eagle_mail_get_wcid_dbg_counter(u32 *msg)
{
	npu_printf("%s() This is not support !!!\n",
		   "npu_mbox_get_wait_wcid_dbg_counter_wrapper");
	msg[2] = 0;
	return 1;
}

int eagle_mail_get_dma_addr(u32 *msg)
{
	u32 dir = msg[2];

	npu_printf("%s L%d not support on bellwether\n",
		   "npu_mbox_txrx_ring_dma_addr_get_wrapper", 10561);
	msg[2] = 0;
	npu_printf("get dma addr. band=%d dir=%d addr=%x\n",
		   msg[0] & 0xF, dir, 0);
	return 1;
}

int eagle_mail_get_ring_size(u32 *msg)
{
	npu_printf("%s L%d not support on bellwether\n",
		   "npu_mbox_txrx_ring_ring_size_get_wrapper", 10555);
	msg[2] = 0;
	npu_printf("%s get wait size =%d\n", "wifi_mail_get_wait_ring_size", 0);
	return 1;
}

int eagle_mail_get_mdc_lock(u32 *msg)
{
	(void)msg;
	return 1;
}

int eagle_mail_get_dump_mapping(u32 *msg)
{
	sram_buf_dump();
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
