/*
 * AN75XX NPU firmware - kite mailbox handlers
 *
 * MT7916 and MT7996. One handler per funcId, indexed by the tables in
 * npu_globals.c. interfaceID in the message header is a band here.
 */

#include "npu_internal.h"
#include "npu_wifi.h"


/* Eagle uses the same wrapper shape but a different helper behind every
 * command: funcId 0 reaches npu_set_pcie_base_eagle rather than
 * npu_set_pcie_base. See npu_wifi_eagle.c. */

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
		return rxd_base_5g & 0x1FFFFFFF;
	return rxd_base_2g & 0x1FFFFFFF;
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

	npu_printf("set band:%d debugflag=%d\n", band, flag);
	npu_mbox_set_debug_flag(band, flag);
	return 1;
}

int wifi_mail_set_wait_inode_cfg(u32 *msg)
{
	u32 a = ((u8 *)msg)[8];
	u32 b = ((u8 *)msg)[9];
	u32 band = msg[0] & 0xF;

	npu_printf("%s L%d set band %d epmask=%d, vap_mask=%d\n",
		   "wifi_mail_set_wait_inode_cfg_info", 293, band, a, b);
	npu_mbox_rx_hw_cfg_set(band, a, b);
	return 1;
}

int wifi_mail_set_wait_inode_stop(u32 *msg)
{
	u32 band = msg[0] & 0xF;

	npu_printf("%s L%d set. band:%d\n",
		   "wifi_mail_set_wait_inode_stop_action", 306, band);
	npu_mbox_stop_set(band);
	return 1;
}

int wifi_mail_set_pcie_swap(u32 *msg)
{
	u32 val = msg[2];

	npu_printf("%s L%d set %d\n",
		   "wifi_mail_set_wait_inode_pcie_swap", 322, val);
	npu_mbox_pcie_swap_set(val);
	return 1;
}

/* SET_WAIT handlers [0]-[23], [29]-[30] — indexed by SDK enum */

int wifi_mail_set_pcie_addr(u32 *msg)
{
	npu_set_pcie_base(msg[2], msg[0] & 0xF);
	return 1;
}

/* the band's npu_init, then its rx ring */
int wifi_mail_set_desc(u32 *msg)
{
	u32 band = msg[0] & 0xF;

	wifi_npu_init(band);
	npu_set_rxd_init(msg[2], band);
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

/* one chip info word per SerDes port, then the PHY tx gpio */
#ifdef AN7552
#define ARHT_PORTS	1
#else
#define ARHT_PORTS	6
#endif

int wifi_mail_set_arht_chip_info(u32 *msg)
{
	u32 i;

	arht_phy_tx_gpio = msg[2 + ARHT_PORTS + 1];
	if (arht_phy_tx_gpio == 0xFFFFFFFF) {
		npu_printf("%s get phy tx gpio error\n",
			   "wifi_mail_set_wait_arht_chip_info");
		return 0;
	}
	for (i = 0; i < ARHT_PORTS; i++)
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
