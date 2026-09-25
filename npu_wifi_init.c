/*
 * AN75XX NPU firmware - WiFi init and mailbox setters
 *
 * Ring and table setup, plus one setter per host parameter. The kite
 * mailbox handlers in npu_wifi_kite.c call the setters.
 */

#include "npu_internal.h"
#include "npu_wifi.h"


/* ================================================================
 * WiFi RXD/TXD initialization
 * ================================================================ */

void __attribute__((noinline)) npu_set_pcie_base(u32 addr, u32 band)
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

/* point one rx descriptor at a fresh buffer id */
static void wifi_rxd_set_buf(u32 desc, u32 buf_id)
{
	*(u32 *)desc = (((buf_id << 12) + wifi_pkt_buf_addr) & 0x3FFFFFFF) |
		       0x80000000;
	*(u16 *)(desc + 6) = (*(u16 *)(desc + 6) & 0x4000) | 0xDAC;
}

/* 5G rx ring */
static int wifi_init_rxd_5g(u32 ring_size, u32 band)
{
	u32 i;
	s32 id;

	npu_printf("[NPU] Enter %s, band_idx=%d, rx_ring_size=%lu\n",
		   "npu_npu_init_rxd_5G", band, ring_size);

	if (ring_size - 1 > 0x5FF)
		npu_printf("ERROR! rx_ring_size = %lu \n", ring_size);

	if (rxd_5g_init_done != 0) {
		npu_printf("5G init second: do np_skb free\n");
		rxd_5g_init_done = 0;
		for (i = 0; i < ring_size; i++)
			buf_id_free(0, band, rxd_5g_bufid_table[i]);
	}

	for (i = 0; i < ring_size; i++) {
		id = buf_id_alloc_hw(0, band);
		if (id == -1) {
			npu_printf("[%s] rxd init: alloc buffid fail!\n",
				   (band != 0) ? "5G" : "2.4G");
			return 1;
		}
		rxd_5g_bufid_table[i] = (u16)id;
		wifi_rxd_set_buf(rxd_base_5g + i * 16, id);
	}

	rxd_5g_cpu_idx = 0;
	rxd_5g_flush_tick = KITE_TICK;
	rxd_5g_init_done = 1;
	return 0;
}

/* 2.4G rx ring */
static int wifi_init_rxd_2g(u32 ring_size, u32 band)
{
	u16 *tbl;
	u32 i;
	s32 id;

	npu_printf("[NPU] Enter(2.4G) %s, band_idx=%d, rx_ring_size=%lu\n",
		   "npu_npu_init_rxd", band, ring_size);

	if (ring_size - 1 > 0x5FF)
		npu_printf("ERROR! rx_ring_size = %lu\n", ring_size);

	tbl = (u16 *)rxd_2g_bufid_base;
	if (tbl != NULL && rxd_2g_init_done != 0) {
		rxd_2g_init_done = 0;
		for (i = 0; i < ring_size; i++)
			buf_id_free(0, band, tbl[i]);
	}

	for (i = 0; i < ring_size; i++) {
		id = buf_id_alloc_hw(0, band);
		if (id == -1) {
			npu_printf("!!!!!!!!!!!! [%s] rxd init: alloc buffid fail!\n",
				   (band != 0) ? "5G" : "2.4G");
			return 1;
		}
		tbl[i] = (u16)id;
		wifi_rxd_set_buf(rxd_base_2g + i * 16, id);
	}

	rxd_2g_cpu_idx = 0;
	rxd_2g_flush_tick = KITE_TICK;
	rxd_2g_init_done = 1;
	return 0;
}

void __attribute__((noinline)) wifi_reset_ba_entry(u32 dir, u32 wcid)
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

/* WiFi queue mutex init: assigns HW mutex indices to queue/BA mutexes */
void wifi_queue_mutex_init(void)
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

/* WiFi BA node init: allocate reorder node pools */
void wifi_ba_node_init(void)
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
	for (i = 0; pool != NULL && i < REORDER_PRI_POOL_SIZE; i++)
		pool[i] = (u16)i;
	reorder_pri_widx = 0;
	reorder_pri_ridx = 0;

	pool = (u16 *)sram_buf_alloc(13);
	reorder_sec_idx_pool = (u32)pool;
	for (i = 0; pool != NULL && i < REORDER_SEC_POOL_SIZE; i++)
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

/* WiFi offload on this package? */
static u8 wifi_get_band_cap(u8 model)
{
	u32 chip_rev = REG32(CHIP_ID_REG) >> 16;
	u32 variant = (REG32(CHIP_VARIANT_REG) & 0xF) |
		      ((REG32(CHIP_VARIANT_REG) >> 3) & 0x10);

	if (model != 0) {
		if (chip_rev == 15 && variant == 1) {
			npu_printf("Chip id(%x) does not support "
				   "NPU Wifi Offload!!!\n", variant);
			return 0;
		}
		npu_printf("Support NPU Wifi Offload\n");
		return 1;
	}

	/* model 0: only these chip 12 packages */
	if (chip_rev == 12 && (variant == 0 || variant == 1 ||
			       variant == 11 || variant == 3 ||
			       variant == 4 || variant == 12)) {
		npu_printf("Chip id(%x) support NPU Wifi ffload\n", variant);
		return 1;
	}
	npu_printf("Chip id(%x) does not support NPU Wifi ffload!!!\n",
		   variant);
	return 0;
}

/* npu_init for one band, from SET_WAIT_DESC */
void __attribute__((noinline)) wifi_npu_init(u32 band)
{
#ifdef WIFI_KITE
	u32 desc_type1, desc_type2;
	u32 bar_5g, bar_2g;

	npu_printf("[NPU] %s...\n", "npu_init");

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
		if (band != 0)
			goto alloc_5g;
		goto alloc_2g;
	case 2:
		if (band != 0) {
			REG32(0x1FA90038) = desc_type1 & 0x1FFFFFFF;
			REG32(0x1FA9003C) = bar_2g & 0x1FFFFFFF;
			goto alloc_5g;
		}
		REG32(PCIE0_MAC_BASE + 0x8030) = desc_type2 & 0x1FFFFFFF;
		REG32(PCIE0_MAC_BASE + 0x8034) = bar_5g & 0x1FFFFFFF;
		goto alloc_2g;
	case 3:
		if (band == 0) {
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
	if (band != 0)
		goto alloc_5g;

alloc_2g:
	ba_table_a = sram_buf_alloc(8);
	rxd_2g_bufid_base = sram_buf_alloc(6);
	rxd_base_2g = wifi_pcie_desc_offset(wifi_pcie_desc_base, 2);
	goto pipeline_init;

alloc_5g:
	ba_table_b = sram_buf_alloc(7);
	rxd_base_5g = wifi_pcie_desc_offset(wifi_pcie_desc_base, 1);

pipeline_init:
	wifi_pkt_queue_init(band);
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

	counter_init(band);
	wcid_counter_init(band);
#endif
}

/* WiFi mailbox setters: simple parameter setters called from host.
 * noinline: each stays a separate callee, not inlined into handlers. */
void __attribute__((noinline)) npu_set_retry_limit(u32 val)
{
	wifi_retry_limit = (u16)val;
	npu_printf("enq_error_retry_times = %d !!!\n", val);
}

void __attribute__((noinline)) npu_set_pcie_port_type(u32 val)
{
	wifi_pcie_port_type = (u8)val;
	npu_printf("PCIe_Port_Type = %d !!!\n", val);
}

void __attribute__((noinline)) npu_set_band_enable(u32 band)
{
	if (band > 1) {
		npu_printf("[ERROR] band_idx is wrong value %d !!!\n", band);
		return;
	}
	wifi_pcie_state[band] = 1;
}

void __attribute__((noinline)) npu_set_force_to_cpu(u8 val)
{
	wifi_force_to_cpu = val;
	npu_printf("isForceToCpu=%s\n", val ? "true" : "false");
	if (wifi_force_to_cpu > 1)
		npu_printf("[ERROR] isForceToCpu is wrong value !!!\n");
}

void __attribute__((noinline)) npu_set_flushall_timeout(u32 val)
{
	wifi_flushall_timeout = (u16)val;
	npu_printf("flushall_timeout=%d\n", val);
}

void __attribute__((noinline)) npu_set_flushone_timeout(u32 val)
{
	wifi_flushone_timeout = (u16)val;
	npu_printf("flushone_timeout=%d\n", val);
}

void __attribute__((noinline)) npu_set_no_ba_test(u8 val)
{
	wifi_no_ba_test = val;
	npu_printf("isforTestNoBA=%s\n", val ? "true" : "false");
	if (wifi_no_ba_test > 1)
		npu_printf("[ERROR] isforTestNoBA is wrong value !!!\n");
}

void __attribute__((noinline)) npu_set_fast_flag(u8 val)
{
	wifi_debug_flags = val;
	npu_printf("npu_wifi_fast_flag = %d !!!\n", val);
}

void __attribute__((noinline)) npu_set_pkt_buf_addr(u32 val)
{
	wifi_pkt_buf_addr = val;
	npu_printf("pkt_buf_addr=%lx\n", val);
}

void __attribute__((noinline)) npu_set_dram_ba_node_addr(u32 val)
{
	wifi_dram_ba_node_addr = (val & 0x3FFFFFFF) | NPU_ADDR_MASK;
	npu_printf("dramBaNodeAddr=%x\n", val);
}

void __attribute__((noinline)) npu_set_driver_model(u32 val)
{
	wifi_driver_model = (u8)val;
	npu_printf("driverModel=%d\n", val);
}

void __attribute__((noinline)) npu_set_band0_on_cpu(u32 val)
{
	wifi_band0_on_cpu = (u8)val;
	npu_printf("npu_band0_on_cpu_support=%s\n",
		   val ? "true" : "false");
}

void __attribute__((noinline)) npu_set_bar_info(u32 band, u32 packed)
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

void __attribute__((noinline)) npu_set_ba_entry(u32 band, u32 packed)
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

void __attribute__((noinline)) npu_set_wait_state(u32 port, u8 state)
{
	u32 i;

	/* the port's flag and its band's */
	if (port <= 15) {
		wifi_wait_state_2g[port] = state;
		wifi_wait_band_2g = state;
	} else {
		wifi_wait_state_5g[port - 16] = state;
		wifi_wait_band_5g = state;
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

void __attribute__((noinline)) npu_set_rxd_init(u32 ring_size, u32 band)
{
	if (band == 1)
		wifi_init_rxd_5g(ring_size, 1);
	else
		wifi_init_rxd_2g(ring_size, band);
}

