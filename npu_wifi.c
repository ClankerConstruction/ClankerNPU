/*
 * AN75XX NPU firmware - WiFi offload entry points
 *
 * The mailbox dispatch the host reaches, the two init wrappers core 0
 * and core 3 call, and the debug counter ISR. The work itself is in the
 * npu_wifi_*.c files beside this one.
 */

#include "npu_internal.h"
#include "npu_wifi.h"


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

#ifdef HAS_WIFI
static int wifi_mail_exceed(u32 *msg, u32 type)
{
	npu_printf("Error: exceed max num!interfaceid=%u wifi_mail_data->funcType=%u wifi_mail_data->funcId=%u\n",
		   msg[0] & 0xF, type, msg[1]);
	return 0;
}

static int wifi_mail_call(wifi_mail_fn_t fn, u32 *msg, const char *op)
{
	int ret;

	if (!fn)
		return 1;
	ret = fn(msg);
	if (!(u16)ret)
		npu_printf("wifi_mail_%s_operation fail !\n", op);
	return (u16)ret;
}

/* MFUNC_WIFI: msg[0] bits 7:4 funcType, bits 3:0 interface; msg[1] funcId */
int wifi_mail_dispatch(u32 base, u32 cnt)
{
	u32 *msg = (u32 *)((base & 0x3FFFFFFF) | NPU_ADDR_MASK);
	u32 func_type = (msg[0] >> 4) & 0xF;

	(void)cnt;
#ifdef NPU_MAIL_TRACE
	if (func_type != 3 || msg[1] != 0)
		npu_printf("[MAIL]t%d f%d i%d %x %x %x\n", func_type, msg[1],
			   msg[0] & 0xF, msg[2], msg[3], msg[4]);
#endif
	switch (func_type) {
	case 1:
		if (msg[1] > 30)
			return wifi_mail_exceed(msg, 1);
		return wifi_mail_call(set_wait_func_table[msg[1]], msg, "set_wait");
	case 2:
		/* no set_nowait handler is installed */
		if (msg[1])
			return wifi_mail_exceed(msg, 2);
		return wifi_mail_call(NULL, msg, "set_nowait");
	case 3:
		if (msg[1] > 9)
			return wifi_mail_exceed(msg, 3);
		return wifi_mail_call(get_wait_func_table[msg[1]], msg, "get_wait");
	case 4:
		if (msg[1])
			return wifi_mail_exceed(msg, 4);
		return wifi_mail_call(NULL, msg, "get_nowait");
	default:
		npu_printf("not support unknow funcType\n");
		return 1;
	}
}
#endif /* HAS_WIFI */

/* Core0 WiFi init wrapper */
void core0_wifi_init_wrapper(void)
{
#ifdef HAS_WIFI
	int result;

#ifdef WIFI_KITE
	/* no TDMA rx ring on kite */
#ifdef HAS_BME
	tdma_tx_init();
#endif
	wifi_bridge_init();
	npu_printf("%s finish\n", "core0_wifi_init_wrapper");
#else
	/* after tdma_init: it wipes SRAM and restarts the bump allocator */
	wifi_pcie_desc_alloc();
#endif

#ifdef WIFI_EAGLE
	eagle_rx_init();
	eagle_tx_ring_desc[0] = eagle_ring_desc_base(3);
	eagle_tx_ring_desc[1] = eagle_ring_desc_base(4);
	if (eagle_tx_ring_desc[0] == 0)
		npu_printf("%s:[ERROR]!!!! can't get tx ring0 desc base:%x\n",
			   "npu_offload_wifi_tx_ring_init", 0);
	if (eagle_tx_ring_desc[1] == 0)
		npu_printf("%s:[ERROR]!!!! can't get tx ring1 desc base:%x\n",
			   "npu_offload_wifi_tx_ring_init", 0);
#endif

#ifdef WIFI_EAGLE
	eagle_msdu_pg_pool_init();
#ifdef HAS_BME
	tdma_tx_init();
	tdma_rx_init();
#endif
	eagle_init_done = 1;
	npu_printf("NPU init Version: %s\n", NPU_INIT_VERSION);
#endif
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
	} while (!(*(volatile u8 *)&wifi_debug_flags & 2));
#else
	eagle_core3_loop();
#endif
	npu_printf("%s finish\n", "core3_wifi_init_wrapper");
#endif
}
