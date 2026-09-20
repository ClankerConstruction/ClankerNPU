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
/* WiFi funcType dispatch (callback[0] — MFUNC_WIFI)
 *
 * Blob dispatches all four funcTypes via function pointer tables in .data:
 *   SET_WAIT(1): set_wait_func_table[31] indexed by funcId (up to 30)
 *   SET_NO_WAIT(2): single fn ptr, only funcId=0
 *   GET_WAIT(3): get_wait_func_table[10] indexed by funcId (up to 9)
 *   GET_NO_WAIT(4): single fn ptr, only funcId=0
 *
 * All handlers receive the DMA-translated msg pointer. */
int wifi_mail_dispatch(u32 base, u32 cnt)
{
	u32 *msg = (u32 *)((base & 0x3FFFFFFF) | NPU_ADDR_MASK);
	u32 func_type = (msg[0] >> 4) & 0xF;
	u32 func_id;

#ifdef NPU_MAIL_TRACE
	if (func_type != 3 || msg[1] != 0)
		npu_printf("[MAIL]t%d f%d i%d %x %x %x\n", func_type, msg[1],
			   msg[0] & 0xF, msg[2], msg[3], msg[4]);
#endif

	switch (func_type) {
	case 1: /* SET_WAIT */
		return wifi_mail_set_wait(base, cnt);
	case 2: /* SET_NO_WAIT */
#ifdef WIFI_KITE
		return wifi_mail_set_event(base, cnt);
#elif defined(WIFI_EAGLE)
		return eagle_mail_set_event(base, cnt);
#else
		return 0;
#endif
	case 3: /* GET_WAIT */
		func_id = msg[1];
		if (func_id > 9) {
			npu_printf("Error: exceed max num!interfaceid=%u "
				   "wifi_mail_data->funcType=%u "
				   "wifi_mail_data->funcId=%u\n",
				   msg[0] & 0xF, func_type, func_id);
			return 1;
		}
		if (get_wait_func_table[func_id])
			return get_wait_func_table[func_id](msg);
		return 1;
	case 4: /* GET_NO_WAIT */
		if (msg[1] != 0) {
			npu_printf("Error: exceed max num!interfaceid=%u "
				   "wifi_mail_data->funcType=%u "
				   "wifi_mail_data->funcId=%u\n",
				   msg[0] & 0xF, func_type, msg[1]);
		}
		return 1;
	default:
		npu_printf("not support unknow funcType\n");
		return 1;
	}
}

/* WiFi mail set_wait handler: dispatches sub-commands from host */
int __attribute__((noinline)) wifi_mail_set_wait(u32 base, u32 cnt)
{
	u32 *msg = (u32 *)((base & 0x3FFFFFFF) | NPU_ADDR_MASK);
	u32 func_id = msg[1];

	(void)cnt;
	if (func_id > 30) {
		npu_printf("Error: exceed max num!interfaceid=%u "
			   "wifi_mail_data->funcType=%u "
			   "wifi_mail_data->funcId=%u\n",
			   msg[0] & 0xF, 1, func_id);
		return 1;
	}
	if (set_wait_func_table[func_id])
		return set_wait_func_table[func_id](msg);
	return 1;
}

/* WiFi mail set_event handler */
int __attribute__((noinline)) wifi_mail_set_event(u32 base, u32 cnt)
{
	u32 *msg = (u32 *)((base & 0x3FFFFFFF) | NPU_ADDR_MASK);
	u32 cmd = msg[1];

	switch (cmd) {
	case 0:
		wifi_npu_init(msg[0] & 0xF);
		break;
	default:
		npu_printf("set_event: unknown cmd %d\n", cmd);
		break;
	}
	return 1;
}
#endif /* HAS_WIFI */

/* Core0 WiFi init wrapper */
void core0_wifi_init_wrapper(void)
{
#ifdef HAS_WIFI
	int result;

	/* after tdma_init: it wipes SRAM and restarts the bump allocator */
	wifi_pcie_desc_alloc();

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

#ifdef HAS_BME
	tdma_tx_init();
	tdma_rx_init();
#endif
#ifndef WIFI_EAGLE
	wifi_bridge_init();
	npu_printf("%s finish\n", "core0_wifi_init_wrapper");
#endif

#ifdef WIFI_EAGLE
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
	} while (!(wifi_debug_flags & 2));
#else
	eagle_core3_loop();
#endif
	npu_printf("%s finish\n", "core3_wifi_init_wrapper");
#endif
}
