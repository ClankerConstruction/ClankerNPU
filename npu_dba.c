/*
 * AN75XX NPU firmware - GPON dynamic bandwidth allocation
 *
 * AN7583 with a WiFi chip only. Core 5 runs the allocation loop against
 * the FTTR block at 0x1FBE4000.
 */

#include "npu_internal.h"


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
