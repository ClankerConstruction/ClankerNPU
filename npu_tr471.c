/*
 * AN75XX NPU firmware - TR-471 test infrastructure
 *
 * AN7581 with a WiFi chip only. ITU-T Y.1540 latency and loss
 * measurement. Core 7 calls the init from the tunnel offload loop.
 */

#include "npu_internal.h"


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
