/*
 * AN75XX NPU firmware - boot, per-core entry points and trap handling
 *
 * crt0.S calls npu_init() and then core_dispatch(). core_dispatch()
 * sends each hart to the loop it owns. Those loops do not return.
 */

#include "npu_internal.h"


/* ================================================================
 * Chip ID and module detection
 * ================================================================ */

/* write reg unless it reads back as absent */
static void usb_phy_update(u32 reg, u32 clr, u32 set)
{
	u32 v = REG32(reg);

	if (v != 0xDEADBEEF)
		REG32(reg) = (v & ~clr) | set;
}

/* mtk_usb_PowerDown: port 0 powers down every PHY, 1-4 one each */
int mtk_usb_powerdown(u32 port)
{
	switch (port) {
	case 0:
		usb_phy_update(0x1FAC080C, 0xA0000000, 0xA0000000);
		delay_ms(1);
		usb_phy_update(0x1FAE080C, 0xA0000000, 0xA0000000);
		delay_ms(1);
		usb_phy_update(0x1FAC0318, 0x800000, 0);
		delay_ms(1);
		usb_phy_update(0x1FAE0318, 0x800000, 0);
		delay_ms(1);
		return 0;
	case 1:
		usb_phy_update(0x1FAC080C, 0xA0000000, 0xA0000000);
		return 0;
	case 2:
		usb_phy_update(0x1FAE080C, 0xA0000000, 0xA0000000);
		return 0;
	case 3:
		usb_phy_update(0x1FAC0318, 0x800000, 0x800000);
		delay_ms(1);
		return 0;
	case 4:
		usb_phy_update(0x1FAE0318, 0x800000, 0x800000);
		delay_ms(1);
		return 0;
	default:
		npu_printf("mtk_usb_PowerDown(%d). arg error\n", port);
		return -1;
	}
}

/* power-down hooks for ports the package does not have */
static void usb1_off(void)
{
	mtk_usb_powerdown(1);
	REG32(0x1FB00830) |= 0x8000;
}

static void usb2_off(void)
{
	mtk_usb_powerdown(2);
}

static void pcie0_off(void)
{
	REG32(0x1FA5B460) = 0;
}

static void pcie1_off(void)
{
	REG32(0x1FA5C460) = 0;
}

static void npu_reboot(void);
static s32 chip_idx = -1;
static void (*port_off_hooks[6])(void);

/* Reboot */
static void npu_reboot(void)
{
	npu_printf("REBOOTING...\n");
	delay_ms_mcycle(10);
	REG32(0x1FB00040) = 0x80000001;
}


/* ================================================================
 * Per-core main functions
 * ================================================================ */

#ifdef HAS_TUNNEL
/* The tunnel offload never returns. On the eight-core part core 7 runs
 * it; the six-core part has no core 7 and gives it to core 0 once the
 * WiFi init is done. */
static void __attribute__((noreturn)) tunnel_offload_loop(u32 core,
							  const char *who)
{
	u32 pkt_len = 0, desc_ptr = 0;

	npu_printf("%s for npu tunnel offload\n", who);

	/* mailbox 7 (source 15) on every part */
	plic_register_isr(15, mbox_isr);
	npu_bridge_buf_init();

#ifdef HAS_TR471
	tr471_main_init();
#endif
	while (1) {
		/* ECN marking only while the queue is empty */
		if (tunnel_dequeue(core, &pkt_len, &desc_ptr) != 0) {
			if (!tunnel_ecn_enabled)
				continue;
			l4s_ecn_process(1);
			l4s_ecn_process(2);
			if (tunnel_dequeue(core, &pkt_len, &desc_ptr) != 0)
				continue;
		}
		if (tunnel_offload_handler(core, pkt_len,
					   (u32 *)desc_ptr) == -1)
			tunnel_pkt_drop(core, pkt_len, desc_ptr);
	}
}
#endif

static void __attribute__((noinline)) core0_main(void)
{
	tdma_init();

#ifdef HAS_BME
	/* kite runs the BMGR; eagle runs the buffer id pool */
	if (tdma_bmgr_mode != 0) {
		npu_printf("do tdma_bmgr_init\n");
		tdma_bmgr_init();
	} else {
		bufid_pool_init();
	}
#elif defined(HAS_WIFI)
	buf_mgr_init();
#endif

#if defined(AN7552) && defined(WIFI_EAGLE)
	/* bridge (type 129) before the WiFi SRAM types */
	npu_bridge_buf_init();
#endif
	core0_wifi_init_wrapper();
#ifndef AN7552
	/* AN7552 has no debug counter ISR */
	plic_register_isr(59, dbg_cnt_isr);
#endif

#if defined(AN7583) && defined(HAS_TUNNEL)
	tunnel_offload_loop(0, "npu_tunnel_offload");
#else
#if !(defined(AN7552) && defined(WIFI_EAGLE))
	npu_bridge_buf_init();
#endif
	npu_printf("%s\n", "core0_main");
#if defined(AN7552) && defined(WIFI_EAGLE)
	/* TODO run the tunnel dequeue here too */
	eagle_core0_loop();
#endif
#endif
}

static void __attribute__((noinline)) core1_main(void)
{
#if defined(WIFI_EAGLE)
	eagle_rxdmad_loop();
	npu_printf("%s finish\n", "core1_wifi_init_wrapper");
#elif defined(HAS_WIFI)
	kite_core1_loop();
#endif
}

#if MAX_CORE_NUM > 2
static void __attribute__((noinline)) core2_main(void)
{
	plic_register_isr(18, timer_isr);
#ifdef WIFI_EAGLE
	eagle_tx_fast_path();
#elif defined(HAS_WIFI)
	kite_core2_loop();
#endif
}

static void __attribute__((noinline)) core3_main(void)
{
	s32 i;

	/* find the package, then switch off the ports it does not have */
	for (i = 0; chip_cap_query(i, 0) != -1; i++) {
		if (chip_cap_query(i, 0)) {
			chip_idx = i;
			goto found;
		}
	}
	npu_printf("unknown chipid, module load fail!\n");
	npu_reboot();
found:
	if (!chip_cap_query(chip_idx, 2))
		port_off_hooks[1] = usb1_off;
	if (!chip_cap_query(chip_idx, 3))
		port_off_hooks[2] = usb2_off;
	if (!chip_cap_query(chip_idx, 4))
		port_off_hooks[3] = pcie0_off;
	if (!chip_cap_query(chip_idx, 5))
		port_off_hooks[4] = pcie1_off;
	for (i = 0; i < 6; i++)
		if (port_off_hooks[i])
			port_off_hooks[i]();

	core3_wifi_init_wrapper();
}

static void core4_wifi_init_wrapper(void)
{
#if defined(WIFI_EAGLE)
	eagle_rx_refill_loop();
#elif defined(HAS_WIFI)
	npu_printf("%s core 4 do nothing\n", "core4_wifi_init_wrapper");
#endif
}

static void __attribute__((noinline)) core4_main(void)
{
	npu_printf("%s\n", "core4_main");
	core4_wifi_init_wrapper();
}

static void __attribute__((noinline)) core5_main(void)
{
#if defined(HAS_DBA)
	core5_dba_main();
#elif defined(AN7581) && defined(HAS_WIFI)
	npu_printf("%s\n", "core5_main");
	wifi_bridge_loop();
#else
	npu_printf("%s\n", "core5_main");
#endif
}
#endif /* MAX_CORE_NUM > 2 */

#if MAX_CORE_NUM > 6
static void core6_wifi_init_wrapper(void)
{
#ifdef HAS_WIFI
	npu_printf("%s core 6 do nothing\n", "core6_wifi_init_wrapper");
#endif
}

static void __attribute__((noinline)) core6_main(void)
{
	npu_printf("%s\n", "core6_main");
	core6_wifi_init_wrapper();
}

static void core7_wifi_init_wrapper(void)
{
#ifdef HAS_WIFI
	npu_printf("%s core 7 do nothing\n", "core7_wifi_init_wrapper");
#endif
}

static void __attribute__((noinline)) core7_main(void)
{
#ifdef HAS_TUNNEL
	core7_wifi_init_wrapper();
	tunnel_offload_loop(7, "core7_main");
#else
	npu_printf("%s\n", "core7_main");
#endif
}

#endif /* MAX_CORE_NUM > 6 */


/* ================================================================
 * Core dispatch
 * ================================================================ */

void core_dispatch(int a0, int a1)
{
	u32 core = get_hartid();

	(void)a0;
	(void)a1;

	switch (core) {
	case 0:
		core0_main();
		break;
	case 1:
		core1_main();
		break;
#if MAX_CORE_NUM > 2
	case 2:
		core2_main();
		break;
	case 3:
		core3_main();
		break;
	case 4:
		core4_main();
		break;
	case 5:
		core5_main();
		break;
#endif
#if MAX_CORE_NUM > 6
	case 6:
		core6_main();
		break;
	case 7:
		core7_main();
		break;
#endif
	default:
		npu_printf("\nWrong Core ID: %d", core);
		break;
	}
}


/* ================================================================
 * Trap vector (machine-mode exception/interrupt handler)
 * ================================================================ */

/* Returns the mepc to resume at. Only the machine external interrupt goes to
 * the PLIC; anything else is reported and stepped over. Routing an exception
 * through the ISR table lands in default_isr, whose npu_printf re-enters a
 * printf that already holds the printf mutex, and a held-mutex acquire stalls
 * the NPU bus. */
u32 trap_dispatch(u32 mcause, u32 mepc, u32 *sp, u32 ra)
{
	if (mcause == MCAUSE_MACHINE_EXT_IRQ) {
		call_isr_by_src(REG32(PLIC_CLAIM_REG) - 1);
		return mepc;
	}

	npu_printf("UnKnow Interrupts (mcause:0x%x, RA:0x%x, SP:0x%x, EPC:%x)\n",
		   mcause, ra, (u32)sp, mepc);

	/* step over the faulting instruction; it may be compressed */
	return mepc + (((*(volatile u16 *)mepc) & 3) == 3 ? 4 : 2);
}

void __attribute__((naked, aligned(4))) trap_vector(void)
{
	__asm__ volatile(
		"addi sp, sp, -128\n"
		"sw ra,   0(sp)\n"
		"sw t0,   4(sp)\n"
		"sw t1,   8(sp)\n"
		"sw t2,  12(sp)\n"
		"sw a0,  16(sp)\n"
		"sw a1,  20(sp)\n"
		"sw a2,  24(sp)\n"
		"sw a3,  28(sp)\n"
		"sw a4,  32(sp)\n"
		"sw a5,  36(sp)\n"
		"sw a6,  40(sp)\n"
		"sw a7,  44(sp)\n"
		"sw t3,  48(sp)\n"
		"sw t4,  52(sp)\n"
		"sw t5,  56(sp)\n"
		"sw t6,  60(sp)\n"
		"sw s0,  64(sp)\n"
		"sw s1,  68(sp)\n"
		"sw s2,  72(sp)\n"
		"sw s3,  76(sp)\n"
		"sw s4,  80(sp)\n"
		"sw s5,  84(sp)\n"
		"sw s6,  88(sp)\n"
		"sw s7,  92(sp)\n"
		"sw s8,  96(sp)\n"
		"sw s9, 100(sp)\n"
		"sw s10,104(sp)\n"
		"sw s11,108(sp)\n"
		"csrr a0, mcause\n"
		"csrr a1, mepc\n"
		"mv a2, sp\n"
		"mv a3, ra\n"
		"call trap_dispatch\n"
		"csrw mepc, a0\n"
		"li t0, 0x1800\n"
		"csrs mstatus, t0\n"
		"lw ra,   0(sp)\n"
		"lw t0,   4(sp)\n"
		"lw t1,   8(sp)\n"
		"lw t2,  12(sp)\n"
		"lw a0,  16(sp)\n"
		"lw a1,  20(sp)\n"
		"lw a2,  24(sp)\n"
		"lw a3,  28(sp)\n"
		"lw a4,  32(sp)\n"
		"lw a5,  36(sp)\n"
		"lw a6,  40(sp)\n"
		"lw a7,  44(sp)\n"
		"lw t3,  48(sp)\n"
		"lw t4,  52(sp)\n"
		"lw t5,  56(sp)\n"
		"lw t6,  60(sp)\n"
		"lw s0,  64(sp)\n"
		"lw s1,  68(sp)\n"
		"lw s2,  72(sp)\n"
		"lw s3,  76(sp)\n"
		"lw s4,  80(sp)\n"
		"lw s5,  84(sp)\n"
		"lw s6,  88(sp)\n"
		"lw s7,  92(sp)\n"
		"lw s8,  96(sp)\n"
		"lw s9, 100(sp)\n"
		"lw s10,104(sp)\n"
		"lw s11,108(sp)\n"
		"addi sp, sp, 128\n"
		"mret\n"
	);
}


/* ================================================================
 * NPU initialization (called from crt0 before core_dispatch)
 * ================================================================ */

/* .data + .bss must fit the global-variable window of the NPU SRAM */
#define GLB_VAR_SRAM_SIZE  0x7800
extern char __bss_end[];

void npu_init(void)
{
	u32 hart = get_hartid();
	u32 mib21;

	/* hart 0 clears sync flag; others wait */
	if (hart != 0)
		delay_ms_mcycle(10);
	else
		core_sync_flag = 0;
	if (hart != 0 && core_sync_flag == 0) {
		do
			delay_ms_mcycle(100);
		while (core_sync_flag == 0);
	}

	/* init PLIC for this hart */
	plic_init();

	/* configure trap vector and enable machine external interrupts */
	irq_disable();
	csr_write(mtvec, (unsigned long)trap_vector);
	csr_write(mie, 0x800);
	irq_enable();

	/* hart 0 conditional block: first-time initialization */
	if (npu_reset_pending != 0) {
		npu_reset_pending = 0;
		/* MIB21 gates npu_printf and does not survive the reset pulse */
		mib21 = REG32(NPU_MIB21);
		sim_mode_flag = REG32(NPU_MIB12);
		REG32(NPU_SCU_RSTCTRL1) = NPU_SCU_RST_ALL;
		delay_ms_mcycle(1);
		REG32(NPU_SCU_RSTCTRL1) = 0;
		delay_ms_mcycle(1);
		REG32(NPU_THREAD_ENABLE) = 4;
		REG32(NPU_THREAD_ENABLE) = 1;
		delay_ms_mcycle(1);
		REG32(NPU_MIB0) = ALL_FF;
		boot_uart_init();
		REG32(NPU_MIB21) = mib21;
		npu_printf("core freq at %d MHz\n", cpu_clock_get());
		if ((u32)__bss_end - NPU_SRAM_BASE > GLB_VAR_SRAM_SIZE)
			npu_printf("Error: OVER GLB_VAR_SRAM_SIZE. "
				   "_data:0x%x, __bss_end:0x%x\n",
				   NPU_SRAM_BASE, (u32)__bss_end);
		plic_register_isr(22, uart_rx_isr);
		timer_init(0, 1, 10);
		mailbox_init();
	}

	/* hart 0 signals others to proceed */
	if (hart == 0)
		core_sync_flag = 1;

	/* boot signature */
	REG32(NPU_MIB31) = INIT_COMPLETE;
}
