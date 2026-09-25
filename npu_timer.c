/*
 * AN75XX NPU firmware - timers, CPU clock and delays
 *
 * AN7581 has 4 timers, AN7552 has 8 and AN7583 has 16 over two banks.
 * The five tables in npu_globals.c index them.
 */

#include "npu_internal.h"


/* ================================================================
 * Timer
 * ================================================================ */

u32 timer_get_bit(u32 src)
{
	u32 idx;

	for (idx = 0; idx < NPU_TIMER_NUM; idx++) {
		if (timer_irq_map[idx] == src)
			return timer_clr_bit[idx];
	}
	npu_printf("Error: %s can't find intrBit for intSrc:%d\n",
		   "get_tmrIntrBit_byIntSrc", src);
	return 20;
}

void timer_isr(int src)
{
	u32 bit = timer_get_bit((u32)src);
	u32 base;
	u32 tick;

	if (src >= 24 && src <= 31)
		base = NPU_TIMER1_BASE;
	else
		base = NPU_TIMER0_BASE;

#ifdef AN7552
	/* ack from the control word seen at the first tick; the tick
	 * count is the AN7552 time base */
	if (timer_prev_ctrl == 0)
		timer_prev_ctrl = REG32(base);
	REG32(base) = (timer_prev_ctrl & 0x1E0001EF) | (1u << bit);
	timer_int_count++;
	return;
#endif
	/* ack: rewrite the control word with only this timer's clear bit set */
	timer_prev_ctrl = REG32(base);
	REG32(base) = (timer_prev_ctrl & 0x1E0001EF) | (1u << bit);
	timer_int_count++;

	tick = timer_raw_tick + 1;
	timer_raw_tick = tick;

	if (tick % 100 == 0)
		timer_slow_tick++;

	if (timer_tod_usec + 100000 > 999999999u) {
		timer_tod_sec++;
		timer_tod_usec -= 999900000;
	} else {
		timer_tod_usec += 100000;
	}
}

void timer_init(int timer, int enable, int period)
{
	u32 base = (timer >= 8) ? NPU_TIMER1_BASE : NPU_TIMER0_BASE;
	u32 pair = 0;
	u32 ctrl;

	timer_int_count = 0;
	if ((u32)timer >= NPU_TIMER_NUM) {
		npu_printf("%s timer_no:%d is wrong, should be smaller than %d\n",
			   "timer_init", timer, NPU_TIMER_NUM);
		return;
	}
#if !defined(AN7581)
	if ((u32)(timer - 3) <= 3)
		pair = timer_pair_bit[timer - 3];
#endif
	if (!enable) {
		REG32(base) &= ~(1u << timer_bit_map[timer]);
		return;
	}

#if defined(AN7583)
	timer_clk_mhz = 50;
	npu_printf("%s: Timer clk is running at %d Mhz\n", "timer_init", 50);
	REG32(timer_reload_reg[timer]) = 50000 * (u32)period;
#else
	timer_clk_mhz = cpu_clock_div4();
	npu_printf("%s: Timer clk is running at %d Mhz\n", "timer_init",
		   timer_clk_mhz);
#if defined(AN7581)
	REG32(timer_reload_reg[timer]) =
		(u32)period * cpu_clock_div4() * 1000 / 100;
#else
	REG32(timer_reload_reg[timer]) = 1000 * (u32)period * cpu_clock_div4();
	/* AN7552 has no timer core: the tick runs on hart 0 */
	plic_register_isr(timer_irq_map[timer], timer_isr);
#endif
#endif
	ctrl = REG32(base) | (1u << timer_bit_map[timer]);
	if (pair)
		ctrl &= ~(1u << pair);
	REG32(base) = ctrl;
}

void delay_1ms(u32 ms)
{
	volatile u32 i, j;

	for (i = 0; i < ms; i++)
		for (j = 0; j < 25000; j++)
			;
}

/* PLL clock: selector picks a frequency, bits[2:0]+1 is the divider */
#define PLL_CFG_REG 0x1FA201FC
#if defined(AN7583)
#define PLL_SEL_SHIFT   9
#define PLL_FREQ_TABLE  { 666, 800, 720, 600 }
#else
#define PLL_SEL_SHIFT   8
#define PLL_FREQ_TABLE  { 800, 750, 720, 600 }
#endif

u32 cpu_clock_get(void)
{
	static const u32 pll_freq[] = PLL_FREQ_TABLE;

	if (sim_mode_flag != 0)
		return 100;
	return pll_freq[(REG32(PLL_CFG_REG) >> PLL_SEL_SHIFT) & 3] /
	       ((REG32(PLL_CFG_REG) & 7) + 1);
}

static u32 cpu_clock_div2(void)
{
	if (sim_mode_flag != 0)
		return 50;
	return cpu_clock_get() >> 1;
}

u32 cpu_clock_div4(void)
{
	if (sim_mode_flag != 0)
		return 25;
	return cpu_clock_get() >> 2;
}

static void watchdog_timer_init(int enable, int period)
{
	if (!enable)
		return;
#if defined(AN7583)
	REG32(NPU_TIMER_WDT_RELOAD) = 50000 * (u32)period;
#else
	REG32(NPU_TIMER_WDT_RELOAD) = 1000 * (u32)period * cpu_clock_div4();
#endif
	REG32(NPU_TIMER0_BASE) &= ~0x20u;
	REG32(NPU_TIMER0_BASE) |= 0x2000020u;
}

#ifdef AN7581
static void multi_bank_timer_init(int enable, int prescale, int period, int bank)
{
	if (!enable) {
		REG32(NPU_TIMER_BANK_CTRL(bank)) &= 0xFDFFFFDFu;
		return;
	}
	REG32(NPU_TIMER_BANK_PRESCALE(bank)) = (prescale != -1 && prescale != 1)
		? 25000 * (u32)prescale : (u32)prescale;
	REG32(NPU_TIMER_BANK_RELOAD(bank)) = 1000 * (u32)period * cpu_clock_div4();
	REG32(NPU_TIMER_BANK_CTRL(bank)) &= ~0x20u;
	REG32(NPU_TIMER_BANK_CTRL(bank)) |= 0x2000020u;
}
#endif

static void cpu_timer_init(int idx, int enable, int period)
{
	u32 bit = 1u << idx;

	if (idx > 2)
		npu_printf("%s cpu_tmr:%d is wrong, should be 0 or 1\n",
			   "cpu_timer_init", idx);
	if (enable) {
#if defined(AN7583)
		REG32(CPU_TIMER_RELOAD(idx)) = 50000 * (u32)period;
#else
		REG32(CPU_TIMER_RELOAD(idx)) = 1000 * (u32)period *
			(sim_mode_flag ? 50u : 200u);
#endif
		REG32(CPU_TIMER_COUNTER(idx)) = 0;
		REG32(NPU_CPU_TIMER_BASE) |= bit;
	} else {
		REG32(NPU_CPU_TIMER_BASE) &= ~bit;
	}
}

void delay_us(u32 us)
{
	u32 target = 800 * us;
	u32 prev = csr_read(mcycle);
	u32 elapsed = 0;

	do {
		u32 cur = csr_read(mcycle);

		if (cur >= prev)
			elapsed += cur - prev;
		else
			elapsed += cur - prev - 1;
		prev = cur;
	} while (elapsed < target);
}

void delay_ms(u32 ms);

void delay_ms_mcycle(u32 ms)
{
	u32 clk = 1000 * cpu_clock_get();
	u32 target, prev, elapsed = 0;

	if (ms >= 0xFFFFFFFFu / clk) {
		npu_printf("Error(%s) ms:%d is too large\n", "__delay", ms);
		return;
	}
	target = ms * clk;
	prev = csr_read(mcycle);

	do {
		u32 cur = csr_read(mcycle);

		if (cur >= prev)
			elapsed += cur - prev;
		else
			elapsed += cur - prev - 1;
		prev = cur;
	} while (elapsed < target);
}

void delay_ms(u32 ms)
{
	delay_ms_mcycle(ms);
}
