/*
 * AN75XX NPU firmware - reconstructed from binary
 *
 * Single-file firmware for Airoha AN7552/AN7581/AN7583 NPU.
 * Compile-time variants via -DAN75xx -DMTxxxx.
 */

#include "npu_config.h"
#include "npu_types.h"
#include "npu_regs.h"

/* ---- forward declarations ---- */

/* trap handler (in assembly section at bottom) */
void trap_vector(void);

/* core main functions */
static void core0_main(void);
static void core1_main(void);
#if MAX_CORE_NUM > 2
static void core2_main(void);
static void core3_main(void);
static void core4_main(void);
static void core5_main(void);
#endif
#if MAX_CORE_NUM > 6
static void core6_main(void);
static void core7_main(void);
#endif

/* subsystem init */
static void plic_init(void);
static void timer_init(int timer, int enable, int period);
static void mailbox_init(void);

/* PLIC */
static void default_isr(int src);
void call_isr_by_src(u32 src);
static void plic_enable(u32 src);
static void plic_disable(u32 src);

/* mailbox */
static void mbox_isr(int src);

/* printf */
int npu_printf(const char *fmt, ...);

/* mutex */
static void hw_mutex_lock(u32 *desc);
static void hw_mutex_unlock(u32 *desc);

/* WiFi handlers - forward declared for data init */
#ifdef WIFI_KITE
static int wifi_mail_set_wait(u32 base, u32 cnt);
static int wifi_mail_set_event(u32 base, u32 cnt);
#endif

/* tunnel handlers */
#ifdef HAS_TUNNEL
static int tunnel_mail_handler(u32 base, u32 cnt);
#endif

/* DBA handlers */
#ifdef HAS_DBA
static int dba_mail_handler(u32 base, u32 cnt);
#endif

/* ================================================================
 * Global variables - .data section (initialized)
 *
 * Order must match npu_data.bin layout for binary-correct output.
 * Addresses shown are for AN7583_MT7996 reference variant.
 * ================================================================ */

/* 0x000: system config base */
u32 npu_max_frame_size = 1500;
static u32 __data_pad0[3];

/* 0x010: WiFi HW queue-to-AC mapping (3 bands x 16 entries) */
static u32 wifi_queue_map_b0[16] = {
	18, 19, 20, 52, 53, 54, 55, 21, 24, 25, 26, 27, 28, 29, 30, 31
};
static u32 wifi_queue_map_b1[8] = {
	16, 17, 18, 21, 22, 23, 24, 19
};
static u32 wifi_queue_map_b2[8] = {
	16, 17, 18, 21, 22, 23, 24, 19
};

/* 0x090: timer bit-index table */
static u32 timer_bit_map[16] = {
	0, 1, 2, 5, 6, 7, 8, 3, 0, 1, 2, 5, 6, 7, 8, 3
};

/* 0x0D0: PLIC source-to-register mapping (2 sets of 16) */
static u32 plic_src_reg_map[32] = {
	0x1EC10108, 0x1EC10110, 0x1EC10118, 0x1EC10120,
	0x1EC10128, 0x1EC10130, 0x1EC10138, 0x1EC10140,
	0x1EC10148, 0x1EC10150, 0x1EC10158, 0x1EC10160,
	0x1EC10168, 0x1EC10170, 0x1EC10178, 0x1EC10180,
	0x1EC10108, 0x1EC10110, 0x1EC10118, 0x1EC10120,
	0x1EC10128, 0x1EC10130, 0x1EC10138, 0x1EC10140,
	0x1EC10148, 0x1EC10150, 0x1EC10158, 0x1EC10160,
	0x1EC10168, 0x1EC10170, 0x1EC10178, 0x1EC10180,
};

/* 0x150+: additional PLIC config and WiFi handler pointers
 * (exact layout TBD - placeholder for binary matching) */
static u32 plic_config_ext[16];

/* 0x1C0: mailbox command dispatch table (kite WiFi) */
#ifdef WIFI_KITE
static mbox_handler_t wifi_mbox_handlers[10];
#endif

/* 0x1EC: WiFi chip name table (kite only, 8 bytes per entry) */
#ifdef WIFI_KITE
static const char wifi_chip_names[6][8] = {
	"MT7915A", "MT7915D", "MT7916",
	"MT7902",  "MT7990",  ""
};
#endif

/* 0x220: mailbox wrapper function pointer tables */
static mbox_handler_t mbox_pri_handlers[10];
static mbox_handler_t mbox_ext_handlers[32];

/* 0x2D0: tunnel config structures */
#ifdef HAS_TUNNEL
static u32 tunnel_config_ptrs[4];
static u8 tunnel_config_area[0x500];
#endif

/* 0x7E8: debug/exception handler dispatch */
static u32 exception_handlers[12];
static u32 exception_sentinels[4] = {
	0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0x0000FFFF
};

/* 0x820: UART console descriptors */
static u8 uart_desc[2][32] = {
	{ 0x01, 0x00, 'A','S','C','I','I', 0 },
	{ 0 }
};

/* 0x864: mutex descriptor pair table */
static u32 mutex_initial_state;
static u32 mutex_desc_pairs[512];

/* 0xC70: per-variant config scalars */
static s32 wifi_chip_index = -1;

#ifdef HAS_DBA
static u8 dba_cfg_flag0 = 1;
static u8 dba_cfg_flag1 = 1;
static u8 dba_cfg_flag2;
static u32 dba_alloc_gran = 5;
static u32 dba_tick_count = 10;
static u32 dba_defaults[4] = { 1, 1, 1, 1 };
static u32 dba_band_count = 1;
static u32 dba_bw_unit = 46;
static u32 dba_max_alloc_ids = 128;
static u32 dba_cfg3 = 1;
static u32 dba_max_frame = 1500;
static u32 dba_alloc_mask = 0xFFFF;
#endif

u32 npu_reset_pending = 1;
u32 npu_printf_prefix = 1;

#ifdef HAS_DBA
static u8 dba_cfg4 = 1;
#endif

#ifdef HAS_TUNNEL
static u32 tunnel_max_count = 7;
static u32 tunnel_timeout_ms = 100;
static u8 tunnel_cfg_flag = 1;
#endif

static s32 sentinel_1 = -1;

#ifdef HAS_DBA
static u16 dba_cfg_word0 = 25;
static u16 dba_cfg_word1 = 10;
#endif

static s32 sentinel_2 = -1;

#ifdef HAS_TUNNEL
static u32 tunnel_dispatch_ptr;
#endif

static s32 sentinel_3 = -1;
static u32 config_flags = 0x00020000;

static const char delay_name[] = "__delay";

/* ================================================================
 * Global variables - .bss section (zero-initialized)
 *
 * Order must match BSS layout for correct GP-relative access.
 * ================================================================ */

/* mailbox handler slots */
static mbox_handler_t mbox_core_handlers[6];

/* printf subsystem */
static u8 printf_cfg_flags[4];
static u8 printf_desc[8];
static u8 printf_flag;

/* timer subsystem */
static u32 timer_context[10];
static u32 timer_ref_counts[8];
static u8 srv6_my_ipv6[16];
static u32 timer_isr_context[12];
static isr_fn_t timer_isr_handler0;
static isr_fn_t timer_isr_handler1;
static u32 timer_isr_pad[4];
static isr_fn_t timer_isr_handler2;
static u32 timer_pad2[100];
static isr_fn_t timer_callback;
static u32 timer_pad3;

/* printf buffer */
static char printf_buf[1024];

/* tunnel context */
#ifdef HAS_TUNNEL
static u8 tunnel_ctx[3][1024];
#endif

/* BME/bridge */
static u32 bme_base_addr;
static u32 bme_pad[38];
static u32 bme_config;
static u32 bme_pad2[10];
static u32 bme_desc_count;
static u32 bme_status;
static u32 bme_pad3;
static u32 bme_ring_state[4];
static u32 tdma_bme_dscp_idx;
static u32 tdma_bme_dscp_base;
static u32 npu_bridge_base;

/* PLIC ISR table: 192 function pointers */
static isr_fn_t plic_isr_table[192];

/* PLIC config */
static u32 plic_cfg[3];
static u32 printf_mutex_desc[2];

/* SRAM buffer manager */
static u32 sram_buf_mutex[2];
static u32 sram_buf_pad[4];
static u32 sram_buf_max_use;
static u32 sram_buf_cur_idx;
static u16 sram_buf_entries[200];
static u32 sram_buf_pad2[8];

/* WiFi state */
static u32 wifi_state[256];

#ifdef HAS_TUNNEL
static u8 tunnel_srv6_hdr_len[8];
#endif

/* DBA state */
#ifdef HAS_DBA
static u32 dba_state[128];
static u32 npu_fttr_base;
static u8 dba_band_switch;
static u8 dba_bwmap_switch;
static u32 dba_alloc_state[20];
static u32 dba_timer0_snap;
#endif

/* PLIC threshold per-hart */
static u8 plic_threshold_table[8];
static u32 plic_isr_init_done;
static u32 plic_cfg2;
static u32 plic_state;

/* npu_init sync */
static u32 mib12_snapshot;
static u32 core_sync_flag;

/* WiFi extended state */
static u32 wifi_ext_state[32];

/* ================================================================
 * Utility functions
 * ================================================================ */

void *npu_memset(void *dst, int c, u32 n)
{
	u8 *d = (u8 *)dst;

	while (n--)
		*d++ = (u8)c;
	return dst;
}

void *npu_memcpy(void *dst, const void *src, u32 n)
{
	u8 *d = (u8 *)dst;
	const u8 *s = (const u8 *)src;

	while (n--)
		*d++ = *s++;
	return dst;
}

static u32 npu_strlen(const char *s)
{
	const char *p = s;

	while (*p)
		p++;
	return (u32)(p - s);
}

static char get_core_char(void)
{
	u32 id = get_hartid();

	if (id <= 7)
		return (char)('0' + id);
	return 'X';
}

/* ================================================================
 * Hardware mutex
 * ================================================================ */

static void hw_mutex_lock(u32 *desc)
{
	u32 idx = (desc[0] & HW_MUTEX_IDX_MASK) >> 2;
	u32 hart = get_hartid();
	u32 val = (hart << 8) | 0x40;

	REG32(HW_MUTEX_ACQ(idx)) = val;
	while (!(REG32(HW_MUTEX_STATUS(hart, idx)) & 0x10000))
		REG32(HW_MUTEX_ACQ(idx)) = val;
}

static void hw_mutex_unlock(u32 *desc)
{
	u32 idx = (desc[0] & HW_MUTEX_IDX_MASK) >> 2;
	u32 hart = get_hartid();

	REG32(HW_MUTEX_REL(hart, idx)) = (hart << 8);
}

static void hw_mutex_lock_pri(u32 *desc)
{
	u32 idx = (desc[0] & HW_MUTEX_IDX_MASK) >> 2;
	u32 hart = get_hartid();
	u32 val;

	if (desc[1] != 0)
		val = (hart << 8) | 0x10040;
	else
		val = (hart << 8) | 0x40;

	REG32(HW_MUTEX_ACQ(idx)) = val;
	while (!(REG32(HW_MUTEX_STATUS(hart, idx)) & 0x10000))
		REG32(HW_MUTEX_ACQ(idx)) = val;
}

static void hw_mutex_unlock_pri(u32 *desc)
{
	hw_mutex_unlock(desc);
}

/* ================================================================
 * PLIC (Platform-Level Interrupt Controller)
 * ================================================================ */

static void default_isr(int src)
{
	npu_printf("(%s) implement code to clear intrSrc:%d then execute ISR here\n",
		   "default_isr", src);
}

void call_isr_by_src(u32 src)
{
	get_hartid();
	if (src > 191) {
		npu_printf("%s just return due to intSrc:%d > %d\n",
			   "call_isr_bySrc", src, 191);
		return;
	}
	plic_isr_table[src](src);
	REG32(PLIC_CLAIM_REG) = src + 1;
}

static void plic_set_threshold(void)
{
	u32 hart = get_hartid();

	REG32(PLIC_THRESHOLD_REG) = plic_threshold_table[hart] & 0x1F;
}

static u32 plic_get_priority(u32 src)
{
	if (src > 191) {
		npu_printf("%s just return due to intSrc:%d > %d\n",
			   "PLIC_get_priority_bySrc", src, 191);
		return 16;
	}
	return REG32(PLIC_PRIORITY(src)) & 0x1F;
}

static void plic_set_priority(u32 src, u8 prio)
{
	if (src > 191) {
		npu_printf("%s just return due to intSrc:%d > %d\n",
			   "PLIC_set_priority_bySrc", src, 191);
		return;
	}
	REG32(PLIC_PRIORITY(src)) = prio & 0x1F;
}

static void plic_enable(u32 src)
{
	u32 word_idx, bit;

	if (src > 191) {
		npu_printf("%s just return due to intSrc:%d > %d\n",
			   "PLIC_enable_intSrc_toOneCore", src, 191);
		return;
	}
	word_idx = (src + 1) >> 5;
	bit = 1u << ((src + 1) & 0x1F);
	REG32(PLIC_MASK_REG(word_idx)) &= ~bit;
	REG32(PLIC_ENABLE_REG(word_idx)) |= bit;
}

static void plic_disable(u32 src)
{
	u32 word_idx, bit;

	if (src > 191) {
		npu_printf("%s just return due to intSrc:%d > %d\n",
			   "PLIC_DISABLE_intSrc_toOneCore", src, 191);
		return;
	}
	word_idx = (src + 1) >> 5;
	bit = 1u << ((src + 1) & 0x1F);
	REG32(PLIC_MASK_REG(word_idx)) |= bit;
	REG32(PLIC_ENABLE_REG(word_idx)) &= ~bit;
}

static void plic_enable_wrapper(u32 src)
{
	plic_enable(src);
}

static void plic_init(void)
{
	u32 i;

	plic_set_threshold();

	for (i = 0; i < 192; i++) {
		if (i == 95)
			REG32(PLIC_PRIORITY(95)) = 17;
		else
			REG32(PLIC_PRIORITY(i)) = 16;

		if (get_hartid() == 0 && plic_isr_init_done == 0)
			plic_isr_table[i] = default_isr;
	}

	if (get_hartid() == 0)
		plic_isr_init_done = 1;
}

static void plic_register_isr(u32 src, isr_fn_t handler)
{
	if (src < 192) {
		plic_isr_table[src] = handler;
		plic_enable(src);
	}
}

/* ================================================================
 * Timer
 * ================================================================ */

static u32 timer_get_bit(u32 src)
{
	u32 idx;

	if (src >= 24 && src <= 31)
		idx = src - 24 + 8;
	else
		idx = src - 16;

	if (idx < 16)
		return timer_bit_map[idx];
	return 0;
}

static void timer_isr(int src)
{
	u32 bit = timer_get_bit((u32)src);
	u32 base;

	if (src >= 24 && src <= 31)
		base = NPU_TIMER1_BASE;
	else
		base = NPU_TIMER0_BASE;

	/* clear timer interrupt */
	REG32(base) |= (1u << bit);
	REG32(base) &= ~(1u << bit);
}

static void timer_init(int timer, int enable, int period)
{
	u32 base = NPU_TIMER0_BASE;
	u32 clk_mhz = 25;
	u32 reload;

	if (!enable) {
		REG32(base) &= ~(1u << timer);
		return;
	}

	reload = (u32)((u64)1000 * period * clk_mhz / 100);
	REG32(base + 0x10 + timer * 4) = reload;
	REG32(base) |= (1u << timer);
}

static void delay_1ms(u32 ms)
{
	volatile u32 i, j;

	for (i = 0; i < ms; i++)
		for (j = 0; j < 25000; j++)
			;
}

/* ================================================================
 * vsprintf (minimal implementation)
 * ================================================================ */

static int npu_vsprintf(char *buf, const char *fmt, u32 *args)
{
	char *p = buf;
	const char *f = fmt;
	int arg_idx = 0;
	char tmp[12];
	int i, len;

	while (*f) {
		if (*f != '%') {
			*p++ = *f++;
			continue;
		}
		f++;

		/* flags */
		int pad_zero = 0, left = 0, width = 0, is_long = 0;

		if (*f == '-') { left = 1; f++; }
		if (*f == '0') { pad_zero = 1; f++; }
		while (*f >= '0' && *f <= '9')
			width = width * 10 + (*f++ - '0');
		if (*f == 'l') { is_long = 1; f++; }
		if (*f == 'l') { f++; } /* ignore ll */

		switch (*f) {
		case 'd': {
			s32 v = (s32)args[arg_idx++];
			int neg = 0;
			u32 uv;

			if (v < 0) { neg = 1; uv = (u32)(-v); }
			else uv = (u32)v;

			len = 0;
			do { tmp[len++] = '0' + (uv % 10); uv /= 10; } while (uv);
			if (neg) tmp[len++] = '-';
			for (i = 0; i < width - len; i++)
				*p++ = pad_zero ? '0' : ' ';
			for (i = len - 1; i >= 0; i--)
				*p++ = tmp[i];
			break;
		}
		case 'u': {
			u32 v = args[arg_idx++];

			len = 0;
			do { tmp[len++] = '0' + (v % 10); v /= 10; } while (v);
			for (i = 0; i < width - len; i++)
				*p++ = pad_zero ? '0' : ' ';
			for (i = len - 1; i >= 0; i--)
				*p++ = tmp[i];
			break;
		}
		case 'x':
		case 'X': {
			u32 v = args[arg_idx++];
			const char *hex = (*f == 'X') ?
				"0123456789ABCDEF" : "0123456789abcdef";

			len = 0;
			do { tmp[len++] = hex[v & 0xF]; v >>= 4; } while (v);
			for (i = 0; i < width - len; i++)
				*p++ = pad_zero ? '0' : ' ';
			for (i = len - 1; i >= 0; i--)
				*p++ = tmp[i];
			break;
		}
		case 's': {
			const char *s = (const char *)args[arg_idx++];

			if (!s) s = "(null)";
			while (*s)
				*p++ = *s++;
			break;
		}
		case 'c':
			*p++ = (char)args[arg_idx++];
			break;
		case 'p': {
			u32 v = args[arg_idx++];
			const char *hex = "0123456789abcdef";

			*p++ = '0'; *p++ = 'x';
			for (i = 7; i >= 0; i--)
				*p++ = hex[(v >> (i * 4)) & 0xF];
			break;
		}
		case '%':
			*p++ = '%';
			break;
		default:
			*p++ = '%';
			*p++ = *f;
			break;
		}
		f++;
	}
	*p = '\0';
	return (int)(p - buf);
}

/* ================================================================
 * UART / Printf
 * ================================================================ */

static void uart_putc(char c)
{
	while (!(REG32(UART_TX_STATUS) & UART_TX_READY_BIT))
		;
	REG32(UART_TX_DATA) = (u32)c;
}

static void uart_puts_raw(const char *s)
{
	while (*s) {
		if (*s == '\n') {
			uart_putc('\r');
		}
		uart_putc(*s++);
	}
}

static void printf_enable(int enable)
{
	npu_printf_prefix = (u32)enable;
}

int npu_printf(const char *fmt, ...)
{
	int len;
	u32 *args;

	/* check if printing is suppressed by host */
	if (REG32(NPU_MIB(21)) != 0)
		return 0;

	irq_disable();
	hw_mutex_lock_pri(printf_mutex_desc);

	/* format into buffer */
	args = (u32 *)&fmt + 1;
	len = npu_vsprintf(printf_buf, fmt, args);

	/* print core prefix if enabled */
	if (npu_printf_prefix) {
		uart_putc('[');
		uart_putc('C');
		uart_putc(get_core_char());
		uart_putc(']');
	}

	/* output with LF→CRLF conversion */
	uart_puts_raw(printf_buf);

	hw_mutex_unlock_pri(printf_mutex_desc);
	irq_enable();

	return len;
}

/* boot printf (uses boot UART at 0x1EC10000) */
static int boot_printf(const char *fmt, ...)
{
	int len;
	u32 *args;
	char *p;

	hw_mutex_lock_pri(printf_mutex_desc);

	args = (u32 *)&fmt + 1;
	len = npu_vsprintf(printf_buf, fmt, args);

	/* output core prefix */
	while (!(REG32(BOOT_UART_STATUS) & UART_TX_READY_BIT)) ;
	REG32(BOOT_UART_TX) = '[';
	while (!(REG32(BOOT_UART_STATUS) & UART_TX_READY_BIT)) ;
	REG32(BOOT_UART_TX) = 'C';
	while (!(REG32(BOOT_UART_STATUS) & UART_TX_READY_BIT)) ;
	REG32(BOOT_UART_TX) = get_core_char();
	while (!(REG32(BOOT_UART_STATUS) & UART_TX_READY_BIT)) ;
	REG32(BOOT_UART_TX) = ']';

	/* output buffer */
	p = printf_buf;
	while (*p) {
		if (*p == '\n') {
			while (!(REG32(BOOT_UART_STATUS) & UART_TX_READY_BIT)) ;
			REG32(BOOT_UART_TX) = '\r';
		}
		while (!(REG32(BOOT_UART_STATUS) & UART_TX_READY_BIT)) ;
		REG32(BOOT_UART_TX) = *p++;
	}

	hw_mutex_unlock_pri(printf_mutex_desc);
	return len;
}

/* ================================================================
 * Mailbox communication
 *
 * 80 bytes per core in dispatch table:
 *   [0..63]  = raw data DWORDs (16 slots, indexed by func_id)
 *   [32..63] = raw data WORDs (overlaps with above)
 *   [48..79] = callback function pointers (8 slots)
 * ================================================================ */

/* mailbox dispatch table: 6 cores x 80 bytes */
static u8 mbox_dispatch[MAX_CORE_NUM][80];

/* mailbox handler slots registered during init */
static mbox_handler_t mbox_wifi_handler;
static mbox_handler_t mbox_wifi_handler2;
static mbox_handler_t mbox_notify_handler;
static mbox_handler_t mbox_cfg_handler;

static void mbox_isr(int src)
{
	u32 mbox_idx = (u32)src - 8;
	u32 rptr, base_ptr, max_cnt, func_idx;
	mbox_handler_t handler;

	if ((u8)mbox_idx != mbox_idx) {
		npu_printf("Error: core_id:%d != mbox_idx:%d\n",
			   mbox_idx, (u8)mbox_idx);
		return;
	}

	/* clear interrupt (write-1-to-clear) */
	REG32(MBOX_INT_STS) = (1u << mbox_idx);

	/* verify clear */
	if ((REG32(MBOX_INT_STS) >> mbox_idx) & 1) {
		npu_printf("Error(%s): cleaning mbox intr isn't done (mbox_status:0x%x, mbox_idx:%d)\n",
			   "mBox_isr", REG32(MBOX_INT_STS), (u8)mbox_idx);
		return;
	}

	/* read queue registers */
	base_ptr = REG32(MBQ_BASE_PTR(mbox_idx));
	max_cnt = REG32(MBQ_MAX_CNT(mbox_idx));
	rptr = REG32(MBQ_RPTR(mbox_idx));

	/* set response status if not already set */
	if (!(rptr & 1)) {
		rptr = (rptr & 0xFFFFFF00u) | (rptr & 0xE1) | 0x6;
		REG32(MBQ_RPTR(mbox_idx)) = rptr;
	}

	func_idx = (rptr >> 11) & 0xF;

	if (rptr & 0x20) {
		/* raw data path: store base_ptr and max_cnt */
		u32 *data = (u32 *)&mbox_dispatch[mbox_idx][0];
		u16 *cnt = (u16 *)&mbox_dispatch[mbox_idx][32];

		data[func_idx] = base_ptr;
		cnt[func_idx] = (u16)max_cnt;
	} else {
		/* callback path: dispatch to handler */
		u32 *callbacks = (u32 *)&mbox_dispatch[mbox_idx][48];

		handler = (mbox_handler_t)(void *)callbacks[func_idx];
		if (handler) {
			u32 ret = (u32)handler(base_ptr, max_cnt);
			rptr = (rptr & 0xFFFFFFE3u) | ((ret & 7) << 2);
		}

		/* signal completion */
		if (rptr & 1)
			REG32(MBQ_RPTR(mbox_idx)) = (rptr & ~2u) | 2;
	}
}

static void mailbox_init(void)
{
	u32 i;
	u32 *callbacks;

	/* enable per-core mailbox interrupts */
	for (i = 0; i < MAX_CORE_NUM; i++) {
		REG32(NPU_MBOX_BASE + 0x008 + i * 4) = (1u << i);
		plic_register_isr(8 + i, mbox_isr);
	}

	REG32(MBOX_INT_MASK0) = 256;
	plic_cfg[0] = 14;
	plic_state = 0;

	/* zero all dispatch tables */
	for (i = 0; i < MAX_CORE_NUM; i++)
		npu_memset(mbox_dispatch[i], 0, 80);

	/* register default handlers into core 0's callback slots */
	callbacks = (u32 *)&mbox_dispatch[0][48];
	callbacks[0] = (u32)(void *)mbox_wifi_handler;
	callbacks[1] = (u32)(void *)mbox_wifi_handler2;
	callbacks[5] = (u32)(void *)mbox_cfg_handler;

	/* register handler in last core's callback slot */
	callbacks = (u32 *)&mbox_dispatch[MAX_CORE_NUM - 1][48];
	callbacks[3] = (u32)(void *)mbox_notify_handler;
}

/* notify host via mailbox queue 8 */
static int mbox_notify_host(u32 base_ptr, u32 max_cnt, u32 func_id)
{
	u32 timeout = 30;
	u32 rptr;

	hw_mutex_lock_pri(printf_mutex_desc);

	REG32(MBQ_BASE_PTR(8)) = base_ptr;
	REG32(MBQ_MAX_CNT(8)) = max_cnt;
	REG32(MBQ_RPTR(8)) = (func_id << 11) | 1;
	REG32(MBQ_WPTR(8)) = 1;

	while (timeout--) {
		rptr = REG32(MBQ_RPTR(8));
		if (rptr & 2)
			break;
	}

	hw_mutex_unlock_pri(printf_mutex_desc);
	return (rptr & 2) ? 0 : -1;
}

/* ================================================================
 * Chip ID and module detection
 * ================================================================ */

static u32 chip_id;
static u32 chip_rev;

static void chip_id_query(void)
{
	chip_id = REG32(CHIP_ID_REG);
	chip_rev = REG32(CHIP_VARIANT_REG);

	npu_printf("chip_id=0x%x, chip_rev=0x%x\n", chip_id, chip_rev);
}

/* USB power down */
static void usb_powerdown(void)
{
	/* USB PHY power control */
	REG32(0x1FAC0700) |= (1u << 4);
	REG32(0x1FAE0700) |= (1u << 4);
}

/* Reboot */
static void npu_reboot(void)
{
	npu_printf("REBOOTING...\n");
	REG32(0x1FB00040) = 0x80000001;
}

/* ================================================================
 * SRAM buffer management
 *
 * 512KB SRAM at 0x3E800000, bump-allocated with alignment.
 * Alloc table: 100 entries of {u16 addr_type, u16 pad, u32 base}.
 * Layout descriptors in .rodata (dword_84020BC4).
 * ================================================================ */

/* SRAM alloc table entry: 8 bytes (u16 type + pad + u32 addr) */
#define SRAM_BASE         0x3E800000
#define SRAM_END          0x3E87FFFE
#define SRAM_SIZE         0x80000
#define SRAM_MAX_ENTRIES  100
#define SRAM_ERROR_ADDR   0x3E880000

static u32 sram_alloc_offset;
static u32 sram_alloc_count;
static u32 sram_alloc_calls;
static u16 sram_alloc_table[SRAM_MAX_ENTRIES * 4];

static void sram_buf_dump(void)
{
	u32 i;
	u16 *entry = sram_alloc_table;

	npu_printf("base:%x,total size:%x,current max use size:%x\n",
		   SRAM_BASE, SRAM_SIZE, sram_alloc_offset);
	for (i = 0; i < SRAM_MAX_ENTRIES; i++) {
		u32 addr = *(u32 *)(entry + 2);

		if (addr == 0)
			break;
		npu_printf("Idx=%d,AddrType=%d,baseaddr=%x\n",
			   i, (u32)entry[0], addr);
		entry += 4;
	}
}

/* ================================================================
 * Debug counter ISR
 * ================================================================ */

static void dbg_cnt_isr(int src)
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

/* ================================================================
 * TDMA / Buffer Manager initialization
 * ================================================================ */

/* Buffer manager state (software free-list path) */
static u32 buf_mgr_alloc_cfg;
static u32 buf_mgr_alloc_idx;
static u32 buf_mgr_free_cfg;
static u32 buf_mgr_free_idx;
static u32 buf_mgr_id_base;
static u16 buf_mgr_alloc_widx;
static u16 buf_mgr_free_widx;

/* TDMA init: zero 512KB SRAM, clear alloc table */
static void tdma_init(void)
{
	npu_memset((void *)SRAM_BASE, 0, SRAM_SIZE);
	npu_memset(sram_alloc_table, 0, sizeof(u16) * 4 * SRAM_MAX_ENTRIES);
	sram_buf_mutex[0] = 2;
	sram_buf_mutex[1] = 0;
	sram_alloc_offset = 0;
	sram_alloc_count = 0;
}

#ifdef WIFI_KITE
/* TDMA BME init: hardware buffer-ID allocator path */
static void tdma_bmgr_init(void)
{
	u32 buf_base;

	npu_printf("do tdma_bmgr_init\n");

	/* allocate 5600-entry buffer ID pool from SRAM */
	/* buf_base = sram_buf_alloc(138); -- simplified */
	buf_base = SRAM_BASE + sram_alloc_offset;
	sram_alloc_offset += 5600 * 2;

	REG32(BMGR_BUF_ID_BASE) = buf_base;
	REG32(BMGR_BASE + 0x004) = 0;
	REG32(BMGR_BASE + 0x008) = 5600;
	REG32(BMGR_BASE + 0x00C) = 2;
	REG32(BMGR_BASE + 0x010) = 2;
	REG32(BMGR_BASE + 0x028) = 15;
	REG32(BMGR_BASE + 0x030) = 7;
	REG32(BMGR_INIT) = 1;

	/* poll for HW init completion */
	while (!(REG32(BMGR_BASE + 0x02C) & 1))
		;

	/* register BME done ISR on PLIC source 33 */
	plic_enable_wrapper(33);
}
#endif

/* Software buffer manager init (non-TDMA path) */
static void buf_mgr_init(void)
{
	u16 *pool;
	u32 i;

	buf_mgr_alloc_cfg = 12;
	buf_mgr_alloc_idx = 0;
	buf_mgr_free_cfg = 13;
	buf_mgr_free_idx = 0;

	pool = (u16 *)(SRAM_BASE + sram_alloc_offset);
	sram_alloc_offset += 5600 * 2;
	buf_mgr_id_base = (u32)pool;

	for (i = 0; i < 5600; i++)
		pool[i] = (u16)i;

	buf_mgr_alloc_widx = 0;
	buf_mgr_free_widx = 0;
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

/* WiFi bridge state */
static u8 wifi_bridge_enabled;
static u16 wifi_bridge_ch_count;
static u8 wifi_bridge_report;
static u8 wifi_bridge_active;
static u32 wifi_tx_pending;
static u32 wifi_rx_pending;

/* WiFi bridge init */
static void wifi_bridge_init(void)
{
#ifdef HAS_WIFI
	u32 i;

	wifi_bridge_report = 0;
	wifi_bridge_enabled = 1;
	wifi_bridge_ch_count = 3;

	/* clear TX ring state */
	for (i = 0; i < 16; i++) {
		/* zero ring descriptors, stats, set seq = 0xFF */
	}

	/* clear RX ring state */
	for (i = 0; i < 16; i++) {
		/* zero ring descriptors, stats, set seq = 0xFF */
	}

	wifi_bridge_active = 0;
	wifi_rx_pending = 0;
	wifi_tx_pending = 0;
#endif
}

/* Core0 WiFi init wrapper */
static void core0_wifi_init_wrapper(void)
{
#ifdef HAS_WIFI
	/* sub_84006278: tdma_tx_init - TX ring descriptors */
	/* sub_84013618: wifi_bridge_init */
	wifi_bridge_init();
	npu_printf("%s finish\n", "core0_wifi_init_wrapper");
	/* sub_840144F4: hostadpt_init check */
#endif
}

/* Core3 WiFi init wrapper - polls for init completion */
static void core3_wifi_init_wrapper(void)
{
#ifdef HAS_WIFI
	/* wait for WiFi init to complete */
	while (!(wifi_bridge_active & 2)) {
		if (wifi_tx_pending != 0) {
			/* process TX */
		}
		if (wifi_rx_pending != 0) {
			/* process RX */
		}
	}
	npu_printf("%s finish\n", "core3_wifi_init_wrapper");
#endif
}

/* WiFi bridge main loop (runs on dedicated core) */
static void wifi_bridge_loop(void)
{
#ifdef HAS_WIFI
	while (1) {
		if (wifi_tx_pending != 0) {
			/* sub_84012410: TX processing */
		}
		if (wifi_rx_pending != 0 && wifi_bridge_enabled) {
			/* sub_84012114: RX processing */
		}
	}
#endif
}

/* ================================================================
 * Tunnel offload (AN758X)
 * ================================================================ */

#ifdef HAS_TUNNEL

static void tunnel_init(void)
{
	npu_printf("tunnel_init\n");
	npu_memset(tunnel_ctx, 0, sizeof(tunnel_ctx));
}

static void tunnel_process(void)
{
	/* tunnel packet processing loop */
}

static int tunnel_mail_handler(u32 base, u32 cnt)
{
	npu_printf("tunnel_mail_handler\n");
	return 1;
}

#endif /* HAS_TUNNEL */

/* ================================================================
 * DBA - Dynamic Bandwidth Allocation (AN7583 + WiFi)
 * ================================================================ */

#ifdef HAS_DBA

static void dba_init(void)
{
	npu_printf("dba_init\n");
	npu_memset(dba_state, 0, sizeof(dba_state));
	npu_fttr_base = NPU_FTTR_BASE;
	npu_printf("npu_fttr_base=%x\n", npu_fttr_base);
}

static void dba_main_loop(void)
{
	/* DBA processing loop */
	while (1) {
		if (npu_fttr_base == 0) {
			npu_printf("error, npu_fttr_base is not init\n");
			return;
		}
		/* process DBA events */
		break;
	}
}

static int dba_mail_handler(u32 base, u32 cnt)
{
	npu_printf("dba_mail_handler\n");
	return 1;
}

static void dba_timer_handler(int src)
{
	/* DBA timer interrupt handler */
}

#endif /* HAS_DBA */

/* ================================================================
 * TR-471 test infrastructure (AN7581 + WiFi)
 * ================================================================ */

#ifdef HAS_TR471

static void tr471_main_init(void)
{
	npu_printf("tr471_main_init\n");
}

#endif

/* ================================================================
 * Per-core main functions
 * ================================================================ */

static void core0_main(void)
{
	/* init TDMA and SRAM alloc, then buffer manager */
	tdma_init();

#ifdef WIFI_KITE
	tdma_bmgr_init();
#else
	buf_mgr_init();
#endif

	/* WiFi bridge init */
	core0_wifi_init_wrapper();

	/* register debug counter ISR on PLIC source 59 */
	plic_register_isr(59, dbg_cnt_isr);

	npu_printf("%s\n", "core0_main");
}

static void core1_main(void)
{
	npu_printf("%s\n", "core1_main");
#ifdef HAS_WIFI
	/* WiFi slow path handler loop (sub_840135A8) */
	wifi_bridge_loop();
#endif
}

#if MAX_CORE_NUM > 2
static void core2_main(void)
{
	npu_printf("%s\n", "core2_main");
	/* register timer ISR on PLIC source 18 */
	plic_register_isr(18, timer_isr);
}

static void core3_main(void)
{
	npu_printf("%s\n", "core3_main");

	chip_id_query();
	usb_powerdown();
	core3_wifi_init_wrapper();
}

static void core4_main(void)
{
	npu_printf("%s\n", "core4_main");
#ifdef HAS_WIFI
	/* WiFi handler - variant-specific */
#endif
}

static void core5_main(void)
{
#if defined(HAS_DBA)
	npu_printf("%s: start\n", "core5_dba_main");

	npu_memset(dba_state, 0, sizeof(dba_state));
	get_hartid();
	npu_fttr_base = NPU_FTTR_BASE;
	npu_printf("npu_fttr_base=%x\n", npu_fttr_base);

	/* register DBA mailbox ISR */
	plic_register_isr(8 + 5, mbox_isr);

	dba_main_loop();
#elif defined(AN7581) && defined(HAS_WIFI)
	npu_printf("%s\n", "core5_main");
	/* AN7581 core5: WiFi handler */
#else
	npu_printf("%s\n", "core5_main");
#endif
}
#endif /* MAX_CORE_NUM > 2 */

#if MAX_CORE_NUM > 6
static void core6_main(void)
{
	npu_printf("%s\n", "core6_main");
#ifdef HAS_WIFI
	/* AN7581 core6: WiFi handler */
#endif
}

static void core7_main(void)
{
#ifdef HAS_TUNNEL
	npu_printf("%s for npu tunnel offload\n", "core7_main");

	/* register tunnel PLIC interrupt 15 */
	plic_register_isr(15, mbox_isr);

	tunnel_init();

#ifdef HAS_TR471
	tr471_main_init();
#endif

	/* tunnel processing loop */
	while (1) {
		tunnel_process();
	}
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
		"bgez a0, 1f\n"
		/* interrupt: claim from PLIC, dispatch ISR */
		"lui a0, 0x0C200\n"
		"lw a0, 4(a0)\n"
		"addi a0, a0, -1\n"
		"call call_isr_by_src\n"
		"j 2f\n"
		"1:\n"
		"call call_isr_by_src\n"
		"2:\n"
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

void npu_init(void)
{
	u32 hart = get_hartid();

	/* hart 0 clears sync flag; others wait */
	if (hart == 0)
		core_sync_flag = 0;
	else if (core_sync_flag == 0) {
		while (core_sync_flag == 0)
			;
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
		mib12_snapshot = REG32(NPU_MIB12);
		REG32(NPU_SCU_RSTCTRL1) = 0;
		REG32(NPU_THREAD_ENABLE) = 1;
		REG32(NPU_MIB0) = ALL_FF;
		plic_enable_wrapper(22);
		timer_init(0, 1, 10);
		mailbox_init();
	}

	/* hart 0 signals others to proceed */
	if (hart == 0)
		core_sync_flag = 1;

	/* boot signature */
	REG32(NPU_MIB31) = INIT_COMPLETE;
}
