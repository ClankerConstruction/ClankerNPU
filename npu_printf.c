#include "npu_internal.h"

/* ================================================================
 * vsprintf (minimal implementation)
 * ================================================================ */

static int npu_vsprintf(char *buf, const char *fmt, u32 *args)
{
	char *p = buf;
	const char *f = fmt;
	int arg_idx = 0;
	char tmp[24];
	int i, len;

	while (*f) {
		if (*f != '%') {
			*p++ = *f++;
			continue;
		}
		f++;

		int pad_zero = 0, left = 0, width = 0, is_ll = 0;

		if (*f == '-') { left = 1; f++; }
		if (*f == '0') { pad_zero = 1; f++; }
		while (*f >= '0' && *f <= '9')
			width = width * 10 + (*f++ - '0');
		if (*f == 'l') {
			f++;
			if (*f == 'l') { is_ll = 1; f++; }
		} else if (*f == 'z') {
			f++;
		}

		switch (*f) {
		case 'd': {
			if (is_ll) {
				u64 v64 = args[arg_idx] |
					  ((u64)args[arg_idx + 1] << 32);
				arg_idx += 2;
				s64 sv = (s64)v64;
				int neg = 0;
				if (sv < 0) { neg = 1; v64 = (u64)(-sv); }
				len = 0;
				do {
					tmp[len++] = '0' + (u32)(v64 % 10);
					v64 /= 10;
				} while (v64);
				if (neg) tmp[len++] = '-';
			} else {
				s32 v = (s32)args[arg_idx++];
				int neg = 0;
				u32 uv;
				if (v < 0) { neg = 1; uv = (u32)(-v); }
				else uv = (u32)v;
				len = 0;
				do {
					tmp[len++] = '0' + (uv % 10);
					uv /= 10;
				} while (uv);
				if (neg) tmp[len++] = '-';
			}
			if (!left)
				for (i = 0; i < width - len; i++)
					*p++ = pad_zero ? '0' : ' ';
			for (i = len - 1; i >= 0; i--)
				*p++ = tmp[i];
			if (left)
				for (i = 0; i < width - len; i++)
					*p++ = ' ';
			break;
		}
		case 'u': {
			if (is_ll) {
				u64 v64 = args[arg_idx] |
					  ((u64)args[arg_idx + 1] << 32);
				arg_idx += 2;
				len = 0;
				do {
					tmp[len++] = '0' + (u32)(v64 % 10);
					v64 /= 10;
				} while (v64);
			} else {
				u32 v = args[arg_idx++];
				len = 0;
				do {
					tmp[len++] = '0' + (v % 10);
					v /= 10;
				} while (v);
			}
			if (!left)
				for (i = 0; i < width - len; i++)
					*p++ = pad_zero ? '0' : ' ';
			for (i = len - 1; i >= 0; i--)
				*p++ = tmp[i];
			if (left)
				for (i = 0; i < width - len; i++)
					*p++ = ' ';
			break;
		}
		case 'x':
		case 'X': {
			const char *hex = (*f == 'X') ?
				"0123456789ABCDEF" : "0123456789abcdef";
			if (is_ll) {
				u64 v64 = args[arg_idx] |
					  ((u64)args[arg_idx + 1] << 32);
				arg_idx += 2;
				len = 0;
				do {
					tmp[len++] = hex[v64 & 0xF];
					v64 >>= 4;
				} while (v64);
			} else {
				u32 v = args[arg_idx++];
				len = 0;
				do {
					tmp[len++] = hex[v & 0xF];
					v >>= 4;
				} while (v);
			}
			if (!left)
				for (i = 0; i < width - len; i++)
					*p++ = pad_zero ? '0' : ' ';
			for (i = len - 1; i >= 0; i--)
				*p++ = tmp[i];
			if (left)
				for (i = 0; i < width - len; i++)
					*p++ = ' ';
			break;
		}
		case 's': {
			const char *s = (const char *)args[arg_idx++];
			int slen;

			if (!s) s = "(null)";
			slen = 0;
			while (s[slen]) slen++;
			if (!left)
				for (i = 0; i < width - slen; i++)
					*p++ = ' ';
			while (*s)
				*p++ = *s++;
			if (left)
				for (i = 0; i < width - slen; i++)
					*p++ = ' ';
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

/* The host console shares this UART. Give up on a byte rather than spin
 * forever holding the printf mutex, which would stall every other core
 * on the acquire. */
static void uart_putc(char c)
{
	u32 spin = 200000;

	while (!(REG32(UART_TX_STATUS) & UART_TX_READY_BIT)) {
		if (--spin == 0)
			return;
	}
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

static const char *uart_debug_cmd(char c)
{
	u32 idx = uart_cmd_idx;

	if (idx > 31) {
		uart_cmd_idx = 0;
		return "Error: exceed max_char_num:%d ... reset char_idx\n\n";
	}

	uart_cmd_buf[idx] = (u8)c;
	if (c != '\r') {
		uart_cmd_idx = idx + 1;
		return NULL;
	}

	if (uart_cmd_buf[0] != 'r' && uart_cmd_buf[0] != 'w') {
		goto bad_cmd;
	}
	if (uart_cmd_buf[0] == 'r' && uart_cmd_buf[1] != 'd')
		goto bad_cmd;
	if (uart_cmd_buf[0] == 'w' && uart_cmd_buf[1] != 't')
		goto bad_cmd;

	{
		u32 is_write = (uart_cmd_buf[0] == 'w');
		u32 addr = 0, val = 0;
		u32 pos = 3, digit;
		u8 ch;

		for (; pos <= 10; pos++) {
			ch = uart_cmd_buf[pos];
			if (ch >= '0' && ch <= '9')
				digit = ch - '0';
			else if (ch >= 'A' && ch <= 'F')
				digit = ch - 'A' + 10;
			else if (ch >= 'a' && ch <= 'f')
				digit = ch - 'a' + 10;
			else
				break;
			addr = (addr << 4) | digit;
		}

		if (is_write) {
			if (pos == 11)
				pos = 12;
			for (; pos <= 19; pos++) {
				ch = uart_cmd_buf[pos];
				if (ch >= '0' && ch <= '9')
					digit = ch - '0';
				else if (ch >= 'A' && ch <= 'F')
					digit = ch - 'A' + 10;
				else if (ch >= 'a' && ch <= 'f')
					digit = ch - 'a' + 10;
				else
					break;
				val = (val << 4) | digit;
			}
			*(volatile u32 *)addr = val;
			uart_cmd_idx = 0;
			return "wt(0x%x)==0x%x\n";
		}

		uart_cmd_idx = 0;
		return "rd(0x%x)==0x%x\n";
	}

bad_cmd:
	uart_cmd_buf[idx] = 0;
	uart_cmd_idx = 0;
	return "Correct Cmd: 'rd regAddr' or 'wt regAddr val'\n";
}

int npu_printf(const char *fmt, ...)
{
	int len;
	u32 mie;
	__builtin_va_list ap;

	/* check if printing is suppressed by host */
	if (REG32(NPU_MIB(21)) != 0)
		return 0;

	mie = irq_save();
	hw_mutex_lock_pri(printf_mutex_desc);

	/* format into buffer */
	__builtin_va_start(ap, fmt);
	len = npu_vsprintf(printf_buf, fmt, (u32 *)ap);
	__builtin_va_end(ap);

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
	irq_restore(mie);

	return len;
}

void boot_uart_init(void)
{
	REG32(BOOT_UART_FCR) = 15;
	REG32(BOOT_UART_MCR) = 0;
	REG32(BOOT_UART_SCR) = 0;
	REG32(BOOT_UART_IER) = 1;
	REG32(BOOT_UART_LCR) = 0x80;		/* DLAB: DLL/DLM visible */
	REG32(BOOT_UART_FRACDIV) = 0xEA00FDE8;
	REG32(BOOT_UART_DLL) = 1;
	REG32(BOOT_UART_DLM) = 0;
	REG32(BOOT_UART_LCR) = 3;		/* 8N1 */

	printf_mutex_desc[0] = 15;
	printf_mutex_desc[1] = 0;
	boot_printf("%s done via base:0x%x\n\n\n", "npu_uart_init",
		    BOOT_UART_BASE);
}

/* boot printf (uses boot UART at 0x1EC10000) */
int boot_printf(const char *fmt, ...)
{
	int len;
	__builtin_va_list ap;
	char *p;

	hw_mutex_lock_pri(printf_mutex_desc);

	__builtin_va_start(ap, fmt);
	len = npu_vsprintf(printf_buf, fmt, (u32 *)ap);
	__builtin_va_end(ap);

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

