/*
 * AN75XX NPU firmware - memory and string helpers
 *
 * The firmware links with -nostdlib, so it carries its own copies.
 */

#include "npu_internal.h"


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

u32 npu_strlen(const char *s)
{
	const char *p = s;

	while (*p)
		p++;
	return (u32)(p - s);
}

char get_core_char(void)
{
	u32 id = get_hartid();

	if (id <= 7)
		return (char)('0' + id);
	return 'X';
}

static u16 npu_htons(u16 x)
{
	return (u16)((x >> 8) | (x << 8));
}

static void wfi_idle(void)
{
	__asm__ volatile("wfi");
}
