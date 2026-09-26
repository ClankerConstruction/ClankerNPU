/*
 * AN75XX NPU firmware - SRAM buffer manager
 *
 * A bump allocator over the SRAM at 0x3E800000, keyed by address type.
 * tdma_init() zeroes the SRAM and restarts the allocator, so nothing may
 * claim a block before that call.
 */

#include "npu_internal.h"


/* ================================================================
 * SRAM buffer management
 *
 * SRAM at 0x3E800000, bump-allocated with alignment.
 * Size and max alloc entries vary by SoC.
 * ================================================================ */

#define SRAM_BASE         0x3E800000
#if defined(AN7552)
#define SRAM_SIZE         0x40000
#define SRAM_MAX_ENTRIES  50
#elif defined(AN7581)
#define SRAM_SIZE         0x78000
#define SRAM_MAX_ENTRIES  100
#else /* AN7583 */
#define SRAM_SIZE         0x80000
#define SRAM_MAX_ENTRIES  100
#endif
#define SRAM_END          (SRAM_BASE + SRAM_SIZE - 2)
#define SRAM_ERROR_ADDR   (SRAM_BASE + SRAM_SIZE)

u32 sram_alloc_offset;
static u32 sram_alloc_count;
u16 sram_alloc_table[SRAM_MAX_ENTRIES * 4];
/* entries hold the address as a u32 over two u16 slots */
typedef u32 __attribute__((may_alias)) u32_alias;

/* SRAM region type descriptors: {u16 addr_type, u8 align_class, u8 pad, u32 size} */
struct sram_region_desc {
	u16 addr_type;
	u8 align;
	u8 pad;
	u32 size;
};

static u32 sram_buf_alloc_impl(u16 addr_type, u32 size_class)
{
	u32 i;
	u16 *entry;
	u8 *base;
	u32 offset;

	hw_mutex_lock(sram_buf_mutex);

	if (sram_alloc_offset >= SRAM_SIZE && sram_alloc_count >= SRAM_MAX_ENTRIES) {
		hw_mutex_unlock(sram_buf_mutex);
		npu_printf("sram is over the max!!current para:AddrType=%d,idx=%d,tmp_restore_index=%d\n",
			   addr_type, sram_alloc_offset, sram_alloc_count);
		return SRAM_ERROR_ADDR;
	}

	/* check if already allocated */
	if (sram_alloc_count > 0) {
		for (i = 0; i < sram_alloc_count; i++) {
			if (sram_alloc_table[i * 4] == addr_type) {
				u32 existing = *(u32_alias *)&sram_alloc_table[i * 4 + 2];

				npu_printf("already exist!!AddrType=%d,npu_init_sram_addr=%x\n",
					   addr_type, existing);
				hw_mutex_unlock(sram_buf_mutex);
				return existing;
			}
		}
	}

	/* bump-allocate with alignment */
	offset = sram_alloc_offset;
	base = (u8 *)SRAM_BASE;
	if (offset != 0) {
		base += offset;
		/* align to 16 or 32 bytes depending on type */
		if ((u32)base & 0x1F)
			base = (u8 *)(((u32)base + 0x1F) & ~0x1F);
		if ((u32)base > SRAM_END) {
			npu_printf("alloc fail!!current para:AddrType=%d,idx=%x,tmp_restore_index=%d,npu_init_sram_addr=%x\n",
				   addr_type, sram_alloc_offset, sram_alloc_count, (u32)base);
			hw_mutex_unlock(sram_buf_mutex);
			return SRAM_ERROR_ADDR;
		}
	} else {
		base = (u8 *)SRAM_BASE;
	}

	/* record entry */
	entry = &sram_alloc_table[sram_alloc_count * 4];
	entry[0] = addr_type;
	*(u32_alias *)(entry + 2) = (u32)base;
	sram_alloc_offset = (u32)base - SRAM_BASE + size_class;
	sram_alloc_count++;
	NDBG_CNT(NC_SRAM_ALLOCS);
	NDBG_SET(NC_SRAM_USED, sram_alloc_offset);
	NDBG_TRACE(NDBG_SRAM, addr_type, (u32)base, size_class);

	hw_mutex_unlock(sram_buf_mutex);
	return (u32)base;
}

/* Reserved size per address type. Two classes: types up to 128 are the
 * WiFi rings and tables, 129 and above the bridge and tunnel buffers. */
struct sram_size_ent {
	u16 addr_type;
	u32 size;
};

#ifdef WIFI_KITE
/* the kite tables, one per SoC */
#if defined(AN7552)
static const struct sram_size_ent sram_size_lo[] = {
	{   1, 0x0C000 }, {   2, 0x02000 }, {   3, 0x01000 }, {   9, 0x002A8 },
	{  10, 0x002A8 }, {  11, 0x00078 }, {  14, 0x00600 }, {  15, 0x00600 },
	{   6, 0x00C00 }, {   4, 0x11940 }, {   7, 0x08340 }, {   8, 0x08340 },
	{  12, 0x00FA0 }, {  13, 0x02710 }, {  19, 0x00400 }, {  20, 0x00400 },
};

static const struct sram_size_ent sram_size_hi[] = {
	{ 138, 0x02BC0 }, { 129, 0x10FFF }, { 132, 0x04000 }, { 134, 0x01000 },
	{ 137, 0x00004 },
};
#else
static const struct sram_size_ent sram_size_lo[] = {
	{   1, 0x0C040 }, {   2, 0x02020 }, {   3, 0x01020 }, {   9, 0x003E8 },
	{  10, 0x003E8 }, {  11, 0x00078 }, {  14, 0x00600 }, {  15, 0x00600 },
	{   6, 0x00C04 }, {   4, 0x11940 }, {   7, 0x08340 }, {   8, 0x08340 },
	{  12, 0x00FA0 }, {  13, 0x02710 }, {  19, 0x00400 }, {  20, 0x00400 },
	{  21, 0x06400 },
};

#if defined(AN7581)
static const struct sram_size_ent sram_size_hi[] = {
	{ 138, 0x02BC0 }, { 129, 0x13FFF }, { 132, 0x11000 }, { 133, 0x01080 },
	{ 130, 0x00004 }, { 137, 0x00004 },
};
#else
static const struct sram_size_ent sram_size_hi[] = {
	{ 138, 0x02BC0 }, { 129, 0x13FFF }, { 132, 0x04000 }, { 133, 0x00080 },
	{ 134, 0x01000 }, { 136, 0x08010 }, { 130, 0x00004 }, { 137, 0x00004 },
};
#endif
#endif
#elif defined(AN7552)
/* AN7552 eagle */
static const struct sram_size_ent sram_size_lo[] = {
	{   1, 0x12080 }, {   2, 0x01800 }, {   3, 0x01800 }, {   9, 0x002A8 },
	{  10, 0x002A8 }, {  11, 0x00078 }, {  14, 0x00600 }, {  15, 0x00600 },
	{  22, 0x01008 }, {  25, 0x00090 }, {  26, 0x00010 }, {  30, 0x00100 },
	{  31, 0x02BC0 },
};

static const struct sram_size_ent sram_size_hi[] = {
	{ 138, 0x05780 }, { 129, 0x10FFF }, { 132, 0x04000 }, { 134, 0x01000 },
	{ 137, 0x00004 },
};
#else
static const struct sram_size_ent sram_size_lo[] = {
	{   1, 0x220C0 }, {   2, 0x01818 }, {   3, 0x01818 }, {   9, 0x003E8 },
	{  10, 0x003E8 }, {  11, 0x00078 }, {  14, 0x00600 }, {  15, 0x00600 },
	{  22, 0x01008 }, {  25, 0x00040 }, {  26, 0x00010 }, {  30, 0x00100 },
	{  18, 0x06800 }, {  16, 0x02020 }, {  17, 0x02020 }, {  28, 0x06800 },
	{  29, 0x01000 }, {  23, 0x00800 },
};

static const struct sram_size_ent sram_size_hi[] = {
	{ 138, 0x06000 }, { 129, 0x13FFF }, { 132, 0x04000 }, { 133, 0x10000 },
	{ 134, 0x01000 }, { 136, 0x08010 }, { 130, 0x00004 }, { 137, 0x00004 },
};
#endif

/* Types missing from the tables above, sized from the loops that fill
 * them; any other type gets a bounded default. */
#define SRAM_DEFAULT_SIZE  0x2000

static const struct sram_size_ent sram_size_ext[] = {
	{  12, 0x01000 },	/* reorder primary index pool, 2000 u16 */
	{  13, 0x02800 },	/* reorder secondary index pool, 5000 u16 */
#ifdef HAS_EAGLE_STA_QLIMIT
	{  41, 0x0A800 },	/* station per tx token, per station counts */
#endif
};

static u32 sram_type_size(u32 addr_type)
{
	const struct sram_size_ent *t;
	u32 n, i;

	if (addr_type <= 128) {
		t = sram_size_lo;
		n = sizeof(sram_size_lo) / sizeof(sram_size_lo[0]);
	} else {
		t = sram_size_hi;
		n = sizeof(sram_size_hi) / sizeof(sram_size_hi[0]);
	}
	for (i = 0; i < n; i++) {
		if (t[i].addr_type == addr_type)
			return t[i].size;
	}

	n = sizeof(sram_size_ext) / sizeof(sram_size_ext[0]);
	for (i = 0; i < n; i++) {
		if (sram_size_ext[i].addr_type == addr_type)
			return sram_size_ext[i].size;
	}

	npu_printf("AddrType=%d is not in the size table, reserving 0x%x\n",
		   addr_type, SRAM_DEFAULT_SIZE);
	return SRAM_DEFAULT_SIZE;
}

u32 sram_buf_alloc(u32 addr_type)
{
	u32 size, result;

	if (addr_type == 0 || addr_type > 255)
		return 0;

	size = sram_type_size(addr_type);
	result = sram_buf_alloc_impl((u16)addr_type, size);
	if (result == SRAM_ERROR_ADDR)
		return 0;
	return result;
}

void sram_buf_init(void)
{
	npu_memset((void *)SRAM_BASE, 0, SRAM_SIZE);
	npu_memset(sram_alloc_table, 0, sizeof(sram_alloc_table));
	sram_buf_mutex[0] = 18;
	sram_buf_mutex[1] = 0;
	sram_alloc_offset = 0;
	sram_alloc_count = 0;
}

void sram_buf_dump(void)
{
	u32 i;
	u16 *entry = sram_alloc_table;

	npu_printf("base:%x,total size:%x,current max use size:%x\n",
		   SRAM_BASE, SRAM_SIZE, sram_alloc_offset);
	for (i = 0; i < SRAM_MAX_ENTRIES; i++) {
		u32 addr = *(u32_alias *)(entry + 2);

		if (addr == 0)
			break;
		npu_printf("Idx=%d,AddrType=%d,baseaddr=%x\n",
			   i, (u32)entry[0], addr);
		entry += 4;
	}
}
