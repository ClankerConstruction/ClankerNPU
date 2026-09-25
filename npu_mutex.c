/*
 * AN75XX NPU firmware - hardware mutex
 *
 * The mutex block is at 0x1EC03000. The hardware arbitrates between
 * harts, so an acquire is a single write and one status read. Callers
 * never spin.
 *
 * A hart that acquires a mutex it already holds stalls the NPU bus.
 */

#include "npu_internal.h"


/* ================================================================
 * Hardware mutex
 * ================================================================ */

/* Acquire is a single try: the hardware arbitrates, callers do not spin.
 * Returns 0 when this hart owns the mutex, -1 otherwise. */
NPU_HOT int hw_mutex_lock(u32 *desc)
{
	u32 off = (desc[0] * 4) & HW_MUTEX_OFF_MASK;
	u32 hart = get_hartid();
	u32 sts;

	REG32(HW_MUTEX_ACQ(off)) = (hart << 8) | 0x40;
	sts = REG32(HW_MUTEX_STATUS(hart, off));
	if (!(sts & HW_MUTEX_HELD))
		return -1;
	return ((sts >> 8) & 0xFF) == hart ? 0 : -1;
}

NPU_HOT int hw_mutex_unlock(u32 *desc)
{
	u32 off = (desc[0] * 4) & HW_MUTEX_OFF_MASK;
	u32 hart = get_hartid();

	REG32(HW_MUTEX_REL(hart, off)) = (hart << 8);
	return 0;
}

int hw_mutex_lock_pri(u32 *desc)
{
	u32 off = (desc[0] * 4) & HW_MUTEX_OFF_MASK;
	u32 hart = get_hartid();
	u32 sts;

	if (desc[1] != 0)
		REG32(HW_MUTEX_PRI_ACQ(off)) = (hart << 8) | 0x10040;
	else
		REG32(HW_MUTEX_ACQ(off)) = (hart << 8) | 0x40;

	sts = REG32(HW_MUTEX_STATUS(hart, off));
	if (!(sts & HW_MUTEX_HELD))
		return -1;
	return ((sts >> 8) & 0xFF) == hart ? 0 : -1;
}

int hw_mutex_unlock_pri(u32 *desc)
{
	return hw_mutex_unlock(desc);
}
