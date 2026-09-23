/*
 * AN75XX NPU firmware - mailbox
 *
 * The host reaches the firmware through the mailbox block at 0x1EC0C000.
 * Mailbox n+1 interrupts core n. Queue 8 carries notifications from the
 * NPU to the host.
 */

#include "npu_internal.h"


/* ================================================================
 * Mailbox communication
 *
 * 80 bytes per core in dispatch table:
 *   [0..63]  = raw data DWORDs (16 slots, indexed by func_id)
 *   [32..63] = raw data WORDs (overlaps with above)
 *   [48..79] = callback function pointers (8 slots)
 * ================================================================ */

/* mailbox dispatch table: 6 cores x 80 bytes */
u8 mbox_dispatch[MAX_CORE_NUM][80];


void mbox_isr(int src)
{
	u32 mbox_idx = (u8)(src - 8);
	u32 core = get_hartid();
	u32 rptr, base_ptr, max_cnt, func_idx, ret = 0;
	mbox_handler_t handler;

	if (mbox_idx != core) {
		npu_printf("Error: core_id:%d != mbox_idx:%d\n", core, mbox_idx);
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

	/* read queue registers; length and arg are 16-bit */
	base_ptr = REG32(MBQ_CTRL0(mbox_idx));
	max_cnt = REG32(MBQ_CTRL1(mbox_idx)) & 0xFFFF;
	rptr = REG32(MBQ_CTRL3(mbox_idx)) & 0xFFFF;

	/* set response status if not already set */
	if (!(rptr & 1)) {
		rptr = ((rptr & 0xFF00u) | (rptr & 0xE1) | 0x6) & 0xFFFF;
		REG32(MBQ_CTRL3(mbox_idx)) = rptr;
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
			ret = (u32)handler(base_ptr, max_cnt);
			rptr = (rptr & 0xFFFFFFE3u) | ((ret & 7) << 2);
		}

		ndbg->hart[core].mails++;
		ndbg->hart[core].last_mail = func_idx << 24 |
			(base_ptr & 0xFFFF) << 8 | (ret & 0xFF);
		NDBG_TRACE(NDBG_MBOX, mbox_idx << 8 | func_idx, base_ptr, ret);

		/* blocking: set done; else return result via queue 8 */
		if (rptr & 1)
			REG32(MBQ_CTRL3(mbox_idx)) = (rptr | 2) & 0xFFFF;
		else
			mbox_notify_host(core, func_idx, ret);
	}
}

void mailbox_init(void)
{
	u32 i;
	u32 *callbacks;

	/* route mailbox n to core n-1 */
	for (i = 0; i < MAX_CORE_NUM; i++) {
		REG32(MBOX_INT_MASK(i + 1)) = (1u << i);
		plic_register_isr(8 + i, mbox_isr);
	}

	REG32(MBOX_INT_MASK0) = 256;
#if defined(AN7581)
	mbox_notify_mutex[0] = 30;
#else
	mbox_notify_mutex[0] = 14;
#endif
	mbox_notify_mutex[1] = 0;

	/* zero all dispatch tables */
	for (i = 0; i < MAX_CORE_NUM; i++)
		npu_memset(mbox_dispatch[i], 0, 80);

	/* register handlers into core 0's callback slots */
	callbacks = (u32 *)&mbox_dispatch[0][48];
#ifdef HAS_WIFI
	callbacks[0] = (u32)(void *)wifi_mail_dispatch;
#endif
	callbacks[1] = (u32)(void *)tunnel_mail_dispatch;

#ifdef HAS_TR471
#ifdef WIFI_KITE
	callbacks[4] = (u32)(void *)kite_wifi_config;
#elif defined(WIFI_EAGLE)
	callbacks[4] = (u32)(void *)eagle_wifi_config;
#endif
#endif

#ifdef HAS_TUNNEL
	callbacks[5] = (u32)(void *)hwnat_mail_dispatch;
#endif

#ifdef HAS_DBA
	{
		u32 *dba_cb = (u32 *)&mbox_dispatch[5][48];
		dba_cb[3] = (u32)(void *)dba_mail_handler;
	}
#endif
}

/* notify host via mailbox queue 8 */
int mbox_notify_host(u32 core_id, u32 func_id, u32 len)
{
	u32 timeout = 30;
	u32 sts;

	hw_mutex_lock_pri(mbox_notify_mutex);

	REG32(MBQ_CTRL0(MBQ_NOTIFY)) = core_id & 0xF;
	REG32(MBQ_CTRL1(MBQ_NOTIFY)) = len & 0xFFFF;
	REG32(MBQ_CTRL3(MBQ_NOTIFY)) = (func_id & 0xF) << 11;
	REG32(MBQ_CTRL2(MBQ_NOTIFY)) = REG32(MBQ_CTRL2(MBQ_NOTIFY)) + 1;

	sts = REG32(MBQ_CTRL3(MBQ_NOTIFY)) & 0xFFFF;
	while (!(sts & 2) && timeout--) {
		delay_ms_mcycle(1);
		sts = REG32(MBQ_CTRL3(MBQ_NOTIFY)) & 0xFFFF;
	}

	if (!(sts & 2))
		npu_printf("Error: %s timeout for core_id:%d, func_id:%d\n",
			   "npuMbox_notify_host", core_id, func_id);

	hw_mutex_unlock_pri(mbox_notify_mutex);
	return (sts & 2) ? (sts >> 2) & 7 : 0;
}
