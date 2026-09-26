/*
 * AN75XX NPU firmware - per-station queue limit
 *
 * Frames the NPU hands to the WiFi chip wait there outside any host
 * queue, bounded only by the shared tx token pool. Per station, count
 * them and drop against a hard limit and against a standing queue.
 */

#include "npu_internal.h"


#ifdef HAS_EAGLE_STA_QLIMIT

#define STA_Q_LIMIT		8192	/* frames; leaves 3000 tokens free */
#define STA_Q_TARGET		4096	/* frames; ~30 ms at 1.6 Gbit/s */
#define STA_Q_INTERVAL_MS	100

/* drop state per station, the sending hart only */
struct sta_q_aqm {
	u32 above;	/* when a queue above target becomes standing */
	u32 next;	/* next drop while dropping */
	u16 count;	/* drops in this dropping state */
	u8 last;	/* count when the last dropping state began */
	u8 flags;
};
#define AQM_ABOVE	1
#define AQM_DROPPING	2

static struct sta_q_aqm *sta_q_aqm;

/* Nothing in the chip: every token unmapped, every station at zero.
 * Runs when the tx done ring is set up, before the sending hart starts. */
void sta_q_init(void)
{
	u32 base = sram_buf_alloc(41), i;

	if (base == 0)
		return;
	wifi_sta_q.limit = STA_Q_LIMIT;
	wifi_sta_q.target = STA_Q_TARGET;
	wifi_sta_q.interval = STA_Q_INTERVAL_MS * 1000 * cpu_clock_get();
	sta_q_sent = (volatile u16 *)base + STA_Q_TOKENS;
	sta_q_done = sta_q_sent + STA_Q_STAS;
	sta_q_aqm = (struct sta_q_aqm *)(sta_q_done + STA_Q_STAS);
	for (i = 0; i < STA_Q_TOKENS; i++)
		((volatile u16 *)base)[i] = STA_Q_NONE;
	for (i = 0; i < STA_Q_STAS; i++) {
		sta_q_sent[i] = 0;
		sta_q_done[i] = 0;
	}
	npu_memset(sta_q_aqm, 0, STA_Q_STAS * sizeof(*sta_q_aqm));
	/* last: the hooks start counting once the map is there */
	sta_q_tok = (volatile u16 *)base;
}

/* 1: drop the frame. A station's frames in the chip that stay at or
 * above target for an interval start drops, spaced interval / sqrt(n)
 * as CoDel does (RFC 8289); dipping below target ends them. */
NPU_HOT int sta_q_decide(u32 sta, u32 now)
{
	struct sta_q_aqm *a = &sta_q_aqm[sta];
	volatile struct wifi_sta_q *cfg = &wifi_sta_q;
	u32 interval = cfg->interval, target = cfg->target, limit = cfg->limit;
	s16 d = (s16)(sta_q_sent[sta] - sta_q_done[sta]);
	/* done never passes sent, but a stale count must not read as full */
	u32 q = d > 0 ? (u32)d : 0, n;

	if (limit != 0 && q >= limit) {
		cfg->limit_drops++;
		return 1;
	}
	if (target == 0)
		return 0;
	if (q < target) {
		a->flags = 0;
		return 0;
	}
	if ((a->flags & AQM_ABOVE) == 0) {
		a->flags = AQM_ABOVE;
		a->above = now + interval;
		return 0;
	}
	if ((s32)(now - a->above) < 0)
		return 0;

	if (a->flags & AQM_DROPPING) {
		if ((s32)(now - a->next) < 0)
			return 0;
		if (a->count != 0xFFFF)
			a->count++;
		/* no frame for a while: space from now, not a late next */
		if ((s32)(now - a->next) > (s32)interval)
			a->next = now;
	} else {
		/* back into dropping soon after leaving: keep the rate */
		n = (u16)(a->count - a->last);
		a->count = (n > 1 && (s32)(now - a->next) < (s32)(16 * interval)) ?
			   n : 1;
		a->last = (a->count > 255) ? 255 : (u8)a->count;
		a->flags |= AQM_DROPPING;
		a->next = now;
	}
	a->next += interval / npu_isqrt(a->count);
	cfg->aqm_drops++;
	return 1;
}

#endif /* HAS_EAGLE_STA_QLIMIT */
