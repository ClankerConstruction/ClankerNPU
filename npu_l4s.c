/*
 * AN75XX NPU firmware - L4S ECN marking
 *
 * The tunnel offload loop calls l4s_ecn_process() for each bridge port.
 * It reads the queue length the QoS block reports and sets the ECN
 * congestion experienced bit when the queue is over its threshold.
 */

#include "npu_internal.h"


#ifdef HAS_TUNNEL

int l4s_set_config(u32 cmd, u32 arg)
{
	switch (cmd) {
	case 0:
		tunnel_ecn_enabled = 0;
		l4s_debug_enable = 0;
		npu_printf("L4S_SET_DISABLE!!!\n");
		break;
	case 1:
		tunnel_ecn_enabled = 1;
		npu_printf("L4S_SET_ENABLE!!!\n");
		break;
	case 2:
		tunnel_ecn_enabled = 1;
		l4s_debug_enable = arg;
		npu_printf("L4S_SET_DEBUG_%s!!!\n",
			   arg ? "ENABLE" : "DISABLE");
		break;
	case 3:
		l4s_qid = arg;
		npu_printf("L4S_SET_QID: %u\n", arg);
		break;
	default:
		npu_printf("L4S_SET_CMD_ERROR!!!\n");
		break;
	}
	return 0;
}

static u16 l4s_be16(u16 v)
{
	return (v >> 8) | (v << 8);
}

/* set CE in the IPv4 or IPv6 header past any VLAN tags or PPPoE */
static void l4s_ecn_mark(u8 *pkt)
{
	u8 *p;
	u16 type;

	if (!pkt)
		return;
	p = pkt + 12;
	for (;;) {
		type = *(u16 *)p;
		if ((type & ~0x10) != l4s_be16(0x8100) &&
		    type != l4s_be16(0x88A8) && type != l4s_be16(0x884C))
			break;
		p += 4;
	}

	if (type == l4s_be16(0x8864)) {
		type = *(u16 *)(p + 8);
		if (type == l4s_be16(0x0021))
			p[11] |= 3;
		else if (type == l4s_be16(0x0057))
			p[11] |= 0x30;
		return;
	}
	type = l4s_be16(type);
	if (type == 0x0800)
		p[3] |= 3;
	else if (type == 0x86DD)
		p[3] |= 0x30;
}

/* Forward what the host queued on this bridge channel, marking CE
 * while the QoS queue is over l4s_qlen_thresh. */
void l4s_ecn_process(u32 port)
{
	u32 remaining = (u8)REG32(0x1EC12050 + 4 * port);
	u32 credits = 0;
	u32 phys, info, len, w0, w4, band, grp, q;
	u8 ecn;
	volatile u32 *d;

	if (remaining == 0 || !tunnel_ecn_enabled) {
		delay_us(1);
		return;
	}

	do {
		phys = REG32(0x1EC12080 + 16 * port);
		d = (volatile u32 *)(phys | 0x20000000);
		len = d[1] & 0x3FFFF;
		info = *(volatile u16 *)((u32)d + 18);
		w0 = d[0];
		ecn = *(volatile u8 *)((u32)d + 20);
		npu_memset((void *)(d + 1), 0, 28);

		band = (info >> 5) & 0xFF;
		grp = info >> 11;
		w4 = 0x3800 | (len & 0xFFFF) << 14 | grp << 3;
		d[4] = w4;
		d[5] = 0x7F0007FF | ((info >> 9) & 1) << 14 |
		       (info & 31) << 15 | (band & 15) << 20;
		d[0] = port << 4;
		d[4] = w4 | (l4s_qid & 7);
		len = (w0 & 0xFFFF) + 32;
		remaining--;

		if (ecn)
			ecn |= 0x40;
		*(volatile u8 *)((u32)d + 27) = ecn;
		q = ++l4s_pkt_count;

		/* read the queue length every 10 packets and at each
		 * 100-tick window, logging once per window */
		if (timer_raw_tick % 100 != 0) {
			l4s_log_phase = 0;
		} else if (!l4s_log_phase) {
			if (l4s_debug_enable)
				npu_printf("CORE%d: ECN Tag: pkts(%d %d), qlen %d, qid %u\n",
					   port, q, l4s_mark_count,
					   l4s_qlen_thresh, l4s_qid);
			l4s_log_phase = 1;
			l4s_pkt_count = 0;
			l4s_mark_count = 0;
			q = 0;
		}
		if (q % 10 == 0) {
			u32 sel = grp << 3 | l4s_qid | 0x1000000;

			if ((band & 7) == 1) {
				REG32(0x1FB55100) = sel;
				l4s_qlen = REG32(0x1FB55104) & 0xFFFF;
			} else if ((band & 15) == 2) {
				REG32(0x1FB57100) = sel;
				l4s_qlen = REG32(0x1FB57104) & 0xFFFF;
			} else {
				l4s_qlen = 0;
			}
		}
		if (l4s_qlen > l4s_qlen_thresh) {
			l4s_ecn_mark((u8 *)d + 32);
			l4s_mark_count++;
		}

		while (credits == 0) {
			credits = (REG32(0x1EC12050 + 4 * port) >> 8) & 0xFF;
			if (credits == 0)
				delay_us(1);
		}
		REG32(0x1EC12104 + 32 * port) = len << 16;
		REG32(0x1EC12108 + 32 * port) = port | 0xC0000000;
		credits--;
		REG32(0x1EC12100 + 32 * port) = phys & 0x1FFFFFFF;
	} while (remaining != 0);
}

#endif /* HAS_TUNNEL */
