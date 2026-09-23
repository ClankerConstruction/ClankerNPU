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

void l4s_ecn_process(u32 port)
{
	volatile u32 *ring_stat = (volatile u32 *)(0x1EC12050 + 4 * port);
	u32 desc_remaining;
	u32 tx_credits;
	u32 ring_base, desc_out, doorbell, trigger;
	u32 port_cmd, port_idx;
	u32 desc_ptr, uncached;
	u32 hdr_field, pkt_len, desc_w0, ecn_byte;
	u32 band_sel, band_idx;
	u32 pkt_data, total;
	u32 qthresh;

	desc_remaining = (u8)(*ring_stat);
	if (desc_remaining == 0 || !tunnel_ecn_enabled)
		return;

	ring_base = 0x1EC12080 + 16 * port;
	desc_out  = 0x1EC12100 + 32 * port;
	doorbell  = 0x1EC12104 + 32 * port;
	trigger   = 0x1EC12108 + 32 * port;
	port_cmd  = port | 0xC0000000;
	port_idx  = 16 * port;
	tx_credits = 0;

	while (1) {
		desc_ptr = REG32(ring_base);
		uncached = desc_ptr | 0x20000000;

		hdr_field = *(volatile u16 *)(uncached + 0x12);
		pkt_len   = *(volatile u32 *)(uncached + 0x04) & 0x3FFFF;
		desc_w0   = *(volatile u32 *)(uncached);
		ecn_byte  = *(volatile u8  *)(uncached + 0x14);

		npu_memset((void *)(uncached + 4), 0, 28);

		band_sel = hdr_field >> 5;
		band_idx = hdr_field >> 11;

		*(volatile u32 *)(uncached + 0x10) =
			(pkt_len << 14) | 0x3800 | (band_idx << 3);
		*(volatile u32 *)(uncached + 0x14) =
			0x7F0007FF
			| ((u32)((hdr_field & 0x200) != 0) << 14)
			| ((hdr_field & 0x1F) << 15)
			| ((band_sel & 0xF) << 20);

		*(volatile u32 *)(uncached + 0x10) =
			(pkt_len << 14) | 0x3800 | (band_idx << 3) |
			(l4s_qid & 7);
		*(volatile u32 *)(uncached) = port_idx;

		pkt_data = ((u16)desc_w0 + 32) << 16;
		--desc_remaining;

		if (ecn_byte != 0)
			ecn_byte |= 0x40;

		total = ++l4s_pkt_count;
		*(volatile u8 *)(uncached + 0x1B) = ecn_byte;

		if (total % 10 == 0)
			goto query_hw;

		if (l4s_qlen >= l4s_cached_qthresh)
			goto submit;
		++l4s_skip_count;
		goto submit;

query_hw:
		if ((band_sel & 7) == 1) {
			REG32(0x1FB55100) = (band_idx << 3) |
					    l4s_qid | 0x1000000;
			qthresh = (u16)REG32(0x1FB55104);
		} else if ((band_sel & 0xF) == 2) {
			REG32(0x1FB57100) = (band_idx << 3) |
					    l4s_qid | 0x1000000;
			qthresh = (u16)REG32(0x1FB57104);
		} else {
			qthresh = 0;
		}
		l4s_cached_qthresh = qthresh;

		if (l4s_qlen < qthresh)
			++l4s_skip_count;

submit:
		if (tx_credits == 0) {
			while (1) {
				tx_credits = ((volatile u8 *)ring_stat)[1];
				if (tx_credits != 0)
					break;
			}
		}

		REG32(doorbell) = pkt_data;
		--tx_credits;
		REG32(trigger) = port_cmd;
		REG32(desc_out) = desc_ptr;

		if (desc_remaining == 0)
			return;
	}
}

#endif /* HAS_TUNNEL */
