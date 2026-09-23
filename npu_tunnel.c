/*
 * AN75XX NPU firmware - tunnel offload
 *
 * AN7581 and AN7583 only. The host hands the NPU a packet through the
 * bridge ring, the NPU rewrites the headers the UDF byte asks for, and
 * hands it back. npu_ppe.c programs the PPE that forwards the result.
 */

#include "npu_internal.h"


/* ================================================================
 * Tunnel funcId dispatch (callback[1] — MFUNC_TUNNEL)
 *
 * All variants register this at callback[1]. The table is pre-
 * initialized in .data; entries point to
 * individual tunnel handler functions. Tunnel messages are in
 * SRAM so base_ptr is accessed without DMA translation.
 * ================================================================ */

int tunnel_mail_dispatch(u32 base, u32 cnt)
{
	u32 func_id = *(volatile u32 *)base;

	(void)cnt;
	if (tunnel_func_table[func_id])
		return tunnel_func_table[func_id](base, cnt);
	return 0;
}


#ifdef HAS_TUNNEL

static u16 bswap16(u16 v)
{
	return (v >> 8) | (v << 8);
}

static u32 tunnel_sram_base(void)
{
	return npu_bridge_addr();
}

/* egress command word: first and last segment, segment type */
#define SEG_FIRST		0x80000000
#define SEG_LAST		0x40000000
#define SEG_TYPE(t)		((t) << 24)

/* rewrite the be16 at byte offset off of the segment on the way out */
#define PATCH(off, v)	(0xC0000000 | ((off) & 0x3FFF) << 16 | bswap16((u16)(v)))

/* one segment of len bytes at addr + off, with up to four patches */
static int tunnel_seg(u32 port, u32 addr, u32 len, u32 off, u32 cmd,
		      u32 p0, u32 p1, u32 p2, u32 p3)
{
	return npu_bridge_egress(port, addr & 0x1FFFFFFF,
				 len << 16 | (off & 0xFFFF),
				 cmd | (port & 7), p0, p1, p2, p3);
}

/* whole packet, or its 32-byte descriptor and the data past split
 * bytes */
static int tunnel_send_split(u32 port, u32 addr, u32 fwd, u32 len, u32 split)
{
	u32 type = SEG_TYPE(fwd != 0);

	len = (u16)len;
	split = (u16)split;
	if (split == 0)
		return tunnel_seg(port, addr, len, 0, SEG_FIRST | SEG_LAST | type,
				  0, 0, 0, 0);
	tunnel_seg(port, addr, 32, 0, SEG_FIRST | SEG_TYPE(1), 0, 0, 0, 0);
	tunnel_seg(port, addr, len - 32 - split, split + 32, SEG_LAST | type,
		   0, 0, 0, 0);
	return 0;
}

void tunnel_pkt_drop(u32 port, u32 pkt_len, u32 desc)
{
	tunnel_seg(port, desc, pkt_len, 0, SEG_FIRST | SEG_LAST | SEG_TYPE(2),
		   0, 0, 0, 0);
}


static void tunnel_desc_flush(u32 port, u32 desc, u32 pkt_len,
			      u32 w0, u32 w1, u32 w2, u32 w3, u32 w4)
{
	volatile u32 *out = (volatile u32 *)(0x1EC12100 + 32 * port);

	out[0] = w0;
	out[1] = desc;
	out[2] = w1;
	out[3] = w2;
	out[4] = w3;
	out[5] = w4;
	out[6] = 0xFFFF;
	REG32(0x1EC12104 + 32 * port) = 1;
}

s32 tunnel_dequeue(u32 port, u32 *pkt_len, u32 *desc_ptr)
{
	u32 remain = tunnel_pending[port];
	u32 *ring_desc;

	if (remain == 0) {
		remain = (u8)REG32(0x1EC12050 + 4 * port);
		tunnel_pending[port] = remain;
		if (remain == 0)
			return -1;
	}

	ring_desc = (u32 *)(REG32(0x1EC12080 + 16 * port) | 0x20000000);
	*desc_ptr = (u32)ring_desc;
	*pkt_len = (*ring_desc & 0xFFFF) + 32;
	tunnel_pending[port] = remain - 1;

	return 0;
}

void tunnel_process(void)
{
	if (tunnel_offload_ready != 0)
		return;
	if (tunnel_test_param != 0 && tunnel_test_mode != 0)
		return;
}

/* RFC 1624 checksum update for a new total length and fragment word */
static u16 ipv4_csum_update(u8 *iph, u16 totlen, u16 frag)
{
	u32 sum;

	sum = (u16)~bswap16(*(u16 *)(iph + 10)) +
	      (u16)~bswap16(*(u16 *)(iph + 2)) + totlen;
	sum = (sum & 0xFFFF) + (sum >> 16) +
	      (u16)~bswap16(*(u16 *)(iph + 6)) + frag;
	sum = (sum & 0xFFFF) + (sum >> 16);
	return (u16)~sum;
}

/* descriptor, a stored header, then the packet past split */
static void tunnel_send_insert(u32 port, u32 hdr, u32 hdr_fwd, u32 hdr_len,
			       u32 desc, u32 fwd, u32 len, u32 split,
			       u32 p0, u32 p1, u32 p2, u32 p3)
{
	split = (u16)split;
	tunnel_seg(port, desc, 32, 0, SEG_FIRST | SEG_TYPE(1), 0, 0, 0, 0);
	tunnel_seg(port, hdr, hdr_len, 0, SEG_TYPE((hdr_fwd != 0) + 4),
		   p0, p1, p2, p3);
	tunnel_seg(port, desc, len - split - 32, split + 32,
		   SEG_LAST | SEG_TYPE(fwd != 0), 0, 0, 0, 0);
}

static s32 vxlan_encap(u32 port, u32 len, u32 *desc, u32 udf)
{
	u32 hdr = tunnel_sram_base() + (udf - 1) * 128;

	desc[0] = port << 4;
	desc[5] = 0x7F4007FF;
	desc[6] = 0xFFFF;
	desc[4] = udf << 14 | 0x3800;
	tunnel_send_insert(port, hdr, 1, 50, (u32)desc, 0, len, 0,
			   PATCH(0x10, len + 4), PATCH(0x26, len - 16), 0, 0);
	return 0;
}

/* two outer packets, the inner IPv4 split at the MTU */
static s32 vxlan_encap_frag(u32 port, u32 len, u32 *desc, u32 udf,
			    u32 hdr_off)
{
	u32 hdr = tunnel_sram_base() + (udf - 1) * 128;
	u8 *iph = (u8 *)desc + hdr_off + 32;
	u32 units = (u16)((tunnel_encap_mtu - 70) >> 3);
	u32 fp = units << 3;
	u32 rem = (u16)(len - fp);

	desc[4] = udf << 14 | 0x3800;
	desc[0] = port << 4;
	desc[5] = 0x7F4007FF;
	desc[6] = 0xFFFF;

	tunnel_seg(port, (u32)desc, 32, 0, SEG_FIRST | SEG_TYPE(1), 0, 0, 0, 0);
	tunnel_seg(port, hdr, 50, 0, SEG_TYPE(5),
		   PATCH(0x10, fp + 70), PATCH(0x26, fp + 50), 0, 0);
	tunnel_seg(port, (u32)desc, fp + 34, 32, SEG_LAST | SEG_TYPE(1),
		   PATCH(0x30, fp + 20), PATCH(0x34, 0x2000),
		   PATCH(0x38, ipv4_csum_update(iph, fp + 20, 0x2000)), 0);

	tunnel_seg(port, (u32)desc, 32, 0, SEG_FIRST | SEG_TYPE(1), 0, 0, 0, 0);
	tunnel_seg(port, hdr, 50, 0, SEG_TYPE(5),
		   PATCH(0x10, rem + 4), PATCH(0x26, rem - 16), 0, 0);
	tunnel_seg(port, (u32)desc, 34, 32, SEG_TYPE(1),
		   PATCH(0x30, rem - 46), PATCH(0x34, units),
		   PATCH(0x38, ipv4_csum_update(iph, rem - 46, units)), 0);
	tunnel_seg(port, (u32)desc, len - 66 - fp, fp + 66, SEG_LAST,
		   0, 0, 0, 0);
	return 0;
}

/* UDF 1-20 encapsulate, 21-40 strip the outer 50 bytes */
static s32 tunnel_vxlan(u32 port, u32 len, u32 *desc, u32 udf)
{
	if ((u8)(udf - 1) > 19) {
		desc[4] = udf << 14 | 0x3800;
		desc[5] = 0x7F4007FF;
		desc[0] = port << 4;
		desc[6] = 0xFFFF;
		return tunnel_send_split(port, (u32)desc, 0, len, 50);
	}
	if (tunnel_encap_mtu >= len + 4)
		return vxlan_encap(port, len, desc, udf);
	return vxlan_encap_frag(port, len, desc, udf,
				(desc[0] >> 20) & 0x7F);
}

/* insert the stored IPv6 + SRH header after the MACs */
static s32 srv6_encap(u32 port, u32 len, u32 *desc, u32 udf, u32 hdr_off)
{
	u32 hdr = tunnel_sram_base() + (udf - 21) * 128;
	u32 hlen = tunnel_srv6_hdr_len[udf - 41];

	desc[4] = udf << 14 | 0x3800;
	desc[5] = 0x7F4007FF;
	desc[0] = port << 4;
	desc[6] = 0xFFFF;
	tunnel_seg(port, (u32)desc, 44, 0, SEG_FIRST | SEG_TYPE(1), 0, 0, 0, 0);
	tunnel_seg(port, hdr, hlen - 12, 12, SEG_TYPE(5),
		   PATCH(0x12, len - hdr_off - 86 + hlen), 0, 0, 0);
	tunnel_seg(port, (u32)desc, len - 46, 46, SEG_LAST, 0, 0, 0, 0);
	return 0;
}

/* strip the outer IPv6 (and SRH when srh is set), inner is IPv4 */
static s32 srv6_decap(u32 port, u32 len, u32 *desc, u32 hdr_off, u32 srh)
{
	u32 strip = 40;

	if (srh)
		strip = ((u8 *)desc)[hdr_off + 73] * 8 + 48;
	tunnel_seg(port, (u32)desc, 46, 0, SEG_FIRST | SEG_TYPE(1),
		   PATCH(0x2C, 0x0800), 0, 0, 0);
	tunnel_seg(port, (u32)desc, len - 46 - strip, 46 + strip, SEG_LAST,
		   0, 0, 0, 0);
	return 0;
}

/* SRv6 endpoint: decap, next segment, or last segment with PSP */
static s32 srv6_end(u32 port, u32 len, u32 *desc, u32 udf, u32 hdr_off)
{
	u8 *pkt = (u8 *)desc + hdr_off;
	u8 *ip6 = pkt + 32;
	u8 *srh = pkt + 72;
	u32 left, ext;
	u16 plen;

	desc[4] = udf << 14 | 0x3800;
	desc[5] = 0x7F4007FF;
	desc[0] = port << 4;
	desc[6] = 0xFFFF;

	if (ip6[6] == 4)
		return srv6_decap(port, len, desc, hdr_off, 0);
	if (srh[3] == 0)
		return srv6_decap(port, len, desc, hdr_off, 1);

	left = (u8)(srh[3] - 1);
	ext = srh[1];
	npu_memcpy(pkt + 56, srh + 8 + 16 * left, 16);
	if (left != 0 || (s8)srh[5] >= 0) {
		srh[3] = left;
		return tunnel_send_split(port, (u32)desc, 0, len, 0);
	}

	ip6[6] = srh[0];
	ext = (u8)(ext * 8 + 8);
	plen = bswap16(*(u16 *)(ip6 + 4)) - ext;
	*(u16 *)(ip6 + 4) = bswap16(plen);
	tunnel_seg(port, (u32)desc, hdr_off + 72, 0, SEG_FIRST | SEG_TYPE(1),
		   0, 0, 0, 0);
	tunnel_seg(port, (u32)desc, plen, ext + hdr_off + 72, SEG_LAST,
		   0, 0, 0, 0);
	return 0;
}

/* UDF 41-48 encapsulate, 49-56 SRv6 endpoint */
static s32 tunnel_srv6(u32 port, u32 len, u32 *desc, u32 udf)
{
	u32 hdr_off = (desc[0] >> 20) & 0x7F;

	if ((u8)(udf - 41) <= 7)
		return srv6_encap(port, len, desc, udf, hdr_off);
	if ((u8)(udf - 49) <= 7)
		return srv6_end(port, len, desc, udf, hdr_off);
	return -1;
}

/* egress w0..w2 only, spinning for credit, cached per channel */
static void tunnel_egress_cached(u32 port, u32 w0, u32 w1, u32 w2)
{
	u32 credit = tunnel_credit[port];

	while (credit == 0)
		credit = (REG32(0x1EC12050 + 4 * port) >> 8) & 0xFF;
	REG32(0x1EC12104 + 32 * port) = w1;
	REG32(0x1EC12108 + 32 * port) = w2;
	REG32(0x1EC12100 + 32 * port) = w0;
	tunnel_credit[port] = credit - 1;
}

/* descriptor and hlen header bytes, a new header, then the packet
 * past split */
static void tunnel_send_replace(u32 port, u32 desc, u32 len, u32 hlen,
				u32 split, u32 hdr, u32 hdr_len)
{
	desc &= 0x1FFFFFFF;
	hlen = (u16)hlen;
	split = (u16)split;
	tunnel_egress_cached(port, desc, (hlen + 32) << 16, port | 0x81000000);
	tunnel_egress_cached(port, hdr & 0x1FFFFFFF, hdr_len << 16,
			     port | 0x81000000);
	tunnel_egress_cached(port, desc,
			     (len - split - 32 - hlen) << 16 |
			     (u16)(split + 32 + hlen), port | 0x40000000);
}

/* UDF 65-68 */
static s32 tunnel_map(u32 port, u32 len, u32 *desc, u32 udf, u32 w1,
		      u32 idx)
{
	u8 *b = (u8 *)desc;
	u16 *h = (u16 *)desc;
	u32 d = (u32)desc & 0x1FFFFFFF;
	u32 hdr = tunnel_sram_base() + 0xE80;

	desc[0] = port << 4;
	desc[4] = udf << 14 | 0x3800;
	desc[5] = 0x7F4007FF;
	desc[6] = 0xFFFF;

	switch (udf) {
	case 65:
		h[25] = bswap16(len - 106);
		b[52] = b[95];
		tunnel_egress_cached(port, d, 86 << 16, port | 0x81000000);
		tunnel_egress_cached(port, d, (u16)(len - 106) << 16 | 106,
				     port | 0x40000000);
		break;
	case 66:
		h[24] = bswap16(len - 86);
		b[55] = b[72];
		h[54] = bswap16(w1);
		tunnel_egress_cached(port, d, 66 << 16, port | 0x81000000);
		tunnel_egress_cached(port, d, (u16)(len - 106) << 16 | 106,
				     port | 0x40000000);
		break;
	case 67:
		npu_memcpy((void *)hdr,
			   (void *)(tunnel_map_info_base + 40 * idx), 40);
		h[22] = bswap16(0x86DD);
		*(u16 *)(hdr + 4) = bswap16(len - 66);
		*(u8 *)(hdr + 6) = b[55];
		tunnel_send_replace(port, (u32)desc, len, 14, 20, hdr, 40);
		break;
	default:
		npu_memcpy((void *)hdr,
			   (void *)(tunnel_map_info_base + 40 * idx), 24);
		h[22] = bswap16(0x0800);
		*(u16 *)(hdr + 2) = bswap16(len - 66);
		*(u8 *)(hdr + 9) = b[52];
		tunnel_send_replace(port, (u32)desc, len, 14, 44, hdr, 24);
		break;
	}
	return 0;
}

/* desc[0] of IPv4 fragments and reassembled packets */
#if defined(AN7581)
#define TUNNEL_V4_FRAG_INFO	0x2200
#else
#define TUNNEL_V4_FRAG_INFO	0x1200
#endif

/* PPPoE length patch, the session header optionally behind one VLAN */
static u32 pppoe_len_patch(u32 *desc, u32 len)
{
	u16 *h = (u16 *)desc;

	if (h[22] == bswap16(0x8864))
		return PATCH(0x32, len);
	if (h[22] == bswap16(0x8100) && h[24] == bswap16(0x8864))
		return PATCH(0x36, len);
	return 0;
}

/* two IPv4 fragments, headers rewritten by patches */
static s32 frag_v4(u32 port, u32 len, u32 *desc, u32 mtu, u32 hdr_off)
{
	u32 units = (u16)((mtu - 20) >> 3);
	u32 fp = units << 3;
	u32 h = (u16)hdr_off;

	desc[0] = (port & 31) << 4 | TUNNEL_V4_FRAG_INFO;
	tunnel_seg(port, (u32)desc, fp + hdr_off + 52, 0,
		   SEG_FIRST | SEG_LAST | SEG_TYPE(1),
		   PATCH(h + 34, fp + 20), PATCH(h + 38, 0x2000),
		   pppoe_len_patch(desc, fp + 22), 0);
	tunnel_seg(port, (u32)desc, hdr_off + 52, 0, SEG_FIRST | SEG_TYPE(1),
		   PATCH(h + 34, len - 32 - h - fp), PATCH(h + 38, units),
		   pppoe_len_patch(desc, len - fp - hdr_off - 30), 0);
	tunnel_seg(port, (u32)desc, len - fp - hdr_off - 52, fp + hdr_off + 52,
		   SEG_LAST, 0, 0, 0, 0);
	return 0;
}

/* two IPv6 fragments; the fragment header is an 8-byte scratch
 * segment filled by patches */
static s32 frag_v6(u32 port, u32 len, u32 *desc, u32 mtu, u32 hdr_off)
{
	u32 frag_hdr = tunnel_sram_base() + 0xE00;
	u8 *nh = (u8 *)desc + hdr_off + 38;
	u32 units = (u16)((mtu - 48) >> 3);
	u32 fs = units << 3;
	u32 h = (u16)hdr_off;
	u32 next, id;

	desc[0] = (port & 31) << 4 | 0x200;
	next = *nh;
	*nh = 44;
	id = ++tunnel_ipv6_frag_id;

	tunnel_seg(port, (u32)desc, hdr_off + 72, 0, SEG_FIRST | SEG_TYPE(1),
		   PATCH(h + 36, fs + 8), pppoe_len_patch(desc, fs + 50), 0, 0);
	tunnel_seg(port, frag_hdr, 8, 0, SEG_TYPE(1), PATCH(0, next << 8),
		   PATCH(2, 1), PATCH(4, id >> 16), PATCH(6, id));
	tunnel_seg(port, (u32)desc, fs, hdr_off + 72, SEG_LAST | SEG_TYPE(1),
		   0, 0, 0, 0);

	tunnel_seg(port, (u32)desc, hdr_off + 72, 0, SEG_FIRST | SEG_TYPE(1),
		   PATCH(h + 36, len - 64 - h - fs),
		   pppoe_len_patch(desc, len - hdr_off - fs - 22), 0, 0);
	tunnel_seg(port, frag_hdr, 8, 0, SEG_TYPE(1), PATCH(0, next << 8),
		   PATCH(2, fs), PATCH(4, id >> 16), PATCH(6, id));
	tunnel_seg(port, (u32)desc, len - hdr_off - fs - 72, hdr_off + 72 + fs,
		   SEG_LAST, 0, 0, 0, 0);
	return 0;
}

/* split a packet longer than the MTU in desc[4] */
static s32 tunnel_fragment(u32 port, u32 len, u32 *desc)
{
	u32 w0 = desc[0];
	u32 mtu = ((u16 *)desc)[9];
	u32 hdr_off = (w0 >> 20) & 0x7F;

	if (hdr_off + mtu >= (u16)w0) {
		npu_printf("error pkt_len=%d mtu=%d in %s,%d\n", (u16)w0, mtu,
			   "npu_tunnel_offload_fragment_op", 233);
		return -1;
	}
	if (w0 & (1 << 27))
		return frag_v4(port, len, desc, mtu, hdr_off);
	if (w0 & (1 << 28))
		return frag_v6(port, len, desc, mtu, hdr_off);
	npu_printf("error ether type in %s,%d\n",
		   "npu_tunnel_offload_fragment_op", 249);
	return -1;
}

s32 tunnel_offload_handler(u32 port, u32 pkt_len, u32 *desc)
{
	u32 w0 = desc[0];
	u32 w1 = desc[1];
	u32 opcode = (w1 >> 28) & 7;
	u32 hdr_off, udf;

	if (opcode == 1)
		return tunnel_fragment(port, pkt_len, desc);

	if (opcode == 2) {
		/* reassemble: dispatch IPv4 vs IPv6 */
		hdr_off = (w0 >> 20) & 0x7F;

		if (w1 & (1 << 27)) {
			/* IPv4 reassembly */
			u8 *l3 = (u8 *)desc + hdr_off + 32;
			u16 frag_flags;

			desc[0] = (16 * (port & 0x1F)) | 0x2200;
			frag_flags = ((u16 *)l3)[3];

			if (frag_flags & bswap16(0x2000)) {
				if (tunnel_v4_reasm_desc != 0) {
					npu_printf("pkt loss 1 in %s,%d\n",
						   "npu_tunnel_offload_reassemble_v4",
						   272);
					tunnel_pkt_drop(port,
							tunnel_v4_reasm_len,
							tunnel_v4_reasm_desc);
				}
				tunnel_v4_reasm_desc = (u32)desc;
				tunnel_v4_reasm_len = pkt_len;
				tunnel_v4_reasm_hdroff = hdr_off;
				((u16 *)l3)[3] = frag_flags &
					bswap16(0xDFFF);
				return 0;
			}

			if (tunnel_v4_reasm_desc == 0) {
				npu_printf("pkt loss 2 in %s,%d\n",
					   "npu_tunnel_offload_reassemble_v4",
					   287);
				return -1;
			}

			{
				u8 *saved = (u8 *)tunnel_v4_reasm_desc +
					tunnel_v4_reasm_hdroff;
				u16 saved_ipid = ((u16 *)(saved + 32))[2];
				u16 cur_ipid = ((u16 *)l3)[2];
				u16 cur_fragoff;
				u16 saved_totlen;
				u32 ihl;

				if (saved_ipid != cur_ipid) {
					npu_printf("pkt err 1 in %s,%d\n",
						   "npu_tunnel_offload_reassemble_v4",
						   295);
					return -1;
				}

				cur_fragoff = 8 * bswap16(((u16 *)l3)[3]);
				ihl = (4 * l3[0]) & 0x3C;
				saved_totlen = bswap16(((u16 *)(saved + 32))[1]);

				if (cur_fragoff != saved_totlen - ihl) {
					npu_printf("pkt err 2 in %s,%d\n",
						   "npu_tunnel_offload_reassemble_v4",
						   302);
					return -1;
				}

				((u16 *)(saved + 32))[1] = bswap16(
					bswap16(((u16 *)(saved + 32))[1]) +
					pkt_len - 52 - hdr_off);

				{
					u32 merged = tunnel_v4_reasm_len - 82 -
						tunnel_v4_reasm_hdroff +
						pkt_len - hdr_off;

					if (((u16 *)desc)[22] == bswap16(0x8864) ||
					    (((u16 *)desc)[22] == bswap16(0x8100) &&
					     ((u16 *)desc)[24] == bswap16(0x8864)))
						bswap16((u16)merged);

					tunnel_desc_flush(port, (u32)desc,
							  pkt_len, 0, 0, 0,
							  0, 0);
					tunnel_desc_flush(port, (u32)desc,
							  pkt_len, 0, 0, 0,
							  0, 0);
				}

				tunnel_v4_reasm_desc = 0;
				tunnel_v4_reasm_len = 0;
				tunnel_v4_reasm_hdroff = 0;
				return 0;
			}
		}

		if (w1 & (1 << 28)) {
			/* IPv6 reassembly */
			u8 *pkt = (u8 *)desc + hdr_off;

			desc[0] = (16 * (port & 0x1F)) | 0x200;

			if (pkt[38] != 44) {
				npu_printf("IPv6 next header error in %s,%d\n",
					   "npu_tunnel_offload_reassemble_v6",
					   346);
				return -1;
			}

			{
				u16 *fhdr = (u16 *)(pkt + 72);
				u16 frag_flags = fhdr[1];

				if (bswap16(1) & frag_flags) {
					if (tunnel_v6_reasm_desc != 0) {
						npu_printf("pkt loss 1 in %s,%d\n",
							   "npu_tunnel_offload_reassemble_v6",
							   354);
						tunnel_pkt_drop(port,
								tunnel_v6_reasm_len,
								tunnel_v6_reasm_desc);
					}
					tunnel_v6_reasm_desc = (u32)desc;
					tunnel_v6_reasm_len = pkt_len;
					tunnel_v6_reasm_hdroff = hdr_off;
					*((u8 *)(pkt + 72) + 6) = pkt[72];
					return 0;
				}

				if (tunnel_v6_reasm_desc == 0) {
					npu_printf("pkt loss 2 in %s,%d\n",
						   "npu_tunnel_offload_reassemble_v6",
						   369);
					return -1;
				}

				{
					u8 *saved = (u8 *)tunnel_v6_reasm_desc +
						tunnel_v6_reasm_hdroff;
					u16 *saved_fhdr = (u16 *)(saved + 72);
					u16 *saved_ipv6 = (u16 *)(saved + 32);
					u16 saved_frag_id = saved_fhdr[2];
					u16 cur_frag_id = ((u16 *)(pkt + 72))[2];
					u16 saved_frag_off = saved_fhdr[3];
					u16 cur_frag_off = ((u16 *)(pkt + 72))[3];
					u32 cur_off, saved_plen;

					if (saved_frag_id != cur_frag_id ||
					    saved_frag_off != cur_frag_off) {
						npu_printf("pkt err 1 in %s,%d\n",
							   "npu_tunnel_offload_reassemble_v6",
							   378);
						return -1;
					}

					cur_off = (bswap16(frag_flags) &
						   ~7) << 16 >> 16;
					saved_plen = bswap16(saved_ipv6[2]);

					if (cur_off != saved_plen - 8) {
						npu_printf("pkt err 2 in %s,%d\n",
							   "npu_tunnel_offload_reassemble_v6",
							   385);
						return -1;
					}

					saved_ipv6[2] = bswap16(
						bswap16(saved_ipv6[2]) +
						pkt_len - 88 - hdr_off);

					{
						u32 merged =
							tunnel_v6_reasm_len -
							118 -
							tunnel_v6_reasm_hdroff +
							pkt_len - hdr_off;

						if (((u16 *)desc)[22] == bswap16(0x8864) ||
						    (((u16 *)desc)[22] == bswap16(0x8100) &&
						     ((u16 *)desc)[24] == bswap16(0x8864)))
							bswap16((u16)merged);

						tunnel_desc_flush(port,
								  (u32)desc,
								  pkt_len,
								  0, 0, 0,
								  0, 0);
						tunnel_desc_flush(port,
								  (u32)desc,
								  pkt_len,
								  0, 0, 0,
								  0, 0);
						tunnel_desc_flush(port,
								  (u32)desc,
								  pkt_len,
								  0, 0, 0,
								  0, 0);
					}

					tunnel_v6_reasm_desc = 0;
					tunnel_v6_reasm_len = 0;
					tunnel_v6_reasm_hdroff = 0;
					return 0;
				}
			}
		}

		npu_printf("error ether type in %s,%d\n",
			   "npu_tunnel_offload_reassemble_op", 435);
		return -1;
	}

	/* encap/decap: dispatch on UDF byte */
	udf = ((u8 *)desc)[20];
	hdr_off = (w0 >> 20) & 0x7F;

	if ((u8)(udf - 1) <= 39)
		return tunnel_vxlan(port, pkt_len, desc, udf);
	if ((u8)(udf - 41) <= 15)
		return tunnel_srv6(port, pkt_len, desc, udf);
	if ((u8)(udf - 65) <= 3)
		return tunnel_map(port, pkt_len, desc, udf, w1 & 0x3FFFF,
				  ((u16 *)desc)[8]);

	npu_printf("invalid, hop_flags %d udf %d in %s,%d\n",
		   opcode, udf, "npu_tunnel_offload_common_op", 187);
	return -1;
}


int tunnel_mail_frag_mtu(u32 base, u32 cnt)
{
	u32 idx = *(u8 *)(base + 8);
	u32 mtu = *(u32 *)(base + 12);

	(void)cnt;
	fragment_mtu[idx] = mtu;
	npu_printf("set fragment mtu-%d: %d\n", idx, mtu);
	return 1;
}

/* tunnel mailbox sub-handlers */
int tunnel_mail_store_hdr(u32 base, u32 cnt)
{
	u32 idx = *(u8 *)(base + 8);
	u32 bridge_addr = npu_bridge_addr();

	(void)cnt;
	npu_memcpy((void *)(bridge_addr + idx * 128), (void *)(base + 9), 50);
	return 1;
}

int tunnel_mail_store_srv6(u32 base, u32 cnt)
{
	u32 idx = *(u8 *)(base + 8);
	u32 len = *(u8 *)(base + 9);
	u32 bridge_addr = npu_bridge_addr();

	(void)cnt;
	if (idx > 7) {
		npu_printf("invalid idx %d in %s,%d\n",
			   idx, "tunnel_mail_npu_store_srv6_hdr", 107);
		return 1;
	}

	npu_memcpy((void *)(bridge_addr + (idx + 20) * 128),
		   (void *)(base + 10), len);
	tunnel_srv6_hdr_len[idx] = (u8)len;
	return 1;
}

int tunnel_mail_set_srv6_addr(u32 base, u32 cnt)
{
	(void)cnt;
	npu_memcpy(srv6_my_ipv6, (void *)(base + 8), 16);
	npu_printf("set srv6 my ipv6: %02X%02X:%02X%02X:%02X%02X:%02X%02X:%02X%02X:%02X%02X:%02X%02X:%02X%02X\n",
		   srv6_my_ipv6[0], srv6_my_ipv6[1], srv6_my_ipv6[2], srv6_my_ipv6[3],
		   srv6_my_ipv6[4], srv6_my_ipv6[5], srv6_my_ipv6[6], srv6_my_ipv6[7],
		   srv6_my_ipv6[8], srv6_my_ipv6[9], srv6_my_ipv6[10], srv6_my_ipv6[11],
		   srv6_my_ipv6[12], srv6_my_ipv6[13], srv6_my_ipv6[14], srv6_my_ipv6[15]);
	return 1;
}

int tunnel_mail_nop(u32 base, u32 cnt)
{
	(void)base; (void)cnt;
	return 1;
}

int tunnel_mail_vxlan_mtu(u32 base, u32 cnt)
{
	(void)cnt;
	tunnel_encap_mtu = *(u32 *)(base + 8);
	npu_printf("set vxlan mtu %d\n", tunnel_encap_mtu);
	return 1;
}

/* bridge counters dump, reset or reassembly flush */
int tunnel_mail_bridge_dbg(u32 base, u32 cnt)
{
	(void)cnt;
	npu_bridge_debug(*(u32 *)(base + 8));
	return 1;
}

int tunnel_mail_map_info(u32 base, u32 cnt)
{
	(void)cnt;
	tunnel_map_info_base = *(u32 *)(base + 8);
	npu_printf("map_info_base %x\n", tunnel_map_info_base);
	return 1;
}

int tunnel_mail_l4s(u32 base, u32 cnt)
{
	(void)cnt;
	l4s_set_config(*(u32 *)(base + 8), *(u32 *)(base + 12));
	return 1;
}

#endif /* HAS_TUNNEL */
