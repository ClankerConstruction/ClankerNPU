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
	return npu_bridge_pkt_base + 0x10000;
}

static void bridge_cmd_submit(u32 port, u32 desc, u32 cmd, u32 arg0,
			      u32 arg1, u32 arg2, u32 arg3, u32 arg4)
{
	volatile u32 *status = (volatile u32 *)(0x1EC12050 + 4 * port);

	if ((REG32(0x1EC12050 + 4 * port) & 0xFF00) == 0)
		return;

	REG32(0x1EC12100 + 32 * port + 0) = cmd;
	REG32(0x1EC12100 + 32 * port + 4) = desc;
	REG32(0x1EC12100 + 32 * port + 8) = arg0;
	REG32(0x1EC12100 + 32 * port + 12) = arg1;
	REG32(0x1EC12100 + 32 * port + 16) = arg2;
	REG32(0x1EC12100 + 32 * port + 20) = arg3;
	REG32(0x1EC12100 + 32 * port + 24) = arg4;

	REG32(0x1EC12104 + 32 * port) = 1;
	(void)status;
}

void tunnel_pkt_drop(u32 port, u32 pkt_len, u32 desc)
{
	bridge_cmd_submit(port, desc, pkt_len << 16,
			  (port & 7) | 0xC2000000, 0, 0, 0, 0);
}

static void tunnel_pkt_continue(u32 port, u32 pkt_len, u32 desc)
{
	bridge_cmd_submit(port, desc, pkt_len << 16,
			  (port & 7) | 0xC0000000, 0, 0, 0, 0);
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
	*pkt_len = *ring_desc + 32;
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

s32 tunnel_offload_handler(u32 port, u32 pkt_len, u32 *desc)
{
	u32 w0 = desc[0];
	u32 w1 = desc[1];
	u32 opcode = (w1 >> 28) & 7;
	u32 hdr_off, mtu, udf;
	u32 sram;
	u16 *hw;

	if (opcode == 1) {
		/* fragment: dispatch IPv4 vs IPv6 */
		u16 plen = (u16)w0;
		mtu = ((u16 *)desc)[9];
		hdr_off = (w0 >> 20) & 0x7F;

		if (hdr_off + mtu >= plen) {
			npu_printf("error pkt_len=%d mtu=%d in %s,%d\n",
				   plen, mtu,
				   "npu_tunnel_offload_fragment_op", 233);
			return -1;
		}

		if (w1 & (1 << 27)) {
			/* IPv4 fragmentation */
			u32 frag_units = (mtu - 20) >> 3;
			u32 frag_payload = 8 * frag_units;

			desc[0] = (16 * (port & 0x1F)) | 0x2200;
			hw = (u16 *)((u8 *)desc + hdr_off + 32);
			hw[1] = bswap16((u16)(frag_payload + 20));
			hw[3] = bswap16(0x2000);

			if (((u16 *)desc)[22] == bswap16(0x8864) ||
			    (((u16 *)desc)[22] == bswap16(0x8100) &&
			     ((u16 *)desc)[24] == bswap16(0x8864)))
				hw[1] = bswap16((u16)(frag_payload + 22));

			tunnel_desc_flush(port, (u32)desc, pkt_len,
					  0, 0, 0, 0, 0);

			hw = (u16 *)((u8 *)desc + hdr_off + 32);
			hw[1] = bswap16((u16)(pkt_len - 32 - hdr_off -
					      frag_payload));
			hw[3] = bswap16((u16)frag_units);

			if (((u16 *)desc)[22] == bswap16(0x8864) ||
			    (((u16 *)desc)[22] == bswap16(0x8100) &&
			     ((u16 *)desc)[24] == bswap16(0x8864))) {
				u32 adj = pkt_len - frag_payload - hdr_off - 30;
				hw[1] = bswap16((u16)adj);
			}

			tunnel_desc_flush(port, (u32)desc, pkt_len,
					  0, 0, 0, 0, 0);
			tunnel_desc_flush(port, (u32)desc, pkt_len,
					  0, 0, 0, 0, 0);
			return 0;
		}

		if (w1 & (1 << 28)) {
			/* IPv6 fragmentation */
			u8 *pkt = (u8 *)desc + hdr_off + 38;
			u8 orig_nh = *pkt;
			u32 frag_units = (mtu - 48) >> 3;
			u32 frag_sz = 8 * frag_units;
			u32 frag_id;

			tunnel_sram_base();
			desc[0] = (16 * (port & 0x1F)) | 0x200;
			*pkt = 44;
			tunnel_ipv6_frag_id++;
			frag_id = tunnel_ipv6_frag_id;

			hw = (u16 *)((u8 *)desc + hdr_off + 32);
			hw[2] = bswap16((u16)(frag_sz + 8));

			if (((u16 *)desc)[22] == bswap16(0x8864) ||
			    (((u16 *)desc)[22] == bswap16(0x8100) &&
			     ((u16 *)desc)[24] == bswap16(0x8864)))
				hw[2] = bswap16((u16)(frag_sz + 50));

			tunnel_desc_flush(port, (u32)desc, pkt_len,
					  0, 0, 0, 0, 0);

			/* frag header: nh, reserved, offset|M, id */
			hw = (u16 *)((u8 *)desc + hdr_off + 32);
			hw[3] = bswap16((u16)(orig_nh << 8));
			hw[4] = bswap16(1);
			hw[5] = bswap16((u16)(frag_id >> 16));
			hw[6] = bswap16((u16)frag_id);

			tunnel_desc_flush(port, (u32)desc, pkt_len,
					  0, 0, 0, 0, 0);
			tunnel_desc_flush(port, (u32)desc, pkt_len,
					  0, 0, 0, 0, 0);

			/* second fragment payload */
			hw = (u16 *)((u8 *)desc + hdr_off + 32);
			hw[2] = bswap16((u16)(pkt_len - 64 - hdr_off -
					      frag_sz));

			if (((u16 *)desc)[22] == bswap16(0x8864) ||
			    (((u16 *)desc)[22] == bswap16(0x8100) &&
			     ((u16 *)desc)[24] == bswap16(0x8864))) {
				u32 adj = pkt_len - hdr_off - frag_sz - 22;
				hw[2] = bswap16((u16)adj);
			}

			tunnel_desc_flush(port, (u32)desc, pkt_len,
					  0, 0, 0, 0, 0);

			hw[3] = bswap16((u16)(orig_nh << 8));
			hw[4] = bswap16((u16)frag_sz);
			hw[5] = bswap16((u16)(frag_id >> 16));
			hw[6] = bswap16((u16)frag_id);

			tunnel_desc_flush(port, (u32)desc, pkt_len,
					  0, 0, 0, 0, 0);
			tunnel_desc_flush(port, (u32)desc, pkt_len,
					  0, 0, 0, 0, 0);
			return 0;
		}

		npu_printf("error ether type in %s,%d\n",
			   "npu_tunnel_offload_fragment_op", 249);
		return -1;
	}

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

	if ((u8)(udf - 1) <= 0x27) {
		/* UDF 1-40: tunnel encapsulation */
		if ((u8)(udf - 1) <= 0x13) {
			/* UDF 1-20: IPv4 encap */
			sram = tunnel_sram_base();

			if (tunnel_encap_mtu < pkt_len + 4) {
				u32 frag_units = (tunnel_encap_mtu - 70) >> 3;
				u32 frag_payload = 8 * (u16)frag_units;

				desc[4] = (udf << 14) | 0x3800;
				desc[0] = 16 * port;
				desc[5] = 0x7F3FFFFF;
				desc[6] = 0xFFFF;

				bswap16((u16)(frag_payload + 70));
				bswap16((u16)(frag_payload + 50));
				bswap16((u16)(frag_payload + 20));
				bswap16(0x2000);

				{
					u32 rem = pkt_len - 15 - frag_payload;

					bswap16((u16)(rem + 4));
					bswap16((u16)(rem - 16));
					bswap16((u16)(rem - 46));
					bswap16((u16)frag_units);
				}
				return 0;
			}

			desc[0] = 16 * port;
			desc[5] = 0x7F3FFFFF;
			desc[6] = 0xFFFF;
			desc[4] = (udf << 14) | 0x3800;
			bswap16((u16)(pkt_len + 4));
			bswap16((u16)(pkt_len - 16));
			tunnel_pkt_continue(port, pkt_len, (u32)desc);
			return 0;
		}

		/* UDF 21-40: pass-through encap */
		desc[4] = (udf << 14) | 0x3800;
		desc[5] = 0x7F3FFFFF;
		desc[0] = 16 * port;
		desc[6] = 0xFFFF;
		tunnel_pkt_continue(port, (u32)desc, 0);
		return 0;
	}

	if ((u8)(udf - 41) <= 0xF) {
		/* UDF 41-56: SRv6 processing */
		if ((u8)(udf - 41) <= 7) {
			/* UDF 41-48: SRv6 decap */
			sram = tunnel_sram_base();
			desc[4] = (udf << 14) | 0x3800;
			desc[5] = 0x7F3FFFFF;
			desc[0] = 16 * port;
			desc[6] = 0xFFFF;

			tunnel_desc_flush(port, (u32)desc, pkt_len,
					  0, 0, 0, 0, 0);

			{
				u32 adj = pkt_len - hdr_off - 86 +
					tunnel_srv6_hdr_len[udf - 41];
				bswap16((u16)adj);
			}

			tunnel_desc_flush(port, (u32)desc, pkt_len,
					  0, 0, 0, 0, 0);
			tunnel_desc_flush(port, (u32)desc, pkt_len,
					  0, 0, 0, 0, 0);
			return 0;
		}

		if ((u8)(udf - 49) > 7)
			return -1;

		/* UDF 49-56: SRv6 transit */
		desc[4] = (udf << 14) | 0x3800;
		desc[5] = 0x7F3FFFFF;
		desc[0] = 16 * port;
		desc[6] = 0xFFFF;

		{
			u8 *pkt = (u8 *)desc + hdr_off;
			u8 *ipv6 = pkt + 32;
			u8 *srh = pkt + 72;
			u8 segs_left = srh[3];

			if (ipv6[6] == 4 || segs_left == 0) {
				sram = tunnel_sram_base();
				bswap16(0x800);
				tunnel_desc_flush(port, (u32)desc, pkt_len,
						  0, 0, 0, 0, 0);
				tunnel_desc_flush(port, (u32)desc, pkt_len,
						  0, 0, 0, 0, 0);
				return 0;
			}

			{
				u8 seg_idx = segs_left - 1;
				u8 seg_size = srh[1];

				npu_memcpy(pkt + 56, srh + 8 + 16 * seg_idx,
					   16);

				if (seg_idx != 0 || (s8)srh[5] >= 0) {
					srh[3] = seg_idx;
					tunnel_pkt_continue(port, (u32)desc,
							    0);
					return 0;
				}

				{
					u16 payload_len =
						bswap16(((u16 *)ipv6)[2]);

					ipv6[6] = srh[0];
					((u16 *)ipv6)[2] = bswap16(
						payload_len -
						(u8)(8 * seg_size + 8));

					tunnel_desc_flush(port, (u32)desc,
							  pkt_len, 0, 0, 0,
							  0, 0);
					tunnel_desc_flush(port, (u32)desc,
							  pkt_len, 0, 0, 0,
							  0, 0);
					return 0;
				}
			}
		}
	}

	if ((u8)(udf - 65) > 3) {
		npu_printf("invalid, hop_flags %d udf %d in %s,%d\n",
			   opcode, udf,
			   "npu_tunnel_offload_common_op", 187);
		return -1;
	}

	/* UDF 65-68: special encap/decap */
	switch (udf) {
	case 65: /* 'A': decap to IPv6 inner */
		desc[0] = 16 * port;
		desc[4] = 1079296;
		desc[5] = 0x7F3FFFFF;
		desc[6] = 0xFFFF;
		((u16 *)desc)[25] = bswap16((u16)(pkt_len - 106));
		((u8 *)desc)[52] = ((u8 *)desc)[95];
		return 0;

	case 66: /* 'B': decap to IPv4 inner */
		desc[0] = 16 * port;
		desc[5] = 0x7F3FFFFF;
		desc[4] = 1095680;
		desc[6] = 0xFFFF;
		((u16 *)desc)[24] = bswap16((u16)(pkt_len - 86));
		((u8 *)desc)[55] = ((u8 *)desc)[72];
		((u16 *)desc)[54] = bswap16((u16)(w1 & 0x3FFFFF));
		return 0;

	case 67: { /* 'C': SRv6 encap with IPv6 outer */
		u32 seg_idx = ((u16 *)desc)[8];

		sram = tunnel_sram_base();
		desc[0] = 16 * port;
		desc[5] = 0x7F3FFFFF;
		desc[6] = 0xFFFF;
		desc[4] = 1112064;
		npu_memcpy((void *)(sram + 3712),
			   (void *)(tunnel_srv6_seg_table + 40 * seg_idx),
			   40);
		((u16 *)desc)[22] = bswap16(0x86DD);
		*(u16 *)(sram + 3716) = bswap16((u16)(pkt_len - 66));
		*(u8 *)(sram + 3718) = ((u8 *)desc)[55];
		return 0;
	}

	case 68: { /* 'D': GRE/IP encap with IPv4 outer */
		u32 seg_idx = ((u16 *)desc)[8];

		sram = tunnel_sram_base();
		desc[0] = 16 * port;
		desc[5] = 0x7F3FFFFF;
		desc[6] = 0xFFFF;
		desc[4] = 1128448;
		npu_memcpy((void *)(sram + 3712),
			   (void *)(tunnel_srv6_seg_table + 40 * seg_idx),
			   24);
		((u16 *)desc)[22] = bswap16(0x800);
		*(u16 *)(sram + 3714) = bswap16((u16)(pkt_len - 66));
		*(u8 *)(sram + 3721) = ((u8 *)desc)[52];
		return 0;
	}

	default:
		return port;
	}
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
