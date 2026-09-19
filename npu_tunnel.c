#include "npu_internal.h"

/* ================================================================
 * Tunnel funcId dispatch (callback[1] — MFUNC_TUNNEL)
 *
 * All variants register this at callback[1]. The table is pre-
 * initialized in the blob's .data section; entries point to
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

/* ================================================================
 * Tunnel offload (AN758X)
 * ================================================================ */

#ifdef HAS_TUNNEL

/* PPE register bases for tunnel offload */
#define PPE0_CTRL       0x1FB50E00
#define PPE0_CTRL2      0x1FB50E04
#define PPE0_IPCHK_CFG  0x1FB50E08
#define PPE0_IPCHK0     0x1FB50E0C
#define PPE0_IPCHK1     0x1FB50E10
#define PPE0_IPCHK2     0x1FB50E14
#define PPE0_IPCHK3     0x1FB50E18
#define PPE1_OFFSET     0x1000
#define PPE1_CTRL       0x1FB51E00
#define PPE1_CTRL2      0x1FB51E04
#define PPE0_MISC       0x1FB50E1C
#define PPE1_MISC       0x1FB51E1C
#define PPE0_PARSER     0x1FB50E88
#define PPE0_ETYPE_EN   0x1FB50E8C
#define PPE0_ETYPE_TBL  0x1FB50ED0
#define PPE1_PARSER     0x1FB51E88
#define PPE0_SHAPER     0x1FB50E28
#define PPE0_ENABLE     0x1FB50E50
#define PPE1_ENABLE     0x1FB51E50
#define GDM_BASE        0x1FB50000
#define GDM_PARSE0      0x1FB50280
#define PPE_QDMA0       0x1FB50500
#define PPE_QDMA1       0x1FB51500
#define PPE_QDMA2       0x1FB52500
#define PPE_QDMA_EXTRA  0x1FB51100
#define PPE0_FLT_BASE   0x1FB50F00
#define PPE1_FLT_BASE   0x1FB51F00

#define CHIP_FAMILY     (REG32(CHIP_ID_REG) >> 16)
#define CHIP_REV5       (((REG32(CHIP_VARIANT_REG) >> 3) & 0x10) | \
			 (REG32(CHIP_VARIANT_REG) & 0xF))

static s32 chip_cap_query(u32 idx, u32 query)
{
	struct { u32 match; u8 caps; } tbl[32];
	u32 fam = CHIP_FAMILY;
	u32 rev = CHIP_REV5;

	npu_memset(tbl, 0, sizeof(tbl));

	tbl[ 0] = (__typeof__(tbl[0])){ fam == 14 && rev ==  0, 0x1F };
	tbl[ 1] = (__typeof__(tbl[0])){ fam == 14 && rev ==  1, 0x1E };
	tbl[ 2] = (__typeof__(tbl[0])){ fam == 14 && rev ==  2, 0x1B };
	tbl[ 3] = (__typeof__(tbl[0])){ fam == 14 && (rev == 3 || rev == 13), 0x13 };
	tbl[ 4] = (__typeof__(tbl[0])){ fam == 14 && rev ==  4, 0x1B };
	tbl[ 5] = (__typeof__(tbl[0])){ fam == 14 && rev ==  5, 0x1B };
	tbl[ 6] = (__typeof__(tbl[0])){ fam == 14 && rev ==  6, 0x03 };
	tbl[ 7] = (__typeof__(tbl[0])){ fam == 14 && rev ==  7, 0x1F };
	tbl[ 8] = (__typeof__(tbl[0])){ fam == 14 && rev ==  8, 0x1B };
	tbl[ 9] = (__typeof__(tbl[0])){ fam == 14 && rev ==  9, 0x13 };
	tbl[10] = (__typeof__(tbl[0])){ fam == 14 && rev == 10, 0x1F };
	tbl[11] = (__typeof__(tbl[0])){ fam == 14 && rev == 11, 0x1A };
	tbl[12] = (__typeof__(tbl[0])){ fam == 14 && rev == 12, 0x1B };
	tbl[13] = (__typeof__(tbl[0])){ fam == 16 && rev ==  0, 0x1F };
	tbl[14] = (__typeof__(tbl[0])){ fam == 16 && rev ==  1, 0x1F };
	tbl[15] = (__typeof__(tbl[0])){ fam == 16 && rev ==  2, 0x1F };
	tbl[16] = (__typeof__(tbl[0])){ fam == 16 && rev ==  3, 0x1F };
	tbl[17] = (__typeof__(tbl[0])){ fam == 16 && rev ==  5, 0x01 };
	tbl[18] = (__typeof__(tbl[0])){ fam == 16 && rev ==  6, 0x1F };
	tbl[19] = (__typeof__(tbl[0])){ fam == 16 && rev ==  7, 0x1F };
	tbl[20] = (__typeof__(tbl[0])){ fam == 16 && rev ==  8, 0x1F };
	tbl[21] = (__typeof__(tbl[0])){ fam == 16 && rev ==  9, 0x1F };
	tbl[22] = (__typeof__(tbl[0])){ fam == 16 && rev == 10, 0x1F };
	tbl[23] = (__typeof__(tbl[0])){ fam == 16 && rev == 11, 0x1F };
	tbl[24] = (__typeof__(tbl[0])){ fam == 16 && rev == 12, 0x1F };
	tbl[25] = (__typeof__(tbl[0])){ fam == 16 && rev == 13, 0x1F };
	tbl[26] = (__typeof__(tbl[0])){ fam == 16 && rev == 16, 0x1F };
	tbl[27] = (__typeof__(tbl[0])){ fam == 16 && rev == 18, 0x1F };
	tbl[28] = (__typeof__(tbl[0])){ fam == 16 && rev == 28, 0x1F };
	tbl[29] = (__typeof__(tbl[0])){ fam == 16 && rev == 21, 0x01 };
	tbl[30] = (__typeof__(tbl[0])){ fam == 16 && rev == 22, 0x1F };
	tbl[31].match = (u32)-1;

	if (idx > 31)
		return -1;

	switch (query) {
	case 0:  return (s32)tbl[idx].match;
	case 1:  return  tbl[idx].caps & 1;
	case 2:  return (tbl[idx].caps >> 1) & 1;
	case 3:  return (tbl[idx].caps >> 2) & 1;
	case 4:  return (tbl[idx].caps >> 3) & 1;
	case 5:  return (tbl[idx].caps >> 4) & 1;
	default: return (s32)tbl[idx].match;
	}
}

static int l4s_set_config(u32 cmd, u32 arg)
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

int tunnel_mail_frag_mtu(u32 base, u32 cnt)
{
	u32 idx = *(u8 *)(base + 8);
	u32 mtu = *(u32 *)(base + 12);

	(void)cnt;
	fragment_mtu[idx] = mtu;
	npu_printf("set fragment mtu-%d: %d\n", idx, mtu);
	return 1;
}

static void tunnel_ppe_reset(void)
{
	u32 chip_rev = REG32(CHIP_ID_REG) >> 16;

	/* clear PPE0 control bits */
	REG32(PPE0_CTRL) &= ~1u;
	REG32(PPE0_CTRL) &= ~2u;
	REG32(PPE0_CTRL) &= ~0x100u;
	REG32(PPE0_CTRL) &= ~0x200u;
	REG32(PPE0_CTRL) &= ~0x40u;
	REG32(PPE0_CTRL) &= ~0x1000u;
	REG32(PPE0_CTRL) &= ~0x20u;

	/* dual-PPE (AN7581) */
	if (chip_rev == 14) {
		REG32(PPE1_CTRL) &= ~1u;
		REG32(PPE1_CTRL) &= ~2u;
		REG32(PPE1_CTRL) &= ~0x100u;
		REG32(PPE1_CTRL) &= ~0x200u;
		REG32(PPE1_CTRL) &= ~0x40u;
		REG32(PPE1_CTRL) &= ~0x1000u;
		REG32(PPE1_CTRL) &= ~0x20u;
	}

	/* SRv6/MAP-T disable */
	if (chip_rev == 10 || chip_rev == 12 || chip_rev == 14 ||
	    chip_rev == 15 || chip_rev == 16) {
		REG32(PPE0_CTRL) &= ~0x8000u;
		if (chip_rev == 14)
			REG32(PPE1_CTRL) &= ~0x8000u;
	}

	if (chip_rev == 11)
		REG32(PPE0_CTRL) ^= ~REG32(PPE0_CTRL) & 0x8000;

	/* clear remaining bits */
	REG32(PPE0_CTRL) &= ~0x10u;
	REG32(PPE0_CTRL) ^= ~(u8)REG32(PPE0_CTRL) & 8;
	REG32(PPE0_CTRL) ^= ~(u8)REG32(PPE0_CTRL) & 4;

	if (chip_rev == 14) {
		REG32(PPE1_CTRL) &= ~0x10u;
		REG32(PPE1_CTRL) ^= ~(u8)REG32(PPE1_CTRL) & 8;
		REG32(PPE1_CTRL) ^= ~(u8)REG32(PPE1_CTRL) & 4;
	}

	/* VXLAN/GRE disable */
	if (chip_rev == 12 || chip_rev == 14 ||
	    chip_rev == 15 || chip_rev == 16) {
		REG32(PPE0_CTRL) &= ~0x10000u;
		REG32(PPE0_CTRL) &= ~0x20000u;
		if (chip_rev == 14) {
			REG32(PPE1_CTRL) &= ~0x10000u;
			REG32(PPE1_CTRL) &= ~0x20000u;
		}
	}

	/* preserve only bit 16 of ctrl2 */
	REG32(PPE0_CTRL2) &= 0x10000u;
	if (chip_rev == 14)
		REG32(PPE1_CTRL2) = REG32(PPE0_CTRL2) & 0x10000;

	/* clear misc interrupt bits */
	REG32(PPE0_MISC) &= ~0x80u;
	REG32(PPE0_MISC) &= ~0x100u;
	REG32(PPE0_MISC) &= ~0x200u;
	REG32(PPE0_MISC) &= ~0x400u;
	REG32(PPE0_MISC) &= ~0x800u;

	if (chip_rev == 14) {
		REG32(PPE0_MISC) &= ~0x80u;
		REG32(PPE0_MISC) &= ~0x100u;
		REG32(PPE0_MISC) &= ~0x200u;
		REG32(PPE0_MISC) &= ~0x400u;
		REG32(PPE0_MISC) &= ~0x800u;
	}
}

static void ppe_qdma_config(u32 dir)
{
	u32 chip_rev = CHIP_FAMILY;
	u32 v1, v2, v3, v4, v5, v6, port;

	if (dir == 0) {
		v1 = 5;
		v2 = 0;
		v3 = 0;
		v4 = 0;
		v5 = 0;
		v6 = (hwnat_wan_xsi == 0) ? 0 : 5;
		port = 0;
	} else {
		v1 = 3;
		if (hwnat_wan_xsi == 0) {
			v1 = 4;
			if (chip_rev == 14)
				v1 = (REG32(PPE1_CTRL) & 1) ? 8 : 4;
		}
		v2 = 64;
		v3 = 1024;
		v4 = 0x4000;
		v6 = 4;
		v5 = (chip_rev == 14 || chip_rev == 16) ? 4 : 0;
		port = 4;
	}

	REG32(PPE_QDMA0) = (REG32(PPE_QDMA0) & 0xFFFFFF00) | port | v2;
	REG32(PPE_QDMA0) = (REG32(PPE_QDMA0) & 0xFFFF00FF) | v3 | v4;

	if (hwnat_wan_mode != 1 && (hwnat_wan_xsi | hwnat_ae_wan_sel) == 0) {
		REG32(PPE_QDMA1) = v1 | (REG32(PPE_QDMA1) & 0xFFFFFFF0);
		REG32(PPE_QDMA1) = (16 * v1) | (REG32(PPE_QDMA1) & 0xFFFFFF0F);
		REG32(PPE_QDMA1) = ((v1 << 8) & 0xFFFF0FFF) |
				   (REG32(PPE_QDMA1) & 0xFFFF00FF) | (v1 << 12);
	}
	if (chip_rev == 10) {
		REG32(PPE_QDMA_EXTRA) = (REG32(PPE_QDMA_EXTRA) & 0xFFFFFFF0) | v6;
		REG32(PPE_QDMA_EXTRA) = (v6 << 12) | ((v6 << 8) & 0xFFFF0FFF) |
					((16 * v6) & 0xFFFF00FF) |
					(REG32(PPE_QDMA_EXTRA) & 0xFFFF000F);
	}
	if (chip_rev == 14 || chip_rev == 16) {
		REG32(PPE_QDMA2) = v5 | (REG32(PPE_QDMA2) & 0xFFFFFFF0);
		REG32(PPE_QDMA2) = (16 * v5) | (REG32(PPE_QDMA2) & 0xFFFFFF0F);
		REG32(PPE_QDMA2) = (v5 << 8) | (REG32(PPE_QDMA2) & 0xFFFFF0FF);
		REG32(PPE_QDMA2) = (v5 << 12) | (REG32(PPE_QDMA2) & 0xFFFF0FFF);
	}

	/* Where an unmatched packet goes. */
	if (hwnat_wan_xsi != 0) {
		REG32(0x1FB50E48) = 349440;
		if (chip_rev == 14)
			REG32(0x1FB51E48) = 349440;
	} else {
		REG32(0x1FB50E48) = 1280;
		if (chip_rev == 14)
			REG32(0x1FB51E48) = 1280;
	}
}

/* Which IP protocols the PPE singles out. In black list mode the six it
 * names - TCP, UDP, IPv6, IPIP, ICMP, ICMPv6 - are the ones it will not
 * bind a flow for, so they go to the CPU instead of being forwarded in
 * hardware. White list mode names GRE and ESP instead. The blob picks
 * black at build time. */
static void ppe_ip_check_init(u32 blacklist)
{
	u32 chip_rev = CHIP_FAMILY;

	REG32(PPE0_IPCHK0) = 0;
	REG32(PPE0_IPCHK1) = 0;
	REG32(PPE0_IPCHK2) = 0;
	REG32(PPE0_IPCHK3) = 0;
	if (chip_rev == 14) {
		REG32(PPE0_IPCHK0 + PPE1_OFFSET) = 0;
		REG32(PPE0_IPCHK1 + PPE1_OFFSET) = 0;
		REG32(PPE0_IPCHK2 + PPE1_OFFSET) = 0;
		REG32(PPE0_IPCHK3 + PPE1_OFFSET) = 0;
	}

	if (blacklist != 0) {
		npu_printf("IP check use Black List\n");
		REG32(PPE0_IPCHK_CFG) = 0xF000F;
		REG32(PPE0_CTRL2) |= 0x10000u;
		REG32(PPE0_IPCHK0) = 0x04291106;	/* IPIP IPv6 UDP TCP */
		REG32(PPE0_IPCHK1) = 0x00003A01;	/* ICMPv6 ICMP */
		if (chip_rev == 14) {
			REG32(PPE0_IPCHK_CFG + PPE1_OFFSET) = 0xF000F;
			REG32(PPE0_CTRL2 + PPE1_OFFSET) |= 0x10000u;
			REG32(PPE0_IPCHK0 + PPE1_OFFSET) = 0x04291106;
			REG32(PPE0_IPCHK1 + PPE1_OFFSET) = 0x00003A01;
		}
	} else {
		npu_printf("IP check use White List\n");
		REG32(PPE0_IPCHK_CFG) = 0x70007;
		REG32(PPE0_CTRL2) &= ~0x10000u;
		REG32(PPE0_IPCHK0) = 0x0000322F;	/* ESP GRE */
		if (chip_rev == 14) {
			REG32(PPE0_IPCHK_CFG + PPE1_OFFSET) = 0x70007;
			REG32(PPE0_CTRL2 + PPE1_OFFSET) &= ~0x10000u;
			REG32(PPE0_IPCHK0 + PPE1_OFFSET) = 0x0000322F;
		}
	}
}

/* The PPE walks past an ethertype only if it has been told about it.
 * Sixteen slots, two to a word, with one register holding the enable
 * bits: inner in the low half, outer in the high. Without at least the
 * IPv4 slot the PPE cannot reach an IP header, so it cannot match a
 * flow and cannot decide to send an unmatched one to the CPU. */
static int ppe_ethertype_set(u32 idx, u32 enable, u32 outer, u32 ethertype)
{
	u32 chip_rev = CHIP_FAMILY;
	u32 bit, reg, val, shift;

	if (idx > 15) {
		npu_printf("Error index %d, should be 0~15!\n", idx);
		return 1;
	}

	bit = 1u << (outer != 0 ? idx + 16 : idx);
	if (enable != 0)
		REG32(PPE0_ETYPE_EN) |= bit;
	else
		REG32(PPE0_ETYPE_EN) &= ~bit;
	if (chip_rev == 14)
		REG32(PPE0_ETYPE_EN + PPE1_OFFSET) = REG32(PPE0_ETYPE_EN);

	shift = 16 * (idx & 1);
	reg = PPE0_ETYPE_TBL + 4 * (idx >> 1);
	val = (REG32(reg) & ~(0xFFFFu << shift)) | (ethertype << shift);
	REG32(reg) = val;
	if (chip_rev == 14)
		REG32(reg + PPE1_OFFSET) = val;
	return 0;
}

static void ppe_ethertype_init(void)
{
	u32 cfg = REG32(GDM_BASE) >> 16;

	ppe_ethertype_set(0, 1, 0, 0x8100);		/* VLAN */
	ppe_ethertype_set(1, 1, 0, 0x88A8);		/* QinQ */
	if (cfg != 0x8100 && cfg != 0x88A8)
		ppe_ethertype_set(2, 1, 0, cfg);

	if (hwnat_cds != 0)
		return;

	ppe_ethertype_set(3, 1, 0, 0x0800);		/* IPv4 */
	ppe_ethertype_set(4, 1, 0, 0x86DD);		/* IPv6 */
}

static void ppe_filter_config(void)
{
	u32 chip_rev = CHIP_FAMILY;

	REG32(PPE0_FLT_BASE + 0x04) = 131336144;
	REG32(PPE0_FLT_BASE + 0x08) = 131860440;
	REG32(PPE0_FLT_BASE + 0x0C) = 132384736;
	REG32(PPE0_FLT_BASE + 0x10) = 2024;

	if (chip_rev == 14) {
		REG32(PPE1_FLT_BASE + 0x04) = 131336144;
		REG32(PPE1_FLT_BASE + 0x08) = 131860440;
		REG32(PPE1_FLT_BASE + 0x0C) = 132384736;
		REG32(PPE1_FLT_BASE + 0x10) = 2024;
	}

	if (hwnat_max_packet_2000 != 0)
		REG32(0x1FB50514) |= 0x0FA40000;
	else
		REG32(0x1FB50514) |= 0x06A40000;
}

static void ppe_enable_config(void)
{
	u32 chip_rev = CHIP_FAMILY;

	if (chip_rev == 10 || chip_rev == 12 || chip_rev == 14 ||
	    chip_rev == 15 || chip_rev == 16) {
		if (chip_rev == 12 || chip_rev == 15) {
			REG32(PPE0_ENABLE) |= 1u;
			REG32(PPE0_ENABLE) |= 0x10000u;
			REG32(PPE0_ENABLE) |= 0x1000000u;
		} else {
			REG32(PPE0_ENABLE) |= 1u;
			REG32(PPE0_ENABLE) &= ~0x10000u;
			if (chip_rev == 14) {
				REG32(PPE1_ENABLE) |= 1u;
				REG32(PPE1_ENABLE) &= ~0x10000u;
			}
		}
		REG32(PPE0_ENABLE) |= 0x100u;
		if (chip_rev == 14)
			REG32(PPE1_ENABLE) |= 0x100u;

		REG32(PPE0_MISC) = (REG32(PPE0_MISC) & 0xF8FFFFFF) | 0x4000000;
		if (chip_rev == 14 && (REG32(PPE1_CTRL) & 1)) {
			REG32(PPE0_MISC) = (REG32(PPE0_MISC) & 0xF8FFFFFF) |
					   0x3000000;
			REG32(PPE1_MISC) = (REG32(PPE1_MISC) & 0xF8FFFFFF) |
					   0x3000000;
			REG32(PPE0_MISC) = (REG32(PPE0_MISC) & 0xFFFFFFF8) | 4;
			REG32(PPE1_MISC) = (REG32(PPE1_MISC) & 0xFFFFFFF8) | 5;
		} else {
			REG32(PPE0_MISC) = (REG32(PPE0_MISC) & 0xFFFFFFF8) | 6;
		}
	} else {
		REG32(PPE0_MISC) = (REG32(PPE0_MISC) & 0xFFFFFFF8) | 6;
	}
	REG32(0x1FB50E44) = 0x12345678;
	if (chip_rev == 14)
		REG32(0x1FB51E44) = 0x12345678;
}

void tunnel_init(void)
{
	u32 i, chip_rev;

	npu_printf("tunnel_init\n");
	npu_memset(tunnel_ctx, 0, sizeof(tunnel_ctx));

	for (i = 0; ; i++) {
		if (chip_cap_query(i, 0) == -1) {
			npu_printf("unknown chipid, module load fail!\n");
			ppe_module_idx = i;
			break;
		}
		if (chip_cap_query(i, 0) != 0) {
			ppe_module_idx = i;
			break;
		}
	}

	chip_cap_query(i, 2);
	chip_cap_query(i, 3);
	chip_cap_query(i, 4);
	chip_cap_query(i, 5);

	chip_rev = CHIP_FAMILY;

	if (chip_rev == 10 || chip_rev == 12)
		REG32(0x1FB50FF0) &= 0xFF7FF080;

	if (chip_rev == 10 || chip_rev == 12 || chip_rev == 14 ||
	    chip_rev == 15 || chip_rev == 16) {
		REG32(PPE0_CTRL) |= 0x40u;
		REG32(PPE0_MISC) |= 0x3000u;
		REG32(PPE0_MISC) &= ~0x10000000u;
		if (chip_rev == 10)
			REG32(0x1FB50FF0) &= ~1u;
		if (chip_rev == 14) {
			REG32(PPE1_CTRL) |= 0x40u;
			REG32(PPE1_MISC) |= 0x3000u;
			REG32(PPE1_MISC) &= ~0x10000000u;
			REG32(PPE1_MISC) |= 0x8000000u;
		}
	}

	ppe_ip_check_init(1);

	/* parser config */
	{
		u32 parser = 0x7F | 0xF00;
		if (chip_rev == 12 || chip_rev == 14 ||
		    chip_rev == 15 || chip_rev == 16)
			parser |= 0x100000;
		REG32(PPE0_PARSER) = parser;
		if (chip_rev == 14)
			REG32(PPE1_PARSER) = parser;

		if (chip_rev == 10 || chip_rev == 12 || chip_rev == 14 ||
		    chip_rev == 15 || chip_rev == 16) {
			REG32(PPE0_PARSER) &= ~0x20000u;
			if (chip_rev == 14)
				REG32(PPE1_PARSER) &= ~0x20000u;
			REG32(PPE0_PARSER) |= 0x10000u;
			if (chip_rev == 14)
				REG32(PPE1_PARSER) |= 0x10000u;
			if (chip_rev == 12 || chip_rev == 14 ||
			    chip_rev == 15 || chip_rev == 16) {
				REG32(PPE0_PARSER) |= 0xC0000u;
				if (chip_rev == 14)
					REG32(PPE1_PARSER) |= 0xC0000u;
			}
			ppe_ethertype_init();
		}
	}

	ppe_filter_config();

	/* GDM egress config */
	{
		u32 egr = REG32(PPE0_CTRL2) & 0x10000;
		if (hwnat_ppe_type == 1)
			egr |= 0x8000;
		else if (hwnat_ppe_type == 3)
			egr |= 0x6A0F7C0;
		else
			egr |= 0x620B0C0;
		REG32(PPE0_CTRL2) = egr;
		if (chip_rev == 14)
			REG32(PPE1_CTRL2) = egr;
	}

	ppe_enable_config();

	if (chip_rev == 11) {
		REG32(0x1FB50EF4) = 3146112;
		REG32(0x1FB50EF0) = 268445184;
	}

	/* PPE forwarding control bits */
	REG32(PPE0_CTRL) ^= ~(u8)REG32(PPE0_CTRL) & 2;
	REG32(PPE0_CTRL) ^= ~(u16)REG32(PPE0_CTRL) & 0x100;
	REG32(PPE0_CTRL) ^= ~(u16)REG32(PPE0_CTRL) & 0x200;
	REG32(PPE0_CTRL) ^= ~(u8)REG32(PPE0_CTRL) & 0x40;
	REG32(PPE0_CTRL) ^= ~REG32(PPE0_CTRL) & 0x1000;
	REG32(PPE0_CTRL) &= ~0x3Cu;
	if (chip_rev == 14) {
		REG32(PPE1_CTRL) ^= ~(u8)REG32(PPE1_CTRL) & 2;
		REG32(PPE1_CTRL) ^= ~(u16)REG32(PPE1_CTRL) & 0x100;
		REG32(PPE1_CTRL) ^= ~(u16)REG32(PPE1_CTRL) & 0x200;
		REG32(PPE1_CTRL) ^= ~(u8)REG32(PPE1_CTRL) & 0x40;
		REG32(PPE1_CTRL) = ((~REG32(PPE1_CTRL) & 0x1000) ^
				    REG32(PPE1_CTRL)) & 0xFFFFFFC3;
	}

	if (chip_rev == 10 || chip_rev == 12 || chip_rev == 14 ||
	    chip_rev == 15 || chip_rev == 16) {
		REG32(PPE0_CTRL) ^= ~REG32(PPE0_CTRL) & 0x8000;
		if (chip_rev == 14)
			REG32(PPE1_CTRL) ^= ~REG32(PPE1_CTRL) & 0x8000;
	}

	if (chip_rev == 11)
		REG32(PPE0_CTRL) ^= ~REG32(PPE0_CTRL) & 0x8000;

	REG32(PPE0_CTRL) ^= (REG32(PPE0_CTRL) & 1) == 0;
	REG32(PPE0_CTRL2) ^= ~REG32(PPE0_CTRL2) & 0x100000;
	REG32(PPE0_CTRL2) ^= ~REG32(PPE0_CTRL2) & 0x80000;
	if (chip_rev == 14) {
		REG32(PPE1_CTRL) ^= (REG32(PPE1_CTRL) & 1) == 0;
		REG32(PPE1_CTRL2) ^= (~((~REG32(PPE1_CTRL2) & 0x100000) ^
				       REG32(PPE1_CTRL2)) & 0x80000) ^
				     (~REG32(PPE1_CTRL2) & 0x100000);
	}

	if (chip_rev == 12 || chip_rev == 14 ||
	    chip_rev == 15 || chip_rev == 16) {
		REG32(PPE0_CTRL) ^= ~REG32(PPE0_CTRL) & 0x10000;
		REG32(PPE0_CTRL) ^= ~REG32(PPE0_CTRL) & 0x20000;
		if (chip_rev == 14) {
			REG32(PPE1_CTRL) ^= (~((~REG32(PPE1_CTRL) & 0x10000) ^
					       REG32(PPE1_CTRL)) & 0x20000) ^
					    (~REG32(PPE1_CTRL) & 0x10000);
		}
	}

	REG32(PPE0_CTRL2) &= ~0x200C0u;
	if (chip_rev == 14)
		REG32(PPE1_CTRL2) &= ~0x200C0u;

	if (chip_rev == 15 || chip_rev == 16)
		REG32(0x1FB50E58) = 0x01406082;

	ppe_qdma_config(1);
}

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

int tunnel_mail_l4s_stub(u32 base, u32 cnt)
{
	(void)base; (void)cnt;
	npu_printf("L4S not support!!!\n");
	return 1;
}

int tunnel_mail_reset(u32 base, u32 cnt)
{
	(void)base; (void)cnt;
	tunnel_ppe_reset();
	return 1;
}

/* HWNAT config the host hands over at init */
static void hwnat_set_wait_init(u32 addr)
{
	hwnat_cds = *(volatile u8 *)(addr + 8);
	hwnat_xpon_hal_api_ng = *(volatile u8 *)(addr + 9);
	hwnat_wan_xsi = *(volatile u8 *)(addr + 10);
	hwnat_ct_joyme4 = *(volatile u8 *)(addr + 11);
	hwnat_max_packet_2000 = *(volatile u8 *)(addr + 12);
	hwnat_ppe_type = REG32(addr + 16);
	hwnat_wan_mode = REG32(addr + 20);
	hwnat_ae_wan_sel = REG32(addr + 24);
	hwnat_ready = 1;
	tunnel_init();
}

/* SET_WAIT_API sub-dispatch, keyed by the _hwnat_set_func_id the host
 * puts at +8 */
static int hwnat_set_wait_api(u32 addr)
{
	u32 cmd = REG32(addr + 8);

	if (cmd > 3 || mbox_ext_handlers[cmd] == NULL) {
		npu_printf("%s not support cmd:%d\n",
			   "hwnat_mail_set_wait_api", cmd);
		return 0;
	}
	return mbox_ext_handlers[cmd](addr, 0);
}

int hwnat_mail_dispatch(u32 base, u32 cnt)
{
	u32 addr = (base & 0x3FFFFFFF) | NPU_ADDR_MASK;
	u32 func_type = *(volatile u32 *)addr;
	u32 func_id;
	int result;

	(void)cnt;
	if (func_type != 1) {
		u32 i;

		npu_printf("not support unknow funcType\n");
		for (i = 0; i < 7; i++)
			npu_printf("Offset: %08zx, Value: 0x%08x\n",
				   i * 4, *(volatile u32 *)(addr + i * 4));
		return 0;
	}

	func_id = *(volatile u32 *)(addr + 4);
	if (func_id < 1 || func_id > 5) {
		npu_printf("Error: invalid funcId! hwnat_mail_data->funcType=%u hwnat_mail_data->funcId=%u\n",
			   func_type, func_id);
		return 0;
	}

	switch (func_id) {
	case 1:			/* HWNAT_INIT */
		hwnat_set_wait_init(addr);
		result = 1;
		break;
	case 2:			/* HWNAT_DEINIT */
		hwnat_ready = 0;
		result = 1;
		break;
	case 3:			/* API */
		result = hwnat_set_wait_api(addr);
		break;
	case 4:			/* FLOW_STATS_SETUP, not implemented */
		result = 0;
		break;
	default:		/* L4S_SETUP */
		npu_printf("L4S not support!!!\n");
		result = 1;
		break;
	}

	if (result == 0)
		npu_printf("hwnat_mail_set_wait_operation fail !\n");
	return result;
}

#endif /* HAS_TUNNEL */

