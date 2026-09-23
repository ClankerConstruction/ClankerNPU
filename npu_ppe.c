/*
 * AN75XX NPU firmware - PPE and HWNAT setup
 *
 * The host sends SET_WAIT_HWNAT_INIT with the board configuration, and
 * tunnel_init() programs the packet processing engine from it: which
 * ethertypes to parse, which IP protocols to bind a flow for, the flow
 * table scan and ageing timers, and the QDMA egress ports.
 *
 * Every register write is keyed on the chip family, which CHIP_FAMILY
 * reads at run time. Family 14 is the only one with a second PPE.
 */

#include "npu_internal.h"


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
#define PPE0_BNDR       0x1FB50E28
#define PPE0_BIND_LMT0  0x1FB50E2C
#define PPE0_BIND_LMT1  0x1FB50E30
#define PPE0_KA         0x1FB50E34
#define PPE0_UNB_AGE    0x1FB50E38
#define PPE0_BND_AGE0   0x1FB50E3C
#define PPE0_BND_AGE1   0x1FB50E40
#define PPE0_HASH_SEED  0x1FB50E44
#define PPE0_DFT_CPORT  0x1FB50E48
#define PPE0_FOE_PAUSE  0x1FB50F34
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

s32 chip_cap_query(u32 idx, u32 query)
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

#if defined(AN758X) && defined(WIFI_EAGLE)
/* Parts sold without xPON, or with GPON/EPON only, get the PON MAC
 * disabled. Runs on core 3 every 300 timer ticks. */
static s32 xpon_chip_idx = -1;

void xpon_license_check(void)
{
	u32 mode = REG32(0x1FB00070) & 0xFF;
	u32 fam = CHIP_FAMILY, rev = CHIP_REV5;
	s32 i;

	/* AN7551GT/PT: GPON/EPON only */
	if (fam == 14 && (rev == 10 || rev == 4) && (mode & 0xF7) != 0) {
		npu_printf("7551GT/PT only support GPON/EPON\n");
		goto limit_pon;
	}

	for (i = 0; chip_cap_query(i, 0) != -1; i++) {
		if (chip_cap_query(i, 0) != 0) {
			xpon_chip_idx = i;
			if (chip_cap_query(i, 1))
				return;
			goto no_xpon;
		}
	}
	npu_printf("unknow chipid, module load fail!\n");
	if (chip_cap_query(xpon_chip_idx, 1))
		return;
no_xpon:
	npu_printf("Current IC do not support XPON !!!\n");
	if (fam != 14 || (rev != 1 && rev != 11))	/* AN7566GT/PT */
		return;
	if (mode == 0 || mode == 20) {
		REG32(0x1FB640BC) = 1;
		return;
	}
limit_pon:
	if ((u8)(mode - 9) <= 4)
		REG32(0x1FB65104) = 1;
	else if ((u8)(mode - 6) <= 2 || mode == 21)
		REG32(0x1FB66080) = 0;
}
#endif

void tunnel_ppe_reset(void)
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
		REG32(PPE0_DFT_CPORT) = 349440;
		if (chip_rev == 14)
			REG32(PPE0_DFT_CPORT + PPE1_OFFSET) = 349440;
	} else {
		REG32(PPE0_DFT_CPORT) = 1280;
		if (chip_rev == 14)
			REG32(PPE0_DFT_CPORT + PPE1_OFFSET) = 1280;
	}
}

/* Which IP protocols the PPE singles out. In black list mode the six it
 * names - TCP, UDP, IPv6, IPIP, ICMP, ICMPv6 - are the ones it will not
 * bind a flow for, so they go to the CPU instead of being forwarded in
 * hardware. White list mode names GRE and ESP instead. Black is
 * picked at build time. */
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

/* CDS and the xPON HAL parse IPv6 themselves, so the slot keeps its
 * ethertype but loses its enable bit. */
static void ppe_ethertype_fixup(void)
{
	u32 chip_rev = CHIP_FAMILY;

	if ((hwnat_cds | hwnat_xpon_hal_api_ng) == 0)
		return;

	if (chip_rev == 10 || chip_rev == 12 || chip_rev == 14 ||
	    chip_rev == 15 || chip_rev == 16) {
		ppe_ethertype_set(4, 0, 0, 0x86DD);
	} else {
		REG32(GDM_PARSE0 + 0x04) &= ~8u;
		REG32(GDM_PARSE0 + 0x14) |= 0x8000000u;
		REG32(GDM_PARSE0 + 0x04) &= ~0x10u;
		REG32(GDM_PARSE0 + 0x18) =
			(REG32(GDM_PARSE0 + 0x18) & 0xFFFF0000) | 0x86DD;
	}
}

/* The flow table keeps ageing while the host walks it unless it is told
 * to hold. Only the parsers without a PPE flow-pause register need it. */
static void ppe_foe_pause(u32 pause)
{
	u32 chip_rev = CHIP_FAMILY;

	if (chip_rev == 10 || chip_rev == 12 || chip_rev == 14 ||
	    chip_rev == 15 || chip_rev == 16)
		return;

	if (pause != 0)
		REG32(PPE0_FOE_PAUSE) = 51;
	else
		REG32(PPE0_FOE_PAUSE) &= ~1u;
}

static void ppe_filter_config(void)
{
	u32 chip_rev = CHIP_FAMILY;

	if (chip_rev == 12 || chip_rev == 14 ||
	    chip_rev == 15 || chip_rev == 16) {
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
	} else {
		REG32(PPE0_FLT_BASE + 0x0C) = 131336144;
		REG32(PPE0_FLT_BASE + 0x10) = 131860440;
		REG32(PPE0_FLT_BASE + 0x14) = 2016;
	}

	if (hwnat_max_packet_2000 != 0)
		REG32(0x1FB50514) |= 0x0FA40000;
	else
		REG32(0x1FB50514) |= 0x06A40000;
}

/* PPE0 0xE50 selector fields; bits 31:28 = 3 on the newer parts */
static void ppe_tb_sel(u32 sel)
{
	u32 chip_rev = CHIP_FAMILY;

	if (chip_rev == 10 || chip_rev == 12 || chip_rev == 14 ||
	    chip_rev == 15 || chip_rev == 16) {
		REG32(PPE0_ENABLE) = (REG32(PPE0_ENABLE) & ~0xF0u) |
				     ((sel << 4) & 0xFF);
		REG32(PPE0_ENABLE) = (REG32(PPE0_ENABLE) & 0xFFFF0FFF) | 0x1000;
		if (chip_rev == 14) {
			REG32(PPE1_ENABLE) = (REG32(PPE1_ENABLE) & ~0xF0u) |
					     ((sel << 4) & 0xFF);
			REG32(PPE1_ENABLE) = (REG32(PPE1_ENABLE) & 0xFFFF0FFF) |
					     0x1000;
		}
	}
	REG32(PPE0_ENABLE) = (REG32(PPE0_ENABLE) & 0xFF0FFFFF) |
			     ((sel << 20) & 0xF00000);
	if (chip_rev == 12 || chip_rev == 14 || chip_rev == 15 ||
	    chip_rev == 16)
		REG32(PPE0_ENABLE) = (REG32(PPE0_ENABLE) & 0x0FFFFFFF) |
				     0x30000000;
}

/* Flow-table scan: turn the table walker on, tell it how many entries
 * the chip has, and seed the hash. */
static void ppe_enable_config(u32 sel)
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

		REG32(PPE0_MISC) = (REG32(PPE0_MISC) & 0xF8FFFFFF) | 0x3000000;
		if (chip_rev == 14 && (REG32(PPE1_CTRL) & 1)) {
			REG32(PPE0_MISC) = (REG32(PPE0_MISC) & 0xF8FFFFFF) |
					   0x2000000;
			REG32(PPE1_MISC) = (REG32(PPE1_MISC) & 0xF8FFFFFF) |
					   0x2000000;
		}
	}

	REG32(PPE0_MISC) = (REG32(PPE0_MISC) & 0xFFFFFFF8) | 4;
	ppe_tb_sel(sel);
	REG32(PPE0_HASH_SEED) = 0x12345678;
	if (chip_rev == 14)
		REG32(PPE0_HASH_SEED + PPE1_OFFSET) = 0x12345678;

	if (chip_rev == 10 || chip_rev == 12 || chip_rev == 14 ||
	    chip_rev == 15 || chip_rev == 16) {
		REG32(PPE0_MISC) &= ~8u;
		if (chip_rev == 14)
			REG32(PPE1_MISC) &= ~8u;
	} else {
		switch (hwnat_ppe_type) {
		case 1:
			REG32(PPE0_MISC) |= 0x10000u;
			break;
		case 2:
			REG32(PPE0_MISC) &= ~0x10000u;
			REG32(PPE0_MISC) &= ~8u;
			break;
		case 3:
			REG32(PPE0_MISC) &= ~0x10000u;
			REG32(PPE0_MISC) |= 8u;
			break;
		default:
			break;
		}
	}

	REG32(PPE0_MISC) |= 0x30u;
	if (chip_rev == 14)
		REG32(PPE1_MISC) |= 0x30u;
}

/* Ageing and keepalive for bound flows. The PPE stops handing unmatched
 * packets to the CPU once the table fills, so a flow that is never aged
 * out is a flow the CPU never sees again. */
static void ppe_flow_timer_config(void)
{
	REG32(PPE0_MISC) |= 0x80u;
	REG32(PPE0_MISC) |= 0x100u;
	REG32(PPE0_UNB_AGE) = (REG32(PPE0_UNB_AGE) & 0xFFFF) | 0x03E80000;
	REG32(PPE0_UNB_AGE) = (REG32(PPE0_UNB_AGE) & 0xFFFFFF00) | 3;
	REG32(PPE0_MISC) |= 0x200u;
	REG32(PPE0_MISC) |= 0x400u;
	REG32(PPE0_MISC) |= 0x800u;
	REG32(PPE0_BND_AGE0) = (REG32(PPE0_BND_AGE0) & ~0x7FFFu) | 0x3C;
	REG32(PPE0_BND_AGE0) = (REG32(PPE0_BND_AGE0) & ~0x7FFF0000u) | 0xF0000;
	REG32(PPE0_BND_AGE1) = (REG32(PPE0_BND_AGE1) & ~0x7FFF0000u) | 0x50000;
	REG32(PPE0_BND_AGE1) = (REG32(PPE0_BND_AGE1) & ~0x7FFFu) | 0x3C;

	REG32(PPE0_MISC) |= 0x3000u;
	REG32(PPE0_KA) = (REG32(PPE0_KA) & 0xFFFF0000) | 1;
	REG32(PPE0_KA) = (REG32(PPE0_KA) & ~0xFF0000u) | 0x10000;
	REG32(PPE0_KA) = (REG32(PPE0_KA) & ~0xFF000000u) | 0x1000000;
	REG32(PPE0_BIND_LMT1) = (REG32(PPE0_BIND_LMT1) & ~0xFF0000u) | 0x10000;

	REG32(PPE0_BIND_LMT0) = (REG32(PPE0_BIND_LMT0) & ~0x3FFFu) | 0xFA0;
	REG32(PPE0_BIND_LMT0) = (REG32(PPE0_BIND_LMT0) & ~0x3FFF0000u) |
				0x0FA00000;
	REG32(PPE0_BIND_LMT1) = (REG32(PPE0_BIND_LMT1) & ~0x3FFFu) | 0x1F40;
	REG32(PPE0_BNDR) = 30;
	REG32(PPE0_BNDR) = (REG32(PPE0_BNDR) & 0xFFFF) | 0x001E0000;
}

static void ppe1_flow_timer_config(void)
{
	REG32(PPE1_MISC) |= 0x80u;
	REG32(PPE1_MISC) |= 0x100u;
	REG32(PPE0_UNB_AGE + PPE1_OFFSET) =
		(REG32(PPE0_UNB_AGE + PPE1_OFFSET) & 0xFFFF) | 0x03E80000;
	REG32(PPE0_UNB_AGE + PPE1_OFFSET) =
		(REG32(PPE0_UNB_AGE + PPE1_OFFSET) & 0xFFFFFF00) | 3;
	REG32(PPE1_MISC) |= 0x200u;
	REG32(PPE1_MISC) |= 0x400u;
	REG32(PPE1_MISC) |= 0x800u;
	REG32(PPE0_BND_AGE0 + PPE1_OFFSET) =
		(REG32(PPE0_BND_AGE0 + PPE1_OFFSET) & ~0x7FFFu) | 0x3C;
	REG32(PPE0_BND_AGE0 + PPE1_OFFSET) =
		(REG32(PPE0_BND_AGE0 + PPE1_OFFSET) & ~0x7FFF0000u) | 0xF0000;
	REG32(PPE0_BND_AGE1 + PPE1_OFFSET) =
		(REG32(PPE0_BND_AGE1 + PPE1_OFFSET) & ~0x7FFF0000u) | 0x50000;
	REG32(PPE0_BND_AGE1 + PPE1_OFFSET) =
		(REG32(PPE0_BND_AGE1 + PPE1_OFFSET) & ~0x7FFFu) | 0x3C;

	REG32(PPE1_MISC) |= 0x3000u;
	REG32(PPE0_KA + PPE1_OFFSET) =
		(REG32(PPE0_KA + PPE1_OFFSET) & 0xFFFF0000) | 1;
	REG32(PPE0_KA + PPE1_OFFSET) =
		(REG32(PPE0_KA + PPE1_OFFSET) & ~0xFF0000u) | 0x10000;
	REG32(PPE0_KA + PPE1_OFFSET) =
		(REG32(PPE0_KA + PPE1_OFFSET) & ~0xFF000000u) | 0x1000000;
	REG32(PPE0_BIND_LMT1 + PPE1_OFFSET) =
		(REG32(PPE0_BIND_LMT1 + PPE1_OFFSET) & ~0xFF0000u) | 0x10000;

	REG32(PPE0_BIND_LMT0 + PPE1_OFFSET) =
		(REG32(PPE0_BIND_LMT0 + PPE1_OFFSET) & ~0x3FFFu) | 0xFA0;
	REG32(PPE0_BIND_LMT0 + PPE1_OFFSET) =
		(REG32(PPE0_BIND_LMT0 + PPE1_OFFSET) & ~0x3FFF0000u) |
		0x0FA00000;
	REG32(PPE0_BIND_LMT1 + PPE1_OFFSET) =
		(REG32(PPE0_BIND_LMT1 + PPE1_OFFSET) & ~0x3FFFu) | 0x1F40;
	REG32(PPE0_BNDR + PPE1_OFFSET) = 30;
	REG32(PPE0_BNDR + PPE1_OFFSET) =
		(REG32(PPE0_BNDR + PPE1_OFFSET) & 0xFFFF) | 0x001E0000;
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
		u32 parser = 0x7D;

		if (chip_rev == 10 || chip_rev == 12 || chip_rev == 14 ||
		    chip_rev == 15 || chip_rev == 16)
			parser = 0x7F;
		if (chip_rev == 12 || chip_rev == 14 ||
		    chip_rev == 15 || chip_rev == 16)
			parser |= 0x100000;
		parser |= 0xF00;
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
		} else {
			/* No PPE parser: the GDM does the ethertype match. */
			u32 cfg = REG32(GDM_BASE) >> 16;

			REG32(GDM_PARSE0) = 1;
			REG32(GDM_PARSE0 + 0x04) |= 1u;
			REG32(GDM_PARSE0 + 0x10) =
				(REG32(GDM_PARSE0 + 0x10) & 0xFFFF0000) | 0x8100;
			REG32(GDM_PARSE0 + 0x04) |= 2u;
			REG32(GDM_PARSE0 + 0x10) =
				(REG32(GDM_PARSE0 + 0x10) & 0x7757FFFF) |
				0x88A80000;
			if (cfg != 0x8100 && cfg != 0x88A8) {
				REG32(GDM_PARSE0 + 0x04) |= 4u;
				REG32(GDM_PARSE0 + 0x14) =
					(REG32(GDM_PARSE0 + 0x14) & 0xFFFF0000) |
					cfg;
			}
			if (hwnat_cds == 0) {
				REG32(GDM_PARSE0 + 0x04) |= 8u;
				REG32(GDM_PARSE0 + 0x14) |= 0x8000000u;
				REG32(GDM_PARSE0 + 0x04) |= 0x10u;
				REG32(GDM_PARSE0 + 0x18) =
					(REG32(GDM_PARSE0 + 0x18) & 0xFFFF0000) |
					0x86DD;
			}
		}
	}

	ppe_ethertype_fixup();
	ppe_foe_pause(1);
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

	ppe_enable_config(0);

	if (chip_rev == 11) {
		REG32(0x1FB50EF4) = 3146112;
		REG32(0x1FB50EF0) = 268445184;
	}

	ppe_flow_timer_config();
	if (chip_rev == 14)
		ppe1_flow_timer_config();

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
		REG32(0x1FB50E58) = 0x01404082;

	ppe_qdma_config(1);
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
