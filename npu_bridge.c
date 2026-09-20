/*
 * AN75XX NPU firmware - NPU bridge
 *
 * Four DMA channels between the NPU and the host packet buffers.
 */

#include "npu_internal.h"


/* ================================================================
 * NPU bridge (DMA channels between NPU and host)
 * ================================================================ */

#define NPU_BRIDGE_REG_BASE    0x1EC12000
#define BRIDGE_CH_CTRL(ch)     (NPU_BRIDGE_REG_BASE + 0x010 + (ch) * 0x10)
#define BRIDGE_CH_STATUS(ch)   (NPU_BRIDGE_REG_BASE + 0x050 + (ch) * 0x20)
#define BRIDGE_PKT_BUF_BASE_REG 0x1EC12010
#define BRIDGE_PKT_BUF_CFG    0x1EC12018

u32 npu_bridge_pkt_base;
static u32 bridge_tx_count[4];

u32 npu_bridge_addr(void)
{
	return npu_bridge_base + 0x10000;
}

void npu_bridge_buf_init(void)
{
	u32 i;
	u32 *ch_status;

	npu_bridge_base = sram_buf_alloc(129);
	npu_printf("npuBridgeBase=%x\n", npu_bridge_base);
	npu_bridge_pkt_base = npu_bridge_base;
	npu_printf("%s: NPU_BRIDGE_BASE_PACKET_BUFFER(0x%x)=0x%x, NPU_BRIDGE_PACKET_BUF_SIZE:0x%x\n",
		   "npu_bridge_buf_init",
		   (u32)&npu_bridge_pkt_base,
		   npu_bridge_base,
		   0x10000);

	REG32(BRIDGE_PKT_BUF_BASE_REG) = 264192;
	REG32(BRIDGE_PKT_BUF_CFG) = 1;

	delay_1ms(10);

	for (i = 0; i < 4; i++) {
		ch_status = (u32 *)(0x1EC12210 + i * 16);
		if (*ch_status & 1)
			npu_printf("npu bridge channel-%d buf init sucess\n", i);
		else
			npu_printf("npu bridge channel-%d buf init fail\n", i);
		*ch_status = 1;
	}
}

static int npu_bridge_ingress(u32 ch, u32 *pkt_ptr, u32 *desc_ptr)
{
	u32 cnt;

	cnt = bridge_tx_count[ch];
	if (cnt == 0) {
		cnt = (u8)REG32(0x1EC12050 + ch * 4);
		bridge_tx_count[ch] = cnt;
		if (cnt == 0)
			return -1;
	}

	*desc_ptr = REG32(0x1EC12080 + ch * 16) | 0x20000000;
	*pkt_ptr = *(volatile u32 *)(*desc_ptr) + 32;
	bridge_tx_count[ch] = cnt - 1;
	return 0;
}

static int npu_bridge_egress(u32 ch, u32 w0, u32 w1, u32 w2,
			     u32 w3, u32 w4, u32 w5, u32 w6)
{
	u32 base;

	if ((REG32(0x1EC12050 + ch * 4) & 0xFF00) == 0) {
		npu_printf("npu bridge egress fail, channel-%d, epkt_info_w0=%x, epkt_info_w1=%x, epkt_info_w2=0x%x \n",
			   ch, w0, w1, w2);
		return -1;
	}

	base = 0x1EC12100 + ch * 32;
	REG32(base + 0x04) = w1;
	REG32(base + 0x08) = w2;
	REG32(base + 0x0C) = w3;
	REG32(base + 0x10) = w4;
	REG32(base + 0x14) = w5;
	REG32(base + 0x18) = w6;
	REG32(base + 0x00) = w0;
	return 0;
}

static int npu_bridge_send(u32 ch, u32 w0, u32 fwd, u32 len, u32 flags)
{
	if (flags != 0)
		return 0;
	return npu_bridge_egress(ch, w0, len << 16,
				 (ch & 7) | 0xC0000000 | ((fwd != 0) << 24),
				 0, 0, 0, 0);
}

static int npu_bridge_send_direct(u32 ch, u32 data, u32 len)
{
	return npu_bridge_egress(ch, data, len << 16,
				 (ch & 7) | 0xC2000000,
				 0, 0, 0, 0);
}
