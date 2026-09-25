/*
 * AN75XX NPU firmware - field debug block
 *
 * A fixed 4 KB window in the global SRAM that the host reads and writes
 * with plain 32-bit accesses: version, per-hart heartbeat and trap
 * record, counters, trace rings and a command mailbox. docs/debug.md
 * has the layout and the commands.
 */

#include "npu_internal.h"
#include "npu_wifi.h"

#define NDBG_XSTR(x)	#x
#define NDBG_STR(x)	NDBG_XSTR(x)

enum {
	NDBG_CMD_PING = 1, NDBG_CMD_READ, NDBG_CMD_WRITE, NDBG_CMD_COPY,
	NDBG_CMD_HEXDUMP, NDBG_CMD_STATUS, NDBG_CMD_CLEAR, NDBG_CMD_TRACE,
	NDBG_CMD_BRIDGE, NDBG_CMD_SRAM, NDBG_CMD_CSR, NDBG_CMD_PROF,
};

enum { NDBG_OK, NDBG_EUNKNOWN, NDBG_EARG, NDBG_ENOTSUP };

static const char *const ndbg_names[16] = {
	"mbox", "wifi", "tdma", "tunnel", "l4s", "ppe", "dba", "sram",
	"trap", "", "", "", "", "", "", "stats",
};

extern u16 sram_alloc_table[];
extern u32 sram_alloc_offset;
extern char __data_start[], __bss_end[];
extern u8 mbox_dispatch[][80];

/* link.ld keeps the globals below this */
__asm__(".globl __ndbg_base\n.set __ndbg_base, " NDBG_STR(NDBG_BASE));

static void ndbg_sym(u32 i, u32 tag, u32 addr)
{
	ndbg->sym[i].tag = tag;
	ndbg->sym[i].addr = addr;
}

/* hart 0, before the other harts are released */
void npu_dbg_init(void)
{
	volatile u32 *p = (volatile u32 *)NDBG_BASE;
	const char *v = NPU_VERSION;
	u32 i;

	for (i = 0; i < NDBG_SIZE / 4; i++)
		p[i] = 0;
	for (i = 0; i < 47 && v[i]; i++)
		ndbg->version[i / 4] |= (u32)(u8)v[i] << (24 - 8 * (i & 3));
	ndbg->layout = NDBG_LAYOUT;
	ndbg->size = NDBG_SIZE;
	ndbg->harts = MAX_CORE_NUM;
#ifdef NPU_DATAPATH_DBG
	ndbg->print_mask = 1u << NDBG_STATS | 1u << NDBG_WIFI;
#endif

	i = 0;
	ndbg_sym(i++, NDBG_TAG('G', 'L', 'O', 'B'), (u32)__data_start);
	ndbg_sym(i++, NDBG_TAG('B', 'S', 'S', 'E'), (u32)__bss_end);
	ndbg_sym(i++, NDBG_TAG('M', 'B', 'O', 'X'), (u32)mbox_dispatch);
	ndbg_sym(i++, NDBG_TAG('S', 'R', 'A', 'M'), (u32)sram_alloc_table);
	ndbg_sym(i++, NDBG_TAG('T', 'I', 'C', 'K'), (u32)&timer_raw_tick);
#ifdef HAS_TUNNEL
	ndbg_sym(i++, NDBG_TAG('B', 'R', 'D', 'G'), (u32)&npu_bridge_base);
	ndbg_sym(i++, NDBG_TAG('T', 'U', 'N', 'F'), (u32)tunnel_func_table);
	ndbg_sym(i++, NDBG_TAG('L', '4', 'S', 'E'), (u32)&tunnel_ecn_enabled);
#endif
#ifdef WIFI_EAGLE
	ndbg_sym(i++, NDBG_TAG('E', 'D', 'B', 'G'), (u32)&dbg);
#endif
#ifdef NPU_PROFILE
	ndbg_sym(i++, NDBG_TAG('P', 'R', 'O', 'F'), (u32)&npu_prof);
#endif
#ifdef WIFI_KITE
	ndbg_sym(i++, NDBG_TAG('K', 'F', 'L', 'G'), (u32)&wifi_debug_flags);
	ndbg_sym(i++, NDBG_TAG('K', 'C', '2', 'G'), (u32)&counter_base_2g);
	ndbg_sym(i++, NDBG_TAG('K', 'C', '5', 'G'), (u32)&counter_base_5g);
#endif

	/* written last: a reader that sees the magic sees the rest */
	ndbg->magic = NDBG_MAGIC;
}

void npu_dbg_trace(u32 sub, u32 ev, u32 a, u32 b)
{
	u32 h = get_hartid();
	u32 i = ndbg->hart[h].trace_wr++ & 15;
	volatile struct ndbg_trace *t = &ndbg->trace[h][i];

	t->tick = timer_raw_tick;
	t->id = h << 28 | (sub & 0xFF) << 20 | (ev & 0xFFFFF);
	t->a = a;
	t->b = b;
	/* never from an ISR: the interrupted code may hold the printf mutex */
	if ((ndbg->print_mask & (1u << sub)) && !ndbg->hart[h].in_isr)
		npu_printf("[DBG] %s ev %x %x %x\n", ndbg_names[sub & 15],
			   ev, a, b);
}

/* oldest entry first, every hart */
static void ndbg_trace_dump(void)
{
	u32 h, n, i, w;
	volatile struct ndbg_trace *t;

	for (h = 0; h < MAX_CORE_NUM; h++) {
		w = ndbg->hart[h].trace_wr;
		n = w < 16 ? w : 16;
		for (i = w - n; i != w; i++) {
			t = &ndbg->trace[h][i & 15];
			npu_printf("[DBG] trace C%d t=%d %s ev %x %x %x\n", h,
				   t->tick, ndbg_names[(t->id >> 20) & 15],
				   t->id & 0xFFFFF, t->a, t->b);
		}
	}
}

static void ndbg_status(u32 mask)
{
	u32 h;
	volatile struct ndbg_hart *hp;

	if (mask == 0)
		mask = ~0u;
	npu_printf("[DBG] %s harts %d tick %d\n", NPU_VERSION, MAX_CORE_NUM,
		   timer_raw_tick);
	if (mask & (1u << NDBG_MBOX | 1u << NDBG_TRAP)) {
		for (h = 0; h < MAX_CORE_NUM; h++) {
			hp = &ndbg->hart[h];
			npu_printf("[DBG] C%d loop %x beat %d mails %d last %x traps %d cause %x epc %x\n",
				   h, hp->loop, hp->beat, hp->mails,
				   hp->last_mail, hp->trap_cnt, hp->mcause,
				   hp->mepc);
		}
	}
#ifdef WIFI_EAGLE
	if (mask & (1u << NDBG_WIFI))
		eagle_dbg_print();
#endif
	if (mask & (1u << NDBG_TDMA))
		npu_printf("[DBG] tdma tx_full %d glb %x ring0 sw %d hw %d\n",
			   ndbg->cnt[NC_TDMA_TX_FULL], REG32(TDMA_GLB_CFG),
			   tdma_tx_sw_idx[0],
			   REG32(TDMA_TX_RING0_DMA_IDX) & 0xFFFF);
#ifdef HAS_TUNNEL
	if (mask & (1u << NDBG_TUNNEL)) {
		u32 ch;

		npu_printf("[DBG] tunnel pkts %d vxlan %d/%d srv6 %d/%d map %d frag %d reasm %d drop %d invalid %d egress_fail %d\n",
			   ndbg->cnt[NC_TUN_PKTS], ndbg->cnt[NC_TUN_VXLAN_ENC],
			   ndbg->cnt[NC_TUN_VXLAN_DEC],
			   ndbg->cnt[NC_TUN_SRV6_ENC],
			   ndbg->cnt[NC_TUN_SRV6_END], ndbg->cnt[NC_TUN_MAP],
			   ndbg->cnt[NC_TUN_FRAG], ndbg->cnt[NC_TUN_REASM],
			   ndbg->cnt[NC_TUN_DROP], ndbg->cnt[NC_TUN_INVALID],
			   ndbg->cnt[NC_BRIDGE_EGRESS_FAIL]);
		for (ch = 0; ch < 4; ch++)
			npu_printf("[DBG] bridge ch%d waiting %d credits %d\n",
				   ch, REG32(0x1EC12050 + 4 * ch) & 0xFF,
				   (REG32(0x1EC12050 + 4 * ch) >> 8) & 0xFF);
	}
	if (mask & (1u << NDBG_L4S))
		npu_printf("[DBG] l4s %s qid %d thresh %d pkts %d marks %d qlen %d\n",
			   tunnel_ecn_enabled ? "on" : "off", l4s_qid,
			   l4s_qlen_thresh, ndbg->cnt[NC_L4S_PKTS],
			   ndbg->cnt[NC_L4S_MARKS], ndbg->cnt[NC_L4S_QLEN]);
#endif
	if (mask & (1u << NDBG_PPE))
		npu_printf("[DBG] ppe mails %d last %x\n",
			   ndbg->cnt[NC_PPE_MAILS], ndbg->cnt[NC_PPE_LAST]);
#ifdef HAS_DBA
	if (mask & (1u << NDBG_DBA))
		npu_printf("[DBG] dba mails %d frames %d\n",
			   ndbg->cnt[NC_DBA_MAILS], ndbg->cnt[NC_DBA_FRAMES]);
#endif
	if (mask & (1u << NDBG_SRAM))
		npu_printf("[DBG] sram allocs %d used %x\n",
			   ndbg->cnt[NC_SRAM_ALLOCS], sram_alloc_offset);
}

static u32 ndbg_csr(u32 idx, u32 *val)
{
	switch (idx) {
	case 0: *val = csr_read(mstatus); break;
	case 1: *val = csr_read(mie); break;
	case 2: *val = csr_read(mip); break;
	case 3: *val = csr_read(mtvec); break;
	case 4: *val = csr_read(mcycle); break;
	case 5: *val = csr_read(minstret); break;
	case 6: *val = csr_read(mhartid); break;
	default: return NDBG_EARG;
	}
	return NDBG_OK;
}

#ifdef NPU_PROFILE
volatile struct npu_prof npu_prof;

void npu_prof_add(u32 sec, u32 c0, u32 i0, u32 units)
{
	volatile struct npu_prof_sec *s = &npu_prof.sec[sec];
	u32 c = (u32)csr_read(mcycle) - c0;
	u32 i = (u32)csr_read(minstret) - i0;

	if (units == 0) {
		s->idle_calls++;
		s->idle_cycles += c;
		return;
	}
	s->calls++;
	s->units += units;
	s->cycles += c;
	s->instret += i;
	if (c > s->max_cycles)
		s->max_cycles = c;
}

/* one PC sample of the target hart into the copy buffer */
void npu_prof_sample(void)
{
	u32 b = (REG32(NPU_CSR_PC(npu_prof.target)) - npu_prof.base) >>
		npu_prof.shift;

	npu_prof.samples++;
	if (b < 256)
		ndbg->buf[b]++;
	else
		npu_prof.outside++;
}

/* 0 stop and clear, 1 clear and sample, 2 stop */
static u32 npu_prof_cmd(u32 op, u32 a1, u32 a2)
{
	volatile u32 *p = (volatile u32 *)&npu_prof;
	u32 i;

	if (op > 2 || (op == 1 && (a1 & 0xFF) >= MAX_CORE_NUM))
		return NDBG_EARG;
	npu_prof.sampler = 0;
	if (op == 2)
		return NDBG_OK;
	for (i = 0; i < sizeof(npu_prof) / 4; i++)
		p[i] = 0;
	for (i = 0; i < 256; i++)
		ndbg->buf[i] = 0;
	if (op == 1) {
		npu_prof.target = a1 & 0xFF;
		npu_prof.shift = (a1 >> 16) & 31;
		npu_prof.base = a2 ? a2 : 0x84000000;
		npu_prof.sampler = ((a1 >> 8) & 0xFF) + 1;
	}
	return NDBG_OK;
}
#endif

/* host command; runs on hart cmd_hart from its main loop */
void npu_dbg_service(u32 hart)
{
	u32 cmd = ndbg->cmd, a0 = ndbg->arg[0], a1 = ndbg->arg[1];
	u32 st = NDBG_OK, v = 0, i;

	switch (cmd) {
	case NDBG_CMD_PING:
		ndbg->ret[0] = hart;
		ndbg->ret[1] = (u32)csr_read(mcycle);
		break;
	case NDBG_CMD_READ:
		if (a0 & 3) {
			st = NDBG_EARG;
			break;
		}
		ndbg->ret[0] = *(volatile u32 *)a0;
		break;
	case NDBG_CMD_WRITE:
		if (a0 & 3) {
			st = NDBG_EARG;
			break;
		}
		*(volatile u32 *)a0 = a1;
		ndbg->ret[0] = *(volatile u32 *)a0;
		break;
	case NDBG_CMD_COPY:
		if ((a0 & 3) || a1 > sizeof(ndbg->buf)) {
			st = NDBG_EARG;
			break;
		}
		for (i = 0; i < (a1 + 3) / 4; i++)
			ndbg->buf[i] = ((volatile u32 *)a0)[i];
		ndbg->ret[0] = i * 4;
		break;
	case NDBG_CMD_HEXDUMP:
		npu_hexdump("dbg", a0 & ~3u, a1 ? a1 : 64);
		break;
	case NDBG_CMD_STATUS:
		ndbg_status(a0);
		break;
	case NDBG_CMD_CLEAR:
		for (i = 0; i < NC_MAX; i++)
			ndbg->cnt[i] = 0;
		for (i = 0; i < MAX_CORE_NUM; i++)
			ndbg->hart[i].trace_wr = 0;
		break;
	case NDBG_CMD_TRACE:
		ndbg_trace_dump();
		break;
#ifdef HAS_TUNNEL
	case NDBG_CMD_BRIDGE:
		npu_bridge_debug(a0);
		break;
#endif
	case NDBG_CMD_SRAM:
		sram_buf_dump();
		break;
	case NDBG_CMD_CSR:
		st = ndbg_csr(a0, &v);
		ndbg->ret[0] = v;
		break;
#ifdef NPU_PROFILE
	case NDBG_CMD_PROF:
		st = npu_prof_cmd(a0, a1, ndbg->arg[2]);
		break;
#endif
	default:
		st = cmd <= NDBG_CMD_CSR ? NDBG_ENOTSUP : NDBG_EUNKNOWN;
		break;
	}
	ndbg->status = st;
	ndbg->done++;
	/* cleared last: the host polls cmd for 0 */
	ndbg->cmd = 0;
}

/* harts with nothing else to run serve commands from here */
void __attribute__((noreturn)) npu_dbg_idle(void)
{
	npu_dbg_loop(NDBG_TAG('I', 'D', 'L', 'E'));
	while (1) {
		npu_dbg_poll();
		delay_us(10);
	}
}
