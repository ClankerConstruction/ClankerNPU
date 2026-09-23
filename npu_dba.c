/*
 * AN75XX NPU firmware - GPON dynamic bandwidth allocation
 *
 * AN7583 with WiFi only. Core 5 drains the FTTR reports every frame,
 * then writes the next bandwidth map. The host mails MFUNC_DBA.
 */

#include "npu_internal.h"

#ifdef HAS_DBA

/* FTTR registers; every access is followed by a read of DBA_SYNC_REG */
#define DBA_SYNC_REG		0x1FB00020
#define FTTR_BWMAP_W0		(NPU_FTTR_BASE + 0x008)
#define FTTR_BWMAP_W1		(NPU_FTTR_BASE + 0x00C)
#define FTTR_BWMAP_CTRL		(NPU_FTTR_BASE + 0x010)
#define FTTR_INT_STS		(NPU_FTTR_BASE + 0x408)
#define FTTR_MAP_MODE		(NPU_FTTR_BASE + 0x418)
#define FTTR_RPT_WIDX		(NPU_FTTR_BASE + 0x498)
#define FTTR_RPT_BASE		(NPU_FTTR_BASE + 0x49C)
#define FTTR_RPT_RIDX		(NPU_FTTR_BASE + 0x4A0)
#define FTTR_RPT_CNT		(NPU_FTTR_BASE + 0x4A4)
#define FTTR_RPT_CFG		(NPU_FTTR_BASE + 0x4A8)
#define FTTR_RPT_SIZE		(NPU_FTTR_BASE + 0x4AC)

/* SRAM type of the report ring the FTTR block writes */
#define DBA_RPT_SRAM_TYPE	136
#define DBA_ALLOC_NUM		128

static inline u32 fttr_rd(u32 reg)
{
	u32 v = REG32(reg);

	(void)REG32(DBA_SYNC_REG);
	return v;
}

static inline void fttr_wr(u32 reg, u32 v)
{
	REG32(reg) = v;
	(void)REG32(DBA_SYNC_REG);
}

/* host mail, DBA_MAIL_Data_t */
enum {
	DBA_SET_WAIT = 1,
	DBA_SET_NO_WAIT,
	DBA_GET_WAIT,
};

enum {
	DBA_FN_PRINT_DBA_EN,
	DBA_FN_PRINT_NO_IDLE_EN,
	DBA_FN_CLEAN_NO_IDLE,
	DBA_FN_DBA_TIMER_EN,
	DBA_FN_ONU_STATE,
	DBA_FN_ONU_US_FEC,
	DBA_FN_TCONT_DBA,
	DBA_FN_TCONT_STATE,
	DBA_FN_GET_TCONT_INFO,
	DBA_FN_PLOAMU_FLAG,
	DBA_FN_SW_BWMAP_TEST,
	DBA_FN_SET_OLT_PHY_DATA,
	DBA_FN_GET_OLT_PHY_DATA,
	DBA_FN_SET_OLT_MAC_DATA,
	DBA_FN_GET_OLT_MAC_DATA,
	DBA_FN_TCONT_MAPPING_MODE,
	DBA_FN_OLT_MODE,
	DBA_FN_ALL_BAND_TO_SINGLE_ONU,
	DBA_FN_NEW_DBA_EN,
	DBA_FN_CLEAN_ONU_TX_BURST,
	DBA_FN_GET_ONU_TX_BURST,
};

struct dba_mail {
	u16 alloc_id;
	u16 fix_band;
	u16 max_band;
	u16 assure_band;
	u8 func_type;
	u8 func_id;
	u8 get_timer_en;
	u8 do_timer_en;
	u8 enable;
	u8 onu_id;
	u8 state;
	u8 sw_bwmap_test;
	u8 dba_type;
	u8 valid;
	u8 fec;
	u8 tcont_id;
	u8 tcont_map_mode;
	u8 ploamu_flag;
	u32 reg;
	u32 reg_val;
	u8 olt_mode;
	u8 all_band_to_single_onu;
};

/* one upstream report as the FTTR block writes it */
struct dba_hw_rpt {
	u16 non_idle;
	u16 bwmap_byte;
	u32 word;		/* 14:0 dbru, 26:15 alloc id, 31:27 gtc flag */
};

struct dba_rpt {
	u16 non_idle;
	u16 bwmap_byte;
	u16 dbru;
};

/* host config of one T-CONT */
struct dba_tcont_cfg {
	u8 flags;		/* bit0 valid, bit1 fec, 3:2 map mode */
	u8 pad0;
	u16 alloc_id;
	u8 dba_type;		/* 1 fix, 2 assure, 3/5 assure + more, 4 best effort */
	u8 pad1[3];
	u32 fix_bw;
	u32 assure_bw;
	u32 max_bw;
};

/* run-time state of one alloc id; 128 of them on core 5's stack */
struct dba_alloc {
	struct dba_tcont_cfg cfg;
	u8 ring_idx;		/* 3:0 read slot, 7:4 write slot */
	u8 f21;
	u8 f22;
	s8 f23;
	struct dba_rpt ring[2];
	s32 cur_bw;
	u16 f40;
	u16 f42;
	u32 f44;
	u8 state;
	u8 f49;
	u8 f50;
	u8 f51;
};

/* alloc id <-> index layout per FTTR map mode */
struct dba_map {
	u8 onu_mask;
	u8 pad;
	u16 tc_mask;
	u8 tc_shift;
	u8 idx_shift;
	u8 n_onu;
	u8 n_tc;
};

static const struct dba_map dba_map_tbl[4] = {
	{ 0x1f, 0, 0x0300, 8, 5, 32, 4 },
	{ 0x0f, 0, 0x0700, 8, 4, 16, 8 },
	{ 0x07, 0, 0x0f00, 8, 3, 8, 16 },
	{ 0x3f, 0, 0x0200, 9, 6, 64, 2 },
};

static struct {
	u8 changed;
	u8 mode;
	u8 olt_mode;
	struct dba_tcont_cfg cfg[DBA_ALLOC_NUM];
} dba_tcont;

static u32 dba_onu_tx_burst[64];
static struct dba_alloc *dba_ctx;
static u32 npu_fttr_base;

static u32 dba_frame_cnt;
static u8 dba_mode;
static u32 dba_non_idle_cnt1;
static u32 dba_non_idle_cnt2;
static u32 dba_sw_bwmap_test;

static u32 dba_rpt_tick;
static u32 dba_get_timer_en;
static u32 dba_do_timer_en;
static u32 dba_print_en;
static u32 dba_print_no_idle_en;
static u32 dba_clean_no_idle;
static u32 dba_no_idle_dump;
static u32 dba_rpt_dump;


/* ================================================================
 * FTTR access
 * ================================================================ */

static u32 dba_fttr_base_get(void)
{
	if (npu_fttr_base == 0) {
		npu_printf("error, npu_fttr_base is not init\n");
		return 0;
	}
	return npu_fttr_base;
}

/* op 0/1: counts, 2: alloc id to index, 3/5: build index/alloc id, 4: onu */
static u32 dba_aid_conv(u32 val, u32 idx, u32 mode, u32 op)
{
	const struct dba_map *m = &dba_map_tbl[mode > 3 ? 3 : (u8)mode];

	switch (op) {
	case 0:
		return m->n_onu;
	case 1:
		return m->n_tc;
	case 2:
		return (u16)((u8)((val & m->tc_mask) >> m->tc_shift) <<
			     m->idx_shift | (val & m->onu_mask));
	case 3:
		return (u16)((u8)val | idx << m->idx_shift);
	case 4:
		return val & m->onu_mask;
	case 5:
		return (u8)val | ((idx << m->tc_shift) & m->tc_mask);
	default:
		return (u16)(idx << m->idx_shift);
	}
}


/* ================================================================
 * T-CONT table
 * ================================================================ */

/* default alloc ids for the map mode the FTTR block reports */
static int dba_tcont_init(void)
{
	u32 v = fttr_rd(FTTR_MAP_MODE);
	const struct dba_map *m;
	struct dba_tcont_cfg *c;
	u32 onu;
	u8 t;

	dba_tcont.changed = 0;
	dba_tcont.mode = v;
	m = &dba_map_tbl[(u8)v > 3 ? 3 : (u8)v];

	for (onu = 0; onu < m->n_onu; onu++) {
		for (t = 0; t < m->n_tc; t++) {
			c = &dba_tcont.cfg[(u16)(t << m->idx_shift | onu)];
			c->alloc_id = (m->tc_mask & (t << m->tc_shift)) | (s16)onu;
			c->flags = (c->flags & 0xF0) | ((v & 3) << 2);
			if (t == 0) {
				c->dba_type = 1;
				c->fix_bw = 90;
				c->assure_bw = 0;
				c->max_bw = 0;
			} else {
				c->dba_type = 4;
				c->fix_bw = 0;
				c->assure_bw = 0;
				c->max_bw = 19340;
			}
		}
	}
	return 0;
}

static int dba_tcont_load(struct dba_alloc *ctx)
{
	u32 i;

	dba_tcont.changed = 0;
	for (i = 0; i < DBA_ALLOC_NUM; i++)
		npu_memcpy(&ctx[i].cfg, &dba_tcont.cfg[i], sizeof(ctx[i].cfg));
	return 0;
}

static int dba_cfg_init(void)
{
	dba_tcont_init();
	dba_tcont_load(dba_ctx);
	return 0;
}

/* onu an index belongs to under a map mode, -1 for an unknown mode */
static int dba_idx_onu(u32 i, u8 mode)
{
	switch (mode) {
	case 0:
		return i & 31;
	case 1:
		return i & 15;
	case 2:
		return i & 7;
	case 3:
		return i & 63;
	}
	return -1;
}


/* ================================================================
 * Host mail
 * ================================================================ */

static void dba_mail_set(struct dba_mail *m)
{
	struct dba_tcont_cfg *c;
	u8 mode = dba_tcont.mode;
	u32 i;

	switch (m->func_id) {
	case DBA_FN_PRINT_DBA_EN:
		dba_print_en = m->enable;
		break;
	case DBA_FN_PRINT_NO_IDLE_EN:
		dba_print_no_idle_en = m->enable;
		break;
	case DBA_FN_CLEAN_NO_IDLE:
		dba_clean_no_idle = m->enable;
		break;
	case DBA_FN_DBA_TIMER_EN:
		dba_get_timer_en = m->get_timer_en;
		dba_do_timer_en = m->do_timer_en;
		break;
	case DBA_FN_SW_BWMAP_TEST:
		dba_sw_bwmap_test = m->sw_bwmap_test;
		break;
	case DBA_FN_ONU_STATE:
		if (m->state)
			break;
		for (i = 0; i < DBA_ALLOC_NUM; i++)
			if (dba_idx_onu(i, mode) == m->onu_id)
				dba_tcont.cfg[i].flags &= ~1;
		dba_tcont.changed = 1;
		break;
	case DBA_FN_ONU_US_FEC:
		for (i = 0; i < DBA_ALLOC_NUM; i++) {
			c = &dba_tcont.cfg[i];
			if (dba_idx_onu(i, mode) == m->onu_id)
				c->flags = (c->flags & ~2) | ((m->fec & 1) << 1);
		}
		dba_tcont.changed = 1;
		break;
	case DBA_FN_TCONT_DBA:
		c = &dba_tcont.cfg[m->tcont_id];
		c->fix_bw = m->fix_band;
		c->max_bw = m->max_band;
		c->assure_bw = m->assure_band;
		c->dba_type = m->dba_type;
		dba_tcont.changed = 1;
		c->alloc_id = m->alloc_id;
		break;
	case DBA_FN_TCONT_STATE:
		c = &dba_tcont.cfg[m->tcont_id];
		dba_tcont.changed = 1;
		c->flags = (c->flags & ~1) | (m->valid & 1);
		break;
	case DBA_FN_GET_TCONT_INFO:
		c = &dba_tcont.cfg[m->tcont_id];
		npu_printf("***NPU alloc_id:%d, tcont_id:%d, fix_band:%d, max_band:%d, assure:%d, dba_type:%d, current_band:%d***\n",
			   c->alloc_id, m->tcont_id, c->fix_bw, c->max_bw,
			   c->assure_bw, c->dba_type,
			   dba_ctx[m->tcont_id].cur_bw);
		break;
	case DBA_FN_SET_OLT_PHY_DATA:
	case DBA_FN_SET_OLT_MAC_DATA:
		fttr_wr(m->reg, m->reg_val);
		break;
	case DBA_FN_TCONT_MAPPING_MODE:
		dba_tcont.changed = 1;
		dba_tcont.mode = m->tcont_map_mode;
		break;
	case DBA_FN_OLT_MODE:
		if (m->olt_mode == 1) {
			dba_burst_ovh = 75;
			dba_tcont.olt_mode = 1;
		} else {
			dba_burst_ovh = 46;
			dba_tcont.olt_mode = 0;
		}
		break;
	case DBA_FN_ALL_BAND_TO_SINGLE_ONU:
		dba_single_onu_en = m->all_band_to_single_onu;
		break;
	case DBA_FN_NEW_DBA_EN:
		dba_new_en = m->enable;
		break;
	case DBA_FN_CLEAN_ONU_TX_BURST:
		dba_onu_tx_burst[m->onu_id] = 0;
		break;
	}
}

static void dba_mail_get(struct dba_mail *m)
{
	switch (m->func_id) {
	case DBA_FN_GET_OLT_PHY_DATA:
	case DBA_FN_GET_OLT_MAC_DATA:
		m->reg_val = fttr_rd(m->reg);
		break;
	case DBA_FN_GET_ONU_TX_BURST:
		m->reg_val = dba_onu_tx_burst[m->onu_id];
		break;
	}
}

int dba_mail_handler(u32 base, u32 cnt)
{
	struct dba_mail *m = (struct dba_mail *)((base & 0x3FFFFFFF) |
						 NPU_ADDR_MASK);

	(void)cnt;

	switch (m->func_type) {
	case DBA_SET_WAIT:
		dba_mail_set(m);
		break;
	case DBA_GET_WAIT:
		dba_mail_get(m);
		break;
	default:
		npu_printf("not support unknow funcType, funcType is %d\n",
			   m->func_type);
		break;
	}
	return 1;
}


/* ================================================================
 * Upstream reports
 * ================================================================ */

/* keep the last two reports of an alloc id, dropping the oldest */
static int dba_rpt_push(struct dba_alloc *a, struct dba_rpt *r)
{
	u32 rd = a->ring_idx & 0xF;
	u32 wr = ((a->ring_idx >> 4) & 0xF) + 1;

	if (wr == 2)
		wr = 0;
	if (wr == rd) {
		rd++;
		if (rd == 2)
			rd = 0;
		rd &= 0xF;
	}
	a->ring[wr].dbru = r->dbru;
	a->ring[wr].non_idle = r->non_idle;
	a->ring[wr].bwmap_byte = r->bwmap_byte;
	a->ring_idx = rd | (wr & 0xF) << 4;
	return 0;
}

static void dba_rpt_store(struct dba_hw_rpt *hw, struct dba_rpt *r)
{
	u32 i;

	if ((hw->word & 0x7FF8000) == 0x7F8000)
		return;
	i = (u8)dba_aid_conv((u16)((hw->word >> 15) & 0xFFF), 0, dba_mode, 2);
	r->dbru = hw->word & 0x7FFF;
	r->non_idle = hw->non_idle;
	r->bwmap_byte = hw->bwmap_byte;
	dba_rpt_push(&dba_ctx[i], r);
}

static void dba_rpt_print(const char *const *fmt, struct dba_hw_rpt *hw)
{
	npu_printf(fmt[0], hw->word >> 27);
	npu_printf(fmt[1], (u16)((hw->word >> 15) & 0xFFF));
	npu_printf(fmt[2], (u16)(hw->word & 0x7FFF));
	npu_printf(fmt[3], hw->bwmap_byte);
	npu_printf(fmt[4], hw->non_idle);
}

static const char *const dba_rpt1_fmt[5] = {
	"rpt_1:gtc_flag:0x%x\n",
	"rpt_1:alloc_id:0x%x\n",
	"rpt_1:dbru:0x%x\n",
	"rpt_1:bwmap_byte:0x%x\n",
	"rpt_1:non_idle_gem_byte:0x%x\n",
};

static const char *const dba_rpt2_fmt[5] = {
	"rpt_2:gtc_flag:0x%x\n",
	"rpt_2:alloc_id:0x%x\n",
	"rpt_2:dbru:0x%x\n",
	"rpt_2:bwmap_byte:0x%x\n",
	"rpt_2:non_idle_gem_byte:0x%x\n",
};

/* drain at most 80 report pairs the FTTR block queued */
static void dba_rpt_drain(void)
{
	struct dba_rpt rpt = { 0 };
	struct dba_hw_rpt *r1, *r2;
	u32 ridx, pending, last, idx, off, n;
	int dumped = 0;

	ridx = fttr_rd(FTTR_RPT_RIDX);
	pending = fttr_rd(FTTR_RPT_CNT) & 0xFFFF;
	last = fttr_rd(FTTR_RPT_WIDX) >> 16;
	idx = ridx >> 16;
	if (!pending)
		return;

	for (n = 80; ; ) {
		off = idx * 16;
		r2 = (struct dba_hw_rpt *)(dba_fttr_base_get() + off);
		r1 = (struct dba_hw_rpt *)(dba_fttr_base_get() + off + 8);
		if (dba_rpt_dump) {
			dumped = 1;
			dba_rpt_print(dba_rpt1_fmt, r1);
			dba_rpt_print(dba_rpt2_fmt, r2);
		}
		dba_rpt_store(r1, &rpt);
		dba_rpt_store(r2, &rpt);

		if (dba_rpt_step < pending)
			idx += dba_rpt_step;
		else
			idx++;
		if (last < idx)
			idx = 0;
		ridx = (ridx & 0xFFFF) | (idx & 0xFFFF) << 16;
		fttr_wr(FTTR_RPT_RIDX, ridx);
		pending = fttr_rd(FTTR_RPT_CNT) & 0xFFFF;
		if (--n == 0 || pending == 0)
			break;
	}
	if (dba_rpt_dump && dumped)
		dba_rpt_dump = 0;
}


/* ================================================================
 * Frame handler
 * ================================================================ */

static void dba_frame_stats(void)
{
	u32 tick = dba_rpt_tick + 1;
	u32 clean = dba_clean_no_idle;

	if (dba_print_en) {
		dba_rpt_tick = tick;
		if (tick % 8000 == 0) {
			dba_print_en = 0;
			dba_rpt_tick = 0;
			dba_rpt_dump = 1;
		}
	} else if (dba_print_no_idle_en) {
		dba_rpt_tick = tick;
		if (tick == 8000) {
			dba_print_no_idle_en = 0;
			dba_rpt_tick = 0;
			dba_no_idle_dump = 1;
		}
	} else {
		dba_rpt_tick = tick;
	}

	if (clean) {
		dba_non_idle_cnt1 = 0;
		dba_non_idle_cnt2 = 0;
		dba_clean_no_idle = 0;
	}
	if (dba_no_idle_dump) {
		npu_printf("**********dba_rpt_1, non_idle_count1:%u************\n",
			   dba_non_idle_cnt1);
		npu_printf("**********dba_rpt_2, non_idle_count2:%u************\n",
			   dba_non_idle_cnt2);
		dba_no_idle_dump = 0;
	}
}

/* run once per FTTR frame interrupt */
static void dba_frame_handler(void)
{
	(void)csr_read(mcycle);
	if (dba_get_timer_en) {
		dba_frame_stats();
		if (dba_tcont.changed) {
			if (dba_tcont.mode != dba_mode) {
				dba_tcont_init();
				dba_mode = dba_tcont.mode;
			}
			dba_tcont_load(dba_ctx);
		}
		dba_rpt_drain();
	}

	if (!dba_log_en && dba_rpt_tick % 16000 == 0)
		dba_log_en = 1;
	dba_frame_cnt++;
}


/* ================================================================
 * Core 5 entry
 * ================================================================ */

void core5_dba_main(void)
{
	struct dba_alloc ctx[DBA_ALLOC_NUM];

	npu_printf("%s: start\n", __func__);
	npu_memset(ctx, 0, sizeof(ctx));
	plic_register_isr(8 + get_hartid(), mbox_isr);
	dba_ctx = npu_memset(ctx, 0, sizeof(ctx));

	npu_fttr_base = sram_buf_alloc(DBA_RPT_SRAM_TYPE);
	npu_printf("npu_fttr_base=%x\n", npu_fttr_base);

	/* point the FTTR block at the report ring and enable it */
	fttr_wr(FTTR_RPT_BASE, npu_fttr_base);
	fttr_wr(FTTR_RPT_SIZE, 0xFFF00);
	fttr_wr(FTTR_RPT_CFG, (fttr_rd(FTTR_RPT_CFG) & 0xFFF00000) |
		(npu_fttr_base >> 12));
	fttr_wr(FTTR_RPT_CFG, fttr_rd(FTTR_RPT_CFG) | 0x80000000);

	dba_cfg_init();

	while (1) {
		if (!(fttr_rd(FTTR_INT_STS) & 3))
			continue;
		fttr_wr(FTTR_INT_STS, 3);
		dba_frame_handler();
	}
}

#endif /* HAS_DBA */
