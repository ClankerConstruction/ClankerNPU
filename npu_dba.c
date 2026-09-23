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

/* upstream bwmap entry: ctrl word to FTTR_BWMAP_W1, times to W0 */
#define BWMAP_DBRU_MASK		(3 << 1)
#define BWMAP_DBRU		(1 << 1)
#define BWMAP_FEC		(1 << 3)
#define BWMAP_PLOAMU		(1 << 4)
#define BWMAP_AID_MASK		0x0003FFC0
#define BWMAP_AID(x)		(((x) & 0xFFF) << 6)
#define BWMAP_OLT		(1 << 25)
#define BWMAP_END_MASK		(3 << 27)
#define BWMAP_END_WRAP		(1 << 27)	/* map goes on in the next frame */
#define BWMAP_END_LAST		(3 << 27)	/* last entry of the map */
#define BWMAP_W0(start, stop)	((u32)(start) << 16 | (stop))

/* upstream frame length in bwmap units */
#define DBA_FRAME_LEN		0x4BC2
#define DBA_STOP_MAX		(DBA_FRAME_LEN - 1)

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
	u8 step_cnt;		/* states 1, 2 and 5 */
	u8 grow_cnt;		/* state 4 */
	s8 hold_cnt;		/* state 3 */
	struct dba_rpt ring[2];
	s32 cur_bw;		/* grant for the next map */
	u16 ref_dbru;
	u16 pad;
	u32 idle_avg;		/* running average of unused grant */
	u8 state;		/* 2:0 state, 5:3 group, bit 6 last step went down */
	u8 pad1[3];
};

/* upstream budget of one map, filled by dba_budget_calc */
struct dba_budget {
	u16 total;
	u16 pad;
	s32 remain;
	u16 cnt;
	u16 share;
	u32 demand;
	u32 over_share;
	s32 left;
	u32 over_assure;
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

static struct {
	u32 ctrl;
	u16 stop;
	u16 start;
} dba_bwmap;
static u16 dba_bwmap_pos;
static u8 dba_bwmap_nonempty;
static u32 npu_fttr_base;

static u32 dba_frame_cnt;
static u8 dba_mode;
static u8 dba_bwmap_left;
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

static u8 dba_act_onus;
static u8 dba_dual_frame;


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

static int dba_bwmap_write(u32 w0, u32 w1)
{
	fttr_wr(FTTR_BWMAP_W0, w0);
	fttr_wr(FTTR_BWMAP_W1, w1);
	return 0;
}

/* bit 0: bank the FTTR block uses, bit 1: bank requested */
static int dba_bwmap_bank_get(u32 *hw, u32 *sw)
{
	u32 v = fttr_rd(FTTR_BWMAP_CTRL);

	*hw = v & 1;
	*sw = (v >> 1) & 1;
	return 0;
}

static int dba_bwmap_bank_set(u32 bank)
{
	u32 v = fttr_rd(FTTR_BWMAP_CTRL);

	fttr_wr(FTTR_BWMAP_CTRL, (v & ~2) | (bank & 1) << 1);
	return 0;
}

static int dba_bwmap_empty_set(u32 empty)
{
	u32 v = fttr_rd(FTTR_BWMAP_CTRL);

	fttr_wr(FTTR_BWMAP_CTRL, (v & ~4) | (empty & 1) << 2);
	return 0;
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
 * Bandwidth budget
 * ================================================================ */

/* upstream bytes left once fixed and assured grants are taken */
static int dba_budget_calc(struct dba_alloc *ctx, struct dba_budget *b)
{
	u8 mode = dba_tcont.mode;
	const struct dba_map *m = &dba_map_tbl[mode > 3 ? 3 : mode];
	u8 olt = dba_tcont.olt_mode;
	u8 dual = dba_dual_frame;
	u8 n_full = 0, n_act = 0;
	s32 sum = 0;
	struct dba_alloc *a;
	u16 onu, ovh;
	int first;
	u8 t;

	for (onu = 0; onu < m->n_onu; onu++) {
		if ((ctx[onu].cfg.flags & 3) == 3)
			n_full++;
		first = 1;
		for (t = 0; t < m->n_tc; t++) {
			a = &ctx[(u16)((s16)onu | t << m->idx_shift)];
			if (!(a->cfg.flags & 1))
				continue;
			if (first)
				n_act++;
			first = 0;
			switch (a->cfg.dba_type) {
			case 1:
				sum += a->cfg.fix_bw;
				break;
			case 2:
				sum += a->cfg.assure_bw;
				break;
			case 3:
			case 5:
				b->cnt++;
				sum += a->cfg.assure_bw;
				break;
			case 4:
				b->cnt++;
				break;
			}
		}
	}

	if (olt) {
		ovh = (u16)dba_burst_ovh;
	} else {
		ovh = n_full ? 64 : 46;
		dba_burst_ovh = ovh;
	}
	if (dual)
		b->total = (u16)(0x97E0 - ovh * dba_act_onus);
	else
		b->total = (u16)(0x4B8C - n_full * 36 - (u16)(ovh * n_act));
	b->remain = b->total - sum;
	if (b->remain > 0 && b->cnt)
		b->share = b->remain / b->cnt;
	else
		b->share = 0;
	return 0;
}


/* ================================================================
 * Per alloc id grants
 * ================================================================ */

/* insertion step: sink list[start + n - 1] below larger grants */
static int dba_sort_bw(struct dba_alloc *ctx, u16 *list, u32 n, int start)
{
	u16 last;
	s32 key;
	int i;

	if (n <= 1)
		return 0;
	last = list[start + n - 1];
	key = (u16)ctx[last].cur_bw;
	for (i = start + n - 2; i >= start; i--) {
		if (key >= ctx[list[i]].cur_bw)
			break;
		list[i + 1] = list[i];
		list[i] = last;
	}
	return 0;
}

#define DBA_ST_MASK	0x07	/* state bits 2:0 */
#define DBA_ST_DEC	0x40	/* last step decreased cur_bw */
#define DBA_GRP_MASK	0x38	/* bits 5:3: 8 above share, 0x10 above assured */
#define DBA_GRP_SHARE	0x08
#define DBA_GRP_ASSURE	0x10

static inline void dba_set_state(struct dba_alloc *a, u8 st)
{
	a->state = (a->state & ~DBA_ST_MASK) | st;
}

/* adapt a->cur_bw from the read and write report slots */
static int dba_alloc_adapt(struct dba_alloc *a)
{
	u32 w = a->ring_idx;
	u8 dual = dba_dual_frame;
	u32 wr = (w >> 4) & 15;
	u32 rd = w & 15;
	struct dba_rpt *cur = &a->ring[wr];
	struct dba_rpt *prev = &a->ring[rd];
	u16 gap_r = 0, gap_w = 0, mx, mid;
	u32 need, acc, avg, d, lim, mode;
	s32 bw;
	u8 type;

	if (dual) {
		int ni = cur->non_idle + prev->non_idle;
		int bwb = cur->bwmap_byte + prev->bwmap_byte;

		if (!prev->dbru && ni < bwb)
			gap_r = bwb - ni;
		if (!cur->dbru && ni < bwb)
			gap_w = bwb - ni;
		need = cur->dbru * 48;
		if (ni > 7 || (a->state & DBA_ST_MASK) != 3)
			acc = a->idle_avg + gap_r + gap_w;
		else
			acc = a->idle_avg + (gap_r >> 3) + (u16)(gap_w >> 3);
	} else {
		if (rd == wr) {
			if (!a->cur_bw)
				a->cur_bw = 2;
			return 0;
		}
		need = cur->dbru * 48;
		if (!prev->dbru && prev->non_idle < prev->bwmap_byte)
			gap_r = prev->bwmap_byte - prev->non_idle;
		if (!cur->dbru && cur->non_idle < cur->bwmap_byte)
			gap_w = cur->bwmap_byte - cur->non_idle;
		if (prev->non_idle > 7 || (a->state & DBA_ST_MASK) != 3)
			a->idle_avg += gap_r;
		else
			a->idle_avg += gap_r >> 3;
		if (cur->non_idle > 7 || (a->state & DBA_ST_MASK) != 3)
			acc = a->idle_avg + gap_w;
		else
			acc = a->idle_avg + (u16)(gap_w >> 3);
	}
	avg = acc >> 1;
	a->idle_avg = avg;

	switch (a->state & DBA_ST_MASK) {
	case 1:
		bw = a->cur_bw;
		if (gap_w | gap_r) {
			a->step_cnt = 2;
			dba_set_state(a, 2);
			d = (avg << 1) >> 3;
			if (dba_tcont.olt_mode)
				d = (avg << 1) >> 4;
			bw -= d;
			a->cur_bw = bw;
			a->state |= DBA_ST_DEC;
			break;
		}
		mx = prev->dbru;
		if (mx < cur->dbru)
			mx = cur->dbru;
		if (a->ref_dbru < mx) {
			d = (u32)(mx - a->ref_dbru) * 48;
			a->ref_dbru = mx;
			a->step_cnt = 0;
			d = dba_tcont.olt_mode ? d >> 2 : d >> 1;
			bw += d;
			a->cur_bw = bw;
			break;
		}
		lim = bw * 2;
		if ((s32)lim < 2000)
			lim = 2000;
		if (need + lim >= (u32)(bw * 10)) {
			d = bw;
			if (bw < 1000)
				d = 1000;
			a->step_cnt = 0;
			if (dba_tcont.olt_mode)
				d >>= 1;
			bw += d;
			a->cur_bw = bw;
			break;
		}
		if (cur->dbru < prev->dbru) {
			d = prev->dbru - cur->dbru;
			if (cur->dbru < d)
				d -= cur->dbru;
			d = (d * 48) >> 3;
			a->step_cnt = 2;
			if (dba_tcont.olt_mode)
				d >>= 1;
			bw -= d;
			a->cur_bw = bw;
		}
		dba_set_state(a, 2);
		a->ref_dbru = mx;
		break;
	case 2:
		a->step_cnt += 2;
		mid = (cur->dbru + prev->dbru) >> 1;
		if (acc > 1)
			a->state |= DBA_ST_DEC;
		if (a->ref_dbru < mid || a->step_cnt > 15) {
			dba_set_state(a, 3);
			a->ref_dbru = cur->dbru;
			a->idle_avg = 0;
			a->step_cnt = 0;
			a->grow_cnt = 0;
			a->hold_cnt = 0;
			d = 0;
		} else if (a->state & DBA_ST_DEC) {
			d = (avg << 1) / a->step_cnt;
		} else if (cur->dbru >= prev->dbru) {
			d = (u32)(a->ref_dbru - mid) * 48;
			d = d > 127 ? d >> 3 : 16;
			a->ref_dbru = mid;
		} else {
			d = prev->dbru - cur->dbru;
			if (cur->dbru < d)
				d -= cur->dbru;
			d = (d * 48) >> 3;
			a->ref_dbru = mid;
		}
		bw = a->cur_bw;
		if (dba_tcont.olt_mode)
			bw -= d >> 1;
		else
			bw -= d;
		a->cur_bw = bw;
		break;
	case 3:
		mx = cur->dbru;
		if (mx < prev->dbru)
			mx = prev->dbru;
		bw = a->cur_bw;
		if (bw * 10 < mx * 48) {
			a->grow_cnt = 2;
			dba_set_state(a, 4);
			a->ref_dbru = 0;
		} else {
			a->grow_cnt = 0;
		}
		if ((a->state & DBA_ST_MASK) == 3 && gap_w > 16 && gap_r > 16) {
			dba_set_state(a, 5);
			a->step_cnt = 2;
		}
		if (a->hold_cnt < 0)
			break;
		a->hold_cnt += 2;
		if (a->state & DBA_ST_DEC) {
			if (acc <= 1)
				break;
		} else {
			if (mx) {
				lim = bw * 2;
				if ((s32)lim < 2000)
					lim = 2000;
				if (bw * 10 < (s32)(mx * 48 + lim))
					break;
			}
			if (acc <= 1) {
				if (mx < a->ref_dbru) {
					bw -= dba_tcont.olt_mode ? 8 : 16;
					a->cur_bw = bw;
				}
				break;
			}
		}
		d = avg;
		if (d > 16)
			d = 16;
		if (dba_tcont.olt_mode)
			d >>= 1;
		bw -= d;
		a->cur_bw = bw;
		a->idle_avg = 0;
		a->state |= DBA_ST_DEC;
		break;
	case 4:
		bw = a->cur_bw;
		if (bw * 10 >= cur->dbru * 48) {
			if (cur->dbru <= 0x3ffe) {
				a->grow_cnt = 0;
				dba_set_state(a, 3);
				break;
			}
			d = 0;
		} else {
			a->grow_cnt += 2;
			if (a->grow_cnt <= 7)
				break;
			d = cur->dbru * 48 - bw * 10;
			if (a->ref_dbru) {
				if (a->ref_dbru < cur->dbru)
					d = (cur->dbru - a->ref_dbru) * 48;
				else
					d = bw / 10 + 16;
			}
			lim = bw * 2;
			if ((s32)lim < 2000)
				lim = 2000;
			if (lim + bw * 2 < d) {
				d >>= 3;
				dba_set_state(a, 1);
			} else {
				d >>= 3;
				if ((u32)(bw / 10 + 16) < d)
					d = bw / 10 + 16;
			}
			d = avg < d ? d - avg : 0;
		}
		a->idle_avg = 0;
		if (dba_tcont.olt_mode)
			d >>= 1;
		bw += d;
		a->cur_bw = bw;
		a->step_cnt = 0;
		a->state &= ~DBA_ST_DEC;
		a->grow_cnt = 0;
		a->hold_cnt = 0;
		a->ref_dbru = cur->dbru;
		break;
	case 5:
		bw = a->cur_bw;
		if (gap_w <= 16 || gap_r <= 16) {
			a->step_cnt = 0;
			dba_set_state(a, 3);
			break;
		}
		a->step_cnt += 2;
		if ((a->state & DBA_ST_MASK) != 5 || a->step_cnt <= 7)
			break;
		d = bw / 10 + 16;
		if (d * 2 < (acc >> 4)) {
			dba_set_state(a, 2);
			a->step_cnt = 0;
		} else if ((acc >> 4) < d) {
			d = acc >> 4;
		}
		if (dba_tcont.olt_mode)
			d >>= 1;
		bw -= d;
		a->cur_bw = bw;
		mx = cur->dbru;
		if (prev->dbru < mx)
			mx = prev->dbru;
		a->ref_dbru = mx;
		a->idle_avg = 0;
		a->grow_cnt = 0;
		a->hold_cnt = 0;
		a->state |= DBA_ST_DEC;
		break;
	default:
		dba_set_state(a, 1);
		bw = need >> 1;
		if (dba_tcont.olt_mode)
			bw = need >> 2;
		a->cur_bw = bw;
		a->ref_dbru = cur->dbru;
		break;
	}

	/* clamp to the T-CONT contract */
	if (bw <= 8)
		a->state &= ~DBA_ST_MASK;
	if (bw < 2)
		bw = 2;
	type = a->cfg.dba_type;
	a->cur_bw = bw;
	if ((u8)(type - 3) <= 2) {
		if (dual) {
			if ((u32)bw >= a->cfg.max_bw * 2)
				bw = a->cfg.max_bw * 2;
		} else if ((u32)bw > a->cfg.max_bw) {
			bw = a->cfg.max_bw;
		}
		a->cur_bw = bw;
		if (type == 5 && (u32)bw < a->cfg.fix_bw)
			a->cur_bw = a->cfg.fix_bw;
	} else if (type == 2) {
		if ((u32)bw >= a->cfg.assure_bw)
			bw = a->cfg.assure_bw;
		a->cur_bw = bw;
	}

	/* report consumed: read slot catches up with write slot */
	a->ring_idx = (a->ring_idx & 0xf0) | wr;
	mode = (a->cfg.flags >> 2) & 3;
	if (mode == 0 || mode == 1 || mode == 2) {
		if (a->cfg.alloc_id & ~31)
			return 0;
	} else if (a->cfg.alloc_id & ~63) {
		return 0;
	}
	bw = a->cur_bw;
	if (dual) {
		if (bw <= 14)
			a->cur_bw = 15;
		else if (bw > 192)
			a->cur_bw = 192;
	} else if (bw < 27) {
		a->cur_bw = 27;
	}
	return 0;
}

/* per-alloc demand into b, then fair share of the rest */
static int dba_bw_request(struct dba_alloc *ctx, struct dba_budget *b)
{
	u16 list[128];
	struct dba_alloc *a = ctx, *p;
	s16 n = 0, n_over = 0;
	u16 cnt, share;
	s32 bw, rem;
	int i, k, j, first;
	u8 st;

	npu_memset(list, 0, sizeof(list));
	dba_bwmap_left = 0;
	for (i = 0; i < 128; i++, a++) {
		if (!(a->cfg.flags & 1))
			continue;
		dba_bwmap_left++;
		if (a->cfg.dba_type != 1)
			dba_alloc_adapt(a);
		switch (a->cfg.dba_type) {
		case 1:
			bw = a->cfg.fix_bw;
			if (dba_dual_frame)
				bw <<= 1;
			a->cur_bw = bw;
			break;
		case 2:
			bw = a->cur_bw;
			break;
		case 3:
		case 5:
			bw = a->cur_bw;
			if ((u32)bw <= a->cfg.assure_bw || bw <= b->share)
				break;
			a->state = (a->state & ~DBA_GRP_MASK) | DBA_GRP_SHARE;
			list[n] = i;
			n++;
			dba_sort_bw(ctx, list, (u16)n, 0);
			bw = a->cur_bw;
			b->over_share += bw;
			if (b->share >= a->cfg.assure_bw)
				break;
			b->over_assure += bw - a->cfg.assure_bw;
			n_over++;
			a->state = (a->state & ~DBA_GRP_MASK) | DBA_GRP_ASSURE;
			break;
		case 4:
			bw = a->cur_bw;
			if (bw <= b->share) {
				a->state &= ~DBA_GRP_MASK;
				break;
			}
			a->state = (a->state & ~DBA_GRP_MASK) | DBA_GRP_SHARE;
			list[n] = i;
			n++;
			dba_sort_bw(ctx, list, (u16)n, 0);
			bw = a->cur_bw;
			b->over_share += bw;
			break;
		default:
			a->cur_bw = 2;
			bw = 2;
			break;
		}
		b->demand += bw;
	}

	if (b->demand <= b->total)
		return 0;
	rem = b->total - b->demand + b->over_share;
	b->left = rem;
	if (rem <= 0) {
		for (k = 0; k < n; k++)
			ctx[list[k]].cur_bw = 0;
		return 0;
	}

	/* list is sorted by cur_bw, smallest first */
	cnt = n;
	if (!cnt)
		return 0;
	first = 0;
	for (k = 0; k < cnt; ) {
		if (rem < 0)
			rem = 0;
		share = n > 0 ? rem / n : rem;
		p = &ctx[(u8)list[k]];
		st = p->state & DBA_GRP_MASK;
		if (share < p->cur_bw && !first) {
			/* first overflow: assured-only entries drop back */
			for (j = k; j < cnt; j++) {
				struct dba_alloc *q = &ctx[(u8)list[j]];

				if ((q->state & DBA_GRP_MASK) != DBA_GRP_ASSURE)
					continue;
				if (q->cfg.assure_bw >= share)
					continue;
				n_over--;
				b->over_assure -= (u16)(q->cur_bw - q->cfg.assure_bw);
				q->state = (q->state & ~DBA_GRP_MASK) | DBA_GRP_SHARE;
			}
			n -= n_over > 0 ? n_over : 0;
			rem += b->over_assure;
			first = 1;
			continue;
		}
		if (first) {
			if (st == DBA_GRP_ASSURE) {
				bw = p->cfg.assure_bw;
			} else {
				n--;
				bw = share;
			}
			p->cur_bw = bw;
			rem -= bw;
		} else {
			if (st == DBA_GRP_ASSURE) {
				b->over_assure -= (u16)(p->cur_bw - p->cfg.assure_bw);
				n_over--;
			}
			n--;
			rem -= p->cur_bw;
		}
		k++;
	}
	b->left = rem;
	return 0;
}


/* ================================================================
 * Bandwidth map
 * ================================================================ */

/* stage one bwmap entry for alloc @idx and write it */
static int dba_config_bwmap_handler(struct dba_alloc *ctx, u16 idx, u32 fec,
				    u16 slot, u16 onu)
{
	struct dba_alloc *a;
	u16 start = dba_bwmap_pos;
	u16 end, stop;
	u32 ctrl;

	/* the first T-CONT of a burst pays the burst overhead */
	if (!(slot & 3) && start)
		start += dba_burst_ovh;

	a = &ctx[idx];
	dba_bwmap.start = start;
	end = a->cur_bw + start;
	stop = end - 1;
	dba_bwmap.stop = stop;
	dba_bwmap_pos = end;

	if (stop > DBA_STOP_MAX) {
		if (dba_log_en) {
			dba_log_en = 0;
			npu_printf("****config_bwmap_handler failed1, st:%d, ss:%d, alloc_id:0x%x****\n",
				   stop, start, a->cfg.alloc_id);
		}
		return 0;
	}
	if (start >= stop) {
		if (dba_log_en) {
			dba_log_en = 0;
			npu_printf("****config_bwmap_handler failed2, st:%d, ss:%d, alloc_id:0x%x****\n",
				   stop, start, a->cfg.alloc_id);
		}
		return 0;
	}

	ctrl = dba_bwmap.ctrl;
	ctrl = (ctrl & ~BWMAP_DBRU_MASK) | ((a->cfg.dba_type != 1) << 1);
	ctrl = (ctrl & ~BWMAP_PLOAMU) | ((idx == onu) << 4);
	ctrl = (ctrl & ~BWMAP_FEC) | ((fec != 0) << 3);
	dba_bwmap.ctrl = ctrl;
	ctrl = (ctrl & ~BWMAP_AID_MASK) | BWMAP_AID(a->cfg.alloc_id);
	dba_bwmap.ctrl = ctrl;
	if (dba_tcont.olt_mode == 1)
		dba_bwmap.ctrl = ctrl | BWMAP_OLT;

	dba_bwmap_left--;
	ctrl = dba_bwmap.ctrl & ~BWMAP_END_LAST;
	if (!dba_bwmap_left)
		ctrl |= BWMAP_END_LAST;
	dba_bwmap.ctrl = ctrl;
	dba_bwmap_write(BWMAP_W0(dba_bwmap.start, dba_bwmap.stop), ctrl);
	return 0;
}

/* build the bwmap of all ONUs, then switch the FTTR bank */
static int dba_bwmap_switch(struct dba_alloc *ctx)
{
	struct dba_alloc *a;
	u16 list[32];
	u32 b0, b1;
	u8 old, mode, n_onu, n_tcont, j, olt;
	u16 idx, cnt, k, acc, delta;
	s16 onu;
	int last, rem;

	npu_memset(list, 0, sizeof(list));
	old = dba_bwmap_nonempty;
	dba_bwmap_nonempty = dba_bwmap_left != 0;
	b0 = 0;
	b1 = 0;
	dba_bwmap_pos = 0;

	mode = dba_tcont.mode;
	if (mode > 3)
		mode = 3;
	n_onu = dba_map_tbl[mode].n_onu;
	n_tcont = dba_map_tbl[mode].n_tc;

	for (onu = 0; onu < n_onu; onu++) {
		u8 flags = ctx[onu].cfg.flags;

		if (!(flags & 1))
			continue;
		dba_onu_tx_burst[onu]++;

		if (!(flags & 2)) {
			/* plain order: T-CONT 0..n-1 of this ONU */
			for (j = 0; j < n_tcont; j++) {
				idx = onu | (j << dba_map_tbl[mode].idx_shift);
				a = &ctx[idx];
				if (!(a->cfg.flags & 1))
					continue;
				if (!a->cur_bw) {
					if (dba_log_en) {
						dba_log_en = 0;
						npu_printf("**********max_req_bw == 0**********\n");
					}
					continue;
				}
				dba_config_bwmap_handler(ctx, idx, 0, j, onu);
			}
			continue;
		}

		/* FEC on: sort by bandwidth, keep grants off RS(255,239) parity */
		npu_memset(list, 0, sizeof(list));
		cnt = 0;
		for (j = 0; j < n_tcont; j++) {
			idx = onu | (j << dba_map_tbl[mode].idx_shift);
			a = &ctx[idx];
			if (!(a->cfg.flags & 1) || !a->cur_bw)
				continue;
			list[cnt] = idx;
			dba_sort_bw(ctx, list, cnt, 1);
			cnt++;
		}
		if (!cnt)
			continue;

		olt = dba_tcont.olt_mode;
		acc = 0;
		for (k = 0; k < cnt; k++) {
			a = &ctx[list[k]];
			last = k == cnt - 1;
			delta = 0;
			if (a->cur_bw <= 1) {
				delta = 2 - a->cur_bw;
				a->cur_bw = 2;
			}
			if (last && a->cur_bw < 18)
				a->cur_bw = 18;

			if (olt) {
				u32 len = a->cur_bw * 2;

				rem = (acc + len + 2) % 255;
				if (rem > 238) {
					if (!last) {
						len = len - rem + 255;
						if (len & 1) {
							delta += (256 - rem) >> 1;
							len++;
						} else {
							delta += (255 - rem) >> 1;
						}
						a->cur_bw = len >> 1;
					}
				} else if (!last) {
					if ((acc + len + 3) % 255 > 236) {
						len += 20;
						delta += 10;
						a->cur_bw = len >> 1;
					}
				} else if (rem < 18) {
					len -= rem;
					len = len + (len & 1) - 2;
					a->cur_bw = len >> 1;
				}
				acc += len;
			} else {
				s32 len = a->cur_bw;

				rem = (acc + len + 2) % 255;
				if (rem > 238) {
					if (!last) {
						len += 255 - rem;
						delta += 255 - rem;
						a->cur_bw = len;
					}
				} else if (!last) {
					if ((acc + len + 3) % 255 > 236) {
						len += 19;
						delta += 19;
						a->cur_bw = len;
					}
				} else if ((u16)rem < 18) {
					len -= rem + 1;
					a->cur_bw = len;
				}
				acc += len;
			}

			/* the next alloc pays for what this one grew */
			if (!last && delta)
				ctx[list[k + 1]].cur_bw -= delta;
		}

		for (k = 0; k < cnt; k++) {
			a = &ctx[list[k]];
			if (!(a->cfg.flags & 1))
				continue;
			if (!a->cur_bw) {
				if (dba_log_en) {
					dba_log_en = 0;
					npu_printf("**********max_req_bw == 0**********\n");
				}
				continue;
			}
			dba_config_bwmap_handler(ctx, list[k], 1, k, onu);
		}
	}

	dba_bwmap_bank_get(&b0, &b1);
	if (b0 != b1) {
		npu_printf("******switch bwmap failed last time*********\n");
		return 0;
	}
	if (b0)
		dba_bwmap_bank_set(0);
	else
		dba_bwmap_bank_set(1);

	if (dba_bwmap_nonempty != old) {
		npu_printf("FTTR DBA empty bwmap config switched,  switched to %d\n",
			   dba_bwmap_nonempty);
		if (dba_bwmap_nonempty)
			dba_bwmap_empty_set(0);
		else
			dba_bwmap_empty_set(1);
	}
	return 0;
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
	struct dba_budget bud = { 0 };

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

	if ((dba_frame_cnt & 1) && dba_do_timer_en) {
		dba_budget_calc(dba_ctx, &bud);
		dba_bw_request(dba_ctx, &bud);
		dba_bwmap_switch(dba_ctx);
	}
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
