#ifndef NPU_TYPES_H
#define NPU_TYPES_H

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;
typedef unsigned long long u64;
typedef signed char s8;
typedef signed short s16;
typedef signed int s32;
typedef signed long long s64;

#define NULL ((void *)0)

typedef void (*isr_fn_t)(int src);
typedef int (*mbox_handler_t)(u32 base_ptr, u32 max_cnt);

/* RISC-V CSR access */
#define csr_read(reg) ({ \
	unsigned long __v; \
	__asm__ volatile("csrr %0, " #reg : "=r"(__v)); \
	__v; })

#define csr_write(reg, val) ({ \
	__asm__ volatile("csrw " #reg ", %0" :: "r"((unsigned long)(val))); })

#define csr_set(reg, val) ({ \
	unsigned long __v; \
	__asm__ volatile("csrrs %0, " #reg ", %1" : "=r"(__v) : "r"((unsigned long)(val))); \
	__v; })

#define csr_clear(reg, val) ({ \
	unsigned long __v; \
	__asm__ volatile("csrrc %0, " #reg ", %1" : "=r"(__v) : "r"((unsigned long)(val))); \
	__v; })

#define csr_clear_imm(reg, imm) ({ \
	unsigned long __v; \
	__asm__ volatile("csrrci %0, " #reg ", " #imm : "=r"(__v)); \
	__v; })

#define csr_set_imm(reg, imm) ({ \
	unsigned long __v; \
	__asm__ volatile("csrrsi %0, " #reg ", " #imm : "=r"(__v)); \
	__v; })

static inline void irq_disable(void) { csr_clear_imm(mstatus, 8); }
static inline void irq_enable(void)  { csr_set_imm(mstatus, 8); }

static inline u32 get_hartid(void) { return (u32)csr_read(mhartid); }

#endif /* NPU_TYPES_H */
