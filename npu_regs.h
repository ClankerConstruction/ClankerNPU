#ifndef NPU_REGS_H
#define NPU_REGS_H

#include "npu_types.h"

#define REG32(addr) (*(volatile u32 *)(addr))

/* SoC identification */
#define CHIP_ID_REG             0x1FB00064
#define CHIP_VARIANT_REG        0x1FB00284

/* NPU cluster (boot control) */
#ifdef AN7552
#define NPU_CLUSTER_BASE        0x1EC08000
#else
#define NPU_CLUSTER_BASE        0x1EC06000
#endif
#define CR_CORE_BOOT_TRIGGER    (NPU_CLUSTER_BASE + 0x000)
#define CR_CORE_BOOT_CONFIG     (NPU_CLUSTER_BASE + 0x004)
#define CR_CORE_BOOT_BASE(n)    (NPU_CLUSTER_BASE + 0x020 + ((n) << 2))

/* NPU MIB registers (host ↔ firmware parameter passing) */
#define NPU_MIB_BASE            0x1EC0C000
#define NPU_MIB(n)              (NPU_MIB_BASE + 0x140 + ((n) << 2))
#define NPU_MIB0                NPU_MIB(0)
#define NPU_MIB8                NPU_MIB(8)
#define NPU_MIB9                NPU_MIB(9)
#define NPU_MIB10               NPU_MIB(10)
#define NPU_MIB11               NPU_MIB(11)
#define NPU_MIB12               NPU_MIB(12)
#define NPU_MIB13               NPU_MIB(13)
#define NPU_MIB21               NPU_MIB(21)
#define NPU_MIB31               NPU_MIB(31)

/* NPU debug CSR */
#define NPU_DEBUG_BASE          0x1EC05000
#define NPU_CSR_PC(core)        (NPU_DEBUG_BASE + ((core) * 0x10) + 0x00)
#define NPU_CSR_SP(core)        (NPU_DEBUG_BASE + ((core) * 0x10) + 0x04)
#define NPU_CSR_RA(core)        (NPU_DEBUG_BASE + ((core) * 0x10) + 0x08)

/* Hardware mutex. off = (mutex id * 4) & HW_MUTEX_OFF_MASK; hart field is 2 bits. */
#define HW_MUTEX_BASE           0x1EC03000
#define HW_MUTEX_HART(hart)     (((hart) << 10) & 0xC00)
#define HW_MUTEX_STATUS(hart, off)  (HW_MUTEX_BASE + HW_MUTEX_HART(hart) + (off))
#define HW_MUTEX_PRI_ACQ(off)   (HW_MUTEX_BASE + 0x080 + (off))
#define HW_MUTEX_TRY_ACQ(off)   (HW_MUTEX_BASE + 0x100 + (off))
#define HW_MUTEX_ACQ(off)       (HW_MUTEX_BASE + 0x180 + (off))
#define HW_MUTEX_REL(hart, off) (HW_MUTEX_BASE + 0x200 + HW_MUTEX_HART(hart) + (off))
#define HW_MUTEX_NOTIFY(hart, off) (HW_MUTEX_BASE + 0x380 + HW_MUTEX_HART(hart) + (off))
#define HW_MUTEX_HELD           0x10000
#ifdef AN7581
#define HW_MUTEX_OFF_MASK       0x7C		/* 32 mutexes */
#else
#define HW_MUTEX_OFF_MASK       0x3C		/* 16 mutexes */
#endif

/* Mailbox. One 16-byte control block per queue at +0x030, matching the
 * host driver's CR_MBQ<n>_CTRL0..3. Queue index is the core id; queue 8 is
 * the NPU-to-host notify channel. */
#define NPU_MBOX_BASE           0x1EC0C000
#define MBOX_INT_STS            (NPU_MBOX_BASE + 0x000)
#define MBOX_INT_MASK(n)        (NPU_MBOX_BASE + 0x004 + ((n) * 4))
#define MBOX_INT_MASK0          MBOX_INT_MASK(0)
#define MBQ_CTRL0(q)            (NPU_MBOX_BASE + 0x030 + ((q) * 0x10))	/* buffer phys addr */
#define MBQ_CTRL1(q)            (NPU_MBOX_BASE + 0x034 + ((q) * 0x10))	/* length */
#define MBQ_CTRL2(q)            (NPU_MBOX_BASE + 0x038 + ((q) * 0x10))	/* doorbell counter */
#define MBQ_CTRL3(q)            (NPU_MBOX_BASE + 0x03C + ((q) * 0x10))	/* arg + status */
#define MBQ_NOTIFY              8

/* Host adaptor */
/* PCIe inbound windows: base and end of the slice of NPU SRAM the WiFi
 * chip behind each port may reach */
#define PCIE0_WIN_BASE          0x1FA90038
#define PCIE0_WIN_END           0x1FA9003C
#define PCIE1_WIN_BASE          0x1FC28030
#define PCIE1_WIN_END           0x1FC28034

#define NPU_HOSTADPT_BASE       0x1EC0D100
#define HOSTADPT_TX_BASE_PTR(r) (NPU_HOSTADPT_BASE + 0x000 + ((r) * 0x10))
#define HOSTADPT_TX_MAX_CNT(r)  (NPU_HOSTADPT_BASE + 0x004 + ((r) * 0x10))
#define HOSTADPT_TX_CPU_IDX(r)  (NPU_HOSTADPT_BASE + 0x008 + ((r) * 0x10))
#define HOSTADPT_TX_DMA_IDX(r)  (NPU_HOSTADPT_BASE + 0x00C + ((r) * 0x10))
#define HOSTADPT_RX_BASE_PTR(r) (NPU_HOSTADPT_BASE + 0x080 + ((r) * 0x10))
#define HOSTADPT_RX_MAX_CNT(r)  (NPU_HOSTADPT_BASE + 0x084 + ((r) * 0x10))
#define HOSTADPT_RX_DMA_IDX(r)  (NPU_HOSTADPT_BASE + 0x088 + ((r) * 0x10))
#define HOSTADPT_RX_CPU_IDX(r)  (NPU_HOSTADPT_BASE + 0x08C + ((r) * 0x10))
/* host -> NPU rings, only present where the host offloads WiFi tx */
#define HOSTADPT_IN_BASE_PTR(r) (NPU_HOSTADPT_BASE - 0x060 + ((r) * 0x10))
#define HOSTADPT_IN_MAX_CNT(r)  (NPU_HOSTADPT_BASE - 0x05C + ((r) * 0x10))
#define HOSTADPT_IN_CPU_IDX(r)  (NPU_HOSTADPT_BASE - 0x054 + ((r) * 0x10))

/* PLIC (Platform-Level Interrupt Controller)
 * Sources are 0-based internally; hardware uses 1-based.
 * Priority[src]: 0x0C000004 + src*4  (source src+1 in PLIC terms)
 * Enable word:   0x0C002000 + word*4
 * Mask word:     0x0C004000 + word*4
 * Threshold:     0x0C200000
 * Claim/Complete: 0x0C200004 (returns/accepts 1-based source ID)
 */
#define PLIC_BASE               0x0C000000
#define PLIC_PRIORITY(src)      (0x0C000004 + ((src) * 4))
#define PLIC_ENABLE_REG(word)   (0x0C002000 + ((word) * 4))
#define PLIC_MASK_REG(word)     (0x0C003000 + ((word) * 4))
#define PLIC_THRESHOLD_REG      0x0C200000
#define PLIC_CLAIM_REG          0x0C200004
#define PLIC_MAX_SOURCE         192

/* mcause: interrupt bit set, code 11 */
#define MCAUSE_MACHINE_EXT_IRQ  0x8000000BU

/* NPU bridge (DMA channels) */
#define NPU_BRIDGE_BASE         0x1EC0C000
#define BRIDGE_CH_BASE(ch)      (NPU_BRIDGE_BASE + ((ch) * 0x40))

/* PLIC interrupt sources for BME/BMGR (per-SoC) */
#if defined(AN7552)
#define INTR_BME_DONE           24
#define INTR_BMGR               25
#else
#define INTR_BME_DONE           32
#define INTR_BMGR               33
#endif

/* BME (Buffer Move Engine) - AN7552 and AN7583 (with WiFi) */
#if defined(AN7552)
#define BME_BASE                0x1EC08000
#else /* AN7583 */
#define BME_BASE                0x1EC0B800
#endif
#define BME_CSR_BASE_ADDR       (BME_BASE + 0x014)
#define BME_CSR_MAX_INDEX       (BME_BASE + 0x018)
#define BME_CSR_SW_INDEX        (BME_BASE + 0x01C)
#define BME_CSR_HW_INDEX        (BME_BASE + 0x020)
#define BME_CSR_CTRL            (BME_BASE + 0x024)

/* BMGR (Buffer Manager) - AN7552 only */
#define BMGR_BASE               0x1EC08800
#define BMGR_BUF_ID_BASE        (BMGR_BASE + 0x000)
#define BMGR_INIT               (BMGR_BASE + 0x004)

/* TDMA engine */
#define TDMA_BASE               0x1FB50000
#define TDMA_TX_BASE_PTR(ring)  (TDMA_BASE + 0x800 + ((ring) * 0x10))
#define TDMA_RX_BASE_PTR(ring)  (TDMA_BASE + 0x900 + ((ring) * 0x10))

/* UART - boot UART and runtime UART */
#define BOOT_UART_BASE          0x1EC10000
#define BOOT_UART_TX            (BOOT_UART_BASE + 0x000)
#define BOOT_UART_DLL           (BOOT_UART_BASE + 0x000)
#define BOOT_UART_IER           (BOOT_UART_BASE + 0x004)
#define BOOT_UART_DLM           (BOOT_UART_BASE + 0x004)
#define BOOT_UART_FCR           (BOOT_UART_BASE + 0x008)
#define BOOT_UART_LCR           (BOOT_UART_BASE + 0x00C)
#define BOOT_UART_MCR           (BOOT_UART_BASE + 0x010)
#define BOOT_UART_STATUS        (BOOT_UART_BASE + 0x014)
#define BOOT_UART_SCR           (BOOT_UART_BASE + 0x024)
#define BOOT_UART_FRACDIV       (BOOT_UART_BASE + 0x02C)
#define NPU_UART_BASE           0x1FBF0000
#define UART_TX_DATA            (NPU_UART_BASE + 0x000)
#define UART_TX_STATUS          (NPU_UART_BASE + 0x014)
#define UART_TX_READY_BIT       (1 << 5)

/* Timer */
#define NPU_TIMER0_BASE         0x1EC10100
#define NPU_TIMER1_BASE         0x1EC10200
#define NPU_CPU_TIMER_BASE      0x1EC10900
#define NPU_TIMER_CTRL          0x1EC10100
#define NPU_TIMER_CTRL2         0x1EC10108
#define NPU_TIMER_WDT_RELOAD    (NPU_TIMER0_BASE + 0x34)
#define NPU_TIMER_BANK_CTRL(b)  (NPU_TIMER0_BASE + ((b) << 8))
#define NPU_TIMER_BANK_PRESCALE(b) (NPU_TIMER0_BASE + 0x2C + ((b) << 8))
#define NPU_TIMER_BANK_RELOAD(b) (NPU_TIMER0_BASE + 0x34 + ((b) << 8))
#define CPU_TIMER_RELOAD(idx)   (0x1EC10904 + ((idx) << 3))
#define CPU_TIMER_COUNTER(idx)  (0x1EC10908 + ((idx) << 3))

/* PCIe bases per SoC */
#if defined(AN7583)
#define PCIE0_MAC_BASE          0x1FC20000
#define PCIE1_MAC_BASE          0x1FA92000
#elif defined(AN7552)
#define PCIE0_MAC_BASE          0x1FA91000
#define PCIE1_MAC_BASE          0x1FA92000
#else /* AN7581 */
#define PCIE0_MAC_BASE          0x1FC00000
#define PCIE1_MAC_BASE          0x1FC20000
#define PCIE2_MAC_BASE          0x1FC40000
#endif

/* PPE filter registers */
#define PPE0_FILTER_BASE        (TDMA_BASE + 0xE00)
#define PPE1_FILTER_BASE        (TDMA_BASE + 0x1E00)

/* SCU */
#define NPU_SCU_BASE            0x1EC11000
#define NPU_SCU_WDOG_RST_CFG    (NPU_SCU_BASE + 0x038)
#define NPU_SCU_PMC             (NPU_SCU_BASE + 0x080)
#define NPU_SCU_RSTCTRL1        (NPU_SCU_BASE + 0x834)
/* every block behind the NPU internal bus, held for the 1ms reset pulse */
#define NPU_SCU_RST_ALL         0x10001788

/* Thread manager */
#define NPU_THREAD_BASE         0x1EC00000
#define NPU_THREAD_ENABLE       (NPU_THREAD_BASE + 0xF00)

/* Debug counters */
#define NPU_DBG_CNT_INTR_CORE   0x1EC0FF0C
#define NPU_DBG_CNT_BASE        0x1EC0FF10

/* USB power */
#define USB_CTRL_BASE           0x1FA20700

/* FTTR (GPON DBA) - AN7583 only */
#define NPU_FTTR_BASE           0x1FBE4000

/* DMA copy engine (bridge DMA for host ring packets) */
#define DMA_COPY_BASE           0x1FB30000
#define DMA_COPY_SRC(ch)        (DMA_COPY_BASE + (ch) * 16)
#define DMA_COPY_DST(ch)        (DMA_COPY_BASE + (ch) * 16 + 4)
#define DMA_COPY_CTRL(ch)       (DMA_COPY_BASE + (ch) * 16 + 8)
#define DMA_COPY_STATUS         (DMA_COPY_BASE + 0x204)

/* NPU bridge debug */
#define NPU_BRIDGE_DBG_BASE     0x1EC12290

/* NPU SRAM (seen from NPU side) */
#define NPU_SRAM_BASE           0x3E900000
#define NPU_SRAM_SIZE_64K       0x10000

/* DRAM image base */
#define DRAM_IMAGE_BASE         0x84000000

/* SRAM alias: host physical + 0x20000000 */
#define HOST_TO_NPU_SRAM(x)    ((x) + 0x20000000)
#define NPU_TO_HOST_SRAM(x)    ((x) - 0x20000000)

/* DMA address mask (host physical → NPU-visible) */
#define NPU_ADDR_MASK           0x40000000

/* Stack size per hart */
#define STACK_SIZE              0x4000

/* Core counts */
#if defined(AN7552)
#define MAX_CORE_NUM            2
#elif defined(AN7583)
#define MAX_CORE_NUM            6
#else /* AN7581 */
#define MAX_CORE_NUM            8
#endif

/* Mailbox function IDs */
#define MFUNC_WIFI              0
#define MFUNC_TUNNEL            1
#define MFUNC_NOTIFY            2
#define MFUNC_DBA               3
#define MFUNC_TR471             4
#define MFUNC_HWNAT             5
#define MAX_MBOX_FUNC           8

/* WiFi interfaces */
#define INTERFACE_2G            0
#define INTERFACE_5G            1

/* WiFi driver models */
#define DRIVER_MODEL_MT7915A    0
#define DRIVER_MODEL_MT7915D    1
#define DRIVER_MODEL_MT7916D    2

/* Init complete sentinel */
#define INIT_COMPLETE           0xCCCCCCCC
#define ALL_FF                  0xFFFFFFFF

#endif /* NPU_REGS_H */
