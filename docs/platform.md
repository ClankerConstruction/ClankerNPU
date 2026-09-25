# Platform Services

Hardware mutex, interrupts, timers and console output. Every other
subsystem builds on these.

## Hardware mutex

The mutex block at `0x1EC03000` arbitrates between harts. A mutex is a
two-word descriptor: the mutex id and a priority flag.

```
off  = (id * 4) & mask       mask 0x3C on AN7552/AN7583 (16 mutexes)
                             mask 0x7C on AN7581 (32 mutexes)
0x000 + hart*0x400 + off     status: bit16 held, bits[15:8] owner hart
0x080 + off                  priority acquire: write (hart<<8)|0x10040
0x180 + off                  acquire: write (hart<<8)|0x40
0x200 + hart*0x400 + off     release: write hart<<8
```

- Acquire is one write and one status read. It returns 0 when this hart
  owns the mutex, -1 otherwise. Callers do not spin.
- The hart field is 2 bits wide, so harts 4..7 alias onto 0..3.
- Acquiring a mutex the hart already holds stalls the NPU bus: the write
  never completes and the hart freezes a few instructions later. Code that
  holds a mutex must not call anything that takes the same one, including
  from an ISR on the same hart.

| id | owner |
|---:|---|
| 3, 4 | tx token alloc, free |
| 5, 6 | kite BA tables, 5 GHz / 2.4 GHz |
| 8, 9 | kite reorder node alloc, free |
| 10 | eagle packet queue, kite 2.4 GHz node queue |
| 11 | kite 5 GHz node queue |
| 12, 13 | rx buffer id alloc, free |
| 14 | host notify (30 on AN7581) |
| 15 | printf |
| 18 | SRAM allocator |

## PLIC

192 sources. The hardware numbers them from 1; the firmware numbers
them from 0 and adds 1 when it touches a register.

```
0x0C000004 + src*4   priority: 16 for every source, 17 for source 95
0x0C002000 + w*4     enable
0x0C003000 + w*4     mask
0x0C200000           threshold
0x0C200004           claim and complete
```

Each hart has its own enable, mask and claim state. A source interrupts
only the harts that enabled it. The handler table `plic_isr_table` is
shared.

`plic_register_isr(src, fn)`:

- stores `fn` if the source has only the default handler,
- otherwise keeps the existing handler and prints
  `Error: src:N already registered ISR, just exit`,
- enables the source on the calling hart in both cases.

Registering a source that already has a handler is how a second hart
enables it for itself.

| source | handler | hart |
|---:|---|---|
| 8 + n | mailbox of core n | core 0 registers all; core 5 enables 13 again for DBA |
| 15 | mailbox of core 7 | tunnel offload hart |
| 18 | timer tick | core 2; core 0 on AN7552 |
| 22 | boot UART rx console | core 0 |
| 24 / 32 | BME done (AN7552 / others), kite | core 0 |
| 25 / 33 | BMGR status (AN7552 / others) | core 0 |
| 59 | debug counter | core 0, not on AN7552 |
| 95 | PPE WiFi buffer return, eagle | core 0 |

## Timers

`NPU_TIMER0_BASE` is `0x1EC10100` and `NPU_TIMER1_BASE` is `0x1EC10200`.
AN7581 has 4 timers, AN7552 8 and AN7583 16 over the two banks. Five
tables in `npu_globals.c` index them: PLIC source, clear bit, enable bit,
counter register and reload register.

| SoC | timer clock | reload |
|---|---|---|
| AN7583 | 50 MHz | `50000 * period` |
| AN7552 | CPU clock / 4 | `1000 * period * clk` |
| AN7581 | CPU clock / 4 | `1000 * period * clk / 100` |

`npu_init` starts timer 0 with period 10. Core 2 takes its interrupt
(source 18). The ISR acks by rewriting the control word as
`(ctrl & 0x1E0001EF) | (1 << clear_bit)` and advances:

- `timer_raw_tick` on every interrupt,
- `timer_slow_tick` every 100 raw ticks,
- a seconds and microseconds time of day.

AN7552 has no core 2: `timer_init` registers the ISR on core 0. The
tick must be acked, or the level interrupt re-enters forever and
starves core 0. The AN7552 ISR acks from the control word saved at
the first tick and only counts `timer_int_count`, the AN7552 kite
ageing clock (`KITE_TICK`).

`delay_us` (800 cycles per microsecond) and `delay_ms` busy-wait on
`mcycle`. `delay_ms` scales by the CPU clock and rejects a delay whose
cycle count would overflow 32 bits. `delay_1ms` is a plain loop.

## Console output

Two output paths share mutex 15.

| function | UART | prefix | line end |
|---|---|---|---|
| `npu_printf` | SoC UART `0x1FBF0000` | `[Cn]`, n = hart | LF then CR |
| `boot_printf` | boot UART `0x1EC10000` | `[Cn]` always | CR then LF |

`npu_printf` prints nothing while `MIB21` is nonzero, which lets the host
silence the firmware. It runs with interrupts off and gives up on a
character after 200000 busy polls, so a stuck UART cannot hold the mutex
forever.

The formatter supports `%d %u %x %X %s %c %p %%`, the `l` and `ll`
modifiers, width, `0` padding and `-` alignment.

### Boot UART console

Source 22 is the boot UART receiver. The ISR echoes each byte and feeds
a line parser that accepts two fixed-column commands, ended by CR:

```
rd AAAAAAAA              read the 32-bit register at 0xAAAAAAAA
wt AAAAAAAA VVVVVVVV     write it, then read it back
```

Replies go out through `boot_printf` as `rd(0xA)==0xV` or
`wt(0xA)==0xV`. A malformed line prints the expected syntax.
