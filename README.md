# Airoha AN75XX NPU Firmware

Bare-metal C firmware for the RISC-V packet-processing NPU embedded in
Airoha xPON-family SoCs (AN7552, AN7581, AN7583).

## Hardware

| SoC | Cores | WiFi Chips | Features |
|-----|------:|------------|----------|
| AN7552 | 2 | MT7916 MT7991 MT7993 | Base HWNAT offload |
| AN7581 | 8 | MT7916 MT7992 MT7996 | + tunnel offload, TR-471 (with WiFi) |
| AN7583 | 6 | MT7916 MT7992 MT7993 MT7996 NOWIFI | + tunnel offload, DBA/GPON (with WiFi) |

The NPU is a bare-metal RV32IMC cluster with no OS, no MMU. Each hart runs
a tight polling loop processing packets from the host kernel driver.

**Core assignment (AN7581 example):**
- Core 0: init + WiFi 2G RX/TX bridge loop
- Core 7: WiFi 5G pipeline + mailbox polling
- Cores 1–6: idle WFI (reserved for future offload)

### Memory Map

```
0x0C000000  PLIC (192 sources, 1-based HW IDs)
0x1EC00000  NPU thread manager / SCU / HW mutex / mailbox / MIB
0x1EC0D100  Host adaptor (DMA ring descriptors)
0x1FB50000  TDMA engine + PPE filter registers
0x1FBF0000  UART
0x3E800000  SRAM (AN7552=256KB, AN7581=480KB, AN7583=512KB, single-cycle)
0x84000000  DRAM (firmware code, loaded by kernel driver)
```

The NPU sees host physical addresses offset by +0x20000000 for SRAM and
ORed with 0x40000000 for DRAM DMA addresses.

### Clock

PLL register at 0x1FA201FC. bits[2:0]+1 is the divider; the frequency
selector differs per SoC:

| SoC | Selector | Table (MHz) |
|-----|----------|-------------|
| AN7552, AN7581 | bits[7:6] | 800, 750, 720, 600 |
| AN7583 | bits[8:7] | 666, 800, 720, 600 |

### Boot Reset Sequence

Hart 0 runs this once, gated by `npu_reset_pending` (set to 1 in
`.data`, cleared on the first pass). Secondary harts wait 10ms, then
poll `core_sync_flag` every 100ms until hart 0 sets it.

```
MIB21 (0x1EC0C194) saved            printf gate, lost across the reset
sim_mode_flag = MIB12 (0x1EC0C170)
RSTCTRL1 (0x1EC11834) = 0x10001788  assert reset on the NPU internal bus
delay 1ms
RSTCTRL1 = 0                        release
delay 1ms
THREAD_ENABLE (0x1EC00F00) = 4, 1
delay 1ms
MIB0 (0x1EC0C140) = 0xFFFFFFFF
boot UART init (0x1EC10000)
MIB21 restored
"core freq at %d MHz"
```

RSTCTRL1 is at SCU+0x834. SCU+0x000 is not the reset control; writing
it leaves the hart stuck on its next MMIO access, with no UART output
and every host mailbox call timing out.

### Hardware Mutex

Block at 0x1EC03000. A descriptor is two words — the mutex id and a
priority flag.

```
off  = (id * 4) & mask      mask 0x3C on AN7552/AN7583 (16 mutexes)
                            mask 0x7C on AN7581 (32 mutexes)
0x000 + hart*0x400 + off    status: bit16 held, bits[15:8] owner hart
0x080 + off                 priority acquire, write (hart<<8)|0x10040
0x180 + off                 acquire, write (hart<<8)|0x40
0x200 + hart*0x400 + off    release, write hart<<8
```

The hart field is 2 bits, so harts 4-7 alias onto 0-3. Acquire is a
single write plus one status read — the hardware arbitrates and callers
never spin. Mutex 15 is the printf lock.

Acquiring a mutex that is already held stalls the NPU bus: the acquire
write never completes, the store buffer fills, and the hart freezes a
few instructions later. Nothing may re-enter a printf while it holds the
printf mutex, which is why the trap handler reports faults instead of
dispatching them through the PLIC ISR table.

### WiFi Mail

Command handlers reach the host over mailbox function 0. `interfaceID`
in the message header means a band on the kite path and a **ring index**
on the eagle path:

| ring | eagle meaning |
|-----:|---------------|
| 0, 1 | RRO rx rings |
| 5, 6 | MSDU page rings |
| 8, 9 | indirect command ring |
| 10, 11 | tx done rings |
| 15 | all bases set; publish every ring's cpu index |

Host addresses at or above `0xC0000000` are outside the window the NPU
can reach. Both paths use the same dispatch tables, one entry per funcId,
with a different handler set behind each.

### Mailbox

`0x1EC0C000`, matching the host driver's `CR_MBOX_*` / `CR_MBQ<n>_CTRL*`.

```
+0x000               interrupt status (write 1 to clear)
+0x004 + n*4         interrupt mask n; mask n+1 = 1<<n routes mailbox n+1 to core n
+0x030 + q*0x10      CTRL0 buffer physical address     (q = core id)
+0x034 + q*0x10      CTRL1 length          (16-bit)
+0x038 + q*0x10      CTRL2 doorbell counter, incremented by the sender
+0x03C + q*0x10      CTRL3 arg + status    (16-bit: bit0 wait, bit1 done,
                                            bits[4:2] return, bits[14:11] func id)
+0x140 + n*4         MIB n
```

Queue 8 is the NPU-to-host notify channel. The notify mutex id is 14 on
AN7552 and AN7583, 30 on AN7581.

### PLIC

192 sources, hardware ids are source+1.

```
0x0C000004 + src*4   priority (16 for every source, 17 for source 95)
0x0C002000 + w*4     enable
0x0C003000 + w*4     mask / pending
0x0C200000           threshold
0x0C200004           claim and complete
```

### TDMA

The TDMA WiFi path exists only where `HAS_BME` does - AN7552 and AN7583
with a WiFi chip. AN7581 has no `tdma_tx_init`, `tdma_rx_init` or
`tdma_bme_init` on any of its three WiFi variants, and AN7583_NOWIFI
has no TDMA ring registers at all.

Within that path the per-variant differences are:

| what | AN7552 / AN7583 |
|------|-----------------|
| TX rings | 2, base 0x1FB50800, stride 0x10 |
| RX rings | 2, base 0x1FB50900, 1024 descriptors of 32 bytes |
| descriptor pattern | `(d & 0x3FFFC000) \| 0xC0000800` |
| flow control | runtime chip id check, AN7552 takes 0x1FB501BC |

`TDMA_WIFI_BUF_CFG` at 0x1FB50FE8 is the one field that splits by WiFi
chip rather than SoC:

| WiFi | write |
|------|-------|
| eagle (MT7991/7992/7993) | `(x & 0xFFB300FF) \| 0x190100` |
| kite (MT7916/7996) | `(x & 0xFFF300FF) \| 0x590100` |

`tdma_set_tx_ring_to_int` writes `0x01010101`/`1` for ring 0 and
`0x02020202`/`0x11` for ring 1, then reads both registers back to log
them. The read-back differs from what was written - the hardware does
not keep bit 0 of 0x1FB50A28 - so a log showing `=0` and `=10` is
correct.

### Timers

`NPU_TIMER0_BASE` is `0x1EC10100` and `NPU_TIMER1_BASE` `0x1EC10200`; the
CPU timer block is at `0x1EC10900`. AN7581 has 4 timers, AN7552 has 8 and
AN7583 has 16 across the two banks. Five `.data` tables index them: PLIC
source, interrupt-clear bit, enable bit, counter register and reload
register. The reload value is `50000 * period` on AN7583,
`1000 * period * clk` on AN7552 and `1000 * period * clk / 100` on AN7581.

The ISR acks by rewriting the control word as
`(ctrl & 0x1E0001EF) | (1 << clear_bit)`.

## Build

Requires `riscv64-unknown-elf-gcc` (tested with GCC 14.2.0 on Debian).

```sh
make SOC=AN7581 WIFI=MT7916       # one variant
make all-variants                  # all available variants
make SOC=AN7583 WIFI=MT7996 disasm # disassemble
```

Default: `SOC=AN7583 WIFI=MT7996`.

Outputs per variant in `build/<SOC>_<WIFI>/`:
- `npu_rv32.bin`: code + rodata, loaded to DRAM at 0x84000000
- `npu_data.bin`: initialized data, loaded to SRAM at 0x3E900000
- `firmware.map`: linker map
- `firmware.elf`: full ELF (for `nm`, `objdump`, etc.)

Compiler flags: `-Os -ffunction-sections -fdata-sections`
Linker flags: `--gc-sections --relax` + libgcc (for 64-bit division on RV32)

## Source Layout

| File | Lines | Purpose |
|------|------:|---------|
| `npu_main.c` | ~1700 | Globals, core infra (PLIC, timer, mutex, SRAM, mailbox), boot, DBA |
| `npu_wifi.c` | ~4500 | WiFi subsystem: BA, reorder, classifier, bridge, mailbox handlers |
| `npu_tunnel.c` | ~1300 | Tunnel offload, L4S ECN, SRv6, fragmentation |
| `npu_printf.c` | ~340 | vsprintf, UART output, debug console |
| `npu_internal.h` | ~430 | Cross-file prototypes and extern declarations |
| `npu_config.h` | 59 | `#ifdef` variant selection |
| `npu_regs.h` | 222 | MMIO register definitions |
| `npu_types.h` | 52 | `u8`/`u16`/`u32`/`u64`/`s32` typedefs, struct types |
| `crt0.S` | 137 | Reset vector, BSS clear, stack setup, per-hart dispatch |
| `link.ld` | 71 | Linker script (DRAM + SRAM regions) |
| `Makefile` | 75 | Build system with all 11 variants |

All compile-time variants are handled with `#ifdef` across the source
files. Convention: `AN75XX` = all three SoCs, `AN758X` = AN7581 + AN7583.
All `.data` globals stay in `npu_main.c` to preserve `npu_data.bin` layout.

### Conditional Feature Flags

| Flag | Condition | Enables |
|------|-----------|---------|
| `WIFI_KITE` | MT7916 / MT7996 | TDMA RX path, WiFi chip name table, function dispatch tables |
| `WIFI_EAGLE` | MT7991 / MT7992 / MT7993 | RRO, MSDU page ring, indirect command ring |
| `HAS_WIFI` | any WiFi chip | WiFi bridge, mailbox handlers and dispatch tables, BA reorder |
| `HAS_TUNNEL` | AN758X | Tunnel offload (IPv4/IPv6/SRv6), L4S ECN |
| `HAS_DBA` | AN7583 + WiFi | GPON DBA (dynamic bandwidth allocation) |
| `HAS_TR471` | AN7581 + WiFi | TR-471 latency/loss measurement |

## Binary Separation: npu_rv32.bin vs npu_data.bin

The firmware produces two binaries because they target different memory:

**npu_rv32.bin** (code + rodata → DRAM): Read-mostly, fetched through
instruction cache. Sits in host DDR, large but higher latency from the
NPU's perspective.

**npu_data.bin** (initialized globals → SRAM): Mutable working data the
NPU reads/writes every packet. SRAM is NPU-local with single-cycle
access, function pointer tables, lookup arrays, configuration state all
live here for performance.

The kernel driver loads each to its respective address. BSS follows
.data in SRAM and is zeroed by `crt0.S`. Stacks are placed in DRAM
after .text (16KB per hart).

## Kernel Driver Interface

The NPU firmware communicates with the Linux kernel driver
(`airoha_npu.ko`) via:

- **Mailbox** at 0x1EC0C000: 6 function slots (WiFi, tunnel, notify,
  DBA, TR-471, HWNAT). Each message carries an interfaceID, funcType
  (SET_WAIT/SET_NO_WAIT/GET_WAIT/GET_NO_WAIT), funcId, and data payload.
- **MIB registers** at 0x1EC0C140: Parameter passing (PCIe addresses,
  SRAM buffer pointers, init handshake).
- **Host adaptor rings** at 0x1EC0D100: DMA descriptor rings for packet
  transfer between host and NPU.

## Implementation Status

361 functions defined in source. After `--gc-sections`, AN7581_MT7916
(the largest variant) retains 125 functions / 33KB code which are the reachable
set from core entry points.

### What's Implemented

- Full boot sequence (crt0 → per-hart dispatch → core0/core7 init)
- WiFi bridge TX/RX loop (kite and eagle paths)
- WiFi mailbox handlers (SET_WAIT: 31 commands, GET_WAIT: 10 commands)
- Packet classifier and multi-descriptor handler
- BA (block-ack) reorder engine
- SRAM free-buffer ring (5600 entries of u16 buffer indices)
- Tunnel offload (store header, SRv6, fragmentation MTU, reset)
- PPE filter/enable configuration
- L4S ECN marking
- DBA subsystem (GPON bandwidth allocation)
- HW mutex (SCU spinlock) acquire/release
- npu_printf with %d/%u/%x/%s/%llu/%lld/%llx and width/pad support
- CPU clock readout, delay_us/delay_ms, timer tick counters
- Timer extensions: watchdog, multi-bank (AN7581), CPU timer init
- UART debug console

### What's Missing

- **Datapath workers** runs a polling loop on cores 1, 3 and
  4; currently no frame moves through the NPU even
  though every ring is allocated and programmed:

  | core | blob | what it does |
  |-----:|------|--------------|
  | 1 | `sub_84012380` | `kite_handle_rxdmad_c_ring`: waits on the ready flags, then loops on `sub_84011686` |
  | 3 | `sub_84000AA6` | noreturn worker over the per-band dispatch it builds at `0x3E900CC0` |
  | 4 | `sub_84013B8C` | polls the rx ring at the descriptor base with the cpu index, `sub_84011150` per descriptor |

  `wifi_bridge_loop` and `wifi_pipeline_worker` poll `wifi_tx_pending`
  and `wifi_rx_pending`, which nothing ever raises - the kite design
  expects an ISR to set them, and the eagle workers poll the rings
  directly instead.
- **Eagle rx descriptor ring fill** `npu_mbox_init_rxd_wrapper`
  (set_wait funcId 1) validates the ring index but does not populate
  descriptors. The per-ring initialisers behind it (RRO, MSDU page,
  indirect command, tx done) are not reconstructed.
- **Eagle PCIe window publish** set_port_type records the type but
  does not rewrite the per-port windows at `0x1FA90038` / `0x1FC28030`.
- **TR-471** test infrastructure (~22 functions) is latency/loss
  measurement per ITU-T Y.1540
- **Thread manager** (~10 functions) advanced multi-hart scheduling
- **TDMA TX-done / RX fast-path** (~9 functions) TX completion
  callbacks, optimized RX
- **PPE filter tables** (~5 functions) full filter programming
- **Timer extensions** (~5 functions) watchdog, periodic callbacks
- **Trap vector** full 32-GPR context save/restore (current crt0 has
  a minimal trap handler)
- **Pre-computed .data tables** BA session SRAM pointer table (~2KB),
  tunnel template headers, TR-471 config structs, MIB address arrays.
  Current .data is 116 bytes (AN7583_MT7993); the tables are not yet
  populated.
- **Boot UART RX console** `uart_debug_cmd` parses `rd`/`wt`, but no
  ISR is registered on PLIC source 22, which `npu_init` enables.

## Verification

Build all variants and check the outputs:

```sh
# build and check sizes
make all-variants
for v in build/*/; do
  echo "$(basename $v): code=$(wc -c < $v/npu_rv32.bin) data=$(wc -c < $v/npu_data.bin)"
done

# disassemble for offline analysis
make SOC=AN7581 WIFI=MT7916 disasm
```

`firmware.map` lists every symbol address for each variant.
