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

PLL register at 0x1FA201FC, bits[7:6] select base frequency from
{800, 750, 720, 600} MHz, bits[2:0]+1 is the divider.

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
| `HAS_WIFI` | any WiFi chip | WiFi bridge, mailbox handlers, BA reorder |
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

- **TR-471** test infrastructure (~22 functions) for latency/loss
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
  Current .data is 212 bytes; the tables are not yet populated.

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
