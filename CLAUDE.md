# CLAUDE.md

Firmware for the RISC-V NPU in Airoha AN7552, AN7581 and AN7583 SoCs.
Start with [README.md](README.md) for the variant matrix and core map,
then the subsystem page in [docs/](docs/) for the code you touch.

## Build and check

```sh
make SOC=AN7583 WIFI=MT7993     # one variant, output in build/AN7583_MT7993/
make all-variants               # all 11; run before every commit
make SOC=... WIFI=... disasm    # build/<variant>/firmware.dis
```

- Every variant must build, with no new warnings.
- A change aimed at one variant must leave the other images unchanged.
  Save `md5sum build/*/npu_*.bin` before the change and compare after,
  both built with a fixed `GITREV=x`: the git hash is in every image.
- `.data` plus `.bss` must stay under `0x7800` bytes (see
  `firmware.map`). The boot log prints `OVER GLB_VAR_SRAM_SIZE`
  otherwise.
- SRAM blocks must fit the part: 256 KB on AN7552, 480 KB on AN7581,
  512 KB on AN7583.

## Variants

Code for one SoC or WiFi family sits behind the flags in `npu_config.h`
(`AN7552`, `AN758X`, `WIFI_EAGLE`, `WIFI_KITE`, `HAS_BME`,
`HAS_NPU_WIFI_TX`, ...). Put family-only code in the file named after
that family. Do not add runtime checks for what a build flag already
decides; runtime checks are for chip id and package differences within
one SoC.

## Rules the hardware imposes

- **Cross-core flags are `volatile`.** A loop that waits on a flag
  another core sets hangs forever if the compiler hoists the load.
- **Never re-acquire a held mutex.** Acquire is one try; acquiring a
  mutex the same hart holds stalls the NPU bus and freezes the hart.
  That includes an ISR on the same hart taking a mutex its interrupted
  code holds. Mask the interrupt around the critical section instead.
- **No printing from the trap path.** `npu_printf` holds mutex 15.
- **SRAM allocation starts after `tdma_init`.** It wipes SRAM and resets
  the allocator. A new address type needs a size entry in every
  variant's table in `npu_sram.c`.
- **PLIC registration keeps the first handler.** Registering again only
  enables the source on the calling hart.
- **Addresses.** Hardware and the host get physical addresses
  (`& 0x1FFFFFFF` for SRAM). Addresses from the host are read through
  `(addr & 0x3FFFFFFF) | 0x40000000`.
- **Rings the chip fills need their cpu index written back**, or the
  chip stalls once it catches up.

## Code conventions

- Shared globals live in `npu_globals.c`. It is the only source of
  `.data` and is compiled first, so its order is the layout of
  `npu_data.bin`. Pre-initialized tables use
  `__attribute__((section(".data")))`.
- Linux kernel C style: tabs, `u8`/`u16`/`u32`/`s32` from
  `npu_types.h`, `REG32()` for MMIO, register addresses in `npu_regs.h`.
- Log strings are part of the interface. Keep them verbatim, spelling
  included (`sucess`, `unknow`).
- Comments say what the code cannot: at most 3 lines, 15 words a line.
- Debug output goes behind a build option (`NPU_MAIL_TRACE`,
  `NPU_DATAPATH_DBG`), not into the default image.

## Documentation

`README.md` is the overview; `docs/*.md` hold the detail per subsystem.
Update the page for the subsystem you change in the same commit. Write
how the code works, as fact.

## Commits

- One logical change per commit.
- Title in Linux kernel style, module first: `wifi: eagle: fix ...`,
  `tdma: add ...`, `docs: ...`.
- Body: whether it is a feature, a fix or a cleanup, then what changed
  and why, in a few lines.
