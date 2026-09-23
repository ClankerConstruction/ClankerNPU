# Field Debugging

Every image carries a 4 KB debug block at a fixed SRAM address. The
host reads and writes it with the stock `sys` shell command, so a
deployed unit needs nothing extra:

| command | effect |
|---|---|
| `sys memrl ADDR` | read one 32-bit word |
| `sys memwl ADDR VALUE` | write one 32-bit word |
| `sys memory ADDR LEN` | hexdump `LEN` bytes, 16 per line |

All arguments are hex, with or without `0x`. The output goes to the
kernel log, so read it with `dmesg` (run `dmesg -n 1` first to keep the
console quiet). `sys` accepts physical addresses from `0x1C000000` to
`0x3FFFFFFF`, which covers the NPU SRAM and every NPU and frame engine
register.

## Addresses

The NPU sees its SRAM at `0x3E8xxxxx`/`0x3E9xxxxx`; the host sees the
same bytes at `0x1E8xxxxx`/`0x1E9xxxxx`. Clear the top three bits of
any address the NPU console prints before handing it to `sys`.

The debug block is at NPU `0x3E906800`, host **`0x1E906800`**, up to
`0x1E9077FF`. It sits above the firmware globals, which the linker
keeps below it.

`sys memory` prints every word most significant byte first. Tags and
text in the block are stored so that they read in order in that dump:

```
# sys memory 1e906800 40
1e906800  4e 44 42 47.00 00 00 01.00 00 10 00.00 00 00 06  |NDBG............|
1e906810  54 4c 42 37.2e 38 2e 30.2e 30 5f 76.30 30 33 2e  |TLB7.8.0.0_v003.|
1e906820  4d 54 37 39.39 33 2e 32.34 31 36 35.32 38 00 00  |MT7993.2416528..|
```

`NDBG` (`0x4E444247`) at offset 0 means the firmware is running and
the block is valid. The version string is the one the boot log prints
as `NPU Version:`.

## Layout

| offset | size | field |
|---:|---:|---|
| `0x000` | 4 | magic `0x4E444247` (`NDBG`) |
| `0x004` | 4 | layout version, 1 |
| `0x008` | 4 | block size, `0x1000` |
| `0x00C` | 4 | number of harts |
| `0x010` | 48 | version string |
| `0x040` | 4 | `trace_mask` |
| `0x044` | 4 | `print_mask` |
| `0x048` | 4 | `cmd_hart`: hart that runs the next command |
| `0x04C` | 4 | `cmd`: command number, 0 when idle |
| `0x050` | 12 | `arg0`, `arg1`, `arg2` |
| `0x05C` | 8 | `ret0`, `ret1` |
| `0x064` | 4 | `done`: count of completed commands |
| `0x068` | 4 | `status` of the last command |
| `0x080` | 8 x 48 | hart records |
| `0x200` | 16 x 8 | symbol table |
| `0x280` | 96 x 4 | counters |
| `0x400` | 8 x 16 x 16 | trace rings, one per hart |
| `0xC00` | 1024 | copy buffer |

### Hart records

Hart `n` starts at `0x1E906880 + n * 0x30`:

| offset | field |
|---:|---|
| `+0x00` | loop tag: which loop the hart runs |
| `+0x04` | heartbeat: bumped on every pass of that loop |
| `+0x08` | 1 while the hart is inside an interrupt handler |
| `+0x0C` | trace write index |
| `+0x10` | mailbox commands this hart handled |
| `+0x14` | last mailbox command: `slot << 24 \| buffer bits 15:0 << 8 \| return` |
| `+0x18` | exceptions taken |
| `+0x1C` | `mcause` of the last exception |
| `+0x20` | `mepc` of the last exception |
| `+0x24` | `mtval` of the last exception |
| `+0x28` | `ra` of the last exception |
| `+0x2C` | `sp` of the last exception |

| tag | loop |
|---|---|
| `TUNL` | tunnel offload and L4S |
| `ERXD` | eagle rxdmad ring |
| `ETXF` | eagle WiFi tx fast path |
| `EC3L` | eagle host adaptor, tx done, package check |
| `EC0L` | eagle AN7552 core 0: queue drain and refill |
| `ERFL` | eagle rx refill |
| `KRX1`, `KRX2`, `KPIP` | kite rx, 2.4 GHz rx, classifier |
| `DBA5` | GPON DBA |
| `IDLE` | nothing to run; serves debug commands |

A hart whose tag is 0 never reached its main loop. A hart whose
heartbeat does not change between two reads is stuck: the loop is
blocked inside one pass. A tag with a moving heartbeat is alive.

### Symbol table

Sixteen `{tag, NPU address}` pairs locate internal state without a map
file. Convert the address to host view before reading it.

| tag | what |
|---|---|
| `GLOB` | start of the globals (`.data`) |
| `BSSE` | end of the globals |
| `MBOX` | mailbox handler table, 80 bytes per hart |
| `SRAM` | SRAM allocation table: `{u16 type, u16 pad, u32 addr}` per entry |
| `TICK` | timer tick counter |
| `BRDG` | variable holding the NPU bridge buffer address |
| `TUNF` | tunnel mail handler table |
| `L4SE` | L4S enable flag |
| `EDBG` | eagle datapath counters, `struct eagle_dbg` in `npu_wifi.h` |
| `KFLG` | kite debug flags: bit 2 turns the kite counter blocks on |
| `KC2G`, `KC5G` | variables holding the kite 2.4 GHz and 5 GHz counter block addresses |

### Counters

Word `i` is at `0x1E906A80 + i * 4`. They count from boot or from the
last `CLEAR` command.

| word | counter |
|---:|---|
| 0 | tunnel packets taken off the bridge |
| 1, 2 | VXLAN encapsulated, decapsulated |
| 3, 4 | SRv6 encapsulated, endpoint |
| 5 | address mapping (MAP-T, UDF 65..68) |
| 6, 7 | IP fragmentation, reassembly |
| 8 | tunnel packets dropped by the handler |
| 9 | packets with an unknown UDF |
| 10 | bridge egress refused for lack of credit |
| 12, 13 | L4S packets forwarded, packets marked CE |
| 14 | last sampled L4S queue length |
| 16, 17 | HWNAT mails, last `funcId << 8 \| result` |
| 20, 21 | DBA mails, DBA frame interrupts |
| 24 | TDMA tx submits that gave up on a full ring |
| 32, 33 | SRAM allocations, bytes allocated |

## Commands

A command runs on the hart named in `cmd_hart`, from that hart's main
loop. Pick a hart whose heartbeat moves; an `IDLE` hart, or core 3 on
the eagle parts, answers at once. The tunnel hart only answers between
packets.

1. write `arg0` and `arg1` (`0x1E906850`, `0x1E906854`),
2. write the hart number to `0x1E906848`,
3. write the command number to `0x1E90684C` (last),
4. read `0x1E90684C` until it is 0, then read `status` (`0x1E906868`)
   and `ret0`/`ret1` (`0x1E90685C`, `0x1E906860`).

| cmd | name | args | result |
|---:|---|---|---|
| 1 | `PING` | | `ret0` hart, `ret1` cycle counter |
| 2 | `READ` | `arg0` NPU address | `ret0` word |
| 3 | `WRITE` | `arg0` NPU address, `arg1` value | `ret0` value read back |
| 4 | `COPY` | `arg0` NPU address, `arg1` bytes (max `0x400`) | words at `0x1E907400`, `ret0` bytes copied |
| 5 | `HEXDUMP` | `arg0` NPU address, `arg1` bytes (0: 64) | console |
| 6 | `STATUS` | `arg0` subsystem mask (0: all) | console |
| 7 | `CLEAR` | | counters and trace rings zeroed |
| 8 | `TRACE` | | trace rings to the console, oldest first |
| 9 | `BRIDGE` | `arg0` 0 dump, 1 reset, 2 flush reassembly | console (tunnel parts) |
| 10 (`0xA`) | `SRAM` | | SRAM allocation table to the console |
| 11 (`0xB`) | `CSR` | `arg0` 0 `mstatus`, 1 `mie`, 2 `mip`, 3 `mtvec`, 4 `mcycle`, 5 `minstret`, 6 `mhartid` | `ret0` |

`status`: 0 done, 1 unknown command, 2 bad argument, 3 not built for
this part. `READ`, `WRITE` and `COPY` take the NPU's own view, so they
also reach what the host cannot map, such as the firmware image at
`0x84000000`.

Console output is the NPU's lines on the shared serial console, each
prefixed `[Cn]` with the hart that printed it.

```
# sys memwl 1e906848 3; sys memwl 1e90684c 6
[C3][DBG] TLB7.8.0.0_v003.MT7993.2416528 harts 6 tick 10144
[C3][DBG] C0 loop 54554e4c beat 588883457 mails 46 last 1400001 traps 0 cause 0 epc 0
[C3][DBG] C1 loop 45525844 beat 8724252 mails 0 last 0 traps 0 cause 0 epc 0
...
[C3][DBG] tunnel pkts 1699652 vxlan 0/0 srv6 629769/600607 map 469276 frag 0 reasm 0 drop 0 invalid 0 egress_fail 0
[C3][DBG] bridge ch0 waiting 0 credits 8
[C3][DBG] l4s on qid 7 thresh 100 pkts 869291 marks 490 qlen 0
[C3][DBG] ppe mails 1 last 101
[C3][DBG] sram allocs 23 used 6ff60
```

## Trace and print masks

Bit `n` of `trace_mask` (`0x1E906840`) records events of subsystem `n`
in the trace ring of the hart that saw them. The same bit in
`print_mask` (`0x1E906844`) also prints each event to the console, except
events raised inside an interrupt handler: those stay in the ring, since
the handler may have interrupted a print.

| bit | subsystem | events (`ev`, `a`, `b`) |
|---:|---|---|
| 0 | `mbox` | `mailbox << 8 \| slot`, buffer address, return value |
| 1 | `wifi` | `funcType << 8 \| funcId`, words 2 and 3 of the WiFi mail |
| 2 | `tdma` | band, software index, hardware index of a full tx ring |
| 3 | `tunnel` | 1 drop (descriptor, length), 2 unknown UDF (UDF, descriptor word 1) |
| 5 | `ppe` | HWNAT `funcId`, result, first argument |
| 6 | `dba` | DBA `funcType`, buffer address |
| 7 | `sram` | address type, address, size |
| 8 | `trap` | `mcause`, `mepc`, `mtval` |
| 15 | `stats` | print only: eagle datapath counters every two seconds from core 3 |

Print bit 1 (`wifi`) also prints the first host tx frame and the first
WiFi tx descriptors per band, and the first four large rx descriptors.

A ring entry is `{tick, hart << 28 | subsystem << 20 | ev, a, b}`; the
ring of hart `n` starts at `0x1E906C00 + n * 0x100` and holds the last
16 events. The hart's write index (hart record `+0x0C`) modulo 16 is the
next slot. `TRACE` prints every ring in order.

The `NPUDBG=1` build starts with print bits 1 and 15 set.

## Recipes

**Is the firmware up, and which one?** `sys memory 1e906800 40`: `NDBG`
and the version string. No magic means the NPU never ran `npu_init`.

**Is every hart alive?** `sys memory 1e906880 180` twice, a second
apart. Each hart record starts with its loop tag; the next word is the
heartbeat. A frozen heartbeat names the stuck hart; its trap fields
say whether it took an exception.

**Did the host's command reach the NPU?** Set `trace_mask` to 3, repeat
the host action, then run `TRACE` on an idle hart. Each mailbox command
shows the slot, the buffer and the return value; WiFi mails also show
the function type and id. The hart record's mail count and last mail
say the same without tracing.

**WiFi frames stop.** Set `print_mask` to `8000` for two-second counter
lines from core 3, or run `STATUS` with mask 2 once. The counters in
`struct eagle_dbg` say which stage frames stop at: taken off the host
ring (`in`), staged, pushed to the WiFi ring, rx descriptors parsed,
queued to the host, sent to the wire. Kite parts keep per-band counter
blocks that run only while bit 2 of the byte at `KFLG` is set: read the
word holding that byte, set the bit, write the word back.

**Tunnel or L4S traffic misbehaves.** `STATUS` with mask `18`: per
class tunnel counts, drops, unknown UDFs, egress refusals, and for each
bridge channel the packets waiting and the egress credits. Waiting
packets that do not drain with credits available mean the tunnel hart
is not polling; check its heartbeat. `BRIDGE` with `arg0` 0 prints the
bridge's own per-channel counters. The host side counts the same path:
`echo dump > /proc/tc3162/hwnat_npu_debug` prints the frame engine's
counters towards the NPU followed by the bridge counters.

**Where did SRAM go?** `SRAM` prints every allocation with its type and
address. The host-view base of any ring is the address with the top
three bits cleared.

**Read NPU-only memory.** `COPY` with the NPU address and a length,
then `sys memory 1e907400 LEN`. For one word, `READ`.
