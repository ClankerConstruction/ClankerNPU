# Eagle WiFi Datapath (MT7991, MT7992, MT7993)

The eagle chips reorder frames themselves (RRO) and hand the NPU one
descriptor per received frame on the rxdmad ring. The NPU forwards each
frame to the wired side or to the host, keeps the chip's rx rings
stocked, and on AN7581 and AN7583 also feeds the chip's tx rings.

Code: `npu_wifi_eagle.c` (mail handlers, ring setup) and
`npu_wifi_eagle_dp.c` (the per-core loops).

## Cores

```mermaid
flowchart LR
  subgraph HOST["ARM host"]
    DRV["WiFi driver"]
  end
  FE["frame engine / PPE"]
  subgraph NPU
    C0["core 0<br/>PPE buffer return ISR"]
    C1["core 1<br/>rxdmad"]
    C2["core 2<br/>tx fast path"]
    C3["core 3<br/>host adaptor, tx done"]
    C4["core 4<br/>rx refill"]
  end
  WIFI["WiFi chip"]

  WIFI -->|"rxdmad 16B x1536"| C1
  C1 -->|"dst_sel=1: TDMA tx"| FE
  C1 -->|"packet queue 12B x512"| C3
  FE -->|"not forwarded: FIFO"| C0
  C0 -->|"packet queue"| C3
  C3 -->|"out ring 24B x512"| DRV
  DRV -->|"in ring 208B"| C3
  C3 -->|"staging 16B x512"| C2
  FE -->|"TDMA rx: bound LAN flows"| C2
  C2 -->|"tx ring 16B x2048"| WIFI
  WIFI -->|"tx done 16B"| C3
  C4 -->|"rx rings 16B x1536 x2"| WIFI
```

| core | loop | work |
|---:|---|---|
| 0 | `ppe_wifi_bufid_isr` (PLIC 95) | frames the PPE did not forward go to the host queue; free-only entries return the buffer, 16 to a hold of mutex 13 with `HAS_ID_BATCH`. With `HAS_HOT_TEXT` it reads the FIFO count again before returning, up to 256 entries |
| 1 | `eagle_rxdmad_loop` | one rxdmad descriptor at a time: `dst_sel` frames to TDMA tx, the rest to the host queue; chains multi-buffer frames. An empty ring waits 500 delay loops, 20 with `HAS_FAST_POLL` |
| 2 | `eagle_tx_fast_path` | staged host frames and the TDMA rx ring into the WiFi tx ring, paced by the slots the chip has handed back |
| 3 | `eagle_core3_loop` | host adaptor in rings to staging, packet queue to host adaptor out rings, tx done ring |
| 4 | `eagle_rx_refill_loop` | refills both rx rings |

Core 2 counts a band's free room from the tx ring itself, not from the
PCIe DMA index (1460 cycles a read). The chip hands slots back in order
and sets bit 31 of each descriptor's word 1, so slot `cpu + n - 1` being
back means `n` slots are free. `eagle_tx_ring_room` reads the last slot
of the range first and binary searches only when it is not back: one to
eleven SRAM reads. Five slots stay free.

Without `HAS_FAST_POLL` (AN7581) core 2 serves one band at a time: it
polls that band once per free slot, about 1900 times on an idle ring,
then waits 1000 delay loops. A frame for the other band waits out that
spin, a few hundred us. With `HAS_FAST_POLL` (AN7583) each pass serves
both bands once and keeps the room from the last count, lowered as it
fills slots; it counts again when that drops to 165 slots or fewer. A
pass takes up to 32 staged host frames (`EAGLE_TX_HOST_BUDGET`),
then up to 128 LAN frames into the room left, keeping 6 slots free. The
cpu index goes to the chip once per batch of host frames and once per 8
LAN frames. A LAN frame goes to the ring its descriptor names (word 4
bit 25), which need not be the TDMA ring's band; every band a batch
filled is published, and `lanxband` in the stats print counts such
frames.

AN7552 has two cores and no NPU tx path: core 1 runs the rxdmad loop and
core 0 runs `eagle_core0_loop`, which drains the packet queues and
refills the rx rings. See [AN7552](#an7552).

## Rings and buffers

| ring | location | entry | filled by |
|---|---|---:|---|
| rxdmad (indirect command) | PCIe descriptor block + `0x0E0A0` | 16 B | WiFi chip |
| rx ring 0, 1 | block + `0x00000`, `0x140A0` | 16 B | core 4 |
| MSDU page ring | block + `0x0E020` | 16 B | NPU, at ring init |
| WiFi tx ring 0, 1 | block + `0x06020`, `0x1A0C0` | 16 B | core 2 |
| tx done | host-given base | 16 B | WiFi chip |
| host adaptor in 0, 1 | `0x1EC0D0A0`, `0x1EC0D0B0` | 208 B | host |
| host adaptor out 0, 1 | `0x1EC0D180`, `0x1EC0D190` | 24 B | core 3 |
| TDMA tx 0, 1 | `0x1FB50800`, `0x1FB50810` | 8 B | core 1 |
| TDMA rx 0, 1 | `0x1FB50900`, `0x1FB50910` | 32 B | PPE |
| PPE buffer return | `0x1FB50FDC`..`0x1FB50FE4` | FIFO | PPE |

The PCIe descriptor block is SRAM type 1; see
[memory.md](memory.md#pcie-descriptor-block).

Two buffer pools:

- **rx buffer ids**: 12288 ids (11200 on AN7552) over the WiFi packet
  buffer, 2 KB each with 192 bytes of headroom. Whoever finishes with a
  frame returns its id: the PPE return ISR on core 0, the rxdmad hart
  on a drop, core 3 after the host copy. Returns take mutex 13 and read
  the ring's write index only while holding it. With `HAS_HOT_TEXT`
  mutex 13 and the packet queue's mutex 10 are taken inline.
- **tx tokens**: 13312 tokens over the NPU tx packet buffer (13056 in
  the free ring with `HAS_EAGLE_TX_JUMBO`). The tx done
  ring returns them: each slot holds an rx buffer id whose buffer the
  chip fills with a token report, 15-bit tokens two per word after a
  12-byte header.

On AN7552 and AN7581 (`HAS_EAGLE_SYNC`) the refill marks an rx buffer id
once it has left its descriptor, and clears the mark of the id it puts
there; a new ring's ids start marked on AN7581. The rxdmad path waits
1 ms for a segment whose id is not marked yet. SRAM type 31 holds the
marks: a byte per id on AN7552, a bit per id on AN7581, as the stock
images keep them.

Each token owns one 2 KB NPU tx buffer (`token << 11` past the tx
packet buffer base). A LAN frame never needs more: TDMA rx fills one
buffer. A frame the host sends through the host adaptor can be longer,
up to the WLAN MTU. With `HAS_EAGLE_TX_JUMBO` the top 256 token ids
(13056 to 13311) never enter the free ring; they form 128 pairs whose
two buffers are adjacent. Core 3 stages a host frame over 2048 bytes in
the first token of a free pair, copied whole up to 4096 bytes, and gives
the pair back when the tx done report lists that token. Staging and tx
done both run on core 3, so the pairs' free stack takes no lock. With
every pair in flight the frame stays in the host in ring until one is
back. The ring reset (`np_skb_tx_force_reset`) leaves the pairs out too
and frees them all.

With `HAS_CACHED_TXDONE` (AN7583) core 3 reads a token report through
the D-cache alias (`0x8xxxxxxx`) instead of the uncached one. The cache
does not see the chip's writes, so it first drops every 64-byte line the
report covers with the vendor op `0xFC2` ([platform.md](platform.md#core-isa-and-data-cache)),
bounded to the 2 KB buffer. One line fill then serves 16 words; the
uncached read cost 80 to 95 cycles a word. A report of about 63 tokens
(a 5 GHz download) took 14265 cycles and takes 10618; one of about 39
tokens (2.4 GHz) 10347 and 7855 (`ETXD`, `PROF=1`). The stock firmware
reads its RRO MSDU pages the same way.

With `HAS_ID_BATCH` (AN7583) both pools move ids in batches, one mutex
hold each: the refill takes up to 32 ids for the slots the chip handed
back, the host drain returns the ids of up to 16 frames, the tx done
ring frees up to 32 tokens, and the LAN to WiFi drain takes tokens for
up to 8 filled TDMA descriptors and gives back any it did not use. The
host drain stops after a frame the full out ring refused.

## Bring-up

The host hands over the rings with WiFi SET_WAIT and GET_WAIT mails
(see [mailbox.md](mailbox.md#wifi-commands-slot-0)). The interface id in
the mail header is a ring id.

| ring id | ring |
|---:|---|
| 0, 1 | rx rings |
| 5, 6 | MSDU page ring / WiFi tx rings |
| 8, 9 | indirect command (rxdmad) ring |
| 10, 11 | tx done rings |
| 15 | all bases set |

Each loop waits for its own set of flags:

| core | starts when |
|---:|---|
| 1 | rx and tx enabled (SET_WAIT 24 cases 2 and 7) |
| 2 | core 0 init done, tx done ring set up (SET_WAIT 1, ring 10), tx on (case 7) |
| 3 | core 0 init done, RRO state up and tx enabled |
| 4 | core 0 init done, rx and tx enabled, both rx rings set up (SET_WAIT 1, rings 0 and 1) |

A stop (case 4) clears the flags and every loop idles until they are set
again.

`GET_WAIT 4` answers per ring id with a physical address
(`& 0x1FFFFFFF`), and the host programs the chip from it:

| id | answer |
|---:|---|
| 0, 1 | rx ring base |
| 5, 6 | WiFi tx ring base; sets every descriptor to `0x80000000` first |
| 8, 9 | rxdmad ring base |
| 10 | MSDU page ring base |

`SET_WAIT 24` (inode tx/rx register) cases:

| case | action |
|---:|---|
| 0 | RRO address element table: 128 tables of 64 KB, 8 sessions each |
| 1 | particular session table |
| 2 | rx on: TDMA flow control on, rx and tx enabled |
| 3 | re-arm one session's elements |
| 4 | stop: TDMA flow control off, every loop pauses |
| 5 | host register for the rxdmad cpu index |
| 6 | restart: wait for TDMA idle, rebuild buffer ids and counters |
| 7 | tx on |

## PCIe windows

The rings live in NPU SRAM. The WiFi chip reaches them through a PCIe
inbound window, a base and an end address, both physical. Until
`SET_WAIT 14` opens it, the chip can neither fetch a tx descriptor nor
write an rx one, and its DMA index never moves.

| register | PCIe 0 | PCIe 1 |
|---|---|---|
| base | `0x1FA90038` | `0x1FC28030` (`0x1FA90030` on AN7552) |
| end | `0x1FA9003C` | `0x1FC28034` (`0x1FA90034` on AN7552) |

| port type | mapping |
|---:|---|
| 0 | PCIe 1 gets the whole block |
| 1 | PCIe 0 gets the whole block |
| 2 | band 0 (up to rx ring 1) on PCIe 1, band 1 on PCIe 0 |
| 3 | band 0 on PCIe 0, band 1 on PCIe 1 |

## Descriptors

- **Own bit.** Bit 31 of word 1 on rx and tx rings; set means the NPU
  owns the slot. A refill hands a slot back with
  `{phys + 192, 0x07000100, id << 16}`. A tx push writes `0x4C4048`.
- **Generation.** The rxdmad ring has no own bit. Each descriptor
  carries a 4-bit generation in the top nibble of word 3 (word 1 on the
  8-byte indirect command ring) that the chip increments on every wrap;
  the NPU tracks the one it expects. Ring init stamps every descriptor
  with `0xF` (`0xE` on the narrow ring), a value the NPU never expects,
  so an untouched ring reads as empty.
- **CPU index.** Every ring the chip fills needs its cpu index advanced,
  or the chip stalls when it catches up:

  | ring | when | written to |
  |---|---|---|
  | rxdmad | every 128 descriptors | ring register + 8, or the host register from case 5 |
  | rx ring 0, 1 | every 128 refills | ring register + 8 |
  | tx done | every 16 | ring register + 8 |

## Traffic directions

- **WiFi to LAN.** Core 1 sends `dst_sel=1` frames to TDMA tx. The PPE
  forwards bound flows and returns the rest on the buffer return FIFO;
  core 0 queues those to the host. With `HAS_HOT_TEXT` (AN7583) a whole,
  error-free frame with WiFi printing off takes a call-free path in
  `eagle_rxdmad_loop` to TDMA tx or the band 1 queue; any other
  descriptor goes to `eagle_rxdmad_handle`.
- **WiFi to host.** Frames without `dst_sel`, with errors, or with force
  to CPU set go on the packet queue; core 3 copies them to the host
  adaptor out ring and returns the buffer id. `EDBG` counts why, for
  frames `eagle_rxdmad_handle` sees: `rxh_err` (descriptor error),
  `rxh_flag` (the chip asks for the host) and `rxh_raw` (no ethernet
  header offset). Frames the PPE returns are not among them.
  Frames spanning several buffers go on the multi-segment queue. Core
  3 hands a frame up only whole: for a segment core 1 has not queued
  yet it polls that entry up to 10000 times, three times at most, then
  retries on its next pass.
- **Host to WiFi.** Core 3 copies host TXDs from the in ring into the
  staging ring; core 2 moves them into the ring 0/1 TXD space. TXD word 7
  bit 27 selects the token layout.
- **LAN to WiFi.** The host sends a flow itself until HWNAT binds it.
  After that the PPE delivers its frames on TDMA rx. Core 2 swaps a
  fresh tx token into the TDMA slot and fills the TXP words
  (`token << 16 | 0x80`, `wcid << 8 | bss | 0x1000000`, buffer, length)
  in the ring 5/6 TXD space.
- **Tx done.** Token reports (types 6 and 24) free tx tokens. Any other
  event is queued to the host on band 0.

## Per-station queue limit

LAN to WiFi frames bypass the host's queueing, so the only limit on what
waits in the WiFi chip is the tx token pool, which every station and the
host path share. One fast download can hold most of it: the TCP window
then sits in the chip as delay, and when the pool runs dry the NPU drops
frames and host frames to every station wait for a token.

With `HAS_EAGLE_STA_QLIMIT` (AN7581 and AN7583, the eagle builds with an
NPU tx path) core 2 counts each station's frames in the chip, times one
of them at a time through the chip, and drops a TDMA rx frame,
re-arming its slot, when:

- the station has `limit` frames in the chip, or
- the station's queue has stood above target for `interval`: its frames
  spend `delay` or more in the chip with at least `min_q` of them there,
  or its count is at or above `target`. Drops then follow CoDel's
  control law (RFC 8289): the next one comes `interval / sqrt(n)` later,
  and they stop as soon as the queue is below target. Bursts shorter
  than `interval` pass untouched, and frames of `small` bytes or less
  are never dropped this way.

| field of `wifi_sta_q` | default | |
|---|---:|---|
| `limit` | 8192 | frames; leaves about 2800 tokens for others |
| `target` | 0 | frames; off |
| `interval` | 100 ms | in cycles |
| `delay` | 10 ms | in cycles |
| `min_q` | 64 | frames |
| `small` | 256 | bytes |

Zero turns a field off. The fields can change at run time; the debug
block tag `SQLM` gives their address. [sta-qlimit.md](sta-qlimit.md)
has the measurements behind the defaults, the trade-offs and the host
commands.

The accounting and the drop decision are in `npu_sta_q.c` and know
nothing of eagle; the datapath calls four hooks: `sta_q_drop` before a
frame takes a token, `sta_q_sent_tok` before the chip can see it,
`sta_q_unsent_tok` when the ring write fails, and `sta_q_done_tok` for
each token a tx done report frees.

The station is the wcid in TDMA rx word 4. SRAM type 41 holds a station
per tx token, and per station the frames sent (written by core 2 only),
the frames done (core 3 only), the drop state (core 2 only) and the
probe that times one frame (core 2 arms it, core 3 completes it), for
wcids below 1024. A token maps to no station while it is free or carries
a host frame. `limit_drops` and `aqm_drops` in `wifi_sta_q` count the two
kinds of drop.

With `USE_CLANKER_DRIVER` (`make CLANKER=1`) host frames count too, and
may be dropped: the vendor host driver does not expect that, so default
builds leave them alone. Core 3 stages host frames, so it finds their
station in the TXP behind the TXD:

| TXD / TXP form | station |
|---|---|
| HIF TXP v1 (vendor default, mt76) | `rept_wds_wcid`, TXP bytes 5-6 |
| HIF TXP v2/v3 (vendor SW A-MSDU): TXD word 0 version 2 or 3 in bits 22:19, word 1 zero | TXP word 2 bits 27:16 |
| MAC TXP (TXD word 7 bit 27, AddBA) | none, not limited |

Each counter keeps one writer: core 3 counts host frames sent and dropped
in their own arrays, and the decision uses host sent + LAN sent - done.
SRAM type 41 grows to 64 KB for them.

## AN7552

- Two cores: core 1 rxdmad, core 0 packet queue drain and rx refill,
  started once init, rx, tx and both rx rings are up.
- No NPU tx path (`HAS_NPU_WIFI_TX` unset): no host adaptor in rings, no
  tx tokens, no TDMA rx ring. The tx ring mails print
  `TCSUPPORT_NPU_WIFI_TX is not set` and return 1.
- Core 0 masks PLIC 95 around buffer frees in its drain loop, because
  the ISR on the same hart frees under the same mutex.
- Chip family 15 with package variant 0 forces every frame to the CPU.
- A sync byte per buffer id (SRAM type 31) lets the rxdmad path wait
  briefly for a refill that has not landed yet.
