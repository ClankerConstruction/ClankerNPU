# Vendor Firmware Errata

Defects found in the vendor NPU firmware for these parts, how they
show, and what this firmware does about them. Host-side issues that
decide whether an NPU path is used at all are listed after them.

## NPU firmware

| id | parts | defect | this firmware |
|---|---|---|---|
| E1 | AN7581, AN7583 | Tunnel mail dispatch indexes its 10-entry handler table with the host's function id unchecked. An id of 10 or more calls whatever word follows the table. | Fixed: ids past the table return 0, like an empty slot. |
| E2 | kite (MT7916, MT7996), pipeline mode | `tdma_tx_submit` takes hardware mutex 0 and keeps it when it gives up on a full TDMA ring. The other forwarding hart then waits on the mutex forever. | Fixed: the mutex is released on the give-up path. |
| E3 | kite, pipeline mode | A frame the classifier hart cannot take is returned to the software buffer pool, which this mode never allocates from. Its buffer id is lost. | Fixed: the buffer goes back to the buffer manager. |
| E4 | AN7581, AN7583 | The frame engine sends IP fragmentation work to NPU bridge channel 3. No hart polls channel 3: the tunnel hart polls its own channel and L4S polls 1 and 2. | Kept. The shipped host never binds a flow that needs fragmenting and sends oversized packets of bound flows to the CPU, so nothing reaches channel 3. |
| E5 | AN7583, GPON DBA | New-DBA mode keeps a 4-entry ONU list and writes past it with more than four ONUs. | Kept; the list size is part of the host interface. |
| E6 | AN7581, AN7583 | L4S keeps one packet count, mark count and sampled queue length for channels 1 and 2 together. Its debug line prints the mark threshold where it says `qlen`. | Kept, log text included. The debug block adds the sampled queue length as counter 14. |
| E7 | all | An exception prints from the trap handler. If it hits while the same hart holds the print mutex, the hart freezes. | The trap is recorded in the debug block before the print, so a frozen hart still leaves `mcause`, `mepc`, `mtval`, `ra` and `sp` behind. |
| E8 | AN7581, AN7583 | Hart 0 prints `Error: src:8 already registered ISR` at boot: the tunnel loop registers its mailbox source a second time. | Kept. Registering again only enables the source on the calling hart, which is what the loop needs. |

## Host side

These come from the host software shipped with the vendor firmware and
show with either NPU image.

| id | feature | issue | workaround |
|---|---|---|---|
| H1 | MAP-T | The IPv4/IPv6 translator tracks TCP windows. When the PPE offloads one direction of a TCP flow first, the translator no longer sees that direction's data and drops the other direction's segments as out of window, so that direction never binds. The connection stalls at a few hundred kbit/s. UDP is not affected. | Bind both directions at once: `hw_nat -N 1` (bind threshold 1 packet per second). |
| H2 | SRv6 | Forwarded LAN traffic into an `encap seg6` route re-enters IPv6 forwarding. The default IPv6 firewall drops it without a counter. | Accept the forwarded IPv6 traffic on the WAN, and set `ip sr tunsrc` to the address given to the NPU with `myip`. |
| H3 | VXLAN | On AN7581 and AN7583 the host hands VXLAN to the frame engine's tunnel table and never stores an NPU header template, so NPU UDFs 1..40 are never used. | None needed; VXLAN runs in hardware. |
| H4 | L4S | Enabling L4S affects only flows bound afterwards. | Flush the flow table after `echo enable > /proc/tc3162/hwnat_l4s`. |

## Verified paths

On AN7583 with MT7993, this firmware and the vendor firmware give the
same results for routing, VXLAN, SRv6 (NPU channel 0), L4S with CE
marking (channels 1 and 2), and WiFi to wired and wired to WiFi
forwarding. MAP-T TCP stalls on both as H1 describes; with H1 applied
this firmware carries it through UDF 65 (up) and 66 (down).
