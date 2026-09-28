#ifndef NPU_CONFIG_H
#define NPU_CONFIG_H

/* Boot version line: release.wifi chip.git hash */
#define NPU_INIT_VERSION    "TLB7.8.0.0_v003"
#define NPU_VERSION	NPU_INIT_VERSION "." NPU_WIFI_NAME "." NPU_GIT_REV

/*
 * Build-time variant selection.
 * Define exactly one SoC: AN7552, AN7581, AN7583
 * Define exactly one WiFi chip (or NOWIFI): MT7916, MT7991, MT7992, MT7993, MT7996, NOWIFI
 *
 * SoC → core count:
 *   AN7552 → 2 cores
 *   AN7581 → 8 cores
 *   AN7583 → 6 cores
 *
 * WiFi chip → driver path:
 *   MT7916, MT7996              → kite (TDMA RX path, WiFi chip name table)
 *   MT7991, MT7992, MT7993      → eagle (RRO, MSDU page ring, ind cmd ring)
 *   NOWIFI                      → no WiFi offload
 */

#if defined(AN7552)
#define MAX_CORE_NUM    2
#define NPU_TIMER_NUM   8
#define AN75XX
#elif defined(AN7581)
#define MAX_CORE_NUM    8
#define NPU_TIMER_NUM   4
#define AN75XX
#define AN758X
#elif defined(AN7583)
#define MAX_CORE_NUM    6
#define NPU_TIMER_NUM   16
#define AN75XX
#define AN758X
#else
#error "Define one SoC: AN7552, AN7581, or AN7583"
#endif

#if defined(MT7916) || defined(MT7996)
#define WIFI_KITE
#define HAS_WIFI
#elif defined(MT7991) || defined(MT7992) || defined(MT7993)
#define WIFI_EAGLE
#define HAS_WIFI
#elif defined(NOWIFI)
/* no WiFi offload */
#else
#error "Define one WiFi chip: MT7916, MT7991, MT7992, MT7993, MT7996, or NOWIFI"
#endif

#if defined(AN7583) && !defined(NOWIFI)
#define HAS_DBA
#endif

#if defined(AN758X)
#define HAS_TUNNEL
#endif

#if defined(AN7581) && defined(HAS_WIFI)
#define HAS_TR471
#endif

#if !defined(AN7581) && defined(HAS_WIFI)
#define HAS_BME
#endif

/* The host pushes WiFi tx frames through the host adaptor ring instead of
 * the WiFi PCIe ring. Not built for AN7552 or for MT7916; on AN7583 only
 * the eagle chips carry it. */
#if defined(HAS_WIFI) && ((defined(AN7581) && !defined(MT7916)) || \
			  (defined(AN7583) && defined(WIFI_EAGLE)))
#define HAS_NPU_WIFI_TX
#endif

/* Per-packet functions in one .text.hot block, small helpers forced
 * inline. Hart 1's rx path: AN7552 kite, AN7583 eagle. */
#if (defined(AN7552) && defined(WIFI_KITE)) || \
    (defined(AN7583) && defined(WIFI_EAGLE))
#define HAS_HOT_TEXT
#endif

/* Rx buffer ids and tx tokens move in batches, one mutex hold each */
#if defined(AN7583) && defined(WIFI_EAGLE)
#define HAS_ID_BATCH
#endif

/* WiFi rx and tx loops poll without long idle waits: core 2 serves
 * both bands every pass instead of spinning on one */
#if defined(AN7583) && defined(WIFI_EAGLE)
#define HAS_FAST_POLL
#endif

/* The out ring copy runs while the next frame is read and claimed */
#if defined(AN7583) && defined(WIFI_EAGLE)
#define HAS_ASYNC_COPY
#endif

/* LAN -> WiFi frames each station has in the WiFi chip: a hard limit,
 * and drops against a standing queue, as CoDel does. Needs the NPU tx
 * path with a tx token per frame (eagle). */
#if defined(HAS_NPU_WIFI_TX) && defined(WIFI_EAGLE)
#define HAS_EAGLE_STA_QLIMIT
#endif

/* A host tx frame longer than one 2 KB NPU tx buffer takes one of a few
 * reserved pairs of adjacent buffers instead of being cut short. */
#if defined(HAS_NPU_WIFI_TX) && defined(WIFI_EAGLE)
#define HAS_EAGLE_TX_JUMBO
#endif

/* Tx done reports are read through the D-cache after their lines are
 * invalidated, as the stock firmware reads RRO pages: one line fill
 * per 64 bytes instead of an uncached load per word. */
#if defined(AN7583) && defined(WIFI_EAGLE)
#define HAS_CACHED_TXDONE
#endif

/* The rx refill marks when an rx buffer id has left its descriptor;
 * the rxdmad path and TDMA tx wait briefly for an id not yet marked.
 * A byte per id on AN7552, a bit per id on AN7581, as the stock images. */
#if (defined(AN7552) || defined(AN7581)) && defined(WIFI_EAGLE)
#define HAS_EAGLE_SYNC
#endif

/* The trap entry saves only the registers a C call may clobber.
 * AN7583 eagle takes a PPE buffer return interrupt per frame. */
#if defined(AN7583) && defined(WIFI_EAGLE)
#define HAS_LEAN_TRAP
#endif

#endif /* NPU_CONFIG_H */
