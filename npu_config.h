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

/* The trap entry saves only the registers a C call may clobber.
 * AN7583 eagle takes a PPE buffer return interrupt per frame. */
#if defined(AN7583) && defined(WIFI_EAGLE)
#define HAS_LEAN_TRAP
#endif

#endif /* NPU_CONFIG_H */
