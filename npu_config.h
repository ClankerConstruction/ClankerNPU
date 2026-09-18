#ifndef NPU_CONFIG_H
#define NPU_CONFIG_H

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
#define AN75XX
#elif defined(AN7581)
#define MAX_CORE_NUM    8
#define AN75XX
#define AN758X
#elif defined(AN7583)
#define MAX_CORE_NUM    6
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

#endif /* NPU_CONFIG_H */
