/*---------------------------------------------------------------------------------------------------------------------------------------------------
 * irmp.c - infrared multi-protocol decoder, supports several remote control protocols
 *
 * Copyright (c) 2009-2015 Frank Meyer - frank(at)fli4l.de
 *
 * $Id: irmp.c,v 1.170 2015/01/28 09:18:30 fm Exp $
 *
 * Supported AVR mikrocontrollers:
 *
 * ATtiny87,  ATtiny167
 * ATtiny45,  ATtiny85
 * ATtiny44,  ATtiny84
 * ATmega8,   ATmega16,  ATmega32
 * ATmega162
 * ATmega164, ATmega324, ATmega644,  ATmega644P, ATmega1284, ATmega1284P
 * ATmega88,  ATmega88P, ATmega168,  ATmega168P, ATmega328P
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *---------------------------------------------------------------------------------------------------------------------------------------------------
 */

#include "irmp.h"

#if IRMP_SUPPORT_GRUNDIG_PROTOCOL == 1 || IRMP_SUPPORT_NOKIA_PROTOCOL == 1 || IRMP_SUPPORT_IR60_PROTOCOL == 1
#  define IRMP_SUPPORT_GRUNDIG_NOKIA_IR60_PROTOCOL  1
#else
#  define IRMP_SUPPORT_GRUNDIG_NOKIA_IR60_PROTOCOL  0
#endif

#if IRMP_SUPPORT_SIEMENS_PROTOCOL == 1 || IRMP_SUPPORT_RUWIDO_PROTOCOL == 1
#  define IRMP_SUPPORT_SIEMENS_OR_RUWIDO_PROTOCOL   1
#else
#  define IRMP_SUPPORT_SIEMENS_OR_RUWIDO_PROTOCOL   0
#endif

#if IRMP_SUPPORT_RC5_PROTOCOL == 1 ||                   \
    IRMP_SUPPORT_RC6_PROTOCOL == 1 ||                   \
    IRMP_SUPPORT_GRUNDIG_NOKIA_IR60_PROTOCOL == 1 ||    \
    IRMP_SUPPORT_SIEMENS_OR_RUWIDO_PROTOCOL == 1 ||     \
    IRMP_SUPPORT_IR60_PROTOCOL == 1 ||                  \
    IRMP_SUPPORT_A1TVBOX_PROTOCOL == 1 ||               \
    IRMP_SUPPORT_ORTEK_PROTOCOL == 1
#  define IRMP_SUPPORT_MANCHESTER                   1
#else
#  define IRMP_SUPPORT_MANCHESTER                   0
#endif

#if IRMP_SUPPORT_NETBOX_PROTOCOL == 1
#  define IRMP_SUPPORT_SERIAL                       1
#else
#  define IRMP_SUPPORT_SERIAL                       0
#endif

#define IRMP_KEY_REPETITION_LEN                 (uint_fast16_t)(F_INTERRUPTS * 150.0e-3 + 0.5)           // autodetect key repetition within 150 msec

#define MIN_TOLERANCE_00                        1.0                           // -0%
#define MAX_TOLERANCE_00                        1.0                           // +0%

#define MIN_TOLERANCE_05                        0.95                          // -5%
#define MAX_TOLERANCE_05                        1.05                          // +5%

#define MIN_TOLERANCE_10                        0.9                           // -10%
#define MAX_TOLERANCE_10                        1.1                           // +10%

#define MIN_TOLERANCE_15                        0.85                          // -15%
#define MAX_TOLERANCE_15                        1.15                          // +15%

#define MIN_TOLERANCE_20                        0.8                           // -20%
#define MAX_TOLERANCE_20                        1.2                           // +20%

#define MIN_TOLERANCE_30                        0.7                           // -30%
#define MAX_TOLERANCE_30                        1.3                           // +30%

#define MIN_TOLERANCE_40                        0.6                           // -40%
#define MAX_TOLERANCE_40                        1.4                           // +40%

#define MIN_TOLERANCE_50                        0.5                           // -50%
#define MAX_TOLERANCE_50                        1.5                           // +50%

#define MIN_TOLERANCE_60                        0.4                           // -60%
#define MAX_TOLERANCE_60                        1.6                           // +60%

#define MIN_TOLERANCE_70                        0.3                           // -70%
#define MAX_TOLERANCE_70                        1.7                           // +70%

#define SIRCS_START_BIT_PULSE_LEN_MIN           ((uint_fast8_t)(F_INTERRUPTS * SIRCS_START_BIT_PULSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define SIRCS_START_BIT_PULSE_LEN_MAX           ((uint_fast8_t)(F_INTERRUPTS * SIRCS_START_BIT_PULSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define SIRCS_START_BIT_PAUSE_LEN_MIN           ((uint_fast8_t)(F_INTERRUPTS * SIRCS_START_BIT_PAUSE_TIME * MIN_TOLERANCE_20 + 0.5) - 1)
#if IRMP_SUPPORT_NETBOX_PROTOCOL                // only 5% to avoid conflict with NETBOX:
#  define SIRCS_START_BIT_PAUSE_LEN_MAX         ((uint_fast8_t)(F_INTERRUPTS * SIRCS_START_BIT_PAUSE_TIME * MAX_TOLERANCE_05 + 0.5))
#else                                           // only 5% + 1 to avoid conflict with RC6:
#  define SIRCS_START_BIT_PAUSE_LEN_MAX         ((uint_fast8_t)(F_INTERRUPTS * SIRCS_START_BIT_PAUSE_TIME * MAX_TOLERANCE_05 + 0.5) + 1)
#endif
#define SIRCS_1_PULSE_LEN_MIN                   ((uint_fast8_t)(F_INTERRUPTS * SIRCS_1_PULSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define SIRCS_1_PULSE_LEN_MAX                   ((uint_fast8_t)(F_INTERRUPTS * SIRCS_1_PULSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define SIRCS_0_PULSE_LEN_MIN                   ((uint_fast8_t)(F_INTERRUPTS * SIRCS_0_PULSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define SIRCS_0_PULSE_LEN_MAX                   ((uint_fast8_t)(F_INTERRUPTS * SIRCS_0_PULSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define SIRCS_PAUSE_LEN_MIN                     ((uint_fast8_t)(F_INTERRUPTS * SIRCS_PAUSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define SIRCS_PAUSE_LEN_MAX                     ((uint_fast8_t)(F_INTERRUPTS * SIRCS_PAUSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)

#define NEC_START_BIT_PULSE_LEN_MIN             ((uint_fast8_t)(F_INTERRUPTS * NEC_START_BIT_PULSE_TIME * MIN_TOLERANCE_30 + 0.5) - 1)
#define NEC_START_BIT_PULSE_LEN_MAX             ((uint_fast8_t)(F_INTERRUPTS * NEC_START_BIT_PULSE_TIME * MAX_TOLERANCE_30 + 0.5) + 1)
#define NEC_START_BIT_PAUSE_LEN_MIN             ((uint_fast8_t)(F_INTERRUPTS * NEC_START_BIT_PAUSE_TIME * MIN_TOLERANCE_30 + 0.5) - 1)
#define NEC_START_BIT_PAUSE_LEN_MAX             ((uint_fast8_t)(F_INTERRUPTS * NEC_START_BIT_PAUSE_TIME * MAX_TOLERANCE_30 + 0.5) + 1)
#define NEC_REPEAT_START_BIT_PAUSE_LEN_MIN      ((uint_fast8_t)(F_INTERRUPTS * NEC_REPEAT_START_BIT_PAUSE_TIME * MIN_TOLERANCE_30 + 0.5) - 1)
#define NEC_REPEAT_START_BIT_PAUSE_LEN_MAX      ((uint_fast8_t)(F_INTERRUPTS * NEC_REPEAT_START_BIT_PAUSE_TIME * MAX_TOLERANCE_30 + 0.5) + 1)
#define NEC_PULSE_LEN_MIN                       ((uint_fast8_t)(F_INTERRUPTS * NEC_PULSE_TIME * MIN_TOLERANCE_30 + 0.5) - 1)
#define NEC_PULSE_LEN_MAX                       ((uint_fast8_t)(F_INTERRUPTS * NEC_PULSE_TIME * MAX_TOLERANCE_30 + 0.5) + 1)
#define NEC_1_PAUSE_LEN_MIN                     ((uint_fast8_t)(F_INTERRUPTS * NEC_1_PAUSE_TIME * MIN_TOLERANCE_30 + 0.5) - 1)
#define NEC_1_PAUSE_LEN_MAX                     ((uint_fast8_t)(F_INTERRUPTS * NEC_1_PAUSE_TIME * MAX_TOLERANCE_30 + 0.5) + 1)
#define NEC_0_PAUSE_LEN_MIN                     ((uint_fast8_t)(F_INTERRUPTS * NEC_0_PAUSE_TIME * MIN_TOLERANCE_30 + 0.5) - 1)
#define NEC_0_PAUSE_LEN_MAX                     ((uint_fast8_t)(F_INTERRUPTS * NEC_0_PAUSE_TIME * MAX_TOLERANCE_30 + 0.5) + 1)
// autodetect nec repetition frame within 50 msec:
// NEC seems to send the first repetition frame after 40ms, further repetition frames after 100 ms
#if 0
#define NEC_FRAME_REPEAT_PAUSE_LEN_MAX          (uint_fast16_t)(F_INTERRUPTS * NEC_FRAME_REPEAT_PAUSE_TIME * MAX_TOLERANCE_20 + 0.5)
#else
#define NEC_FRAME_REPEAT_PAUSE_LEN_MAX          (uint_fast16_t)(F_INTERRUPTS * 100.0e-3 * MAX_TOLERANCE_20 + 0.5)
#endif

#define SAMSUNG_START_BIT_PULSE_LEN_MIN         ((uint_fast8_t)(F_INTERRUPTS * SAMSUNG_START_BIT_PULSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define SAMSUNG_START_BIT_PULSE_LEN_MAX         ((uint_fast8_t)(F_INTERRUPTS * SAMSUNG_START_BIT_PULSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define SAMSUNG_START_BIT_PAUSE_LEN_MIN         ((uint_fast8_t)(F_INTERRUPTS * SAMSUNG_START_BIT_PAUSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define SAMSUNG_START_BIT_PAUSE_LEN_MAX         ((uint_fast8_t)(F_INTERRUPTS * SAMSUNG_START_BIT_PAUSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define SAMSUNG_PULSE_LEN_MIN                   ((uint_fast8_t)(F_INTERRUPTS * SAMSUNG_PULSE_TIME * MIN_TOLERANCE_30 + 0.5) - 1)
#define SAMSUNG_PULSE_LEN_MAX                   ((uint_fast8_t)(F_INTERRUPTS * SAMSUNG_PULSE_TIME * MAX_TOLERANCE_30 + 0.5) + 1)
#define SAMSUNG_1_PAUSE_LEN_MIN                 ((uint_fast8_t)(F_INTERRUPTS * SAMSUNG_1_PAUSE_TIME * MIN_TOLERANCE_30 + 0.5) - 1)
#define SAMSUNG_1_PAUSE_LEN_MAX                 ((uint_fast8_t)(F_INTERRUPTS * SAMSUNG_1_PAUSE_TIME * MAX_TOLERANCE_30 + 0.5) + 1)
#define SAMSUNG_0_PAUSE_LEN_MIN                 ((uint_fast8_t)(F_INTERRUPTS * SAMSUNG_0_PAUSE_TIME * MIN_TOLERANCE_30 + 0.5) - 1)
#define SAMSUNG_0_PAUSE_LEN_MAX                 ((uint_fast8_t)(F_INTERRUPTS * SAMSUNG_0_PAUSE_TIME * MAX_TOLERANCE_30 + 0.5) + 1)

#define MATSUSHITA_START_BIT_PULSE_LEN_MIN      ((uint_fast8_t)(F_INTERRUPTS * MATSUSHITA_START_BIT_PULSE_TIME * MIN_TOLERANCE_20 + 0.5) - 1)
#define MATSUSHITA_START_BIT_PULSE_LEN_MAX      ((uint_fast8_t)(F_INTERRUPTS * MATSUSHITA_START_BIT_PULSE_TIME * MAX_TOLERANCE_20 + 0.5) + 1)
#define MATSUSHITA_START_BIT_PAUSE_LEN_MIN      ((uint_fast8_t)(F_INTERRUPTS * MATSUSHITA_START_BIT_PAUSE_TIME * MIN_TOLERANCE_20 + 0.5) - 1)
#define MATSUSHITA_START_BIT_PAUSE_LEN_MAX      ((uint_fast8_t)(F_INTERRUPTS * MATSUSHITA_START_BIT_PAUSE_TIME * MAX_TOLERANCE_20 + 0.5) + 1)
#define MATSUSHITA_PULSE_LEN_MIN                ((uint_fast8_t)(F_INTERRUPTS * MATSUSHITA_PULSE_TIME * MIN_TOLERANCE_40 + 0.5) - 1)
#define MATSUSHITA_PULSE_LEN_MAX                ((uint_fast8_t)(F_INTERRUPTS * MATSUSHITA_PULSE_TIME * MAX_TOLERANCE_40 + 0.5) + 1)
#define MATSUSHITA_1_PAUSE_LEN_MIN              ((uint_fast8_t)(F_INTERRUPTS * MATSUSHITA_1_PAUSE_TIME * MIN_TOLERANCE_40 + 0.5) - 1)
#define MATSUSHITA_1_PAUSE_LEN_MAX              ((uint_fast8_t)(F_INTERRUPTS * MATSUSHITA_1_PAUSE_TIME * MAX_TOLERANCE_40 + 0.5) + 1)
#define MATSUSHITA_0_PAUSE_LEN_MIN              ((uint_fast8_t)(F_INTERRUPTS * MATSUSHITA_0_PAUSE_TIME * MIN_TOLERANCE_40 + 0.5) - 1)
#define MATSUSHITA_0_PAUSE_LEN_MAX              ((uint_fast8_t)(F_INTERRUPTS * MATSUSHITA_0_PAUSE_TIME * MAX_TOLERANCE_40 + 0.5) + 1)

#define KASEIKYO_START_BIT_PULSE_LEN_MIN        ((uint_fast8_t)(F_INTERRUPTS * KASEIKYO_START_BIT_PULSE_TIME * MIN_TOLERANCE_20 + 0.5) - 1)
#define KASEIKYO_START_BIT_PULSE_LEN_MAX        ((uint_fast8_t)(F_INTERRUPTS * KASEIKYO_START_BIT_PULSE_TIME * MAX_TOLERANCE_20 + 0.5) + 1)
#define KASEIKYO_START_BIT_PAUSE_LEN_MIN        ((uint_fast8_t)(F_INTERRUPTS * KASEIKYO_START_BIT_PAUSE_TIME * MIN_TOLERANCE_20 + 0.5) - 1)
#define KASEIKYO_START_BIT_PAUSE_LEN_MAX        ((uint_fast8_t)(F_INTERRUPTS * KASEIKYO_START_BIT_PAUSE_TIME * MAX_TOLERANCE_20 + 0.5) + 1)
#define KASEIKYO_PULSE_LEN_MIN                  ((uint_fast8_t)(F_INTERRUPTS * KASEIKYO_PULSE_TIME * MIN_TOLERANCE_50 + 0.5) - 1)
#define KASEIKYO_PULSE_LEN_MAX                  ((uint_fast8_t)(F_INTERRUPTS * KASEIKYO_PULSE_TIME * MAX_TOLERANCE_50 + 0.5) + 1)
#define KASEIKYO_1_PAUSE_LEN_MIN                ((uint_fast8_t)(F_INTERRUPTS * KASEIKYO_1_PAUSE_TIME * MIN_TOLERANCE_30 + 0.5) - 1)
#define KASEIKYO_1_PAUSE_LEN_MAX                ((uint_fast8_t)(F_INTERRUPTS * KASEIKYO_1_PAUSE_TIME * MAX_TOLERANCE_30 + 0.5) + 1)
#define KASEIKYO_0_PAUSE_LEN_MIN                ((uint_fast8_t)(F_INTERRUPTS * KASEIKYO_0_PAUSE_TIME * MIN_TOLERANCE_50 + 0.5) - 1)
#define KASEIKYO_0_PAUSE_LEN_MAX                ((uint_fast8_t)(F_INTERRUPTS * KASEIKYO_0_PAUSE_TIME * MAX_TOLERANCE_50 + 0.5) + 1)

#define RECS80_START_BIT_PULSE_LEN_MIN          ((uint_fast8_t)(F_INTERRUPTS * RECS80_START_BIT_PULSE_TIME * MIN_TOLERANCE_20 + 0.5) - 1)
#define RECS80_START_BIT_PULSE_LEN_MAX          ((uint_fast8_t)(F_INTERRUPTS * RECS80_START_BIT_PULSE_TIME * MAX_TOLERANCE_20 + 0.5) + 1)
#define RECS80_START_BIT_PAUSE_LEN_MIN          ((uint_fast8_t)(F_INTERRUPTS * RECS80_START_BIT_PAUSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define RECS80_START_BIT_PAUSE_LEN_MAX          ((uint_fast8_t)(F_INTERRUPTS * RECS80_START_BIT_PAUSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define RECS80_PULSE_LEN_MIN                    ((uint_fast8_t)(F_INTERRUPTS * RECS80_PULSE_TIME * MIN_TOLERANCE_20 + 0.5) - 1)
#define RECS80_PULSE_LEN_MAX                    ((uint_fast8_t)(F_INTERRUPTS * RECS80_PULSE_TIME * MAX_TOLERANCE_20 + 0.5) + 1)
#define RECS80_1_PAUSE_LEN_MIN                  ((uint_fast8_t)(F_INTERRUPTS * RECS80_1_PAUSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define RECS80_1_PAUSE_LEN_MAX                  ((uint_fast8_t)(F_INTERRUPTS * RECS80_1_PAUSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define RECS80_0_PAUSE_LEN_MIN                  ((uint_fast8_t)(F_INTERRUPTS * RECS80_0_PAUSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define RECS80_0_PAUSE_LEN_MAX                  ((uint_fast8_t)(F_INTERRUPTS * RECS80_0_PAUSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)


#if IRMP_SUPPORT_BOSE_PROTOCOL == 1 // BOSE conflicts with RC5, so keep tolerance for RC5 minimal here:
#define RC5_START_BIT_LEN_MIN                   ((uint_fast8_t)(F_INTERRUPTS * RC5_BIT_TIME * MIN_TOLERANCE_05 + 0.5) - 1)
#define RC5_START_BIT_LEN_MAX                   ((uint_fast8_t)(F_INTERRUPTS * RC5_BIT_TIME * MAX_TOLERANCE_05 + 0.5) + 1)
#else
#define RC5_START_BIT_LEN_MIN                   ((uint_fast8_t)(F_INTERRUPTS * RC5_BIT_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define RC5_START_BIT_LEN_MAX                   ((uint_fast8_t)(F_INTERRUPTS * RC5_BIT_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#endif

#define RC5_BIT_LEN_MIN                         ((uint_fast8_t)(F_INTERRUPTS * RC5_BIT_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define RC5_BIT_LEN_MAX                         ((uint_fast8_t)(F_INTERRUPTS * RC5_BIT_TIME * MAX_TOLERANCE_10 + 0.5) + 1)

#define DENON_PULSE_LEN_MIN                     ((uint_fast8_t)(F_INTERRUPTS * DENON_PULSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define DENON_PULSE_LEN_MAX                     ((uint_fast8_t)(F_INTERRUPTS * DENON_PULSE_TIME * MAX_TOLERANCE_20 + 0.5) + 1)
#define DENON_1_PAUSE_LEN_MIN                   ((uint_fast8_t)(F_INTERRUPTS * DENON_1_PAUSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define DENON_1_PAUSE_LEN_MAX                   ((uint_fast8_t)(F_INTERRUPTS * DENON_1_PAUSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
// RUWIDO (see t-home-mediareceiver-15kHz.txt) conflicts here with DENON
#define DENON_0_PAUSE_LEN_MIN                   ((uint_fast8_t)(F_INTERRUPTS * DENON_0_PAUSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define DENON_0_PAUSE_LEN_MAX                   ((uint_fast8_t)(F_INTERRUPTS * DENON_0_PAUSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define DENON_AUTO_REPETITION_PAUSE_LEN         ((uint_fast16_t)(F_INTERRUPTS * DENON_AUTO_REPETITION_PAUSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)

#define THOMSON_PULSE_LEN_MIN                   ((uint_fast8_t)(F_INTERRUPTS * THOMSON_PULSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define THOMSON_PULSE_LEN_MAX                   ((uint_fast8_t)(F_INTERRUPTS * THOMSON_PULSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define THOMSON_1_PAUSE_LEN_MIN                 ((uint_fast8_t)(F_INTERRUPTS * THOMSON_1_PAUSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define THOMSON_1_PAUSE_LEN_MAX                 ((uint_fast8_t)(F_INTERRUPTS * THOMSON_1_PAUSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define THOMSON_0_PAUSE_LEN_MIN                 ((uint_fast8_t)(F_INTERRUPTS * THOMSON_0_PAUSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define THOMSON_0_PAUSE_LEN_MAX                 ((uint_fast8_t)(F_INTERRUPTS * THOMSON_0_PAUSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)

#define RC6_START_BIT_PULSE_LEN_MIN             ((uint_fast8_t)(F_INTERRUPTS * RC6_START_BIT_PULSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define RC6_START_BIT_PULSE_LEN_MAX             ((uint_fast8_t)(F_INTERRUPTS * RC6_START_BIT_PULSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define RC6_START_BIT_PAUSE_LEN_MIN             ((uint_fast8_t)(F_INTERRUPTS * RC6_START_BIT_PAUSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define RC6_START_BIT_PAUSE_LEN_MAX             ((uint_fast8_t)(F_INTERRUPTS * RC6_START_BIT_PAUSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define RC6_TOGGLE_BIT_LEN_MIN                  ((uint_fast8_t)(F_INTERRUPTS * RC6_TOGGLE_BIT_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define RC6_TOGGLE_BIT_LEN_MAX                  ((uint_fast8_t)(F_INTERRUPTS * RC6_TOGGLE_BIT_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define RC6_BIT_PULSE_LEN_MIN                   ((uint_fast8_t)(F_INTERRUPTS * RC6_BIT_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define RC6_BIT_PULSE_LEN_MAX                   ((uint_fast8_t)(F_INTERRUPTS * RC6_BIT_TIME * MAX_TOLERANCE_60 + 0.5) + 1)       // pulses: 300 - 800
#define RC6_BIT_PAUSE_LEN_MIN                   ((uint_fast8_t)(F_INTERRUPTS * RC6_BIT_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define RC6_BIT_PAUSE_LEN_MAX                   ((uint_fast8_t)(F_INTERRUPTS * RC6_BIT_TIME * MAX_TOLERANCE_20 + 0.5) + 1)       // pauses: 300 - 600

#define RECS80EXT_START_BIT_PULSE_LEN_MIN       ((uint_fast8_t)(F_INTERRUPTS * RECS80EXT_START_BIT_PULSE_TIME * MIN_TOLERANCE_20 + 0.5) - 1)
#define RECS80EXT_START_BIT_PULSE_LEN_MAX       ((uint_fast8_t)(F_INTERRUPTS * RECS80EXT_START_BIT_PULSE_TIME * MAX_TOLERANCE_20 + 0.5) + 1)
#define RECS80EXT_START_BIT_PAUSE_LEN_MIN       ((uint_fast8_t)(F_INTERRUPTS * RECS80EXT_START_BIT_PAUSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define RECS80EXT_START_BIT_PAUSE_LEN_MAX       ((uint_fast8_t)(F_INTERRUPTS * RECS80EXT_START_BIT_PAUSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define RECS80EXT_PULSE_LEN_MIN                 ((uint_fast8_t)(F_INTERRUPTS * RECS80EXT_PULSE_TIME * MIN_TOLERANCE_20 + 0.5) - 1)
#define RECS80EXT_PULSE_LEN_MAX                 ((uint_fast8_t)(F_INTERRUPTS * RECS80EXT_PULSE_TIME * MAX_TOLERANCE_20 + 0.5) + 1)
#define RECS80EXT_1_PAUSE_LEN_MIN               ((uint_fast8_t)(F_INTERRUPTS * RECS80EXT_1_PAUSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define RECS80EXT_1_PAUSE_LEN_MAX               ((uint_fast8_t)(F_INTERRUPTS * RECS80EXT_1_PAUSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define RECS80EXT_0_PAUSE_LEN_MIN               ((uint_fast8_t)(F_INTERRUPTS * RECS80EXT_0_PAUSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define RECS80EXT_0_PAUSE_LEN_MAX               ((uint_fast8_t)(F_INTERRUPTS * RECS80EXT_0_PAUSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)

#define NUBERT_START_BIT_PULSE_LEN_MIN          ((uint_fast8_t)(F_INTERRUPTS * NUBERT_START_BIT_PULSE_TIME * MIN_TOLERANCE_20 + 0.5) - 1)
#define NUBERT_START_BIT_PULSE_LEN_MAX          ((uint_fast8_t)(F_INTERRUPTS * NUBERT_START_BIT_PULSE_TIME * MAX_TOLERANCE_20 + 0.5) + 1)
#define NUBERT_START_BIT_PAUSE_LEN_MIN          ((uint_fast8_t)(F_INTERRUPTS * NUBERT_START_BIT_PAUSE_TIME * MIN_TOLERANCE_20 + 0.5) - 1)
#define NUBERT_START_BIT_PAUSE_LEN_MAX          ((uint_fast8_t)(F_INTERRUPTS * NUBERT_START_BIT_PAUSE_TIME * MAX_TOLERANCE_20 + 0.5) + 1)
#define NUBERT_1_PULSE_LEN_MIN                  ((uint_fast8_t)(F_INTERRUPTS * NUBERT_1_PULSE_TIME * MIN_TOLERANCE_20 + 0.5) - 1)
#define NUBERT_1_PULSE_LEN_MAX                  ((uint_fast8_t)(F_INTERRUPTS * NUBERT_1_PULSE_TIME * MAX_TOLERANCE_20 + 0.5) + 1)
#define NUBERT_1_PAUSE_LEN_MIN                  ((uint_fast8_t)(F_INTERRUPTS * NUBERT_1_PAUSE_TIME * MIN_TOLERANCE_20 + 0.5) - 1)
#define NUBERT_1_PAUSE_LEN_MAX                  ((uint_fast8_t)(F_INTERRUPTS * NUBERT_1_PAUSE_TIME * MAX_TOLERANCE_20 + 0.5) + 1)
#define NUBERT_0_PULSE_LEN_MIN                  ((uint_fast8_t)(F_INTERRUPTS * NUBERT_0_PULSE_TIME * MIN_TOLERANCE_20 + 0.5) - 1)
#define NUBERT_0_PULSE_LEN_MAX                  ((uint_fast8_t)(F_INTERRUPTS * NUBERT_0_PULSE_TIME * MAX_TOLERANCE_20 + 0.5) + 1)
#define NUBERT_0_PAUSE_LEN_MIN                  ((uint_fast8_t)(F_INTERRUPTS * NUBERT_0_PAUSE_TIME * MIN_TOLERANCE_20 + 0.5) - 1)
#define NUBERT_0_PAUSE_LEN_MAX                  ((uint_fast8_t)(F_INTERRUPTS * NUBERT_0_PAUSE_TIME * MAX_TOLERANCE_20 + 0.5) + 1)

#define SPEAKER_START_BIT_PULSE_LEN_MIN          ((uint_fast8_t)(F_INTERRUPTS * SPEAKER_START_BIT_PULSE_TIME * MIN_TOLERANCE_20 + 0.5) - 1)
#define SPEAKER_START_BIT_PULSE_LEN_MAX          ((uint_fast8_t)(F_INTERRUPTS * SPEAKER_START_BIT_PULSE_TIME * MAX_TOLERANCE_20 + 0.5) + 1)
#define SPEAKER_START_BIT_PAUSE_LEN_MIN          ((uint_fast8_t)(F_INTERRUPTS * SPEAKER_START_BIT_PAUSE_TIME * MIN_TOLERANCE_20 + 0.5) - 1)
#define SPEAKER_START_BIT_PAUSE_LEN_MAX          ((uint_fast8_t)(F_INTERRUPTS * SPEAKER_START_BIT_PAUSE_TIME * MAX_TOLERANCE_20 + 0.5) + 1)
#define SPEAKER_1_PULSE_LEN_MIN                  ((uint_fast8_t)(F_INTERRUPTS * SPEAKER_1_PULSE_TIME * MIN_TOLERANCE_20 + 0.5) - 1)
#define SPEAKER_1_PULSE_LEN_MAX                  ((uint_fast8_t)(F_INTERRUPTS * SPEAKER_1_PULSE_TIME * MAX_TOLERANCE_20 + 0.5) + 1)
#define SPEAKER_1_PAUSE_LEN_MIN                  ((uint_fast8_t)(F_INTERRUPTS * SPEAKER_1_PAUSE_TIME * MIN_TOLERANCE_20 + 0.5) - 1)
#define SPEAKER_1_PAUSE_LEN_MAX                  ((uint_fast8_t)(F_INTERRUPTS * SPEAKER_1_PAUSE_TIME * MAX_TOLERANCE_20 + 0.5) + 1)
#define SPEAKER_0_PULSE_LEN_MIN                  ((uint_fast8_t)(F_INTERRUPTS * SPEAKER_0_PULSE_TIME * MIN_TOLERANCE_20 + 0.5) - 1)
#define SPEAKER_0_PULSE_LEN_MAX                  ((uint_fast8_t)(F_INTERRUPTS * SPEAKER_0_PULSE_TIME * MAX_TOLERANCE_20 + 0.5) + 1)
#define SPEAKER_0_PAUSE_LEN_MIN                  ((uint_fast8_t)(F_INTERRUPTS * SPEAKER_0_PAUSE_TIME * MIN_TOLERANCE_20 + 0.5) - 1)
#define SPEAKER_0_PAUSE_LEN_MAX                  ((uint_fast8_t)(F_INTERRUPTS * SPEAKER_0_PAUSE_TIME * MAX_TOLERANCE_20 + 0.5) + 1)

#define BANG_OLUFSEN_START_BIT1_PULSE_LEN_MIN   ((uint_fast8_t)(F_INTERRUPTS * BANG_OLUFSEN_START_BIT1_PULSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define BANG_OLUFSEN_START_BIT1_PULSE_LEN_MAX   ((uint_fast8_t)(F_INTERRUPTS * BANG_OLUFSEN_START_BIT1_PULSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define BANG_OLUFSEN_START_BIT1_PAUSE_LEN_MIN   ((uint_fast8_t)(F_INTERRUPTS * BANG_OLUFSEN_START_BIT1_PAUSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define BANG_OLUFSEN_START_BIT1_PAUSE_LEN_MAX   ((uint_fast8_t)(F_INTERRUPTS * BANG_OLUFSEN_START_BIT1_PAUSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define BANG_OLUFSEN_START_BIT2_PULSE_LEN_MIN   ((uint_fast8_t)(F_INTERRUPTS * BANG_OLUFSEN_START_BIT2_PULSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define BANG_OLUFSEN_START_BIT2_PULSE_LEN_MAX   ((uint_fast8_t)(F_INTERRUPTS * BANG_OLUFSEN_START_BIT2_PULSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define BANG_OLUFSEN_START_BIT2_PAUSE_LEN_MIN   ((uint_fast8_t)(F_INTERRUPTS * BANG_OLUFSEN_START_BIT2_PAUSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define BANG_OLUFSEN_START_BIT2_PAUSE_LEN_MAX   ((uint_fast8_t)(F_INTERRUPTS * BANG_OLUFSEN_START_BIT2_PAUSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define BANG_OLUFSEN_START_BIT3_PULSE_LEN_MIN   ((uint_fast8_t)(F_INTERRUPTS * BANG_OLUFSEN_START_BIT3_PULSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define BANG_OLUFSEN_START_BIT3_PULSE_LEN_MAX   ((uint_fast8_t)(F_INTERRUPTS * BANG_OLUFSEN_START_BIT3_PULSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define BANG_OLUFSEN_START_BIT3_PAUSE_LEN_MIN   ((uint_fast8_t)(F_INTERRUPTS * BANG_OLUFSEN_START_BIT3_PAUSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define BANG_OLUFSEN_START_BIT3_PAUSE_LEN_MAX   ((PAUSE_LEN)(F_INTERRUPTS * BANG_OLUFSEN_START_BIT3_PAUSE_TIME * MAX_TOLERANCE_05 + 0.5) + 1) // value must be below IRMP_TIMEOUT
#define BANG_OLUFSEN_START_BIT4_PULSE_LEN_MIN   ((uint_fast8_t)(F_INTERRUPTS * BANG_OLUFSEN_START_BIT4_PULSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define BANG_OLUFSEN_START_BIT4_PULSE_LEN_MAX   ((uint_fast8_t)(F_INTERRUPTS * BANG_OLUFSEN_START_BIT4_PULSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define BANG_OLUFSEN_START_BIT4_PAUSE_LEN_MIN   ((uint_fast8_t)(F_INTERRUPTS * BANG_OLUFSEN_START_BIT4_PAUSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define BANG_OLUFSEN_START_BIT4_PAUSE_LEN_MAX   ((uint_fast8_t)(F_INTERRUPTS * BANG_OLUFSEN_START_BIT4_PAUSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define BANG_OLUFSEN_PULSE_LEN_MIN              ((uint_fast8_t)(F_INTERRUPTS * BANG_OLUFSEN_PULSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define BANG_OLUFSEN_PULSE_LEN_MAX              ((uint_fast8_t)(F_INTERRUPTS * BANG_OLUFSEN_PULSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define BANG_OLUFSEN_1_PAUSE_LEN_MIN            ((uint_fast8_t)(F_INTERRUPTS * BANG_OLUFSEN_1_PAUSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define BANG_OLUFSEN_1_PAUSE_LEN_MAX            ((uint_fast8_t)(F_INTERRUPTS * BANG_OLUFSEN_1_PAUSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define BANG_OLUFSEN_0_PAUSE_LEN_MIN            ((uint_fast8_t)(F_INTERRUPTS * BANG_OLUFSEN_0_PAUSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define BANG_OLUFSEN_0_PAUSE_LEN_MAX            ((uint_fast8_t)(F_INTERRUPTS * BANG_OLUFSEN_0_PAUSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define BANG_OLUFSEN_R_PAUSE_LEN_MIN            ((uint_fast8_t)(F_INTERRUPTS * BANG_OLUFSEN_R_PAUSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define BANG_OLUFSEN_R_PAUSE_LEN_MAX            ((uint_fast8_t)(F_INTERRUPTS * BANG_OLUFSEN_R_PAUSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define BANG_OLUFSEN_TRAILER_BIT_PAUSE_LEN_MIN  ((uint_fast8_t)(F_INTERRUPTS * BANG_OLUFSEN_TRAILER_BIT_PAUSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define BANG_OLUFSEN_TRAILER_BIT_PAUSE_LEN_MAX  ((uint_fast8_t)(F_INTERRUPTS * BANG_OLUFSEN_TRAILER_BIT_PAUSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)

#define IR60_TIMEOUT_LEN                        ((uint_fast8_t)(F_INTERRUPTS * IR60_TIMEOUT_TIME * 0.5))
#define GRUNDIG_NOKIA_IR60_START_BIT_LEN_MIN    ((uint_fast8_t)(F_INTERRUPTS * GRUNDIG_NOKIA_IR60_BIT_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define GRUNDIG_NOKIA_IR60_START_BIT_LEN_MAX    ((uint_fast8_t)(F_INTERRUPTS * GRUNDIG_NOKIA_IR60_BIT_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define GRUNDIG_NOKIA_IR60_BIT_LEN_MIN          ((uint_fast8_t)(F_INTERRUPTS * GRUNDIG_NOKIA_IR60_BIT_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define GRUNDIG_NOKIA_IR60_BIT_LEN_MAX          ((uint_fast8_t)(F_INTERRUPTS * GRUNDIG_NOKIA_IR60_BIT_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define GRUNDIG_NOKIA_IR60_PRE_PAUSE_LEN_MIN    ((uint_fast8_t)(F_INTERRUPTS * GRUNDIG_NOKIA_IR60_PRE_PAUSE_TIME * MIN_TOLERANCE_20 + 0.5) + 1)
#define GRUNDIG_NOKIA_IR60_PRE_PAUSE_LEN_MAX    ((uint_fast8_t)(F_INTERRUPTS * GRUNDIG_NOKIA_IR60_PRE_PAUSE_TIME * MAX_TOLERANCE_20 + 0.5) + 1)

#define SIEMENS_OR_RUWIDO_START_BIT_PULSE_LEN_MIN       ((uint_fast8_t)(F_INTERRUPTS * SIEMENS_OR_RUWIDO_START_BIT_PULSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define SIEMENS_OR_RUWIDO_START_BIT_PULSE_LEN_MAX       ((uint_fast8_t)(F_INTERRUPTS * SIEMENS_OR_RUWIDO_START_BIT_PULSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define SIEMENS_OR_RUWIDO_START_BIT_PAUSE_LEN_MIN       ((uint_fast8_t)(F_INTERRUPTS * SIEMENS_OR_RUWIDO_START_BIT_PAUSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define SIEMENS_OR_RUWIDO_START_BIT_PAUSE_LEN_MAX       ((uint_fast8_t)(F_INTERRUPTS * SIEMENS_OR_RUWIDO_START_BIT_PAUSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define SIEMENS_OR_RUWIDO_BIT_PULSE_LEN_MIN             ((uint_fast8_t)(F_INTERRUPTS * SIEMENS_OR_RUWIDO_BIT_PULSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define SIEMENS_OR_RUWIDO_BIT_PULSE_LEN_MAX             ((uint_fast8_t)(F_INTERRUPTS * SIEMENS_OR_RUWIDO_BIT_PULSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define SIEMENS_OR_RUWIDO_BIT_PAUSE_LEN_MIN             ((uint_fast8_t)(F_INTERRUPTS * SIEMENS_OR_RUWIDO_BIT_PAUSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define SIEMENS_OR_RUWIDO_BIT_PAUSE_LEN_MAX             ((uint_fast8_t)(F_INTERRUPTS * SIEMENS_OR_RUWIDO_BIT_PAUSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)

#define FDC_START_BIT_PULSE_LEN_MIN             ((uint_fast8_t)(F_INTERRUPTS * FDC_START_BIT_PULSE_TIME * MIN_TOLERANCE_05 + 0.5) - 1)   // 5%: avoid conflict with NETBOX
#define FDC_START_BIT_PULSE_LEN_MAX             ((uint_fast8_t)(F_INTERRUPTS * FDC_START_BIT_PULSE_TIME * MAX_TOLERANCE_05 + 0.5))
#define FDC_START_BIT_PAUSE_LEN_MIN             ((uint_fast8_t)(F_INTERRUPTS * FDC_START_BIT_PAUSE_TIME * MIN_TOLERANCE_05 + 0.5) - 1)
#define FDC_START_BIT_PAUSE_LEN_MAX             ((uint_fast8_t)(F_INTERRUPTS * FDC_START_BIT_PAUSE_TIME * MAX_TOLERANCE_05 + 0.5))
#define FDC_PULSE_LEN_MIN                       ((uint_fast8_t)(F_INTERRUPTS * FDC_PULSE_TIME * MIN_TOLERANCE_40 + 0.5) - 1)
#define FDC_PULSE_LEN_MAX                       ((uint_fast8_t)(F_INTERRUPTS * FDC_PULSE_TIME * MAX_TOLERANCE_50 + 0.5) + 1)
#define FDC_1_PAUSE_LEN_MIN                     ((uint_fast8_t)(F_INTERRUPTS * FDC_1_PAUSE_TIME * MIN_TOLERANCE_20 + 0.5) - 1)
#define FDC_1_PAUSE_LEN_MAX                     ((uint_fast8_t)(F_INTERRUPTS * FDC_1_PAUSE_TIME * MAX_TOLERANCE_20 + 0.5) + 1)
#if 0
#define FDC_0_PAUSE_LEN_MIN                     ((uint_fast8_t)(F_INTERRUPTS * FDC_0_PAUSE_TIME * MIN_TOLERANCE_40 + 0.5) - 1)   // could be negative: 255
#else
#define FDC_0_PAUSE_LEN_MIN                     (1)                                                                         // simply use 1
#endif
#define FDC_0_PAUSE_LEN_MAX                     ((uint_fast8_t)(F_INTERRUPTS * FDC_0_PAUSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)

#define RCCAR_START_BIT_PULSE_LEN_MIN           ((uint_fast8_t)(F_INTERRUPTS * RCCAR_START_BIT_PULSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define RCCAR_START_BIT_PULSE_LEN_MAX           ((uint_fast8_t)(F_INTERRUPTS * RCCAR_START_BIT_PULSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define RCCAR_START_BIT_PAUSE_LEN_MIN           ((uint_fast8_t)(F_INTERRUPTS * RCCAR_START_BIT_PAUSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define RCCAR_START_BIT_PAUSE_LEN_MAX           ((uint_fast8_t)(F_INTERRUPTS * RCCAR_START_BIT_PAUSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define RCCAR_PULSE_LEN_MIN                     ((uint_fast8_t)(F_INTERRUPTS * RCCAR_PULSE_TIME * MIN_TOLERANCE_20 + 0.5) - 1)
#define RCCAR_PULSE_LEN_MAX                     ((uint_fast8_t)(F_INTERRUPTS * RCCAR_PULSE_TIME * MAX_TOLERANCE_20 + 0.5) + 1)
#define RCCAR_1_PAUSE_LEN_MIN                   ((uint_fast8_t)(F_INTERRUPTS * RCCAR_1_PAUSE_TIME * MIN_TOLERANCE_30 + 0.5) - 1)
#define RCCAR_1_PAUSE_LEN_MAX                   ((uint_fast8_t)(F_INTERRUPTS * RCCAR_1_PAUSE_TIME * MAX_TOLERANCE_30 + 0.5) + 1)
#define RCCAR_0_PAUSE_LEN_MIN                   ((uint_fast8_t)(F_INTERRUPTS * RCCAR_0_PAUSE_TIME * MIN_TOLERANCE_30 + 0.5) - 1)
#define RCCAR_0_PAUSE_LEN_MAX                   ((uint_fast8_t)(F_INTERRUPTS * RCCAR_0_PAUSE_TIME * MAX_TOLERANCE_30 + 0.5) + 1)

#define JVC_START_BIT_PULSE_LEN_MIN             ((uint_fast8_t)(F_INTERRUPTS * JVC_START_BIT_PULSE_TIME * MIN_TOLERANCE_40 + 0.5) - 1)
#define JVC_START_BIT_PULSE_LEN_MAX             ((uint_fast8_t)(F_INTERRUPTS * JVC_START_BIT_PULSE_TIME * MAX_TOLERANCE_40 + 0.5) + 1)
#define JVC_REPEAT_START_BIT_PAUSE_LEN_MIN      ((uint_fast8_t)(F_INTERRUPTS * (JVC_FRAME_REPEAT_PAUSE_TIME - IRMP_TIMEOUT_TIME) * MIN_TOLERANCE_40 + 0.5) - 1)  // HACK!
#define JVC_REPEAT_START_BIT_PAUSE_LEN_MAX      ((uint_fast8_t)(F_INTERRUPTS * (JVC_FRAME_REPEAT_PAUSE_TIME - IRMP_TIMEOUT_TIME) * MAX_TOLERANCE_70 + 0.5) - 1)  // HACK!
#define JVC_PULSE_LEN_MIN                       ((uint_fast8_t)(F_INTERRUPTS * JVC_PULSE_TIME * MIN_TOLERANCE_40 + 0.5) - 1)
#define JVC_PULSE_LEN_MAX                       ((uint_fast8_t)(F_INTERRUPTS * JVC_PULSE_TIME * MAX_TOLERANCE_40 + 0.5) + 1)
#define JVC_1_PAUSE_LEN_MIN                     ((uint_fast8_t)(F_INTERRUPTS * JVC_1_PAUSE_TIME * MIN_TOLERANCE_40 + 0.5) - 1)
#define JVC_1_PAUSE_LEN_MAX                     ((uint_fast8_t)(F_INTERRUPTS * JVC_1_PAUSE_TIME * MAX_TOLERANCE_40 + 0.5) + 1)
#define JVC_0_PAUSE_LEN_MIN                     ((uint_fast8_t)(F_INTERRUPTS * JVC_0_PAUSE_TIME * MIN_TOLERANCE_40 + 0.5) - 1)
#define JVC_0_PAUSE_LEN_MAX                     ((uint_fast8_t)(F_INTERRUPTS * JVC_0_PAUSE_TIME * MAX_TOLERANCE_40 + 0.5) + 1)
// autodetect JVC repetition frame within 50 msec:
#define JVC_FRAME_REPEAT_PAUSE_LEN_MAX          (uint_fast16_t)(F_INTERRUPTS * JVC_FRAME_REPEAT_PAUSE_TIME * MAX_TOLERANCE_20 + 0.5)

#define NIKON_START_BIT_PULSE_LEN_MIN           ((uint_fast8_t)(F_INTERRUPTS * NIKON_START_BIT_PULSE_TIME * MIN_TOLERANCE_20 + 0.5) - 1)
#define NIKON_START_BIT_PULSE_LEN_MAX           ((uint_fast8_t)(F_INTERRUPTS * NIKON_START_BIT_PULSE_TIME * MAX_TOLERANCE_20 + 0.5) + 1)
#define NIKON_START_BIT_PAUSE_LEN_MIN           ((uint_fast16_t)(F_INTERRUPTS * NIKON_START_BIT_PAUSE_TIME * MIN_TOLERANCE_20 + 0.5) - 1)
#define NIKON_START_BIT_PAUSE_LEN_MAX           ((uint_fast16_t)(F_INTERRUPTS * NIKON_START_BIT_PAUSE_TIME * MAX_TOLERANCE_20 + 0.5) + 1)
#define NIKON_REPEAT_START_BIT_PAUSE_LEN_MIN    ((uint_fast8_t)(F_INTERRUPTS * NIKON_REPEAT_START_BIT_PAUSE_TIME * MIN_TOLERANCE_20 + 0.5) - 1)
#define NIKON_REPEAT_START_BIT_PAUSE_LEN_MAX    ((uint_fast8_t)(F_INTERRUPTS * NIKON_REPEAT_START_BIT_PAUSE_TIME * MAX_TOLERANCE_20 + 0.5) + 1)
#define NIKON_PULSE_LEN_MIN                     ((uint_fast8_t)(F_INTERRUPTS * NIKON_PULSE_TIME * MIN_TOLERANCE_20 + 0.5) - 1)
#define NIKON_PULSE_LEN_MAX                     ((uint_fast8_t)(F_INTERRUPTS * NIKON_PULSE_TIME * MAX_TOLERANCE_20 + 0.5) + 1)
#define NIKON_1_PAUSE_LEN_MIN                   ((uint_fast8_t)(F_INTERRUPTS * NIKON_1_PAUSE_TIME * MIN_TOLERANCE_20 + 0.5) - 1)
#define NIKON_1_PAUSE_LEN_MAX                   ((uint_fast8_t)(F_INTERRUPTS * NIKON_1_PAUSE_TIME * MAX_TOLERANCE_20 + 0.5) + 1)
#define NIKON_0_PAUSE_LEN_MIN                   ((uint_fast8_t)(F_INTERRUPTS * NIKON_0_PAUSE_TIME * MIN_TOLERANCE_20 + 0.5) - 1)
#define NIKON_0_PAUSE_LEN_MAX                   ((uint_fast8_t)(F_INTERRUPTS * NIKON_0_PAUSE_TIME * MAX_TOLERANCE_20 + 0.5) + 1)
#define NIKON_FRAME_REPEAT_PAUSE_LEN_MAX        (uint_fast16_t)(F_INTERRUPTS * NIKON_FRAME_REPEAT_PAUSE_TIME * MAX_TOLERANCE_20 + 0.5)

#define KATHREIN_START_BIT_PULSE_LEN_MIN        ((uint_fast8_t)(F_INTERRUPTS * KATHREIN_START_BIT_PULSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define KATHREIN_START_BIT_PULSE_LEN_MAX        ((uint_fast8_t)(F_INTERRUPTS * KATHREIN_START_BIT_PULSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define KATHREIN_START_BIT_PAUSE_LEN_MIN        ((uint_fast8_t)(F_INTERRUPTS * KATHREIN_START_BIT_PAUSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define KATHREIN_START_BIT_PAUSE_LEN_MAX        ((uint_fast8_t)(F_INTERRUPTS * KATHREIN_START_BIT_PAUSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define KATHREIN_1_PULSE_LEN_MIN                ((uint_fast8_t)(F_INTERRUPTS * KATHREIN_1_PULSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define KATHREIN_1_PULSE_LEN_MAX                ((uint_fast8_t)(F_INTERRUPTS * KATHREIN_1_PULSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define KATHREIN_1_PAUSE_LEN_MIN                ((uint_fast8_t)(F_INTERRUPTS * KATHREIN_1_PAUSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define KATHREIN_1_PAUSE_LEN_MAX                ((uint_fast8_t)(F_INTERRUPTS * KATHREIN_1_PAUSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define KATHREIN_0_PULSE_LEN_MIN                ((uint_fast8_t)(F_INTERRUPTS * KATHREIN_0_PULSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define KATHREIN_0_PULSE_LEN_MAX                ((uint_fast8_t)(F_INTERRUPTS * KATHREIN_0_PULSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define KATHREIN_0_PAUSE_LEN_MIN                ((uint_fast8_t)(F_INTERRUPTS * KATHREIN_0_PAUSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define KATHREIN_0_PAUSE_LEN_MAX                ((uint_fast8_t)(F_INTERRUPTS * KATHREIN_0_PAUSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define KATHREIN_SYNC_BIT_PAUSE_LEN_MIN         ((uint_fast8_t)(F_INTERRUPTS * KATHREIN_SYNC_BIT_PAUSE_LEN_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define KATHREIN_SYNC_BIT_PAUSE_LEN_MAX         ((uint_fast8_t)(F_INTERRUPTS * KATHREIN_SYNC_BIT_PAUSE_LEN_TIME * MAX_TOLERANCE_10 + 0.5) + 1)

#define NETBOX_START_BIT_PULSE_LEN_MIN          ((uint_fast8_t)(F_INTERRUPTS * NETBOX_START_BIT_PULSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define NETBOX_START_BIT_PULSE_LEN_MAX          ((uint_fast8_t)(F_INTERRUPTS * NETBOX_START_BIT_PULSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define NETBOX_START_BIT_PAUSE_LEN_MIN          ((uint_fast8_t)(F_INTERRUPTS * NETBOX_START_BIT_PAUSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define NETBOX_START_BIT_PAUSE_LEN_MAX          ((uint_fast8_t)(F_INTERRUPTS * NETBOX_START_BIT_PAUSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define NETBOX_PULSE_LEN                        ((uint_fast8_t)(F_INTERRUPTS * NETBOX_PULSE_TIME))
#define NETBOX_PAUSE_LEN                        ((uint_fast8_t)(F_INTERRUPTS * NETBOX_PAUSE_TIME))
#define NETBOX_PULSE_REST_LEN                   ((uint_fast8_t)(F_INTERRUPTS * NETBOX_PULSE_TIME / 4))
#define NETBOX_PAUSE_REST_LEN                   ((uint_fast8_t)(F_INTERRUPTS * NETBOX_PAUSE_TIME / 4))

#define LEGO_START_BIT_PULSE_LEN_MIN            ((uint_fast8_t)(F_INTERRUPTS * LEGO_START_BIT_PULSE_TIME * MIN_TOLERANCE_40 + 0.5) - 1)
#define LEGO_START_BIT_PULSE_LEN_MAX            ((uint_fast8_t)(F_INTERRUPTS * LEGO_START_BIT_PULSE_TIME * MAX_TOLERANCE_40 + 0.5) + 1)
#define LEGO_START_BIT_PAUSE_LEN_MIN            ((uint_fast8_t)(F_INTERRUPTS * LEGO_START_BIT_PAUSE_TIME * MIN_TOLERANCE_40 + 0.5) - 1)
#define LEGO_START_BIT_PAUSE_LEN_MAX            ((uint_fast8_t)(F_INTERRUPTS * LEGO_START_BIT_PAUSE_TIME * MAX_TOLERANCE_40 + 0.5) + 1)
#define LEGO_PULSE_LEN_MIN                      ((uint_fast8_t)(F_INTERRUPTS * LEGO_PULSE_TIME * MIN_TOLERANCE_40 + 0.5) - 1)
#define LEGO_PULSE_LEN_MAX                      ((uint_fast8_t)(F_INTERRUPTS * LEGO_PULSE_TIME * MAX_TOLERANCE_40 + 0.5) + 1)
#define LEGO_1_PAUSE_LEN_MIN                    ((uint_fast8_t)(F_INTERRUPTS * LEGO_1_PAUSE_TIME * MIN_TOLERANCE_40 + 0.5) - 1)
#define LEGO_1_PAUSE_LEN_MAX                    ((uint_fast8_t)(F_INTERRUPTS * LEGO_1_PAUSE_TIME * MAX_TOLERANCE_40 + 0.5) + 1)
#define LEGO_0_PAUSE_LEN_MIN                    ((uint_fast8_t)(F_INTERRUPTS * LEGO_0_PAUSE_TIME * MIN_TOLERANCE_40 + 0.5) - 1)
#define LEGO_0_PAUSE_LEN_MAX                    ((uint_fast8_t)(F_INTERRUPTS * LEGO_0_PAUSE_TIME * MAX_TOLERANCE_40 + 0.5) + 1)

#define BOSE_START_BIT_PULSE_LEN_MIN             ((uint_fast8_t)(F_INTERRUPTS * BOSE_START_BIT_PULSE_TIME * MIN_TOLERANCE_30 + 0.5) - 1)
#define BOSE_START_BIT_PULSE_LEN_MAX             ((uint_fast8_t)(F_INTERRUPTS * BOSE_START_BIT_PULSE_TIME * MAX_TOLERANCE_30 + 0.5) + 1)
#define BOSE_START_BIT_PAUSE_LEN_MIN             ((uint_fast8_t)(F_INTERRUPTS * BOSE_START_BIT_PAUSE_TIME * MIN_TOLERANCE_30 + 0.5) - 1)
#define BOSE_START_BIT_PAUSE_LEN_MAX             ((uint_fast8_t)(F_INTERRUPTS * BOSE_START_BIT_PAUSE_TIME * MAX_TOLERANCE_30 + 0.5) + 1)
#define BOSE_PULSE_LEN_MIN                       ((uint_fast8_t)(F_INTERRUPTS * BOSE_PULSE_TIME * MIN_TOLERANCE_30 + 0.5) - 1)
#define BOSE_PULSE_LEN_MAX                       ((uint_fast8_t)(F_INTERRUPTS * BOSE_PULSE_TIME * MAX_TOLERANCE_30 + 0.5) + 1)
#define BOSE_1_PAUSE_LEN_MIN                     ((uint_fast8_t)(F_INTERRUPTS * BOSE_1_PAUSE_TIME * MIN_TOLERANCE_30 + 0.5) - 1)
#define BOSE_1_PAUSE_LEN_MAX                     ((uint_fast8_t)(F_INTERRUPTS * BOSE_1_PAUSE_TIME * MAX_TOLERANCE_30 + 0.5) + 1)
#define BOSE_0_PAUSE_LEN_MIN                     ((uint_fast8_t)(F_INTERRUPTS * BOSE_0_PAUSE_TIME * MIN_TOLERANCE_30 + 0.5) - 1)
#define BOSE_0_PAUSE_LEN_MAX                     ((uint_fast8_t)(F_INTERRUPTS * BOSE_0_PAUSE_TIME * MAX_TOLERANCE_30 + 0.5) + 1)
#define BOSE_FRAME_REPEAT_PAUSE_LEN_MAX          (uint_fast16_t)(F_INTERRUPTS * 100.0e-3 * MAX_TOLERANCE_20 + 0.5)

#define A1TVBOX_START_BIT_PULSE_LEN_MIN         ((uint_fast8_t)(F_INTERRUPTS * A1TVBOX_START_BIT_PULSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define A1TVBOX_START_BIT_PULSE_LEN_MAX         ((uint_fast8_t)(F_INTERRUPTS * A1TVBOX_START_BIT_PULSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define A1TVBOX_START_BIT_PAUSE_LEN_MIN         ((uint_fast8_t)(F_INTERRUPTS * A1TVBOX_START_BIT_PAUSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define A1TVBOX_START_BIT_PAUSE_LEN_MAX         ((uint_fast8_t)(F_INTERRUPTS * A1TVBOX_START_BIT_PAUSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define A1TVBOX_BIT_PULSE_LEN_MIN               ((uint_fast8_t)(F_INTERRUPTS * A1TVBOX_BIT_PULSE_TIME * MIN_TOLERANCE_30 + 0.5) - 1)
#define A1TVBOX_BIT_PULSE_LEN_MAX               ((uint_fast8_t)(F_INTERRUPTS * A1TVBOX_BIT_PULSE_TIME * MAX_TOLERANCE_30 + 0.5) + 1)
#define A1TVBOX_BIT_PAUSE_LEN_MIN               ((uint_fast8_t)(F_INTERRUPTS * A1TVBOX_BIT_PAUSE_TIME * MIN_TOLERANCE_30 + 0.5) - 1)
#define A1TVBOX_BIT_PAUSE_LEN_MAX               ((uint_fast8_t)(F_INTERRUPTS * A1TVBOX_BIT_PAUSE_TIME * MAX_TOLERANCE_30 + 0.5) + 1)

#define ORTEK_START_BIT_PULSE_LEN_MIN           ((uint_fast8_t)(F_INTERRUPTS * ORTEK_START_BIT_PULSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define ORTEK_START_BIT_PULSE_LEN_MAX           ((uint_fast8_t)(F_INTERRUPTS * ORTEK_START_BIT_PULSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define ORTEK_START_BIT_PAUSE_LEN_MIN           ((uint_fast8_t)(F_INTERRUPTS * ORTEK_START_BIT_PAUSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define ORTEK_START_BIT_PAUSE_LEN_MAX           ((uint_fast8_t)(F_INTERRUPTS * ORTEK_START_BIT_PAUSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define ORTEK_BIT_PULSE_LEN_MIN                 ((uint_fast8_t)(F_INTERRUPTS * ORTEK_BIT_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define ORTEK_BIT_PULSE_LEN_MAX                 ((uint_fast8_t)(F_INTERRUPTS * ORTEK_BIT_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define ORTEK_BIT_PAUSE_LEN_MIN                 ((uint_fast8_t)(F_INTERRUPTS * ORTEK_BIT_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define ORTEK_BIT_PAUSE_LEN_MAX                 ((uint_fast8_t)(F_INTERRUPTS * ORTEK_BIT_TIME * MAX_TOLERANCE_10 + 0.5) + 1)

#define TELEFUNKEN_START_BIT_PULSE_LEN_MIN      ((uint_fast8_t)(F_INTERRUPTS * TELEFUNKEN_START_BIT_PULSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define TELEFUNKEN_START_BIT_PULSE_LEN_MAX      ((uint_fast8_t)(F_INTERRUPTS * TELEFUNKEN_START_BIT_PULSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define TELEFUNKEN_START_BIT_PAUSE_LEN_MIN      ((uint_fast8_t)(F_INTERRUPTS * (TELEFUNKEN_START_BIT_PAUSE_TIME) * MIN_TOLERANCE_10 + 0.5) - 1)
#define TELEFUNKEN_START_BIT_PAUSE_LEN_MAX      ((uint_fast8_t)(F_INTERRUPTS * (TELEFUNKEN_START_BIT_PAUSE_TIME) * MAX_TOLERANCE_10 + 0.5) - 1)
#define TELEFUNKEN_PULSE_LEN_MIN                ((uint_fast8_t)(F_INTERRUPTS * TELEFUNKEN_PULSE_TIME * MIN_TOLERANCE_30 + 0.5) - 1)
#define TELEFUNKEN_PULSE_LEN_MAX                ((uint_fast8_t)(F_INTERRUPTS * TELEFUNKEN_PULSE_TIME * MAX_TOLERANCE_30 + 0.5) + 1)
#define TELEFUNKEN_1_PAUSE_LEN_MIN              ((uint_fast8_t)(F_INTERRUPTS * TELEFUNKEN_1_PAUSE_TIME * MIN_TOLERANCE_30 + 0.5) - 1)
#define TELEFUNKEN_1_PAUSE_LEN_MAX              ((uint_fast8_t)(F_INTERRUPTS * TELEFUNKEN_1_PAUSE_TIME * MAX_TOLERANCE_30 + 0.5) + 1)
#define TELEFUNKEN_0_PAUSE_LEN_MIN              ((uint_fast8_t)(F_INTERRUPTS * TELEFUNKEN_0_PAUSE_TIME * MIN_TOLERANCE_30 + 0.5) - 1)
#define TELEFUNKEN_0_PAUSE_LEN_MAX              ((uint_fast8_t)(F_INTERRUPTS * TELEFUNKEN_0_PAUSE_TIME * MAX_TOLERANCE_30 + 0.5) + 1)
// autodetect TELEFUNKEN repetition frame within 50 msec:
// #define TELEFUNKEN_FRAME_REPEAT_PAUSE_LEN_MAX   (uint_fast16_t)(F_INTERRUPTS * TELEFUNKEN_FRAME_REPEAT_PAUSE_TIME * MAX_TOLERANCE_20 + 0.5)

#define ROOMBA_START_BIT_PULSE_LEN_MIN          ((uint_fast8_t)(F_INTERRUPTS * ROOMBA_START_BIT_PULSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define ROOMBA_START_BIT_PULSE_LEN_MAX          ((uint_fast8_t)(F_INTERRUPTS * ROOMBA_START_BIT_PULSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define ROOMBA_START_BIT_PAUSE_LEN_MIN          ((uint_fast8_t)(F_INTERRUPTS * ROOMBA_START_BIT_PAUSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define ROOMBA_START_BIT_PAUSE_LEN_MAX          ((uint_fast8_t)(F_INTERRUPTS * ROOMBA_START_BIT_PAUSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define ROOMBA_1_PAUSE_LEN_EXACT                ((uint_fast8_t)(F_INTERRUPTS * ROOMBA_1_PAUSE_TIME + 0.5))
#define ROOMBA_1_PULSE_LEN_MIN                  ((uint_fast8_t)(F_INTERRUPTS * ROOMBA_1_PULSE_TIME * MIN_TOLERANCE_20 + 0.5) - 1)
#define ROOMBA_1_PULSE_LEN_MAX                  ((uint_fast8_t)(F_INTERRUPTS * ROOMBA_1_PULSE_TIME * MAX_TOLERANCE_20 + 0.5) + 1)
#define ROOMBA_1_PAUSE_LEN_MIN                  ((uint_fast8_t)(F_INTERRUPTS * ROOMBA_1_PAUSE_TIME * MIN_TOLERANCE_20 + 0.5) - 1)
#define ROOMBA_1_PAUSE_LEN_MAX                  ((uint_fast8_t)(F_INTERRUPTS * ROOMBA_1_PAUSE_TIME * MAX_TOLERANCE_20 + 0.5) + 1)
#define ROOMBA_0_PAUSE_LEN                      ((uint_fast8_t)(F_INTERRUPTS * ROOMBA_0_PAUSE_TIME))
#define ROOMBA_0_PULSE_LEN_MIN                  ((uint_fast8_t)(F_INTERRUPTS * ROOMBA_0_PULSE_TIME * MIN_TOLERANCE_20 + 0.5) - 1)
#define ROOMBA_0_PULSE_LEN_MAX                  ((uint_fast8_t)(F_INTERRUPTS * ROOMBA_0_PULSE_TIME * MAX_TOLERANCE_20 + 0.5) + 1)
#define ROOMBA_0_PAUSE_LEN_MIN                  ((uint_fast8_t)(F_INTERRUPTS * ROOMBA_0_PAUSE_TIME * MIN_TOLERANCE_20 + 0.5) - 1)
#define ROOMBA_0_PAUSE_LEN_MAX                  ((uint_fast8_t)(F_INTERRUPTS * ROOMBA_0_PAUSE_TIME * MAX_TOLERANCE_20 + 0.5) + 1)

#define RCMM32_START_BIT_PULSE_LEN_MIN          ((uint_fast8_t)(F_INTERRUPTS * RCMM32_START_BIT_PULSE_TIME * MIN_TOLERANCE_05 + 0.5) - 1)
#define RCMM32_START_BIT_PULSE_LEN_MAX          ((uint_fast8_t)(F_INTERRUPTS * RCMM32_START_BIT_PULSE_TIME * MAX_TOLERANCE_05 + 0.5) + 1)
#define RCMM32_START_BIT_PAUSE_LEN_MIN          ((uint_fast8_t)(F_INTERRUPTS * RCMM32_START_BIT_PAUSE_TIME * MIN_TOLERANCE_05 + 0.5) - 1)
#define RCMM32_START_BIT_PAUSE_LEN_MAX          ((uint_fast8_t)(F_INTERRUPTS * RCMM32_START_BIT_PAUSE_TIME * MAX_TOLERANCE_05 + 0.5) + 1)
#define RCMM32_BIT_PULSE_LEN_MIN                ((uint_fast8_t)(F_INTERRUPTS * RCMM32_PULSE_TIME * MIN_TOLERANCE_05 + 0.5) - 1)
#define RCMM32_BIT_PULSE_LEN_MAX                ((uint_fast8_t)(F_INTERRUPTS * RCMM32_PULSE_TIME * MAX_TOLERANCE_05 + 0.5) + 1)
#define RCMM32_BIT_00_PAUSE_LEN_MIN             ((uint_fast8_t)(F_INTERRUPTS * RCMM32_00_PAUSE_TIME * MIN_TOLERANCE_05 + 0.5) - 1)
#define RCMM32_BIT_00_PAUSE_LEN_MAX             ((uint_fast8_t)(F_INTERRUPTS * RCMM32_00_PAUSE_TIME * MAX_TOLERANCE_05 + 0.5) + 1)
#define RCMM32_BIT_01_PAUSE_LEN_MIN             ((uint_fast8_t)(F_INTERRUPTS * RCMM32_01_PAUSE_TIME * MIN_TOLERANCE_05 + 0.5) - 1)
#define RCMM32_BIT_01_PAUSE_LEN_MAX             ((uint_fast8_t)(F_INTERRUPTS * RCMM32_01_PAUSE_TIME * MAX_TOLERANCE_05 + 0.5) + 1)
#define RCMM32_BIT_10_PAUSE_LEN_MIN             ((uint_fast8_t)(F_INTERRUPTS * RCMM32_10_PAUSE_TIME * MIN_TOLERANCE_05 + 0.5) - 1)
#define RCMM32_BIT_10_PAUSE_LEN_MAX             ((uint_fast8_t)(F_INTERRUPTS * RCMM32_10_PAUSE_TIME * MAX_TOLERANCE_05 + 0.5) + 1)
#define RCMM32_BIT_11_PAUSE_LEN_MIN             ((uint_fast8_t)(F_INTERRUPTS * RCMM32_11_PAUSE_TIME * MIN_TOLERANCE_05 + 0.5) - 1)
#define RCMM32_BIT_11_PAUSE_LEN_MAX             ((uint_fast8_t)(F_INTERRUPTS * RCMM32_11_PAUSE_TIME * MAX_TOLERANCE_05 + 0.5) + 1)

#define RADIO1_START_BIT_PULSE_LEN_MIN          ((uint_fast8_t)(F_INTERRUPTS * RADIO1_START_BIT_PULSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define RADIO1_START_BIT_PULSE_LEN_MAX          ((uint_fast8_t)(F_INTERRUPTS * RADIO1_START_BIT_PULSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define RADIO1_START_BIT_PAUSE_LEN_MIN          ((uint_fast8_t)(F_INTERRUPTS * RADIO1_START_BIT_PAUSE_TIME * MIN_TOLERANCE_10 + 0.5) - 1)
#define RADIO1_START_BIT_PAUSE_LEN_MAX          ((uint_fast8_t)(F_INTERRUPTS * RADIO1_START_BIT_PAUSE_TIME * MAX_TOLERANCE_10 + 0.5) + 1)
#define RADIO1_1_PAUSE_LEN_EXACT                ((uint_fast8_t)(F_INTERRUPTS * RADIO1_1_PAUSE_TIME + 0.5))
#define RADIO1_1_PULSE_LEN_MIN                  ((uint_fast8_t)(F_INTERRUPTS * RADIO1_1_PULSE_TIME * MIN_TOLERANCE_20 + 0.5) - 1)
#define RADIO1_1_PULSE_LEN_MAX                  ((uint_fast8_t)(F_INTERRUPTS * RADIO1_1_PULSE_TIME * MAX_TOLERANCE_20 + 0.5) + 1)
#define RADIO1_1_PAUSE_LEN_MIN                  ((uint_fast8_t)(F_INTERRUPTS * RADIO1_1_PAUSE_TIME * MIN_TOLERANCE_20 + 0.5) - 1)
#define RADIO1_1_PAUSE_LEN_MAX                  ((uint_fast8_t)(F_INTERRUPTS * RADIO1_1_PAUSE_TIME * MAX_TOLERANCE_20 + 0.5) + 1)
#define RADIO1_0_PAUSE_LEN                      ((uint_fast8_t)(F_INTERRUPTS * RADIO1_0_PAUSE_TIME))
#define RADIO1_0_PULSE_LEN_MIN                  ((uint_fast8_t)(F_INTERRUPTS * RADIO1_0_PULSE_TIME * MIN_TOLERANCE_20 + 0.5) - 1)
#define RADIO1_0_PULSE_LEN_MAX                  ((uint_fast8_t)(F_INTERRUPTS * RADIO1_0_PULSE_TIME * MAX_TOLERANCE_20 + 0.5) + 1)
#define RADIO1_0_PAUSE_LEN_MIN                  ((uint_fast8_t)(F_INTERRUPTS * RADIO1_0_PAUSE_TIME * MIN_TOLERANCE_20 + 0.5) - 1)
#define RADIO1_0_PAUSE_LEN_MAX                  ((uint_fast8_t)(F_INTERRUPTS * RADIO1_0_PAUSE_TIME * MAX_TOLERANCE_20 + 0.5) + 1)

#define AUTO_FRAME_REPETITION_LEN               (uint_fast16_t)(F_INTERRUPTS * AUTO_FRAME_REPETITION_TIME + 0.5)       // use uint_fast16_t!

#define PARITY_CHECK_OK                         1
#define PARITY_CHECK_FAILED                     0

typedef struct
{
    uint_fast8_t    protocol;                                                // ir protocol
    uint_fast8_t    pulse_1_len_min;                                         // minimum length of pulse with bit value 1
    uint_fast8_t    pulse_1_len_max;                                         // maximum length of pulse with bit value 1
    uint_fast8_t    pause_1_len_min;                                         // minimum length of pause with bit value 1
    uint_fast8_t    pause_1_len_max;                                         // maximum length of pause with bit value 1
    uint_fast8_t    pulse_0_len_min;                                         // minimum length of pulse with bit value 0
    uint_fast8_t    pulse_0_len_max;                                         // maximum length of pulse with bit value 0
    uint_fast8_t    pause_0_len_min;                                         // minimum length of pause with bit value 0
    uint_fast8_t    pause_0_len_max;                                         // maximum length of pause with bit value 0
    uint_fast8_t    address_offset;                                          // address offset
    uint_fast8_t    address_end;                                             // end of address
    uint_fast8_t    command_offset;                                          // command offset
    uint_fast8_t    command_end;                                             // end of command
    uint_fast8_t    complete_len;                                            // complete length of frame
    uint_fast8_t    stop_bit;                                                // flag: frame has stop bit
    uint_fast8_t    lsb_first;                                               // flag: LSB first
    uint_fast8_t    flags;                                                   // some flags
} IRMP_PARAMETER;

#if IRMP_SUPPORT_SIRCS_PROTOCOL == 1

static const PROGMEM IRMP_PARAMETER sircs_param =
{
    IRMP_SIRCS_PROTOCOL,                                                // protocol:        ir protocol
    SIRCS_1_PULSE_LEN_MIN,                                              // pulse_1_len_min: minimum length of pulse with bit value 1
    SIRCS_1_PULSE_LEN_MAX,                                              // pulse_1_len_max: maximum length of pulse with bit value 1
    SIRCS_PAUSE_LEN_MIN,                                                // pause_1_len_min: minimum length of pause with bit value 1
    SIRCS_PAUSE_LEN_MAX,                                                // pause_1_len_max: maximum length of pause with bit value 1
    SIRCS_0_PULSE_LEN_MIN,                                              // pulse_0_len_min: minimum length of pulse with bit value 0
    SIRCS_0_PULSE_LEN_MAX,                                              // pulse_0_len_max: maximum length of pulse with bit value 0
    SIRCS_PAUSE_LEN_MIN,                                                // pause_0_len_min: minimum length of pause with bit value 0
    SIRCS_PAUSE_LEN_MAX,                                                // pause_0_len_max: maximum length of pause with bit value 0
    SIRCS_ADDRESS_OFFSET,                                               // address_offset:  address offset
    SIRCS_ADDRESS_OFFSET + SIRCS_ADDRESS_LEN,                           // address_end:     end of address
    SIRCS_COMMAND_OFFSET,                                               // command_offset:  command offset
    SIRCS_COMMAND_OFFSET + SIRCS_COMMAND_LEN,                           // command_end:     end of command
    SIRCS_COMPLETE_DATA_LEN,                                            // complete_len:    complete length of frame
    SIRCS_STOP_BIT,                                                     // stop_bit:        flag: frame has stop bit
    SIRCS_LSB,                                                          // lsb_first:       flag: LSB first
    SIRCS_FLAGS                                                         // flags:           some flags
};

#endif

#if IRMP_SUPPORT_NEC_PROTOCOL == 1

static const PROGMEM IRMP_PARAMETER nec_param =
{
    IRMP_NEC_PROTOCOL,                                                  // protocol:        ir protocol
    NEC_PULSE_LEN_MIN,                                                  // pulse_1_len_min: minimum length of pulse with bit value 1
    NEC_PULSE_LEN_MAX,                                                  // pulse_1_len_max: maximum length of pulse with bit value 1
    NEC_1_PAUSE_LEN_MIN,                                                // pause_1_len_min: minimum length of pause with bit value 1
    NEC_1_PAUSE_LEN_MAX,                                                // pause_1_len_max: maximum length of pause with bit value 1
    NEC_PULSE_LEN_MIN,                                                  // pulse_0_len_min: minimum length of pulse with bit value 0
    NEC_PULSE_LEN_MAX,                                                  // pulse_0_len_max: maximum length of pulse with bit value 0
    NEC_0_PAUSE_LEN_MIN,                                                // pause_0_len_min: minimum length of pause with bit value 0
    NEC_0_PAUSE_LEN_MAX,                                                // pause_0_len_max: maximum length of pause with bit value 0
    NEC_ADDRESS_OFFSET,                                                 // address_offset:  address offset
    NEC_ADDRESS_OFFSET + NEC_ADDRESS_LEN,                               // address_end:     end of address
    NEC_COMMAND_OFFSET,                                                 // command_offset:  command offset
    NEC_COMMAND_OFFSET + NEC_COMMAND_LEN,                               // command_end:     end of command
    NEC_COMPLETE_DATA_LEN,                                              // complete_len:    complete length of frame
    NEC_STOP_BIT,                                                       // stop_bit:        flag: frame has stop bit
    NEC_LSB,                                                            // lsb_first:       flag: LSB first
    NEC_FLAGS                                                           // flags:           some flags
};

static const PROGMEM IRMP_PARAMETER nec_rep_param =
{
    IRMP_NEC_PROTOCOL,                                                  // protocol:        ir protocol
    NEC_PULSE_LEN_MIN,                                                  // pulse_1_len_min: minimum length of pulse with bit value 1
    NEC_PULSE_LEN_MAX,                                                  // pulse_1_len_max: maximum length of pulse with bit value 1
    NEC_1_PAUSE_LEN_MIN,                                                // pause_1_len_min: minimum length of pause with bit value 1
    NEC_1_PAUSE_LEN_MAX,                                                // pause_1_len_max: maximum length of pause with bit value 1
    NEC_PULSE_LEN_MIN,                                                  // pulse_0_len_min: minimum length of pulse with bit value 0
    NEC_PULSE_LEN_MAX,                                                  // pulse_0_len_max: maximum length of pulse with bit value 0
    NEC_0_PAUSE_LEN_MIN,                                                // pause_0_len_min: minimum length of pause with bit value 0
    NEC_0_PAUSE_LEN_MAX,                                                // pause_0_len_max: maximum length of pause with bit value 0
    0,                                                                  // address_offset:  address offset
    0,                                                                  // address_end:     end of address
    0,                                                                  // command_offset:  command offset
    0,                                                                  // command_end:     end of command
    0,                                                                  // complete_len:    complete length of frame
    NEC_STOP_BIT,                                                       // stop_bit:        flag: frame has stop bit
    NEC_LSB,                                                            // lsb_first:       flag: LSB first
    NEC_FLAGS                                                           // flags:           some flags
};

#endif

#if IRMP_SUPPORT_NEC42_PROTOCOL == 1

static const PROGMEM IRMP_PARAMETER nec42_param =
{
    IRMP_NEC42_PROTOCOL,                                                // protocol:        ir protocol
    NEC_PULSE_LEN_MIN,                                                  // pulse_1_len_min: minimum length of pulse with bit value 1
    NEC_PULSE_LEN_MAX,                                                  // pulse_1_len_max: maximum length of pulse with bit value 1
    NEC_1_PAUSE_LEN_MIN,                                                // pause_1_len_min: minimum length of pause with bit value 1
    NEC_1_PAUSE_LEN_MAX,                                                // pause_1_len_max: maximum length of pause with bit value 1
    NEC_PULSE_LEN_MIN,                                                  // pulse_0_len_min: minimum length of pulse with bit value 0
    NEC_PULSE_LEN_MAX,                                                  // pulse_0_len_max: maximum length of pulse with bit value 0
    NEC_0_PAUSE_LEN_MIN,                                                // pause_0_len_min: minimum length of pause with bit value 0
    NEC_0_PAUSE_LEN_MAX,                                                // pause_0_len_max: maximum length of pause with bit value 0
    NEC42_ADDRESS_OFFSET,                                               // address_offset:  address offset
    NEC42_ADDRESS_OFFSET + NEC42_ADDRESS_LEN,                           // address_end:     end of address
    NEC42_COMMAND_OFFSET,                                               // command_offset:  command offset
    NEC42_COMMAND_OFFSET + NEC42_COMMAND_LEN,                           // command_end:     end of command
    NEC42_COMPLETE_DATA_LEN,                                            // complete_len:    complete length of frame
    NEC_STOP_BIT,                                                       // stop_bit:        flag: frame has stop bit
    NEC_LSB,                                                            // lsb_first:       flag: LSB first
    NEC_FLAGS                                                           // flags:           some flags
};

#endif

#if IRMP_SUPPORT_LGAIR_PROTOCOL == 1

static const PROGMEM IRMP_PARAMETER lgair_param =
{
    IRMP_LGAIR_PROTOCOL,                                                // protocol:        ir protocol
    NEC_PULSE_LEN_MIN,                                                  // pulse_1_len_min: minimum length of pulse with bit value 1
    NEC_PULSE_LEN_MAX,                                                  // pulse_1_len_max: maximum length of pulse with bit value 1
    NEC_1_PAUSE_LEN_MIN,                                                // pause_1_len_min: minimum length of pause with bit value 1
    NEC_1_PAUSE_LEN_MAX,                                                // pause_1_len_max: maximum length of pause with bit value 1
    NEC_PULSE_LEN_MIN,                                                  // pulse_0_len_min: minimum length of pulse with bit value 0
    NEC_PULSE_LEN_MAX,                                                  // pulse_0_len_max: maximum length of pulse with bit value 0
    NEC_0_PAUSE_LEN_MIN,                                                // pause_0_len_min: minimum length of pause with bit value 0
    NEC_0_PAUSE_LEN_MAX,                                                // pause_0_len_max: maximum length of pause with bit value 0
    LGAIR_ADDRESS_OFFSET,                                               // address_offset:  address offset
    LGAIR_ADDRESS_OFFSET + LGAIR_ADDRESS_LEN,                           // address_end:     end of address
    LGAIR_COMMAND_OFFSET,                                               // command_offset:  command offset
    LGAIR_COMMAND_OFFSET + LGAIR_COMMAND_LEN,                           // command_end:     end of command
    LGAIR_COMPLETE_DATA_LEN,                                            // complete_len:    complete length of frame
    NEC_STOP_BIT,                                                       // stop_bit:        flag: frame has stop bit
    NEC_LSB,                                                            // lsb_first:       flag: LSB first
    NEC_FLAGS                                                           // flags:           some flags
};

#endif

#if IRMP_SUPPORT_SAMSUNG_PROTOCOL == 1

static const PROGMEM IRMP_PARAMETER samsung_param =
{
    IRMP_SAMSUNG_PROTOCOL,                                              // protocol:        ir protocol
    SAMSUNG_PULSE_LEN_MIN,                                              // pulse_1_len_min: minimum length of pulse with bit value 1
    SAMSUNG_PULSE_LEN_MAX,                                              // pulse_1_len_max: maximum length of pulse with bit value 1
    SAMSUNG_1_PAUSE_LEN_MIN,                                            // pause_1_len_min: minimum length of pause with bit value 1
    SAMSUNG_1_PAUSE_LEN_MAX,                                            // pause_1_len_max: maximum length of pause with bit value 1
    SAMSUNG_PULSE_LEN_MIN,                                              // pulse_0_len_min: minimum length of pulse with bit value 0
    SAMSUNG_PULSE_LEN_MAX,                                              // pulse_0_len_max: maximum length of pulse with bit value 0
    SAMSUNG_0_PAUSE_LEN_MIN,                                            // pause_0_len_min: minimum length of pause with bit value 0
    SAMSUNG_0_PAUSE_LEN_MAX,                                            // pause_0_len_max: maximum length of pause with bit value 0
    SAMSUNG_ADDRESS_OFFSET,                                             // address_offset:  address offset
    SAMSUNG_ADDRESS_OFFSET + SAMSUNG_ADDRESS_LEN,                       // address_end:     end of address
    SAMSUNG_COMMAND_OFFSET,                                             // command_offset:  command offset
    SAMSUNG_COMMAND_OFFSET + SAMSUNG_COMMAND_LEN,                       // command_end:     end of command
    SAMSUNG_COMPLETE_DATA_LEN,                                          // complete_len:    complete length of frame
    SAMSUNG_STOP_BIT,                                                   // stop_bit:        flag: frame has stop bit
    SAMSUNG_LSB,                                                        // lsb_first:       flag: LSB first
    SAMSUNG_FLAGS                                                       // flags:           some flags
};

#endif

#if IRMP_SUPPORT_TELEFUNKEN_PROTOCOL == 1

static const PROGMEM IRMP_PARAMETER telefunken_param =
{
    IRMP_TELEFUNKEN_PROTOCOL,                                           // protocol:        ir protocol
    TELEFUNKEN_PULSE_LEN_MIN,                                           // pulse_1_len_min: minimum length of pulse with bit value 1
    TELEFUNKEN_PULSE_LEN_MAX,                                           // pulse_1_len_max: maximum length of pulse with bit value 1
    TELEFUNKEN_1_PAUSE_LEN_MIN,                                         // pause_1_len_min: minimum length of pause with bit value 1
    TELEFUNKEN_1_PAUSE_LEN_MAX,                                         // pause_1_len_max: maximum length of pause with bit value 1
    TELEFUNKEN_PULSE_LEN_MIN,                                           // pulse_0_len_min: minimum length of pulse with bit value 0
    TELEFUNKEN_PULSE_LEN_MAX,                                           // pulse_0_len_max: maximum length of pulse with bit value 0
    TELEFUNKEN_0_PAUSE_LEN_MIN,                                         // pause_0_len_min: minimum length of pause with bit value 0
    TELEFUNKEN_0_PAUSE_LEN_MAX,                                         // pause_0_len_max: maximum length of pause with bit value 0
    TELEFUNKEN_ADDRESS_OFFSET,                                          // address_offset:  address offset
    TELEFUNKEN_ADDRESS_OFFSET + TELEFUNKEN_ADDRESS_LEN,                 // address_end:     end of address
    TELEFUNKEN_COMMAND_OFFSET,                                          // command_offset:  command offset
    TELEFUNKEN_COMMAND_OFFSET + TELEFUNKEN_COMMAND_LEN,                 // command_end:     end of command
    TELEFUNKEN_COMPLETE_DATA_LEN,                                       // complete_len:    complete length of frame
    TELEFUNKEN_STOP_BIT,                                                // stop_bit:        flag: frame has stop bit
    TELEFUNKEN_LSB,                                                     // lsb_first:       flag: LSB first
    TELEFUNKEN_FLAGS                                                    // flags:           some flags
};

#endif

#if IRMP_SUPPORT_MATSUSHITA_PROTOCOL == 1

static const PROGMEM IRMP_PARAMETER matsushita_param =
{
    IRMP_MATSUSHITA_PROTOCOL,                                           // protocol:        ir protocol
    MATSUSHITA_PULSE_LEN_MIN,                                           // pulse_1_len_min: minimum length of pulse with bit value 1
    MATSUSHITA_PULSE_LEN_MAX,                                           // pulse_1_len_max: maximum length of pulse with bit value 1
    MATSUSHITA_1_PAUSE_LEN_MIN,                                         // pause_1_len_min: minimum length of pause with bit value 1
    MATSUSHITA_1_PAUSE_LEN_MAX,                                         // pause_1_len_max: maximum length of pause with bit value 1
    MATSUSHITA_PULSE_LEN_MIN,                                           // pulse_0_len_min: minimum length of pulse with bit value 0
    MATSUSHITA_PULSE_LEN_MAX,                                           // pulse_0_len_max: maximum length of pulse with bit value 0
    MATSUSHITA_0_PAUSE_LEN_MIN,                                         // pause_0_len_min: minimum length of pause with bit value 0
    MATSUSHITA_0_PAUSE_LEN_MAX,                                         // pause_0_len_max: maximum length of pause with bit value 0
    MATSUSHITA_ADDRESS_OFFSET,                                          // address_offset:  address offset
    MATSUSHITA_ADDRESS_OFFSET + MATSUSHITA_ADDRESS_LEN,                 // address_end:     end of address
    MATSUSHITA_COMMAND_OFFSET,                                          // command_offset:  command offset
    MATSUSHITA_COMMAND_OFFSET + MATSUSHITA_COMMAND_LEN,                 // command_end:     end of command
    MATSUSHITA_COMPLETE_DATA_LEN,                                       // complete_len:    complete length of frame
    MATSUSHITA_STOP_BIT,                                                // stop_bit:        flag: frame has stop bit
    MATSUSHITA_LSB,                                                     // lsb_first:       flag: LSB first
    MATSUSHITA_FLAGS                                                    // flags:           some flags
};

#endif

#if IRMP_SUPPORT_KASEIKYO_PROTOCOL == 1

static const PROGMEM IRMP_PARAMETER kaseikyo_param =
{
    IRMP_KASEIKYO_PROTOCOL,                                             // protocol:        ir protocol
    KASEIKYO_PULSE_LEN_MIN,                                             // pulse_1_len_min: minimum length of pulse with bit value 1
    KASEIKYO_PULSE_LEN_MAX,                                             // pulse_1_len_max: maximum length of pulse with bit value 1
    KASEIKYO_1_PAUSE_LEN_MIN,                                           // pause_1_len_min: minimum length of pause with bit value 1
    KASEIKYO_1_PAUSE_LEN_MAX,                                           // pause_1_len_max: maximum length of pause with bit value 1
    KASEIKYO_PULSE_LEN_MIN,                                             // pulse_0_len_min: minimum length of pulse with bit value 0
    KASEIKYO_PULSE_LEN_MAX,                                             // pulse_0_len_max: maximum length of pulse with bit value 0
    KASEIKYO_0_PAUSE_LEN_MIN,                                           // pause_0_len_min: minimum length of pause with bit value 0
    KASEIKYO_0_PAUSE_LEN_MAX,                                           // pause_0_len_max: maximum length of pause with bit value 0
    KASEIKYO_ADDRESS_OFFSET,                                            // address_offset:  address offset
    KASEIKYO_ADDRESS_OFFSET + KASEIKYO_ADDRESS_LEN,                     // address_end:     end of address
    KASEIKYO_COMMAND_OFFSET,                                            // command_offset:  command offset
    KASEIKYO_COMMAND_OFFSET + KASEIKYO_COMMAND_LEN,                     // command_end:     end of command
    KASEIKYO_COMPLETE_DATA_LEN,                                         // complete_len:    complete length of frame
    KASEIKYO_STOP_BIT,                                                  // stop_bit:        flag: frame has stop bit
    KASEIKYO_LSB,                                                       // lsb_first:       flag: LSB first
    KASEIKYO_FLAGS                                                      // flags:           some flags
};

#endif

#if IRMP_SUPPORT_RECS80_PROTOCOL == 1

static const PROGMEM IRMP_PARAMETER recs80_param =
{
    IRMP_RECS80_PROTOCOL,                                               // protocol:        ir protocol
    RECS80_PULSE_LEN_MIN,                                               // pulse_1_len_min: minimum length of pulse with bit value 1
    RECS80_PULSE_LEN_MAX,                                               // pulse_1_len_max: maximum length of pulse with bit value 1
    RECS80_1_PAUSE_LEN_MIN,                                             // pause_1_len_min: minimum length of pause with bit value 1
    RECS80_1_PAUSE_LEN_MAX,                                             // pause_1_len_max: maximum length of pause with bit value 1
    RECS80_PULSE_LEN_MIN,                                               // pulse_0_len_min: minimum length of pulse with bit value 0
    RECS80_PULSE_LEN_MAX,                                               // pulse_0_len_max: maximum length of pulse with bit value 0
    RECS80_0_PAUSE_LEN_MIN,                                             // pause_0_len_min: minimum length of pause with bit value 0
    RECS80_0_PAUSE_LEN_MAX,                                             // pause_0_len_max: maximum length of pause with bit value 0
    RECS80_ADDRESS_OFFSET,                                              // address_offset:  address offset
    RECS80_ADDRESS_OFFSET + RECS80_ADDRESS_LEN,                         // address_end:     end of address
    RECS80_COMMAND_OFFSET,                                              // command_offset:  command offset
    RECS80_COMMAND_OFFSET + RECS80_COMMAND_LEN,                         // command_end:     end of command
    RECS80_COMPLETE_DATA_LEN,                                           // complete_len:    complete length of frame
    RECS80_STOP_BIT,                                                    // stop_bit:        flag: frame has stop bit
    RECS80_LSB,                                                         // lsb_first:       flag: LSB first
    RECS80_FLAGS                                                        // flags:           some flags
};

#endif

#if IRMP_SUPPORT_RC5_PROTOCOL == 1

static const PROGMEM IRMP_PARAMETER rc5_param =
{
    IRMP_RC5_PROTOCOL,                                                  // protocol:        ir protocol
    RC5_BIT_LEN_MIN,                                                    // pulse_1_len_min: here: minimum length of short pulse
    RC5_BIT_LEN_MAX,                                                    // pulse_1_len_max: here: maximum length of short pulse
    RC5_BIT_LEN_MIN,                                                    // pause_1_len_min: here: minimum length of short pause
    RC5_BIT_LEN_MAX,                                                    // pause_1_len_max: here: maximum length of short pause
    0,                                                                  // pulse_0_len_min: here: not used
    0,                                                                  // pulse_0_len_max: here: not used
    0,                                                                  // pause_0_len_min: here: not used
    0,                                                                  // pause_0_len_max: here: not used
    RC5_ADDRESS_OFFSET,                                                 // address_offset:  address offset
    RC5_ADDRESS_OFFSET + RC5_ADDRESS_LEN,                               // address_end:     end of address
    RC5_COMMAND_OFFSET,                                                 // command_offset:  command offset
    RC5_COMMAND_OFFSET + RC5_COMMAND_LEN,                               // command_end:     end of command
    RC5_COMPLETE_DATA_LEN,                                              // complete_len:    complete length of frame
    RC5_STOP_BIT,                                                       // stop_bit:        flag: frame has stop bit
    RC5_LSB,                                                            // lsb_first:       flag: LSB first
    RC5_FLAGS                                                           // flags:           some flags
};

#endif

#if IRMP_SUPPORT_DENON_PROTOCOL == 1

static const PROGMEM IRMP_PARAMETER denon_param =
{
    IRMP_DENON_PROTOCOL,                                                // protocol:        ir protocol
    DENON_PULSE_LEN_MIN,                                                // pulse_1_len_min: minimum length of pulse with bit value 1
    DENON_PULSE_LEN_MAX,                                                // pulse_1_len_max: maximum length of pulse with bit value 1
    DENON_1_PAUSE_LEN_MIN,                                              // pause_1_len_min: minimum length of pause with bit value 1
    DENON_1_PAUSE_LEN_MAX,                                              // pause_1_len_max: maximum length of pause with bit value 1
    DENON_PULSE_LEN_MIN,                                                // pulse_0_len_min: minimum length of pulse with bit value 0
    DENON_PULSE_LEN_MAX,                                                // pulse_0_len_max: maximum length of pulse with bit value 0
    DENON_0_PAUSE_LEN_MIN,                                              // pause_0_len_min: minimum length of pause with bit value 0
    DENON_0_PAUSE_LEN_MAX,                                              // pause_0_len_max: maximum length of pause with bit value 0
    DENON_ADDRESS_OFFSET,                                               // address_offset:  address offset
    DENON_ADDRESS_OFFSET + DENON_ADDRESS_LEN,                           // address_end:     end of address
    DENON_COMMAND_OFFSET,                                               // command_offset:  command offset
    DENON_COMMAND_OFFSET + DENON_COMMAND_LEN,                           // command_end:     end of command
    DENON_COMPLETE_DATA_LEN,                                            // complete_len:    complete length of frame
    DENON_STOP_BIT,                                                     // stop_bit:        flag: frame has stop bit
    DENON_LSB,                                                          // lsb_first:       flag: LSB first
    DENON_FLAGS                                                         // flags:           some flags
};

#endif

#if IRMP_SUPPORT_RC6_PROTOCOL == 1

static const PROGMEM IRMP_PARAMETER rc6_param =
{
    IRMP_RC6_PROTOCOL,                                                  // protocol:        ir protocol

    RC6_BIT_PULSE_LEN_MIN,                                              // pulse_1_len_min: here: minimum length of short pulse
    RC6_BIT_PULSE_LEN_MAX,                                              // pulse_1_len_max: here: maximum length of short pulse
    RC6_BIT_PAUSE_LEN_MIN,                                              // pause_1_len_min: here: minimum length of short pause
    RC6_BIT_PAUSE_LEN_MAX,                                              // pause_1_len_max: here: maximum length of short pause
    0,                                                                  // pulse_0_len_min: here: not used
    0,                                                                  // pulse_0_len_max: here: not used
    0,                                                                  // pause_0_len_min: here: not used
    0,                                                                  // pause_0_len_max: here: not used
    RC6_ADDRESS_OFFSET,                                                 // address_offset:  address offset
    RC6_ADDRESS_OFFSET + RC6_ADDRESS_LEN,                               // address_end:     end of address
    RC6_COMMAND_OFFSET,                                                 // command_offset:  command offset
    RC6_COMMAND_OFFSET + RC6_COMMAND_LEN,                               // command_end:     end of command
    RC6_COMPLETE_DATA_LEN_SHORT,                                        // complete_len:    complete length of frame
    RC6_STOP_BIT,                                                       // stop_bit:        flag: frame has stop bit
    RC6_LSB,                                                            // lsb_first:       flag: LSB first
    RC6_FLAGS                                                           // flags:           some flags
};

#endif

#if IRMP_SUPPORT_RECS80EXT_PROTOCOL == 1

static const PROGMEM IRMP_PARAMETER recs80ext_param =
{
    IRMP_RECS80EXT_PROTOCOL,                                            // protocol:        ir protocol
    RECS80EXT_PULSE_LEN_MIN,                                            // pulse_1_len_min: minimum length of pulse with bit value 1
    RECS80EXT_PULSE_LEN_MAX,                                            // pulse_1_len_max: maximum length of pulse with bit value 1
    RECS80EXT_1_PAUSE_LEN_MIN,                                          // pause_1_len_min: minimum length of pause with bit value 1
    RECS80EXT_1_PAUSE_LEN_MAX,                                          // pause_1_len_max: maximum length of pause with bit value 1
    RECS80EXT_PULSE_LEN_MIN,                                            // pulse_0_len_min: minimum length of pulse with bit value 0
    RECS80EXT_PULSE_LEN_MAX,                                            // pulse_0_len_max: maximum length of pulse with bit value 0
    RECS80EXT_0_PAUSE_LEN_MIN,                                          // pause_0_len_min: minimum length of pause with bit value 0
    RECS80EXT_0_PAUSE_LEN_MAX,                                          // pause_0_len_max: maximum length of pause with bit value 0
    RECS80EXT_ADDRESS_OFFSET,                                           // address_offset:  address offset
    RECS80EXT_ADDRESS_OFFSET + RECS80EXT_ADDRESS_LEN,                   // address_end:     end of address
    RECS80EXT_COMMAND_OFFSET,                                           // command_offset:  command offset
    RECS80EXT_COMMAND_OFFSET + RECS80EXT_COMMAND_LEN,                   // command_end:     end of command
    RECS80EXT_COMPLETE_DATA_LEN,                                        // complete_len:    complete length of frame
    RECS80EXT_STOP_BIT,                                                 // stop_bit:        flag: frame has stop bit
    RECS80EXT_LSB,                                                      // lsb_first:       flag: LSB first
    RECS80EXT_FLAGS                                                     // flags:           some flags
};

#endif

#if IRMP_SUPPORT_NUBERT_PROTOCOL == 1

static const PROGMEM IRMP_PARAMETER nubert_param =
{
    IRMP_NUBERT_PROTOCOL,                                               // protocol:        ir protocol
    NUBERT_1_PULSE_LEN_MIN,                                             // pulse_1_len_min: minimum length of pulse with bit value 1
    NUBERT_1_PULSE_LEN_MAX,                                             // pulse_1_len_max: maximum length of pulse with bit value 1
    NUBERT_1_PAUSE_LEN_MIN,                                             // pause_1_len_min: minimum length of pause with bit value 1
    NUBERT_1_PAUSE_LEN_MAX,                                             // pause_1_len_max: maximum length of pause with bit value 1
    NUBERT_0_PULSE_LEN_MIN,                                             // pulse_0_len_min: minimum length of pulse with bit value 0
    NUBERT_0_PULSE_LEN_MAX,                                             // pulse_0_len_max: maximum length of pulse with bit value 0
    NUBERT_0_PAUSE_LEN_MIN,                                             // pause_0_len_min: minimum length of pause with bit value 0
    NUBERT_0_PAUSE_LEN_MAX,                                             // pause_0_len_max: maximum length of pause with bit value 0
    NUBERT_ADDRESS_OFFSET,                                              // address_offset:  address offset
    NUBERT_ADDRESS_OFFSET + NUBERT_ADDRESS_LEN,                         // address_end:     end of address
    NUBERT_COMMAND_OFFSET,                                              // command_offset:  command offset
    NUBERT_COMMAND_OFFSET + NUBERT_COMMAND_LEN,                         // command_end:     end of command
    NUBERT_COMPLETE_DATA_LEN,                                           // complete_len:    complete length of frame
    NUBERT_STOP_BIT,                                                    // stop_bit:        flag: frame has stop bit
    NUBERT_LSB,                                                         // lsb_first:       flag: LSB first
    NUBERT_FLAGS                                                        // flags:           some flags
};

#endif

#if IRMP_SUPPORT_SPEAKER_PROTOCOL == 1

static const PROGMEM IRMP_PARAMETER speaker_param =
{
    IRMP_SPEAKER_PROTOCOL,                                              // protocol:        ir protocol
    SPEAKER_1_PULSE_LEN_MIN,                                            // pulse_1_len_min: minimum length of pulse with bit value 1
    SPEAKER_1_PULSE_LEN_MAX,                                            // pulse_1_len_max: maximum length of pulse with bit value 1
    SPEAKER_1_PAUSE_LEN_MIN,                                            // pause_1_len_min: minimum length of pause with bit value 1
    SPEAKER_1_PAUSE_LEN_MAX,                                            // pause_1_len_max: maximum length of pause with bit value 1
    SPEAKER_0_PULSE_LEN_MIN,                                            // pulse_0_len_min: minimum length of pulse with bit value 0
    SPEAKER_0_PULSE_LEN_MAX,                                            // pulse_0_len_max: maximum length of pulse with bit value 0
    SPEAKER_0_PAUSE_LEN_MIN,                                            // pause_0_len_min: minimum length of pause with bit value 0
    SPEAKER_0_PAUSE_LEN_MAX,                                            // pause_0_len_max: maximum length of pause with bit value 0
    SPEAKER_ADDRESS_OFFSET,                                             // address_offset:  address offset
    SPEAKER_ADDRESS_OFFSET + SPEAKER_ADDRESS_LEN,                       // address_end:     end of address
    SPEAKER_COMMAND_OFFSET,                                             // command_offset:  command offset
    SPEAKER_COMMAND_OFFSET + SPEAKER_COMMAND_LEN,                       // command_end:     end of command
    SPEAKER_COMPLETE_DATA_LEN,                                          // complete_len:    complete length of frame
    SPEAKER_STOP_BIT,                                                   // stop_bit:        flag: frame has stop bit
    SPEAKER_LSB,                                                        // lsb_first:       flag: LSB first
    SPEAKER_FLAGS                                                       // flags:           some flags
};

#endif

#if IRMP_SUPPORT_BANG_OLUFSEN_PROTOCOL == 1

static const PROGMEM IRMP_PARAMETER bang_olufsen_param =
{
    IRMP_BANG_OLUFSEN_PROTOCOL,                                         // protocol:        ir protocol
    BANG_OLUFSEN_PULSE_LEN_MIN,                                         // pulse_1_len_min: minimum length of pulse with bit value 1
    BANG_OLUFSEN_PULSE_LEN_MAX,                                         // pulse_1_len_max: maximum length of pulse with bit value 1
    BANG_OLUFSEN_1_PAUSE_LEN_MIN,                                       // pause_1_len_min: minimum length of pause with bit value 1
    BANG_OLUFSEN_1_PAUSE_LEN_MAX,                                       // pause_1_len_max: maximum length of pause with bit value 1
    BANG_OLUFSEN_PULSE_LEN_MIN,                                         // pulse_0_len_min: minimum length of pulse with bit value 0
    BANG_OLUFSEN_PULSE_LEN_MAX,                                         // pulse_0_len_max: maximum length of pulse with bit value 0
    BANG_OLUFSEN_0_PAUSE_LEN_MIN,                                       // pause_0_len_min: minimum length of pause with bit value 0
    BANG_OLUFSEN_0_PAUSE_LEN_MAX,                                       // pause_0_len_max: maximum length of pause with bit value 0
    BANG_OLUFSEN_ADDRESS_OFFSET,                                        // address_offset:  address offset
    BANG_OLUFSEN_ADDRESS_OFFSET + BANG_OLUFSEN_ADDRESS_LEN,             // address_end:     end of address
    BANG_OLUFSEN_COMMAND_OFFSET,                                        // command_offset:  command offset
    BANG_OLUFSEN_COMMAND_OFFSET + BANG_OLUFSEN_COMMAND_LEN,             // command_end:     end of command
    BANG_OLUFSEN_COMPLETE_DATA_LEN,                                     // complete_len:    complete length of frame
    BANG_OLUFSEN_STOP_BIT,                                              // stop_bit:        flag: frame has stop bit
    BANG_OLUFSEN_LSB,                                                   // lsb_first:       flag: LSB first
    BANG_OLUFSEN_FLAGS                                                  // flags:           some flags
};

#endif

#if IRMP_SUPPORT_GRUNDIG_NOKIA_IR60_PROTOCOL == 1

static uint_fast8_t first_bit;

static const PROGMEM IRMP_PARAMETER grundig_param =
{
    IRMP_GRUNDIG_PROTOCOL,                                              // protocol:        ir protocol

    GRUNDIG_NOKIA_IR60_BIT_LEN_MIN,                                     // pulse_1_len_min: here: minimum length of short pulse
    GRUNDIG_NOKIA_IR60_BIT_LEN_MAX,                                     // pulse_1_len_max: here: maximum length of short pulse
    GRUNDIG_NOKIA_IR60_BIT_LEN_MIN,                                     // pause_1_len_min: here: minimum length of short pause
    GRUNDIG_NOKIA_IR60_BIT_LEN_MAX,                                     // pause_1_len_max: here: maximum length of short pause
    0,                                                                  // pulse_0_len_min: here: not used
    0,                                                                  // pulse_0_len_max: here: not used
    0,                                                                  // pause_0_len_min: here: not used
    0,                                                                  // pause_0_len_max: here: not used
    GRUNDIG_ADDRESS_OFFSET,                                             // address_offset:  address offset
    GRUNDIG_ADDRESS_OFFSET + GRUNDIG_ADDRESS_LEN,                       // address_end:     end of address
    GRUNDIG_COMMAND_OFFSET,                                             // command_offset:  command offset
    GRUNDIG_COMMAND_OFFSET + GRUNDIG_COMMAND_LEN + 1,                   // command_end:     end of command (USE 1 bit MORE to STORE NOKIA DATA!)
    NOKIA_COMPLETE_DATA_LEN,                                            // complete_len:    complete length of frame, here: NOKIA instead of GRUNDIG!
    GRUNDIG_NOKIA_IR60_STOP_BIT,                                        // stop_bit:        flag: frame has stop bit
    GRUNDIG_NOKIA_IR60_LSB,                                             // lsb_first:       flag: LSB first
    GRUNDIG_NOKIA_IR60_FLAGS                                            // flags:           some flags
};

#endif

#if IRMP_SUPPORT_SIEMENS_OR_RUWIDO_PROTOCOL == 1

static const PROGMEM IRMP_PARAMETER ruwido_param =
{
    IRMP_RUWIDO_PROTOCOL,                                               // protocol:        ir protocol
    SIEMENS_OR_RUWIDO_BIT_PULSE_LEN_MIN,                                // pulse_1_len_min: here: minimum length of short pulse
    SIEMENS_OR_RUWIDO_BIT_PULSE_LEN_MAX,                                // pulse_1_len_max: here: maximum length of short pulse
    SIEMENS_OR_RUWIDO_BIT_PAUSE_LEN_MIN,                                // pause_1_len_min: here: minimum length of short pause
    SIEMENS_OR_RUWIDO_BIT_PAUSE_LEN_MAX,                                // pause_1_len_max: here: maximum length of short pause
    0,                                                                  // pulse_0_len_min: here: not used
    0,                                                                  // pulse_0_len_max: here: not used
    0,                                                                  // pause_0_len_min: here: not used
    0,                                                                  // pause_0_len_max: here: not used
    RUWIDO_ADDRESS_OFFSET,                                              // address_offset:  address offset
    RUWIDO_ADDRESS_OFFSET + RUWIDO_ADDRESS_LEN,                         // address_end:     end of address
    RUWIDO_COMMAND_OFFSET,                                              // command_offset:  command offset
    RUWIDO_COMMAND_OFFSET + RUWIDO_COMMAND_LEN,                         // command_end:     end of command
    SIEMENS_COMPLETE_DATA_LEN,                                          // complete_len:    complete length of frame, here: SIEMENS instead of RUWIDO!
    SIEMENS_OR_RUWIDO_STOP_BIT,                                         // stop_bit:        flag: frame has stop bit
    SIEMENS_OR_RUWIDO_LSB,                                              // lsb_first:       flag: LSB first
    SIEMENS_OR_RUWIDO_FLAGS                                             // flags:           some flags
};

#endif

#if IRMP_SUPPORT_FDC_PROTOCOL == 1

static const PROGMEM IRMP_PARAMETER fdc_param =
{
    IRMP_FDC_PROTOCOL,                                                  // protocol:        ir protocol
    FDC_PULSE_LEN_MIN,                                                  // pulse_1_len_min: minimum length of pulse with bit value 1
    FDC_PULSE_LEN_MAX,                                                  // pulse_1_len_max: maximum length of pulse with bit value 1
    FDC_1_PAUSE_LEN_MIN,                                                // pause_1_len_min: minimum length of pause with bit value 1
    FDC_1_PAUSE_LEN_MAX,                                                // pause_1_len_max: maximum length of pause with bit value 1
    FDC_PULSE_LEN_MIN,                                                  // pulse_0_len_min: minimum length of pulse with bit value 0
    FDC_PULSE_LEN_MAX,                                                  // pulse_0_len_max: maximum length of pulse with bit value 0
    FDC_0_PAUSE_LEN_MIN,                                                // pause_0_len_min: minimum length of pause with bit value 0
    FDC_0_PAUSE_LEN_MAX,                                                // pause_0_len_max: maximum length of pause with bit value 0
    FDC_ADDRESS_OFFSET,                                                 // address_offset:  address offset
    FDC_ADDRESS_OFFSET + FDC_ADDRESS_LEN,                               // address_end:     end of address
    FDC_COMMAND_OFFSET,                                                 // command_offset:  command offset
    FDC_COMMAND_OFFSET + FDC_COMMAND_LEN,                               // command_end:     end of command
    FDC_COMPLETE_DATA_LEN,                                              // complete_len:    complete length of frame
    FDC_STOP_BIT,                                                       // stop_bit:        flag: frame has stop bit
    FDC_LSB,                                                            // lsb_first:       flag: LSB first
    FDC_FLAGS                                                           // flags:           some flags
};

#endif

#if IRMP_SUPPORT_RCCAR_PROTOCOL == 1

static const PROGMEM IRMP_PARAMETER rccar_param =
{
    IRMP_RCCAR_PROTOCOL,                                                // protocol:        ir protocol
    RCCAR_PULSE_LEN_MIN,                                                // pulse_1_len_min: minimum length of pulse with bit value 1
    RCCAR_PULSE_LEN_MAX,                                                // pulse_1_len_max: maximum length of pulse with bit value 1
    RCCAR_1_PAUSE_LEN_MIN,                                              // pause_1_len_min: minimum length of pause with bit value 1
    RCCAR_1_PAUSE_LEN_MAX,                                              // pause_1_len_max: maximum length of pause with bit value 1
    RCCAR_PULSE_LEN_MIN,                                                // pulse_0_len_min: minimum length of pulse with bit value 0
    RCCAR_PULSE_LEN_MAX,                                                // pulse_0_len_max: maximum length of pulse with bit value 0
    RCCAR_0_PAUSE_LEN_MIN,                                              // pause_0_len_min: minimum length of pause with bit value 0
    RCCAR_0_PAUSE_LEN_MAX,                                              // pause_0_len_max: maximum length of pause with bit value 0
    RCCAR_ADDRESS_OFFSET,                                               // address_offset:  address offset
    RCCAR_ADDRESS_OFFSET + RCCAR_ADDRESS_LEN,                           // address_end:     end of address
    RCCAR_COMMAND_OFFSET,                                               // command_offset:  command offset
    RCCAR_COMMAND_OFFSET + RCCAR_COMMAND_LEN,                           // command_end:     end of command
    RCCAR_COMPLETE_DATA_LEN,                                            // complete_len:    complete length of frame
    RCCAR_STOP_BIT,                                                     // stop_bit:        flag: frame has stop bit
    RCCAR_LSB,                                                          // lsb_first:       flag: LSB first
    RCCAR_FLAGS                                                         // flags:           some flags
};

#endif

#if IRMP_SUPPORT_NIKON_PROTOCOL == 1

static const PROGMEM IRMP_PARAMETER nikon_param =
{
    IRMP_NIKON_PROTOCOL,                                                // protocol:        ir protocol
    NIKON_PULSE_LEN_MIN,                                                // pulse_1_len_min: minimum length of pulse with bit value 1
    NIKON_PULSE_LEN_MAX,                                                // pulse_1_len_max: maximum length of pulse with bit value 1
    NIKON_1_PAUSE_LEN_MIN,                                              // pause_1_len_min: minimum length of pause with bit value 1
    NIKON_1_PAUSE_LEN_MAX,                                              // pause_1_len_max: maximum length of pause with bit value 1
    NIKON_PULSE_LEN_MIN,                                                // pulse_0_len_min: minimum length of pulse with bit value 0
    NIKON_PULSE_LEN_MAX,                                                // pulse_0_len_max: maximum length of pulse with bit value 0
    NIKON_0_PAUSE_LEN_MIN,                                              // pause_0_len_min: minimum length of pause with bit value 0
    NIKON_0_PAUSE_LEN_MAX,                                              // pause_0_len_max: maximum length of pause with bit value 0
    NIKON_ADDRESS_OFFSET,                                               // address_offset:  address offset
    NIKON_ADDRESS_OFFSET + NIKON_ADDRESS_LEN,                           // address_end:     end of address
    NIKON_COMMAND_OFFSET,                                               // command_offset:  command offset
    NIKON_COMMAND_OFFSET + NIKON_COMMAND_LEN,                           // command_end:     end of command
    NIKON_COMPLETE_DATA_LEN,                                            // complete_len:    complete length of frame
    NIKON_STOP_BIT,                                                     // stop_bit:        flag: frame has stop bit
    NIKON_LSB,                                                          // lsb_first:       flag: LSB first
    NIKON_FLAGS                                                         // flags:           some flags
};

#endif

#if IRMP_SUPPORT_KATHREIN_PROTOCOL == 1

static const PROGMEM IRMP_PARAMETER kathrein_param =
{
    IRMP_KATHREIN_PROTOCOL,                                             // protocol:        ir protocol
    KATHREIN_1_PULSE_LEN_MIN,                                           // pulse_1_len_min: minimum length of pulse with bit value 1
    KATHREIN_1_PULSE_LEN_MAX,                                           // pulse_1_len_max: maximum length of pulse with bit value 1
    KATHREIN_1_PAUSE_LEN_MIN,                                           // pause_1_len_min: minimum length of pause with bit value 1
    KATHREIN_1_PAUSE_LEN_MAX,                                           // pause_1_len_max: maximum length of pause with bit value 1
    KATHREIN_0_PULSE_LEN_MIN,                                           // pulse_0_len_min: minimum length of pulse with bit value 0
    KATHREIN_0_PULSE_LEN_MAX,                                           // pulse_0_len_max: maximum length of pulse with bit value 0
    KATHREIN_0_PAUSE_LEN_MIN,                                           // pause_0_len_min: minimum length of pause with bit value 0
    KATHREIN_0_PAUSE_LEN_MAX,                                           // pause_0_len_max: maximum length of pause with bit value 0
    KATHREIN_ADDRESS_OFFSET,                                            // address_offset:  address offset
    KATHREIN_ADDRESS_OFFSET + KATHREIN_ADDRESS_LEN,                     // address_end:     end of address
    KATHREIN_COMMAND_OFFSET,                                            // command_offset:  command offset
    KATHREIN_COMMAND_OFFSET + KATHREIN_COMMAND_LEN,                     // command_end:     end of command
    KATHREIN_COMPLETE_DATA_LEN,                                         // complete_len:    complete length of frame
    KATHREIN_STOP_BIT,                                                  // stop_bit:        flag: frame has stop bit
    KATHREIN_LSB,                                                       // lsb_first:       flag: LSB first
    KATHREIN_FLAGS                                                      // flags:           some flags
};

#endif

#if IRMP_SUPPORT_NETBOX_PROTOCOL == 1

static const PROGMEM IRMP_PARAMETER netbox_param =
{
    IRMP_NETBOX_PROTOCOL,                                               // protocol:        ir protocol
    NETBOX_PULSE_LEN,                                                   // pulse_1_len_min: minimum length of pulse with bit value 1, here: exact value
    NETBOX_PULSE_REST_LEN,                                              // pulse_1_len_max: maximum length of pulse with bit value 1, here: rest value
    NETBOX_PAUSE_LEN,                                                   // pause_1_len_min: minimum length of pause with bit value 1, here: exact value
    NETBOX_PAUSE_REST_LEN,                                              // pause_1_len_max: maximum length of pause with bit value 1, here: rest value
    NETBOX_PULSE_LEN,                                                   // pulse_0_len_min: minimum length of pulse with bit value 0, here: exact value
    NETBOX_PULSE_REST_LEN,                                              // pulse_0_len_max: maximum length of pulse with bit value 0, here: rest value
    NETBOX_PAUSE_LEN,                                                   // pause_0_len_min: minimum length of pause with bit value 0, here: exact value
    NETBOX_PAUSE_REST_LEN,                                              // pause_0_len_max: maximum length of pause with bit value 0, here: rest value
    NETBOX_ADDRESS_OFFSET,                                              // address_offset:  address offset
    NETBOX_ADDRESS_OFFSET + NETBOX_ADDRESS_LEN,                         // address_end:     end of address
    NETBOX_COMMAND_OFFSET,                                              // command_offset:  command offset
    NETBOX_COMMAND_OFFSET + NETBOX_COMMAND_LEN,                         // command_end:     end of command
    NETBOX_COMPLETE_DATA_LEN,                                           // complete_len:    complete length of frame
    NETBOX_STOP_BIT,                                                    // stop_bit:        flag: frame has stop bit
    NETBOX_LSB,                                                         // lsb_first:       flag: LSB first
    NETBOX_FLAGS                                                        // flags:           some flags
};

#endif

#if IRMP_SUPPORT_LEGO_PROTOCOL == 1

static const PROGMEM IRMP_PARAMETER lego_param =
{
    IRMP_LEGO_PROTOCOL,                                                 // protocol:        ir protocol
    LEGO_PULSE_LEN_MIN,                                                 // pulse_1_len_min: minimum length of pulse with bit value 1
    LEGO_PULSE_LEN_MAX,                                                 // pulse_1_len_max: maximum length of pulse with bit value 1
    LEGO_1_PAUSE_LEN_MIN,                                               // pause_1_len_min: minimum length of pause with bit value 1
    LEGO_1_PAUSE_LEN_MAX,                                               // pause_1_len_max: maximum length of pause with bit value 1
    LEGO_PULSE_LEN_MIN,                                                 // pulse_0_len_min: minimum length of pulse with bit value 0
    LEGO_PULSE_LEN_MAX,                                                 // pulse_0_len_max: maximum length of pulse with bit value 0
    LEGO_0_PAUSE_LEN_MIN,                                               // pause_0_len_min: minimum length of pause with bit value 0
    LEGO_0_PAUSE_LEN_MAX,                                               // pause_0_len_max: maximum length of pause with bit value 0
    LEGO_ADDRESS_OFFSET,                                                // address_offset:  address offset
    LEGO_ADDRESS_OFFSET + LEGO_ADDRESS_LEN,                             // address_end:     end of address
    LEGO_COMMAND_OFFSET,                                                // command_offset:  command offset
    LEGO_COMMAND_OFFSET + LEGO_COMMAND_LEN,                             // command_end:     end of command
    LEGO_COMPLETE_DATA_LEN,                                             // complete_len:    complete length of frame
    LEGO_STOP_BIT,                                                      // stop_bit:        flag: frame has stop bit
    LEGO_LSB,                                                           // lsb_first:       flag: LSB first
    LEGO_FLAGS                                                          // flags:           some flags
};

#endif

#if IRMP_SUPPORT_THOMSON_PROTOCOL == 1

static const PROGMEM IRMP_PARAMETER thomson_param =
{
    IRMP_THOMSON_PROTOCOL,                                              // protocol:        ir protocol
    THOMSON_PULSE_LEN_MIN,                                              // pulse_1_len_min: minimum length of pulse with bit value 1
    THOMSON_PULSE_LEN_MAX,                                              // pulse_1_len_max: maximum length of pulse with bit value 1
    THOMSON_1_PAUSE_LEN_MIN,                                            // pause_1_len_min: minimum length of pause with bit value 1
    THOMSON_1_PAUSE_LEN_MAX,                                            // pause_1_len_max: maximum length of pause with bit value 1
    THOMSON_PULSE_LEN_MIN,                                              // pulse_0_len_min: minimum length of pulse with bit value 0
    THOMSON_PULSE_LEN_MAX,                                              // pulse_0_len_max: maximum length of pulse with bit value 0
    THOMSON_0_PAUSE_LEN_MIN,                                            // pause_0_len_min: minimum length of pause with bit value 0
    THOMSON_0_PAUSE_LEN_MAX,                                            // pause_0_len_max: maximum length of pause with bit value 0
    THOMSON_ADDRESS_OFFSET,                                             // address_offset:  address offset
    THOMSON_ADDRESS_OFFSET + THOMSON_ADDRESS_LEN,                       // address_end:     end of address
    THOMSON_COMMAND_OFFSET,                                             // command_offset:  command offset
    THOMSON_COMMAND_OFFSET + THOMSON_COMMAND_LEN,                       // command_end:     end of command
    THOMSON_COMPLETE_DATA_LEN,                                          // complete_len:    complete length of frame
    THOMSON_STOP_BIT,                                                   // stop_bit:        flag: frame has stop bit
    THOMSON_LSB,                                                        // lsb_first:       flag: LSB first
    THOMSON_FLAGS                                                       // flags:           some flags
};

#endif

#if IRMP_SUPPORT_BOSE_PROTOCOL == 1

static const PROGMEM IRMP_PARAMETER bose_param =
{
    IRMP_BOSE_PROTOCOL,                                                 // protocol:        ir protocol
    BOSE_PULSE_LEN_MIN,                                                 // pulse_1_len_min: minimum length of pulse with bit value 1
    BOSE_PULSE_LEN_MAX,                                                 // pulse_1_len_max: maximum length of pulse with bit value 1
    BOSE_1_PAUSE_LEN_MIN,                                               // pause_1_len_min: minimum length of pause with bit value 1
    BOSE_1_PAUSE_LEN_MAX,                                               // pause_1_len_max: maximum length of pause with bit value 1
    BOSE_PULSE_LEN_MIN,                                                 // pulse_0_len_min: minimum length of pulse with bit value 0
    BOSE_PULSE_LEN_MAX,                                                 // pulse_0_len_max: maximum length of pulse with bit value 0
    BOSE_0_PAUSE_LEN_MIN,                                               // pause_0_len_min: minimum length of pause with bit value 0
    BOSE_0_PAUSE_LEN_MAX,                                               // pause_0_len_max: maximum length of pause with bit value 0
    BOSE_ADDRESS_OFFSET,                                                // address_offset:  address offset
    BOSE_ADDRESS_OFFSET + BOSE_ADDRESS_LEN,                             // address_end:     end of address
    BOSE_COMMAND_OFFSET,                                                // command_offset:  command offset
    BOSE_COMMAND_OFFSET + BOSE_COMMAND_LEN,                             // command_end:     end of command
    BOSE_COMPLETE_DATA_LEN,                                             // complete_len:    complete length of frame
    BOSE_STOP_BIT,                                                      // stop_bit:        flag: frame has stop bit
    BOSE_LSB,                                                           // lsb_first:       flag: LSB first
    BOSE_FLAGS                                                          // flags:           some flags
};

#endif

#if IRMP_SUPPORT_A1TVBOX_PROTOCOL == 1

static const PROGMEM IRMP_PARAMETER a1tvbox_param =
{
    IRMP_A1TVBOX_PROTOCOL,                                              // protocol:        ir protocol

    A1TVBOX_BIT_PULSE_LEN_MIN,                                          // pulse_1_len_min: here: minimum length of short pulse
    A1TVBOX_BIT_PULSE_LEN_MAX,                                          // pulse_1_len_max: here: maximum length of short pulse
    A1TVBOX_BIT_PAUSE_LEN_MIN,                                          // pause_1_len_min: here: minimum length of short pause
    A1TVBOX_BIT_PAUSE_LEN_MAX,                                          // pause_1_len_max: here: maximum length of short pause
    0,                                                                  // pulse_0_len_min: here: not used
    0,                                                                  // pulse_0_len_max: here: not used
    0,                                                                  // pause_0_len_min: here: not used
    0,                                                                  // pause_0_len_max: here: not used
    A1TVBOX_ADDRESS_OFFSET,                                             // address_offset:  address offset
    A1TVBOX_ADDRESS_OFFSET + A1TVBOX_ADDRESS_LEN,                       // address_end:     end of address
    A1TVBOX_COMMAND_OFFSET,                                             // command_offset:  command offset
    A1TVBOX_COMMAND_OFFSET + A1TVBOX_COMMAND_LEN,                       // command_end:     end of command
    A1TVBOX_COMPLETE_DATA_LEN,                                          // complete_len:    complete length of frame
    A1TVBOX_STOP_BIT,                                                   // stop_bit:        flag: frame has stop bit
    A1TVBOX_LSB,                                                        // lsb_first:       flag: LSB first
    A1TVBOX_FLAGS                                                       // flags:           some flags
};

#endif

#if IRMP_SUPPORT_ORTEK_PROTOCOL == 1

static const PROGMEM IRMP_PARAMETER ortek_param =
{
    IRMP_ORTEK_PROTOCOL,                                                // protocol:        ir protocol

    ORTEK_BIT_PULSE_LEN_MIN,                                            // pulse_1_len_min: here: minimum length of short pulse
    ORTEK_BIT_PULSE_LEN_MAX,                                            // pulse_1_len_max: here: maximum length of short pulse
    ORTEK_BIT_PAUSE_LEN_MIN,                                            // pause_1_len_min: here: minimum length of short pause
    ORTEK_BIT_PAUSE_LEN_MAX,                                            // pause_1_len_max: here: maximum length of short pause
    0,                                                                  // pulse_0_len_min: here: not used
    0,                                                                  // pulse_0_len_max: here: not used
    0,                                                                  // pause_0_len_min: here: not used
    0,                                                                  // pause_0_len_max: here: not used
    ORTEK_ADDRESS_OFFSET,                                               // address_offset:  address offset
    ORTEK_ADDRESS_OFFSET + ORTEK_ADDRESS_LEN,                           // address_end:     end of address
    ORTEK_COMMAND_OFFSET,                                               // command_offset:  command offset
    ORTEK_COMMAND_OFFSET + ORTEK_COMMAND_LEN,                           // command_end:     end of command
    ORTEK_COMPLETE_DATA_LEN,                                            // complete_len:    complete length of frame
    ORTEK_STOP_BIT,                                                     // stop_bit:        flag: frame has stop bit
    ORTEK_LSB,                                                          // lsb_first:       flag: LSB first
    ORTEK_FLAGS                                                         // flags:           some flags
};

#endif

#if IRMP_SUPPORT_ROOMBA_PROTOCOL == 1

static const PROGMEM IRMP_PARAMETER roomba_param =
{
    IRMP_ROOMBA_PROTOCOL,                                               // protocol:        ir protocol
    ROOMBA_1_PULSE_LEN_MIN,                                             // pulse_1_len_min: minimum length of pulse with bit value 1
    ROOMBA_1_PULSE_LEN_MAX,                                             // pulse_1_len_max: maximum length of pulse with bit value 1
    ROOMBA_1_PAUSE_LEN_MIN,                                             // pause_1_len_min: minimum length of pause with bit value 1
    ROOMBA_1_PAUSE_LEN_MAX,                                             // pause_1_len_max: maximum length of pause with bit value 1
    ROOMBA_0_PULSE_LEN_MIN,                                             // pulse_0_len_min: minimum length of pulse with bit value 0
    ROOMBA_0_PULSE_LEN_MAX,                                             // pulse_0_len_max: maximum length of pulse with bit value 0
    ROOMBA_0_PAUSE_LEN_MIN,                                             // pause_0_len_min: minimum length of pause with bit value 0
    ROOMBA_0_PAUSE_LEN_MAX,                                             // pause_0_len_max: maximum length of pause with bit value 0
    ROOMBA_ADDRESS_OFFSET,                                              // address_offset:  address offset
    ROOMBA_ADDRESS_OFFSET + ROOMBA_ADDRESS_LEN,                         // address_end:     end of address
    ROOMBA_COMMAND_OFFSET,                                              // command_offset:  command offset
    ROOMBA_COMMAND_OFFSET + ROOMBA_COMMAND_LEN,                         // command_end:     end of command
    ROOMBA_COMPLETE_DATA_LEN,                                           // complete_len:    complete length of frame
    ROOMBA_STOP_BIT,                                                    // stop_bit:        flag: frame has stop bit
    ROOMBA_LSB,                                                         // lsb_first:       flag: LSB first
    ROOMBA_FLAGS                                                        // flags:           some flags
};

#endif

#if IRMP_SUPPORT_RCMM_PROTOCOL == 1

static const PROGMEM IRMP_PARAMETER rcmm_param =
{
    IRMP_RCMM32_PROTOCOL,                                               // protocol:        ir protocol

    RCMM32_BIT_PULSE_LEN_MIN,                                           // pulse_1_len_min: here: minimum length of short pulse
    RCMM32_BIT_PULSE_LEN_MAX,                                           // pulse_1_len_max: here: maximum length of short pulse
    0,                                                                  // pause_1_len_min: here: minimum length of short pause
    0,                                                                  // pause_1_len_max: here: maximum length of short pause
    RCMM32_BIT_PULSE_LEN_MIN,                                           // pulse_0_len_min: here: not used
    RCMM32_BIT_PULSE_LEN_MAX,                                           // pulse_0_len_max: here: not used
    0,                                                                  // pause_0_len_min: here: not used
    0,                                                                  // pause_0_len_max: here: not used
    RCMM32_ADDRESS_OFFSET,                                              // address_offset:  address offset
    RCMM32_ADDRESS_OFFSET + RCMM32_ADDRESS_LEN,                         // address_end:     end of address
    RCMM32_COMMAND_OFFSET,                                              // command_offset:  command offset
    RCMM32_COMMAND_OFFSET + RCMM32_COMMAND_LEN,                         // command_end:     end of command
    RCMM32_COMPLETE_DATA_LEN,                                           // complete_len:    complete length of frame
    RCMM32_STOP_BIT,                                                    // stop_bit:        flag: frame has stop bit
    RCMM32_LSB,                                                         // lsb_first:       flag: LSB first
    RCMM32_FLAGS                                                        // flags:           some flags
};

#endif

#if IRMP_SUPPORT_RADIO1_PROTOCOL == 1

static const PROGMEM IRMP_PARAMETER radio1_param =
{
    IRMP_RADIO1_PROTOCOL,                                               // protocol:        ir protocol

    RADIO1_1_PULSE_LEN_MIN,                                             // pulse_1_len_min: minimum length of pulse with bit value 1
    RADIO1_1_PULSE_LEN_MAX,                                             // pulse_1_len_max: maximum length of pulse with bit value 1
    RADIO1_1_PAUSE_LEN_MIN,                                             // pause_1_len_min: minimum length of pause with bit value 1
    RADIO1_1_PAUSE_LEN_MAX,                                             // pause_1_len_max: maximum length of pause with bit value 1
    RADIO1_0_PULSE_LEN_MIN,                                             // pulse_0_len_min: minimum length of pulse with bit value 0
    RADIO1_0_PULSE_LEN_MAX,                                             // pulse_0_len_max: maximum length of pulse with bit value 0
    RADIO1_0_PAUSE_LEN_MIN,                                             // pause_0_len_min: minimum length of pause with bit value 0
    RADIO1_0_PAUSE_LEN_MAX,                                             // pause_0_len_max: maximum length of pause with bit value 0
    RADIO1_ADDRESS_OFFSET,                                              // address_offset:  address offset
    RADIO1_ADDRESS_OFFSET + RADIO1_ADDRESS_LEN,                         // address_end:     end of address
    RADIO1_COMMAND_OFFSET,                                              // command_offset:  command offset
    RADIO1_COMMAND_OFFSET + RADIO1_COMMAND_LEN,                         // command_end:     end of command
    RADIO1_COMPLETE_DATA_LEN,                                           // complete_len:    complete length of frame
    RADIO1_STOP_BIT,                                                    // stop_bit:        flag: frame has stop bit
    RADIO1_LSB,                                                         // lsb_first:       flag: LSB first
    RADIO1_FLAGS                                                        // flags:           some flags
};

#endif

static uint_fast8_t                              irmp_bit;                   // current bit position
static IRMP_PARAMETER                       irmp_param;

#if IRMP_SUPPORT_RC5_PROTOCOL == 1 && (IRMP_SUPPORT_FDC_PROTOCOL == 1 || IRMP_SUPPORT_RCCAR_PROTOCOL == 1)
static IRMP_PARAMETER                       irmp_param2;
#endif

static volatile uint_fast8_t                     irmp_ir_detected = FALSE;
static volatile uint_fast8_t                     irmp_protocol;
static volatile uint_fast16_t                    irmp_address;
static volatile uint_fast16_t                    irmp_command;
static volatile uint_fast16_t                    irmp_id;                    // only used for SAMSUNG protocol
static volatile uint_fast8_t                     irmp_flags;
// static volatile uint_fast8_t                  irmp_busy_flag;

/*---------------------------------------------------------------------------------------------------------------------------------------------------
 *  Initialize IRMP decoder
 *  @details  Configures IRMP input pin
 *---------------------------------------------------------------------------------------------------------------------------------------------------
 */
void
irmp_init (void)
{
   GPIO_InitTypeDef     GPIO_InitStructure;

   /* GPIOx clock enable */
   RCC_APB2PeriphClockCmd(IRMP_PORT_RCC, ENABLE);

   /* GPIO Configuration */
   GPIO_InitStructure.GPIO_Pin = IRMP_BIT;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
   GPIO_Init(IRMP_PORT, &GPIO_InitStructure);
}

/*---------------------------------------------------------------------------------------------------------------------------------------------------
 *  Get IRMP data
 *  @details  gets decoded IRMP data
 *  @param    pointer in order to store IRMP data
 *  @return    TRUE: successful, FALSE: failed
 *---------------------------------------------------------------------------------------------------------------------------------------------------
 */
uint_fast8_t
irmp_get_data (IRMP_DATA * irmp_data_p)
{
    uint_fast8_t   rtc = FALSE;

    if (irmp_ir_detected)
    {
        switch (irmp_protocol)
        {
#if IRMP_SUPPORT_SAMSUNG_PROTOCOL == 1
            case IRMP_SAMSUNG_PROTOCOL:
                if ((irmp_command >> 8) == (~irmp_command & 0x00FF))
                {
                    irmp_command &= 0xff;
                    irmp_command |= irmp_id << 8;
                    rtc = TRUE;
                }
                break;

#if IRMP_SUPPORT_SAMSUNG48_PROTOCOL == 1
            case IRMP_SAMSUNG48_PROTOCOL:
                irmp_command = (irmp_command & 0x00FF) | ((irmp_id & 0x00FF) << 8);
                rtc = TRUE;
                break;
#endif
#endif

#if IRMP_SUPPORT_NEC_PROTOCOL == 1
            case IRMP_NEC_PROTOCOL:
                if ((irmp_command >> 8) == (~irmp_command & 0x00FF))
                {
                    irmp_command &= 0xff;
                    rtc = TRUE;
                }
                else if (irmp_address == 0x87EE)
                {
                    irmp_protocol = IRMP_APPLE_PROTOCOL;
                    irmp_address = (irmp_command & 0xFF00) >> 8;
                    irmp_command &= 0x00FF;
                    rtc = TRUE;
                }
                break;
#endif
#if IRMP_SUPPORT_BOSE_PROTOCOL == 1
            case IRMP_BOSE_PROTOCOL:
                if ((irmp_command >> 8) == (~irmp_command & 0x00FF))
                {
                    irmp_command &= 0xff;
                    rtc = TRUE;
                }
                break;
#endif
#if IRMP_SUPPORT_SIEMENS_OR_RUWIDO_PROTOCOL == 1
            case IRMP_SIEMENS_PROTOCOL:
            case IRMP_RUWIDO_PROTOCOL:
                if (((irmp_command >> 1) & 0x0001) == (~irmp_command & 0x0001))
                {
                    irmp_command >>= 1;
                    rtc = TRUE;
                }
                break;
#endif
#if IRMP_SUPPORT_KATHREIN_PROTOCOL == 1
            case IRMP_KATHREIN_PROTOCOL:
                if (irmp_command != 0x0000)
                {
                    rtc = TRUE;
                }
                break;
#endif
#if IRMP_SUPPORT_RC5_PROTOCOL == 1
            case IRMP_RC5_PROTOCOL:
                irmp_address &= ~0x20;                              // clear toggle bit
                rtc = TRUE;
                break;
#endif
#if IRMP_SUPPORT_IR60_PROTOCOL == 1
            case IRMP_IR60_PROTOCOL:
                if (irmp_command != 0x007d)                         // 0x007d (== 62<<1 + 1) is start instruction frame
                {
                    rtc = TRUE;
                }
                break;
#endif
#if IRMP_SUPPORT_RCCAR_PROTOCOL == 1
            case IRMP_RCCAR_PROTOCOL:
                // frame in irmp_data:
                // Bit 12 11 10 9  8  7  6  5  4  3  2  1  0
                //     V  D7 D6 D5 D4 D3 D2 D1 D0 A1 A0 C1 C0   //         10 9  8  7  6  5  4  3  2  1  0
                irmp_address = (irmp_command & 0x000C) >> 2;    // addr:   0  0  0  0  0  0  0  0  0  A1 A0
                irmp_command = ((irmp_command & 0x1000) >> 2) | // V-Bit:  V  0  0  0  0  0  0  0  0  0  0
                               ((irmp_command & 0x0003) << 8) | // C-Bits: 0  C1 C0 0  0  0  0  0  0  0  0
                               ((irmp_command & 0x0FF0) >> 4);  // D-Bits:          D7 D6 D5 D4 D3 D2 D1 D0
                rtc = TRUE;                                     // Summe:  V  C1 C0 D7 D6 D5 D4 D3 D2 D1 D0
                break;
#endif

#if IRMP_SUPPORT_NETBOX_PROTOCOL == 1                           // squeeze code to 8 bit, upper bit indicates release-key
            case IRMP_NETBOX_PROTOCOL:
                if (irmp_command & 0x1000)                      // last bit set?
                {
                    if ((irmp_command & 0x1f) == 0x15)          // key pressed: 101 01 (LSB)
                    {
                        irmp_command >>= 5;
                        irmp_command &= 0x7F;
                        rtc = TRUE;
                    }
                    else if ((irmp_command & 0x1f) == 0x10)     // key released: 000 01 (LSB)
                    {
                        irmp_command >>= 5;
                        irmp_command |= 0x80;
                        rtc = TRUE;
                    }
                }
                break;
#endif
#if IRMP_SUPPORT_LEGO_PROTOCOL == 1
            case IRMP_LEGO_PROTOCOL:
            {
                uint_fast8_t crc = 0x0F ^ ((irmp_command & 0xF000) >> 12) ^ ((irmp_command & 0x0F00) >> 8) ^ ((irmp_command & 0x00F0) >> 4);

                if ((irmp_command & 0x000F) == crc)
                {
                    irmp_command >>= 4;
                    rtc = TRUE;
                }
                else
                {
                    // rtc = TRUE;                              // don't accept codes with CRC errors
                }
                break;
            }
#endif

            default:
            {
                rtc = TRUE;
                break;
            }
        }

        if (rtc)
        {
            irmp_data_p->protocol = irmp_protocol;
            irmp_data_p->address = irmp_address;
            irmp_data_p->command = irmp_command;
            irmp_data_p->flags   = irmp_flags;
            irmp_command = 0;
            irmp_address = 0;
            irmp_flags   = 0;
        }

        irmp_ir_detected = FALSE;
    }

    return rtc;
}


// these statics must not be volatile, because they are only used by irmp_store_bit(), which is called by irmp_ISR()
static uint_fast16_t irmp_tmp_address;                                      // ir address
static uint_fast16_t irmp_tmp_command;                                      // ir command

#if (IRMP_SUPPORT_RC5_PROTOCOL == 1 && (IRMP_SUPPORT_FDC_PROTOCOL == 1 || IRMP_SUPPORT_RCCAR_PROTOCOL == 1)) || IRMP_SUPPORT_NEC42_PROTOCOL == 1
static uint_fast16_t irmp_tmp_address2;                                     // ir address
static uint_fast16_t irmp_tmp_command2;                                     // ir command
#endif

#if IRMP_SUPPORT_LGAIR_PROTOCOL == 1
static uint_fast16_t irmp_lgair_address;                                    // ir address
static uint_fast16_t irmp_lgair_command;                                    // ir command
#endif

#if IRMP_SUPPORT_SAMSUNG_PROTOCOL == 1
static uint_fast16_t irmp_tmp_id;                                           // ir id (only SAMSUNG)
#endif
#if IRMP_SUPPORT_KASEIKYO_PROTOCOL == 1
static uint8_t      xor_check[6];                                           // check kaseikyo "parity" bits
static uint_fast8_t genre2;                                                 // save genre2 bits here, later copied to MSB in flags
#endif

#if IRMP_SUPPORT_ORTEK_PROTOCOL == 1
static uint_fast8_t  parity;                                                // number of '1' of the first 14 bits, check if even.
#endif

/*---------------------------------------------------------------------------------------------------------------------------------------------------
 *  store bit
 *  @details  store bit in temp address or temp command
 *  @param    value to store: 0 or 1
 *---------------------------------------------------------------------------------------------------------------------------------------------------
 */
// verhindert, dass irmp_store_bit() inline compiliert wird:
// static void irmp_store_bit (uint_fast8_t) __attribute__ ((noinline));

static void
irmp_store_bit (uint_fast8_t value)
{
#if IRMP_SUPPORT_ORTEK_PROTOCOL == 1
    if (irmp_param.protocol == IRMP_ORTEK_PROTOCOL)
    {
        if (irmp_bit < 14)
        {
            if (value)
            {
                parity++;
            }
        }
        else if (irmp_bit == 14)
        {
            if (value)                                                                                      // value == 1: even parity
            {
                if (parity & 0x01)
                {
                    parity = PARITY_CHECK_FAILED;
                }
                else
                {
                    parity = PARITY_CHECK_OK;
                }
            }
            else
            {
                if (parity & 0x01)                                                                          // value == 0: odd parity
                {
                    parity = PARITY_CHECK_OK;
                }
                else
                {
                    parity = PARITY_CHECK_FAILED;
                }
            }
        }
    }
#endif

#if IRMP_SUPPORT_GRUNDIG_NOKIA_IR60_PROTOCOL == 1
    if (irmp_bit == 0 && irmp_param.protocol == IRMP_GRUNDIG_PROTOCOL)
    {
        first_bit = value;
    }
    else
#endif

    if (irmp_bit >= irmp_param.address_offset && irmp_bit < irmp_param.address_end)
    {
        if (irmp_param.lsb_first)
        {
            irmp_tmp_address |= (((uint_fast16_t) (value)) << (irmp_bit - irmp_param.address_offset));   // CV wants cast
        }
        else
        {
            irmp_tmp_address <<= 1;
            irmp_tmp_address |= value;
        }
    }
    else if (irmp_bit >= irmp_param.command_offset && irmp_bit < irmp_param.command_end)
    {
        if (irmp_param.lsb_first)
        {
#if IRMP_SUPPORT_SAMSUNG48_PROTOCOL == 1
            if (irmp_param.protocol == IRMP_SAMSUNG48_PROTOCOL && irmp_bit >= 32)
            {
                irmp_tmp_id |= (((uint_fast16_t) (value)) << (irmp_bit - 32));   // CV wants cast
            }
            else
#endif
            {
                irmp_tmp_command |= (((uint_fast16_t) (value)) << (irmp_bit - irmp_param.command_offset));   // CV wants cast
            }
        }
        else
        {
            irmp_tmp_command <<= 1;
            irmp_tmp_command |= value;
        }
    }

#if IRMP_SUPPORT_LGAIR_PROTOCOL == 1
    if (irmp_param.protocol == IRMP_NEC_PROTOCOL || irmp_param.protocol == IRMP_NEC42_PROTOCOL)
    {
        if (irmp_bit < 8)
        {
            irmp_lgair_address <<= 1;                                                               // LGAIR uses MSB
            irmp_lgair_address |= value;
        }
        else if (irmp_bit < 24)
        {
            irmp_lgair_command <<= 1;                                                               // LGAIR uses MSB
            irmp_lgair_command |= value;
        }
    }
    // NO else!
#endif

#if IRMP_SUPPORT_NEC42_PROTOCOL == 1
    if (irmp_param.protocol == IRMP_NEC42_PROTOCOL && irmp_bit >= 13 && irmp_bit < 26)
    {
        irmp_tmp_address2 |= (((uint_fast16_t) (value)) << (irmp_bit - 13));                             // CV wants cast
    }
    else
#endif

#if IRMP_SUPPORT_SAMSUNG_PROTOCOL == 1
    if (irmp_param.protocol == IRMP_SAMSUNG_PROTOCOL && irmp_bit >= SAMSUNG_ID_OFFSET && irmp_bit < SAMSUNG_ID_OFFSET + SAMSUNG_ID_LEN)
    {
        irmp_tmp_id |= (((uint_fast16_t) (value)) << (irmp_bit - SAMSUNG_ID_OFFSET));                    // store with LSB first
    }
    else
#endif

#if IRMP_SUPPORT_KASEIKYO_PROTOCOL == 1
    if (irmp_param.protocol == IRMP_KASEIKYO_PROTOCOL)
    {
        if (irmp_bit >= 20 && irmp_bit < 24)
        {
            irmp_tmp_command |= (((uint_fast16_t) (value)) << (irmp_bit - 8));       // store 4 system bits (genre 1) in upper nibble with LSB first
        }
        else if (irmp_bit >= 24 && irmp_bit < 28)
        {
            genre2 |= (((uint_fast8_t) (value)) << (irmp_bit - 20));                 // store 4 system bits (genre 2) in upper nibble with LSB first
        }

        if (irmp_bit < KASEIKYO_COMPLETE_DATA_LEN)
        {
            if (value)
            {
                xor_check[irmp_bit / 8] |= 1 << (irmp_bit % 8);
            }
            else
            {
                xor_check[irmp_bit / 8] &= ~(1 << (irmp_bit % 8));
            }
        }
    }
    else
#endif
    {
        ;
    }

    irmp_bit++;
}

/*---------------------------------------------------------------------------------------------------------------------------------------------------
 *  store bit
 *  @details  store bit in temp address or temp command
 *  @param    value to store: 0 or 1
 *---------------------------------------------------------------------------------------------------------------------------------------------------
 */
#if IRMP_SUPPORT_RC5_PROTOCOL == 1 && (IRMP_SUPPORT_FDC_PROTOCOL == 1 || IRMP_SUPPORT_RCCAR_PROTOCOL == 1)
static void
irmp_store_bit2 (uint_fast8_t value)
{
    uint_fast8_t irmp_bit2;

    if (irmp_param.protocol)
    {
        irmp_bit2 = irmp_bit - 2;
    }
    else
    {
        irmp_bit2 = irmp_bit - 1;
    }

    if (irmp_bit2 >= irmp_param2.address_offset && irmp_bit2 < irmp_param2.address_end)
    {
        irmp_tmp_address2 |= (((uint_fast16_t) (value)) << (irmp_bit2 - irmp_param2.address_offset));   // CV wants cast
    }
    else if (irmp_bit2 >= irmp_param2.command_offset && irmp_bit2 < irmp_param2.command_end)
    {
        irmp_tmp_command2 |= (((uint_fast16_t) (value)) << (irmp_bit2 - irmp_param2.command_offset));   // CV wants cast
    }
}
#endif // IRMP_SUPPORT_RC5_PROTOCOL == 1 && (IRMP_SUPPORT_FDC_PROTOCOL == 1 || IRMP_SUPPORT_RCCAR_PROTOCOL == 1)

/*---------------------------------------------------------------------------------------------------------------------------------------------------
 *  ISR routine
 *  @details  ISR routine, called 10000 times per second
 *---------------------------------------------------------------------------------------------------------------------------------------------------
 */
uint_fast8_t
irmp_ISR (void)
{
    static uint_fast8_t     irmp_start_bit_detected;                                // flag: start bit detected
    static uint_fast8_t     wait_for_space;                                         // flag: wait for data bit space
    static uint_fast8_t     wait_for_start_space;                                   // flag: wait for start bit space
    static uint_fast8_t     irmp_pulse_time;                                        // count bit time for pulse
    static PAUSE_LEN        irmp_pause_time;                                        // count bit time for pause
    static uint_fast16_t    last_irmp_address = 0xFFFF;                             // save last irmp address to recognize key repetition
    static uint_fast16_t    last_irmp_command = 0xFFFF;                             // save last irmp command to recognize key repetition
    static uint_fast16_t    key_repetition_len;                                     // SIRCS repeats frame 2-5 times with 45 ms pause
    static uint_fast8_t     repetition_frame_number;
#if IRMP_SUPPORT_DENON_PROTOCOL == 1
    static uint_fast16_t    last_irmp_denon_command;                                // save last irmp command to recognize DENON frame repetition
    static uint_fast16_t    denon_repetition_len = 0xFFFF;                          // denon repetition len of 2nd auto generated frame
#endif
#if IRMP_SUPPORT_RC5_PROTOCOL == 1
    static uint_fast8_t     rc5_cmd_bit6;                                           // bit 6 of RC5 command is the inverted 2nd start bit
#endif
#if IRMP_SUPPORT_MANCHESTER == 1
    static PAUSE_LEN        last_pause;                                             // last pause value
#endif
#if IRMP_SUPPORT_MANCHESTER == 1 || IRMP_SUPPORT_BANG_OLUFSEN_PROTOCOL == 1
    static uint_fast8_t     last_value;                                             // last bit value
#endif
    uint_fast8_t            irmp_input;                                             // input value

    irmp_input = input(IRMP_PIN);

#if IRMP_USE_CALLBACK == 1
    if (irmp_callback_ptr)
    {
        static uint_fast8_t last_inverted_input;

        if (last_inverted_input != !irmp_input)
        {
            (*irmp_callback_ptr) (! irmp_input);
            last_inverted_input = !irmp_input;
        }
    }
#endif // IRMP_USE_CALLBACK == 1

    //irmp_log(irmp_input);                                                       // log ir signal, if IRMP_LOGGING defined

    if (! irmp_ir_detected)                                                     // ir code already detected?
    {                                                                           // no...
        if (! irmp_start_bit_detected)                                          // start bit detected?
        {                                                                       // no...
            if (! irmp_input)                                                   // receiving burst?
            {                                                                   // yes...
//              irmp_busy_flag = TRUE;
                irmp_pulse_time++;                                              // increment counter
            }
            else
            {                                                                   // no...
                if (irmp_pulse_time)                                            // it's dark....
                {                                                               // set flags for counting the time of darkness...
                    irmp_start_bit_detected = 1;
                    wait_for_start_space    = 1;
                    wait_for_space          = 0;
                    irmp_tmp_command        = 0;
                    irmp_tmp_address        = 0;
#if IRMP_SUPPORT_KASEIKYO_PROTOCOL == 1
                    genre2                  = 0;
#endif
#if IRMP_SUPPORT_SAMSUNG_PROTOCOL == 1
                    irmp_tmp_id = 0;
#endif

#if IRMP_SUPPORT_RC5_PROTOCOL == 1 && (IRMP_SUPPORT_FDC_PROTOCOL == 1 || IRMP_SUPPORT_RCCAR_PROTOCOL == 1) || IRMP_SUPPORT_NEC42_PROTOCOL == 1
                    irmp_tmp_command2       = 0;
                    irmp_tmp_address2       = 0;
#endif
#if IRMP_SUPPORT_LGAIR_PROTOCOL == 1
                    irmp_lgair_command      = 0;
                    irmp_lgair_address      = 0;
#endif
                    irmp_bit                = 0xff;
                    irmp_pause_time         = 1;                                // 1st pause: set to 1, not to 0!
#if IRMP_SUPPORT_RC5_PROTOCOL == 1
                    rc5_cmd_bit6            = 0;                                // fm 2010-03-07: bugfix: reset it after incomplete RC5 frame!
#endif
                }
                else
                {
                    if (key_repetition_len < 0xFFFF)                            // avoid overflow of counter
                    {
                        key_repetition_len++;

#if IRMP_SUPPORT_DENON_PROTOCOL == 1
                        if (denon_repetition_len < 0xFFFF)                      // avoid overflow of counter
                        {
                            denon_repetition_len++;

                            if (denon_repetition_len >= DENON_AUTO_REPETITION_PAUSE_LEN && last_irmp_denon_command != 0)
                            {
                                last_irmp_denon_command = 0;
                                denon_repetition_len = 0xFFFF;
                            }
                        }
#endif // IRMP_SUPPORT_DENON_PROTOCOL == 1
                    }
                }
            }
        }
        else
        {
            if (wait_for_start_space)                                           // we have received start bit...
            {                                                                   // ...and are counting the time of darkness
                if (irmp_input)                                                 // still dark?
                {                                                               // yes
                    irmp_pause_time++;                                          // increment counter

#if IRMP_SUPPORT_NIKON_PROTOCOL == 1
                    if (((irmp_pulse_time < NIKON_START_BIT_PULSE_LEN_MIN || irmp_pulse_time > NIKON_START_BIT_PULSE_LEN_MAX) && irmp_pause_time > IRMP_TIMEOUT_LEN) ||
                         irmp_pause_time > IRMP_TIMEOUT_NIKON_LEN)
#else
                    if (irmp_pause_time > IRMP_TIMEOUT_LEN)                     // timeout?
#endif
                    {                                                           // yes...
#if IRMP_SUPPORT_JVC_PROTOCOL == 1
                        if (irmp_protocol == IRMP_JVC_PROTOCOL)                 // don't show eror if JVC protocol, irmp_pulse_time has been set below!
                        {
                            ;
                        }
                        else
#endif // IRMP_SUPPORT_JVC_PROTOCOL == 1
                        {
                        }

                        irmp_start_bit_detected = 0;                            // reset flags, let's wait for another start bit
                        irmp_pulse_time         = 0;
                        irmp_pause_time         = 0;
                    }
                }
                else
                {                                                               // receiving first data pulse!
                    IRMP_PARAMETER * irmp_param_p;
                    irmp_param_p = (IRMP_PARAMETER *) 0;

#if IRMP_SUPPORT_RC5_PROTOCOL == 1 && (IRMP_SUPPORT_FDC_PROTOCOL == 1 || IRMP_SUPPORT_RCCAR_PROTOCOL == 1)
                    irmp_param2.protocol = 0;
#endif

#if IRMP_SUPPORT_SIRCS_PROTOCOL == 1
                    if (irmp_pulse_time >= SIRCS_START_BIT_PULSE_LEN_MIN && irmp_pulse_time <= SIRCS_START_BIT_PULSE_LEN_MAX &&
                        irmp_pause_time >= SIRCS_START_BIT_PAUSE_LEN_MIN && irmp_pause_time <= SIRCS_START_BIT_PAUSE_LEN_MAX)
                    {                                                           // it's SIRCS
                        irmp_param_p = (IRMP_PARAMETER *) &sircs_param;
                    }
                    else
#endif // IRMP_SUPPORT_SIRCS_PROTOCOL == 1

#if IRMP_SUPPORT_JVC_PROTOCOL == 1
                    if (irmp_protocol == IRMP_JVC_PROTOCOL &&                                                       // last protocol was JVC, awaiting repeat frame
                        irmp_pulse_time >= JVC_START_BIT_PULSE_LEN_MIN && irmp_pulse_time <= JVC_START_BIT_PULSE_LEN_MAX &&
                        irmp_pause_time >= JVC_REPEAT_START_BIT_PAUSE_LEN_MIN && irmp_pause_time <= JVC_REPEAT_START_BIT_PAUSE_LEN_MAX)
                    {
                        irmp_param_p = (IRMP_PARAMETER *) &nec_param;
                    }
                    else
#endif // IRMP_SUPPORT_JVC_PROTOCOL == 1

#if IRMP_SUPPORT_NEC_PROTOCOL == 1
                    if (irmp_pulse_time >= NEC_START_BIT_PULSE_LEN_MIN && irmp_pulse_time <= NEC_START_BIT_PULSE_LEN_MAX &&
                        irmp_pause_time >= NEC_START_BIT_PAUSE_LEN_MIN && irmp_pause_time <= NEC_START_BIT_PAUSE_LEN_MAX)
                    {
#if IRMP_SUPPORT_NEC42_PROTOCOL == 1
                        irmp_param_p = (IRMP_PARAMETER *) &nec42_param;
#else
                        irmp_param_p = (IRMP_PARAMETER *) &nec_param;
#endif
                    }
                    else if (irmp_pulse_time >= NEC_START_BIT_PULSE_LEN_MIN        && irmp_pulse_time <= NEC_START_BIT_PULSE_LEN_MAX &&
                             irmp_pause_time >= NEC_REPEAT_START_BIT_PAUSE_LEN_MIN && irmp_pause_time <= NEC_REPEAT_START_BIT_PAUSE_LEN_MAX)
                    {                                                           // it's NEC
#if IRMP_SUPPORT_JVC_PROTOCOL == 1
                        if (irmp_protocol == IRMP_JVC_PROTOCOL)                 // last protocol was JVC, awaiting repeat frame
                        {                                                       // some jvc remote controls use nec repetition frame for jvc repetition frame
                            irmp_param_p = (IRMP_PARAMETER *) &nec_param;
                        }
                        else
#endif // IRMP_SUPPORT_JVC_PROTOCOL == 1
                        {
                            irmp_param_p = (IRMP_PARAMETER *) &nec_rep_param;
                        }
                    }
                    else

#if IRMP_SUPPORT_JVC_PROTOCOL == 1
                    if (irmp_protocol == IRMP_JVC_PROTOCOL &&                   // last protocol was JVC, awaiting repeat frame
                        irmp_pulse_time >= NEC_START_BIT_PULSE_LEN_MIN && irmp_pulse_time <= NEC_START_BIT_PULSE_LEN_MAX &&
                        irmp_pause_time >= NEC_0_PAUSE_LEN_MIN         && irmp_pause_time <= NEC_0_PAUSE_LEN_MAX)
                    {                                                           // it's JVC repetition type 3
                        irmp_param_p = (IRMP_PARAMETER *) &nec_param;
                    }
                    else
#endif // IRMP_SUPPORT_JVC_PROTOCOL == 1

#endif // IRMP_SUPPORT_NEC_PROTOCOL == 1

#if IRMP_SUPPORT_TELEFUNKEN_PROTOCOL == 1
                    if (irmp_pulse_time >= TELEFUNKEN_START_BIT_PULSE_LEN_MIN && irmp_pulse_time <= TELEFUNKEN_START_BIT_PULSE_LEN_MAX &&
                        irmp_pause_time >= TELEFUNKEN_START_BIT_PAUSE_LEN_MIN && irmp_pause_time <= TELEFUNKEN_START_BIT_PAUSE_LEN_MAX)
                    {
                        irmp_param_p = (IRMP_PARAMETER *) &telefunken_param;
                    }
                    else
#endif // IRMP_SUPPORT_TELEFUNKEN_PROTOCOL == 1

#if IRMP_SUPPORT_ROOMBA_PROTOCOL == 1
                    if (irmp_pulse_time >= ROOMBA_START_BIT_PULSE_LEN_MIN && irmp_pulse_time <= ROOMBA_START_BIT_PULSE_LEN_MAX &&
                        irmp_pause_time >= ROOMBA_START_BIT_PAUSE_LEN_MIN && irmp_pause_time <= ROOMBA_START_BIT_PAUSE_LEN_MAX)
                    {
                        irmp_param_p = (IRMP_PARAMETER *) &roomba_param;
                    }
                    else
#endif // IRMP_SUPPORT_ROOMBA_PROTOCOL == 1

#if IRMP_SUPPORT_NIKON_PROTOCOL == 1
                    if (irmp_pulse_time >= NIKON_START_BIT_PULSE_LEN_MIN && irmp_pulse_time <= NIKON_START_BIT_PULSE_LEN_MAX &&
                        irmp_pause_time >= NIKON_START_BIT_PAUSE_LEN_MIN && irmp_pause_time <= NIKON_START_BIT_PAUSE_LEN_MAX)
                    {
                        irmp_param_p = (IRMP_PARAMETER *) &nikon_param;
                    }
                    else
#endif // IRMP_SUPPORT_NIKON_PROTOCOL == 1

#if IRMP_SUPPORT_SAMSUNG_PROTOCOL == 1
                    if (irmp_pulse_time >= SAMSUNG_START_BIT_PULSE_LEN_MIN && irmp_pulse_time <= SAMSUNG_START_BIT_PULSE_LEN_MAX &&
                        irmp_pause_time >= SAMSUNG_START_BIT_PAUSE_LEN_MIN && irmp_pause_time <= SAMSUNG_START_BIT_PAUSE_LEN_MAX)
                    {                                                           // it's SAMSUNG
                        irmp_param_p = (IRMP_PARAMETER *) &samsung_param;
                    }
                    else
#endif // IRMP_SUPPORT_SAMSUNG_PROTOCOL == 1

#if IRMP_SUPPORT_MATSUSHITA_PROTOCOL == 1
                    if (irmp_pulse_time >= MATSUSHITA_START_BIT_PULSE_LEN_MIN && irmp_pulse_time <= MATSUSHITA_START_BIT_PULSE_LEN_MAX &&
                        irmp_pause_time >= MATSUSHITA_START_BIT_PAUSE_LEN_MIN && irmp_pause_time <= MATSUSHITA_START_BIT_PAUSE_LEN_MAX)
                    {                                                           // it's MATSUSHITA
                        irmp_param_p = (IRMP_PARAMETER *) &matsushita_param;
                    }
                    else
#endif // IRMP_SUPPORT_MATSUSHITA_PROTOCOL == 1

#if IRMP_SUPPORT_KASEIKYO_PROTOCOL == 1
                    if (irmp_pulse_time >= KASEIKYO_START_BIT_PULSE_LEN_MIN && irmp_pulse_time <= KASEIKYO_START_BIT_PULSE_LEN_MAX &&
                        irmp_pause_time >= KASEIKYO_START_BIT_PAUSE_LEN_MIN && irmp_pause_time <= KASEIKYO_START_BIT_PAUSE_LEN_MAX)
                    {                                                           // it's KASEIKYO
                        irmp_param_p = (IRMP_PARAMETER *) &kaseikyo_param;
                    }
                    else
#endif // IRMP_SUPPORT_KASEIKYO_PROTOCOL == 1

#if IRMP_SUPPORT_RADIO1_PROTOCOL == 1
                    if (irmp_pulse_time >= RADIO1_START_BIT_PULSE_LEN_MIN && irmp_pulse_time <= RADIO1_START_BIT_PULSE_LEN_MAX &&
                        irmp_pause_time >= RADIO1_START_BIT_PAUSE_LEN_MIN && irmp_pause_time <= RADIO1_START_BIT_PAUSE_LEN_MAX)
                    {
                        irmp_param_p = (IRMP_PARAMETER *) &radio1_param;
                    }
                    else
#endif // IRMP_SUPPORT_RRADIO1_PROTOCOL == 1

#if IRMP_SUPPORT_RECS80_PROTOCOL == 1
                    if (irmp_pulse_time >= RECS80_START_BIT_PULSE_LEN_MIN && irmp_pulse_time <= RECS80_START_BIT_PULSE_LEN_MAX &&
                        irmp_pause_time >= RECS80_START_BIT_PAUSE_LEN_MIN && irmp_pause_time <= RECS80_START_BIT_PAUSE_LEN_MAX)
                    {                                                           // it's RECS80
                        irmp_param_p = (IRMP_PARAMETER *) &recs80_param;
                    }
                    else
#endif // IRMP_SUPPORT_RECS80_PROTOCOL == 1

#if IRMP_SUPPORT_RC5_PROTOCOL == 1
                    if (((irmp_pulse_time >= RC5_START_BIT_LEN_MIN     && irmp_pulse_time <= RC5_START_BIT_LEN_MAX) ||
                         (irmp_pulse_time >= 2 * RC5_START_BIT_LEN_MIN && irmp_pulse_time <= 2 * RC5_START_BIT_LEN_MAX)) &&
                        ((irmp_pause_time >= RC5_START_BIT_LEN_MIN     && irmp_pause_time <= RC5_START_BIT_LEN_MAX) ||
                         (irmp_pause_time >= 2 * RC5_START_BIT_LEN_MIN && irmp_pause_time <= 2 * RC5_START_BIT_LEN_MAX)))
                    {                                                           // it's RC5
#if IRMP_SUPPORT_FDC_PROTOCOL == 1
                        if (irmp_pulse_time >= FDC_START_BIT_PULSE_LEN_MIN && irmp_pulse_time <= FDC_START_BIT_PULSE_LEN_MAX &&
                            irmp_pause_time >= FDC_START_BIT_PAUSE_LEN_MIN && irmp_pause_time <= FDC_START_BIT_PAUSE_LEN_MAX)
                        {
                            memcpy_P (&irmp_param2, &fdc_param, sizeof (IRMP_PARAMETER));
                        }
                        else
#endif // IRMP_SUPPORT_FDC_PROTOCOL == 1

#if IRMP_SUPPORT_RCCAR_PROTOCOL == 1
                        if (irmp_pulse_time >= RCCAR_START_BIT_PULSE_LEN_MIN && irmp_pulse_time <= RCCAR_START_BIT_PULSE_LEN_MAX &&
                            irmp_pause_time >= RCCAR_START_BIT_PAUSE_LEN_MIN && irmp_pause_time <= RCCAR_START_BIT_PAUSE_LEN_MAX)
                        {
                            memcpy_P (&irmp_param2, &rccar_param, sizeof (IRMP_PARAMETER));
                        }
                        else
#endif // IRMP_SUPPORT_RCCAR_PROTOCOL == 1
                        {
                        }

                        irmp_param_p = (IRMP_PARAMETER *) &rc5_param;
                        last_pause = irmp_pause_time;

                        if ((irmp_pulse_time > RC5_START_BIT_LEN_MAX && irmp_pulse_time <= 2 * RC5_START_BIT_LEN_MAX) ||
                            (irmp_pause_time > RC5_START_BIT_LEN_MAX && irmp_pause_time <= 2 * RC5_START_BIT_LEN_MAX))
                        {
                          last_value  = 0;
                          rc5_cmd_bit6 = 1<<6;
                        }
                        else
                        {
                          last_value  = 1;
                        }
                    }
                    else
#endif // IRMP_SUPPORT_RC5_PROTOCOL == 1

#if IRMP_SUPPORT_DENON_PROTOCOL == 1
                    if ( (irmp_pulse_time >= DENON_PULSE_LEN_MIN && irmp_pulse_time <= DENON_PULSE_LEN_MAX) &&
                        ((irmp_pause_time >= DENON_1_PAUSE_LEN_MIN && irmp_pause_time <= DENON_1_PAUSE_LEN_MAX) ||
                         (irmp_pause_time >= DENON_0_PAUSE_LEN_MIN && irmp_pause_time <= DENON_0_PAUSE_LEN_MAX)))
                    {                                                           // it's DENON
                        irmp_param_p = (IRMP_PARAMETER *) &denon_param;
                    }
                    else
#endif // IRMP_SUPPORT_DENON_PROTOCOL == 1

#if IRMP_SUPPORT_THOMSON_PROTOCOL == 1
                    if ( (irmp_pulse_time >= THOMSON_PULSE_LEN_MIN && irmp_pulse_time <= THOMSON_PULSE_LEN_MAX) &&
                        ((irmp_pause_time >= THOMSON_1_PAUSE_LEN_MIN && irmp_pause_time <= THOMSON_1_PAUSE_LEN_MAX) ||
                         (irmp_pause_time >= THOMSON_0_PAUSE_LEN_MIN && irmp_pause_time <= THOMSON_0_PAUSE_LEN_MAX)))
                    {                                                           // it's THOMSON
                        irmp_param_p = (IRMP_PARAMETER *) &thomson_param;
                    }
                    else
#endif // IRMP_SUPPORT_THOMSON_PROTOCOL == 1

#if IRMP_SUPPORT_BOSE_PROTOCOL == 1
                    if (irmp_pulse_time >= BOSE_START_BIT_PULSE_LEN_MIN && irmp_pulse_time <= BOSE_START_BIT_PULSE_LEN_MAX &&
                        irmp_pause_time >= BOSE_START_BIT_PAUSE_LEN_MIN && irmp_pause_time <= BOSE_START_BIT_PAUSE_LEN_MAX)
                    {
                        irmp_param_p = (IRMP_PARAMETER *) &bose_param;
                    }
                    else
#endif // IRMP_SUPPORT_BOSE_PROTOCOL == 1

#if IRMP_SUPPORT_RC6_PROTOCOL == 1
                    if (irmp_pulse_time >= RC6_START_BIT_PULSE_LEN_MIN && irmp_pulse_time <= RC6_START_BIT_PULSE_LEN_MAX &&
                        irmp_pause_time >= RC6_START_BIT_PAUSE_LEN_MIN && irmp_pause_time <= RC6_START_BIT_PAUSE_LEN_MAX)
                    {                                                           // it's RC6
                        irmp_param_p = (IRMP_PARAMETER *) &rc6_param;
                        last_pause = 0;
                        last_value = 1;
                    }
                    else
#endif // IRMP_SUPPORT_RC6_PROTOCOL == 1

#if IRMP_SUPPORT_RECS80EXT_PROTOCOL == 1
                    if (irmp_pulse_time >= RECS80EXT_START_BIT_PULSE_LEN_MIN && irmp_pulse_time <= RECS80EXT_START_BIT_PULSE_LEN_MAX &&
                        irmp_pause_time >= RECS80EXT_START_BIT_PAUSE_LEN_MIN && irmp_pause_time <= RECS80EXT_START_BIT_PAUSE_LEN_MAX)
                    {                                                           // it's RECS80EXT
                        irmp_param_p = (IRMP_PARAMETER *) &recs80ext_param;
                    }
                    else
#endif // IRMP_SUPPORT_RECS80EXT_PROTOCOL == 1

#if IRMP_SUPPORT_NUBERT_PROTOCOL == 1
                    if (irmp_pulse_time >= NUBERT_START_BIT_PULSE_LEN_MIN && irmp_pulse_time <= NUBERT_START_BIT_PULSE_LEN_MAX &&
                        irmp_pause_time >= NUBERT_START_BIT_PAUSE_LEN_MIN && irmp_pause_time <= NUBERT_START_BIT_PAUSE_LEN_MAX)
                    {                                                           // it's NUBERT
#ifdef ANALYZE
                        ANALYZE_PRINTF ("protocol = NUBERT, start bit timings: pulse: %3d - %3d, pause: %3d - %3d\n",
                                        NUBERT_START_BIT_PULSE_LEN_MIN, NUBERT_START_BIT_PULSE_LEN_MAX,
                                        NUBERT_START_BIT_PAUSE_LEN_MIN, NUBERT_START_BIT_PAUSE_LEN_MAX);
#endif // ANALYZE
                        irmp_param_p = (IRMP_PARAMETER *) &nubert_param;
                    }
                    else
#endif // IRMP_SUPPORT_NUBERT_PROTOCOL == 1

#if IRMP_SUPPORT_SPEAKER_PROTOCOL == 1
                    if (irmp_pulse_time >= SPEAKER_START_BIT_PULSE_LEN_MIN && irmp_pulse_time <= SPEAKER_START_BIT_PULSE_LEN_MAX &&
                        irmp_pause_time >= SPEAKER_START_BIT_PAUSE_LEN_MIN && irmp_pause_time <= SPEAKER_START_BIT_PAUSE_LEN_MAX)
                    {                                                           // it's SPEAKER
                        irmp_param_p = (IRMP_PARAMETER *) &speaker_param;
                    }
                    else
#endif // IRMP_SUPPORT_SPEAKER_PROTOCOL == 1

#if IRMP_SUPPORT_BANG_OLUFSEN_PROTOCOL == 1
                    if (irmp_pulse_time >= BANG_OLUFSEN_START_BIT1_PULSE_LEN_MIN && irmp_pulse_time <= BANG_OLUFSEN_START_BIT1_PULSE_LEN_MAX &&
                        irmp_pause_time >= BANG_OLUFSEN_START_BIT1_PAUSE_LEN_MIN && irmp_pause_time <= BANG_OLUFSEN_START_BIT1_PAUSE_LEN_MAX)
                    {                                                           // it's BANG_OLUFSEN
                        irmp_param_p = (IRMP_PARAMETER *) &bang_olufsen_param;
                        last_value = 0;
                    }
                    else
#endif // IRMP_SUPPORT_BANG_OLUFSEN_PROTOCOL == 1

#if IRMP_SUPPORT_GRUNDIG_NOKIA_IR60_PROTOCOL == 1
                    if (irmp_pulse_time >= GRUNDIG_NOKIA_IR60_START_BIT_LEN_MIN && irmp_pulse_time <= GRUNDIG_NOKIA_IR60_START_BIT_LEN_MAX &&
                        irmp_pause_time >= GRUNDIG_NOKIA_IR60_PRE_PAUSE_LEN_MIN && irmp_pause_time <= GRUNDIG_NOKIA_IR60_PRE_PAUSE_LEN_MAX)
                    {                                                           // it's GRUNDIG
                        irmp_param_p = (IRMP_PARAMETER *) &grundig_param;
                        last_pause = irmp_pause_time;
                        last_value  = 1;
                    }
                    else
#endif // IRMP_SUPPORT_GRUNDIG_NOKIA_IR60_PROTOCOL == 1

#if IRMP_SUPPORT_SIEMENS_OR_RUWIDO_PROTOCOL == 1
                    if (((irmp_pulse_time >= SIEMENS_OR_RUWIDO_START_BIT_PULSE_LEN_MIN && irmp_pulse_time <= SIEMENS_OR_RUWIDO_START_BIT_PULSE_LEN_MAX) ||
                         (irmp_pulse_time >= 2 * SIEMENS_OR_RUWIDO_START_BIT_PULSE_LEN_MIN && irmp_pulse_time <= 2 * SIEMENS_OR_RUWIDO_START_BIT_PULSE_LEN_MAX)) &&
                        ((irmp_pause_time >= SIEMENS_OR_RUWIDO_START_BIT_PAUSE_LEN_MIN && irmp_pause_time <= SIEMENS_OR_RUWIDO_START_BIT_PAUSE_LEN_MAX) ||
                         (irmp_pause_time >= 2 * SIEMENS_OR_RUWIDO_START_BIT_PAUSE_LEN_MIN && irmp_pause_time <= 2 * SIEMENS_OR_RUWIDO_START_BIT_PAUSE_LEN_MAX)))
                    {                                                           // it's RUWIDO or SIEMENS
                        irmp_param_p = (IRMP_PARAMETER *) &ruwido_param;
                        last_pause = irmp_pause_time;
                        last_value  = 1;
                    }
                    else
#endif // IRMP_SUPPORT_SIEMENS_OR_RUWIDO_PROTOCOL == 1

#if IRMP_SUPPORT_FDC_PROTOCOL == 1
                    if (irmp_pulse_time >= FDC_START_BIT_PULSE_LEN_MIN && irmp_pulse_time <= FDC_START_BIT_PULSE_LEN_MAX &&
                        irmp_pause_time >= FDC_START_BIT_PAUSE_LEN_MIN && irmp_pause_time <= FDC_START_BIT_PAUSE_LEN_MAX)
                    {
                        irmp_param_p = (IRMP_PARAMETER *) &fdc_param;
                    }
                    else
#endif // IRMP_SUPPORT_FDC_PROTOCOL == 1

#if IRMP_SUPPORT_RCCAR_PROTOCOL == 1
                    if (irmp_pulse_time >= RCCAR_START_BIT_PULSE_LEN_MIN && irmp_pulse_time <= RCCAR_START_BIT_PULSE_LEN_MAX &&
                        irmp_pause_time >= RCCAR_START_BIT_PAUSE_LEN_MIN && irmp_pause_time <= RCCAR_START_BIT_PAUSE_LEN_MAX)
                    {
                        irmp_param_p = (IRMP_PARAMETER *) &rccar_param;
                    }
                    else
#endif // IRMP_SUPPORT_RCCAR_PROTOCOL == 1

#if IRMP_SUPPORT_KATHREIN_PROTOCOL == 1
                    if (irmp_pulse_time >= KATHREIN_START_BIT_PULSE_LEN_MIN && irmp_pulse_time <= KATHREIN_START_BIT_PULSE_LEN_MAX &&
                        irmp_pause_time >= KATHREIN_START_BIT_PAUSE_LEN_MIN && irmp_pause_time <= KATHREIN_START_BIT_PAUSE_LEN_MAX)
                    {                                                           // it's KATHREIN
                        irmp_param_p = (IRMP_PARAMETER *) &kathrein_param;
                    }
                    else
#endif // IRMP_SUPPORT_KATHREIN_PROTOCOL == 1

#if IRMP_SUPPORT_NETBOX_PROTOCOL == 1
                    if (irmp_pulse_time >= NETBOX_START_BIT_PULSE_LEN_MIN && irmp_pulse_time <= NETBOX_START_BIT_PULSE_LEN_MAX &&
                        irmp_pause_time >= NETBOX_START_BIT_PAUSE_LEN_MIN && irmp_pause_time <= NETBOX_START_BIT_PAUSE_LEN_MAX)
                    {                                                           // it's NETBOX
                        irmp_param_p = (IRMP_PARAMETER *) &netbox_param;
                    }
                    else
#endif // IRMP_SUPPORT_NETBOX_PROTOCOL == 1

#if IRMP_SUPPORT_LEGO_PROTOCOL == 1
                    if (irmp_pulse_time >= LEGO_START_BIT_PULSE_LEN_MIN && irmp_pulse_time <= LEGO_START_BIT_PULSE_LEN_MAX &&
                        irmp_pause_time >= LEGO_START_BIT_PAUSE_LEN_MIN && irmp_pause_time <= LEGO_START_BIT_PAUSE_LEN_MAX)
                    {
                        irmp_param_p = (IRMP_PARAMETER *) &lego_param;
                    }
                    else
#endif // IRMP_SUPPORT_LEGO_PROTOCOL == 1

#if IRMP_SUPPORT_A1TVBOX_PROTOCOL == 1
                    if (irmp_pulse_time >= A1TVBOX_START_BIT_PULSE_LEN_MIN && irmp_pulse_time <= A1TVBOX_START_BIT_PULSE_LEN_MAX &&
                        irmp_pause_time >= A1TVBOX_START_BIT_PAUSE_LEN_MIN && irmp_pause_time <= A1TVBOX_START_BIT_PAUSE_LEN_MAX)
                    {                                                           // it's A1TVBOX
                        irmp_param_p = (IRMP_PARAMETER *) &a1tvbox_param;
                        last_pause = 0;
                        last_value = 1;
                    }
                    else
#endif // IRMP_SUPPORT_A1TVBOX_PROTOCOL == 1

#if IRMP_SUPPORT_ORTEK_PROTOCOL == 1
                    if (irmp_pulse_time >= ORTEK_START_BIT_PULSE_LEN_MIN && irmp_pulse_time <= ORTEK_START_BIT_PULSE_LEN_MAX &&
                        irmp_pause_time >= ORTEK_START_BIT_PAUSE_LEN_MIN && irmp_pause_time <= ORTEK_START_BIT_PAUSE_LEN_MAX)
                    {                                                           // it's ORTEK (Hama)
                        irmp_param_p = (IRMP_PARAMETER *) &ortek_param;
                        last_pause  = 0;
                        last_value  = 1;
                        parity      = 0;
                    }
                    else
#endif // IRMP_SUPPORT_ORTEK_PROTOCOL == 1

#if IRMP_SUPPORT_RCMM_PROTOCOL == 1
                    if (irmp_pulse_time >= RCMM32_START_BIT_PULSE_LEN_MIN && irmp_pulse_time <= RCMM32_START_BIT_PULSE_LEN_MAX &&
                        irmp_pause_time >= RCMM32_START_BIT_PAUSE_LEN_MIN && irmp_pause_time <= RCMM32_START_BIT_PAUSE_LEN_MAX)
                    {                                                           // it's RCMM
                        irmp_param_p = (IRMP_PARAMETER *) &rcmm_param;
                    }
                    else
#endif // IRMP_SUPPORT_RCMM_PROTOCOL == 1
                    {
                        irmp_start_bit_detected = 0;                            // wait for another start bit...
                    }

                    if (irmp_start_bit_detected)
                    {
                        memcpy_P (&irmp_param, irmp_param_p, sizeof (IRMP_PARAMETER));

                        if (! (irmp_param.flags & IRMP_PARAM_FLAG_IS_MANCHESTER))
                        {
                        }
                        else
                        {
                        }

#if IRMP_SUPPORT_RC5_PROTOCOL == 1 && (IRMP_SUPPORT_FDC_PROTOCOL == 1 || IRMP_SUPPORT_RCCAR_PROTOCOL == 1)
                        if (irmp_param2.protocol)
                        {
                        }
#endif


#if IRMP_SUPPORT_RC6_PROTOCOL == 1
                        if (irmp_param.protocol == IRMP_RC6_PROTOCOL)
                        {
                        }
#endif

                        if (! (irmp_param.flags & IRMP_PARAM_FLAG_IS_MANCHESTER))
                        {
                        }
                        else
                        {
                        }
                    }

                    irmp_bit = 0;

#if IRMP_SUPPORT_MANCHESTER == 1
                    if ((irmp_param.flags & IRMP_PARAM_FLAG_IS_MANCHESTER) &&
                         irmp_param.protocol != IRMP_RUWIDO_PROTOCOL && // Manchester, but not RUWIDO
                         irmp_param.protocol != IRMP_RC6_PROTOCOL)      // Manchester, but not RC6
                    {
                        if (irmp_pause_time > irmp_param.pulse_1_len_max && irmp_pause_time <= 2 * irmp_param.pulse_1_len_max)
                        {
                            irmp_store_bit ((irmp_param.flags & IRMP_PARAM_FLAG_1ST_PULSE_IS_1) ? 0 : 1);
                        }
                        else if (! last_value)  // && irmp_pause_time >= irmp_param.pause_1_len_min && irmp_pause_time <= irmp_param.pause_1_len_max)
                        {
                            irmp_store_bit ((irmp_param.flags & IRMP_PARAM_FLAG_1ST_PULSE_IS_1) ? 1 : 0);
                        }
                    }
                    else
#endif // IRMP_SUPPORT_MANCHESTER == 1

#if IRMP_SUPPORT_SERIAL == 1
                    if (irmp_param.flags & IRMP_PARAM_FLAG_IS_SERIAL)
                    {
                        ; // do nothing
                    }
                    else
#endif // IRMP_SUPPORT_SERIAL == 1


#if IRMP_SUPPORT_DENON_PROTOCOL == 1
                    if (irmp_param.protocol == IRMP_DENON_PROTOCOL)
                    {

                        if (irmp_pause_time >= DENON_1_PAUSE_LEN_MIN && irmp_pause_time <= DENON_1_PAUSE_LEN_MAX)
                        {                                                       // pause timings correct for "1"?
                            irmp_store_bit (1);
                        }
                        else // if (irmp_pause_time >= DENON_0_PAUSE_LEN_MIN && irmp_pause_time <= DENON_0_PAUSE_LEN_MAX)
                        {                                                       // pause timings correct for "0"?
                            irmp_store_bit (0);
                        }
                    }
                    else
#endif // IRMP_SUPPORT_DENON_PROTOCOL == 1
#if IRMP_SUPPORT_THOMSON_PROTOCOL == 1
                    if (irmp_param.protocol == IRMP_THOMSON_PROTOCOL)
                    {
                        if (irmp_pause_time >= THOMSON_1_PAUSE_LEN_MIN && irmp_pause_time <= THOMSON_1_PAUSE_LEN_MAX)
                        {                                                       // pause timings correct for "1"?
                          irmp_store_bit (1);
                        }
                        else // if (irmp_pause_time >= THOMSON_0_PAUSE_LEN_MIN && irmp_pause_time <= THOMSON_0_PAUSE_LEN_MAX)
                        {                                                       // pause timings correct for "0"?
                          irmp_store_bit (0);
                        }
                    }
                    else
#endif // IRMP_SUPPORT_THOMSON_PROTOCOL == 1
                    {
                        ;                                                       // else do nothing
                    }

                    irmp_pulse_time = 1;                                        // set counter to 1, not 0
                    irmp_pause_time = 0;
                    wait_for_start_space = 0;
                }
            }
            else if (wait_for_space)                                            // the data section....
            {                                                                   // counting the time of darkness....
                uint_fast8_t got_light = FALSE;

                if (irmp_input)                                                 // still dark?
                {                                                               // yes...
                    if (irmp_bit == irmp_param.complete_len && irmp_param.stop_bit == 1)
                    {
                        if (
#if IRMP_SUPPORT_MANCHESTER == 1
                            (irmp_param.flags & IRMP_PARAM_FLAG_IS_MANCHESTER) ||
#endif
#if IRMP_SUPPORT_SERIAL == 1
                            (irmp_param.flags & IRMP_PARAM_FLAG_IS_SERIAL) ||
#endif
                            (irmp_pulse_time >= irmp_param.pulse_0_len_min && irmp_pulse_time <= irmp_param.pulse_0_len_max))
                        {
                            irmp_param.stop_bit = 0;
                        }
                        else
                        {
                            irmp_start_bit_detected = 0;                        // wait for another start bit...
                            irmp_pulse_time         = 0;
                            irmp_pause_time         = 0;
                        }
                    }
                    else
                    {
                        irmp_pause_time++;                                                          // increment counter

#if IRMP_SUPPORT_SIRCS_PROTOCOL == 1
                        if (irmp_param.protocol == IRMP_SIRCS_PROTOCOL &&                           // Sony has a variable number of bits:
                            irmp_pause_time > SIRCS_PAUSE_LEN_MAX &&                                // minimum is 12
                            irmp_bit >= 12 - 1)                                                     // pause too long?
                        {                                                                           // yes, break and close this frame
                            irmp_param.complete_len = irmp_bit + 1;                                 // set new complete length
                            got_light = TRUE;                                                       // this is a lie, but helps (generates stop bit)
                            irmp_tmp_address |= (irmp_bit - SIRCS_MINIMUM_DATA_LEN + 1) << 8;       // new: store number of additional bits in upper byte of address!
                            irmp_param.command_end = irmp_param.command_offset + irmp_bit + 1;      // correct command length
                            irmp_pause_time = SIRCS_PAUSE_LEN_MAX - 1;                              // correct pause length
                        }
                        else
#endif
#if IRMP_SUPPORT_SERIAL == 1
                        // NETBOX generates no stop bit, here is the timeout condition:
                        if ((irmp_param.flags & IRMP_PARAM_FLAG_IS_SERIAL) && irmp_param.protocol == IRMP_NETBOX_PROTOCOL &&
                            irmp_pause_time >= NETBOX_PULSE_LEN * (NETBOX_COMPLETE_DATA_LEN - irmp_bit))
                        {
                            got_light = TRUE;                                                       // this is a lie, but helps (generates stop bit)
                        }
                        else
#endif
#if IRMP_SUPPORT_GRUNDIG_NOKIA_IR60_PROTOCOL == 1
                        if (irmp_param.protocol == IRMP_GRUNDIG_PROTOCOL && !irmp_param.stop_bit)
                        {
                            if (irmp_pause_time > IR60_TIMEOUT_LEN && (irmp_bit == 5 || irmp_bit == 6))
                            {
                                got_light = TRUE;                                       // this is a lie, but generates a stop bit ;-)
                                irmp_param.stop_bit = TRUE;                             // set flag

                                irmp_param.protocol         = IRMP_IR60_PROTOCOL;       // change protocol
                                irmp_param.complete_len     = IR60_COMPLETE_DATA_LEN;   // correct complete len
                                irmp_param.address_offset   = IR60_ADDRESS_OFFSET;
                                irmp_param.address_end      = IR60_ADDRESS_OFFSET + IR60_ADDRESS_LEN;
                                irmp_param.command_offset   = IR60_COMMAND_OFFSET;
                                irmp_param.command_end      = IR60_COMMAND_OFFSET + IR60_COMMAND_LEN;

                                irmp_tmp_command <<= 1;
                                irmp_tmp_command |= first_bit;
                            }
                            else if (irmp_pause_time >= 2 * irmp_param.pause_1_len_max && irmp_bit >= GRUNDIG_COMPLETE_DATA_LEN - 2)
                            {                                                           // special manchester decoder
                                irmp_param.complete_len = GRUNDIG_COMPLETE_DATA_LEN;    // correct complete len
                                got_light = TRUE;                                       // this is a lie, but generates a stop bit ;-)
                                irmp_param.stop_bit = TRUE;                             // set flag
                            }
                            else if (irmp_bit >= GRUNDIG_COMPLETE_DATA_LEN)
                            {
                                irmp_param.protocol         = IRMP_NOKIA_PROTOCOL;      // change protocol
                                irmp_param.address_offset   = NOKIA_ADDRESS_OFFSET;
                                irmp_param.address_end      = NOKIA_ADDRESS_OFFSET + NOKIA_ADDRESS_LEN;
                                irmp_param.command_offset   = NOKIA_COMMAND_OFFSET;
                                irmp_param.command_end      = NOKIA_COMMAND_OFFSET + NOKIA_COMMAND_LEN;

                                if (irmp_tmp_command & 0x300)
                                {
                                    irmp_tmp_address = (irmp_tmp_command >> 8);
                                    irmp_tmp_command &= 0xFF;
                                }
                            }
                        }
                        else
#endif
#if IRMP_SUPPORT_SIEMENS_OR_RUWIDO_PROTOCOL == 1
                        if (irmp_param.protocol == IRMP_RUWIDO_PROTOCOL && !irmp_param.stop_bit)
                        {
                            if (irmp_pause_time >= 2 * irmp_param.pause_1_len_max && irmp_bit >= RUWIDO_COMPLETE_DATA_LEN - 2)
                            {                                                           // special manchester decoder
                                irmp_param.complete_len = RUWIDO_COMPLETE_DATA_LEN;     // correct complete len
                                got_light = TRUE;                                       // this is a lie, but generates a stop bit ;-)
                                irmp_param.stop_bit = TRUE;                             // set flag
                            }
                            else if (irmp_bit >= RUWIDO_COMPLETE_DATA_LEN)
                            {
                                irmp_param.protocol         = IRMP_SIEMENS_PROTOCOL;    // change protocol
                                irmp_param.address_offset   = SIEMENS_ADDRESS_OFFSET;
                                irmp_param.address_end      = SIEMENS_ADDRESS_OFFSET + SIEMENS_ADDRESS_LEN;
                                irmp_param.command_offset   = SIEMENS_COMMAND_OFFSET;
                                irmp_param.command_end      = SIEMENS_COMMAND_OFFSET + SIEMENS_COMMAND_LEN;

                                //                   76543210
                                // RUWIDO:  AAAAAAAAACCCCCCCp
                                // SIEMENS: AAAAAAAAAAACCCCCCCCCCp
                                irmp_tmp_address <<= 2;
                                irmp_tmp_address |= (irmp_tmp_command >> 6);
                                irmp_tmp_command &= 0x003F;
//                              irmp_tmp_command <<= 4;
                                irmp_tmp_command |= last_value;
                            }
                        }
                        else
#endif
#if IRMP_SUPPORT_ROOMBA_PROTOCOL == 1
                        if (irmp_param.protocol == IRMP_ROOMBA_PROTOCOL &&                          // Roomba has no stop bit
                            irmp_bit >= ROOMBA_COMPLETE_DATA_LEN - 1)                               // it's the last data bit...
                        {                                                                           // break and close this frame
                            if (irmp_pulse_time >= ROOMBA_1_PULSE_LEN_MIN && irmp_pulse_time <= ROOMBA_1_PULSE_LEN_MAX)
                            {
                                irmp_pause_time = ROOMBA_1_PAUSE_LEN_EXACT;
                            }
                            else if (irmp_pulse_time >= ROOMBA_0_PULSE_LEN_MIN && irmp_pulse_time <= ROOMBA_0_PULSE_LEN_MAX)
                            {
                                irmp_pause_time = ROOMBA_0_PAUSE_LEN;
                            }

                            got_light = TRUE;                                                       // this is a lie, but helps (generates stop bit)
                        }
                        else
#endif
#if IRMP_SUPPORT_MANCHESTER == 1
                        if ((irmp_param.flags & IRMP_PARAM_FLAG_IS_MANCHESTER) &&
                            irmp_pause_time >= 2 * irmp_param.pause_1_len_max && irmp_bit >= irmp_param.complete_len - 2 && !irmp_param.stop_bit)
                        {                                                       // special manchester decoder
                            got_light = TRUE;                                   // this is a lie, but generates a stop bit ;-)
                            irmp_param.stop_bit = TRUE;                         // set flag
                        }
                        else
#endif // IRMP_SUPPORT_MANCHESTER == 1
                        if (irmp_pause_time > IRMP_TIMEOUT_LEN)                 // timeout?
                        {                                                       // yes...
                            if (irmp_bit == irmp_param.complete_len - 1 && irmp_param.stop_bit == 0)
                            {
                                irmp_bit++;
                            }
#if IRMP_SUPPORT_JVC_PROTOCOL == 1
                            else if (irmp_param.protocol == IRMP_NEC_PROTOCOL && (irmp_bit == 16 || irmp_bit == 17))      // it was a JVC stop bit
                            {
                                irmp_param.stop_bit     = TRUE;                                     // set flag
                                irmp_param.protocol     = IRMP_JVC_PROTOCOL;                        // switch protocol
                                irmp_param.complete_len = irmp_bit;                                 // patch length: 16 or 17
                                irmp_tmp_command        = (irmp_tmp_address >> 4);                  // set command: upper 12 bits are command bits
                                irmp_tmp_address        = irmp_tmp_address & 0x000F;                // lower 4 bits are address bits
                                irmp_start_bit_detected = 1;                                        // tricky: don't wait for another start bit...
                            }
#endif // IRMP_SUPPORT_JVC_PROTOCOL == 1
#if IRMP_SUPPORT_LGAIR_PROTOCOL == 1
                            else if (irmp_param.protocol == IRMP_NEC_PROTOCOL && (irmp_bit == 28 || irmp_bit == 29))      // it was a LGAIR stop bit
                            {
                                irmp_param.stop_bit     = TRUE;                                     // set flag
                                irmp_param.protocol     = IRMP_LGAIR_PROTOCOL;                      // switch protocol
                                irmp_param.complete_len = irmp_bit;                                 // patch length: 16 or 17
                                irmp_tmp_command        = irmp_lgair_command;                       // set command: upper 8 bits are command bits
                                irmp_tmp_address        = irmp_lgair_address;                       // lower 4 bits are address bits
                                irmp_start_bit_detected = 1;                                        // tricky: don't wait for another start bit...
                            }
#endif // IRMP_SUPPORT_LGAIR_PROTOCOL == 1

#if IRMP_SUPPORT_NEC42_PROTOCOL == 1
#if IRMP_SUPPORT_NEC_PROTOCOL == 1
                            else if (irmp_param.protocol == IRMP_NEC42_PROTOCOL && irmp_bit == 32)      // it was a NEC stop bit
                            {
                                irmp_param.stop_bit     = TRUE;                                     // set flag
                                irmp_param.protocol     = IRMP_NEC_PROTOCOL;                        // switch protocol
                                irmp_param.complete_len = irmp_bit;                                 // patch length: 16 or 17

                                //        0123456789ABC0123456789ABC0123456701234567
                                // NEC42: AAAAAAAAAAAAAaaaaaaaaaaaaaCCCCCCCCcccccccc
                                // NEC:   AAAAAAAAaaaaaaaaCCCCCCCCcccccccc
                                irmp_tmp_address        |= (irmp_tmp_address2 & 0x0007) << 13;      // fm 2012-02-13: 12 -> 13
                                irmp_tmp_command        = (irmp_tmp_address2 >> 3) | (irmp_tmp_command << 10);
                            }
#endif // IRMP_SUPPORT_NEC_PROTOCOL == 1
#if IRMP_SUPPORT_LGAIR_PROTOCOL == 1
                            else if (irmp_param.protocol == IRMP_NEC42_PROTOCOL && irmp_bit == 28)      // it was a NEC stop bit
                            {
                                irmp_param.stop_bit     = TRUE;                                     // set flag
                                irmp_param.protocol     = IRMP_LGAIR_PROTOCOL;                      // switch protocol
                                irmp_param.complete_len = irmp_bit;                                 // patch length: 16 or 17
                                irmp_tmp_address        = irmp_lgair_address;
                                irmp_tmp_command        = irmp_lgair_command;
                            }
#endif // IRMP_SUPPORT_LGAIR_PROTOCOL == 1
#if IRMP_SUPPORT_JVC_PROTOCOL == 1
                            else if (irmp_param.protocol == IRMP_NEC42_PROTOCOL && (irmp_bit == 16 || irmp_bit == 17))  // it was a JVC stop bit
                            {
                                irmp_param.stop_bit     = TRUE;                                     // set flag
                                irmp_param.protocol     = IRMP_JVC_PROTOCOL;                        // switch protocol
                                irmp_param.complete_len = irmp_bit;                                 // patch length: 16 or 17

                                //        0123456789ABC0123456789ABC0123456701234567
                                // NEC42: AAAAAAAAAAAAAaaaaaaaaaaaaaCCCCCCCCcccccccc
                                // JVC:   AAAACCCCCCCCCCCC
                                irmp_tmp_command        = (irmp_tmp_address >> 4) | (irmp_tmp_address2 << 9);   // set command: upper 12 bits are command bits
                                irmp_tmp_address        = irmp_tmp_address & 0x000F;                            // lower 4 bits are address bits
                            }
#endif // IRMP_SUPPORT_JVC_PROTOCOL == 1
#endif // IRMP_SUPPORT_NEC42_PROTOCOL == 1

#if IRMP_SUPPORT_SAMSUNG48_PROTOCOL == 1
                            else if (irmp_param.protocol == IRMP_SAMSUNG48_PROTOCOL && irmp_bit == 32)          // it was a SAMSUNG32 stop bit
                            {
                                irmp_param.protocol         = IRMP_SAMSUNG32_PROTOCOL;
                                irmp_param.command_offset   = SAMSUNG32_COMMAND_OFFSET;
                                irmp_param.command_end      = SAMSUNG32_COMMAND_OFFSET + SAMSUNG32_COMMAND_LEN;
                                irmp_param.complete_len     = SAMSUNG32_COMPLETE_DATA_LEN;
                            }
#endif // IRMP_SUPPORT_SAMSUNG_PROTOCOL == 1

#if IRMP_SUPPORT_RCMM_PROTOCOL == 1
                            else if (irmp_param.protocol == IRMP_RCMM32_PROTOCOL && (irmp_bit == 12 || irmp_bit == 24))  // it was a RCMM stop bit
                            {
                                if (irmp_bit == 12)
                                {
                                    irmp_tmp_command = (irmp_tmp_address & 0xFF);                   // set command: lower 8 bits are command bits
                                    irmp_tmp_address >>= 8;                                         // upper 4 bits are address bits
                                    irmp_param.protocol     = IRMP_RCMM12_PROTOCOL;                 // switch protocol
                                }
                                else // if ((irmp_bit == 24)
                                {
                                    irmp_param.protocol     = IRMP_RCMM24_PROTOCOL;                 // switch protocol
                                }
                                irmp_param.stop_bit     = TRUE;                                     // set flag
                                irmp_param.complete_len = irmp_bit;                                 // patch length
                            }
#endif // IRMP_SUPPORT_RCMM_PROTOCOL == 1
                            else
                            {
                                irmp_start_bit_detected = 0;                    // wait for another start bit...
                                irmp_pulse_time         = 0;
                                irmp_pause_time         = 0;
                            }
                        }
                    }
                }
                else
                {                                                               // got light now!
                    got_light = TRUE;
                }

                if (got_light)
                {
#if IRMP_SUPPORT_MANCHESTER == 1
                    if ((irmp_param.flags & IRMP_PARAM_FLAG_IS_MANCHESTER))                                     // Manchester
                    {
#if 1
                        if (irmp_pulse_time > irmp_param.pulse_1_len_max /* && irmp_pulse_time <= 2 * irmp_param.pulse_1_len_max */)
#else // better, but some IR-RCs use asymmetric timings :-/
                        if (irmp_pulse_time > irmp_param.pulse_1_len_max && irmp_pulse_time <= 2 * irmp_param.pulse_1_len_max &&
                            irmp_pause_time <= 2 * irmp_param.pause_1_len_max)
#endif
                        {
#if IRMP_SUPPORT_RC6_PROTOCOL == 1
                            if (irmp_param.protocol == IRMP_RC6_PROTOCOL && irmp_bit == 4 && irmp_pulse_time > RC6_TOGGLE_BIT_LEN_MIN)         // RC6 toggle bit
                            {
                                if (irmp_param.complete_len == RC6_COMPLETE_DATA_LEN_LONG)                      // RC6 mode 6A
                                {
                                    irmp_store_bit (1);
                                    last_value = 1;
                                }
                                else                                                                            // RC6 mode 0
                                {
                                    irmp_store_bit (0);
                                    last_value = 0;
                                }
                            }
                            else
#endif // IRMP_SUPPORT_RC6_PROTOCOL == 1
                            {
                                irmp_store_bit ((irmp_param.flags & IRMP_PARAM_FLAG_1ST_PULSE_IS_1) ? 0  :  1 );

#if IRMP_SUPPORT_RC6_PROTOCOL == 1
                                if (irmp_param.protocol == IRMP_RC6_PROTOCOL && irmp_bit == 4 && irmp_pulse_time > RC6_TOGGLE_BIT_LEN_MIN)      // RC6 toggle bit
                                {
                                    irmp_store_bit (1);

                                    if (irmp_pause_time > 2 * irmp_param.pause_1_len_max)
                                    {
                                        last_value = 0;
                                    }
                                    else
                                    {
                                        last_value = 1;
                                    }
                                }
                                else
#endif // IRMP_SUPPORT_RC6_PROTOCOL == 1
                                {
                                    irmp_store_bit ((irmp_param.flags & IRMP_PARAM_FLAG_1ST_PULSE_IS_1) ? 1 :   0 );
#if IRMP_SUPPORT_RC5_PROTOCOL == 1 && (IRMP_SUPPORT_FDC_PROTOCOL == 1 || IRMP_SUPPORT_RCCAR_PROTOCOL == 1)
                                    if (! irmp_param2.protocol)
#endif
                                    {
                                    }
                                    last_value = (irmp_param.flags & IRMP_PARAM_FLAG_1ST_PULSE_IS_1) ? 1 : 0;
                                }
                            }
                        }
                        else if (irmp_pulse_time >= irmp_param.pulse_1_len_min && irmp_pulse_time <= irmp_param.pulse_1_len_max
                                 /* && irmp_pause_time <= 2 * irmp_param.pause_1_len_max */)
                        {
                            uint_fast8_t manchester_value;

                            if (last_pause > irmp_param.pause_1_len_max && last_pause <= 2 * irmp_param.pause_1_len_max)
                            {
                                manchester_value = last_value ? 0 : 1;
                                last_value  = manchester_value;
                            }
                            else
                            {
                                manchester_value = last_value;
                            }

#if IRMP_SUPPORT_RC5_PROTOCOL == 1 && (IRMP_SUPPORT_FDC_PROTOCOL == 1 || IRMP_SUPPORT_RCCAR_PROTOCOL == 1)
                            if (! irmp_param2.protocol)
#endif
                            {
                            }

#if IRMP_SUPPORT_RC6_PROTOCOL == 1
                            if (irmp_param.protocol == IRMP_RC6_PROTOCOL && irmp_bit == 1 && manchester_value == 1)     // RC6 mode != 0 ???
                            {
                                irmp_param.complete_len = RC6_COMPLETE_DATA_LEN_LONG;
                                irmp_param.address_offset = 5;
                                irmp_param.address_end = irmp_param.address_offset + 15;
                                irmp_param.command_offset = irmp_param.address_end + 1;                                 // skip 1 system bit, changes like a toggle bit
                                irmp_param.command_end = irmp_param.command_offset + 16 - 1;
                                irmp_tmp_address = 0;
                            }
#endif // IRMP_SUPPORT_RC6_PROTOCOL == 1

                            irmp_store_bit (manchester_value);
                        }
                        else
                        {
#if IRMP_SUPPORT_RC5_PROTOCOL == 1 && IRMP_SUPPORT_FDC_PROTOCOL == 1
                            if (irmp_param2.protocol == IRMP_FDC_PROTOCOL &&
                                irmp_pulse_time >= FDC_PULSE_LEN_MIN && irmp_pulse_time <= FDC_PULSE_LEN_MAX &&
                                ((irmp_pause_time >= FDC_1_PAUSE_LEN_MIN && irmp_pause_time <= FDC_1_PAUSE_LEN_MAX) ||
                                 (irmp_pause_time >= FDC_0_PAUSE_LEN_MIN && irmp_pause_time <= FDC_0_PAUSE_LEN_MAX)))
                            {
                                irmp_param.protocol = 0;                // switch to FDC, see below
                            }
                            else
#endif // IRMP_SUPPORT_FDC_PROTOCOL == 1
#if IRMP_SUPPORT_RC5_PROTOCOL == 1 && IRMP_SUPPORT_RCCAR_PROTOCOL == 1
                            if (irmp_param2.protocol == IRMP_RCCAR_PROTOCOL &&
                                irmp_pulse_time >= RCCAR_PULSE_LEN_MIN && irmp_pulse_time <= RCCAR_PULSE_LEN_MAX &&
                                ((irmp_pause_time >= RCCAR_1_PAUSE_LEN_MIN && irmp_pause_time <= RCCAR_1_PAUSE_LEN_MAX) ||
                                 (irmp_pause_time >= RCCAR_0_PAUSE_LEN_MIN && irmp_pause_time <= RCCAR_0_PAUSE_LEN_MAX)))
                            {
                                irmp_param.protocol = 0;                // switch to RCCAR, see below
                            }
                            else
#endif // IRMP_SUPPORT_RCCAR_PROTOCOL == 1
                            {
                                irmp_start_bit_detected = 0;                            // reset flags and wait for next start bit
                                irmp_pause_time         = 0;
                            }
                        }

#if IRMP_SUPPORT_RC5_PROTOCOL == 1 && IRMP_SUPPORT_FDC_PROTOCOL == 1
                        if (irmp_param2.protocol == IRMP_FDC_PROTOCOL && irmp_pulse_time >= FDC_PULSE_LEN_MIN && irmp_pulse_time <= FDC_PULSE_LEN_MAX)
                        {
                            if (irmp_pause_time >= FDC_1_PAUSE_LEN_MIN && irmp_pause_time <= FDC_1_PAUSE_LEN_MAX)
                            {
                                irmp_store_bit2 (1);
                            }
                            else if (irmp_pause_time >= FDC_0_PAUSE_LEN_MIN && irmp_pause_time <= FDC_0_PAUSE_LEN_MAX)
                            {
                                irmp_store_bit2 (0);
                            }

                            if (! irmp_param.protocol)
                            {
                                memcpy (&irmp_param, &irmp_param2, sizeof (IRMP_PARAMETER));
                                irmp_param2.protocol = 0;
                                irmp_tmp_address = irmp_tmp_address2;
                                irmp_tmp_command = irmp_tmp_command2;
                            }
                        }
#endif // IRMP_SUPPORT_FDC_PROTOCOL == 1
#if IRMP_SUPPORT_RC5_PROTOCOL == 1 && IRMP_SUPPORT_RCCAR_PROTOCOL == 1
                        if (irmp_param2.protocol == IRMP_RCCAR_PROTOCOL && irmp_pulse_time >= RCCAR_PULSE_LEN_MIN && irmp_pulse_time <= RCCAR_PULSE_LEN_MAX)
                        {
                            if (irmp_pause_time >= RCCAR_1_PAUSE_LEN_MIN && irmp_pause_time <= RCCAR_1_PAUSE_LEN_MAX)
                            {
                                irmp_store_bit2 (1);
                            }
                            else if (irmp_pause_time >= RCCAR_0_PAUSE_LEN_MIN && irmp_pause_time <= RCCAR_0_PAUSE_LEN_MAX)
                            {
                                irmp_store_bit2 (0);
                            }

                            if (! irmp_param.protocol)
                            {
                                memcpy (&irmp_param, &irmp_param2, sizeof (IRMP_PARAMETER));
                                irmp_param2.protocol = 0;
                                irmp_tmp_address = irmp_tmp_address2;
                                irmp_tmp_command = irmp_tmp_command2;
                            }
                        }
#endif // IRMP_SUPPORT_RCCAR_PROTOCOL == 1

                        last_pause      = irmp_pause_time;
                        wait_for_space  = 0;
                    }
                    else
#endif // IRMP_SUPPORT_MANCHESTER == 1

#if IRMP_SUPPORT_SERIAL == 1
                    if (irmp_param.flags & IRMP_PARAM_FLAG_IS_SERIAL)
                    {
                        while (irmp_bit < irmp_param.complete_len && irmp_pulse_time > irmp_param.pulse_1_len_max)
                        {
                            irmp_store_bit (1);

                            if (irmp_pulse_time >= irmp_param.pulse_1_len_min)
                            {
                                irmp_pulse_time -= irmp_param.pulse_1_len_min;
                            }
                            else
                            {
                                irmp_pulse_time = 0;
                            }
                        }

                        while (irmp_bit < irmp_param.complete_len && irmp_pause_time > irmp_param.pause_1_len_max)
                        {
                            irmp_store_bit (0);

                            if (irmp_pause_time >= irmp_param.pause_1_len_min)
                            {
                                irmp_pause_time -= irmp_param.pause_1_len_min;
                            }
                            else
                            {
                                irmp_pause_time = 0;
                            }
                        }
                        wait_for_space = 0;
                    }
                    else
#endif // IRMP_SUPPORT_SERIAL == 1

#if IRMP_SUPPORT_SAMSUNG_PROTOCOL == 1
                    if (irmp_param.protocol == IRMP_SAMSUNG_PROTOCOL && irmp_bit == 16)       // Samsung: 16th bit
                    {
                        if (irmp_pulse_time >= SAMSUNG_PULSE_LEN_MIN && irmp_pulse_time <= SAMSUNG_PULSE_LEN_MAX &&
                            irmp_pause_time >= SAMSUNG_START_BIT_PAUSE_LEN_MIN && irmp_pause_time <= SAMSUNG_START_BIT_PAUSE_LEN_MAX)
                        {
                            wait_for_space = 0;
                            irmp_bit++;
                        }
                        else  if (irmp_pulse_time >= SAMSUNG_PULSE_LEN_MIN && irmp_pulse_time <= SAMSUNG_PULSE_LEN_MAX)
                        {
#if IRMP_SUPPORT_SAMSUNG48_PROTOCOL == 1
                            irmp_param.protocol         = IRMP_SAMSUNG48_PROTOCOL;
                            irmp_param.command_offset   = SAMSUNG48_COMMAND_OFFSET;
                            irmp_param.command_end      = SAMSUNG48_COMMAND_OFFSET + SAMSUNG48_COMMAND_LEN;
                            irmp_param.complete_len     = SAMSUNG48_COMPLETE_DATA_LEN;
#else
                            irmp_param.protocol         = IRMP_SAMSUNG32_PROTOCOL;
                            irmp_param.command_offset   = SAMSUNG32_COMMAND_OFFSET;
                            irmp_param.command_end      = SAMSUNG32_COMMAND_OFFSET + SAMSUNG32_COMMAND_LEN;
                            irmp_param.complete_len     = SAMSUNG32_COMPLETE_DATA_LEN;
#endif
                            if (irmp_pause_time >= SAMSUNG_1_PAUSE_LEN_MIN && irmp_pause_time <= SAMSUNG_1_PAUSE_LEN_MAX)
                            {
                                irmp_store_bit (1);
                                wait_for_space = 0;
                            }
                            else
                            {
                                irmp_store_bit (0);
                                wait_for_space = 0;
                            }
                        }
                        else
                        {                                                           // timing incorrect!
                            irmp_start_bit_detected = 0;                            // reset flags and wait for next start bit
                            irmp_pause_time         = 0;
                        }
                    }
                    else
#endif // IRMP_SUPPORT_SAMSUNG_PROTOCOL

#if IRMP_SUPPORT_NEC16_PROTOCOL
#if IRMP_SUPPORT_NEC42_PROTOCOL == 1
                    if (irmp_param.protocol == IRMP_NEC42_PROTOCOL &&
#else // IRMP_SUPPORT_NEC_PROTOCOL instead
                    if (irmp_param.protocol == IRMP_NEC_PROTOCOL &&
#endif // IRMP_SUPPORT_NEC42_PROTOCOL == 1
                        irmp_bit == 8 && irmp_pause_time >= NEC_START_BIT_PAUSE_LEN_MIN && irmp_pause_time <= NEC_START_BIT_PAUSE_LEN_MAX)
                    {
                        irmp_param.protocol         = IRMP_NEC16_PROTOCOL;
                        irmp_param.address_offset   = NEC16_ADDRESS_OFFSET;
                        irmp_param.address_end      = NEC16_ADDRESS_OFFSET + NEC16_ADDRESS_LEN;
                        irmp_param.command_offset   = NEC16_COMMAND_OFFSET;
                        irmp_param.command_end      = NEC16_COMMAND_OFFSET + NEC16_COMMAND_LEN;
                        irmp_param.complete_len     = NEC16_COMPLETE_DATA_LEN;
                        wait_for_space = 0;
                    }
                    else
#endif // IRMP_SUPPORT_NEC16_PROTOCOL

#if IRMP_SUPPORT_BANG_OLUFSEN_PROTOCOL == 1
                    if (irmp_param.protocol == IRMP_BANG_OLUFSEN_PROTOCOL)
                    {
                        if (irmp_pulse_time >= BANG_OLUFSEN_PULSE_LEN_MIN && irmp_pulse_time <= BANG_OLUFSEN_PULSE_LEN_MAX)
                        {
                            if (irmp_bit == 1)                                      // Bang & Olufsen: 3rd bit
                            {
                                if (irmp_pause_time >= BANG_OLUFSEN_START_BIT3_PAUSE_LEN_MIN && irmp_pause_time <= BANG_OLUFSEN_START_BIT3_PAUSE_LEN_MAX)
                                {
                                    wait_for_space = 0;
                                    irmp_bit++;
                                }
                                else
                                {                                                   // timing incorrect!
                                    irmp_start_bit_detected = 0;                    // reset flags and wait for next start bit
                                    irmp_pause_time         = 0;
                                }
                            }
                            else if (irmp_bit == 19)                                // Bang & Olufsen: trailer bit
                            {
                                if (irmp_pause_time >= BANG_OLUFSEN_TRAILER_BIT_PAUSE_LEN_MIN && irmp_pause_time <= BANG_OLUFSEN_TRAILER_BIT_PAUSE_LEN_MAX)
                                {
                                    wait_for_space = 0;
                                    irmp_bit++;
                                }
                                else
                                {                                                   // timing incorrect!
                                    irmp_start_bit_detected = 0;                    // reset flags and wait for next start bit
                                    irmp_pause_time         = 0;
                                }
                            }
                            else
                            {
                                if (irmp_pause_time >= BANG_OLUFSEN_1_PAUSE_LEN_MIN && irmp_pause_time <= BANG_OLUFSEN_1_PAUSE_LEN_MAX)
                                {                                                   // pulse & pause timings correct for "1"?
                                    irmp_store_bit (1);
                                    last_value = 1;
                                    wait_for_space = 0;
                                }
                                else if (irmp_pause_time >= BANG_OLUFSEN_0_PAUSE_LEN_MIN && irmp_pause_time <= BANG_OLUFSEN_0_PAUSE_LEN_MAX)
                                {                                                   // pulse & pause timings correct for "0"?
                                    irmp_store_bit (0);
                                    last_value = 0;
                                    wait_for_space = 0;
                                }
                                else if (irmp_pause_time >= BANG_OLUFSEN_R_PAUSE_LEN_MIN && irmp_pause_time <= BANG_OLUFSEN_R_PAUSE_LEN_MAX)
                                {
                                    irmp_store_bit (last_value);
                                    wait_for_space = 0;
                                }
                                else
                                {                                                   // timing incorrect!
                                    irmp_start_bit_detected = 0;                    // reset flags and wait for next start bit
                                    irmp_pause_time         = 0;
                                }
                            }
                        }
                        else
                        {                                                           // timing incorrect!
                            irmp_start_bit_detected = 0;                            // reset flags and wait for next start bit
                            irmp_pause_time         = 0;
                        }
                    }
                    else
#endif // IRMP_SUPPORT_BANG_OLUFSEN_PROTOCOL

#if IRMP_SUPPORT_RCMM_PROTOCOL == 1
                    if (irmp_param.protocol == IRMP_RCMM32_PROTOCOL)
                    {
                        if (irmp_pause_time >= RCMM32_BIT_00_PAUSE_LEN_MIN && irmp_pause_time <= RCMM32_BIT_00_PAUSE_LEN_MAX)
                        {
                            irmp_store_bit (0);
                            irmp_store_bit (0);
                        }
                        else if (irmp_pause_time >= RCMM32_BIT_01_PAUSE_LEN_MIN && irmp_pause_time <= RCMM32_BIT_01_PAUSE_LEN_MAX)
                        {
                            irmp_store_bit (0);
                            irmp_store_bit (1);
                        }
                        else if (irmp_pause_time >= RCMM32_BIT_10_PAUSE_LEN_MIN && irmp_pause_time <= RCMM32_BIT_10_PAUSE_LEN_MAX)
                        {
                            irmp_store_bit (1);
                            irmp_store_bit (0);
                        }
                        else if (irmp_pause_time >= RCMM32_BIT_11_PAUSE_LEN_MIN && irmp_pause_time <= RCMM32_BIT_11_PAUSE_LEN_MAX)
                        {
                            irmp_store_bit (1);
                            irmp_store_bit (1);
                        }
                        wait_for_space = 0;
                    }
                    else
#endif

                    if (irmp_pulse_time >= irmp_param.pulse_1_len_min && irmp_pulse_time <= irmp_param.pulse_1_len_max &&
                        irmp_pause_time >= irmp_param.pause_1_len_min && irmp_pause_time <= irmp_param.pause_1_len_max)
                    {                                                               // pulse & pause timings correct for "1"?
                        irmp_store_bit (1);
                        wait_for_space = 0;
                    }
                    else if (irmp_pulse_time >= irmp_param.pulse_0_len_min && irmp_pulse_time <= irmp_param.pulse_0_len_max &&
                             irmp_pause_time >= irmp_param.pause_0_len_min && irmp_pause_time <= irmp_param.pause_0_len_max)
                    {                                                               // pulse & pause timings correct for "0"?
                        irmp_store_bit (0);
                        wait_for_space = 0;
                    }
                    else
#if IRMP_SUPPORT_KATHREIN_PROTOCOL

                    if (irmp_param.protocol == IRMP_KATHREIN_PROTOCOL &&
                        irmp_pulse_time >= KATHREIN_1_PULSE_LEN_MIN && irmp_pulse_time <= KATHREIN_1_PULSE_LEN_MAX &&
                        (((irmp_bit == 8 || irmp_bit == 6) &&
                                irmp_pause_time >= KATHREIN_SYNC_BIT_PAUSE_LEN_MIN && irmp_pause_time <= KATHREIN_SYNC_BIT_PAUSE_LEN_MAX) ||
                         (irmp_bit == 12 &&
                                irmp_pause_time >= KATHREIN_START_BIT_PAUSE_LEN_MIN && irmp_pause_time <= KATHREIN_START_BIT_PAUSE_LEN_MAX)))

                    {
                        if (irmp_bit == 8)
                        {
                            irmp_bit++;
                            irmp_tmp_command <<= 1;
                        }
                        else
                        {
                            irmp_store_bit (1);
                        }
                        wait_for_space = 0;
                    }
                    else
#endif // IRMP_SUPPORT_KATHREIN_PROTOCOL
                    {                                                               // timing incorrect!
                        irmp_start_bit_detected = 0;                                // reset flags and wait for next start bit
                        irmp_pause_time         = 0;
                    }

                    irmp_pulse_time = 1;                                            // set counter to 1, not 0
                }
            }
            else
            {                                                                       // counting the pulse length ...
                if (! irmp_input)                                                   // still light?
                {                                                                   // yes...
                    irmp_pulse_time++;                                              // increment counter
                }
                else
                {                                                                   // now it's dark!
                    wait_for_space  = 1;                                            // let's count the time (see above)
                    irmp_pause_time = 1;                                            // set pause counter to 1, not 0
                }
            }

            if (irmp_start_bit_detected && irmp_bit == irmp_param.complete_len && irmp_param.stop_bit == 0)    // enough bits received?
            {
                if (last_irmp_command == irmp_tmp_command && key_repetition_len < AUTO_FRAME_REPETITION_LEN)
                {
                    repetition_frame_number++;
                }
                else
                {
                    repetition_frame_number = 0;
                }

#if IRMP_SUPPORT_SIRCS_PROTOCOL == 1
                // if SIRCS protocol and the code will be repeated within 50 ms, we will ignore 2nd and 3rd repetition frame
                if (irmp_param.protocol == IRMP_SIRCS_PROTOCOL && (repetition_frame_number == 1 || repetition_frame_number == 2))
                {
                    key_repetition_len = 0;
                }
                else
#endif

#if IRMP_SUPPORT_ORTEK_PROTOCOL == 1
                // if ORTEK protocol and the code will be repeated within 50 ms, we will ignore 2nd repetition frame
                if (irmp_param.protocol == IRMP_ORTEK_PROTOCOL && repetition_frame_number == 1)
                {
                    key_repetition_len = 0;
                }
                else
#endif

#if IRMP_SUPPORT_KASEIKYO_PROTOCOL == 1
                // if KASEIKYO protocol and the code will be repeated within 50 ms, we will ignore 2nd repetition frame
                if (irmp_param.protocol == IRMP_KASEIKYO_PROTOCOL && repetition_frame_number == 1)
                {
                    key_repetition_len = 0;
                }
                else
#endif

#if IRMP_SUPPORT_SAMSUNG_PROTOCOL == 1
                // if SAMSUNG32 or SAMSUNG48 protocol and the code will be repeated within 50 ms, we will ignore every 2nd frame
                if ((irmp_param.protocol == IRMP_SAMSUNG32_PROTOCOL || irmp_param.protocol == IRMP_SAMSUNG48_PROTOCOL) && (repetition_frame_number & 0x01))
                {
                    key_repetition_len = 0;
                }
                else
#endif

#if IRMP_SUPPORT_NUBERT_PROTOCOL == 1
                // if NUBERT protocol and the code will be repeated within 50 ms, we will ignore every 2nd frame
                if (irmp_param.protocol == IRMP_NUBERT_PROTOCOL && (repetition_frame_number & 0x01))
                {
                    key_repetition_len = 0;
                }
                else
#endif

#if IRMP_SUPPORT_SPEAKER_PROTOCOL == 1
                // if SPEAKER protocol and the code will be repeated within 50 ms, we will ignore every 2nd frame
                if (irmp_param.protocol == IRMP_SPEAKER_PROTOCOL && (repetition_frame_number & 0x01))
                {
                    key_repetition_len = 0;
                }
                else
#endif

                {
                    irmp_ir_detected = TRUE;

#if IRMP_SUPPORT_DENON_PROTOCOL == 1
                    if (irmp_param.protocol == IRMP_DENON_PROTOCOL)
                    {                                                               // check for repetition frame
                        if ((~irmp_tmp_command & 0x3FF) == last_irmp_denon_command) // command bits must be inverted
                        {
                            irmp_tmp_command = last_irmp_denon_command;             // use command received before!
                            last_irmp_denon_command = 0;

                            irmp_protocol = irmp_param.protocol;                    // store protocol
                            irmp_address = irmp_tmp_address;                        // store address
                            irmp_command = irmp_tmp_command;                        // store command
                        }
                        else
                        {
                            if ((irmp_tmp_command & 0x01) == 0x00)
                            {
                                last_irmp_denon_command = irmp_tmp_command;
                                denon_repetition_len = 0;
                                irmp_ir_detected = FALSE;
                            }
                            else
                            {
                                last_irmp_denon_command = 0;
                                irmp_ir_detected = FALSE;
                            }
                        }
                    }
                    else
#endif // IRMP_SUPPORT_DENON_PROTOCOL

#if IRMP_SUPPORT_GRUNDIG_PROTOCOL == 1
                    if (irmp_param.protocol == IRMP_GRUNDIG_PROTOCOL && irmp_tmp_command == 0x01ff)
                    {                                                               // Grundig start frame?
                        irmp_ir_detected = FALSE;
                    }
                    else
#endif // IRMP_SUPPORT_GRUNDIG_PROTOCOL

#if IRMP_SUPPORT_NOKIA_PROTOCOL == 1
                    if (irmp_param.protocol == IRMP_NOKIA_PROTOCOL && irmp_tmp_address == 0x00ff && irmp_tmp_command == 0x00fe)
                    {                                                               // Nokia start frame?
                        irmp_ir_detected = FALSE;
                    }
                    else
#endif // IRMP_SUPPORT_NOKIA_PROTOCOL
                    {
#if IRMP_SUPPORT_NEC_PROTOCOL == 1
                        if (irmp_param.protocol == IRMP_NEC_PROTOCOL && irmp_bit == 0)  // repetition frame
                        {
                            if (key_repetition_len < NEC_FRAME_REPEAT_PAUSE_LEN_MAX)
                            {
                                irmp_tmp_address = last_irmp_address;                   // address is last address
                                irmp_tmp_command = last_irmp_command;                   // command is last command
                                irmp_flags |= IRMP_FLAG_REPETITION;
                                key_repetition_len = 0;
                            }
                            else
                            {
                                irmp_ir_detected = FALSE;
                            }
                        }
#endif // IRMP_SUPPORT_NEC_PROTOCOL

#if IRMP_SUPPORT_KASEIKYO_PROTOCOL == 1
                        if (irmp_param.protocol == IRMP_KASEIKYO_PROTOCOL)
                        {
                            uint_fast8_t xor_value;

                            xor_value = (xor_check[0] & 0x0F) ^ ((xor_check[0] & 0xF0) >> 4) ^ (xor_check[1] & 0x0F) ^ ((xor_check[1] & 0xF0) >> 4);

                            if (xor_value != (xor_check[2] & 0x0F))
                            {
                                irmp_ir_detected = FALSE;
                            }

                            xor_value = xor_check[2] ^ xor_check[3] ^ xor_check[4];

                            if (xor_value != xor_check[5])
                            {
                                irmp_ir_detected = FALSE;
                            }

                            irmp_flags |= genre2;       // write the genre2 bits into MSB of the flag byte
                        }
#endif // IRMP_SUPPORT_KASEIKYO_PROTOCOL == 1

#if IRMP_SUPPORT_ORTEK_PROTOCOL == 1
                        if (irmp_param.protocol == IRMP_ORTEK_PROTOCOL)
                        {
                            if (parity == PARITY_CHECK_FAILED)
                            {
                                irmp_ir_detected = FALSE;
                            }

                            if ((irmp_tmp_address & 0x03) == 0x02)
                            {
                                irmp_ir_detected = FALSE;
                            }
                            irmp_tmp_address >>= 2;
                        }
#endif // IRMP_SUPPORT_ORTEK_PROTOCOL == 1

#if IRMP_SUPPORT_RC6_PROTOCOL == 1
                        if (irmp_param.protocol == IRMP_RC6_PROTOCOL && irmp_param.complete_len == RC6_COMPLETE_DATA_LEN_LONG)     // RC6 mode = 6?
                        {
                            irmp_protocol = IRMP_RC6A_PROTOCOL;
                        }
                        else
#endif // IRMP_SUPPORT_RC6_PROTOCOL == 1
                        {
                            irmp_protocol = irmp_param.protocol;
                        }

#if IRMP_SUPPORT_FDC_PROTOCOL == 1
                        if (irmp_param.protocol == IRMP_FDC_PROTOCOL)
                        {
                            if (irmp_tmp_command & 0x000F)                          // released key?
                            {
                                irmp_tmp_command = (irmp_tmp_command >> 4) | 0x80;  // yes, set bit 7
                            }
                            else
                            {
                                irmp_tmp_command >>= 4;                             // no, it's a pressed key
                            }
                            irmp_tmp_command |= (irmp_tmp_address << 2) & 0x0F00;   // 000000CCCCAAAAAA -> 0000CCCC00000000
                            irmp_tmp_address &= 0x003F;
                        }
#endif

                        irmp_address = irmp_tmp_address;                            // store address
#if IRMP_SUPPORT_NEC_PROTOCOL == 1
                        if (irmp_param.protocol == IRMP_NEC_PROTOCOL)
                        {
                            last_irmp_address = irmp_tmp_address;                   // store as last address, too
                        }
#endif

#if IRMP_SUPPORT_RC5_PROTOCOL == 1
                        if (irmp_param.protocol == IRMP_RC5_PROTOCOL)
                        {
                            irmp_tmp_command |= rc5_cmd_bit6;                       // store bit 6
                        }
#endif
                        irmp_command = irmp_tmp_command;                            // store command

#if IRMP_SUPPORT_SAMSUNG_PROTOCOL == 1
                        irmp_id = irmp_tmp_id;
#endif
                    }
                }

                if (irmp_ir_detected)
                {
                    if (last_irmp_command == irmp_tmp_command &&
                        last_irmp_address == irmp_tmp_address &&
                        key_repetition_len < IRMP_KEY_REPETITION_LEN)
                    {
                        irmp_flags |= IRMP_FLAG_REPETITION;
                    }

                    last_irmp_address = irmp_tmp_address;                           // store as last address, too
                    last_irmp_command = irmp_tmp_command;                           // store as last command, too

                    key_repetition_len = 0;
                }
                else
                {
                }

                irmp_start_bit_detected = 0;                                        // and wait for next start bit
                irmp_tmp_command        = 0;
                irmp_pulse_time         = 0;
                irmp_pause_time         = 0;

#if IRMP_SUPPORT_JVC_PROTOCOL == 1
                if (irmp_protocol == IRMP_JVC_PROTOCOL)                             // the stop bit of JVC frame is also start bit of next frame
                {                                                                   // set pulse time here!
                    irmp_pulse_time = ((uint_fast8_t)(F_INTERRUPTS * JVC_START_BIT_PULSE_TIME));
                }
#endif // IRMP_SUPPORT_JVC_PROTOCOL == 1
            }
        }
    }

    return (irmp_ir_detected);
}
