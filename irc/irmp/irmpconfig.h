/*---------------------------------------------------------------------------------------------------------------------------------------------------
 * irmpconfig.h
 *
 * DO NOT INCLUDE THIS FILE, WILL BE INCLUDED BY IRMP.H!
 *
 * Copyright (c) 2009-2015 Frank Meyer - frank(at)fli4l.de
 * Extensions for PIC 12F1820 W.Strobl 2014-07-20
 *
 * $Id: irmpconfig.h,v 1.124 2015/01/26 13:07:01 fm Exp $
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *---------------------------------------------------------------------------------------------------------------------------------------------------
 */

#ifndef _IRMPCONFIG_H_
#define _IRMPCONFIG_H_

#ifndef _IRMP_H_
#  error please include only irmp.h, not irmpconfig.h
#endif

/*---------------------------------------------------------------------------------------------------------------------------------------------------
 * Change F_INTERRUPTS if you change the number of interrupts per second,
 * Normally, F_INTERRUPTS should be in the range from 10000 to 15000, typical is 15000
 * A value above 15000 costs additional program space, absolute maximum value is 20000.
 * On PIC with XC8/C18 Compiler, use 15151 as the correct value.
 *---------------------------------------------------------------------------------------------------------------------------------------------------
 */
#ifndef F_INTERRUPTS
#  define F_INTERRUPTS                          15000   // interrupts per second, min: 10000, max: 20000, typ: 15000
#endif

/*---------------------------------------------------------------------------------------------------------------------------------------------------
 * Change settings from 1 to 0 if you want to disable one or more decoders.
 * This saves program space.
 *
 * 1 enable  decoder
 * 0 disable decoder
 *
 * The standard decoders are enabled per default.
 * Less common protocols are disabled here, you need to enable them manually.
 *
 * If you want to use FDC or RCCAR simultaneous with RC5 protocol, additional program space is required.
 * If you don't need RC5 when using FDC/RCCAR, you should disable RC5.
 * You cannot enable both DENON and RUWIDO, only enable one of them.
 *---------------------------------------------------------------------------------------------------------------------------------------------------
 */

// typical protocols, disable here!             Enable  Remarks                 F_INTERRUPTS            Program Space
#define IRMP_SUPPORT_SIRCS_PROTOCOL             1       // Sony SIRCS           >= 10000                 ~150 bytes
#define IRMP_SUPPORT_NEC_PROTOCOL               1       // NEC + APPLE          >= 10000                 ~300 bytes
#define IRMP_SUPPORT_SAMSUNG_PROTOCOL           1       // Samsung + Samsg32    >= 10000                 ~300 bytes
#define IRMP_SUPPORT_KASEIKYO_PROTOCOL          1       // Kaseikyo             >= 10000                 ~250 bytes

// more protocols, enable here!                 Enable  Remarks                 F_INTERRUPTS            Program Space
#define IRMP_SUPPORT_JVC_PROTOCOL               0       // JVC                  >= 10000                 ~150 bytes
#define IRMP_SUPPORT_NEC16_PROTOCOL             0       // NEC16                >= 10000                 ~100 bytes
#define IRMP_SUPPORT_NEC42_PROTOCOL             0       // NEC42                >= 10000                 ~300 bytes
#define IRMP_SUPPORT_MATSUSHITA_PROTOCOL        0       // Matsushita           >= 10000                  ~50 bytes
#define IRMP_SUPPORT_DENON_PROTOCOL             0       // DENON, Sharp         >= 10000                 ~250 bytes
#define IRMP_SUPPORT_RC5_PROTOCOL               0       // RC5                  >= 10000                 ~250 bytes
#define IRMP_SUPPORT_RC6_PROTOCOL               0       // RC6 & RC6A           >= 10000                 ~250 bytes
#define IRMP_SUPPORT_IR60_PROTOCOL              0       // IR60 (SDA2008)       >= 10000                 ~300 bytes
#define IRMP_SUPPORT_GRUNDIG_PROTOCOL           0       // Grundig              >= 10000                 ~300 bytes
#define IRMP_SUPPORT_SIEMENS_PROTOCOL           0       // Siemens Gigaset      >= 15000                 ~550 bytes
#define IRMP_SUPPORT_NOKIA_PROTOCOL             0       // Nokia                >= 10000                 ~300 bytes

// exotic protocols, enable here!               Enable  Remarks                 F_INTERRUPTS            Program Space
#define IRMP_SUPPORT_BOSE_PROTOCOL              0       // BOSE                 >= 10000                 ~150 bytes
#define IRMP_SUPPORT_KATHREIN_PROTOCOL          0       // Kathrein             >= 10000                 ~200 bytes
#define IRMP_SUPPORT_NUBERT_PROTOCOL            0       // NUBERT               >= 10000                  ~50 bytes
#define IRMP_SUPPORT_SPEAKER_PROTOCOL           0       // SPEAKER (~NUBERT)    >= 10000                  ~50 bytes
#define IRMP_SUPPORT_BANG_OLUFSEN_PROTOCOL      0       // Bang & Olufsen       >= 10000                 ~200 bytes
#define IRMP_SUPPORT_RECS80_PROTOCOL            0       // RECS80 (SAA3004)     >= 15000                  ~50 bytes
#define IRMP_SUPPORT_RECS80EXT_PROTOCOL         0       // RECS80EXT (SAA3008)  >= 15000                  ~50 bytes
#define IRMP_SUPPORT_THOMSON_PROTOCOL           0       // Thomson              >= 10000                 ~250 bytes
#define IRMP_SUPPORT_NIKON_PROTOCOL             0       // NIKON camera         >= 10000                 ~250 bytes
#define IRMP_SUPPORT_NETBOX_PROTOCOL            0       // Netbox keyboard      >= 10000                 ~400 bytes (PROTOTYPE!)
#define IRMP_SUPPORT_ORTEK_PROTOCOL             0       // ORTEK (Hama)         >= 10000                 ~150 bytes
#define IRMP_SUPPORT_TELEFUNKEN_PROTOCOL        0       // Telefunken 1560      >= 10000                 ~150 bytes
#define IRMP_SUPPORT_FDC_PROTOCOL               0       // FDC3402 keyboard     >= 10000 (better 15000)  ~150 bytes (~400 in combination with RC5)
#define IRMP_SUPPORT_RCCAR_PROTOCOL             0       // RC Car               >= 10000 (better 15000)  ~150 bytes (~500 in combination with RC5)
#define IRMP_SUPPORT_ROOMBA_PROTOCOL            0       // iRobot Roomba        >= 10000                 ~150 bytes
#define IRMP_SUPPORT_RUWIDO_PROTOCOL            0       // RUWIDO, T-Home       >= 15000                 ~550 bytes
#define IRMP_SUPPORT_A1TVBOX_PROTOCOL           0       // A1 TV BOX            >= 15000 (better 20000)  ~300 bytes
#define IRMP_SUPPORT_LEGO_PROTOCOL              0       // LEGO Power RC        >= 20000                 ~150 bytes
#define IRMP_SUPPORT_RCMM_PROTOCOL              0       // RCMM 12,24, or 32    >= 20000                 ~150 bytes
#define IRMP_SUPPORT_LGAIR_PROTOCOL             0       // LG Air Condition     >= 10000                 ~300 bytes
#define IRMP_SUPPORT_SAMSUNG48_PROTOCOL         0       // Samsung48            >= 10000                 ~100 bytes (SAMSUNG must be enabled!)
#define IRMP_SUPPORT_MERLIN_PROTOCOL            0       // Merlin               >= 15000 (better 20000)  ~300 bytes
#define IRMP_SUPPORT_RADIO1_PROTOCOL            0       // RADIO, e.g. TEVION   >= 10000                 ~250 bytes (experimental)


/*---------------------------------------------------------------------------------------------------------------------------------------------------
 * Change hardware pin here for ARM STM32
 *---------------------------------------------------------------------------------------------------------------------------------------------------
 */
#  define IRMP_PORT_LETTER                      C
#  define IRMP_BIT_NUMBER                       13


#endif /* _WC_IRMPCONFIG_H_ */
