/*---------------------------------------------------------------------------------------------------------------------------------------------------
 * irmpsystem.h - system specific includes and defines
 *
 * Copyright (c) 2009-2015 Frank Meyer - frank(at)fli4l.de
 *
 * $Id: irmpsystem.h,v 1.16 2015/01/26 13:07:01 fm Exp $
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *---------------------------------------------------------------------------------------------------------------------------------------------------
 */

#ifndef _IRMPSYSTEM_H_
#define _IRMPSYSTEM_H_

#if !defined(_IRMP_H_) && !defined(_IRSND_H_)
#  error please include only irmp.h or irsnd.h, not irmpsystem.h
#endif

#  include <stm32f10x.h>
#  define ARM_STM32
#  define ARM_STM32F10X
#  define F_CPU (SysCtlClockGet())

#include <string.h>


#  include "stm32f10x_gpio.h"
#  include "stm32f10x_rcc.h"
#  include "stm32f10x_tim.h"
#  include "misc.h"
#  define PROGMEM
#  define memcpy_P                      memcpy


typedef unsigned char                   uint8_t;
typedef unsigned short                  uint16_t;

#ifndef TRUE
#  define TRUE                          1
#  define FALSE                         0
#endif

typedef struct __attribute__ ((__packed__))
{
  uint8_t                               protocol;                                   // protocol, e.g. NEC_PROTOCOL
  uint16_t                              address;                                    // address
  uint16_t                              command;                                    // command
  uint8_t                               flags;                                      // flags, e.g. repetition
} IRMP_DATA;

#endif // _IRMPSYSTEM_H_
