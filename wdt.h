/* Copyright (c) 2002, 2004 Marek Michalkiewicz
   Copyright (c) 2005, 2006, 2007 Eric B. Weddington
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.

   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. */

/* $Id: wdt.h,v 1.44 2007/11/29 22:37:14 arcanum Exp $ */

/*
   avr/wdt.h - macros for AVR watchdog timer
 */

#ifndef _AVR_WDT_H_
#define _AVR_WDT_H_

#include <io.h>
#include <stdint.h>

/** \file */
/** \defgroup avr_watchdog <avr/wdt.h>: Watchdog timer handling
    \code #include <avr/wdt.h> \endcode

    This header file declares the interface to some inline macros
    handling the watchdog timer present in many AVR devices.  In order
    to prevent the watchdog timer configuration from being
    accidentally altered by a crashing application, a special timed
    sequence is required in order to change it.  The macros within
    this header file handle the required sequence automatically
    before changing any value.  Interrupts will be disabled during
    the manipulation.

    \note Depending on the fuse configuration of the particular
    device, further restrictions might apply, in particular it might
    be disallowed to turn off the watchdog timer.

    Note that for newer devices (ATmega88 and newer, effectively any
    AVR that has the option to also generate interrupts), the watchdog
    timer remains active even after a system reset (except a power-on
    condition), using the fastest prescaler value (approximately 15
    ms).  It is therefore required to turn off the watchdog early
    during program startup, the datasheet recommends a sequence like
    the following:

    \code
    #include <stdint.h>
    #include <avr/wdt.h>

    uint8_t mcusr_mirror _attribute_ ((section (".noinit")));

    void get_mcusr(void) \
      __attribute__((naked)) \
      __attribute__((section(".init3")));
    void get_mcusr(void)
    {
      mcusr_mirror = MCUSR;
      MCUSR = 0;
      wdt_disable();
    }
    \endcode

    Saving the value of MCUSR in \c mcusr_mirror is only needed if the
    application later wants to examine the reset source, but clearing
    in particular the watchdog reset flag before disabling the
    watchdog is required, according to the datasheet.
*/

/**
   \ingroup avr_watchdog
   Reset the watchdog timer.  When the watchdog timer is enabled,
   a call to this instruction is required before the timer expires,
   otherwise a watchdog-initiated device reset will occur. */

#define wdt_reset()         __asm__ __volatile__ ("wdr")

#if     defined(WDP3)
#define _WDT_PS3_MASK      _BV(WDP3)
#else
#define _WDT_PS3_MASK      0x00
#endif

#if     defined(WDTCSR)
#define _WDT_CONTROL_REG   WDTCSR
#else
#define _WDT_CONTROL_REG   WDTCR
#endif

#if     defined(WDTOE)
#define _WDT_CHANGE_BIT    WDTOE
#else
#define _WDT_CHANGE_BIT    WDCE
#endif

/* determine sts/out requirements */
#undef  _MMIO_BYTE
#define _MMIO_BYTE(mem_addr) (mem_addr)

#if     (_WDT_CONTROL_REG > 0x5F)
#define _WDT_USE_STS 1 /* sts */
#else
#define _WDT_USE_STS 0 /* out */
#endif

#undef  _MMIO_BYTE
#define _MMIO_BYTE(mem_addr) (*(volatile uint8_t *)(mem_addr))


#if (_WDT_USE_STS) /* use sts instruction */

#define _wdt_write(value)                                       \
    __asm__ __volatile__ (                                      \
    "in   __tmp_reg__,__SREG__"                         "\n\t"  \
    "cli"                                               "\n\t"  \
    "wdr"                                               "\n\t"  \
    "sts  %0,%1"                                        "\n\t"  \
    "out  __SREG__,__tmp_reg__"                         "\n\t"  \
    "sts  %0,%2"                                        "\n\t"  \
    : /* no outputs */                                          \
    : "M" (_SFR_MEM_ADDR(_WDT_CONTROL_REG)),                    \
      "r" ((uint8_t)(_BV(_WDT_CHANGE_BIT)|_BV(WDE))),           \
      "r" ((uint8_t)((value&0x08?_WDT_PS3_MASK:0x00)|           \
          (value&0x80?(((value>>1)&0x48)|_BV(WDIF)):_BV(WDE))|  \
          (value&7)))                                           \
    : "r0"                                                      \
    )

#define wdt_prescale(value)                                     \
    __asm__ __volatile__ (                                      \
    "in   __tmp_reg__,__SREG__"                         "\n\t"  \
    "cli"                                               "\n\t"  \
    "wdr"                                               "\n\t"  \
    "sts  %0,%1"                                        "\n\t"  \
    "out  __SREG__,__tmp_reg__"                         "\n\t"  \
    "sts  %0,%2"                                        "\n\t"  \
    : /* no outputs */                                          \
    : "M" (_SFR_MEM_ADDR(_WDT_CONTROL_REG)),                    \
      "r" ((uint8_t)(_BV(_WDT_CHANGE_BIT)|_BV(WDE))),           \
      "r" ((uint8_t)((value&0x08?_WDT_PS3_MASK:0x00)|           \
          (value&7)))                                           \
    : "r0"                                                      \
    )

#define wdt_disable()                                           \
    __asm__ __volatile__ (                                      \
    "lds   r24,%0"                                  "\n\t"      \
    "andi  r24,0x27"                                "\n\t"      \
    "in   __tmp_reg__,__SREG__"                     "\n\t"      \
    "cli"                                           "\n\t"      \
    "sts  %0,%1"                                    "\n\t"      \
    "out  __SREG__,__tmp_reg__"                     "\n\t"      \
    "sts  %0,r24"                                   "\n\t"      \
    :                                                           \
    : "M" (_SFR_MEM_ADDR(_WDT_CONTROL_REG)),                    \
      "r" ((uint8_t)(_BV(_WDT_CHANGE_BIT)|_BV(WDE)))            \
    : "r0","r24"                                                \
    )

#else /* use out instruction */

#define _wdt_write(value)                                       \
    __asm__ __volatile__ (                                      \
    "in   __tmp_reg__,__SREG__"                     "\n\t"      \
    "cli"                                           "\n\t"      \
    "wdr"                                           "\n\t"      \
    "out  %0,%1"                                    "\n\t"      \
    "out  __SREG__,__tmp_reg__"                     "\n\t"      \
    "out  %0,%2"                                    "\n\t"      \
    : /* no outputs */                                          \
    : "M" (_SFR_IO_ADDR(_WDT_CONTROL_REG)),                     \
      "r" (_BV(_WDT_CHANGE_BIT)|_BV(WDE)),                      \
      "r" ((uint8_t)((value&0x08?_WDT_PS3_MASK:0x00)|           \
          (value&0x80?(((value>>1)&0x48)|_BV(WDIF)):_BV(WDE))|  \
          (value&7)))                                           \
    : "r0"                                                      \
    )

#define wdt_prescale(value)                                     \
    __asm__ __volatile__ (                                      \
    "in   __tmp_reg__,__SREG__"                     "\n\t"      \
    "cli"                                           "\n\t"      \
    "wdr"                                           "\n\t"      \
    "out  %0,%1"                                    "\n\t"      \
    "out  __SREG__,__tmp_reg__"                     "\n\t"      \
    "out  %0,%2"                                    "\n\t"      \
    : /* no outputs */                                          \
    : "M" (_SFR_IO_ADDR(_WDT_CONTROL_REG)),                     \
      "r" ((uint8_t)(_BV(_WDT_CHANGE_BIT)|_BV(WDE))),           \
      "r" ((uint8_t)((value&0x08?_WDT_PS3_MASK:0x00)|           \
          (value&7)))                                           \
    : "r0"                                                      \
    )

/**
   \ingroup avr_watchdog
   Disable the watchdog timer, if possible.  This attempts to turn off the 
   Enable bit in the watchdog control register. See the datasheet for 
   details.
*/

#define wdt_disable()                                           \
    __asm__ __volatile__ (                                      \
    "in   r24,%0"                                   "\n\t"      \
    "andi r24,0x27"                                 "\n\t"      \
    "in   __tmp_reg__,__SREG__"                     "\n\t"      \
    "cli"                                           "\n\t"      \
    "out  %0,%1"                                    "\n\t"      \
    "out  __SREG__,__tmp_reg__"                     "\n\t"      \
    "out  %0,r24"                                   "\n\t"      \
    :                                                           \
    : "M" (_SFR_MEM_ADDR(_WDT_CONTROL_REG)),                    \
      "r" ((uint8_t)(_BV(_WDT_CHANGE_BIT)|_BV(WDE)))            \
    : "r0","r24"                                                \
    )
    
#endif

/**
   \ingroup avr_watchdog
   Enable the watchdog timer, configuring it for expiry after
   \c timeout (which is a combination of the \c WDP0 through
   \c WDP2 bits to write into the \c WDTCR register; For those devices 
   that have a \c WDTCSR register, it uses the combination of the \c WDP0 
   through \c WDP3 bits).

   See also the symbolic constants \c WDTO_15MS et al.
*/

#if     defined(WDIE)
#define WDT_IRQ_ONLY            0x80
#define WDT_IRQ_RST             0x90
#define wdt_irq_on()            _WDT_CONTROL_REG = _BV(WDIE)|_BV(WDIF)
#define wdt_irq_rst_on()        _WDT_CONTROL_REG = _BV(WDIE)|_BV(WDE)|_BV(WDIF)
#define wdt_enable(timeout,...) _wdt_write(((timeout)|(__VA_ARGS__+0)))
#else
#define WDIF                    7
#define wdt_enable(timeout)     _wdt_write(timeout)
#endif

#define wdt_on()                _WDT_CONTROL_REG = _BV(WDE)

/**
   \ingroup avr_watchdog
   Symbolic constants for the watchdog timeout.  Since the watchdog
   timer is based on a free-running RC oscillator, the times are
   approximate only and apply to a supply voltage of 5 V.  At lower
   supply voltages, the times will increase.  For older devices, the
   times will be as large as three times when operating at Vcc = 3 V,
   while the newer devices (e. g. ATmega128, ATmega8) only experience
   a negligible change.

   Possible timeout values are: 15 ms, 30 ms, 60 ms, 120 ms, 250 ms,
   500 ms, 1 s, 2 s.  (Some devices also allow for 4 s and 8 s.)
   Symbolic constants are formed by the prefix
   \c WDTO_, followed by the time.

   Example that would select a watchdog timer expiry of approximately
   500 ms:
   \code
   wdt_enable(WDTO_500MS);
   \endcode
*/
#define WDTO_15MS   0

/** \ingroup avr_watchdog
    See \c WDT0_15MS */
#define WDTO_30MS   1

/** \ingroup avr_watchdog See
    \c WDT0_15MS */
#define WDTO_60MS   2

/** \ingroup avr_watchdog
    See \c WDT0_15MS */
#define WDTO_120MS  3

/** \ingroup avr_watchdog
    See \c WDT0_15MS */
#define WDTO_250MS  4

/** \ingroup avr_watchdog
    See \c WDT0_15MS */
#define WDTO_500MS  5

/** \ingroup avr_watchdog
    See \c WDT0_15MS */
#define WDTO_1S     6

/** \ingroup avr_watchdog
    See \c WDT0_15MS */
#define WDTO_2S     7

#if defined(__DOXYGEN__) || defined(WDP3)

/** \ingroup avr_watchdog
    See \c WDT0_15MS
    Note: This is only available on the 
    ATtiny2313, 
    ATtiny24, ATtiny44, ATtiny84, 
    ATtiny25, ATtiny45, ATtiny85, 
    ATtiny261, ATtiny461, ATtiny861, 
    ATmega48, ATmega88, ATmega168,
    ATmega48P, ATmega88P, ATmega168P, ATmega328P,
    ATmega164P, ATmega324P, ATmega644P, ATmega644,
    ATmega640, ATmega1280, ATmega1281, ATmega2560, ATmega2561,
    ATmega8HVA, ATmega16HVA, ATmega32HVB,
    ATmega406, ATmega1284P,
    AT90PWM1, AT90PWM2, AT90PWM2B, AT90PWM3, AT90PWM3B, AT90PWM216, AT90PWM316
    AT90USB82, AT90USB162,
    AT90USB646, AT90USB647, AT90USB1286, AT90USB1287,
    ATtiny48, ATtiny88.
    */
#define WDTO_4S     8

/** \ingroup avr_watchdog
    See \c WDT0_15MS
    Note: This is only available on the 
    ATtiny2313, 
    ATtiny24, ATtiny44, ATtiny84, 
    ATtiny25, ATtiny45, ATtiny85, 
    ATtiny261, ATtiny461, ATtiny861, 
    ATmega48, ATmega88, ATmega168,
    ATmega48P, ATmega88P, ATmega168P, ATmega328P,
    ATmega164P, ATmega324P, ATmega644P, ATmega644,
    ATmega640, ATmega1280, ATmega1281, ATmega2560, ATmega2561,
    ATmega8HVA, ATmega16HVA, ATmega32HVB,
    ATmega406, ATmega1284P,
    AT90PWM1, AT90PWM2, AT90PWM2B, AT90PWM3, AT90PWM3B, AT90PWM216, AT90PWM316
    AT90USB82, AT90USB162,
    AT90USB646, AT90USB647, AT90USB1286, AT90USB1287,
    ATtiny48, ATtiny88.
    */
#define WDTO_8S     9

#endif


#endif /* _AVR_WDT_H_ */
