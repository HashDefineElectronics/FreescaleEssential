/*
** ###################################################################
**     Processors:          MK30DX64VLH7
**                          MK30DX128VLH7
**                          MK30DX256VLH7
**                          MK30DX64VLK7
**                          MK30DX128VLK7
**                          MK30DX256VLK7
**                          MK30DX128VLL7
**                          MK30DX256VLL7
**                          MK30DX64VMB7
**                          MK30DX128VMB7
**                          MK30DX256VMB7
**                          MK30DX128VML7
**                          MK30DX256VML7
**
**     Compilers:           ARM Compiler
**                          Freescale C/C++ for Embedded ARM
**                          GNU C Compiler
**                          IAR ANSI C/C++ Compiler for ARM
**
**     Reference manual:    K30P144M72SF1RM Rev. 0, Nov 2011
**     Version:             rev. 1.3, 2013-06-24
**
**     Abstract:
**         Provides a system configuration function and a global variable that
**         contains the system frequency. It configures the device and initializes
**         the oscillator (PLL) that is part of the microcontroller device.
**
**     Copyright: 2013 Freescale, Inc. All Rights Reserved.
**
**     http:                 www.freescale.com
**     mail:                 support@freescale.com
**
**     Revisions:
**     - rev. 1.0 (2012-01-15)
**         Initial public version.
**     - rev. 1.1 (2012-04-13)
**         Added new #define symbol MCU_MEM_MAP_VERSION_MINOR.
**         Added new #define symbols <peripheralType>_BASE_PTRS.
**     - rev. 1.2 (2013-04-05)
**         Changed start of doxygen comment.
**     - rev. 1.3 (2013-06-24)
**         NV_FOPT register - NMI_DIS bit added.
**
** ###################################################################
*/

/*!
 * @file MK30D7
 * @version 1.3
 * @date 2013-06-24
 * @brief Device specific configuration file for MK30D7 (implementation file)
 *
 * Provides a system configuration function and a global variable that contains
 * the system frequency. It configures the device and initializes the oscillator
 * (PLL) that is part of the microcontroller device.
 */

#include <stdint.h>
#include "MK30D7.h"

  #define DEFAULT_SYSTEM_CLOCK            20971520u /* Default System clock value */


/* ----------------------------------------------------------------------------
   -- Core clock
   ---------------------------------------------------------------------------- */

uint32_t SystemCoreClock = DEFAULT_SYSTEM_CLOCK;

/* ----------------------------------------------------------------------------
   -- SystemInit()
   ---------------------------------------------------------------------------- */

void SystemInit (void) {
}

/* ----------------------------------------------------------------------------
   -- SystemCoreClockUpdate()
   ---------------------------------------------------------------------------- */

void SystemCoreClockUpdate (void) {
}
