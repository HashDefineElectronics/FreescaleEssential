/*
** ###################################################################
**     Processors:          MK51DX128CLH7
**                          MK51DX128CLK7
**                          MK51DX256CLK7
**                          MK51DX256CLL7
**                          MK51DX128CMB7
**                          MK51DX256CMB7
**                          MK51DX256CML7
**
**     Compilers:           ARM Compiler
**                          Freescale C/C++ for Embedded ARM
**                          GNU C Compiler
**                          IAR ANSI C/C++ Compiler for ARM
**
**     Reference manual:    K51P144M72SF1RM Rev. 0, Nov 2011
**     Version:             rev. 2.3, 2013-06-24
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
**     - rev. 1.0 (2011-11-15)
**         Initial version.
**     - rev. 2.0 (2012-01-15)
**         Initial public version.
**     - rev. 2.1 (2012-04-13)
**         Added new #define symbol MCU_MEM_MAP_VERSION_MINOR.
**         Added new #define symbols <peripheralType>_BASE_PTRS.
**     - rev. 2.2 (2013-04-05)
**         Changed start of doxygen comment.
**     - rev. 2.3 (2013-06-24)
**         NV_FOPT register - NMI_DIS bit added.
**
** ###################################################################
*/

/*!
 * @file MK51D7
 * @version 2.3
 * @date 2013-06-24
 * @brief Device specific configuration file for MK51D7 (implementation file)
 *
 * Provides a system configuration function and a global variable that contains
 * the system frequency. It configures the device and initializes the oscillator
 * (PLL) that is part of the microcontroller device.
 */

#include <stdint.h>
#include "MK51D7.h"

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
