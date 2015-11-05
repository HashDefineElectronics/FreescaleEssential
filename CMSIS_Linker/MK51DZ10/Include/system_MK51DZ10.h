/*
** ###################################################################
**     Processors:          MK51DN512ZCLL10
**                          MK51DX256ZCLL10
**                          MK51DN512ZCLQ10
**                          MK51DN256ZCLQ10
**                          MK51DN512ZCMC10
**                          MK51DX256ZCMC10
**                          MK51DN512ZCMD10
**                          MK51DN256ZCMD10
**                          MK51DX256ZCLK10
**                          MK51DX256ZCMB10
**
**     Compilers:           ARM Compiler
**                          Freescale C/C++ for Embedded ARM
**                          GNU C Compiler
**                          IAR ANSI C/C++ Compiler for ARM
**
**     Reference manual:    K51P144M100SF2RM Rev. 6, 6 Nov 2011
**     Version:             rev. 1.5, 2013-04-05
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
**     - rev. 1.0 (2011-06-10)
**         Initial version
**     - rev. 1.1 (2011-06-29)
**         Order of declarations changed.
**     - rev. 1.2 (2011-09-08)
**         Cortex_Core_Configuration extended with additional parameters.
**         Gap between end of interrupt vector table and flash configuration field filled by default ISR.
**     - rev. 1.3 (2012-01-10)
**         Registers updated according to the new reference manual revision - Rev. 6, 6 Nov 2011
**         ADC - PGALPb bit group added.
**         PDB - Register PO)EN renamed to POEN
**         SDHC - WRBRSTLEN bit group added
**         SIM - Bit DSPI0 in SCGC6 register renamed to SPI0.
**     - rev. 1.4 (2012-04-13)
**         Added new #define symbol MCU_MEM_MAP_VERSION_MINOR.
**         Added new #define symbols <peripheralType>_BASE_PTRS.
**     - rev. 1.5 (2013-04-05)
**         Changed start of doxygen comment.
**
** ###################################################################
*/

/*!
 * @file MK51DZ10
 * @version 1.5
 * @date 2013-04-05
 * @brief Device specific configuration file for MK51DZ10 (header file)
 *
 * Provides a system configuration function and a global variable that contains
 * the system frequency. It configures the device and initializes the oscillator
 * (PLL) that is part of the microcontroller device.
 */

#ifndef SYSTEM_MK51DZ10_H_
#define SYSTEM_MK51DZ10_H_                       /**< Symbol preventing repeated inclusion */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief System clock frequency (core clock)
 *
 * The system clock frequency supplied to the SysTick timer and the processor
 * core clock. This variable can be used by the user application to setup the
 * SysTick timer or configure other parameters. It may also be used by debugger to
 * query the frequency of the debug timer or configure the trace clock speed
 * SystemCoreClock is initialized with a correct predefined value.
 */
extern uint32_t SystemCoreClock;

/**
 * @brief Setup the microcontroller system.
 *
 * Typically this function configures the oscillator (PLL) that is part of the
 * microcontroller device. For systems with variable clock speed it also updates
 * the variable SystemCoreClock. SystemInit is called from startup_device file.
 */
void SystemInit (void);

/**
 * @brief Updates the SystemCoreClock variable.
 *
 * It must be called whenever the core clock is changed during program
 * execution. SystemCoreClockUpdate() evaluates the clock register settings and calculates
 * the current core clock.
 */
void SystemCoreClockUpdate (void);

#ifdef __cplusplus
}
#endif

#endif  /* #if !defined(SYSTEM_MK51DZ10_H_) */
