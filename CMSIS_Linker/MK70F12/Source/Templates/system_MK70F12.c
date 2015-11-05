/*
** ###################################################################
**     Compilers:           ARM Compiler
**                          Freescale C/C++ for Embedded ARM
**                          GNU C Compiler
**                          GNU C Compiler - CodeSourcery Sourcery G++
**                          IAR ANSI C/C++ Compiler for ARM
**
**     Reference manual:    K70P256M150SF3RM, Rev. 2, Dec 2011
**     Version:             rev. 2.1, 2013-10-29
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
**     - rev. 1.0 (2011-08-24)
**         Initial version
**     - rev. 1.1 (2011-11-03)
**         Registers updated according to the new reference manual revision - Rev. 1, Oct 2011
**         Registers of the following modules have been updated - CAN, DDR, I2S, LCDC, MCG, MPU, NFC, RCM, RTC, SDHC, SIM, USBHS, WDOG
**     - rev. 1.2 (2012-01-04)
**         Registers updated according to the new reference manual revision - Rev. 2, Dec 2011
**         EWM - INTEN bit in EWM_CTRL register has been added.
**         PDB - register PDB_PO0EN renamed to PRB_POEN.
**         PMC - BGEN bit in PMC_REGSC register has been removed.
**         SIM - several changes in SCGC registers. Bit USBHS in SOPT2 register removed.
**         UART - new bits RXOFE in regiter CFIFO and RXOF in register SFIFO.
**         DRY - bit THYD in CR register renamed to THYS. Bit group KSL in LR register removed.
**         LCDC - bits GWLPM in LSR register, ERR_RES_EN, GW_ERR_RES_EN in LIER and ERR_RES, GW_ERR_RES in LISR removed.
**     - rev. 1.3 (2012-04-13)
**         Added new #define symbol MCU_MEM_MAP_VERSION_MINOR.
**         Added new #define symbols <peripheralType>_BASE_PTRS.
**     - rev. 1.4 (2012-09-17)
**         DRY module removed.
**     - rev. 1.5 (2012-10-19)
**         RTC - security related registers removed.
**     - rev. 1.6 (2013-04-05)
**         Changed start of doxygen comment.
**     - rev. 1.7 (2013-06-24)
**         MPU - missing region descriptor registers added.
**     - rev. 2.0 (2013-10-29)
**         Register accessor macros added to the memory map.
**         Symbols for Processor Expert memory map compatibility added to the memory map.
**         Startup file for gcc has been updated according to CMSIS 3.2.
**         System initialization updated.
**     - rev. 2.1 (2013-10-29)
**         Definition of BITBAND macros updated to support peripherals with 32-bit acces disabled.
**
** ###################################################################
*/

/*!
 * @file MK70F12
 * @version 2.1
 * @date 2013-10-29
 * @brief Device specific configuration file for MK70F12 (implementation file)
 *
 * Provides a system configuration function and a global variable that contains
 * the system frequency. It configures the device and initializes the oscillator
 * (PLL) that is part of the microcontroller device.
 */

#include <stdint.h>
#include "MK70F12.h"

#define DISABLE_WDOG    1


#define CLOCK_SETUP     0
/* Predefined clock setups
   0 ... Multipurpose Clock Generator (MCG) in FLL Engaged Internal (FEI) mode
         Default  part configuration.
         Reference clock source for MCG module is the slow internal clock source 32.768kHz
         Core clock = 20.97MHz, BusClock = 20.97MHz
   1 ... Multipurpose Clock Generator (MCG) in PLL Engaged External (PEE) mode
         Maximum achievable clock frequency configuration.
         Reference clock source for MCG module is an external clock source 50MHz
         Core clock = 120MHz, BusClock = 60MHz
   2 ... Multipurpose Clock Generator (MCG) in Bypassed Low Power Internal (BLPI) mode
         Core clock/Bus clock derived directly from an fast internal clock 4MHz with no multiplication
         The clock settings is ready for Very Low Power Run mode.
         Core clock = 4MHz, BusClock = 4MHz
   3 ... Multipurpose Clock Generator (MCG) in Bypassed Low Power External (BLPE) mode
         Core clock/Bus clock derived directly from the RTC oscillator clock source 32.768kHz
         The clock settings is ready for Very Low Power Run mode.
         Core clock = 32.768kHz, BusClock = 32.768kHz
   4 ... Multipurpose Clock Generator (MCG) in PLL Engaged External (PEE) mode
         USB clock setup
         USB clock divider is set for USB to receive 48MHz input clock.
         Reference clock source for MCG module is an external clock source 50MHz
         USB clock divider is set for USB to receive 48MHz input clock.
         Core clock = 120MHz, BusClock = 60MHz
*/

/*----------------------------------------------------------------------------
  Define clock source values
 *----------------------------------------------------------------------------*/
#if (CLOCK_SETUP == 0)
    #define CPU_XTAL0_CLK_HZ                50000000u /* Value of the external crystal or oscillator clock frequency in Hz connected to System Oscillator 0 */
    #define CPU_XTAL1_CLK_HZ                8000000u  /* Value of the external crystal or oscillator clock frequency in Hz connected to System Oscillator 1 */
    #define CPU_XTAL32k_CLK_HZ              32768u   /* Value of the external 32k crystal or oscillator clock frequency in Hz */
    #define CPU_INT_SLOW_CLK_HZ             32768u   /* Value of the slow internal oscillator clock frequency in Hz  */
    #define CPU_INT_FAST_CLK_HZ             4000000u /* Value of the fast internal oscillator clock frequency in Hz  */
    #define DEFAULT_SYSTEM_CLOCK            20485760u /* Default System clock value */
#elif (CLOCK_SETUP == 1)
    #define CPU_XTAL0_CLK_HZ                50000000u /* Value of the external crystal or oscillator clock frequency in Hz connected to System Oscillator 0 */
    #define CPU_XTAL1_CLK_HZ                8000000u  /* Value of the external crystal or oscillator clock frequency in Hz connected to System Oscillator 1 */
    #define CPU_XTAL32k_CLK_HZ              32768u   /* Value of the external 32k crystal or oscillator clock frequency in Hz */
    #define CPU_INT_SLOW_CLK_HZ             32768u   /* Value of the slow internal oscillator clock frequency in Hz  */
    #define CPU_INT_FAST_CLK_HZ             4000000u /* Value of the fast internal oscillator clock frequency in Hz  */
    #define DEFAULT_SYSTEM_CLOCK            120000000u /* Default System clock value */
#elif (CLOCK_SETUP == 2)
    #define CPU_XTAL0_CLK_HZ                50000000u /* Value of the external crystal or oscillator clock frequency in Hz connected to System Oscillator 0 */
    #define CPU_XTAL1_CLK_HZ                8000000u  /* Value of the external crystal or oscillator clock frequency in Hz connected to System Oscillator 1 */
    #define CPU_XTAL32k_CLK_HZ              32768u   /* Value of the external 32k crystal or oscillator clock frequency in Hz */
    #define CPU_INT_SLOW_CLK_HZ             32768u   /* Value of the slow internal oscillator clock frequency in Hz  */
    #define CPU_INT_FAST_CLK_HZ             4000000u /* Value of the fast internal oscillator clock frequency in Hz  */
    #define DEFAULT_SYSTEM_CLOCK            4000000u /* Default System clock value */
#elif (CLOCK_SETUP == 3)
    #define CPU_XTAL0_CLK_HZ                50000000u /* Value of the external crystal or oscillator clock frequency in Hz connected to System Oscillator 0 */
    #define CPU_XTAL1_CLK_HZ                8000000u  /* Value of the external crystal or oscillator clock frequency in Hz connected to System Oscillator 1 */
    #define CPU_XTAL32k_CLK_HZ              32768u   /* Value of the external 32k crystal or oscillator clock frequency in Hz */
    #define CPU_INT_SLOW_CLK_HZ             32768u   /* Value of the slow internal oscillator clock frequency in Hz  */
    #define CPU_INT_FAST_CLK_HZ             4000000u /* Value of the fast internal oscillator clock frequency in Hz  */
    #define DEFAULT_SYSTEM_CLOCK            32768u /* Default System clock value */
#elif (CLOCK_SETUP == 4)
    #define CPU_XTAL0_CLK_HZ                50000000u /* Value of the external crystal or oscillator clock frequency in Hz connected to System Oscillator 0 */
    #define CPU_XTAL1_CLK_HZ                8000000u  /* Value of the external crystal or oscillator clock frequency in Hz connected to System Oscillator 1 */
    #define CPU_XTAL32k_CLK_HZ              32768u   /* Value of the external 32k crystal or oscillator clock frequency in Hz */
    #define CPU_INT_SLOW_CLK_HZ             32768u   /* Value of the slow internal oscillator clock frequency in Hz  */
    #define CPU_INT_FAST_CLK_HZ             4000000u /* Value of the fast internal oscillator clock frequency in Hz  */
    #define DEFAULT_SYSTEM_CLOCK            120000000u /* Default System clock value */
#endif /* (CLOCK_SETUP == 4) */


/* ----------------------------------------------------------------------------
   -- Core clock
   ---------------------------------------------------------------------------- */

uint32_t SystemCoreClock = DEFAULT_SYSTEM_CLOCK;

/* ----------------------------------------------------------------------------
   -- SystemInit()
   ---------------------------------------------------------------------------- */

void SystemInit (void) {
#if ((__FPU_PRESENT == 1) && (__FPU_USED == 1))
  SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));    /* set CP10, CP11 Full Access */
#endif /* ((__FPU_PRESENT == 1) && (__FPU_USED == 1)) */
#if (DISABLE_WDOG)
  /* Disable the WDOG module */
  /* WDOG_UNLOCK: WDOGUNLOCK=0xC520 */
  WDOG->UNLOCK = (uint16_t)0xC520u;     /* Key 1 */
  /* WDOG_UNLOCK : WDOGUNLOCK=0xD928 */
  WDOG->UNLOCK  = (uint16_t)0xD928u;    /* Key 2 */
  /* WDOG_STCTRLH: ??=0,DISTESTWDOG=0,BYTESEL=0,TESTSEL=0,TESTWDOG=0,??=0,STNDBYEN=1,WAITEN=1,STOPEN=1,DBGEN=0,ALLOWUPDATE=1,WINEN=0,IRQRSTEN=0,CLKSRC=1,WDOGEN=0 */
  WDOG->STCTRLH = (uint16_t)0x01D2u;
#endif /* (DISABLE_WDOG) */

  /* System clock initialization */
#if (CLOCK_SETUP == 0)
  /* SIM->SCGC5: PORTA=1 */
  SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;   /* Enable clock gate for ports to enable pin routing */
  /* SIM->CLKDIV1: OUTDIV1=0,OUTDIV2=0,OUTDIV3=1,OUTDIV4=1,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0 */
  SIM->CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0x00) |
                SIM_CLKDIV1_OUTDIV2(0x00) |
                SIM_CLKDIV1_OUTDIV3(0x01) |
                SIM_CLKDIV1_OUTDIV4(0x01); /* Update system prescalers */
  /* SIM->SOPT2: PLLFLLSEL=0 */
  SIM->SOPT2 &= (uint32_t)~(uint32_t)(SIM_SOPT2_PLLFLLSEL(0x03)); /* Select FLL as a clock source for various peripherals */
  /* SIM->SOPT1: OSC32KSEL=0 */
  SIM->SOPT1 &= (uint32_t)~(uint32_t)(SIM_SOPT1_OSC32KSEL_MASK); /* System oscillator drives 32 kHz clock for various peripherals */
  /* SIM->SCGC1: OSC1=1 */
  SIM->SCGC1 |= SIM_SCGC1_OSC1_MASK;
  /* Switch to FEI Mode */
  /* MCG->C1: CLKS=0,FRDIV=0,IREFS=1,IRCLKEN=1,IREFSTEN=0 */
  MCG->C1 = MCG_C1_CLKS(0x00) |
           MCG_C1_FRDIV(0x00) |
           MCG_C1_IREFS_MASK |
           MCG_C1_IRCLKEN_MASK;
  /* MCG->C2: LOCRE0=0,??=0,RANGE0=0,HGO0=0,EREFS0=0,LP=0,IRCS=0 */
  MCG->C2 = MCG_C2_RANGE0(0x00);
  /* MCG->C4: DMX32=0,DRST_DRS=0 */
  MCG->C4 &= (uint8_t)~(uint8_t)((MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS(0x03)));
  /* OSC0->CR: ERCLKEN=1,??=0,EREFSTEN=0,??=0,SC2P=0,SC4P=0,SC8P=0,SC16P=0 */
  OSC0->CR = OSC_CR_ERCLKEN_MASK;
  /* OSC1->CR: ERCLKEN=1,??=0,EREFSTEN=0,??=0,SC2P=0,SC4P=0,SC8P=0,SC16P=0 */
  OSC1->CR = OSC_CR_ERCLKEN_MASK;
  /* MCG->C7: OSCSEL=0 */
  MCG->C7 &= (uint8_t)~(uint8_t)(MCG_C7_OSCSEL_MASK);
  /* MCG->C5: PLLREFSEL0=0,PLLCLKEN0=0,PLLSTEN0=0,??=0,??=0,PRDIV0=0 */
  MCG->C5 = MCG_C5_PRDIV0(0x00);
  /* MCG->C6: LOLIE0=0,PLLS=0,CME0=0,VDIV0=0 */
  MCG->C6 = MCG_C6_VDIV0(0x00);
  /* MCG->C11: PLLREFSEL1=0,PLLCLKEN1=0,PLLSTEN1=0,PLLCS=0,??=0,PRDIV1=0 */
  MCG->C11 = MCG_C11_PRDIV1(0x00);
  /* MCG->C12: LOLIE1=0,??=0,CME2=0,VDIV1=0 */
  MCG->C12 = MCG_C12_VDIV1(0x00);
  while((MCG->S & MCG_S_IREFST_MASK) == 0x00U) { /* Check that the source of the FLL reference clock is the internal reference clock. */
  }
  while((MCG->S & 0x0CU) != 0x00U) {    /* Wait until output of the FLL is selected */
  }
#elif ((CLOCK_SETUP == 1) || (CLOCK_SETUP == 4))
  /* SIM->SCGC5: PORTE=1,PORTA=1 */
  SIM->SCGC5 |= (SIM_SCGC5_PORTE_MASK | SIM_SCGC5_PORTA_MASK); /* Enable clock gate for ports to enable pin routing */
  /* SIM->CLKDIV1: OUTDIV1=0,OUTDIV2=1,OUTDIV3=2,OUTDIV4=5,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0 */
  SIM->CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0x00) |
                SIM_CLKDIV1_OUTDIV2(0x01) |
                SIM_CLKDIV1_OUTDIV3(0x02) |
                SIM_CLKDIV1_OUTDIV4(0x05); /* Update system prescalers */
  /* SIM->SOPT2: PLLFLLSEL=1 */
  SIM->SOPT2 = (uint32_t)((SIM->SOPT2 & (uint32_t)~(uint32_t)(
               SIM_SOPT2_PLLFLLSEL(0x02)
              )) | (uint32_t)(
               SIM_SOPT2_PLLFLLSEL(0x01)
              ));                      /* Select PLL 0 as a clock source for various peripherals */
  /* SIM_SOPT1: OSC32KSEL=0 */
  SIM->SOPT1 &= (uint32_t)~(uint32_t)(SIM_SOPT1_OSC32KSEL_MASK); /* System oscillator drives 32 kHz clock for various peripherals */
  /* SIM->SCGC1: OSC1=1 */
  SIM->SCGC1 |= SIM_SCGC1_OSC1_MASK;
  /* PORTA->PCR[18]: ISF=0,MUX=0 */
  PORTA->PCR[18] &= (uint32_t)~(uint32_t)((PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07)));
  /* PORTE->PCR[24]: ISF=0,MUX=0 */
  PORTE->PCR[24] &= (uint32_t)~(uint32_t)((PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07)));
  /* PORTE->PCR[25]: ISF=0,MUX=0 */
  PORTE->PCR[25] &= (uint32_t)~(uint32_t)((PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07)));
  /* Switch to FBE Mode */
  /* MCG->C7: OSCSEL=0 */
  MCG->C7 &= (uint8_t)~(uint8_t)(MCG_C7_OSCSEL_MASK);
  /* MCG->C10: LOCRE2=0,??=0,RANGE1=2,HGO1=0,EREFS1=1,??=0,??=0 */
  MCG->C10 = (MCG_C10_RANGE1(0x02) | MCG_C10_EREFS1_MASK);
  /* MCG->C2: LOCRE0=0,??=0,RANGE0=2,HGO0=0,EREFS0=0,LP=0,IRCS=0 */
  MCG->C2 = MCG_C2_RANGE0(0x02);
  /* OSC0->CR: ERCLKEN=1,??=0,EREFSTEN=0,??=0,SC2P=0,SC4P=0,SC8P=0,SC16P=0 */
  OSC0->CR = OSC_CR_ERCLKEN_MASK;
  /* OSC1->CR: ERCLKEN=1,??=0,EREFSTEN=0,??=0,SC2P=0,SC4P=0,SC8P=0,SC16P=0 */
  OSC1->CR = OSC_CR_ERCLKEN_MASK;
  /* MCG->C1: CLKS=2,FRDIV=5,IREFS=0,IRCLKEN=1,IREFSTEN=0 */
  MCG->C1 = (MCG_C1_CLKS(0x02) | MCG_C1_FRDIV(0x05) | MCG_C1_IRCLKEN_MASK);
  /* MCG->C4: DMX32=0,DRST_DRS=0 */
  MCG->C4 &= (uint8_t)~(uint8_t)((MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS(0x03)));
  /* MCG->C5: PLLREFSEL0=0,PLLCLKEN0=0,PLLSTEN0=0,??=0,??=0,PRDIV0=4 */
  MCG->C5 = MCG_C5_PRDIV0(0x04);
  /* MCG->C6: LOLIE0=0,PLLS=0,CME0=0,VDIV0=8 */
  MCG->C6 = MCG_C6_VDIV0(0x08);
  /* MCG->C11: PLLREFSEL1=1,PLLCLKEN1=0,PLLSTEN1=0,PLLCS=0,??=0,PRDIV1=0 */
  MCG->C11 = (MCG_C11_PLLREFSEL1_MASK | MCG_C11_PRDIV1(0x00));
  /* MCG->C11: PLLCLKEN1=1 */
  MCG->C11 |= MCG_C11_PLLCLKEN1_MASK;   /* Enable the PLL */
  /* MCG->C12: LOLIE1=0,??=0,CME2=0,VDIV1=0x0E */
  MCG->C12 = MCG_C12_VDIV1(0x0E);
  while((MCG->S & MCG_S_IREFST_MASK) != 0x00U) { /* Check that the source of the FLL reference clock is the external reference clock. */
  }
  while((MCG->S & 0x0CU) != 0x08U) {    /* Wait until external reference clock is selected as MCG output */
  }
  /* Switch to PBE Mode */
  /* MCG->C6: LOLIE0=0,PLLS=1,CME0=0,VDIV0=8 */
  MCG->C6 = (MCG_C6_PLLS_MASK | MCG_C6_VDIV0(0x08));
  while((MCG->S & 0x0CU) != 0x08U) {    /* Wait until external reference clock is selected as MCG output */
  }
  while((MCG->S & MCG_S_LOCK0_MASK) == 0x00U) { /* Wait until PLL locked */
  }
  /* Switch to PEE Mode */
  /* MCG->C1: CLKS=0,FRDIV=5,IREFS=0,IRCLKEN=1,IREFSTEN=0 */
  MCG->C1 = (MCG_C1_CLKS(0x00) | MCG_C1_FRDIV(0x05) | MCG_C1_IRCLKEN_MASK);
  while((MCG->S & 0x0CU) != 0x0CU) {    /* Wait until output of the PLL is selected */
  }
  #if (CLOCK_SETUP == 4)
  /* SIM->CLKDIV2: USBHSDIV=1,USBHSFRAC=0,USBFSDIV=4,USBFSFRAC=1 */
  SIM->CLKDIV2 = (uint32_t)((SIM->CLKDIV2 & (uint32_t)~(uint32_t)(
                 SIM_CLKDIV2_USBHSDIV(0x06) |
                 SIM_CLKDIV2_USBHSFRAC_MASK |
                 SIM_CLKDIV2_USBFSDIV(0x03)
                )) | (uint32_t)(
                 SIM_CLKDIV2_USBHSDIV(0x01) |
                 SIM_CLKDIV2_USBFSDIV(0x04) |
                 SIM_CLKDIV2_USBFSFRAC_MASK
                ));
  #endif
#elif (CLOCK_SETUP == 2)
  /* SIM->SCGC5: PORTA=1 */
  SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;   /* Enable clock gate for ports to enable pin routing */
  /* SIM->CLKDIV1: OUTDIV1=0,OUTDIV2=0,OUTDIV3=0,OUTDIV4=1,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0 */
  SIM->CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0x00) |
                SIM_CLKDIV1_OUTDIV2(0x00) |
                SIM_CLKDIV1_OUTDIV3(0x00) |
                SIM_CLKDIV1_OUTDIV4(0x04); /* Update system prescalers */
  /* SIM->SOPT2: PLLFLLSEL=0 */
  SIM->SOPT2 &= (uint32_t)~(uint32_t)(SIM_SOPT2_PLLFLLSEL(0x03)); /* Select FLL as a clock source for various peripherals */
  /* SIM->SOPT1: OSC32KSEL=0 */
  SIM->SOPT1 &= (uint32_t)~(uint32_t)(SIM_SOPT1_OSC32KSEL_MASK); /* System oscillator drives 32 kHz clock for various peripherals */
  /* SIM->SCGC1: OSC1=1 */
  SIM->SCGC1 |= SIM_SCGC1_OSC1_MASK;
  /* MCG->SC: FCRDIV=0 */
  MCG->SC &= (uint8_t)~(uint8_t)(MCG_SC_FCRDIV(0x07));
  /* Switch to FBI Mode */
  /* MCG->C1: CLKS=1,FRDIV=0,IREFS=1,IRCLKEN=1,IREFSTEN=0 */
  MCG->C1 = MCG_C1_CLKS(0x01) |
           MCG_C1_FRDIV(0x00) |
           MCG_C1_IREFS_MASK |
           MCG_C1_IRCLKEN_MASK;
  /* MCG->C2: LOCRE0=0,??=0,RANGE0=0,HGO0=0,EREFS0=0,LP=0,IRCS=1 */
  MCG->C2 = (MCG_C2_RANGE0(0x00) | MCG_C2_IRCS_MASK);
  /* MCG->C4: DMX32=0,DRST_DRS=0 */
  MCG->C4 &= (uint8_t)~(uint8_t)((MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS(0x03)));
  /* OSC0->CR: ERCLKEN=1,??=0,EREFSTEN=0,??=0,SC2P=0,SC4P=0,SC8P=0,SC16P=0 */
  OSC0->CR = OSC_CR_ERCLKEN_MASK;
  /* OSC1->CR: ERCLKEN=1,??=0,EREFSTEN=0,??=0,SC2P=0,SC4P=0,SC8P=0,SC16P=0 */
  OSC1->CR = OSC_CR_ERCLKEN_MASK;
  /* MCG->C7: OSCSEL=0 */
  MCG->C7 &= (uint8_t)~(uint8_t)(MCG_C7_OSCSEL_MASK);
  /* MCG->C5: PLLREFSEL0=0,PLLCLKEN0=0,PLLSTEN0=0,??=0,??=0,PRDIV0=0 */
  MCG->C5 = MCG_C5_PRDIV0(0x00);
  /* MCG->C6: LOLIE0=0,PLLS=0,CME0=0,VDIV0=0 */
  MCG->C6 = MCG_C6_VDIV0(0x00);
  /* MCG->C11: PLLREFSEL1=0,PLLCLKEN1=0,PLLSTEN1=0,PLLCS=0,??=0,PRDIV1=0 */
  MCG->C11 = MCG_C11_PRDIV1(0x00);
  /* MCG->C12: LOLIE1=0,??=0,CME2=0,VDIV1=0 */
  MCG->C12 = MCG_C12_VDIV1(0x00);
  while((MCG->S & MCG_S_IREFST_MASK) == 0x00U) { /* Check that the source of the FLL reference clock is the internal reference clock. */
  }
  while((MCG->S & 0x0CU) != 0x04U) {    /* Wait until internal reference clock is selected as MCG output */
  }
  /* Switch to BLPI Mode */
  /* MCG->C2: LOCRE0=0,??=0,RANGE0=0,HGO0=0,EREFS0=0,LP=1,IRCS=1 */
  MCG->C2 = (MCG_C2_RANGE0(0x00) | MCG_C2_LP_MASK | MCG_C2_IRCS_MASK);
  while((MCG->S & MCG_S_IREFST_MASK) == 0x00U) { /* Check that the source of the FLL reference clock is the internal reference clock. */
  }
  while((MCG->S & MCG_S_IRCST_MASK) == 0x00U) { /* Check that the fast external reference clock is selected. */
  }
#elif (CLOCK_SETUP == 3)
  /* SIM->SCGC6: RTC=1 */
  SIM->SCGC6 |= SIM_SCGC6_RTC_MASK;
  if ((RTC->CR & RTC_CR_OSCE_MASK) == 0u) { /* Only if the OSCILLATOR is not already enabled */
    /* RTC->CR: SC2P=0,SC4P=0,SC8P=0,SC16P=0 */
    RTC->CR &= (uint32_t)~(uint32_t)(
               RTC_CR_SC2P_MASK |
               RTC_CR_SC4P_MASK |
               RTC_CR_SC8P_MASK |
               RTC_CR_SC16P_MASK
              );
    /* RTC->CR: OSCE=1 */
    RTC->CR |= RTC_CR_OSCE_MASK;
    /* RTC->CR: CLKO=0 */
    RTC->CR &= (uint32_t)~(uint32_t)(RTC_CR_CLKO_MASK);
  }
  /* SIM->SCGC5: PORTA=1 */
  SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;   /* Enable clock gate for ports to enable pin routing */
  /* SIM->CLKDIV1: OUTDIV1=0,OUTDIV2=0,OUTDIV3=0,OUTDIV4=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0 */
  SIM->CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0x00) |
                SIM_CLKDIV1_OUTDIV2(0x00) |
                SIM_CLKDIV1_OUTDIV3(0x00) |
                SIM_CLKDIV1_OUTDIV4(0x00); /* Update system prescalers */
  /* SIM->SOPT2: PLLFLLSEL=0 */
  SIM->SOPT2 &= (uint32_t)~(uint32_t)(SIM_SOPT2_PLLFLLSEL(0x03)); /* Select FLL as a clock source for various peripherals */
  /* SIM->SOPT1: OSC32KSEL=0 */
  SIM->SOPT1 &= (uint32_t)~(uint32_t)(SIM_SOPT1_OSC32KSEL_MASK); /* System oscillator drives 32 kHz clock for various peripherals */
  /* SIM->SCGC1: OSC1=1 */
  SIM->SCGC1 |= SIM_SCGC1_OSC1_MASK;
  /* Switch to FBE Mode */
  /* MCG->C7: OSCSEL=1 */
  MCG->C7 |= MCG_C7_OSCSEL_MASK;
  /* MCG->C2: LOCRE0=0,??=0,RANGE0=0,HGO0=0,EREFS0=0,LP=0,IRCS=1 */
  MCG->C2 = (MCG_C2_RANGE0(0x00) | MCG_C2_IRCS_MASK);
  /* OSC0->CR: ERCLKEN=1,??=0,EREFSTEN=0,??=0,SC2P=0,SC4P=0,SC8P=0,SC16P=0 */
  OSC0->CR = OSC_CR_ERCLKEN_MASK;
  /* OSC1->CR: ERCLKEN=1,??=0,EREFSTEN=0,??=0,SC2P=0,SC4P=0,SC8P=0,SC16P=0 */
  OSC1->CR = OSC_CR_ERCLKEN_MASK;
  /* MCG->C1: CLKS=2,FRDIV=0,IREFS=0,IRCLKEN=1,IREFSTEN=0 */
  MCG->C1 = (MCG_C1_CLKS(0x02) | MCG_C1_FRDIV(0x00) | MCG_C1_IRCLKEN_MASK);
  /* MCG->C4: DMX32=0,DRST_DRS=0 */
  MCG->C4 &= (uint8_t)~(uint8_t)((MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS(0x03)));
  /* MCG->C5: PLLREFSEL0=0,PLLCLKEN0=0,PLLSTEN0=0,??=0,??=0,PRDIV0=0 */
  MCG->C5 = MCG_C5_PRDIV0(0x00);
  /* MCG->C6: LOLIE0=0,PLLS=0,CME0=0,VDIV0=0 */
  MCG->C6 = MCG_C6_VDIV0(0x00);
  /* MCG->C11: PLLREFSEL1=0,PLLCLKEN1=0,PLLSTEN1=0,PLLCS=0,??=0,PRDIV1=0 */
  MCG->C11 = MCG_C11_PRDIV1(0x00);
  /* MCG->C12: LOLIE1=0,??=0,CME2=0,VDIV1=0 */
  MCG->C12 = MCG_C12_VDIV1(0x00);
  while((MCG->S & MCG_S_IREFST_MASK) != 0x00U) { /* Check that the source of the FLL reference clock is the external reference clock. */
  }
  while((MCG->S & 0x0CU) != 0x08U) {    /* Wait until external reference clock is selected as MCG output */
  }
  /* Switch to BLPE Mode */
  /* MCG->C2: LOCRE0=0,??=0,RANGE0=0,HGO0=0,EREFS0=0,LP=1,IRCS=1 */
  MCG->C2 = (MCG_C2_RANGE0(0x00) | MCG_C2_LP_MASK | MCG_C2_IRCS_MASK);
  while((MCG->S & 0x0CU) != 0x08U) {    /* Wait until external reference clock is selected as MCG output */
  }
#endif /* (CLOCK_SETUP == 3) */
}

/* ----------------------------------------------------------------------------
   -- SystemCoreClockUpdate()
   ---------------------------------------------------------------------------- */

void SystemCoreClockUpdate (void) {
  uint32_t MCGOUTClock;                                                        /* Variable to store output clock frequency of the MCG module */
  uint8_t Divider;

  if ((MCG->C1 & MCG_C1_CLKS_MASK) == 0x0u) {
    /* Output of FLL or PLL is selected */
    if ((MCG->C6 & MCG_C6_PLLS_MASK) == 0x0u) {
      /* FLL is selected */
      if ((MCG->C1 & MCG_C1_IREFS_MASK) == 0x0u) {
        /* External reference clock is selected */
        if ((MCG->C7 & MCG_C7_OSCSEL_MASK) == 0x0u) {
          MCGOUTClock = CPU_XTAL0_CLK_HZ;                                       /* System oscillator 0 drives MCG clock */
        } else { /* (!((MCG->C7 & MCG_C7_OSCSEL_MASK) == 0x0u)) */
          MCGOUTClock = CPU_XTAL32k_CLK_HZ;                                    /* RTC 32 kHz oscillator drives MCG clock */
        } /* (!((MCG->C7 & MCG_C7_OSCSEL_MASK) == 0x0u)) */
        Divider = (uint8_t)(1u << ((MCG->C1 & MCG_C1_FRDIV_MASK) >> MCG_C1_FRDIV_SHIFT));
        MCGOUTClock = (MCGOUTClock / Divider);  /* Calculate the divided FLL reference clock */
        if ((MCG->C2 & MCG_C2_RANGE0_MASK) != 0x0u) {
          MCGOUTClock /= 32u;                                                  /* If high range is enabled, additional 32 divider is active */
        } /* ((MCG->C2 & MCG_C2_RANGE0_MASK) != 0x0u) */
      } else { /* (!((MCG->C1 & MCG_C1_IREFS_MASK) == 0x0u)) */
        MCGOUTClock = CPU_INT_SLOW_CLK_HZ;                                     /* The slow internal reference clock is selected */
      } /* (!((MCG->C1 & MCG_C1_IREFS_MASK) == 0x0u)) */
      /* Select correct multiplier to calculate the MCG output clock  */
      switch (MCG->C4 & (MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS_MASK)) {
        case 0x0u:
          MCGOUTClock *= 640u;
          break;
        case 0x20u:
          MCGOUTClock *= 1280u;
          break;
        case 0x40u:
          MCGOUTClock *= 1920u;
          break;
        case 0x60u:
          MCGOUTClock *= 2560u;
          break;
        case 0x80u:
          MCGOUTClock *= 732u;
          break;
        case 0xA0u:
          MCGOUTClock *= 1464u;
          break;
        case 0xC0u:
          MCGOUTClock *= 2197u;
          break;
        case 0xE0u:
          MCGOUTClock *= 2929u;
          break;
        default:
          break;
      }
    } else { /* (!((MCG->C6 & MCG_C6_PLLS_MASK) == 0x0u)) */
      /* PLL is selected */
      if ((MCG->C11 & MCG_C11_PLLCS_MASK) != 0x0u) {
        /* PLL1 output is selected */
        if ((MCG->C11 & MCG_C11_PLLREFSEL1_MASK) != 0x0u) {
          /* OSC1 clock source used as an external reference clock */
          MCGOUTClock = CPU_XTAL1_CLK_HZ;
        } else { /* (!((MCG->C11 & MCG_C11_PLLREFSEL1_MASK) != 0x0u)) */
          /* OSC0 clock source used as an external reference clock */
          MCGOUTClock = CPU_XTAL0_CLK_HZ;
        } /* (!((MCG->C11 & MCG_C11_PLLREFSEL1_MASK) != 0x0u)) */
        Divider = (1u + (MCG->C11 & MCG_C11_PRDIV1_MASK));
        MCGOUTClock /= Divider;                                                  /* Calculate the PLL reference clock */
        Divider = ((MCG->C12 & MCG_C12_VDIV1_MASK) + 16u);
        MCGOUTClock = (MCGOUTClock * Divider) / 2u;                              /* Calculate the MCG output clock */
      } else { /* (!((MCG->C11 & MCG_C11_PLLCS_MASK) != 0x0u)) */
        /* PLL0 output is selected */
        if ((MCG->C5 & MCG_C5_PLLREFSEL0_MASK) != 0x0u) {
          /* OSC1 clock source used as an external reference clock */
          MCGOUTClock = CPU_XTAL1_CLK_HZ;
        } else { /* (!((MCG->C5 & MCG_C5_PLLREFSEL0_MASK) != 0x0u)) */
          /* OSC0 clock source used as an external reference clock */
          MCGOUTClock = CPU_XTAL0_CLK_HZ;
        } /* (!((MCG->C5 & MCG_C5_PLLREFSEL0_MASK) != 0x0u)) */
        Divider = (1u + (MCG->C5 & MCG_C5_PRDIV0_MASK));
        MCGOUTClock /= Divider;                                                  /* Calculate the PLL reference clock */
        Divider = ((MCG->C6 & MCG_C6_VDIV0_MASK) + 16u);
        MCGOUTClock = (MCGOUTClock * Divider) / 2u;                              /* Calculate the MCG output clock */
      } /* (!((MCG->C11 & MCG_C11_PLLCS_MASK) != 0x0u)) */
    } /* (!((MCG->C6 & MCG_C6_PLLS_MASK) == 0x0u)) */
  } else if ((MCG->C1 & MCG_C1_CLKS_MASK) == 0x40u) {
    /* Internal reference clock is selected */
    if ((MCG->C2 & MCG_C2_IRCS_MASK) == 0x0u) {
      MCGOUTClock = CPU_INT_SLOW_CLK_HZ;                                       /* Slow internal reference clock selected */
    } else { /* (!((MCG->C2 & MCG_C2_IRCS_MASK) == 0x0u)) */
      MCGOUTClock = CPU_INT_FAST_CLK_HZ / (1 << ((MCG->SC & MCG_SC_FCRDIV_MASK) >> MCG_SC_FCRDIV_SHIFT));  /* Fast internal reference clock selected */
    } /* (!((MCG->C2 & MCG_C2_IRCS_MASK) == 0x0u)) */
  } else if ((MCG->C1 & MCG_C1_CLKS_MASK) == 0x80u) {
    /* External reference clock is selected */
    if ((MCG->C7 & MCG_C7_OSCSEL_MASK) == 0x0u) {
      MCGOUTClock = CPU_XTAL0_CLK_HZ;                                           /* System oscillator drives MCG clock */
    } else { /* (!((MCG->C7 & MCG_C7_OSCSEL_MASK) == 0x0u)) */
      MCGOUTClock = CPU_XTAL32k_CLK_HZ;                                        /* RTC 32 kHz oscillator drives MCG clock */
    } /* (!((MCG->C7 & MCG_C7_OSCSEL_MASK) == 0x0u)) */
  } else { /* (!((MCG->C1 & MCG_C1_CLKS_MASK) == 0x80u)) */
    /* Reserved value */
    return;
  } /* (!((MCG->C1 & MCG_C1_CLKS_MASK) == 0x80u)) */
  SystemCoreClock = (MCGOUTClock / (1u + ((SIM->CLKDIV1 & SIM_CLKDIV1_OUTDIV1_MASK) >> SIM_CLKDIV1_OUTDIV1_SHIFT)));
}
