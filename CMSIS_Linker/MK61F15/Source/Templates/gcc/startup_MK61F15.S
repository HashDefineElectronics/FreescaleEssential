/* ---------------------------------------------------------------------------------------*/
/*  @file:    startup_MK61F15.s                                                           */
/*  @purpose: CMSIS Cortex-M4 Core Device Startup File                                    */
/*            MK61F15                                                                     */
/*  @version: 2.0                                                                         */
/*  @date:    2014-2-3                                                                    */
/*  @build:   b140606                                                                     */
/* ---------------------------------------------------------------------------------------*/
/*                                                                                        */
/* Copyright (c) 1997 - 2014 , Freescale Semiconductor, Inc.                              */
/* All rights reserved.                                                                   */
/*                                                                                        */
/* Redistribution and use in source and binary forms, with or without modification,       */
/* are permitted provided that the following conditions are met:                          */
/*                                                                                        */
/* o Redistributions of source code must retain the above copyright notice, this list     */
/*   of conditions and the following disclaimer.                                          */
/*                                                                                        */
/* o Redistributions in binary form must reproduce the above copyright notice, this       */
/*   list of conditions and the following disclaimer in the documentation and/or          */
/*   other materials provided with the distribution.                                      */
/*                                                                                        */
/* o Neither the name of Freescale Semiconductor, Inc. nor the names of its               */
/*   contributors may be used to endorse or promote products derived from this            */
/*   software without specific prior written permission.                                  */
/*                                                                                        */
/* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND        */
/* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED          */
/* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE                 */
/* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR       */
/* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES         */
/* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;           */
/* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON         */
/* ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT                */
/* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS          */
/* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                           */
/*****************************************************************************/
/* Version: GCC for ARM Embedded Processors                                  */
/*****************************************************************************/
    .syntax unified
    .arch armv7-m

    .section .isr_vector, "a"
    .align 2
    .globl __isr_vector
__isr_vector:
    .long   __StackTop                                      /* Top of Stack */
    .long   Reset_Handler                                   /* Reset Handler */
    .long   NMI_Handler                                     /* NMI Handler*/
    .long   HardFault_Handler                               /* Hard Fault Handler*/
    .long   MemManage_Handler                               /* MPU Fault Handler*/
    .long   BusFault_Handler                                /* Bus Fault Handler*/
    .long   UsageFault_Handler                              /* Usage Fault Handler*/
    .long   0                                               /* Reserved*/
    .long   0                                               /* Reserved*/
    .long   0                                               /* Reserved*/
    .long   0                                               /* Reserved*/
    .long   SVC_Handler                                     /* SVCall Handler*/
    .long   DebugMon_Handler                                /* Debug Monitor Handler*/
    .long   0                                               /* Reserved*/
    .long   PendSV_Handler                                  /* PendSV Handler*/
    .long   SysTick_Handler                                 /* SysTick Handler*/

                                                            /* External Interrupts*/
    .long   DMA0_DMA16_IRQHandler                           /* DMA channel 0/16 transfer complete interrupt*/
    .long   DMA1_DMA17_IRQHandler                           /* DMA channel 1/17 transfer complete interrupt*/
    .long   DMA2_DMA18_IRQHandler                           /* DMA channel 2/18 transfer complete interrupt*/
    .long   DMA3_DMA19_IRQHandler                           /* DMA channel 3/19 transfer complete interrupt*/
    .long   DMA4_DMA20_IRQHandler                           /* DMA channel 4/20 transfer complete interrupt*/
    .long   DMA5_DMA21_IRQHandler                           /* DMA channel 5/21 transfer complete interrupt*/
    .long   DMA6_DMA22_IRQHandler                           /* DMA channel 6/22 transfer complete interrupt*/
    .long   DMA7_DMA23_IRQHandler                           /* DMA channel 7/23 transfer complete interrupt*/
    .long   DMA8_DMA24_IRQHandler                           /* DMA channel 8/24 transfer complete interrupt*/
    .long   DMA9_DMA25_IRQHandler                           /* DMA channel 9/25 transfer complete interrupt*/
    .long   DMA10_DMA26_IRQHandler                          /* DMA channel 10/26 transfer complete interrupt*/
    .long   DMA11_DMA27_IRQHandler                          /* DMA channel 11/27 transfer complete interrupt*/
    .long   DMA12_DMA28_IRQHandler                          /* DMA channel 12/28 transfer complete interrupt*/
    .long   DMA13_DMA29_IRQHandler                          /* DMA channel 13/29 transfer complete interrupt*/
    .long   DMA14_DMA30_IRQHandler                          /* DMA channel 14/30 transfer complete interrupt*/
    .long   DMA15_DMA31_IRQHandler                          /* DMA channel 15/31 transfer complete interrupt*/
    .long   DMA_Error_IRQHandler                            /* DMA error interrupt*/
    .long   MCM_IRQHandler                                  /* Normal interrupt*/
    .long   FTFE_IRQHandler                                 /* FTFE interrupt*/
    .long   Read_Collision_IRQHandler                       /* Read collision interrupt*/
    .long   LVD_LVW_IRQHandler                              /* Low Voltage Detect, Low Voltage Warning*/
    .long   LLW_IRQHandler                                  /* Low Leakage Wakeup*/
    .long   Watchdog_IRQHandler                             /* WDOG interrupt*/
    .long   RNG_IRQHandler                                  /* RNGA interrupt*/
    .long   I2C0_IRQHandler                                 /* I2C0 interrupt*/
    .long   I2C1_IRQHandler                                 /* I2C1 interrupt*/
    .long   SPI0_IRQHandler                                 /* SPI0 interrupt*/
    .long   SPI1_IRQHandler                                 /* SPI1 interrupt*/
    .long   SPI2_IRQHandler                                 /* SPI2 interrupt*/
    .long   CAN0_ORed_Message_buffer_IRQHandler             /* CAN0 OR'd message buffers interrupt*/
    .long   CAN0_Bus_Off_IRQHandler                         /* CAN0 bus off interrupt*/
    .long   CAN0_Error_IRQHandler                           /* CAN0 error interrupt*/
    .long   CAN0_Tx_Warning_IRQHandler                      /* CAN0 Tx warning interrupt*/
    .long   CAN0_Rx_Warning_IRQHandler                      /* CAN0 Rx warning interrupt*/
    .long   CAN0_Wake_Up_IRQHandler                         /* CAN0 wake up interrupt*/
    .long   I2S0_Tx_IRQHandler                              /* I2S0 transmit interrupt*/
    .long   I2S0_Rx_IRQHandler                              /* I2S0 receive interrupt*/
    .long   CAN1_ORed_Message_buffer_IRQHandler             /* CAN1 OR'd message buffers interrupt*/
    .long   CAN1_Bus_Off_IRQHandler                         /* CAN1 bus off interrupt*/
    .long   CAN1_Error_IRQHandler                           /* CAN1 error interrupt*/
    .long   CAN1_Tx_Warning_IRQHandler                      /* CAN1 Tx warning interrupt*/
    .long   CAN1_Rx_Warning_IRQHandler                      /* CAN1 Rx warning interrupt*/
    .long   CAN1_Wake_Up_IRQHandler                         /* CAN1 wake up interrupt*/
    .long   Reserved59_IRQHandler                           /* Reserved interrupt 59*/
    .long   UART0_LON_IRQHandler                            /* UART0 LON interrupt*/
    .long   UART0_RX_TX_IRQHandler                          /* UART0 receive/transmit interrupt*/
    .long   UART0_ERR_IRQHandler                            /* UART0 error interrupt*/
    .long   UART1_RX_TX_IRQHandler                          /* UART1 receive/transmit interrupt*/
    .long   UART1_ERR_IRQHandler                            /* UART1 error interrupt*/
    .long   UART2_RX_TX_IRQHandler                          /* UART2 receive/transmit interrupt*/
    .long   UART2_ERR_IRQHandler                            /* UART2 error interrupt*/
    .long   UART3_RX_TX_IRQHandler                          /* UART3 receive/transmit interrupt*/
    .long   UART3_ERR_IRQHandler                            /* UART3 error interrupt*/
    .long   UART4_RX_TX_IRQHandler                          /* UART4 receive/transmit interrupt*/
    .long   UART4_ERR_IRQHandler                            /* UART4 error interrupt*/
    .long   UART5_RX_TX_IRQHandler                          /* UART5 receive/transmit interrupt*/
    .long   UART5_ERR_IRQHandler                            /* UART5 error interrupt*/
    .long   ADC0_IRQHandler                                 /* ADC0 interrupt*/
    .long   ADC1_IRQHandler                                 /* ADC1 interrupt*/
    .long   CMP0_IRQHandler                                 /* CMP0 interrupt*/
    .long   CMP1_IRQHandler                                 /* CMP1 interrupt*/
    .long   CMP2_IRQHandler                                 /* CMP2 interrupt*/
    .long   FTM0_IRQHandler                                 /* FTM0 fault, overflow and channels interrupt*/
    .long   FTM1_IRQHandler                                 /* FTM1 fault, overflow and channels interrupt*/
    .long   FTM2_IRQHandler                                 /* FTM2 fault, overflow and channels interrupt*/
    .long   CMT_IRQHandler                                  /* CMT interrupt*/
    .long   RTC_IRQHandler                                  /* RTC interrupt*/
    .long   RTC_Seconds_IRQHandler                          /* RTC seconds interrupt*/
    .long   PIT0_IRQHandler                                 /* PIT timer channel 0 interrupt*/
    .long   PIT1_IRQHandler                                 /* PIT timer channel 1 interrupt*/
    .long   PIT2_IRQHandler                                 /* PIT timer channel 2 interrupt*/
    .long   PIT3_IRQHandler                                 /* PIT timer channel 3 interrupt*/
    .long   PDB0_IRQHandler                                 /* PDB0 interrupt*/
    .long   USB0_IRQHandler                                 /* USB0 interrupt*/
    .long   USBDCD_IRQHandler                               /* USBDCD interrupt*/
    .long   ENET_1588_Timer_IRQHandler                      /* Ethernet MAC IEEE 1588 timer interrupt*/
    .long   ENET_Transmit_IRQHandler                        /* Ethernet MAC transmit interrupt*/
    .long   ENET_Receive_IRQHandler                         /* Ethernet MAC receive interrupt*/
    .long   ENET_Error_IRQHandler                           /* Ethernet MAC error and miscelaneous interrupt*/
    .long   Reserved95_IRQHandler                           /* Reserved interrupt 95*/
    .long   SDHC_IRQHandler                                 /* SDHC interrupt*/
    .long   DAC0_IRQHandler                                 /* DAC0 interrupt*/
    .long   DAC1_IRQHandler                                 /* DAC1 interrupt*/
    .long   TSI0_IRQHandler                                 /* TSI0 interrupt*/
    .long   MCG_IRQHandler                                  /* MCG interrupt*/
    .long   LPTimer_IRQHandler                              /* LPTimer interrupt*/
    .long   Reserved102_IRQHandler                          /* Reserved interrupt 102*/
    .long   PORTA_IRQHandler                                /* Port A interrupt*/
    .long   PORTB_IRQHandler                                /* Port B interrupt*/
    .long   PORTC_IRQHandler                                /* Port C interrupt*/
    .long   PORTD_IRQHandler                                /* Port D interrupt*/
    .long   PORTE_IRQHandler                                /* Port E interrupt*/
    .long   PORTF_IRQHandler                                /* Port F interrupt*/
    .long   DDR_IRQHandler                                  /* DDR interrupt*/
    .long   SWI_IRQHandler                                  /* Software interrupt*/
    .long   NFC_IRQHandler                                  /* NAND flash controller interrupt*/
    .long   USBHS_IRQHandler                                /* USB high speed OTG interrupt*/
    .long   Reserved113_IRQHandler                          /* Reserved interrupt 113*/
    .long   CMP3_IRQHandler                                 /* CMP3 interrupt*/
    .long   Reserved115_IRQHandler                          /* Reserved interrupt 115*/
    .long   Reserved116_IRQHandler                          /* Reserved interrupt 116*/
    .long   FTM3_IRQHandler                                 /* FTM3 fault, overflow and channels interrupt*/
    .long   ADC2_IRQHandler                                 /* ADC2 interrupt*/
    .long   ADC3_IRQHandler                                 /* ADC3 interrupt*/
    .long   I2S1_Tx_IRQHandler                              /* I2S1 transmit interrupt*/
    .long   I2S1_Rx_IRQHandler                              /* I2S1 receive interrupt*/
    .long   DefaultISR                                      /* 122*/
    .long   DefaultISR                                      /* 123*/
    .long   DefaultISR                                      /* 124*/
    .long   DefaultISR                                      /* 125*/
    .long   DefaultISR                                      /* 126*/
    .long   DefaultISR                                      /* 127*/
    .long   DefaultISR                                      /* 128*/
    .long   DefaultISR                                      /* 129*/
    .long   DefaultISR                                      /* 130*/
    .long   DefaultISR                                      /* 131*/
    .long   DefaultISR                                      /* 132*/
    .long   DefaultISR                                      /* 133*/
    .long   DefaultISR                                      /* 134*/
    .long   DefaultISR                                      /* 135*/
    .long   DefaultISR                                      /* 136*/
    .long   DefaultISR                                      /* 137*/
    .long   DefaultISR                                      /* 138*/
    .long   DefaultISR                                      /* 139*/
    .long   DefaultISR                                      /* 140*/
    .long   DefaultISR                                      /* 141*/
    .long   DefaultISR                                      /* 142*/
    .long   DefaultISR                                      /* 143*/
    .long   DefaultISR                                      /* 144*/
    .long   DefaultISR                                      /* 145*/
    .long   DefaultISR                                      /* 146*/
    .long   DefaultISR                                      /* 147*/
    .long   DefaultISR                                      /* 148*/
    .long   DefaultISR                                      /* 149*/
    .long   DefaultISR                                      /* 150*/
    .long   DefaultISR                                      /* 151*/
    .long   DefaultISR                                      /* 152*/
    .long   DefaultISR                                      /* 153*/
    .long   DefaultISR                                      /* 154*/
    .long   DefaultISR                                      /* 155*/
    .long   DefaultISR                                      /* 156*/
    .long   DefaultISR                                      /* 157*/
    .long   DefaultISR                                      /* 158*/
    .long   DefaultISR                                      /* 159*/
    .long   DefaultISR                                      /* 160*/
    .long   DefaultISR                                      /* 161*/
    .long   DefaultISR                                      /* 162*/
    .long   DefaultISR                                      /* 163*/
    .long   DefaultISR                                      /* 164*/
    .long   DefaultISR                                      /* 165*/
    .long   DefaultISR                                      /* 166*/
    .long   DefaultISR                                      /* 167*/
    .long   DefaultISR                                      /* 168*/
    .long   DefaultISR                                      /* 169*/
    .long   DefaultISR                                      /* 170*/
    .long   DefaultISR                                      /* 171*/
    .long   DefaultISR                                      /* 172*/
    .long   DefaultISR                                      /* 173*/
    .long   DefaultISR                                      /* 174*/
    .long   DefaultISR                                      /* 175*/
    .long   DefaultISR                                      /* 176*/
    .long   DefaultISR                                      /* 177*/
    .long   DefaultISR                                      /* 178*/
    .long   DefaultISR                                      /* 179*/
    .long   DefaultISR                                      /* 180*/
    .long   DefaultISR                                      /* 181*/
    .long   DefaultISR                                      /* 182*/
    .long   DefaultISR                                      /* 183*/
    .long   DefaultISR                                      /* 184*/
    .long   DefaultISR                                      /* 185*/
    .long   DefaultISR                                      /* 186*/
    .long   DefaultISR                                      /* 187*/
    .long   DefaultISR                                      /* 188*/
    .long   DefaultISR                                      /* 189*/
    .long   DefaultISR                                      /* 190*/
    .long   DefaultISR                                      /* 191*/
    .long   DefaultISR                                      /* 192*/
    .long   DefaultISR                                      /* 193*/
    .long   DefaultISR                                      /* 194*/
    .long   DefaultISR                                      /* 195*/
    .long   DefaultISR                                      /* 196*/
    .long   DefaultISR                                      /* 197*/
    .long   DefaultISR                                      /* 198*/
    .long   DefaultISR                                      /* 199*/
    .long   DefaultISR                                      /* 200*/
    .long   DefaultISR                                      /* 201*/
    .long   DefaultISR                                      /* 202*/
    .long   DefaultISR                                      /* 203*/
    .long   DefaultISR                                      /* 204*/
    .long   DefaultISR                                      /* 205*/
    .long   DefaultISR                                      /* 206*/
    .long   DefaultISR                                      /* 207*/
    .long   DefaultISR                                      /* 208*/
    .long   DefaultISR                                      /* 209*/
    .long   DefaultISR                                      /* 210*/
    .long   DefaultISR                                      /* 211*/
    .long   DefaultISR                                      /* 212*/
    .long   DefaultISR                                      /* 213*/
    .long   DefaultISR                                      /* 214*/
    .long   DefaultISR                                      /* 215*/
    .long   DefaultISR                                      /* 216*/
    .long   DefaultISR                                      /* 217*/
    .long   DefaultISR                                      /* 218*/
    .long   DefaultISR                                      /* 219*/
    .long   DefaultISR                                      /* 220*/
    .long   DefaultISR                                      /* 221*/
    .long   DefaultISR                                      /* 222*/
    .long   DefaultISR                                      /* 223*/
    .long   DefaultISR                                      /* 224*/
    .long   DefaultISR                                      /* 225*/
    .long   DefaultISR                                      /* 226*/
    .long   DefaultISR                                      /* 227*/
    .long   DefaultISR                                      /* 228*/
    .long   DefaultISR                                      /* 229*/
    .long   DefaultISR                                      /* 230*/
    .long   DefaultISR                                      /* 231*/
    .long   DefaultISR                                      /* 232*/
    .long   DefaultISR                                      /* 233*/
    .long   DefaultISR                                      /* 234*/
    .long   DefaultISR                                      /* 235*/
    .long   DefaultISR                                      /* 236*/
    .long   DefaultISR                                      /* 237*/
    .long   DefaultISR                                      /* 238*/
    .long   DefaultISR                                      /* 239*/
    .long   DefaultISR                                      /* 240*/
    .long   DefaultISR                                      /* 241*/
    .long   DefaultISR                                      /* 242*/
    .long   DefaultISR                                      /* 243*/
    .long   DefaultISR                                      /* 244*/
    .long   DefaultISR                                      /* 245*/
    .long   DefaultISR                                      /* 246*/
    .long   DefaultISR                                      /* 247*/
    .long   DefaultISR                                      /* 248*/
    .long   DefaultISR                                      /* 249*/
    .long   DefaultISR                                      /* 250*/
    .long   DefaultISR                                      /* 251*/
    .long   DefaultISR                                      /* 252*/
    .long   DefaultISR                                      /* 253*/
    .long   DefaultISR                                      /* 254*/
    .long   0xFFFFFFFF                                      /*  Reserved for user TRIM value*/

    .size    __isr_vector, . - __isr_vector

/* Flash Configuration */
    .section .FlashConfig, "a"
    .long 0xFFFFFFFF
    .long 0xFFFFFFFF
    .long 0xFFFFFFFF
    .long 0xFFFFFFFE

    .equ _NVIC_ICER0, 0xE000E180
    .equ _NVIC_ICPR0, 0xE000E280
    .text
    .thumb

/* Reset Handler */

    .thumb_func
    .align 2
    .globl   Reset_Handler
    .type    Reset_Handler, %function
Reset_Handler:
    cpsid   i               /* Mask interrupts */
    ldr r0, =_NVIC_ICER0    /* Disable interrupts and clear pending flags */
    ldr r1, =_NVIC_ICPR0
    ldr r2, =0xFFFFFFFF
    mov r3, #8
_irq_clear:
    cbz r3, _irq_clear_end
    str r2, [r0], #4        /* NVIC_ICERx - clear enable IRQ register */
    str r2, [r1], #4        /* NVIC_ICPRx - clear pending IRQ register */
    sub r3, r3, #1
    b _irq_clear
_irq_clear_end:
#ifndef __NO_SYSTEM_INIT
    bl SystemInit
#endif
    cpsie   i               /* Unmask interrupts */
/*     Loop to copy data from read only memory to RAM. The ranges
 *      of copy from/to are specified by following symbols evaluated in
 *      linker script.
 *      __etext: End of code section, i.e., begin of data sections to copy from.
 *      __data_start__/__data_end__: RAM address range that data should be
 *      copied to. Both must be aligned to 4 bytes boundary.  */

    ldr    r1, =__etext
    ldr    r2, =__data_start__
    ldr    r3, =__data_end__

#if 1
/* Here are two copies of loop implemenations. First one favors code size
 * and the second one favors performance. Default uses the first one.
 * Change to "#if 0" to use the second one */
.LC0:
    cmp     r2, r3
    ittt    lt
    ldrlt   r0, [r1], #4
    strlt   r0, [r2], #4
    blt    .LC0
#else
    subs    r3, r2
    ble    .LC1
.LC0:
    subs    r3, #4
    ldr    r0, [r1, r3]
    str    r0, [r2, r3]
    bgt    .LC0
.LC1:
#endif

#ifdef __STARTUP_CLEAR_BSS
/*     This part of work usually is done in C library startup code. Otherwise,
 *     define this macro to enable it in this startup.
 *
 *     Loop to zero out BSS section, which uses following symbols
 *     in linker script:
 *      __bss_start__: start of BSS section. Must align to 4
 *      __bss_end__: end of BSS section. Must align to 4
 */
    ldr r1, =__bss_start__
    ldr r2, =__bss_end__

    movs    r0, 0
.LC2:
    cmp     r1, r2
    itt    lt
    strlt   r0, [r1], #4
    blt    .LC2
#endif /* __STARTUP_CLEAR_BSS */

#ifndef __START
#define __START _start
#endif
    bl    __START
    .pool
    .size Reset_Handler, . - Reset_Handler

  .align	1
  .thumb_func
  .weak	Default_Handler
  .type	Default_Handler, %function
Default_Handler:
  b	.
  .size	Default_Handler, . - Default_Handler

/*    Macro to define default handlers. Default handler
 *    will be weak symbol and just dead loops. They can be
 *    overwritten by other handlers */
  .macro	def_irq_handler	handler_name
  .weak	\handler_name
  .set	\handler_name, Default_Handler
  .endm

/* Exception Handlers */
    def_irq_handler    NMI_Handler
    def_irq_handler    HardFault_Handler
    def_irq_handler    MemManage_Handler
    def_irq_handler    BusFault_Handler
    def_irq_handler    UsageFault_Handler
    def_irq_handler    SVC_Handler
    def_irq_handler    DebugMon_Handler
    def_irq_handler    PendSV_Handler
    def_irq_handler    SysTick_Handler
    def_irq_handler    DMA0_DMA16_IRQHandler
    def_irq_handler    DMA1_DMA17_IRQHandler
    def_irq_handler    DMA2_DMA18_IRQHandler
    def_irq_handler    DMA3_DMA19_IRQHandler
    def_irq_handler    DMA4_DMA20_IRQHandler
    def_irq_handler    DMA5_DMA21_IRQHandler
    def_irq_handler    DMA6_DMA22_IRQHandler
    def_irq_handler    DMA7_DMA23_IRQHandler
    def_irq_handler    DMA8_DMA24_IRQHandler
    def_irq_handler    DMA9_DMA25_IRQHandler
    def_irq_handler    DMA10_DMA26_IRQHandler
    def_irq_handler    DMA11_DMA27_IRQHandler
    def_irq_handler    DMA12_DMA28_IRQHandler
    def_irq_handler    DMA13_DMA29_IRQHandler
    def_irq_handler    DMA14_DMA30_IRQHandler
    def_irq_handler    DMA15_DMA31_IRQHandler
    def_irq_handler    DMA_Error_IRQHandler
    def_irq_handler    MCM_IRQHandler
    def_irq_handler    FTFE_IRQHandler
    def_irq_handler    Read_Collision_IRQHandler
    def_irq_handler    LVD_LVW_IRQHandler
    def_irq_handler    LLW_IRQHandler
    def_irq_handler    Watchdog_IRQHandler
    def_irq_handler    RNG_IRQHandler
    def_irq_handler    I2C0_IRQHandler
    def_irq_handler    I2C1_IRQHandler
    def_irq_handler    SPI0_IRQHandler
    def_irq_handler    SPI1_IRQHandler
    def_irq_handler    SPI2_IRQHandler
    def_irq_handler    CAN0_ORed_Message_buffer_IRQHandler
    def_irq_handler    CAN0_Bus_Off_IRQHandler
    def_irq_handler    CAN0_Error_IRQHandler
    def_irq_handler    CAN0_Tx_Warning_IRQHandler
    def_irq_handler    CAN0_Rx_Warning_IRQHandler
    def_irq_handler    CAN0_Wake_Up_IRQHandler
    def_irq_handler    I2S0_Tx_IRQHandler
    def_irq_handler    I2S0_Rx_IRQHandler
    def_irq_handler    CAN1_ORed_Message_buffer_IRQHandler
    def_irq_handler    CAN1_Bus_Off_IRQHandler
    def_irq_handler    CAN1_Error_IRQHandler
    def_irq_handler    CAN1_Tx_Warning_IRQHandler
    def_irq_handler    CAN1_Rx_Warning_IRQHandler
    def_irq_handler    CAN1_Wake_Up_IRQHandler
    def_irq_handler    Reserved59_IRQHandler
    def_irq_handler    UART0_LON_IRQHandler
    def_irq_handler    UART0_RX_TX_IRQHandler
    def_irq_handler    UART0_ERR_IRQHandler
    def_irq_handler    UART1_RX_TX_IRQHandler
    def_irq_handler    UART1_ERR_IRQHandler
    def_irq_handler    UART2_RX_TX_IRQHandler
    def_irq_handler    UART2_ERR_IRQHandler
    def_irq_handler    UART3_RX_TX_IRQHandler
    def_irq_handler    UART3_ERR_IRQHandler
    def_irq_handler    UART4_RX_TX_IRQHandler
    def_irq_handler    UART4_ERR_IRQHandler
    def_irq_handler    UART5_RX_TX_IRQHandler
    def_irq_handler    UART5_ERR_IRQHandler
    def_irq_handler    ADC0_IRQHandler
    def_irq_handler    ADC1_IRQHandler
    def_irq_handler    CMP0_IRQHandler
    def_irq_handler    CMP1_IRQHandler
    def_irq_handler    CMP2_IRQHandler
    def_irq_handler    FTM0_IRQHandler
    def_irq_handler    FTM1_IRQHandler
    def_irq_handler    FTM2_IRQHandler
    def_irq_handler    CMT_IRQHandler
    def_irq_handler    RTC_IRQHandler
    def_irq_handler    RTC_Seconds_IRQHandler
    def_irq_handler    PIT0_IRQHandler
    def_irq_handler    PIT1_IRQHandler
    def_irq_handler    PIT2_IRQHandler
    def_irq_handler    PIT3_IRQHandler
    def_irq_handler    PDB0_IRQHandler
    def_irq_handler    USB0_IRQHandler
    def_irq_handler    USBDCD_IRQHandler
    def_irq_handler    ENET_1588_Timer_IRQHandler
    def_irq_handler    ENET_Transmit_IRQHandler
    def_irq_handler    ENET_Receive_IRQHandler
    def_irq_handler    ENET_Error_IRQHandler
    def_irq_handler    Reserved95_IRQHandler
    def_irq_handler    SDHC_IRQHandler
    def_irq_handler    DAC0_IRQHandler
    def_irq_handler    DAC1_IRQHandler
    def_irq_handler    TSI0_IRQHandler
    def_irq_handler    MCG_IRQHandler
    def_irq_handler    LPTimer_IRQHandler
    def_irq_handler    Reserved102_IRQHandler
    def_irq_handler    PORTA_IRQHandler
    def_irq_handler    PORTB_IRQHandler
    def_irq_handler    PORTC_IRQHandler
    def_irq_handler    PORTD_IRQHandler
    def_irq_handler    PORTE_IRQHandler
    def_irq_handler    PORTF_IRQHandler
    def_irq_handler    DDR_IRQHandler
    def_irq_handler    SWI_IRQHandler
    def_irq_handler    NFC_IRQHandler
    def_irq_handler    USBHS_IRQHandler
    def_irq_handler    Reserved113_IRQHandler
    def_irq_handler    CMP3_IRQHandler
    def_irq_handler    Reserved115_IRQHandler
    def_irq_handler    Reserved116_IRQHandler
    def_irq_handler    FTM3_IRQHandler
    def_irq_handler    ADC2_IRQHandler
    def_irq_handler    ADC3_IRQHandler
    def_irq_handler    I2S1_Tx_IRQHandler
    def_irq_handler    I2S1_Rx_IRQHandler
    def_irq_handler    DefaultISR

    .end
