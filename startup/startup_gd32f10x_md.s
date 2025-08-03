/*
 * Copyright (c) 2012-2014 ARM LIMITED
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/*
 * This is a generic startup file for Cortex-M3.
 * It's adapted for GD32F10x Medium-Density devices
 * and designed to be compatible with GNU Assembler (GAS).
 *
 * IMPORTANT: This file needs corresponding linker script symbols.
 * Ensure  linker script (gd32f103c8t6.ld) defines:
 * - _estack (End of Stack - Top of RAM)
 * - _sidata (Start of .data section in Flash/ROM)
 * - _sdata (Start of .data section in RAM)
 * - _edata (End of .data section in RAM)
 * - _sbss (Start of .bss section in RAM)
 * - _ebss (End of .bss section in RAM)
 */

    .syntax unified
    .cpu cortex-m3
    .thumb

/* Stack Configuration */
    .word   _estack             /* Top of Stack (defined in linker script) */
    .equ    Stack_Size, 0x00000400 /* 1KB stack size */
    .section ".stack"
    .space  Stack_Size
    .align  3
__initial_sp:

/* Heap Configuration */
    .equ    Heap_Size,  0x00000200 /* 512B heap size */
    .section ".heap"
    .space  Heap_Size
    .align  3
__heap_base:
__heap_limit:

/* Vector Table */
    .section .isr_vector,"a",%progbits
    .align 2
    .globl  __Vectors
    .globl  __Vectors_End
    .globl  __Vectors_Size


__Vectors:
    .word   _estack               /* Top of Stack */
    .word   Reset_Handler         /* Reset Handler */
    .word   NMI_Handler           /* NMI Handler */
    .word   HardFault_Handler     /* Hard Fault Handler */
    .word   MemManage_Handler     /* MPU Fault Handler */
    .word   BusFault_Handler      /* Bus Fault Handler */
    .word   UsageFault_Handler    /* Usage Fault Handler */
    .word   0                     /* Reserved */
    .word   0                     /* Reserved */
    .word   0                     /* Reserved */
    .word   0                     /* Reserved */
    .word   SVC_Handler           /* SVCall Handler */
    .word   DebugMon_Handler      /* Debug Monitor Handler */
    .word   0                     /* Reserved */
    .word   PendSV_Handler        /* PendSV Handler */
    .word   SysTick_Handler       /* SysTick Handler */

    /* External Interrupts (GD32F10x Medium-Density specific) */
    .word   WWDGT_IRQHandler
    .word   LVD_IRQHandler
    .word   TAMPER_IRQHandler
    .word   RTC_IRQHandler
    .word   FMC_IRQHandler
    .word   RCU_CTC_IRQHandler      /* RCU and CTC combined in MD */
    .word   EXTI0_IRQHandler
    .word   EXTI1_IRQHandler
    .word   EXTI2_IRQHandler
    .word   EXTI3_IRQHandler
    .word   EXTI4_IRQHandler
    .word   DMA0_Channel0_IRQHandler
    .word   DMA0_Channel1_IRQHandler
    .word   DMA0_Channel2_IRQHandler
    .word   DMA0_Channel3_IRQHandler
    .word   DMA0_Channel4_IRQHandler
    .word   DMA0_Channel5_IRQHandler
    .word   DMA0_Channel6_IRQHandler
    .word   ADC0_1_IRQHandler
    .word   USBD_HP_CAN0_TX_IRQHandler
    .word   USBD_LP_CAN0_RX0_IRQHandler
    .word   CAN0_RX1_IRQHandler
    .word   CAN0_EWMC_IRQHandler
    .word   EXTI5_9_IRQHandler
    .word   TIMER0_BRK_IRQHandler
    .word   TIMER0_UP_IRQHandler
    .word   TIMER0_TRG_CMT_IRQHandler
    .word   TIMER0_Channel_IRQHandler
    .word   TIMER1_IRQHandler
    .word   TIMER2_IRQHandler
    .word   TIMER3_IRQHandler
    .word   I2C0_EV_IRQHandler
    .word   I2C0_ER_IRQHandler
    .word   I2C1_EV_IRQHandler
    .word   I2C1_ER_IRQHandler
    .word   SPI0_IRQHandler
    .word   SPI1_IRQHandler
    .word   USART0_IRQHandler
    .word   USART1_IRQHandler
    .word   USART2_IRQHandler
    .word   EXTI10_15_IRQHandler
    .word   RTC_Alarm_IRQHandler
    .word   USBD_WKUP_IRQHandler
    .word   Reserved0_IRQHandler     /* Reserved, was TIMER7_BRK_IRQn in HD */
    .word   Reserved1_IRQHandler     /* Reserved, was TIMER7_UP_IRQn in HD */
    .word   Reserved2_IRQHandler     /* Reserved, was TIMER7_TRG_CMT_IRQn in HD */
    .word   Reserved3_IRQHandler     /* Reserved, was TIMER7_Channel_IRQn in HD */
    .word   Reserved4_IRQHandler     /* Reserved, was ADC2_IRQn in HD */
    .word   EXMC_IRQHandler          /* EXMC_IRQn at this position in MD, HD, XD */
    .word   Reserved5_IRQHandler     /* Reserved, was SDIO_IRQn in HD */
    .word   Reserved6_IRQHandler     /* Reserved, was TIMER4_IRQn in HD */
    .word   Reserved7_IRQHandler     /* Reserved, was SPI2_IRQn in HD */
    .word   Reserved8_IRQHandler     /* Reserved, was UART3_IRQn in HD */
    .word   Reserved9_IRQHandler     /* Reserved, was UART4_IRQn in HD */
    .word   Reserved10_IRQHandler    /* Reserved, was TIMER5_IRQn in HD */
    .word   Reserved11_IRQHandler    /* Reserved, was TIMER6_IRQn in HD */
    .word   Reserved12_IRQHandler    /* Reserved, was DMA1_Channel0_IRQn in HD */
    .word   Reserved13_IRQHandler    /* Reserved, was DMA1_Channel1_IRQn in HD */
    .word   Reserved14_IRQHandler    /* Reserved, was DMA1_Channel2_IRQn in HD */
    .word   Reserved15_IRQHandler    /* Reserved, was DMA1_Channel3_Channel4_IRQn in HD */

__Vectors_End:
__Vectors_Size = __Vectors_End - __Vectors

/* Reset Handler */
    .section .text.Reset_Handler
    .thumb_func
    .globl  Reset_Handler
Reset_Handler:
    /* Call SystemInit from system_gd32f10x.c */
    bl      SystemInit

    /* Copy .data section from Flash to RAM */
    ldr     r0, =_sdata         /* Start of .data in RAM */
    ldr     r1, =_edata         /* End of .data in RAM */
    ldr     r2, =_sidata        /* Start of .data in Flash (load address) */
    movs    r3, #0
    cmp     r0, r1
    beq     .L_copy_end
.L_copy_loop:
    ldr     r3, [r2], #4        /* Load 4 bytes from Flash, increment r2 */
    str     r3, [r0], #4        /* Store 4 bytes to RAM, increment r0 */
    cmp     r0, r1
    bne     .L_copy_loop
.L_copy_end:

    /* Clear .bss section in RAM */
    ldr     r0, =_sbss          /* Start of .bss in RAM */
    ldr     r1, =_ebss          /* End of .bss in RAM */
    movs    r2, #0              /* Zero value */
    cmp     r0, r1
    beq     .L_bss_end
.L_bss_loop:
    str     r2, [r0], #4        /* Store zero to RAM, increment r0 */
    cmp     r0, r1
    bne     .L_bss_loop
.L_bss_end:

    /* Jump to main application */      
    bl      main                /* Call main() function */
    b       .                   /* Loop forever if main returns */

/* Default weak handlers (for unused interrupts) */
    .align 1
    .thumb_func
Default_Handler:
    b .

/* Weak aliases for default handlers */
    .weak   NMI_Handler
    .thumb_set NMI_Handler, Default_Handler

    .weak   HardFault_Handler
    .thumb_set HardFault_Handler, Default_Handler

    .weak   MemManage_Handler
    .thumb_set MemManage_Handler, Default_Handler

    .weak   BusFault_Handler
    .thumb_set BusFault_Handler, Default_Handler

    .weak   UsageFault_Handler
    .thumb_set UsageFault_Handler, Default_Handler

    .weak   SVC_Handler
    .thumb_set SVC_Handler, Default_Handler

    .weak   DebugMon_Handler
    .thumb_set DebugMon_Handler, Default_Handler

    .weak   PendSV_Handler
    .thumb_set PendSV_Handler, Default_Handler

    .weak   SysTick_Handler
    .thumb_set SysTick_Handler, Default_Handler

    .weak   WWDGT_IRQHandler
    .thumb_set WWDGT_IRQHandler, Default_Handler

    .weak   LVD_IRQHandler
    .thumb_set LVD_IRQHandler, Default_Handler

    .weak   TAMPER_IRQHandler
    .thumb_set TAMPER_IRQHandler, Default_Handler

    .weak   RTC_IRQHandler
    .thumb_set RTC_IRQHandler, Default_Handler

    .weak   FMC_IRQHandler
    .thumb_set FMC_IRQHandler, Default_Handler

    .weak   RCU_CTC_IRQHandler
    .thumb_set RCU_CTC_IRQHandler, Default_Handler

    .weak   EXTI0_IRQHandler
    .thumb_set EXTI0_IRQHandler, Default_Handler

    .weak   EXTI1_IRQHandler
    .thumb_set EXTI1_IRQHandler, Default_Handler

    .weak   EXTI2_IRQHandler
    .thumb_set EXTI2_IRQHandler, Default_Handler

    .weak   EXTI3_IRQHandler
    .thumb_set EXTI3_IRQHandler, Default_Handler

    .weak   EXTI4_IRQHandler
    .thumb_set EXTI4_IRQHandler, Default_Handler

    .weak   DMA0_Channel0_IRQHandler
    .thumb_set DMA0_Channel0_IRQHandler, Default_Handler

    .weak   DMA0_Channel1_IRQHandler
    .thumb_set DMA0_Channel1_IRQHandler, Default_Handler

    .weak   DMA0_Channel2_IRQHandler
    .thumb_set DMA0_Channel2_IRQHandler, Default_Handler

    .weak   DMA0_Channel3_IRQHandler
    .thumb_set DMA0_Channel3_IRQHandler, Default_Handler

    .weak   DMA0_Channel4_IRQHandler
    .thumb_set DMA0_Channel4_IRQHandler, Default_Handler

    .weak   DMA0_Channel5_IRQHandler
    .thumb_set DMA0_Channel5_IRQHandler, Default_Handler

    .weak   DMA0_Channel6_IRQHandler
    .thumb_set DMA0_Channel6_IRQHandler, Default_Handler

    .weak   ADC0_1_IRQHandler
    .thumb_set ADC0_1_IRQHandler, Default_Handler

    .weak   USBD_HP_CAN0_TX_IRQHandler
    .thumb_set USBD_HP_CAN0_TX_IRQHandler, Default_Handler

    .weak   USBD_LP_CAN0_RX0_IRQHandler
    .thumb_set USBD_LP_CAN0_RX0_IRQHandler, Default_Handler

    .weak   CAN0_RX1_IRQHandler
    .thumb_set CAN0_RX1_IRQHandler, Default_Handler

    .weak   CAN0_EWMC_IRQHandler
    .thumb_set CAN0_EWMC_IRQHandler, Default_Handler

    .weak   EXTI5_9_IRQHandler
    .thumb_set EXTI5_9_IRQHandler, Default_Handler

    .weak   TIMER0_BRK_IRQHandler
    .thumb_set TIMER0_BRK_IRQHandler, Default_Handler

    .weak   TIMER0_UP_IRQHandler
    .thumb_set TIMER0_UP_IRQHandler, Default_Handler

    .weak   TIMER0_TRG_CMT_IRQHandler
    .thumb_set TIMER0_TRG_CMT_IRQHandler, Default_Handler

    .weak   TIMER0_Channel_IRQHandler
    .thumb_set TIMER0_Channel_IRQHandler, Default_Handler

    .weak   TIMER1_IRQHandler
    .thumb_set TIMER1_IRQHandler, Default_Handler

    .weak   TIMER2_IRQHandler
    .thumb_set TIMER2_IRQHandler, Default_Handler

    .weak   TIMER3_IRQHandler
    .thumb_set TIMER3_IRQHandler, Default_Handler

    .weak   I2C0_EV_IRQHandler
    .thumb_set I2C0_EV_IRQHandler, Default_Handler

    .weak   I2C0_ER_IRQHandler
    .thumb_set I2C0_ER_IRQHandler, Default_Handler

    .weak   I2C1_EV_IRQHandler
    .thumb_set I2C1_EV_IRQHandler, Default_Handler

    .weak   I2C1_ER_IRQHandler
    .thumb_set I2C1_ER_IRQHandler, Default_Handler

    .weak   SPI0_IRQHandler
    .thumb_set SPI0_IRQHandler, Default_Handler

    .weak   SPI1_IRQHandler
    .thumb_set SPI1_IRQHandler, Default_Handler

    .weak   USART0_IRQHandler
    .thumb_set USART0_IRQHandler, Default_Handler

    .weak   USART1_IRQHandler
    .thumb_set USART1_IRQHandler, Default_Handler

    .weak   USART2_IRQHandler
    .thumb_set USART2_IRQHandler, Default_Handler

    .weak   EXTI10_15_IRQHandler
    .thumb_set EXTI10_15_IRQHandler, Default_Handler

    .weak   RTC_Alarm_IRQHandler
    .thumb_set RTC_Alarm_IRQHandler, Default_Handler

    .weak   USBD_WKUP_IRQHandler
    .thumb_set USBD_WKUP_IRQHandler, Default_Handler

    .weak   Reserved0_IRQHandler
    .thumb_set Reserved0_IRQHandler, Default_Handler

    .weak   Reserved1_IRQHandler
    .thumb_set Reserved1_IRQHandler, Default_Handler

    .weak   Reserved2_IRQHandler
    .thumb_set Reserved2_IRQHandler, Default_Handler

    .weak   Reserved3_IRQHandler
    .thumb_set Reserved3_IRQHandler, Default_Handler

    .weak   Reserved4_IRQHandler
    .thumb_set Reserved4_IRQHandler, Default_Handler

    .weak   EXMC_IRQHandler
    .thumb_set EXMC_IRQHandler, Default_Handler

    .weak   Reserved5_IRQHandler
    .thumb_set Reserved5_IRQHandler, Default_Handler

    .weak   Reserved6_IRQHandler
    .thumb_set Reserved6_IRQHandler, Default_Handler

    .weak   Reserved7_IRQHandler
    .thumb_set Reserved7_IRQHandler, Default_Handler

    .weak   Reserved8_IRQHandler
    .thumb_set Reserved8_IRQHandler, Default_Handler

    .weak   Reserved9_IRQHandler
    .thumb_set Reserved9_IRQHandler, Default_Handler

    .weak   Reserved10_IRQHandler
    .thumb_set Reserved10_IRQHandler, Default_Handler

    .weak   Reserved11_IRQHandler
    .thumb_set Reserved11_IRQHandler, Default_Handler

    .weak   Reserved12_IRQHandler
    .thumb_set Reserved12_IRQHandler, Default_Handler

    .weak   Reserved13_IRQHandler
    .thumb_set Reserved13_IRQHandler, Default_Handler

    .weak   Reserved14_IRQHandler
    .thumb_set Reserved14_IRQHandler, Default_Handler

    .weak   Reserved15_IRQHandler
    .thumb_set Reserved15_IRQHandler, Default_Handler


    .end
