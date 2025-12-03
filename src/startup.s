/*
 * Startup code for ATSAMS70Q21
 * ARM Cortex-M7 vector table and reset handler
 */

    .syntax unified
    .cpu cortex-m7
    .fpu fpv5-d16
    .thumb

/* External symbols from linker script */
    .word   _estack             /* Top of stack */
    .word   _sdata              /* Start of .data in RAM */
    .word   _edata              /* End of .data in RAM */
    .word   _stext              /* Start of .text in Flash */
    .word   _etext              /* End of .text in Flash (source for .data) */
    .word   _sbss               /* Start of .bss */
    .word   _ebss               /* End of .bss */

/* Vector Table */
    .section .vectors,"a",%progbits
    .type   vector_table, %object
    .size   vector_table, .-vector_table

vector_table:
    .word   _estack                     /* Initial Stack Pointer */
    .word   Reset_Handler               /* Reset Handler */
    .word   NMI_Handler                 /* NMI Handler */
    .word   HardFault_Handler           /* Hard Fault Handler */
    .word   MemManage_Handler           /* MPU Fault Handler */
    .word   BusFault_Handler            /* Bus Fault Handler */
    .word   UsageFault_Handler          /* Usage Fault Handler */
    .word   0                           /* Reserved */
    .word   0                           /* Reserved */
    .word   0                           /* Reserved */
    .word   0                           /* Reserved */
    .word   SVC_Handler                 /* SVCall Handler */
    .word   DebugMon_Handler            /* Debug Monitor Handler */
    .word   0                           /* Reserved */
    .word   PendSV_Handler              /* PendSV Handler */
    .word   SysTick_Handler             /* SysTick Handler */

    /* External Interrupts (SAMS70 specific) */
    .word   SUPC_Handler                /* 0:  Supply Controller */
    .word   RSTC_Handler                /* 1:  Reset Controller */
    .word   RTC_Handler                 /* 2:  Real Time Clock */
    .word   RTT_Handler                 /* 3:  Real Time Timer */
    .word   WDT_Handler                 /* 4:  Watchdog Timer */
    .word   PMC_Handler                 /* 5:  Power Management */
    .word   EFC_Handler                 /* 6:  Flash Controller */
    .word   UART0_Handler               /* 7:  UART 0 */
    .word   UART1_Handler               /* 8:  UART 1 */
    .word   0                           /* 9:  Reserved */
    .word   PIOA_Handler                /* 10: Parallel IO A */
    .word   PIOB_Handler                /* 11: Parallel IO B */
    .word   PIOC_Handler                /* 12: Parallel IO C */
    .word   USART0_Handler              /* 13: USART 0 */
    .word   USART1_Handler              /* 14: USART 1 */
    .word   USART2_Handler              /* 15: USART 2 */
    .word   PIOD_Handler                /* 16: Parallel IO D */
    .word   PIOE_Handler                /* 17: Parallel IO E */
    .word   HSMCI_Handler               /* 18: SD/MMC Controller */
    .word   TWIHS0_Handler              /* 19: Two-Wire Interface 0 */
    .word   TWIHS1_Handler              /* 20: Two-Wire Interface 1 */
    .word   SPI0_Handler                /* 21: SPI 0 */
    .word   SSC_Handler                 /* 22: Synchronous Serial */
    .word   TC0_Handler                 /* 23: Timer Counter 0 */
    .word   TC1_Handler                 /* 24: Timer Counter 1 */
    .word   TC2_Handler                 /* 25: Timer Counter 2 */
    .word   TC3_Handler                 /* 26: Timer Counter 3 */
    .word   TC4_Handler                 /* 27: Timer Counter 4 */
    .word   TC5_Handler                 /* 28: Timer Counter 5 */
    .word   AFEC0_Handler               /* 29: Analog Front End 0 */
    .word   DACC_Handler                /* 30: DAC Controller */
    .word   PWM0_Handler                /* 31: PWM 0 */
    .word   ICM_Handler                 /* 32: Integrity Check Monitor */
    .word   ACC_Handler                 /* 33: Analog Comparator */
    .word   USBHS_Handler               /* 34: USB Host/Device */
    .word   MCAN0_Handler               /* 35: CAN 0 */
    .word   0                           /* 36: Reserved */
    .word   MCAN1_Handler               /* 37: CAN 1 */
    .word   0                           /* 38: Reserved */
    .word   AFEC1_Handler               /* 39: Analog Front End 1 */
    .word   TWIHS2_Handler              /* 40: Two-Wire Interface 2 */
    .word   SPI1_Handler                /* 41: SPI 1 */
    .word   QSPI_Handler                /* 42: Quad SPI */
    .word   UART2_Handler               /* 43: UART 2 */
    .word   UART3_Handler               /* 44: UART 3 */
    .word   UART4_Handler               /* 45: UART 4 */
    .word   TC6_Handler                 /* 46: Timer Counter 6 */
    .word   TC7_Handler                 /* 47: Timer Counter 7 */
    .word   TC8_Handler                 /* 48: Timer Counter 8 */
    .word   TC9_Handler                 /* 49: Timer Counter 9 */
    .word   TC10_Handler                /* 50: Timer Counter 10 */
    .word   TC11_Handler                /* 51: Timer Counter 11 */
    .word   0                           /* 52: Reserved */
    .word   0                           /* 53: Reserved */
    .word   0                           /* 54: Reserved */
    .word   AES_Handler                 /* 55: AES */
    .word   TRNG_Handler                /* 56: True RNG */
    .word   XDMAC_Handler               /* 57: DMA Controller */
    .word   ISI_Handler                 /* 58: Image Sensor Interface */
    .word   PWM1_Handler                /* 59: PWM 1 */
    .word   FPU_Handler                 /* 60: FPU Exception */
    .word   0                           /* 61: Reserved */
    .word   RSWDT_Handler               /* 62: Reinforced WDT */

/*
 * Reset Handler - Entry point after reset
 */
    .section .text.Reset_Handler
    .weak   Reset_Handler
    .type   Reset_Handler, %function
Reset_Handler:
    /* Copy .data section from Flash to RAM */
    ldr r0, =_etext         /* Source (Flash) */
    ldr r1, =_sdata         /* Dest (RAM) */
    ldr r2, =_edata         /* End of .data */

    subs r2, r2, r1         /* Calculate size */
    beq copy_data_done      /* Skip if size = 0 */

copy_data_loop:
    subs r2, #4
    ldr r3, [r0, r2]
    str r3, [r1, r2]
    bgt copy_data_loop

copy_data_done:

    /* Zero-initialize .bss section */
    ldr r0, =_sbss
    ldr r1, =_ebss
    movs r2, #0

zero_bss_loop:
    cmp r0, r1
    bge zero_bss_done
    str r2, [r0]
    adds r0, r0, #4
    b zero_bss_loop

zero_bss_done:

    /* Enable FPU (Cortex-M7 has FPU) */
    ldr r0, =0xE000ED88     /* CPACR (Coprocessor Access Control) */
    ldr r1, [r0]
    orr r1, r1, #(0xF << 20) /* Enable CP10 and CP11 (FPU) */
    str r1, [r0]
    dsb
    isb

    /* Call main() */
    bl main

    /* If main() returns, loop forever */
    b .

    .size Reset_Handler, . - Reset_Handler

/*
 * Default exception/interrupt handlers (weak aliases)
 */
    .macro def_irq_handler handler_name
    .weak \handler_name
    .type \handler_name, %function
\handler_name:
    b .
    .size \handler_name, . - \handler_name
    .endm

    def_irq_handler NMI_Handler
    def_irq_handler HardFault_Handler
    def_irq_handler MemManage_Handler
    def_irq_handler BusFault_Handler
    def_irq_handler UsageFault_Handler
    def_irq_handler SVC_Handler
    def_irq_handler DebugMon_Handler
    def_irq_handler PendSV_Handler
    def_irq_handler SysTick_Handler

    /* Peripheral handlers */
    def_irq_handler SUPC_Handler
    def_irq_handler RSTC_Handler
    def_irq_handler RTC_Handler
    def_irq_handler RTT_Handler
    def_irq_handler WDT_Handler
    def_irq_handler PMC_Handler
    def_irq_handler EFC_Handler
    def_irq_handler UART0_Handler
    def_irq_handler UART1_Handler
    def_irq_handler PIOA_Handler
    def_irq_handler PIOB_Handler
    def_irq_handler PIOC_Handler
    def_irq_handler USART0_Handler
    def_irq_handler USART1_Handler
    def_irq_handler USART2_Handler
    def_irq_handler PIOD_Handler
    def_irq_handler PIOE_Handler
    def_irq_handler HSMCI_Handler
    def_irq_handler TWIHS0_Handler
    def_irq_handler TWIHS1_Handler
    def_irq_handler SPI0_Handler
    def_irq_handler SSC_Handler
    def_irq_handler TC0_Handler
    def_irq_handler TC1_Handler
    def_irq_handler TC2_Handler
    def_irq_handler TC3_Handler
    def_irq_handler TC4_Handler
    def_irq_handler TC5_Handler
    def_irq_handler AFEC0_Handler
    def_irq_handler DACC_Handler
    def_irq_handler PWM0_Handler
    def_irq_handler ICM_Handler
    def_irq_handler ACC_Handler
    def_irq_handler USBHS_Handler
    def_irq_handler MCAN0_Handler
    def_irq_handler MCAN1_Handler
    def_irq_handler AFEC1_Handler
    def_irq_handler TWIHS2_Handler
    def_irq_handler SPI1_Handler
    def_irq_handler QSPI_Handler
    def_irq_handler UART2_Handler
    def_irq_handler UART3_Handler
    def_irq_handler UART4_Handler
    def_irq_handler TC6_Handler
    def_irq_handler TC7_Handler
    def_irq_handler TC8_Handler
    def_irq_handler TC9_Handler
    def_irq_handler TC10_Handler
    def_irq_handler TC11_Handler
    def_irq_handler AES_Handler
    def_irq_handler TRNG_Handler
    def_irq_handler XDMAC_Handler
    def_irq_handler ISI_Handler
    def_irq_handler PWM1_Handler
    def_irq_handler FPU_Handler
    def_irq_handler RSWDT_Handler
