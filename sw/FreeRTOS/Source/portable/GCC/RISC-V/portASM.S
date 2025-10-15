/*
 * FreeRTOS Kernel <DEVELOPMENT BRANCH>
 * Copyright (C) 2021 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */

#include "portContext.h"

.global xPortStartFirstTask
.global pxPortInitialiseStack
.global freertos_risc_v_trap_handler

.extern vTaskSwitchContext
.extern xTaskIncrementTick
.extern pullMachineTimerCompareRegister
.extern pullNextTime
.extern uxTimerIncrementsForOneTick
.extern xTaskReturnAddress
.extern xPortTickInterruptHandler
.extern ullMachineTimerCompareRegisterBase
.extern xYieldRequest


.weak freertos_risc_v_application_exception_handler
.weak freertos_risc_v_application_interrupt_handler
/*-----------------------------------------------------------*/
.macro portUPDATE_MTIMER_COMPARE_REGISTER
    load_x a0, pullMachineTimerCompareRegister  /* Load address of compare register into a0. */
    la a1, ullNextTime                     /* Load the address of ullNextTime into a1. */
    csrr  t0, mhartid
    slli  t0, t0, 3
    add   a0, a0, t0 
    add   a1, a1, t0
    #if( __riscv_xlen == 32 )

        /* Update the 64-bit mtimer compare match value in two 32-bit writes. */
        li a4, -1
        lw a2, 0(a1)                /* Load the low word of ullNextTime into a2. */
        lw a3, 4(a1)                /* Load the high word of ullNextTime into a3. */
        sw a4, 0(a0)                /* Low word no smaller than old value to start with - will be overwritten below. */
        sw a3, 4(a0)                /* Store high word of ullNextTime into compare register.  No smaller than new value. */
        sw a2, 0(a0)                /* Store low word of ullNextTime into compare register. */
        lw t0, uxTimerIncrementsForOneTick  /* Load the value of ullTimerIncrementForOneTick into t0 (could this be optimized by storing in an array next to pullNextTime?). */
        add a4, t0, a2              /* Add the low word of ullNextTime to the timer increments for one tick (assumes timer increment for one tick fits in 32-bits). */
        sltu t1, a4, a2             /* See if the sum of low words overflowed (what about the zero case?). */
        add t2, a3, t1              /* Add overflow to high word of ullNextTime. */
        sw a4, 0(a1)                /* Store new low word of ullNextTime. */
        sw t2, 4(a1)                /* Store new high word of ullNextTime. */

    #endif /* __riscv_xlen == 32 */

    #if( __riscv_xlen == 64 )

        /* Update the 64-bit mtimer compare match value. */
        ld t2, 0(a1)                /* Load ullNextTime into t2. */
        sd t2, 0(a0)                /* Store ullNextTime into compare register. */
        ld t0, uxTimerIncrementsForOneTick  /* Load the value of ullTimerIncrementForOneTick into t0 (could this be optimized by storing in an array next to pullNextTime?). */
        add t4, t0, t2              /* Add ullNextTime to the timer increments for one tick. */
        sd t4, 0(a1)                /* Store ullNextTime. */

    #endif /* __riscv_xlen == 64 */
    .endm
/*-----------------------------------------------------------*/

pxPortInitialiseStack:
    addi a0, a0, -portWORD_SIZE         /* Space for critical nesting count. */
    store_x x0, 0(a0)                   /* Critical nesting count starts at 0 for every task. */

#ifdef __riscv_32e
    addi a0, a0, -(6 * portWORD_SIZE)   /* Space for registers x10-x15. */
#else
    addi a0, a0, -(22 * portWORD_SIZE)  /* Space for registers x10-x31. */
#endif
    store_x a2, 0(a0)                   /* Task parameters (pvParameters parameter) goes into register x10/a0 on the stack. */

    addi a0, a0, -(6 * portWORD_SIZE)   /* Space for registers x5-x9 + taskReturnAddress (register x1). */
    load_x t0, xTaskReturnAddress
    store_x t0, 0(a0)                   /* Return address onto the stack. */

    addi t0, x0, portasmADDITIONAL_CONTEXT_SIZE /* The number of chip specific additional registers. */
chip_specific_stack_frame:              /* First add any chip specific registers to the stack frame being created. */
    beq t0, x0, 1f                      /* No more chip specific registers to save. */
    addi a0, a0, -portWORD_SIZE         /* Make space for chip specific register. */
    store_x x0, 0(a0)                   /* Give the chip specific register an initial value of zero. */
    addi t0, t0, -1                     /* Decrement the count of chip specific registers remaining. */
    j chip_specific_stack_frame         /* Until no more chip specific registers. */
1:
    csrr t0, mstatus                    /* Obtain current mstatus value. */
    andi t0, t0, ~0x8                   /* Ensure interrupts are disabled when the stack is restored within an ISR.  Required when a task is created after the scheduler has been started, otherwise interrupts would be disabled anyway. */
    addi t1, x0, 0x188                  /* Generate the value 0x1880, which are the MPIE=1 and MPP=M_Mode in mstatus. */
    slli t1, t1, 4
    or t0, t0, t1                       /* Set MPIE and MPP bits in mstatus value. */

#if( configENABLE_FPU == 1 )
    /* Mark the FPU as clean in the mstatus value. */
    li t1, ~MSTATUS_FS_MASK
    and t0, t0, t1
    li t1, MSTATUS_FS_CLEAN
    or t0, t0, t1
#endif

#if( configENABLE_VPU == 1 )
    /* Mark the VPU as clean in the mstatus value. */
    li t1, ~MSTATUS_VS_MASK
    and t0, t0, t1
    li t1, MSTATUS_VS_CLEAN
    or t0, t0, t1
#endif

    addi a0, a0, -portWORD_SIZE
    store_x t0, 0(a0)                   /* mstatus onto the stack. */

    addi a0, a0, -portWORD_SIZE
    store_x a1, 0(a0)                   /* mret value (pxCode parameter) onto the stack. */
    ret
/*-----------------------------------------------------------*/

xPortStartFirstTask:
    csrr t0, mhartid
#if __riscv_xlen == 32
    slli t0, t0, 2
#else
    slli t0, t0, 3
#endif
    la t1, pxCurrentTCBs
    add t0, t0, t1
    load_x t2, 0(t0)
    load_x sp, 0( t2 )

    load_x  x1, 0( sp )

    load_x  x5, 1 * portWORD_SIZE( sp )
    addi    x5, x5, 0x08
    csrw    mstatus, x5

    portasmRESTORE_ADDITIONAL_REGISTERS

    load_x  x7,  5  * portWORD_SIZE( sp )
    load_x  x8,  6  * portWORD_SIZE( sp )
    load_x  x9,  7  * portWORD_SIZE( sp )
    load_x  x10, 8  * portWORD_SIZE( sp )
    load_x  x11, 9  * portWORD_SIZE( sp )
    load_x  x12, 10 * portWORD_SIZE( sp )
    load_x  x13, 11 * portWORD_SIZE( sp )
    load_x  x14, 12 * portWORD_SIZE( sp )
    load_x  x15, 13 * portWORD_SIZE( sp )
#ifndef __riscv_32e
    load_x  x16, 14 * portWORD_SIZE( sp )
    load_x  x17, 15 * portWORD_SIZE( sp )
    load_x  x18, 16 * portWORD_SIZE( sp )
    load_x  x19, 17 * portWORD_SIZE( sp )
    load_x  x20, 18 * portWORD_SIZE( sp )
    load_x  x21, 19 * portWORD_SIZE( sp )
    load_x  x22, 20 * portWORD_SIZE( sp )
    load_x  x23, 21 * portWORD_SIZE( sp )
    load_x  x24, 22 * portWORD_SIZE( sp )
    load_x  x25, 23 * portWORD_SIZE( sp )
    load_x  x26, 24 * portWORD_SIZE( sp )
    load_x  x27, 25 * portWORD_SIZE( sp )
    load_x  x28, 26 * portWORD_SIZE( sp )
    load_x  x29, 27 * portWORD_SIZE( sp )
    load_x  x30, 28 * portWORD_SIZE( sp )
    load_x  x31, 29 * portWORD_SIZE( sp )
#endif

#     load_x  x5, portCRITICAL_NESTING_OFFSET * portWORD_SIZE( sp )

#     csrr t0, mhartid
# #if __riscv_xlen == 32
#     slli t0, t0, 2
# #else
#     slli t0, t0, 3
# #endif
#     la x6, pxCriticalNesting
#     add x6, x6, t0
#     store_x x5, 0( x6 )

    load_x  x5, 3 * portWORD_SIZE( sp )
    load_x  x6, 4 * portWORD_SIZE( sp )

    addi    sp, sp, portCONTEXT_SIZE
    ret
/*-----------------------------------------------------------*/

freertos_risc_v_application_exception_handler:
    csrr t0, mcause     
    csrr t1, mepc       
    csrr t2, mstatus    
    j .
/*-----------------------------------------------------------*/

freertos_risc_v_application_interrupt_handler:
    csrr t0, mcause     
    csrr t1, mepc       
    csrr t2, mstatus   
    j .
/*-----------------------------------------------------------*/

.section .text.freertos_risc_v_trap_handler
.align 8
freertos_risc_v_trap_handler:

    portcontextSAVE_CONTEXT_INTERNAL
    
    csrr a0, mcause
    csrr a1, mepc

    bge a0, x0, synchronous_exception

asynchronous_interrupt:
    store_x a1, 0( sp )
    # load_x sp, xISRStackTop           
# #if ( configNUMBER_OF_CORES > 1 )
#     csrr   t0, mhartid                   
#     load_x t1, xSingleISRStackSizeInBytes 
#     mul    t2, t0, t1                     
#     la     sp, xISRStack               
#     add    sp, sp, t2                     
#     add    sp, sp, t1                    
# #else
#     load_x sp, xISRStackTop 
# #endif
    csrr   t0, mhartid                   
    load_x t1, xSingleISRStackSizeInBytes 
    mul    t2, t0, t1                     
    la     sp, xISRStack               
    add    sp, sp, t2                     
    add    sp, sp, t1   
    j handle_interrupt

synchronous_exception:
    addi a1, a1, 4
    store_x a1, 0( sp )
#     # load_x sp, xISRStackTop
# #if ( configNUMBER_OF_CORES > 1 )
#     csrr   t0, mhartid                   
#     load_x t1, xSingleISRStackSizeInBytes 
#     mul    t2, t0, t1                     
#     la     sp, xISRStack               
#     add    sp, sp, t2                     
#     add    sp, sp, t1                    
# #else
#     load_x sp, xISRStackTop  
# #endif
    csrr   t0, mhartid                   
    load_x t1, xSingleISRStackSizeInBytes 
    mul    t2, t0, t1                     
    la     sp, xISRStack               
    add    sp, sp, t2                     
    add    sp, sp, t1   
    j handle_exception

handle_interrupt:
#if( portasmHAS_MTIME != 0 )

    test_if_mtimer:                     /* If there is a CLINT then the mtimer is used to generate the tick interrupt. */
        addi t0, x0, 1
        slli t0, t0, __riscv_xlen - 1   /* LSB is already set, shift into MSB.  Shift 31 on 32-bit or 63 on 64-bit cores. */
        addi t1, t0, 7                  /* 0x8000[]0007 == machine timer interrupt. */
        bne a0, t1, application_interrupt_handler

        portUPDATE_MTIMER_COMPARE_REGISTER
        csrr t0, mhartid
        bnez t0, check_sw_yield
        call xPortTickInterruptHandler
        mv t0, a0
        j 1f
#endif 

check_sw_yield:
    csrr a1, mhartid
    slli a2, a1, 2              
    la   a3, xYieldRequest
    add  a3, a3, a2
    lw   a4, 0(a3)
    beqz a4, 1f                  
    
    sw   zero, 0(a3)           
    li   t0, 1                 

1:
    beqz t0, processed_source     
    j context_switch_handler     

application_interrupt_handler:
    call freertos_risc_v_application_interrupt_handler
    j processed_source

handle_exception:
    /* a0 contains mcause. */
    li t0, 8                                    /* 8 == Environment call from U-mode */
    beq a0, t0, context_switch_handler
    li t0, 11                                   /* 11 == Environment call from M-mode */
    bne a0, t0, application_exception_handler   /* Not an M environment call, so some other exception. */

context_switch_handler:
    csrr a0, mhartid
    call vTaskSwitchContext
    j processed_source

application_exception_handler:
    call freertos_risc_v_application_exception_handler
    j processed_source

processed_source:
    portcontextRESTORE_CONTEXT