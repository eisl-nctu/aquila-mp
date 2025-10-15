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

/*-----------------------------------------------------------
 * Implementation of functions defined in portable.h for the RISC-V port.
 *----------------------------------------------------------*/

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "portmacro.h"
#include <stdbool.h>

/* Standard includes. */
#include "string.h"

/* Let the user override the pre-loading of the initial RA. */
#ifdef configTASK_RETURN_ADDRESS
#define portTASK_RETURN_ADDRESS configTASK_RETURN_ADDRESS
#else
#define portTASK_RETURN_ADDRESS 0
#endif

/* Used to keep track of the number of nested calls to taskENTER_CRITICAL(). */
// size_t volatile pxCriticalNesting[ configNUMBER_OF_CORES ] = { 0xaaaaaaaa, 0xaaaaaaaa, 0xaaaaaaaa, 0xaaaaaaaa };



volatile BaseType_t xYieldRequest[configNUMBER_OF_CORES] = {pdFALSE};

extern void freertos_risc_v_trap_handler(void);

/* The stack used by interrupt service routines. */
#if ( configNUMBER_OF_CORES > 1 )
    __attribute__((aligned(16))) StackType_t xISRStack[ configNUMBER_OF_CORES ][ configISR_STACK_SIZE_WORDS ];
    const size_t xSingleISRStackSizeInBytes = configISR_STACK_SIZE_WORDS * sizeof( StackType_t );
#else
    #ifdef configISR_STACK_SIZE_WORDS
        static __attribute__((aligned(16))) StackType_t xISRStack[ configISR_STACK_SIZE_WORDS ] = { 0 };
        const StackType_t xISRStackTop = ( StackType_t ) &( xISRStack[ configISR_STACK_SIZE_WORDS & ~portBYTE_ALIGNMENT_MASK ] );
    #else
        extern const uint32_t __freertos_irq_stack_top[];
        const StackType_t xISRStackTop = ( StackType_t )__freertos_irq_stack_top;
    #endif
#endif /* configNUMBER_OF_CORES > 1 */

/*
 * Setup the timer to generate the tick interrupts. The implementation in this
 * file is weak to allow application writers to change the timer used to
 * generate the tick interrupt.
 */
void vPortSetupTimerInterrupt(void) __attribute__((weak));

/*-----------------------------------------------------------*/

/* Used to program the machine timer compare register. */
volatile uint32_t ullPortSchedularRunning = false;
volatile uint64_t ullNextTime[configNUMBER_OF_CORES] = {0};


const size_t uxTimerIncrementsForOneTick = (size_t)((configCPU_CLOCK_HZ) / (configTICK_RATE_HZ));
UBaseType_t const ullMachineTimerCompareRegisterBase = configMTIMECMP_BASE_ADDRESS;
volatile uint64_t *pullMachineTimerCompareRegister = NULL;

/* Used to catch tasks that attempt to return from their implementing function. */
size_t xTaskReturnAddress = (size_t)portTASK_RETURN_ADDRESS;

/*-----------------------------------------------------------*/

#if (configMTIME_BASE_ADDRESS != 0) && (configMTIMECMP_BASE_ADDRESS != 0)

void vPortSetupTimerInterrupt(void)
{
    uint32_t ulCurrentTimeHigh, ulCurrentTimeLow;
    volatile uint32_t *const pulTimeHigh = (volatile uint32_t *const)((configMTIME_BASE_ADDRESS) + 4UL);
    volatile uint32_t *const pulTimeLow = (volatile uint32_t *const)(configMTIME_BASE_ADDRESS);

    pullMachineTimerCompareRegister = (volatile uint64_t *)(ullMachineTimerCompareRegisterBase);

    do
    {
        ulCurrentTimeHigh = *pulTimeHigh;
        ulCurrentTimeLow = *pulTimeLow;
    } while (ulCurrentTimeHigh != *pulTimeHigh);

    volatile uint32_t ulHartId;
    __asm volatile("csrr %0, mhartid" : "=r"(ulHartId));
    ullNextTime[ulHartId] = (uint64_t)ulCurrentTimeHigh;
    ullNextTime[ulHartId] <<= 32ULL;
    ullNextTime[ulHartId] |= (uint64_t)ulCurrentTimeLow;
    ullNextTime[ulHartId] += (uint64_t)uxTimerIncrementsForOneTick;
    
    volatile uint64_t * registers_base_address;
    registers_base_address = pullMachineTimerCompareRegister + ulHartId;
    *registers_base_address = ullNextTime[ulHartId];

    /* Prepare the time to use after the next tick interrupt. */
    ullNextTime[ulHartId] += (uint64_t)uxTimerIncrementsForOneTick;
}

#endif /* ( configMTIME_BASE_ADDRESS != 0 ) && ( configMTIME_BASE_ADDRESS != 0 ) */

/*-----------------------------------------------------------*/

BaseType_t xPortStartScheduler(void)
{
    extern void xPortStartFirstTask(void);
//    BaseType_t xCore = rtos_core_id_get();

    uintptr_t trap_addr = ((uintptr_t)freertos_risc_v_trap_handler);
    __asm__ volatile("csrw mtvec, %0" ::"r"(trap_addr));

    vPortSetupTimerInterrupt();



    ullPortSchedularRunning = true;

    #if ((configMTIME_BASE_ADDRESS != 0) && (configMTIMECMP_BASE_ADDRESS != 0))
    {
        __asm volatile("csrs mie, %0" ::"r"(0x88U));  
    }
#endif

    xPortStartFirstTask();

    return pdFAIL;
}

BaseType_t xPortStartSchedulerOncore(void){
    extern void xPortStartFirstTask(void);

    uintptr_t trap_addr = ((uintptr_t)freertos_risc_v_trap_handler);
    __asm__ volatile("csrw mtvec, %0" ::"r"(trap_addr));

    vPortSetupTimerInterrupt();



    while (!ullPortSchedularRunning)
    {
        __asm__ volatile("fence");
    }


    #if ((configMTIME_BASE_ADDRESS != 0) && (configMTIMECMP_BASE_ADDRESS != 0))
    {
        __asm volatile("csrs mie, %0" ::"r"(0x88U));  
    }
#endif

    xPortStartFirstTask();

    return pdFAIL;
}

/*-----------------------------------------------------------*/

void vPortEndScheduler(void)
{
    /* Not implemented. */
    for (;;)
    {
    }
}
/*-----------------------------------------------------------*/

/* SMP utilities */
BaseType_t rtos_core_id_get(void)
{
    int xCoreID;

    __asm volatile("csrr %0, mhartid" : "=r"(xCoreID));

    return xCoreID;
}

void vPortYieldOtherCore(UBaseType_t xCoreID)
{
    if (xCoreID < (UBaseType_t)configNUMBER_OF_CORES)
    {
        xYieldRequest[xCoreID] = pdTRUE;
    }
}

volatile uint32_t xIsrLock = 0;
volatile uint32_t xTaskLock = 0;

uint8_t ucOwnedByCore[configNUMBER_OF_CORES][2];
uint8_t ucRecursionCount[configNUMBER_OF_CORES][2];

bool SpinTryLock(volatile uint32_t *lock)
{
    uint32_t ulHartId = rtos_core_id_get() + 1;
    uint32_t ulPrevVal;

    __asm__ volatile(
        "amoswap.w.aqrl %0, %2, %1"
        : "=r"(ulPrevVal), "+A"(*lock)
        : "r"(ulHartId)
        : "memory");

    return (ulPrevVal == 0);
}

void SpinLock(volatile uint32_t *lock)
{
    for (;;)
    {
        if (SpinTryLock(lock))
        {
            return;
        }
    }
}

void SpinUnlock(volatile uint32_t *lock)
{
    portMEMORY_BARRIER();
    __asm__ volatile("amoswap.w.rl zero, zero, %0" : : "A"(*lock) : "memory");
}

void vPortRecursiveLock(BaseType_t xCoreID,
                        uint32_t ulLockNum,
                        BaseType_t uxAcquire)
{
    volatile uint32_t *pulLock = (ulLockNum == 0) ? &xIsrLock : &xTaskLock;

    configASSERT(ulLockNum < 2);

    if (uxAcquire)
    {
        if (!SpinTryLock(pulLock))
        {
            if (ucOwnedByCore[xCoreID][ulLockNum])
            {
                configASSERT(ucRecursionCount[xCoreID][ulLockNum] < 255);
                ucRecursionCount[xCoreID][ulLockNum]++;
                return;
            }

            SpinLock(pulLock);
        }

        configASSERT(ucRecursionCount[xCoreID][ulLockNum] == 0);
        ucRecursionCount[xCoreID][ulLockNum] = 1;
        ucOwnedByCore[xCoreID][ulLockNum] = 1;
    }
    else
    {
        configASSERT(ucOwnedByCore[xCoreID][ulLockNum]);
        configASSERT(ucRecursionCount[xCoreID][ulLockNum] > 0);

        if (--(ucRecursionCount[xCoreID][ulLockNum]) == 0)
        {
            ucOwnedByCore[xCoreID][ulLockNum] = 0;
            SpinUnlock(pulLock);
        }
    }
}

BaseType_t xPortTickInterruptHandler(void)
{
    BaseType_t xSwitchRequired = pdFALSE;
    UBaseType_t uxSavedInterruptStatus;

    if (ullPortSchedularRunning == true)
    {
        uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
        {

            xSwitchRequired = xTaskIncrementTick();
        }
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
    }

    return xSwitchRequired;
}

