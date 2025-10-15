#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * Application specific definitions for RISC-V SMP port.
 * Adjust these for your hardware and application.
 *----------------------------------------------------------*/

// #define CLINT_CTRL_ADDR                  ( 0xF0000000UL )
// #define configMTIME_BASE_ADDRESS         ( CLINT_CTRL_ADDR + 0x0UL )
// #define configMTIMECMP_BASE_ADDRESS      ( CLINT_CTRL_ADDR + 0x8UL )
#define CLINT_CTRL_ADDR                  ( 0xF0000000UL )
#define configMSIP_BASE_ADDRESS          ( CLINT_CTRL_ADDR + 0x0000UL )
#define configMTIMECMP_BASE_ADDRESS      ( CLINT_CTRL_ADDR + 0x4000UL )
#define configMTIME_BASE_ADDRESS         ( CLINT_CTRL_ADDR + 0xBFF8UL )

/* Scheduler and SMP related */
#define configUSE_PREEMPTION             1
#define configUSE_TIME_SLICING           1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 0
#define configUSE_IDLE_HOOK              0
#define configUSE_PASSIVE_IDLE_HOOK      0
#define configUSE_TICK_HOOK              1
#define configCPU_CLOCK_HZ               ( ( uint32_t ) 60000000 )
#define configTICK_RATE_HZ               ( ( TickType_t ) 100 )
#define configMAX_PRIORITIES             ( 7 )
#define configMINIMAL_STACK_SIZE         ( ( uint32_t ) 512 )
#define configTOTAL_HEAP_SIZE            ( ( size_t ) ( 128 * 1024 ) )
#define configMAX_TASK_NAME_LEN          ( 16 )
#define configUSE_TRACE_FACILITY         0
#define configUSE_16_BIT_TICKS           0
#define configIDLE_SHOULD_YIELD          0

/* Synchronization primitives */
#define configUSE_MUTEXES                1
#define configUSE_RECURSIVE_MUTEXES      1
#define configUSE_COUNTING_SEMAPHORES    1
#define configQUEUE_REGISTRY_SIZE        8
#define configCHECK_FOR_STACK_OVERFLOW   2
#define configUSE_MALLOC_FAILED_HOOK     1

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES            0
#define configMAX_CO_ROUTINE_PRIORITIES  ( 2 )

/* Software timer definitions. */
#define configUSE_TIMERS                 1
#define configTIMER_TASK_PRIORITY        ( configMAX_PRIORITIES - 1 )
#define configTIMER_QUEUE_LENGTH         4
#define configTIMER_TASK_STACK_DEPTH     ( configMINIMAL_STACK_SIZE )

/* API function inclusion. */
#define INCLUDE_vTaskPrioritySet         1
#define INCLUDE_uxTaskPriorityGet        1
#define INCLUDE_vTaskDelete              1
#define INCLUDE_vTaskCleanUpResources    1
#define INCLUDE_vTaskSuspend             1
#define INCLUDE_vTaskDelayUntil          1
#define INCLUDE_vTaskDelay               1
#define INCLUDE_eTaskGetState            1
#define INCLUDE_xTimerPendFunctionCall   1
#define INCLUDE_xTaskAbortDelay          1
#define INCLUDE_xTaskGetHandle           1
#define INCLUDE_xSemaphoreGetMutexHolder 1
#define configASSERT_DEFINED             1

/* Assertions */
#define configASSERT( x )                                       \
    if( ( x ) == 0 )                                            \
    {                                                           \
        taskDISABLE_INTERRUPTS();                               \
        for( ;; );                                              \
    }

/*-----------------------------------------------------------
 * SMP-specific configuration
 *----------------------------------------------------------*/
// configNUMBER_OF_CORES cannot less then 2
#define configNUMBER_OF_CORES            16
#define THREAD                           configNUMBER_OF_CORES

#define N_NEURONS_0 784
#define N_NEURONS_1 48
#define N_NEURONS_2 10
#define IMAGE_SIZE 576
#define N_IMAGES   100

// #define AFFINITY_SPECIFIC_CORE // Assign task to specific core
// #define PROFILE_LOG            // print the data cache profile log

#define configRUN_MULTIPLE_PRIORITIES    1    
#define configUSE_CORE_AFFINITY          1
#define configUSE_PASSIVE_IDLE_HOOK      0
#define portSUPPORT_SMP                  1
#define RTOS_LOCK_COUNT                  2
#define portCRITICAL_NESTING_IN_TCB      1
#define configISR_STACK_SIZE_WORDS       256

#define MALLOC_LOCK_ADDR  0x80000a00u
#define PRINT_LOCK_ADDR  ( ( volatile uint32_t * ) 0x80000a04u )
#endif /* FREERTOS_CONFIG_H */
