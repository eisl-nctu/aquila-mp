// =============================================================================
//  Program : rtos_matrix.c
//  Author  : YU-TING, Lee
//  Date    : Sept/1/2025
// =============================================================================
//  This program implements parallel GeMM  using FreeRTOS SMP.
//  The input matrices are divided into multiple blocks, and each block is 
//  assigned to a separate thread for computation. Each thread computes a portion
//  of the output matrix, and the results are combined to produce the final output.
// -----------------------------------------------------------------------------
//  Revision information:
//
//  Sept/18/2025, by Tzu-Chen Yang:
//    Changed the number of test iterations to 100 and added measurement of computation time
// -----------------------------------------------------------------------------
//  License information:
//
//  This software is released under the BSD-3-Clause Licence,
//  see https://opensource.org/licenses/BSD-3-Clause for details.
//  In the following license statements, "software" refers to the
//  "source code" of the complete hardware/software system.
//
//  Copyright 2025,
//                    Embedded Intelligent Systems Lab (EISL)
//                    Deparment of Computer Science
//                    National Yang Ming Chiao Tung University
//                    Hsinchu, Taiwan.
//
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions are met:
//
//  1. Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//  2. Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//  3. Neither the name of the copyright holder nor the names of its contributors
//     may be used to endorse or promote products derived from this software
//     without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
//  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.
// =============================================================================
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define TASK_STACK_SIZE         (configMINIMAL_STACK_SIZE * 2)
#define TASK_PRIORITY           (tskIDLE_PRIORITY + 1)
#define COORDINATOR_CORE        0 

unsigned int *A, *B, *C;

volatile uint32_t g_ulStartRunFlag = 0;
volatile uint32_t done_calc[TIMES] = {0};
volatile uint32_t done_eval[TIMES] = {0};
volatile uint32_t g_ulWorkersDone = 0;
volatile uint32_t start_print_profile = 0;

#ifdef PROFILE_LOG
// Per-core profiling data arrays
volatile int g_I2M_counter[configNUMBER_OF_CORES];
volatile int g_I2E_counter[configNUMBER_OF_CORES];
volatile int g_I2S_counter[configNUMBER_OF_CORES];
volatile int g_S2M_counter[configNUMBER_OF_CORES];
volatile int g_M2Replaced_counter[configNUMBER_OF_CORES];
volatile int g_E2Replaced_counter[configNUMBER_OF_CORES];
volatile int g_MorE2M_counter[configNUMBER_OF_CORES];

volatile uint64_t g_I2M_latency[configNUMBER_OF_CORES];
volatile uint64_t g_I2E_latency[configNUMBER_OF_CORES];
volatile uint64_t g_I2S_latency[configNUMBER_OF_CORES];
volatile uint64_t g_S2M_latency[configNUMBER_OF_CORES];
volatile uint64_t g_M2Replaced_latency[configNUMBER_OF_CORES];
volatile uint64_t g_E2Replaced_latency[configNUMBER_OF_CORES];

volatile int g_I2M_avg_latency[configNUMBER_OF_CORES];
volatile int g_I2E_avg_latency[configNUMBER_OF_CORES];
volatile int g_I2S_avg_latency[configNUMBER_OF_CORES];
volatile int g_S2M_avg_latency[configNUMBER_OF_CORES];
volatile int g_M2Replaced_avg_latency[configNUMBER_OF_CORES];
volatile int g_E2Replaced_avg_latency[configNUMBER_OF_CORES];

volatile uint64_t context_switch_overhead[configNUMBER_OF_CORES] = {0};
volatile uint64_t context_switch_times[configNUMBER_OF_CORES] = {0};
volatile uint64_t barrier_overhead[THREAD] = {0};
// Mask to track which cores have reported profiling data
volatile uint32_t g_profile_done_mask = 0;
#endif

int task_mask = (1 << THREAD) - 1;
extern void freertos_risc_v_trap_handler(void);
extern void xPortStartSchedulerOncore(void);
void print_64bit(uint64_t val)
{
    char buf[32];
    int i = 0;

    if (val == 0) {
        putchar('0');
        return;
    }

    while (val > 0) {
        buf[i++] = '0' + (val % 10);
        val /= 10;
    }

    while (i--) {
        putchar(buf[i]);
    }
}

static int ipow10(int n) {
    int result = 1;
    while (n-- > 0) {
        result *= 10;
    }
    return result;
}

void double_to_string(double value, char *buffer, int precision)
{
    if (precision > 9) precision = 9; // truncate the length
    int int_part = (int)value;
    double frac = value - (double)int_part;
    if (frac < 0) frac = -frac;

    int scale = ipow10(precision);
    int frac_part = (int)(frac * scale + 0.5);

    sprintf(buffer, "%d.%0*d", int_part, precision, frac_part);
}

void print_double(double val)
{
    char buf[64];
    double_to_string(val, buf, 6); // keep 6 digits
    printf("%s", buf);
}

static inline void lock_print() {
    uint32_t ulHartId = rtos_core_id_get() + 1;
    uint32_t ulPrevVal;
    do {
        __asm__ volatile("amoswap.w.aqrl %0, %2, %1" : "=r"(ulPrevVal), "+A"(*PRINT_LOCK_ADDR) : "r"(ulHartId) : "memory");
    } while (ulPrevVal != 0);
}

static inline void unlock_print() {
    __asm__ volatile("amoswap.w.rl zero, zero, %0" : : "A"(*PRINT_LOCK_ADDR) : "memory");
}

static void atomic_or(volatile uint32_t *addr, int val) {
    __asm__ volatile("amoor.w.aqrl zero, %1, %0" : "+A"(*addr) : "r"(val) : "memory");
}

#ifdef PROFILE_LOG
void vProfile_Task(void *pvParameters) {
    while (start_print_profile == 0) {;}

    int core_id = rtos_core_id_get();

    // base address of the cache profiler registers
    volatile uint32_t *prof_base = (volatile uint32_t *)0xF0010000;

    // --- Counters (32-bit) ---
    g_I2M_counter[core_id]        = prof_base[0x0000 >> 2];
    g_I2E_counter[core_id]        = prof_base[0x0004 >> 2];
    g_I2S_counter[core_id]        = prof_base[0x0008 >> 2];
    g_S2M_counter[core_id]        = prof_base[0x000C >> 2];
    g_M2Replaced_counter[core_id] = prof_base[0x0010 >> 2];
    g_E2Replaced_counter[core_id] = prof_base[0x0014 >> 2];
    g_MorE2M_counter[core_id]     = prof_base[0x0048 >> 2];

    // --- Latency (64-bit counters) ---
    #define READ_LAT64(offset) ({                   \
        uint32_t hi1, lo, hi2;                      \
        do {                                        \
            hi1 = prof_base[(offset) >> 2];         \
            lo  = prof_base[((offset)+4) >> 2];     \
            hi2 = prof_base[(offset) >> 2];         \
        } while (hi1 != hi2);                       \
        ((uint64_t)hi1 << 32) | lo;                 \
    })

    g_I2M_latency[core_id]        = READ_LAT64(0x0018);
    g_I2E_latency[core_id]        = READ_LAT64(0x0020);
    g_I2S_latency[core_id]        = READ_LAT64(0x0028);
    g_S2M_latency[core_id]        = READ_LAT64(0x0030);
    g_M2Replaced_latency[core_id] = READ_LAT64(0x0038);
    g_E2Replaced_latency[core_id] = READ_LAT64(0x0040);

    // --- Average Latency (32-bit) ---
    g_I2M_avg_latency[core_id]        = (int)(g_I2M_latency[core_id] / (g_I2M_counter[core_id] ?: 1));
    g_I2E_avg_latency[core_id]        = (int)(g_I2E_latency[core_id] / (g_I2E_counter[core_id] ?: 1));
    g_I2S_avg_latency[core_id]        = (int)(g_I2S_latency[core_id] / (g_I2S_counter[core_id] ?: 1));
    g_S2M_avg_latency[core_id]        = (int)(g_S2M_latency[core_id] / (g_S2M_counter[core_id] ?: 1));
    g_M2Replaced_avg_latency[core_id] = (int)(g_M2Replaced_latency[core_id] / (g_M2Replaced_counter[core_id] ?: 1));
    g_E2Replaced_avg_latency[core_id] = (int)(g_E2Replaced_latency[core_id] / (g_E2Replaced_counter[core_id] ?: 1));

    // finished collecting statistics
    atomic_or(&g_profile_done_mask, (1u << core_id));

    // --- Core 0 integrates statistics from all cores ---
    if (core_id == 0) {
        uint32_t expect = (1u << configNUMBER_OF_CORES) - 1u;
        while (g_profile_done_mask != expect) {;}

        // Summarize the results
        uint64_t sum_lat[6] = {0};
        uint64_t sum_cnt[6] = {0};

        for (int c = 0; c < configNUMBER_OF_CORES; ++c) {
            sum_cnt[0] += g_I2M_counter[c];
            sum_cnt[1] += g_I2E_counter[c];
            sum_cnt[2] += g_I2S_counter[c];
            sum_cnt[3] += g_S2M_counter[c];
            sum_cnt[4] += g_M2Replaced_counter[c];
            sum_cnt[5] += g_E2Replaced_counter[c];

            sum_lat[0] += g_I2M_latency[c];
            sum_lat[1] += g_I2E_latency[c];
            sum_lat[2] += g_I2S_latency[c];
            sum_lat[3] += g_S2M_latency[c];
            sum_lat[4] += g_M2Replaced_latency[c];
            sum_lat[5] += g_E2Replaced_latency[c];
        }

        // Core 0 output the statistics
        lock_print();
        printf("\n================= Profile Summary (All Cores) =================\n");

        /* Counters of each core */
        printf(">>> Per-Core Counter\n");
        printf("Core |    I2M    I2E    I2S    S2M    M2R    E2R\n");
        printf("-----+---------------------------------------------\n");
        for (int c = 0; c < configNUMBER_OF_CORES; ++c) {
            printf(" %3d | %7u %7u %7u %7u %7u %7u\n",
                c,
                (unsigned)g_I2M_counter[c], (unsigned)g_I2E_counter[c], (unsigned)g_I2S_counter[c],
                (unsigned)g_S2M_counter[c], (unsigned)g_M2Replaced_counter[c], (unsigned)g_E2Replaced_counter[c]);
        }
        printf("==============================================================\n\n");

        /* Weighted Average Latency (cycles) of each core */
        printf(">>> Per-Core Weighted Average Latency (cycles)\n");
        printf("Core |     I2M           I2E           I2S           S2M           M2R           E2R\n");
        printf("-----+---------------------------------------------------------------------------------\n");
        for (int c = 0; c < configNUMBER_OF_CORES; ++c) {
            double l_I2M = g_I2M_counter[c] ? (double)g_I2M_latency[c] / (double)g_I2M_counter[c] : 0.0;
            double l_I2E = g_I2E_counter[c] ? (double)g_I2E_latency[c] / (double)g_I2E_counter[c] : 0.0;
            double l_I2S = g_I2S_counter[c] ? (double)g_I2S_latency[c] / (double)g_I2S_counter[c] : 0.0;
            double l_S2M = g_S2M_counter[c] ? (double)g_S2M_latency[c] / (double)g_S2M_counter[c] : 0.0;
            double l_M2R = g_M2Replaced_counter[c] ? (double)g_M2Replaced_latency[c] / (double)g_M2Replaced_counter[c] : 0.0;
            double l_E2R = g_E2Replaced_counter[c] ? (double)g_E2Replaced_latency[c] / (double)g_E2Replaced_counter[c] : 0.0;

            printf(" %3d | ", c);
            print_double(l_I2M); printf(" ");
            print_double(l_I2E); printf(" ");
            print_double(l_I2S); printf(" ");
            print_double(l_S2M); printf(" ");
            print_double(l_M2R); printf(" ");
            print_double(l_E2R); printf("\n");
        }
        printf(">>> Per-Core overall latency\n");
        printf("Core |     I2M           I2E           I2S           S2M           M2R           E2R\n");
        printf("-----+---------------------------------------------------------------------------------\n");
        for (int c = 0; c < configNUMBER_OF_CORES; ++c) {

            printf(" %3d | ", c);
            print_64bit(g_I2M_latency[c]); printf(" ");
            print_64bit(g_I2E_latency[c]); printf(" ");
            print_64bit(g_I2S_latency[c]); printf(" ");
            print_64bit(g_S2M_latency[c]); printf(" ");
            print_64bit(g_M2Replaced_latency[c] ); printf(" ");
            print_64bit(g_E2Replaced_latency[c] ); printf("\n");
        }
        printf("=================================================================================\n\n");

        /* Weighted Avg (cycles) of all cores */
        printf(">>> Counter Sum (All Cores)\n");
        printf("      I2M      I2E      I2S      S2M      M2R      E2R\n");
        printf("------------------------------------------------------------\n");
        printf("Sum | %7u %7u %7u %7u %7u %7u\n\n",
            (unsigned)sum_cnt[0], (unsigned)sum_cnt[1], (unsigned)sum_cnt[2],
            (unsigned)sum_cnt[3], (unsigned)sum_cnt[4], (unsigned)sum_cnt[5]);

        printf(">>> Weighted Average Latency (All Cores)\n");
        printf("      I2M           I2E           I2S           S2M           M2R           E2R\n");
        printf("------------------------------------------------------------\n");
        printf("Avg | ");
        print_double(sum_cnt[0] ? (double)sum_lat[0] / (double)sum_cnt[0] : 0.0); printf(" ");
        print_double(sum_cnt[1] ? (double)sum_lat[1] / (double)sum_cnt[1] : 0.0); printf(" ");
        print_double(sum_cnt[2] ? (double)sum_lat[2] / (double)sum_cnt[2] : 0.0); printf(" ");
        print_double(sum_cnt[3] ? (double)sum_lat[3] / (double)sum_cnt[3] : 0.0); printf(" ");
        print_double(sum_cnt[4] ? (double)sum_lat[4] / (double)sum_cnt[4] : 0.0); printf(" ");
        print_double(sum_cnt[5] ? (double)sum_lat[5] / (double)sum_cnt[5] : 0.0); printf("\n");

        printf("=================================================================================\n");
        printf(">>> Per-Core Data Cache Write Miss Rate\n");
        printf("---------------------------------------------------------------------------------\n");
        
        uint64_t overall_write_miss = 0;
        uint64_t overall_write_operation = 0;
        
        for (int c = 0; c < configNUMBER_OF_CORES; ++c) {
            uint64_t miss   = g_I2M_counter[c];
            uint64_t total  = g_I2M_counter[c] + g_S2M_counter[c] + g_MorE2M_counter[c];
        
            double write_miss_rate = 0.0;
            if (total > 0) {
                write_miss_rate = (double)miss / (double)total;
            }
            write_miss_rate *= 100;
            printf(" Core %2d | ", c);
            print_double(write_miss_rate);
            printf(" %%\n");
        
            overall_write_miss     += miss;
            overall_write_operation += total;
        }
        
        printf("=================================================================================\n");
        printf(">>> Overall Cache Write Miss Rate (All Cores)\n");
        printf("---------------------------------------------------------------------------------\n");
        
        double overall_write_miss_rate = 0.0;
        if (overall_write_operation > 0) {
            overall_write_miss_rate = (double)overall_write_miss / (double)overall_write_operation;
        }
        overall_write_miss_rate *= 100;
        print_double(overall_write_miss_rate);
        printf(" %%\n");
        
        printf("=================================================================================\n");
        printf("=================================================================================\n");
        printf(">>> Context Switch Overhead (ms)\n");
        printf("---------------------------------------------------------------------------------\n");
    
        double total_overhead = 0;
        uint64_t total_counter = 0;
        for (int c = 0; c < configNUMBER_OF_CORES; c++) {
            double overhead_ms = (double)context_switch_overhead[c] / 1000.0;
            printf("Core %d: ", c);
            print_double(overhead_ms);
            printf(" ms,counter = ");
            print_64bit(context_switch_times[c]);
            printf("\n");
            total_counter += context_switch_times[c];
            total_overhead += (overhead_ms);
        }
    
        printf("---------------------------------------------------------------------------------\n");
        printf("Total latency: ");
        print_double(total_overhead);
        printf(" ms\n");
        printf("Total time: ");
        print_64bit(total_counter);
        printf("\n");
        printf("=================================================================================\n");

        printf("================================================\n");
        printf("barrier overhead:\n");
        double total_barrier = 0;
        for (int c = 0;c < THREAD;c++){
            double overhead_ms = (double)barrier_overhead[c] / 1000.0;
            printf("worker %d: overhead=", c);
            print_double(overhead_ms);
            printf("\n");
            total_barrier += overhead_ms;
        }
        printf("=======================\n");
        printf("total barrier overhead=");
        print_double(total_barrier);
        printf("\n avg barier overhead per thread:");
        double avg_barrier_overhead = total_barrier / (double)THREAD;
        print_double(avg_barrier_overhead);
        printf('\n');
        unlock_print();

        exit(0);
    }

    vTaskDelete(NULL); // worker done
}
#endif

void vWorkerTask(void *pvParameters) {
    // get worker_id and task_name for atomic or
    int worker_id = *((int *)pvParameters);
    const char *task_name = pcTaskGetName(NULL);

    lock_print();
    printf("[%s] (id=%d) started on Core %ld\n", task_name, worker_id, rtos_core_id_get());
    unlock_print();

    int rows_per_worker = N / THREAD;
    int remainder = N % THREAD;

    int start_row;
    int end_row;

    if (worker_id < remainder) {
        start_row = worker_id * (rows_per_worker + 1);
        end_row = start_row + (rows_per_worker + 1);
    } else {
        start_row = remainder * (rows_per_worker + 1) + (worker_id - remainder) * rows_per_worker;
        end_row = start_row + rows_per_worker;
    }
    // lock_print();
    // printf("worker %d, start at %d, end at %d\n", worker_id, start_row, end_row);
    // unlock_print();
    clock_t  tick_task,ticks_per_msec = CLOCKS_PER_SEC/1000;
    tick_task = clock();
    for (int i = 0;i < TIMES;i++){
        for (int r = start_row; r < end_row; r++) {
            const unsigned int *a_row = &A[r * N];
            for (int c = 0; c < N; c++) {
                int acc = 0;
                const unsigned int *bt_row = &B[c * N];
                for (int k = 0; k < N; k++) {
                    acc += a_row[k] * bt_row[k];  
                }
                C[r*N + c] = acc;
            }
        }
        atomic_or(&done_calc[i], (1 << worker_id));
        #ifdef PROFILE_LOG
        clock_t tick = clock();
        #endif
        while (done_calc[i] != task_mask){
        }
        #ifdef PROFILE_LOG
        clock_t tick2 = clock();
        barrier_overhead[worker_id] += (tick2 - tick);
        #endif
        if (worker_id == 0){
            // lock_print();
            // // printf("calculation %d done\n", i);
            // for (int j = 0;j < N * N;j++){
            //     if (C[j] != ans[j]){
            //         printf("wrong answer at index %d\n", j);
            //         exit(1);
            //     }
            // }
            // printf("finish %d\n", i);
            // unlock_print();
        }
        atomic_or(&done_eval[i], 1 << worker_id);
        #ifdef PROFILE_LOG
        tick = clock();
        #endif
        while (done_eval[i] != task_mask){
        }
        #ifdef PROFILE_LOG
        tick2 = clock();
        barrier_overhead[worker_id] += (tick2 - tick);
        #endif
        if (worker_id == 0 && i == TIMES - 1){
            lock_print();
            printf("\n----------------------------------------\n");
            printf("[Coordinator] Compute complete.\n");
            tick_task = (clock() - tick_task) / ticks_per_msec;
            printf("It took %ld ms to multiply two %d x %d matrices %d times.\n\n", tick_task, N, N, TIMES);
            printf("----------------------------------------\n");
            unlock_print();
            #ifdef PROFILE_LOG
            for (int i = 0;i < configNUMBER_OF_CORES;i++){
                xTaskCreateAffinitySet(vProfile_Task, NULL, TASK_STACK_SIZE, NULL,TASK_PRIORITY, 1 << i,NULL);
            }
            start_print_profile = 1;
            #endif
        }
    }
    // for (;;){}
    vTaskDelete(NULL);
}

int main(void) {
    int core_id = rtos_core_id_get();
    if (core_id >= configNUMBER_OF_CORES) {
        while(1);
    }
    if (core_id == COORDINATOR_CORE) {
        *(volatile uint32_t *)PRINT_LOCK_ADDR = 0u;
        *(volatile uint32_t *)MALLOC_LOCK_ADDR = 0u;

        lock_print();
        printf("Core 0: Initializing...\n");
        unlock_print();

        A = (unsigned int *)pvPortMalloc(N * N * sizeof(unsigned int));
        B = (unsigned int *)pvPortMalloc(N * N * sizeof(unsigned int));
        C = (unsigned int *)pvPortMalloc(N * N * sizeof(unsigned int));
        // ans = (unsigned int *)pvPortMalloc(N * N * sizeof(unsigned int));
        if (!A || !B || !C) {printf("malloc fail\n"); while(1);}

        for (int i = 0; i < N * N; i++) { A[i] = rand() % 10; B[i] = rand() % 10; }
        // Transpose matrix B
        for (int i = 0; i < N; ++i) {
            for (int j = i + 1; j < N; ++j) {
                unsigned int tmp = B[i*N + j];
                B[i*N + j] = B[j*N + i];
                B[j*N + i] = tmp;
            }
        }
        
        lock_print();
        printf("Core 0: Creating tasks for a single run...\n");
        unlock_print();
        UBaseType_t mask = (1 << configNUMBER_OF_CORES) - 1; 
        // xTaskCreateAffinitySet(vCoordinatorTask, NULL, TASK_STACK_SIZE, NULL, TASK_PRIORITY, mask, NULL);
        for (int i = 0; i < THREAD; i++) {
            char name[20];
            sprintf(name, "Level1_Worker_%d", i);
            int *id = pvPortMalloc(sizeof(int));  
            *id = i;
            #ifdef AFFINITY_SPECIFIC_CORE
            xTaskCreateAffinitySet(vWorkerTask, name, TASK_STACK_SIZE, (void *)id, TASK_PRIORITY, 1 << i, NULL);
            #else
            xTaskCreateAffinitySet(vWorkerTask, name, TASK_STACK_SIZE, (void *)id, TASK_PRIORITY, mask, NULL);
            #endif
        }

        vTaskStartScheduler();

    } else {
        xPortStartSchedulerOncore();
    }

    for (;;);
    return 0;
}

void vApplicationMallocFailedHook(void) {
    lock_print();
    printf("Malloc failed!\n");
    unlock_print();
    for (;;);
}

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName) { 
    (void)pxTask;
    lock_print();
    printf("Stack overflow in %s\n", pcTaskName);
    unlock_print();
    for (;;);
}

void vApplicationIdleHook(void) {}
void vApplicationTickHook(void) {}

