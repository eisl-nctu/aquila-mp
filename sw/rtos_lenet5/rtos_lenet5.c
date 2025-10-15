// =============================================================================
//  Program : rtos_lenet5.c
//  Author  : Ye Chen
//  Date    : Sept/23/2025
// =============================================================================
//  This program runs LeNet-5 inference on FreeRTOS SMP. Input samples are split
//  across worker tasks pinned to different cores (data-parallel); a coordinator
//  handles setup, timing, and aggregation. Ensure shared buffers (weights,
//  scratch) are thread-safe to avoid races.
//  Original reference (LeNet-5 codebase): https://github.com/wanwujiedao/LeNet-5
// -----------------------------------------------------------------------------
//  Revision information:
//
//  None.
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

#include "file_read.h"
#include "lenet.h"

#define TASK_STACK_SIZE         (configMINIMAL_STACK_SIZE * 10)
#define TASK_PRIORITY           (tskIDLE_PRIORITY + 1)
#define COORDINATOR_CORE        0 
int correct = 0;
#ifdef PROFILE_LOG
volatile uint32_t start_print_profile = 0;
// Mask to track which cores have reported profiling data
volatile uint32_t g_profile_done_mask = 0;
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
#endif

extern void freertos_risc_v_trap_handler(void);
extern void xPortStartSchedulerOncore(void);

typedef struct {
    LeNet5 *net;
    image *images;
    uint8_t *labels;
    Feature *feature;
    int total;
    int worker_id;
} TaskParam;

static TaskParam workerParams[THREAD];


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

void atomic_or(volatile uint32_t *addr, int val) {
    __asm__ volatile("amoor.w.aqrl zero, %1, %0" : "+A"(*addr) : "r"(val) : "memory");
}

void atomic_and(volatile uint32_t *addr, int val) {
    __asm__ volatile("amoand.w.aqrl zero, %1, %0"
                     : "+A"(*addr)
                     : "r"(val)
                     : "memory");
}


void print_double_hex(const char *name, double *addr) {
    uint32_t *p = (uint32_t *)addr;   // the address of reinterpret double
    printf("%s addr = %x value = %f\n", name, addr, *addr);
    printf("Low  : 0x%x\n", p[0]);
    printf("High : 0x%x\n", p[1]);
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
        unlock_print();
        // print_context_switch_overhead();

        exit(0);
    }

    vTaskDelete(NULL); // worker done
}
#endif

void vCoordinatorTask(void *pvParameters) {

    lock_print();
    printf("[Coordinator] (2) Perform the hand-written digits recognition test.\n");
    printf("[Coordinator] Begin computing ...\n");
    unlock_print();

    TaskParam *param = (TaskParam *)pvParameters; 
    clock_t  tick, ticks_per_msec;
    int correct = 0;

    ticks_per_msec = CLOCKS_PER_SEC/1000;
    tick = clock();
    // int answer = testing(param->net, param->images, param->labels, 10, param->feature);
   
    for (int i = 0; i < 10; ++i)
	{
        // lock_print();
		printf("Start calculating picture number %d\n", i);
        // unlock_print();
		int predict_result = Predict(param->net,  param->images[i], 10, param->feature, param->worker_id, i);
		// correct += (param->labels[i] == predict_result);
        // lock_print();
        // printf("l = %d, p = %d", param->labels[i] , predict_result);
        // printf("\n\n");
        // unlock_print();
	}

    tick = (clock() - tick) / ticks_per_msec;
    
    lock_print();
    // printf("correct = %d\n", correct);
    printf("It took %ld msec to read files from the SD card.\n\n", tick);
    unlock_print();

    #ifdef PROFILE_LOG
    for (int i = 0;i < configNUMBER_OF_CORES;i++){
        xTaskCreateAffinitySet(vProfile_Task, NULL, TASK_STACK_SIZE, NULL,TASK_PRIORITY, 1 << i,NULL);
    }
    start_print_profile = 1;
    #else
    exit(0);
    #endif
    for (;;){}
}

void vWorkerTask_level(void *pvParameters) {
    
    TaskParam *param = (TaskParam *)pvParameters; 
    const char *task_name = pcTaskGetName(NULL);

    for (int i = 0;i < 10;i++){
        Predict(param->net,  param->images[i], 10, param->feature, param->worker_id, i);
    }

    while(1);
    
    vTaskDelete(NULL);
    // for (;;){}
}

int main(void) {
    int core_id = rtos_core_id_get();
    if (core_id >= THREAD) {
        while(1);
    }
    if (core_id == COORDINATOR_CORE) {

        *(volatile uint32_t *)PRINT_LOCK_ADDR = 0u;
        *(volatile uint32_t *)MALLOC_LOCK_ADDR = 0u;

        // try to read files from sd card on core 0
        lock_print();
        printf("\n(1) Reading the test images, labels, and neural weights.\n");
        unlock_print();
        
        int n_labels;
        int n_images, n_rows, n_cols;
        
        uint8_t *cnn_labels = random_labels(10);
        image *cnn_images = random_images( 10, 28, 28);
        LeNet5 *net = random_lenet5();

        // uint8_t *cnn_labels = read_labels("t10k-labels-idx1-ubyte", &n_labels);
        // image *cnn_images = read_images("t10k-images-idx3-ubyte", &n_images, &n_rows, &n_cols);
        // LeNet5 *net = read_lenet5("model.dat");
        
        Feature *feature = pvPortMalloc(sizeof(Feature));
        memset(feature, 0, sizeof(Feature));

        for (int i = 0;i<THREAD;i++){
            workerParams[i].net = net;
            workerParams[i].images = cnn_images;
            workerParams[i].labels = cnn_labels;
            workerParams[i].feature = feature;  
            workerParams[i].total = 10;
            workerParams[i].worker_id = i;
        }

        xTaskCreateAffinitySet(vCoordinatorTask, "Coordinator", TASK_STACK_SIZE, (void *)&workerParams[0], TASK_PRIORITY, (1 << COORDINATOR_CORE), NULL);
        
        UBaseType_t mask = (1 << configNUMBER_OF_CORES) - 1; 
        for (int i = 1; i < THREAD; i++) {
            char name[20];
            sprintf(name, "Level1_Worker", i);
            #ifdef AFFINITY_SPECIFIC_CORE
            xTaskCreateAffinitySet(vWorkerTask_level, name, TASK_STACK_SIZE, (void *)&workerParams[i],TASK_PRIORITY,  1 << i,NULL);
            #else
            xTaskCreateAffinitySet(vWorkerTask_level, name, TASK_STACK_SIZE, (void *)&workerParams[i], TASK_PRIORITY, mask, NULL);
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

