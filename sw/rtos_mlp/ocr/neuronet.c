// =============================================================================
//  Program : neuronet.c
//  Author  : Chun-Jen Tsai
//  Date    : Dec/06/2023
// -----------------------------------------------------------------------------
//  Description:
//      This is a neural network library that can be used to implement
//  a inferencing model of the classical multilayer perceptorn (MLP) neural
//  network. It reads a model weights file to setup the MLP. The MLP
//  can have up to MAX_LAYERS, which is defined in neuronet.h. To avoid using
//  the C math library, the relu() fucntion is used for the activation
//  function. This degrades the recognition accuracy significantly, but it
//  serves the teaching purposes well.
//
//  This program is designed as one of the homework projects for the course:
//  Microprocessor Systems: Principles and Implementation
//  Dept. of CS, NYCU (aka NCTU), Hsinchu, Taiwan.
// -----------------------------------------------------------------------------
//  Revision information:
//
//  None.
// =============================================================================

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include "FreeRTOSConfig.h"
#include "neuronet.h"
// #include ""
/* For 2-Core */
volatile unsigned int *print_lock = (unsigned int *)0x80000020U;
volatile unsigned int *done_init = (unsigned int *)0x80000030U;  

// neuronet.h
extern volatile uint32_t done_memcpy[100];
extern volatile uint32_t done_eval[100];
extern volatile uint32_t done_calc[100];
extern volatile uint32_t done_layer_calc1[100];
extern volatile uint32_t done_layer_calc2[100];

int mask = (1 << THREAD) - 1;
NeuroNet nn;
int n_layers, n_neurons[MAX_LAYERS], class_id;
int n_images, n_rows, n_cols;
unsigned correct_count;
clock_t  tick, ticks_per_msec;

float **images = NULL; 
uint8_t *labels = NULL;
float *weights = NULL; 

void initialize() {
    correct_count = 0;
    ticks_per_msec = CLOCKS_PER_SEC/1000;
}

int min(int a, int b) {
    return (a < b) ? a : b;
}
void atomic_or(volatile uint32_t *addr, uint32_t mask);
void lock_print() ;
void unlock_print();
// __attribute__((optimize("O0")))  void atomic_or(volatile unsigned int *addr, int val) {
//     int old = *addr;
//     asm volatile (
//         "amoor.w %0, %2, %1\n"
//         : "=r"(old), "+A"(*addr)
//         : "r"(val)
//         : "memory"
//     );
// }   

__attribute__((optimize("O0")))  void acquire(void) {
    asm volatile ("lui t0, %hi(print_lock)");
    asm volatile ("lw t2, %lo(print_lock)(t0)");
    asm volatile ("li t0, 1");
    asm volatile ("again:");
    asm volatile ("lw t1, (t2)"); // [t2] initial value is zero.
    asm volatile ("bnez t1, again"); // if [t1] isn't zero , jumo to "again" 
    asm volatile ("amoswap.w.aq t1, t0, (t2)");
    asm volatile ("bnez t1, again");
}

__attribute__((optimize("O0"))) void release(void) {
    asm volatile ("lui t0, %hi(print_lock)");
    asm volatile ("lw t2, %lo(print_lock)(t0)");
    asm volatile ("amoswap.w.rl x0, x0, (t2)");
}

/* nn */
void neuronet_init(NeuroNet *nn, int n_layers, int *n_neurons)
{
    int layer_idx, neuron_idx, sum;
    float *head[MAX_LAYERS];  // Pointer to the first neuron value of each layer.

    if (n_layers < 2 || n_layers > MAX_LAYERS)
    {
        printf("ERROR!!!\n");
        printf("layer count is less than 2 or larger than %d\n", MAX_LAYERS);
        return;
    }

    nn->total_neurons = 0;
    
    for (layer_idx = 0; layer_idx < n_layers; layer_idx++)
    {
        nn->n_neurons[layer_idx] = n_neurons[layer_idx];
        nn->total_neurons += n_neurons[layer_idx];
    }

    nn->neurons = (float *) malloc(sizeof(float *) * nn->total_neurons);
    nn->forward_weights = (float **) malloc(sizeof(float *) * nn->total_neurons);
    nn->previous_neurons = (float **) malloc(sizeof(float *) * nn->total_neurons);
    nn->total_layers = n_layers;

    neuron_idx = 0;
    for (layer_idx = 0; layer_idx < nn->total_layers; layer_idx++)
    {
        head[layer_idx] = &(nn->neurons[neuron_idx]);
        neuron_idx += nn->n_neurons[layer_idx];
    }

    // Set a shortcut pointer to the output layer neuron values.
    nn->output = head[nn->total_layers - 1];

    // Set the previous layer neuron pointers for each hidden & output neuron
    for (neuron_idx = nn->n_neurons[0], layer_idx = 1; layer_idx < nn->total_layers; layer_idx++)
    {
        for (unsigned j = 0; j < nn->n_neurons[layer_idx]; ++j, ++neuron_idx)
        {
            nn->previous_neurons[neuron_idx] = head[layer_idx - 1];
        }
    }

    // Initialize the weight array.
    nn->total_weights = 0;
    for (layer_idx = 1; layer_idx < nn->total_layers; layer_idx++)
    {
        // Accumulating # of weights, including one bias value per neuron.
        nn->total_weights += (nn->n_neurons[layer_idx-1] + 1)*nn->n_neurons[layer_idx];
    }
    nn->weights = (float *) malloc(sizeof(float) * nn->total_weights);

    // Set the starting pointer to the forward weights for each neurons.
    sum = 0, neuron_idx = nn->n_neurons[0];
    for (layer_idx = 1; layer_idx < nn->total_layers; layer_idx++)
    {
        for (int idx = 0; idx < nn->n_neurons[layer_idx]; idx++, neuron_idx++)
        {
            nn->forward_weights[neuron_idx] = &(nn->weights[sum]);
            sum += (nn->n_neurons[layer_idx-1] + 1); // add one for bias.
        }
    }
}

void neuronet_load(NeuroNet *nn, float *weights)
{   
    for (int idx = 0; idx < nn->total_weights; idx++)
    {
        nn->weights[idx] = *(weights++);   
    }
    return;
}

void neuronet_free(NeuroNet *nn)
{
    free(nn->weights);
    free(nn->previous_neurons);
    free(nn->forward_weights);
    free(nn->neurons);
}

int neuronet_eval(NeuroNet *nn, float *images, int worker_id, int img_idx)
{
    float inner_product, max;
    float *p_neuron, *p_weight;
    int idx, layer_idx, neuron_idx, max_idx;

    if(worker_id == 0){
        memcpy((void *) nn->neurons, (void *) images, nn->n_neurons[0]*sizeof(float));

    }

    
    //////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////
    atomic_or(&done_memcpy[img_idx], 1 << worker_id);
    while (done_memcpy[img_idx] != mask);
    //////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////

    // Forward computations 
    neuron_idx = nn->n_neurons[0];
    for (layer_idx = 1; layer_idx < nn->total_layers; layer_idx++){
        int total_size = nn->n_neurons[layer_idx];
        // 10 / 8 = 1
        // 2 
        //0 - 1 2 - 3 4 - 5 6 -7 8-9
        int block_size = total_size / THREAD;
        int remainder = total_size % THREAD;
        int start, end;
        if(worker_id < remainder){
            start = (block_size + 1) * worker_id;
            end = start + block_size + 1;
        }
        else {
            start = block_size * worker_id + remainder;
            end = start + block_size;
        }
        // if(hart_id == 0) {
        //     for(int x = 0;x < 8;x++){
        //         int tmp_start;
        //         int tmp_end ;
        //         if(x < remainder){
        //             tmp_start = (block_size + 1) * x;
        //             tmp_end = tmp_start + block_size + 1;
        //         }
        //         else {
        //             tmp_start = block_size * x + remainder;
        //             tmp_end = tmp_start + block_size;
        //         }
        //         printf("hart:%d start = %d, end = %d\n",x, tmp_start, tmp_end);
        //     }
        // }
        for (idx = start; idx < end; idx++){
            // 'p_weight' points to the first forward weight of a layer.
            p_weight = nn->forward_weights[neuron_idx + idx];
            p_neuron = nn->previous_neurons[neuron_idx + idx];
            
            inner_product = 0.0;

            // Loop over all forward-connected neural links.

            for (int jdx = 0; jdx < nn->n_neurons[layer_idx-1]; jdx++){
                inner_product += (*p_neuron++) * (*p_weight++);
            }
            inner_product += *(p_weight); // The last weight of a neuron is the bias.
            nn->neurons[neuron_idx + idx] = relu(inner_product);
#ifdef DEBUG
            acquire();
            printf("[core %d] %f\n", hart_id, inner_product);
            release();
#endif
        }
        neuron_idx += total_size;
        if(layer_idx == 1){
            atomic_or(&done_layer_calc1[img_idx], 1 << worker_id);
            while (done_layer_calc1[img_idx] != mask);
        }
        else{
            atomic_or(&done_layer_calc2[img_idx], 1 << worker_id);
            while (done_layer_calc2[img_idx] != mask);       
        }

    }
    //////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////
    // atomic_or(&done_calc[img_idx], 1 << worker_id);
    // while (done_calc[img_idx] != mask);
    //////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////

    max_idx = 0;
    if(worker_id == 0){
        max = -1.0;
        for (idx = 0; idx < nn->n_neurons[nn->total_layers-1]; idx++)
        {
            if (max < nn->output[idx])
            {
                max_idx = idx;
                max = nn->output[idx];      
            }  
        }
    }

    //////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////
    atomic_or(&done_eval[img_idx], 1 << worker_id);
    while (done_eval[img_idx] != mask);
    // lock_print();
    // printf("%d finsh working\n");
    // unlock_print();
    //////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////

    
    return max_idx;
}

float relu(float x)
{
	return x < 0.0 ? 0.0 : x;
}
