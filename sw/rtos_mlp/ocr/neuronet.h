// =============================================================================
//  Program : neuronet.h
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

#define MAX_LAYERS 8

typedef struct __NeuroNet
{
    float *neurons;             // Array that stores all the neuron values.
    float *weights;             // Array that store all the weights & biases.

    float **previous_neurons;   // Pointers to the previous-layer neurons.
    float **forward_weights;    // Pointers to the weights & bias.

    int n_neurons[MAX_LAYERS];  // The # of neurons in each layer.
    int total_layers;           // The total # of layers.
    int total_neurons;          // The total # of neurons.
    int total_weights;          // The total # of weights.
    float *output;              // Pointer to the neurons of the output layer.
} NeuroNet;


void neuronet_init(NeuroNet *nn, int n_layers, int *n_neurons);
void neuronet_load(NeuroNet *nn, float *weights);
void neuronet_free(NeuroNet *nn);
int  neuronet_eval(NeuroNet *nn, float *images, int hart_id, int img_idx);
float relu(float x);

#define CORE_NUM 8
/* For 2-Core */
#define LOCK_S 0x111
#define LOCK_0 0x222
#define LOCK_1 0x333
#define LOCK_2 0x465
#define LOCK_3 0x567
#define LOCK_INIT 0x444
#define LOCK_EVAL 0x555
#define LOCK_EVAL_MEMCPY 0x666
#define LOCK_EVAL_INNPROC 0x777
#define LOCK_DONE 0x888
#define MULT_CORE
#define LOCK_INITIAL 0x999




extern NeuroNet nn;
extern int n_layers, n_neurons[MAX_LAYERS], class_id;
extern int n_images, n_rows, n_cols;
extern unsigned correct_count;
extern clock_t  tick, ticks_per_msec;

// Data from SD card
extern float **images; 
extern uint8_t *labels;
extern float *weights;

void initialize();


void acquire();
void release();
int min(int a, int b);
// #define DEBUG
