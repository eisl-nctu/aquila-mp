// =============================================================================
//  Program : file_read.h
//  Author  : Chun-Jen Tsai
//  Date    : Dec/06/2023
// -----------------------------------------------------------------------------
//  Description:
//      This is a library of file reading functions for MNIST test
//  images & labels. It also contains a function for reading the model
//  weights file of a neural network.
//
//  This program is designed as one of the homework project for the course:
//  Microprocessor Systems: Principles and Implementation
//  Dept. of CS, NYCU (aka NCTU), Hsinchu, Taiwan.
// -----------------------------------------------------------------------------
//  Revision information:
//
//  None.
// =============================================================================
#include<lenet.h>

#define big2little32(p) ((p[0]<<24)|(p[1]<<16)|(p[2]<<8)|p[3])

float lerp(float a, float b, float f);
float sample(float *image, float x, float y, int w, int h);
void width_normalize(float *image, int width, int height);

image *read_images(const char *filename, int *n_images, int *n_rows, int *n_cols);
uint8_t *read_labels(char *filename, int *n_labels);
LeNet5 *read_lenet5(const char *filename);
// void read_lenet5(const char *filename, LeNet5 *net);
float *read_weights(char *filename, int *n_layers, int *n_neurons);
// #define DEBUG
#define RANDOM

image *random_images(int n_images, int n_rows, int n_cols);
uint8_t *random_labels(int n_labels);
LeNet5 *random_lenet5();
