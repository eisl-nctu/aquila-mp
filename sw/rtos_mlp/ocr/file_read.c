// =============================================================================
//  Program : file_read.c
//  Author  : Chun-Jen Tsai
//  Date    : Dec/06/2023
// -----------------------------------------------------------------------------
//  Description:
//      This is a library of file reading functions for MNIST test
//  images & labels. It also contains a function for reading the model
//  weights file of a neural network.
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
#include "fat32.h"
#include "file_read.h"
#include "FreeRTOS.h"

// Our FAT32 file I/O routine need a large buffer area to read in
// the entire file before processing. the Arty board has 256MB DRAM.
uint8_t *fbuf  = (uint8_t *) 0x81000000L;

float lerp(float a, float b, float f)
{
    return a+(b-a)*f;
}

float sample(float *image, float x, float y, int w, int h)
{
	unsigned ix = x, iy = y, ix1 = ix + 1, iy1 = iy + 1;
	float fx = x - ix, fy = y - iy;
	int vx = 0 <= ix && ix < w;
	int vy = 0 <= iy && iy < h;
	int vx1 = 0 <= ix1 && ix1 < w;
	int vy1 = 0 <= iy1 && iy1 < h;

	float s0 = vx && vy ? image[iy*w + ix] : 0.0;
	float s1 = vx1 && vy ? image[iy*w + ix1] : 0.0;
	float s2 = vx && vy1 ? image[iy1*w + ix] : 0.0;
	float s3 = vx1 && vy1 ? image[iy1*w + ix1] : 0.0;

	return lerp( lerp(s0, s1, fx), lerp(s2, s3, fx), fy );
}

void width_normalize(float *image, int width, int height)
{
    static float *ibuf;
	int min_x = width, min_y = height, max_x = 0, max_y = 0;
	int size = width*height;

    if ((ibuf = malloc(size*sizeof(float))) == NULL)
    {
        printf("width_normalize: out of memory.\n");
        exit (-1);
    }
	for (int i = 0; i < size; ++i)
	{
		if (image[i] >= 0.1)
		{
			int x = i % width, y = i / width;
			min_x = (min_x < x)? min_x : x;
			min_y = (min_y < y)? min_y : y;
			max_x = (max_x > x)? max_x : x;
			max_y = (max_y > y)? max_y : y;
		}
	}

	if (min_x > max_x || min_y > max_y) return;

	int sub_x = max_x - min_x + 1, sub_y = max_y - min_y + 1;
	float x_mul = (float) sub_x / width;
	float y_mul = (float) sub_y / height;
	for (int i = 0; i < size; ++i)
	{
		int x = i % width, y = i / width;
		float tx = x * x_mul + min_x, ty = y * y_mul + min_y;
		ibuf[i] = sample(image, tx, ty, width, height);
	}
	memcpy(image, ibuf, size*sizeof(float));
	free(ibuf);
}

float **read_images(char *filename, int *n_images, int *n_rows, int *n_cols)
{
    
    float **images;
    int idx, jdx, size;

#ifdef RANDOM

    images = (float **) malloc(sizeof(float *) * *n_images);
    size = IMAGE_SIZE;
    for (idx = 0; idx < N_IMAGES; idx++) 
    {
        images[idx] = (float *) malloc(size*sizeof(float));
        for (jdx = 0; jdx < size; jdx++)
        {   
            images[idx][jdx] = (float) rand() / (float) RAND_MAX;
        }
        // Normalize the image size.
        width_normalize(images[idx], *n_cols, *n_rows);
    }

#else
    uint8_t *iptr;
    read_file(filename, fbuf);
    
    iptr = fbuf;
    iptr += sizeof(int); // skip the ID of the file.
    *n_images = big2little32(iptr);
    iptr += sizeof(int);
    *n_rows = big2little32(iptr);
    iptr += sizeof(int);
    *n_cols = big2little32(iptr);
    iptr += sizeof(int);
    size = (*n_rows) * (*n_cols);
    images = (float **) malloc(sizeof(float *) * *n_images);
    
    for (idx = 0; idx < *n_images; idx++) 
    {
        images[idx] = (float *) malloc(size*sizeof(float));
        for (jdx = 0; jdx < size; jdx++)
        {   
            images[idx][jdx] = (float) *(iptr++) / 255.0;
        }
        // Normalize the image size.
        width_normalize(images[idx], *n_cols, *n_rows);
    }
#endif

    return images;
}

uint8_t *read_labels(char *filename)
{
    uint8_t *labels, n_labels;


    n_labels = read_file(filename, fbuf) - 8;

   
    if ((labels = (uint8_t *) malloc(n_labels)) == NULL)
    {
        printf("read_labels: out of memory.\n");
        exit (-1);        
    }
    memcpy((void *) labels, (void *) (fbuf+8), n_labels);
    
    return labels;
}

float *read_weights(char *filename, int *n_layers, int *n_neurons)
{
    
#ifdef RANDOM

    int total_weights = 0;
    float *weights;
    int idx, widx = 0;

    *n_layers = 3;
    n_neurons[0] = N_NEURONS_0;
    n_neurons[1] = N_NEURONS_1;
    n_neurons[2] = N_NEURONS_2;

    for (idx = 0; idx < (*n_layers - 1); idx++) {
        total_weights += n_neurons[idx] * n_neurons[idx + 1]; // full connect
        total_weights += n_neurons[idx + 1]; // bias
    }

    weights = (float *) malloc(total_weights * sizeof(float));
    if (!weights) {
        printf("random_weights: out of memory\n");
        exit(-1);
    }

    for (widx = 0; widx < total_weights; widx++) {
        weights[widx] = ((float) rand() / RAND_MAX) * 2.0f - 1.0f;
    }

    printf("Total random weights = %d (floats)\n", total_weights);
    return (float *) weights;   

#else
    uint8_t *iptr;
    int size;
    float *weights;

    size = read_file(filename, fbuf);

    iptr = (uint8_t *) fbuf;

    *n_layers = *((int *) iptr);
    iptr += sizeof(int);
    for (int idx = 0; idx < *n_layers; idx++)
    {
        n_neurons[idx] = *((int *) iptr);
        printf("Layer %d neurons = %d\n", idx, n_neurons[idx]);
        iptr += sizeof(int);
    }
    if ((weights = (float *) malloc(size-(iptr-fbuf))) == NULL){
        printf("read_weights: out of memory.\n");
        exit (-1);        
    }
    else{
        int weight_bytes = size - (iptr - fbuf);
        int weight_count = weight_bytes / sizeof(float);
        printf("Total weights = %d (floats)\n", weight_count);
    }
    
    memcpy((void *) weights, (void *) iptr, size-(iptr-fbuf));

    return (float *) weights;
#endif
}
