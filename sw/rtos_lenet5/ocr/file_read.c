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


image *read_images(const char *filename, int *n_images, int *n_rows, int *n_cols)
{
    read_file((char*)filename, fbuf);
    uint8_t *iptr = fbuf;

    int magic = big2little32(iptr); iptr += 4;
    *n_images = big2little32(iptr); iptr += 4;
    *n_rows   = big2little32(iptr); iptr += 4;
    *n_cols   = big2little32(iptr); iptr += 4;

    int size = (*n_rows) * (*n_cols);

    // Allocate memory for all images.
    image *images = (image *)pvPortMalloc(sizeof(image) * (*n_images));
    if (!images) {
        printf("OOM in read_images\n");
        exit(-1);
    }

    for (int idx = 0; idx < *n_images; idx++) {
        for (int jdx = 0; jdx < size; jdx++) {
            images[idx][jdx / (*n_cols)][jdx % (*n_cols)] = *(iptr++);
        }
    }

    return images;
}


LeNet5 *read_lenet5(const char *filename)
{
    read_file(filename, fbuf);
    LeNet5 *net = pvPortMalloc(sizeof(LeNet5));
    memcpy(net, fbuf, sizeof(LeNet5));
    return net;
}


uint8_t *read_labels(char *filename, int *n_labels)
{
    uint8_t *labels;
    uint8_t *ptr;
    int magic;

    read_file(filename, fbuf);

    ptr = fbuf;

    magic = (ptr[0] << 24) | (ptr[1] << 16) | (ptr[2] << 8) | ptr[3];
    if (magic != 0x00000801) {
        printf("read_labels: invalid magic number (0x%x)\n", magic);
        exit(-1);
    }
    ptr += 4;

    *n_labels = (ptr[0] << 24) | (ptr[1] << 16) | (ptr[2] << 8) | ptr[3];
    ptr += 4;

    if ((labels = (uint8_t *) pvPortMalloc(*n_labels)) == NULL) {
        printf("read_labels: out of memory.\n");
        exit(-1);
    }

    memcpy(labels, ptr, *n_labels);

    return labels;
}


image *random_images(int n_images, int n_rows, int n_cols)
{
    image *images = pvPortMalloc(sizeof(image) * n_images);
    if (!images) {
        printf("OOM in random_images\n");
        exit(-1);
    }

    for (int idx = 0; idx < n_images; idx++) {
        for (int r = 0; r < n_rows; r++) {
            for (int c = 0; c < n_cols; c++) {
                images[idx][r][c] = rand() % 256;
            }
        }
    }
    return images;
}

uint8_t *random_labels(int n_labels)
{
    uint8_t *labels = pvPortMalloc(n_labels);
    if (!labels) {
        printf("OOM in random_labels\n");
        exit(-1);
    }

    for (int i = 0; i < n_labels; i++) {
        labels[i] = rand() % 10;
    }
    return labels;
}

LeNet5 *random_lenet5()
{
    LeNet5 *net = pvPortMalloc(sizeof(LeNet5));
    if (!net) {
        printf("OOM in random_lenet5\n");
        exit(-1);
    }

    double scale = 0.1;  // control the range of random numbers

    for (int i = 0; i < INPUT; i++)
        for (int j = 0; j < LAYER1; j++)
            for (int k0 = 0; k0 < LENGTH_KERNEL; k0++)
                for (int k1 = 0; k1 < LENGTH_KERNEL; k1++)
                    net->weight0_1[i][j][k0][k1] = ((double)rand() / RAND_MAX - 0.5) * 2 * scale;

    for (int i = 0; i < LAYER2; i++)
        for (int j = 0; j < LAYER3; j++)
            for (int k0 = 0; k0 < LENGTH_KERNEL; k0++)
                for (int k1 = 0; k1 < LENGTH_KERNEL; k1++)
                    net->weight2_3[i][j][k0][k1] = ((double)rand() / RAND_MAX - 0.5) * 2 * scale;

    for (int i = 0; i < LAYER4; i++)
        for (int j = 0; j < LAYER5; j++)
            for (int k0 = 0; k0 < LENGTH_KERNEL; k0++)
                for (int k1 = 0; k1 < LENGTH_KERNEL; k1++)
                    net->weight4_5[i][j][k0][k1] = ((double)rand() / RAND_MAX - 0.5) * 2 * scale;

    for (int i = 0; i < LAYER5 * LENGTH_FEATURE5 * LENGTH_FEATURE5; i++)
        for (int j = 0; j < OUTPUT; j++)
            net->weight5_6[i][j] = ((double)rand() / RAND_MAX - 0.5) * 2 * scale;

    for (int i = 0; i < LAYER1; i++) net->bias0_1[i] = ((double)rand() / RAND_MAX - 0.5) * 2 * scale;
    for (int i = 0; i < LAYER3; i++) net->bias2_3[i] = ((double)rand() / RAND_MAX - 0.5) * 2 * scale;
    for (int i = 0; i < LAYER5; i++) net->bias4_5[i] = ((double)rand() / RAND_MAX - 0.5) * 2 * scale;
    for (int i = 0; i < OUTPUT; i++) net->bias5_6[i] = ((double)rand() / RAND_MAX - 0.5) * 2 * scale;

    return net;
}
