#include "FreeRTOS.h"
#include "lenet.h"
#include "task.h"
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#define sqrt my_sqrt
#define TASK_PRIORITY           (tskIDLE_PRIORITY + 1)
#define TASK_STACK_SIZE         (configMINIMAL_STACK_SIZE * 4 )

void atomic_or(volatile unsigned int *addr, int val);
void lock_print();
void unlock_print();
void print_double(double val);

volatile unsigned int load_input_barrier[10] = {0};
volatile unsigned int done_first_conv[60] = {0};
volatile unsigned int conv_first_barrier[10] = {0};
volatile unsigned int conv_second1_barrier[10] = {0};
volatile unsigned int conv_second2_barrier[10] = {0};
volatile unsigned int pooling_first_barrier[10] = {0};
volatile unsigned int conv_third1_barrier[10] = {0};
volatile unsigned int conv_third2_barrier[10] = {0};
volatile unsigned int pooling_second_barrier[10] = {0};
volatile unsigned int predict_finish[10] = {0};
int mask = (1 << THREAD) - 1;

static double my_sqrt(double x)
{
    if (x <= 0) return 0;      // handle non-positive x
    double guess = x;          // initial guess
    for (int i = 0; i < 20; i++) {
        guess = 0.5 * (guess + x / guess);
    }
    return guess;
}

double relu(double x)
{
	return x*(x > 0);
}

double relugrad(double y)
{
	return y > 0;
}

static void vector_x_matrix(double *src, double *mat, double *des, const long height, const long width)
{
	for (long y = 0; y < width; ++y)
	{
		for (long x = 0; x < height; ++x)
		{
			des[y] += src[x] * mat[x*width + y];
		}
	}
}

static void convolute_valid(const double *src, const double *conv, double *des, const long dh, const long dw,
                                  const long ch, const long cw){

    const long sw = dw + cw - 1;    
    const long total = dh * dw;     
	
    for (long idx = 0; idx < total; ++idx) {
        const long d0 = idx / dw;       
        const long d1 = idx % dw;        

        double acc = 0.0;
        const long src_row0 = d0 * sw + d1;

        for (long c0 = 0; c0 < ch; ++c0) {
            for (long c1 = 0; c1 < cw; ++c1) {
                des[d0 * dw + d1] += src[(d0 + c0)*sw + d1 + c1] * conv[c0*cw + c1];
            }
        }
    }
}



static void subsamp_max_forward(double *src, double *des, const long sh, const long sw, const long dh, const long dw, const long n)
{
	const long srcSize = sh * sw, desSize = dh * dw;
	const long lh = sh / dh, lw = sw / dw;
	for (long i = 0; i < n; ++i)
	{
		for (long d0 = 0; d0 < dh; ++d0)
			for (long d1 = 0; d1 < dw; ++d1)
			{
				long x = d0 * lh * sw + d1 * lw;
				for (long l = 1; l < lh * lw; ++l)
				{
					long index = (d0 * lh + l / lw) * sw + d1 * lw + l % lw;
					x += (src[index] > src[x]) * (index - x);
				}
				des[d0 * dw + d1] = src[x];
			}
		src += srcSize;
		des += desSize;
	}
}




#define GETLENGTH(array) (sizeof(array)/sizeof(*(array)))

#define GETCOUNT(array)  (sizeof(array)/sizeof(double))

#define SUBSAMP_MAX_FORWARD(input,output)								\
{																		\
	subsamp_max_forward((double *)input,(double *)output,				\
							GETLENGTH(*input),GETLENGTH(**input),		\
							GETLENGTH(*output),GETLENGTH(**output),GETLENGTH(output));\
}


#define DOT_PRODUCT_FORWARD(input,output,weight,bias,action)				\
{																			\
	dot_product_forward((double *)input,(double *)weight,(double *)output,	\
				(double *)bias,action,GETLENGTH(weight),GETLENGTH(*weight));\
}


static void dot_product_forward(double *src, double *mat, double *des,double *bias, double(*active)(double), const long height, const long width)
{
	vector_x_matrix(src, mat, des, height, width);
	for (int i = 0; i < width; ++i)
		des[i] = active(des[i] + bias[i]);
}




void load_input(Feature *features, image input)
{
	double (*layer0)[LENGTH_FEATURE0][LENGTH_FEATURE0] = features->input;
	long sz = sizeof(image) / sizeof(**input);

	// double mean = 0, std = 0;
	double *mean = pvPortMalloc(sizeof(double));
	double *std = pvPortMalloc(sizeof(double));

	if (mean == NULL) {
		printf("malloc for mean failed!\n");
		vTaskDelete(NULL); // kill the thread, or do error handling here
	}

	*mean = 0;
	*std = 0;

	for(int i = 0;i<28;i++){
		for(int j = 0;j<28;j++){
			*mean += input[i][j];
			*std += input[i][j] * input[i][j];
		}
	}

	*mean /= sz;
	*std = sqrt(*std / sz - (*mean) * (*mean));
	for(int j = 0; j < sizeof(image) / sizeof(*input); ++j){
		for(int k = 0; k < sizeof(*input) / sizeof(**input); ++k){
			layer0[0][j + PADDING][k + PADDING] = (input[j][k] - *mean) / *std;
		}
	}

	vPortFree(mean);
	vPortFree(std);
}

uint8 get_result(double *output, uint8 count)
{
	uint8 result = 0;
	for (uint8 i = 1; i < count; ++i)
		result += (i - result) * (output[i] > output[result]);
	return result;
}

//////////////////////////////////// first layer convolution //////////////////////////////////////////////////////////////

#define CONVOLUTION_FIRST_LAYER_FORWARD(input,output,weight,bias,action, worker_id, image_index)								    \
{																							    \
	convolution_first_layer_forward((double *)input, (double *)weight, (double *)output, (double *)bias,	\
			action,GETLENGTH(*output),GETLENGTH(**output),GETLENGTH(**weight),				    \
				GETLENGTH(***weight),GETLENGTH(weight),GETLENGTH(*weight), worker_id, image_index);					    \
}

static void convolute_first_layer_valid(const double *src, const double *conv, double *des, const long dh, const long dw, const long ch, const long cw,
	 		int worker_id, int imagex_index, int output_channel_num){

    const long sw = dw + cw - 1;    
    const long total = dh * dw;     

	int start_index = total / THREAD * (worker_id);
	int end_index   = total / THREAD * (worker_id + 1);
	
    for (long idx = start_index; idx < end_index; ++idx) {
        const long d0 = idx / dw;       
        const long d1 = idx % dw;        

        for (long c0 = 0; c0 < ch; ++c0) {
            for (long c1 = 0; c1 < cw; ++c1) {
                des[d0 * dw + d1] += src[(d0 + c0)*sw + d1 + c1] * conv[c0*cw + c1];
            }
        }
    }

	atomic_or(&done_first_conv[imagex_index * 6 + output_channel_num], 1 << worker_id);
	while (done_first_conv[imagex_index * 6 + output_channel_num] != mask);
}


static void convolution_first_layer_forward(
    double *src,        // feature map (one dimension)
    double *conv,       // weights，size sn*dn*(ch*cw)
    double *des,        // feature map
    double *bias,       // channel bias
    double(*active)(double), // activation function
    const long dh,      //  (destination height)
    const long dw,      // (destination width)
    const long ch,      // kernel height
    const long cw,      // kernel width
    const long sn,      // input channel 
    const long dn,       // output channel 
	const int  worker_id,
	const int  image_index
)
{
    const long srcSize = (dh + ch - 1) * (dw + cw - 1);

    const long desSize = dh * dw;

    const long convSize = ch * cw;

    for (int y = 0; y < dn; ++y){                
        for (int x = 0; x < sn; ++x){         
			convolute_first_layer_valid(src + x * srcSize, conv + (x * dn + y) * convSize, des + y * desSize, dh, dw, ch, cw, worker_id, image_index, y);
		}
	}

	if(worker_id == 0){
    	for (int i = 0; i < dn; ++i) {
    	    double *desMat = des + i * desSize;     
    	    for (int j = 0; j < desSize; ++j) {
    	        desMat[j] = active(desMat[j] + bias[i]);
    	    }
    	}
	}

	atomic_or(&conv_first_barrier[image_index], (1 << worker_id));
	while (conv_first_barrier[image_index] != mask);
}

//////////////////////////////////// second layer convolution //////////////////////////////////////////////////////////////

#define CONVOLUTION_SECOND_LAYER_FORWARD(input,output,weight,bias,action, worker_id, image_index)								    \
{																							    \
	convolution_second_layer_forward((double *)input, (double *)weight, (double *)output, (double *)bias,	\
			action,GETLENGTH(*output),GETLENGTH(**output),GETLENGTH(**weight),				    \
				GETLENGTH(***weight),GETLENGTH(weight),GETLENGTH(*weight), worker_id, image_index);					    \
}

static void convolute_second_layer_valid(const double *src, const double *conv, double *des, const long dh, const long dw, const long ch, const long cw){


    const long sw = dw + cw - 1;    
    const long total = dh * dw;     
	
    for (long idx = 0; idx < total; ++idx) {
        const long d0 = idx / dw;       
        const long d1 = idx % dw;        

        double acc = 0.0;
        const long src_row0 = d0 * sw + d1;

        for (long c0 = 0; c0 < ch; ++c0) {
            for (long c1 = 0; c1 < cw; ++c1) {
                des[d0 * dw + d1] += src[(d0 + c0)*sw + d1 + c1] * conv[c0*cw + c1];
            }
        }
    }
}

static void convolution_second_layer_forward(
    double *src,        // feature map (one dimension)
    double *conv,       // weights，size sn*dn*(ch*cw)
    double *des,        // feature map
    double *bias,       // channel bias
    double(*active)(double), // activation function
    const long dh,      //  (destination height)
    const long dw,      // (destination width)
    const long ch,      // kernel height
    const long cw,      // kernel width
    const long sn,      // input channel 
    const long dn,       // output channel 
	const int  worker_id,
	const int  image_index
)
{
    const long srcSize = (dh + ch - 1) * (dw + cw - 1);

    const long desSize = dh * dw;

    const long convSize = ch * cw;
	for (int y = 0; y < dn; ++y){
		if(worker_id == y % THREAD){
			for (int x = 0; x < sn; ++x){  
				convolute_second_layer_valid(src + x * srcSize, conv + (x * dn + y) * convSize, des + y * desSize, dh, dw, ch, cw);
			}
		}
	}

	atomic_or(&conv_second1_barrier[image_index], (1 << worker_id));
	while (conv_second1_barrier[image_index] != mask);

    if(worker_id == 0){
		for (int i = 0; i < dn; ++i) {
    	    double *desMat = des + i * desSize;     
    	    for (int j = 0; j < desSize; ++j) {
    	        desMat[j] = active(desMat[j] + bias[i]);
    	    }
    	}
	}

	atomic_or(&conv_second2_barrier[image_index], (1 << worker_id));
	while (conv_second2_barrier[image_index] != mask);

}

//////////////////////////////////// third layer convolution //////////////////////////////////////////////////////////////

#define CONVOLUTION_THIRD_LAYER_FORWARD(input,output,weight,bias,action, worker_id, image_index)								    \
{																							    \
	convolution_third_layer_forward((double *)input, (double *)weight, (double *)output, (double *)bias,	\
			action,GETLENGTH(*output),GETLENGTH(**output),GETLENGTH(**weight),				    \
				GETLENGTH(***weight),GETLENGTH(weight),GETLENGTH(*weight), worker_id, image_index);					    \
}

static void convolute_third_layer_valid(const double *src, const double *conv, double *des, const long dh, const long dw, const long ch, const long cw){


    const long sw = dw + cw - 1;    
    const long total = dh * dw;     
	
    for (long idx = 0; idx < total; ++idx) {
        const long d0 = idx / dw;       
        const long d1 = idx % dw;        

        double acc = 0.0;
        const long src_row0 = d0 * sw + d1;

        for (long c0 = 0; c0 < ch; ++c0) {
            for (long c1 = 0; c1 < cw; ++c1) {
                des[d0 * dw + d1] += src[(d0 + c0)*sw + d1 + c1] * conv[c0*cw + c1];
            }
        }
    }
}

static void convolution_third_layer_forward(
    double *src,        // feature map (one dimension)
    double *conv,       // weights，size sn*dn*(ch*cw)
    double *des,        // feature map
    double *bias,       // channel bias
    double(*active)(double), // activation function
    const long dh,      //  (destination height)
    const long dw,      // (destination width)
    const long ch,      // kernel height
    const long cw,      // kernel width
    const long sn,      // input channel 
    const long dn,       // output channel 
	const int  worker_id,
	const int  image_index
)
{
    const long srcSize = (dh + ch - 1) * (dw + cw - 1);

    const long desSize = dh * dw;

    const long convSize = ch * cw;
	for (int y = 0; y < dn; ++y){  
		if(worker_id == (y % THREAD)){
				for (int x = 0; x < sn; ++x){
				convolute_third_layer_valid(src + x * srcSize, conv + (x * dn + y) * convSize, des + y * desSize, dh, dw, ch, cw);
			}
		}
	}

	atomic_or(&conv_third1_barrier[image_index], (1 << worker_id));
	while (conv_third1_barrier[image_index] != mask);

    if(worker_id == 0){
		for (int i = 0; i < dn; ++i) {
    	    double *desMat = des + i * desSize;     
    	    for (int j = 0; j < desSize; ++j) {
    	        desMat[j] = active(desMat[j] + bias[i]);
    	    }
    	}
	}

	atomic_or(&conv_third2_barrier[image_index], (1 << worker_id));
	while (conv_third2_barrier[image_index] != mask);

}

#define SUBSAMP_FIRST_LAYER_MAX_FORWARD(input,output, worker_id, image_index)								\
{																		\
	subsamp_first_layer_max_forward((double *)input,(double *)output,				\
							GETLENGTH(*input),GETLENGTH(**input),		\
							GETLENGTH(*output),GETLENGTH(**output),GETLENGTH(output), worker_id, image_index);\
}

static void subsamp_first_layer_max_forward(double *src, double *des, const long sh, const long sw, const long dh, const long dw, const long n, int worker_id, int image_index)
{
	const long srcSize = sh * sw, desSize = dh * dw;
	const long lh = sh / dh, lw = sw / dw;
	for (long i = 0; i < n; ++i)
	{
		if(worker_id == (n % THREAD)){
			for (long d0 = 0; d0 < dh; ++d0)
				for (long d1 = 0; d1 < dw; ++d1)
				{
					long x = d0 * lh * sw + d1 * lw;
					for (long l = 1; l < lh * lw; ++l)
					{
						long index = (d0 * lh + l / lw) * sw + d1 * lw + l % lw;
						x += (src[index] > src[x]) * (index - x);
					}
					des[d0 * dw + d1] = src[x];
				}
			src += srcSize;
			des += desSize;
		}
	}
}

#define SUBSAMP_SECOND_LAYER_MAX_FORWARD(input,output, worker_id, image_index)								\
{																		\
	subsamp_second_layer_max_forward((double *)input,(double *)output,				\
							GETLENGTH(*input),GETLENGTH(**input),		\
							GETLENGTH(*output),GETLENGTH(**output),GETLENGTH(output), worker_id, image_index);\
}

static void subsamp_second_layer_max_forward(double *src, double *des, const long sh, const long sw, const long dh, const long dw, const long n, int worker_id, int image_index)
{
	const long srcSize = sh * sw, desSize = dh * dw;
	const long lh = sh / dh, lw = sw / dw;
	for (long i = 0; i < n; ++i)
	{
		if(worker_id == (n % THREAD)){
			for (long d0 = 0; d0 < dh; ++d0)
				for (long d1 = 0; d1 < dw; ++d1)
				{
					long x = d0 * lh * sw + d1 * lw;
					for (long l = 1; l < lh * lw; ++l)
					{
						long index = (d0 * lh + l / lw) * sw + d1 * lw + l % lw;
						x += (src[index] > src[x]) * (index - x);
					}
					des[d0 * dw + d1] = src[x];
				}
			src += srcSize;
			des += desSize;
		}
	}
}

uint8 Predict(LeNet5 *lenet, image input, uint8 count, Feature *features, int worker_id, int image_index)
{	

	if(worker_id == 0){

		memset(features, 0, sizeof(Feature));
		load_input(features, input);
		
		
		atomic_or(&load_input_barrier[image_index], (1 << worker_id));
		while (load_input_barrier[image_index] != mask);
		// lock_print();
		// printf("finish load input barrier\n");
		// unlock_print();

		CONVOLUTION_FIRST_LAYER_FORWARD(features->input, features->layer1, lenet->weight0_1, lenet->bias0_1, relu, worker_id, image_index);
		// lock_print();
		// printf("finish first layer convolution barrier\n");
		// unlock_print();

		SUBSAMP_FIRST_LAYER_MAX_FORWARD(features->layer1, features->layer2, worker_id, image_index);
		atomic_or(&pooling_first_barrier[image_index], (1 << worker_id));
		while (pooling_first_barrier[image_index] != mask);

    	CONVOLUTION_SECOND_LAYER_FORWARD(features->layer2, features->layer3, lenet->weight2_3, lenet->bias2_3, relu, worker_id, image_index);

    	SUBSAMP_SECOND_LAYER_MAX_FORWARD(features->layer3, features->layer4, worker_id, image_index);
		atomic_or(&pooling_second_barrier[image_index], (1 << worker_id));
		while (pooling_second_barrier[image_index] != mask);
		
    	CONVOLUTION_THIRD_LAYER_FORWARD(features->layer4, features->layer5, lenet->weight4_5, lenet->bias4_5, relu, worker_id, image_index);

    	DOT_PRODUCT_FORWARD(features->layer5, features->output, lenet->weight5_6, lenet->bias5_6, relu);
		// lock_print();
		// for(int i = 0;i<10;i++){
		// 	print_double(features->output[i]); printf(" ");
		// }
		// printf("\n");
		// unlock_print();
		uint8 result = get_result(features->output, count);
		atomic_or(&predict_finish[image_index], 1 << worker_id);
		while (predict_finish[image_index] != mask);
	
		return result;

	}
	else{

		atomic_or(&load_input_barrier[image_index], (1 << worker_id));
		while (load_input_barrier[image_index] != mask);

		CONVOLUTION_FIRST_LAYER_FORWARD(features->input, features->layer1, lenet->weight0_1, lenet->bias0_1, relu, worker_id, image_index);

		SUBSAMP_FIRST_LAYER_MAX_FORWARD(features->layer1, features->layer2, worker_id, image_index);
		atomic_or(&pooling_first_barrier[image_index], (1 << worker_id));
		while (pooling_first_barrier[image_index] != mask);

		CONVOLUTION_SECOND_LAYER_FORWARD(features->layer2, features->layer3, lenet->weight2_3, lenet->bias2_3, relu, worker_id, image_index);
		
		SUBSAMP_SECOND_LAYER_MAX_FORWARD(features->layer3, features->layer4, worker_id, image_index);
		atomic_or(&pooling_second_barrier[image_index], (1 << worker_id));
		while (pooling_second_barrier[image_index] != mask);

		CONVOLUTION_THIRD_LAYER_FORWARD(features->layer4, features->layer5, lenet->weight4_5, lenet->bias4_5, relu, worker_id, image_index);
		
		atomic_or(&predict_finish[image_index], 1 << worker_id);
		while (predict_finish[image_index] != mask);

	}
	
}

void Initial(LeNet5 *lenet)
{
	//srand((unsigned)time(0));
	for (double *pos = (double *)lenet->weight0_1; pos < (double *)lenet->bias0_1; *pos++ = rand()*(2. / RAND_MAX) - 1);
	for (double *pos = (double *)lenet->weight0_1; pos < (double *)lenet->weight2_3; *pos++ *= sqrt(6.0 / (LENGTH_KERNEL * LENGTH_KERNEL * (INPUT + LAYER1))));
	for (double *pos = (double *)lenet->weight2_3; pos < (double *)lenet->weight4_5; *pos++ *= sqrt(6.0 / (LENGTH_KERNEL * LENGTH_KERNEL * (LAYER2 + LAYER3))));
	for (double *pos = (double *)lenet->weight4_5; pos < (double *)lenet->weight5_6; *pos++ *= sqrt(6.0 / (LENGTH_KERNEL * LENGTH_KERNEL * (LAYER4 + LAYER5))));
	for (double *pos = (double *)lenet->weight5_6; pos < (double *)lenet->bias0_1; *pos++ *= sqrt(6.0 / (LAYER5 + OUTPUT)));
	for (int *pos = (int *)lenet->bias0_1; pos < (int *)(lenet + 1); *pos++ = 0);
}

