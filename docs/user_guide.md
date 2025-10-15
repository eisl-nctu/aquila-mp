# User Guide

# Testing Environment
- Vivado version 2024.1
- riscv32-unknown-elf-gcc version 12.2.0
- Verilator version 5.041

## Build the Hardware

1. Run Vivado in batch mode:
    ```
    /Path/To/Vivado/2024.1/bin/vivado -mode batch -source build_[target board].tcl 
    ex:
    /Path/To/Vivado/2024.1/bin/vivado -mode batch -source build_genesys2.tcl 
    ```
2. Run on FPGA:
    - Generate Bitstream.
    - Open GTKterm and program the device.

---
## Verilator Simulation

For detailed Verilator simulation steps, please refer to  
[👉 Verilator Guide](verilator_guide.md)
---

## Software
After running `make`, you can download the program to the FPGA by executing the following command (replace `/dev/ttyUSB0` with the actual serial port of your FPGA):
`cat rtos_run.elf > /dev/ttyUSB0`

### 1. OCR
- **Folder**: `./sw/rtos_ocr`
  Main macro definitions in `FreeRTOSConfig.h` (can be modified as needed):
    - `#define configNUMBER_OF_CORES`: Number of CPU cores, default is 16.
    - `#define THREAD`: Number of threads, can be different from the number of cores.
    - `#define N_NEURONS_0`, `N_NEURONS_1`, `N_NEURONS_2`: Number of neurons in each layer of the neural network.
    - `#define IMAGE_SIZE`: Input data size for a single image.
    - `#define N_IMAGES`: Number of test images.
    - `#define PROFILE_LOG`: Uncomment to enable data cache profile logging.
    - `#define AFFINITY_SPECIFIC_CORE`: Uncomment to assign tasks to specific cores.

### 2. Matrix Multiply
- **Folder**: `./sw/rtos_matrix`
  Main macro definitions in `FreeRTOSConfig.h` (can be modified as needed):
    - `#define configNUMBER_OF_CORES`: Number of CPU cores, default is 16.
    - `#define THREAD`: Number of threads, can be different from the number of cores.
    - `#define N`: Matrix size (matrix is N×N), default is 128.
    - `#define TIMES`: Number of test iterations, default is 100.
    - `#define PROFILE_LOG`: Uncomment to enable data cache profile logging.
    - `#define AFFINITY_SPECIFIC_CORE`: Uncomment to assign tasks to specific cores.

### 3. array sorting
- **Folder**: `./sw/rtos_sorting`
  Main macro definitions in `FreeRTOSConfig.h` (can be modified as needed):
    - `#define configNUMBER_OF_CORES`: Number of CPU cores, default is 16.
    - `#define ARRAY_SIZE`: Array size, default is 48000.
    - `#define PROFILE_LOG`: Uncomment to enable data cache profile logging.
    - `#define AFFINITY_SPECIFIC_CORE`: Uncomment to assign tasks to specific cores.

### 4. Lenet-5
- **Folder**: `./sw/rtos_cnn`
  Main macro definitions in `FreeRTOSConfig.h` (can be modified as needed):
    - `#define configNUMBER_OF_CORES`: Number of CPU cores, default is 16.
    - `#define THREAD`: Number of threads, can be different from the number of cores.
    - `#define PROFILE_LOG`: Uncomment to enable data cache profile logging.
    - `#define AFFINITY_SPECIFIC_CORE`: Uncomment to assign tasks to specific cores.
