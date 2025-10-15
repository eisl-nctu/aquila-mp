# The Aquila multi-Core RISC-V SoC

---

Aquila-mp is an open-source multi-core (at mote 16 cores on Genesys2 FPGA board) system-on-chip featuring a 32-bit RISC-V RV32IMA processor, extended by the [Aquila SoC](https://github.com/eisl-nctu/aquila). It employs the MESI cache coherence protocol to maintain data consistency across L1 and L2 caches in a shared memory system. The processor supports atomic instructions for efficient synchronization and mutual exclusion in multicore environments. Developed using Verilog HDL, the system is synthesized with the Xilinx Vivado toolchain and operates on the Arty A7-100T and Kintex-7 325T , Genesys 2 FPGA board.

![Architecture Diagram](docs/architecture.jpg)

---

## **Specification**

Current features of the Aquila-mp SoC include:

- RV32IMA ISA-compliant.
- Embedded tightly-coupled on-chip memory (TCM).
- 16KB L1 instruction caches, 16KB data caches. (Configurable)
- 256KB L2 cache. (Configurable)
- Multi-core support with coherent data cache controller.
- The number of cores is configurable in the RTL design, allowing selection from 1 up to N cores.
- CLINT for standard timer interrupts.
- The RTL model written in Verilog.
- SD card I/O support.
- FreeRTOS SMP support

---

## **Performance**
The design has been verified using a Xilinx Kintex-7 XC7K325T FPGA device with up to 16 cores at 60MHz, where the parallel computing performance is up to 13 times faster than that of a single core configuration. The scalability of the proposed design is only limited by the logic capacity of the FPGA devices.

## **MESI FSM Diagram**  
The MESI protocol's FSM is visualized below:  

![MESI FSM Diagram](docs/MESI.jpg)

---

## **User's Guide**
For detailed instructions on how to set up and use the Aquila-mp SoC, please refer to the [User's Guide](docs/user_guide.md).

---

## **Acknowledgment**  
Aquila's source code is available on GitHub: [Aquila GitHub Repository](https://github.com/eisl-nctu/aquila)

The multi-core system is extended by contributions from the Embedded Intelligent Systems Laboratory (EISL) at National Yang Ming Chiao Tung University (NYCU). We acknowledge the support and collaboration from the open-source community and our academic partners.

## **Current Work**
Integration of a Memory Management Unit (MMU) to enable Linux OS support.

---

## **Folder and File Descriptions**

### **sw/**  
- **elibc/** – Basic C header library  
- **FreeRTOS/** – FreeRTOS SMP library for aquila-mp   
- **rtos_matrix/** – Matrix multiplication evaluation code for multi-core based on FreeRTOS
- **rtos_sorting/** – Array sorting evaluation code for multi-core based on FreeRTOS
- **rtos_ocr/** – MLP evaluation code for multi-core parallel execution based on FreeRTOS
- **rtos_cnn/** – Lenet-5 convolutional neural network evaluation code for multi-core parallel execution based on FreeRTOS
- **single_core_test/** – Contains the above four programs, but executed using the single-core FreeRTOS library and run on a single core.
- **uartboot/** – Contains UART boot code for core 0
- **uartboot_other_core/** – Contains UART boot code for other core

### **src/**  
- **core_rtl/** – Contains the RTL source code for the Aquila processor core, including modules such as the ALU, control unit, and register file.
- **mem/** – Contains the bootrom code that can be compiled in `sw/uartboot`and `sw/uartboot_other_core`
- **soc_rtl/** – Contains the top-level RTL code for the Aquila SoC  

### **tb_verilator/**
- Contains testbenches for Verilator simulation

### **tcl/build_arty100.tcl**
- Script to build the Aquila-mp SoC on the Arty A7-100T board

### **tcl/build_qmcore.tcl**
- Script to build the Aquila-mp SoC on the QMTech board

### **tcl/build_genesys2.tcl**
- Script to build the Aquila-mp SoC on the Genesys 2 board
