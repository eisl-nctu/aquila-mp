# Verilator Testing Guide

## 1. Set Project Target
Open `aquila_config.vh` at line 58 and select the platform you want to simulate:

- `define ARTY`  
- or `define QMCORE`  

> If not set, errors will occur.

---

## 2. Set the Simulation File Path
Open `aquila_config.vh`, set SIM_FNAME_0 to your ELF file path.

- `define SIM_FNAME_0 "/path/to/your/elf/file.elf"`

---

## 3. Testbench Parameter Settings
Edit `aquila_core_tb.cpp`:

### (1) Simulate UARTBoot or not
- **Skip UARTBoot**: set `ELF_LOAD = 1`
- **Simulate UARTBoot**: set `ELF_LOAD = 0`

### (2) Enable waveform trace
- Enable `#define TRACE` → generate waveform dump
- Comment out `#define TRACE` → no waveform (faster execution)

### (3) Select trace start condition
- **Start from PC**  
  - `TRACE_FROM_SPECIFIED_PC = 1`  
  - `START_LOG_PC = <PC address to start logging>`

- **Start from cycle**  
  - `TRACE_FROM_SPECIFIED_PC = 0`  
  - `TRACE_FROM_SPECIFIED_CYCLE = 1`  
  - `START_LOG_CYCLE = <cycle count to start logging>`

---

## 4. Usage Examples

### Case 1: Simulate UARTBoot and log waveform only after entering `main()` in RTOS
```cpp
#define ELF_LOAD 0
#define TRACE
#define TRACE_FROM_SPECIFIED_PC 1
#define START_LOG_PC 0x800011b8   // Example PC for main
```

### Case 2: Skip UARTBoot, run program directly, no waveform (fast execution)
```cpp
#define ELF_LOAD 1
// #define TRACE   // commented out
```

---

## 5. Build and Run
```bash
make
cd core_obj_dir/
./Vaquila_testharness [elf file path]
```

---

## 6. View Waveform
```bash
gtkwave aquila_core.fst
```

Inside **GTKWave**, load the following configuration for commonly used signals:
```
tb_verilator/test.gtkw
```

---