
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <map>
#include <cstdint>

#include "verilated_vcd_c.h"
#include "verilated_fst_c.h"
#include "Vaquila_testharness.h"
#include "Vaquila_testharness__Syms.h"
#include "verilated.h"

#include "sim_mem.h"
// #define TRACE //enable to generate vcd waveform file
/* #undef TRACE */
#define TRACE_FROM_SPECIFIED_PC 1
#define TRACE_FROM_SPECIFIED_CYCLE 0
#define FENCE_ENABLE
#undef FENCE_ENABLE
#define MAX_SIM_CYCLE 1000000000 //change here to simulate more cycle

#define clk_sec 1
#define UART_SEND_DELAY 10000 /* delay cycle of seding program to uart */
#define UART_LOAD 0 /* load program from uart */
#define ELF_LOAD 0
#define BIN_LOAD 0
#define UART_INBYTE_PC 0x00000280 /* the pc of inbyte() in uartboot */

#define START_LOG_PC  0x800014dc
#define START_LOG_CYCLE 0x0

using namespace std;
static vluint64_t cpuTime = 0;
uint32_t tohost_addr = 0;
Vaquila_testharness* top;
// VerilatedVcdC* Vcdfp;
VerilatedFstC* Vcdfp;
fstream log_file;
bool flush_flag;
uint32_t cur_cycle = 0;
bool start_log;
bool start_dump;
uint32_t mie_r = 0;

int send = 0;
int send_done = 0;

double sc_time_stamp() { return cpuTime; }
void load_simple_asm();

void log_cycle();



static void usage(const char * program_name)
{
    cout << "Usage: " << program_name << " [RISCV_TEST_ELF] [RVTEST(0/1),default 0]" << endl;
}

void log_cycle()
{
    bool dec_stall = top->aquila_testharness->aquila_core->RISCV_CORE0->stall_pipeline;
    bool dec_flush = top->aquila_testharness->aquila_core->RISCV_CORE0->plc2dec_flush || top->aquila_testharness->aquila_core->RISCV_CORE0->irq_taken;
    bool dec_load_hazard = top->aquila_testharness->aquila_core->RISCV_CORE0->dec2plc_load_hazard;
    bool unsupported_instr = top->aquila_testharness->aquila_core->RISCV_CORE0->dec_unsupported_instr; 
    if ((!dec_stall && !dec_flush && !dec_load_hazard) || unsupported_instr) {
        if (flush_flag) {
            flush_flag = false;
        } else {
            if (TRACE_FROM_SPECIFIED_PC && top->aquila_testharness->aquila_core->RISCV_CORE0->fet2dec_pc == START_LOG_PC) {
                printf("start logging from PC: 0x%08x\n", START_LOG_PC);
                start_dump = true;
                start_log = true;
            } else if (TRACE_FROM_SPECIFIED_CYCLE && cur_cycle >= START_LOG_CYCLE) {
                start_dump = true;
                start_log = true;
            } else {
                if (!start_log) return;
            }
            log_file << "#" << setfill('0') << setw(10) << right << cur_cycle << ":" 
                << setfill('0') << setw(8) << right << hex 
                << top->aquila_testharness->aquila_core->RISCV_CORE0->fet2dec_pc << endl;
        }
    } else {
        if (dec_flush && !dec_load_hazard) {
            flush_flag = true;
        }
    }
}

int main(int argc, char **argv)
{
    Verilated::commandArgs(argc,argv);
    map<string,uint64_t> elf_symbols;
    bool rv_test_enable = false;

    if (TRACE_FROM_SPECIFIED_PC) {
        start_log = false;
        start_dump = false;
    } else if (TRACE_FROM_SPECIFIED_CYCLE) {
        start_log = false;
        start_dump = false;
    } else {
        start_log = true;
        start_dump = true;
    }

    if (argc < 2) {
        // usage(argv[0]);
        // cerr << "Please provide riscv test elf file" << endl;
        // return -1;
        argv[1] = "";
    }

    log_file.open("cpu.log", fstream::out);

    if (!log_file.is_open()) {
        cerr << "Failed to open cpu.log file!!!" << endl;
        return -1;
    }

    if (argc >= 3) {
        if (argv[2][0] == '1')
            rv_test_enable = true;
        cout << "set rv_test_enable to " << (rv_test_enable ? "\"true\"" : "\"false\"") << endl;
    }

    top = new Vaquila_testharness("top");
#ifdef TRACE
    Verilated::traceEverOn(true);
    // Vcdfp = new VerilatedVcdC;
    Vcdfp = new VerilatedFstC;
    top->trace(Vcdfp, 99);
    // Vcdfp->open("aquila_core.vcd");
    Vcdfp->open("aquila_core.fst");
#endif
    uint32_t entry_addr = 0x00000000;
    std::string file_name = string(argv[1]);
    char *buf;
    size_t size;

    if (ELF_LOAD) {
        /* load elf to dram and execute it. */
        elf_symbols = sim_mem_load_program(top->aquila_testharness->mock_ram, file_name, &entry_addr);

        if (rv_test_enable) {
            if (elf_symbols.count("tohost")){
                tohost_addr = static_cast<uint32_t>(elf_symbols["tohost"]);
            } else {
                cerr << "no tohost symbols existed.!!!" << endl;
                return -1;
            }
        }
    } 
    top->rst_n = 0;
    cout << "entry_addr = " << "0x" << setfill('0') << setw(8) << right << hex << entry_addr << endl;
    top->main_memory_addr = entry_addr;
    //load_simple_asm();
    /* sim_mem_dump_memory(top->aquila_testharness->mock_ram, "dump.mem"); */
    for (int i = 0 ; i < 5 ; i ++){
        top->clk = 0;
        top->eval ();
        cpuTime += clk_sec;
#ifdef TRACE
        Vcdfp->dump(cpuTime);
#endif
        top->clk = 1;
        top->eval ();
        cpuTime += clk_sec;
#ifdef TRACE
        Vcdfp->dump(cpuTime);
#endif
    }
    top->rst_n = 1;

    uint32_t tohost_val;
    bool wait_inbyte = 0;
    flush_flag = false;
    bool setenv_done = false;
    std::string bootcmd;

    while (true) {
        top->clk = 0;
        top->eval();
        cpuTime += clk_sec;
#ifdef TRACE
        if (start_dump) {
            Vcdfp->dump(cpuTime);
        }
#endif
        log_cycle();
        top->clk = 1;
        top->eval();
        cpuTime += clk_sec;
        cur_cycle++;
#ifdef TRACE
        if (start_dump) {
            Vcdfp->dump(cpuTime);
        }
#endif
        if (rv_test_enable) {
#ifdef FENCE_ENABLE
            tohost_val = sim_mem_tohost_monitor(top->aquila_testharness->mock_ram, tohost_addr);
#else
            // tohost_val = top->aquila_testharness->mock_uart_0->read_tohost();
#endif
            if (tohost_val != 0){
                if (tohost_val == 1)
                    cout << "pass testcase: " << argv[1] << endl;
                else
                    cout << "testcase #" << tohost_val << " failed !!!!!\ntestcase:" << argv[1] << endl;
                break;
            }
        }
        if(top->aquila_testharness->uart_simulation_done == 1)break;
    }
#ifdef TRACE
    Vcdfp->close();
    delete Vcdfp;
#endif
    delete top;

    if (rv_test_enable && tohost_val != 1)
        exit(-1);
    else
        exit(0);
}

void load_simple_asm()
{
    //_start
    top->aquila_testharness->mock_ram->writeWord(0x00000000,0x04c0006f);
    // trap_vector
    top->aquila_testharness->mock_ram->writeWord(0x00000004,0x34202f73);
    top->aquila_testharness->mock_ram->writeWord(0x00000008,0x00800f93);
    top->aquila_testharness->mock_ram->writeWord(0x0000000c,0x03ff0a63);
    top->aquila_testharness->mock_ram->writeWord(0x00000010,0x00900f93);
    top->aquila_testharness->mock_ram->writeWord(0x00000014,0x03ff0663);
    top->aquila_testharness->mock_ram->writeWord(0x00000018,0x00b00f93);
    top->aquila_testharness->mock_ram->writeWord(0x0000001c,0x03ff0263);
    top->aquila_testharness->mock_ram->writeWord(0x00000020,0x00000f17);
    top->aquila_testharness->mock_ram->writeWord(0x00000024,0xfe0f0f13);
    top->aquila_testharness->mock_ram->writeWord(0x00000028,0x000f0463);
    top->aquila_testharness->mock_ram->writeWord(0x0000002c,0x000f0067);
    top->aquila_testharness->mock_ram->writeWord(0x00000030,0x34202f73);
    top->aquila_testharness->mock_ram->writeWord(0x00000034,0x000f5463);
    top->aquila_testharness->mock_ram->writeWord(0x00000038,0x0040006f);
    // handle exception
    top->aquila_testharness->mock_ram->writeWord(0x0000003c,0x5391e193);
    top->aquila_testharness->mock_ram->writeWord(0x00000040,0x00001f17);
    top->aquila_testharness->mock_ram->writeWord(0x00000044,0xfc3f2023);
    top->aquila_testharness->mock_ram->writeWord(0x00000048,0xff9ff06f);
    //reset vector
    top->aquila_testharness->mock_ram->writeWord(0x0000004c,0xf1402573);
    top->aquila_testharness->mock_ram->writeWord(0x00000050,0x00051063);
    top->aquila_testharness->mock_ram->writeWord(0x00000054,0x00000297);
    top->aquila_testharness->mock_ram->writeWord(0x00000058,0x01028293);
    top->aquila_testharness->mock_ram->writeWord(0x0000005c,0x30529073);
    top->aquila_testharness->mock_ram->writeWord(0x00000060,0x18005073);
    top->aquila_testharness->mock_ram->writeWord(0x00000064,0x00000297);
    top->aquila_testharness->mock_ram->writeWord(0x00000068,0x02028293);
    top->aquila_testharness->mock_ram->writeWord(0x0000006c,0x30529073);
    top->aquila_testharness->mock_ram->writeWord(0x00000070,0x800002b7);
}