
`timescale 1 ns / 1 ps
// =============================================================================
//  Program : aquila.v
//  Author  : Hon-Chou Dai (Daichou)
//  Date    : FEB/12/2020
// -----------------------------------------------------------------------------
//  Revision information:
//
//  NONE.
// -----------------------------------------------------------------------------
//  Description:
//  This is the top-level Aquila SoC testharness module.
// -----------------------------------------------------------------------------
//  License information:
//
//  This software is released under the BSD-3-Clause Licence,
//  see https://opensource.org/licenses/BSD-3-Clause for details.
//  In the following license statements, "software" refers to the
//  "source code" of the complete hardware/software system.
//
//  Copyright 2019,
//                    Embedded Intelligent Systems Lab (EISL)
//                    Deparment of Computer Science
//                    National Chiao Tung Uniersity
//                    Hsinchu, Taiwan.
//
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions are met:
//
//  1. Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//  2. Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//  3. Neither the name of the copyright holder nor the names of its contributors
//     may be used to endorse or promote products derived from this software
//     without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
//  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.
// =============================================================================
`include "aquila_config.vh"
module aquila_testharness #(
    // Parameters of Axi Master Bus Interface M_ICACHE_PORT
    parameter integer C_M_IMEM_PORT_ID_WIDTH = 1,
    parameter integer C_M_IMEM_PORT_ADDR_WIDTH = 32,
    parameter integer C_M_IMEM_PORT_DATA_WIDTH = 32,
    parameter integer C_M_IMEM_PORT_AWUSER_WIDTH = 0,
    parameter integer C_M_IMEM_PORT_ARUSER_WIDTH = 0,
    parameter integer C_M_IMEM_PORT_WUSER_WIDTH = 0,
    parameter integer C_M_IMEM_PORT_RUSER_WIDTH = 0,
    parameter integer C_M_IMEM_PORT_BUSER_WIDTH = 0,

    // Parameters of Axi Master Bus Interface M_DMEM_PORT
    parameter integer C_M_DMEM_PORT_ID_WIDTH = 1,
    parameter integer C_M_DMEM_PORT_ADDR_WIDTH = 32,
    parameter integer C_M_DMEM_PORT_DATA_WIDTH = 32,
    parameter integer C_M_DMEM_PORT_AWUSER_WIDTH = 0,
    parameter integer C_M_DMEM_PORT_ARUSER_WIDTH = 0,
    parameter integer C_M_DMEM_PORT_WUSER_WIDTH = 0,
    parameter integer C_M_DMEM_PORT_RUSER_WIDTH = 0,
    parameter integer C_M_DMEM_PORT_BUSER_WIDTH = 0,

    // Parameters of Axi Master Bus Interface M_DEVICE_PORT
    parameter integer C_M_DEVICE_PORT_ADDR_WIDTH = 32,
    parameter integer C_M_DEVICE_PORT_DATA_WIDTH = 32
) (
    input logic clk,
    input logic rst_n,
    input logic [31 : 0] main_memory_addr,
    output logic [31:0] cur_instr_addr
);

    // Declaration of local signals.
    wire RISCV_rst;
    wire intc_irq;
   localparam CORE_NUMS_BITS = (`CORE_NUMS==1) ? 0 : $clog2(`CORE_NUMS);
   localparam CLSIZE = `CLP;
   localparam XLEN = 32;
    // --------- coherence unit ----------------------------------------------------
    // to aquila top
    wire                CU_L1_probe_strobe[0 : `CORE_NUMS-1];
    wire [XLEN-1 : 0]   CU_L1_probe_addr[0 : `CORE_NUMS-1];
    wire                CU_L1_invalidate[0 : `CORE_NUMS-1];
    wire [CLSIZE-1 : 0] CU_L1_data[0 : `CORE_NUMS-1];
    wire                CU_L1_make_exclusive[0 : `CORE_NUMS-1];
    wire                CU_L1_response_ready[0 : `CORE_NUMS-1];
    wire                CU_L1_other_amo[0 : `CORE_NUMS-1];
    wire [CLSIZE-1 : 0]  L1_CU_wb_data[0 : `CORE_NUMS-1];
    wire [CLSIZE-1 : 0]  L1_CU_response_data[0 : `CORE_NUMS-1];
    wire [XLEN-1 : 0]    L1_CU_addr[0 : `CORE_NUMS-1];
    wire                 L1_CU_strobe[0 : `CORE_NUMS-1];
    wire                 L1_CU_rw[0 : `CORE_NUMS-1];
    wire                 L1_CU_share_modify[0 : `CORE_NUMS-1];
    wire                 L1_CU_response_ready[0 : `CORE_NUMS-1];
    wire                 L1_CU_replacement[0 : `CORE_NUMS-1];
    wire                 L1_CU_is_instr_fetch[0 : `CORE_NUMS-1];
    // to L2 cache
    //write back to L2
    wire                CU_L2_wb;
    wire [CLSIZE-1 : 0] CU_L2_wb_data;
    wire                CU_L2_replacement;
    //request L2 data 
    wire                CU_L2_probe_strobe;
    wire [XLEN-1 : 0]   CU_L2_addr;
    wire                CU_L2_rw;
    reg                 L2_CU_ready;
    reg [CLSIZE-1:0]    L2_CU_data;
    reg                 L2_CU_make_exclusive;
    wire                CU_L2_is_instr_fetch;
    wire                CU_L2_invalidate;
    wire                CU_L2_ready;
    wire                L2_invalidate_L1;
    wire [XLEN-1:0]     L2_invalidate_L1_addr;

    // L2 cache signals
    wire                MEM_L2_ready;
    wire [CLSIZE-1:0]   MEM_L2_data;
    wire                L2_MEM_strobe;
    wire                L2_MEM_rw;
    wire [XLEN-1:0]     L2_MEM_addr;
    wire [CLSIZE-1:0]   L2_MEM_data;
    // --------- Amo unit signals -----------------------------------------
    wire                    C2AMO_strobe[0 : `CORE_NUMS-1];
    wire                    C2AMO_rw[0 : `CORE_NUMS-1];
    wire [XLEN-1 : 0]       C2AMO_addr[0 : `CORE_NUMS-1];
    wire [XLEN-1 : 0]       C2AMO_wt_data[0 : `CORE_NUMS-1];
    wire                    C2AMO_done[0 : `CORE_NUMS-1];
    wire [XLEN-1 : 0]       C2AMO_rd_data[0 : `CORE_NUMS-1];
    wire                    C2AMO_is_amo[0 : `CORE_NUMS-1];
    wire [4 : 0]            C2AMO_amo_type[0 : `CORE_NUMS-1];

    wire                    AMO2C_strobe[0 : `CORE_NUMS-1];
    wire                    AMO2C_rw[0 : `CORE_NUMS-1];
    wire [XLEN-1 : 0]       AMO2C_addr[0 : `CORE_NUMS-1];
    wire [XLEN-1 : 0]       AMO2C_wt_data[0 : `CORE_NUMS-1];
    wire                    AMO2C_done[0 : `CORE_NUMS-1];
    wire [XLEN-1 : 0]       AMO2C_rd_data[0 : `CORE_NUMS-1];
    wire [CORE_NUMS_BITS-1 : 0]            AMO_id;
    wire                    AMO_strobe;
    wire                    AMO_rw;
    wire [XLEN-1 : 0]       AMO_addr;
    wire [XLEN-1 : 0]       AMO_wt_data;
    wire                    AMO_data_done;
    wire [XLEN-1 : 0]       AMO_rd_data;
    wire                    AMO_is_amo;
    wire [4 : 0]            AMO_amo_type;

    wire                    AMO_DMEM_strobe;
    wire                    AMO_DMEM_rw;
    wire [XLEN-1 : 0]       AMO_DMEM_addr;
    wire [XLEN-1 : 0]       AMO_DMEM_wt_data;
    wire                    AMO_DMEM_done;
    wire [XLEN-1 : 0]       AMO_DMEM_rd_data;
    // --------- CLINT  interface ----------------------------------------------
    wire                clint_strobe[0 : `CORE_NUMS-1];
    wire [XLEN-1 : 0]   clint_addr[0 : `CORE_NUMS-1];
    wire                clint_we[0 : `CORE_NUMS-1];
    wire [XLEN/8-1 : 0] clint_be[0 : `CORE_NUMS-1];
    wire [XLEN-1 : 0]   clint_din[0 : `CORE_NUMS-1];
    wire [XLEN-1 : 0]   clint_dout[0 : `CORE_NUMS-1];
    wire                clint_ready[0 : `CORE_NUMS-1];
    // Selected clint bus signals
    wire                M_clint_strobe;
    wire [XLEN-1 : 0]   M_clint_addr;
    wire                M_clint_we;
    wire [XLEN-1 : 0]   M_clint_din;
    wire [XLEN-1 : 0]   M_clint_dout;
    wire                M_clint_ready;
    wire                tmr_irq[0 : `CORE_NUMS-1];
    wire                sft_irq[0 : `CORE_NUMS-1];
    // --------- I/O device interface ----------------------------------------------
    //  Device bus signals for core 0 and core 1 
    wire                dev_strobe[0 : `CORE_NUMS-1];
    wire [XLEN-1 : 0]   dev_addr[0 : `CORE_NUMS-1];
    wire                dev_we[0 : `CORE_NUMS-1];
    wire [XLEN/8-1 : 0] dev_be[0 : `CORE_NUMS-1];
    wire [XLEN-1 : 0]   dev_din[0 : `CORE_NUMS-1];
    wire [XLEN-1 : 0]   dev_dout[0 : `CORE_NUMS-1];
    wire                dev_ready[0 : `CORE_NUMS-1];

    // Selected device bus signals
    wire                M_dev_strobe;
    wire [XLEN-1 : 0]   M_dev_addr;
    wire                M_dev_we;
    wire [XLEN/8-1 : 0] M_dev_be;
    wire [XLEN-1 : 0]   M_dev_din;
    wire [XLEN-1 : 0]   M_dev_dout;
    wire                M_dev_ready;
    // Declaration of local signals.
    wire                                      M_IMEM_strobe, M_IMEM_done;
    wire                                      M_DMEM_strobe, M_DMEM_done;
    wire                                      M_DMEM_rw;
    wire [XLEN-1 : 0]     M_IMEM_addr;
    wire [XLEN-1 : 0]     M_DMEM_addr;
    wire [CLSIZE-1 : 0]                   M_IMEM_datain, M_DMEM_datain, M_DMEM_dataout;
    wire [7 : 0]                              M_IMEM_burst_len = `CLP/32; // in XLEN-bit words
    wire [7 : 0]                              M_DMEM_burst_len = `CLP/32; // in XLEN-bit words

    wire                                      M_DEVICE_strobe;
    wire [XLEN-1 : 0]   M_DEVICE_addr;
    wire                                      M_DEVICE_rw;
    wire [XLEN/8-1 : 0] M_DEVICE_byte_enable;
    wire [XLEN-1 : 0]   M_DEVICE_core2dev_data;
    wire                                      M_DEVICE_data_ready;
    wire [XLEN-1 : 0]   M_DEVICE_dev2core_data;

    assign M_DEVICE_strobe = M_dev_strobe;
    assign M_DEVICE_addr = M_dev_addr;
    assign M_DEVICE_rw = M_dev_we;
    assign M_DEVICE_byte_enable = M_dev_be;
    assign M_DEVICE_core2dev_data = M_dev_din;
    assign M_dev_dout = M_DEVICE_dev2core_data;
    assign M_dev_ready = M_DEVICE_data_ready;

    assign M_DMEM_strobe = L2_MEM_strobe;
    assign M_DMEM_addr = L2_MEM_addr;
    assign M_DMEM_rw = L2_MEM_rw;
    assign M_DMEM_dataout = L2_MEM_data;
    assign MEM_L2_data = M_DMEM_datain;
    assign MEM_L2_ready = M_DMEM_done;

    assign M_IMEM_strobe = 0;
    assign M_IMEM_addr = 0;

    wire uart_simulation_done /*verilator public*/;
    // Debug pc
    wire [                              31:0] debug_pc  /*verilator public_flat*/;
    // assign debug_pc = aquila_core.p_i_addr;

    //wire [31:0] debug_rf [0:31]/*verilator public_flat*/;
    //genvar idx;
    //generate
    //for (idx = 0 ; idx < 32 ; idx = idx + 1)
    //  assign debug_rf[i] = aquila_core.RISCV_CORE0.Register_File.rf[i];
    //endgenerate

    wire is_dev_req /* verilator public */;
    wire dev_sel /* verilator public */;

    assign is_dev_req = (M_DEVICE_addr[31:30] == 2'b11);
    assign dev_sel    = (M_DEVICE_addr[27:24] == 4'h0) ? 0 :
                        (M_DEVICE_addr[27:24] == 4'h4) ? 1 : 0;

    wire uart_data_ready;
    wire uart_strobe;
    wire uart_rw;
    wire [C_M_DEVICE_PORT_DATA_WIDTH - 1 : 0] uart_data;
    wire intc_data_ready;
    wire intc_strobe;
    wire intc_rw;
    wire [C_M_DEVICE_PORT_DATA_WIDTH - 1 : 0] intc_data;
    assign M_DEVICE_data_ready = (is_dev_req)? ((dev_sel == 0)? uart_data_ready : intc_data_ready) : 0;
    assign M_DEVICE_dev2core_data = (is_dev_req)? ((dev_sel == 0)? uart_data : intc_data) : 0;
    assign uart_strobe = (is_dev_req && (dev_sel == 0))? M_DEVICE_strobe : 0;
    assign uart_rw = (is_dev_req && (dev_sel == 0))? M_DEVICE_rw : 0;
    assign intc_strobe = (is_dev_req && (dev_sel == 1))? M_DEVICE_strobe : 0;
    assign intc_rw = (is_dev_req && (dev_sel == 1))? M_DEVICE_rw : 0;
    // Instiantiation of the top-level Aquila core module.
    // -----------------------------------------------------------------------------
    //  Aquila processor core.
    //
    // Core 0 is instantiated separately for Verilator to monitor the program counter (PC).
    aquila_top #(.HART_ID(0), .XLEN(XLEN), .CLSIZE(CLSIZE)) 
            aquila_core
            (
                //system signals
                .clk_i(clk),
                .rst_i(~rst_n),

                // Initial program counter address for the Aquila core
                .base_addr_i(main_memory_addr),
                // .base_addr_i(32'b0),
                // Aquila external I/D memory interface signals
                // coherence unit signals
                .CU_L1_probe_strobe_i(CU_L1_probe_strobe[0]),
                .CU_L1_probe_addr_i(CU_L1_probe_addr[0]),
                .CU_L1_invalidate_i(CU_L1_invalidate[0]),
                .CU_L1_data_i(CU_L1_data[0]),
                .CU_L1_make_exclusive_i(CU_L1_make_exclusive[0]),
                .CU_L1_response_ready_i(CU_L1_response_ready[0]),
                .CU_L1_is_amo_i(CU_L1_other_amo[0]),

                .L1_CU_response_data_o(L1_CU_response_data[0]),
                .L1_CU_wb_data_o(L1_CU_wb_data[0]),
                .L1_CU_addr_o(L1_CU_addr[0]),
                .L1_CU_strobe_o(L1_CU_strobe[0]),
                .L1_CU_rw_o(L1_CU_rw[0]),
                .L1_CU_share_modify_o(L1_CU_share_modify[0]),
                .L1_CU_response_ready_o(L1_CU_response_ready[0]),
                .L1_CU_replacement_o(L1_CU_replacement[0]),
                .L1_CU_is_instr_fetch_o(L1_CU_is_instr_fetch[0]),
                
                // Atomic instructions ports 
                .C2AMO_strobe_o(C2AMO_strobe[0]),
                .C2AMO_rw_o(C2AMO_rw[0]),
                .C2AMO_addr_o(C2AMO_addr[0]),
                .C2AMO_data_o(C2AMO_wt_data[0]),
                .C2AMO_data_ready_i(C2AMO_done[0]),
                .C2AMO_data_i(C2AMO_rd_data[0]),
                .C2AMO_is_amo_o(C2AMO_is_amo[0]),
                .C2AMO_amo_type_o(C2AMO_amo_type[0]),

                .AMO2C_strobe_i(AMO2C_strobe[0]),
                .AMO2C_rw_i(AMO2C_rw[0]),
                .AMO2C_addr_i(AMO2C_addr[0]),
                .AMO2C_data_i(AMO2C_wt_data[0]),
                .AMO2C_data_ready_o(AMO2C_done[0]),
                .AMO2C_data_o(AMO2C_rd_data[0]),
                // Aquila M_DEVICE master port interface signals
                // I/O device ports.
                .M_DEVICE_strobe_o(dev_strobe[0]),
                .M_DEVICE_addr_o(dev_addr[0]),
                .M_DEVICE_rw_o(dev_we[0]),
                .M_DEVICE_byte_enable_o(dev_be[0]),
                .M_DEVICE_data_o(dev_din[0]),
                .M_DEVICE_data_ready_i(dev_ready[0]),
                .M_DEVICE_data_i(dev_dout[0]),
                // Aquila M_CLINT master port interface signals
                .M_CLINT_strobe_o(clint_strobe[0]),
                .M_CLINT_addr_o(clint_addr[0]),
                .M_CLINT_rw_o(clint_we[0]),
                .M_CLINT_data_o(clint_din[0]),
                .M_CLINT_data_ready_i(clint_ready[0]),
                .M_CLINT_data_i(clint_dout[0]),

                .tmr_irq_i(tmr_irq[0]),
                .sft_irq_i(sft_irq[0]),
                .ext_irq_i(0)
            );
    genvar i;
    generate
        for (i = 1; i < `CORE_NUMS; i = i + 1)
        begin
            aquila_top #(.HART_ID(i), .XLEN(XLEN), .CLSIZE(CLSIZE)) 
            aquila_top
            (
                //system signals
                .clk_i(clk),
                .rst_i(~rst_n),

                // Initial program counter address for the Aquila core
                .base_addr_i(main_memory_addr),
                // .base_addr_i(32'b0),
                // Aquila external I/D memory interface signals
                // coherence unit signals
                .CU_L1_probe_strobe_i(CU_L1_probe_strobe[i]),
                .CU_L1_probe_addr_i(CU_L1_probe_addr[i]),
                .CU_L1_invalidate_i(CU_L1_invalidate[i]),
                .CU_L1_data_i(CU_L1_data[i]),
                .CU_L1_make_exclusive_i(CU_L1_make_exclusive[i]),
                .CU_L1_response_ready_i(CU_L1_response_ready[i]),
                .CU_L1_is_amo_i(CU_L1_other_amo[i]),

                .L1_CU_response_data_o(L1_CU_response_data[i]),
                .L1_CU_wb_data_o(L1_CU_wb_data[i]),
                .L1_CU_addr_o(L1_CU_addr[i]),
                .L1_CU_strobe_o(L1_CU_strobe[i]),
                .L1_CU_rw_o(L1_CU_rw[i]),
                .L1_CU_share_modify_o(L1_CU_share_modify[i]),
                .L1_CU_response_ready_o(L1_CU_response_ready[i]),
                .L1_CU_replacement_o(L1_CU_replacement[i]),
                .L1_CU_is_instr_fetch_o(L1_CU_is_instr_fetch[i]),
                
                // Atomic instructions ports 
                .C2AMO_strobe_o(C2AMO_strobe[i]),
                .C2AMO_rw_o(C2AMO_rw[i]),
                .C2AMO_addr_o(C2AMO_addr[i]),
                .C2AMO_data_o(C2AMO_wt_data[i]),
                .C2AMO_data_ready_i(C2AMO_done[i]),
                .C2AMO_data_i(C2AMO_rd_data[i]),
                .C2AMO_is_amo_o(C2AMO_is_amo[i]),
                .C2AMO_amo_type_o(C2AMO_amo_type[i]),

                .AMO2C_strobe_i(AMO2C_strobe[i]),
                .AMO2C_rw_i(AMO2C_rw[i]),
                .AMO2C_addr_i(AMO2C_addr[i]),
                .AMO2C_data_i(AMO2C_wt_data[i]),
                .AMO2C_data_ready_o(AMO2C_done[i]),
                .AMO2C_data_o(AMO2C_rd_data[i]),
                // Aquila M_DEVICE master port interface signals
                // I/O device ports.
                .M_DEVICE_strobe_o(dev_strobe[i]),
                .M_DEVICE_addr_o(dev_addr[i]),
                .M_DEVICE_rw_o(dev_we[i]),
                .M_DEVICE_byte_enable_o(dev_be[i]),
                .M_DEVICE_data_o(dev_din[i]),
                .M_DEVICE_data_ready_i(dev_ready[i]),
                .M_DEVICE_data_i(dev_dout[i]),
                // Aquila M_CLINT master port interface signals
                .M_CLINT_strobe_o(clint_strobe[i]),
                .M_CLINT_addr_o(clint_addr[i]),
                .M_CLINT_rw_o(clint_we[i]),
                .M_CLINT_data_o(clint_din[i]),
                .M_CLINT_data_ready_i(clint_ready[i]),
                .M_CLINT_data_i(clint_dout[i]),
                .tmr_irq_i(tmr_irq[i]),
                .sft_irq_i(sft_irq[i]),
                .ext_irq_i('b0)
            );
        end
    endgenerate 
   
// -----------------------------------------------------------------------------
//  Atomic unit arbiter.
//     
amo_arbiter #(.XLEN(XLEN), .CLSIZE(CLSIZE), .CORE_NUMS(`CORE_NUMS), .CORE_NUMS_BITS(CORE_NUMS_BITS))
Amo_arbiter
(   //===================== System signals =====================//
    .clk_i(clk),
    .rst_i(~rst_n),

    // Aquila core  device interface
    .P_strobe_i(C2AMO_strobe),
    .P_addr_i(C2AMO_addr),
    .P_rw_i(C2AMO_rw),
    .P_data_i(C2AMO_wt_data),
    .P_data_ready_o(C2AMO_done),
    .P_data_o(C2AMO_rd_data),
    .P_is_amo_i(C2AMO_is_amo),
    .P_amo_type_i(C2AMO_amo_type),

    // Aquila device slave interface
    .AMO_id_o(AMO_id),
    .AMO_strobe_o(AMO_strobe),
    .AMO_addr_o(AMO_addr),
    .AMO_rw_o(AMO_rw),
    .AMO_data_o(AMO_wt_data),
    .AMO_data_ready_i(AMO_data_done),
    .AMO_data_i(AMO_rd_data),
    .AMO_is_amo_o(AMO_is_amo),
    .AMO_amo_type_o(AMO_amo_type)
);

// ----------------------------------------------------------------------------
//  The Atomic Unit (Overseer of RISCV atomic instructions).
//
// processor to atomic unit

atomic_unit #(.CORE_NUMS(`CORE_NUMS), .CORE_NUMS_BITS(CORE_NUMS_BITS), .XLEN(XLEN), .CLSIZE(CLSIZE))
ATOM_U(
    .clk_i(clk),
    .rst_i(~rst_n),

    .core_id_i(AMO_id), // number of RISCV cores (# of core_top modules)
    .core_strobe_i(AMO_strobe),
    .core_rw_i(AMO_rw),
    .core_addr_i(AMO_addr),
    .core_data_i(AMO_wt_data),
    .core_done_o(AMO_data_done),
    .core_data_o(AMO_rd_data),

    .core_is_amo_i(AMO_is_amo),
    .core_amo_type_i(AMO_amo_type),

    .M_DMEM_strobe_o(AMO_DMEM_strobe),
    .M_DMEM_rw_o(AMO_DMEM_rw),
    .M_DMEM_addr_o(AMO_DMEM_addr),
    .M_DMEM_data_o(AMO_DMEM_wt_data),
    .M_DMEM_done_i(AMO_DMEM_done),
    .M_DMEM_data_i(AMO_DMEM_rd_data)
);

generate
    for(i = 0; i < `CORE_NUMS; i = i + 1) begin
        assign AMO2C_strobe[i]   = (AMO_id == i) ? AMO_DMEM_strobe : 'b0;
        assign AMO2C_rw[i]       = (AMO_id == i) ? AMO_DMEM_rw : 'b0;
        assign AMO2C_addr[i]     = (AMO_id == i) ? AMO_DMEM_addr : 'b0;
        assign AMO2C_wt_data[i]  = (AMO_id == i) ? AMO_DMEM_wt_data : 'b0;
    end
endgenerate
assign AMO_DMEM_done = AMO2C_done[AMO_id];
assign AMO_DMEM_rd_data = AMO2C_rd_data[AMO_id];
// -----------------------------------------------------------------------------
//  Clint Arbiter.
bus_arbiter #(.XLEN(XLEN), .CORE_NUMS(`CORE_NUMS), .CORE_NUMS_BITS(CORE_NUMS_BITS))
clint_arbiter(
    .clk_i(clk),
    .rst_i(~rst_n), 

    .P_strobe_i(clint_strobe),
    .P_addr_i(clint_addr),
    .P_rw_i(clint_we),
    .P_data_i(clint_din),
    .P_data_ready_o(clint_ready),
    .P_data_o(clint_dout),

    .ARBITER_strobe_o(M_clint_strobe),
    .ARBITER_addr_o(M_clint_addr),
    .ARBITER_rw_o(M_clint_we),
    .ARBITER_data_o(M_clint_din),
    .ARBITER_data_ready_i(M_clint_ready),
    .ARBITER_data_i(M_clint_dout)
);
//  Instiantiation of the Core Local Interrupt controller (CLINT) module.
//
clint CLINT(
    .clk_i(clk),
    .rst_i(~rst_n),
    .en_i(M_clint_strobe),
    .we_i(M_clint_we),
    .addr_i(M_clint_addr),
    .data_i(M_clint_din),
    .data_o(M_clint_dout),
    .data_ready_o(M_clint_ready),

    .tmr_irq_o(tmr_irq),
    .sft_irq_o(sft_irq)
);
// -----------------------------------------------------------------------------
//  Device Arbiter.
bus_arbiter #(.XLEN(XLEN), .CORE_NUMS(`CORE_NUMS), .CORE_NUMS_BITS(CORE_NUMS_BITS))
device_arbiter(
    .clk_i(clk),
    .rst_i(~rst_n), 

    .P_strobe_i(dev_strobe),
    .P_addr_i(dev_addr),
    .P_rw_i(dev_we),
    .P_byte_enable_i(dev_be),
    .P_data_i(dev_din),
    .P_data_ready_o(dev_ready),
    .P_data_o(dev_dout),

    .ARBITER_strobe_o(M_dev_strobe),
    .ARBITER_addr_o(M_dev_addr),
    .ARBITER_rw_o(M_dev_we),
    .ARBITER_byte_enable_o(M_dev_be),
    .ARBITER_data_o(M_dev_din),
    .ARBITER_data_ready_i(M_dev_ready),
    .ARBITER_data_i(M_dev_dout)
);

// ----------------------------------
//  coherence unit
// ----------------------------------
 coherence_unit #(.XLEN(32), .CLSIZE(CLSIZE), .CORE_NUMS(`CORE_NUMS), .CORE_NUMS_BITS(CORE_NUMS_BITS)) 
    coherence_unit
    (
        //===================== System signals =====================//
        .clk_i(clk),
        .rst_i(~rst_n),
        //===================== to aquila top =====================//
        // request data from L1 cache
        .broadcast_strobe(L1_CU_strobe),
        .read_write(L1_CU_rw),
        .data_address(L1_CU_addr),
        .share_modify(L1_CU_share_modify),
        .Data(CU_L1_data),
        .request_data_ready(CU_L1_response_ready),
        .replacement(L1_CU_replacement),
        .L1_is_instr_fetch(L1_CU_is_instr_fetch),

        // response data to L1 cache
        .probe_strobe(CU_L1_probe_strobe),
        .probe_data_address(CU_L1_probe_addr),
        .invalidate(CU_L1_invalidate),
        .make_exclusive_L1(CU_L1_make_exclusive),
        .response_data(L1_CU_response_data),
        .write_back_data(L1_CU_wb_data),
        .response_data_ready(L1_CU_response_ready),
        //===================== L2 cache =====================//
        //write back to L2
        .write_to_L2(CU_L2_wb),
        .data_to_L2(CU_L2_wb_data),
        .replacement_L2(CU_L2_replacement),
        //request L2 data
        .probe_strobe_L2(CU_L2_probe_strobe),
        .address_to_L2(CU_L2_addr),
        .L1_iswrite_L2(CU_L2_rw),
        .response_data_L2(L2_CU_data),
        .response_ready_L2(L2_CU_ready),
        .make_L1_exclusive_L2(L2_CU_make_exclusive),
        .L1_is_instr_fetch_L2(CU_L2_is_instr_fetch),
        // Invalidate L2 data
        .invalidate_L2(CU_L2_invalidate),
        //L2 invalidate L1 signal
        .CU_L2_ready_o(CU_L2_ready),
        .L2_invalidate_L1_addr_i(L2_invalidate_L1_addr),
        .L2_invalidate_L1_i(L2_invalidate_L1),
        //atomic
        .P_amo_strobe(AMO_strobe),
        .AMO_id(AMO_id),
        .other_amo_done(C2AMO_done),
        .CU_L1_other_amo_o(CU_L1_other_amo)
    );


// ----------------------------------
//  L2 cache
// ----------------------------------
L2cache #(.XLEN(XLEN), .CLSIZE(CLSIZE), .CACHE_SIZE(`L2CACHE_SIZE))
    L2cache
    (
        //system signals
        .clk_i(clk),
        .rst_i(~rst_n),
        //coherence unit signals
        .wb_i(CU_L2_wb),
        .wb_replacement_i(CU_L2_replacement),
        .wb_data_i(CU_L2_wb_data),
        .probe_strobe_i(CU_L2_probe_strobe),
        .probe_rw_i(CU_L2_rw),
        .CU_L2_addr_i(CU_L2_addr),
        .response_data_o(L2_CU_data),
        .response_ready_o(L2_CU_ready),
        .make_exclusive_o(L2_CU_make_exclusive),
        .invalidate_i(CU_L2_invalidate),
        .L1_is_instr_fetch_i(CU_L2_is_instr_fetch),
        .CU_ready_i(CU_L2_ready),
        .invalidate_L1_addr_o(L2_invalidate_L1_addr),
        .invalidate_L1_o(L2_invalidate_L1),
        // memory controller signals
        .m_ready_i(MEM_L2_ready),
        .m_data_i(MEM_L2_data),
        .m_strobe_o(L2_MEM_strobe),
        .m_rw_o(L2_MEM_rw),
        .m_addr_o(L2_MEM_addr),
        .m_data_o(L2_MEM_data)
    );

    dp_ram  mock_ram (
        .clk(clk),
        .rst_n(rst_n),
        .strobe_imem(M_IMEM_strobe),
        .addr_imem_i(M_IMEM_addr),
        .rdata_imem_o(M_IMEM_datain),
        .done_imem_o(M_IMEM_done),

        .strobe_dmem(M_DMEM_strobe),
        .addr_dmem_i(M_DMEM_addr),
        .wdata_dmem_i(M_DMEM_dataout),
        .rdata_dmem_o(M_DMEM_datain),
        .rw_dmem_i(M_DMEM_rw),
        .done_dmem_o(M_DMEM_done)
    );

    uart UART(
        .clk(clk),
        .rst(~rst_n),

        .EN(uart_strobe),
        .ADDR(M_DEVICE_addr[3:2]),
        .WR(uart_rw),
        .BE(M_DEVICE_byte_enable),
        .DATAI(M_DEVICE_core2dev_data),
        .DATAO(uart_data),
        .READY(uart_data_ready),

        .RXD(uart_rx),
        .TXD(uart_tx),

        .simulation_done(uart_simulation_done),
        .uart_core_sel(0)
    );

    // mock_uart mock_uart_0 (
    //     .clk  (clk),
    //     .rst_n(rst_n),

    //     .M_DEVICE_strobe       (uart_strobe),
    //     .M_DEVICE_addr         (M_DEVICE_addr),
    //     .M_DEVICE_rw           (uart_rw),
    //     .M_DEVICE_byte_enable  (M_DEVICE_byte_enable),
    //     .M_DEVICE_core2dev_data(M_DEVICE_core2dev_data),
    //     .M_DEVICE_data_ready   (uart_data_ready),
    //     .M_DEVICE_dev2core_data(uart_data),
    //     .intr_o                (uart_intr)

    // );

    intc intc_0 (
        .clk                   (clk),
        .rst_n                 (rst_n),
        .M_DEVICE_strobe       (intc_strobe),
        .M_DEVICE_addr         (M_DEVICE_addr),
        .M_DEVICE_rw           (intc_rw),
        .M_DEVICE_byte_enable  (M_DEVICE_byte_enable),
        .M_DEVICE_core2dev_data(M_DEVICE_core2dev_data),
        .M_DEVICE_data_ready   (intc_data_ready),
        .M_DEVICE_dev2core_data(intc_data),
        .intr                  (uart_intr),
        .irq                   (intc_irq)
    );
endmodule