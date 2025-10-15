`timescale 1ns / 1ps
// =============================================================================
//  Program : soc_top.v
//  Author  : Chun-Jen Tsai
//  Date    : Feb/16/2020
// -----------------------------------------------------------------------------
//  Description:
//  This is the top-level Aquila IP wrapper for a processor SoC.
// -----------------------------------------------------------------------------
//  Revision information:
//
//  This module is based on the soc_top.v module written by Jin-you Wu
//  on Feb/28/2019. The original module was a stand-alone top-level module
//  for an SoC. This rework makes it a module embedded inside an AXI IP.
//
//  Jan/12/2020, by Chun-Jen Tsai:
//    Added a on-chip Tightly-Coupled Memory (TCM) to the aquila SoC.
//
//  Sep/12/2022, by Chun-Jen Tsai:
//    Fix an issue of missing reset signal across clock domains.
//    Use the clock wizard to generate the Aquila clock on Arty.
//
//  Feb/12/2025, by Tzu-Chen Yang:
//    Added modifications to support multi-core functionality at the top level.
//
//  Aug/21/2025, by Tzu-Chen Yang:
//    Move the CLINT module to the global scope and connect it to each core as well as the arbiter.
// -----------------------------------------------------------------------------
//  License information:
//
//  This software is released under the BSD-3-Clause Licence,
//  see https://opensource.org/licenses/BSD-3-Clause for details.
//  In the following license statements, "software" refers to the
//  "source code" of the complete hardware/software system.
//
//  Copyright 2019 -,
//                    Embedded Intelligent Systems Lab (EISL)
//                    Deparment of Computer Science
//                    National Yang Ming Chiao Tung Uniersity
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

module soc_top #( parameter XLEN = 32, parameter CLSIZE = `CLP )
(
`ifdef ARTY
    input                 sys_clk_i,
    input                 resetn_i,
`elsif QMCore
    input                 sys_clk_i,
`elsif KC705
    input                 sys_clk_p,
    input                 sys_clk_n,
    input                 reset_i,
`elsif Genesys2
    input                 sys_clk_p,
    input                 sys_clk_n,
    input                 resetn_i,
`else
    input                 clk_i,
    input                 reset_i,
`endif

`ifdef ENABLE_DDRx_MEMORY
`ifdef DDR3
    // DDR3 PHY signals
    output [`DRAMA-1 : 0] ddr3_addr,
    output [`BA-1 : 0]    ddr3_ba,
    output                ddr3_cas_n,
    output [0 : 0]        ddr3_ck_n,
    output [0 : 0]        ddr3_ck_p,
    output [0 : 0]        ddr3_cke,
    output                ddr3_ras_n,
    output                ddr3_reset_n,
    output                ddr3_we_n,
    inout  [`DRAMD-1 : 0] ddr3_dq,
    inout  [`DQSP-1 : 0]  ddr3_dqs_n,
    inout  [`DQSP-1 : 0]  ddr3_dqs_p,
`ifndef QMCore
    output [0 : 0]        ddr3_cs_n,
`endif
    output [`DQSP-1 : 0]  ddr3_dm,
    output [0 : 0]        ddr3_odt,
`else
    // DDR4 PHY signals
    output                ddr4_act_n,
    output [`DRAMA-1 : 0] ddr4_addr,
    output [`BA-1 : 0]    ddr4_ba,
    output [ 0 : 0]       ddr4_bg,
    output [ 0 : 0]       ddr4_cke,
    output [ 0 : 0]       ddr4_odt,
    output [ 0 : 0]       ddr4_cs_n,
    output [ 0 : 0]       ddr4_ck_t,
    output [ 0 : 0]       ddr4_ck_c,
    output                ddr4_reset_n,
    inout  [`DQSP-1 : 0]  ddr4_dm_dbi_n,
    inout  [`DRAMD-1 : 0] ddr4_dq,
    inout  [`DQSP-1 : 0]  ddr4_dqs_t,
    inout  [`DQSP-1 : 0]  ddr4_dqs_c,
`endif
`endif

    // sdcard
    output                spi_mosi,
    input                 spi_miso,
    output                spi_sck,
    output [0 : 0]        spi_ss,

    // uart
    input                 uart_rx,
    output                uart_tx,

    // buttons & leds
    input  [0 : `USRP-1]  usr_btn,
    output [0 : `USRP-1]  usr_led
);

// System clock, reset, and LEDs.
`ifdef ARTY
wire clk_166M, clk_200M;
`else
wire clk_200M;
`endif

wire usr_reset;
wire ui_clk, ui_rst;
wire clk, rst;
wire rst_mem_arb;

// uart
localparam CORE_NUMS_BITS = (`CORE_NUMS==1) ? 1 : $clog2(`CORE_NUMS);
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

`ifdef ENABLE_DDRx_MEMORY
// --------- cdc_sync ----------------------------------------------------------

wire                MEM_strobe_ui_clk;
wire [XLEN-1 : 0]   MEM_addr_ui_clk;
wire                MEM_rw_ui_clk;
wire [CLSIZE-1 : 0] MEM_wt_data_ui_clk;
wire                MEM_done_ui_clk;
wire [CLSIZE-1 : 0] MEM_rd_data_ui_clk;
wire [127:0]        dummy_MEM_rd_data_2;
wire                dummy_MEM_ready_2;

// --------- Memory Controller Interface ---------------------------------------
// Xilinx MIG memory controller user-logic interface signals
wire                MEM_calib;
wire                MEM_en;
wire [`UIADR-1 : 0] MEM_addr;
wire [  2 : 0]      MEM_cmd;

wire [`WDFP-1 : 0]  MEM_wdf_data;
wire                MEM_wdf_end;
wire [`WDFP/8-1:0]  MEM_wdf_mask;
wire                MEM_wdf_wren;
wire                MEM_wdf_rdy;

wire [`WDFP-1 : 0]  MEM_rd_data;
wire                MEM_rd_data_end;
wire                MEM_rd_data_valid;
wire                MEM_rdy;

// Only for DDR3
wire                MEM_sr_req;
wire                MEM_ref_req;
wire                MEM_zq_req;
wire                MEM_sr_active;
wire                MEM_ref_ack;
wire                MEM_zq_ack;
`endif

// Uart
wire                uart_sel;
wire [XLEN-1 : 0]   uart_dout;
wire                uart_ready;

// SPI for SD Card
wire                spi_sel;
wire [XLEN-1 : 0]   spi_dout;
wire                spi_ready;

// DSA device signals
wire                dsa_sel;
wire [XLEN-1 : 0]   dsa_dout;
wire                dsa_ready;

// --------- System Clock Generator --------------------------------------------
// Generates the system clock & reset signal.
`ifdef ARTY // ARTY A7, reset button adopts negative logic
assign usr_reset = ~resetn_i;

clk_wiz_0 Clock_Generator(
    .clk_in1(sys_clk_i), // On-board oscillator clock input
    .clk_out1(clk),      // System clock for the Aquila SoC
    .clk_out2(clk_166M), // Clock input to the MIG Memory controller
    .clk_out3(clk_200M)  // DRAM Reference clock for MIG
);
`elsif QMCore
assign usr_reset = ~(&usr_btn);
wire   resetn_i  =  &usr_btn;

clk_wiz_0 Clock_Generator(
    .clk_in1(sys_clk_i), // On-board oscillator clock input
    .clk_out1(clk),      // System clock for the Aquila SoC
    .clk_out2(clk_200M)  // DRAM Reference clock for MIG
);
`elsif Genesys2 // Genesys2, reset button adopts negative logic
assign usr_reset = ~resetn_i;

clk_wiz_0 Clock_Generator(
    .clk_in1(ui_clk),  // Clock input from the MIG Memory controller
    .clk_out1(clk)     // System clock for the Aquila SoC
);
`else // KC705, reset button adopts positive logic
assign usr_reset = reset_i;

clk_wiz_0 Clock_Generator(
    .clk_in1(ui_clk),  // Clock input from the MIG Memory controller
    .clk_out1(clk)     // System clock for the Aquila SoC
);
`endif

// -----------------------------------------------------------------------------
// Generate a system reset signal in the Aquila Core clock domain.
// The reset (rst) should lasts for at least 5 cycles to initialize all the
// all the pipeline registers. The reset signal can also be sent to the clock
// domains with clock rates less than five times of 'clk'.
//
localparam SR_N = 5;
reg [SR_N-1:0] rst_sr = {SR_N{1'b1}};
assign rst = rst_sr[SR_N-1];
always @(posedge clk) begin
    if (usr_reset)
        rst_sr <= {SR_N{1'b1}};
    else
        rst_sr <= {rst_sr[SR_N-2 : 0], 1'b0};
end

// -----------------------------------------------------------------------------
// Generate a system reset signal in the MIG memory controller user-logic
// interface clock domain.  Note that, for some reason, the MIG ui_rst cannot
// be used if the MIG sys_clk_i() is driven by the clock wizard.
//
reg [SR_N-1:0] rst_mem_sr = {SR_N{1'b1}};
assign rst_mem = rst_mem_sr[SR_N-1];
always @(posedge ui_clk) begin
    if (usr_reset)
        rst_mem_sr <= {SR_N{1'b1}};
    else
        rst_mem_sr <= {rst_mem_sr[SR_N-2 : 0], 1'b0};
end

// -----------------------------------------------------------------------------
//  Aquila processor core.
//
    genvar i;
    generate
        for (i = 0; i < `CORE_NUMS; i = i + 1)
        begin
            aquila_top #(.HART_ID(i), .XLEN(XLEN), .CLSIZE(CLSIZE)) 
            aquila_top
            (
                //system signals
                .clk_i(clk),
                .rst_i(rst),

                // Initial program counter address for the Aquila core
                .base_addr_i(32'b0),

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

                .ext_irq_i('b0),
                .tmr_irq_i(tmr_irq[i]),
                .sft_irq_i(sft_irq[i])
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
    .rst_i(rst),

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
    .rst_i(rst),

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
    .rst_i(rst), 

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
    .rst_i(rst),
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
    .rst_i(rst), 

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

// -----------------------------------------------------------------------------
//  Device address decoder.
//
//       [0] 0xC000_0000 - 0xC0FF_FFFF : UART device
//       [1] 0xC100_0000 - 0xC1FF_FFFF : GPIO device
//       [2] 0xC200_0000 - 0xC2FF_FFFF : SPI device
//       [3] 0xC400_0000 - 0xC4FF_FFFF : DSA device
assign uart_sel  = (M_dev_addr[XLEN-1:XLEN-8] == 8'hC0);
assign spi_sel   = (M_dev_addr[XLEN-1:XLEN-8] == 8'hC2);
assign M_dev_dout  = (uart_sel)? uart_dout :
                   (spi_sel)?   spi_dout :
                              {XLEN{1'b0}};
assign M_dev_ready = (uart_sel)? uart_ready :
                   (spi_sel)?   spi_ready :
                                      1'b0;

// ----------------------------------------------------------------------------
//  UART Controller with a simple memory-mapped I/O interface.
//
`define BAUD_RATE	115200

uart #(.BAUD(`SOC_CLK/`BAUD_RATE))
UART(
    .clk(clk),
    .rst(rst),

    .EN(M_dev_strobe & uart_sel),
    .ADDR(M_dev_addr[3:2]),
    .WR(M_dev_we),
    .BE(M_dev_be),
    .DATAI(M_dev_din),
    .DATAO(uart_dout),
    .READY(uart_ready),

    .RXD(uart_rx),
    .TXD(uart_tx)
);

// -----------------------------------------------------------------------------
//  SD Card SPI Controller with AXI bus interface.
//
wire                axi_arvalid;
wire                axi_awvalid;
wire [6 : 0]        axi_awaddr;
wire [6 : 0]        axi_araddr;
wire                axi_arready;
wire                axi_awready;
wire [XLEN/8-1 : 0] axi_wstrb;
wire                axi_wready;
wire                axi_rready;
wire                axi_wvalid;
wire                axi_rvalid;
wire [1 : 0]        axi_bresp;
wire [1 : 0]        axi_rresp;
wire                axi_bvalid;
wire                axi_bready;
wire [XLEN-1 : 0]   axi_rdata;
wire [XLEN-1 : 0]   axi_wdata;

// The following signals are unused output signals from the Xilinx AXI SPI IP:
wire              io0_t_dummy, io1_t_dummy, sck_t_dummy, ss_t_dummy;
wire              io1_o_dummy, irpt_dummy;

// ---------------------------------------
//  Aquila local bus to AXI bus interface
// ---------------------------------------
core2axi_if #(.XLEN(32), .AXI_ADDR_LEN(7))
Core2AXI_0 (
    .clk_i(clk),
    .rst_i(rst),

    // Aquila M_DEVICE master interface signals.
    .S_DEVICE_strobe_i(M_dev_strobe & spi_sel),
    .S_DEVICE_addr_i(M_dev_addr),
    .S_DEVICE_rw_i(M_dev_we),
    .S_DEVICE_byte_enable_i(M_dev_be),
    .S_DEVICE_data_i(M_dev_din),
    .S_DEVICE_data_ready_o(spi_ready),
    .S_DEVICE_data_o(spi_dout),

    // Converted AXI master interface signals.
    .m_axi_awaddr(axi_awaddr),   // Master write address signals.
    .m_axi_awvalid(axi_awvalid), // Master write addr/ctrl is valid.
    .m_axi_awready(axi_awready), // Slave ready to receive write command.
    .m_axi_wdata(axi_wdata),     // Master write data signals.
    .m_axi_wstrb(axi_wstrb),     // Master byte select signals.
    .m_axi_wvalid(axi_wvalid),   // Master write data is valid.
    .m_axi_wready(axi_wready),   // Slave ready to receive write data.
    .m_axi_bresp(axi_bresp),     // Slave write-op response signal.
    .m_axi_bvalid(axi_bvalid),   // Slave write-op response is valid.
    .m_axi_bready(axi_bready),   // Master ready to receive write response.
    .m_axi_araddr(axi_araddr),   // Master read address signals.
    .m_axi_arvalid(axi_arvalid), // Master read addr/ctrl is valid.
    .m_axi_arready(axi_arready), // Slave is ready to receive read command.
    .m_axi_rdata(axi_rdata),     // Slave read data signals.
    .m_axi_rresp(axi_rresp),     // Slave read-op response signal
    .m_axi_rvalid(axi_rvalid),   // Slave read response is valid.
    .m_axi_rready(axi_rready)    // Master ready to receive read response.
);

// ----------------------------------
//  SPI controller
// ----------------------------------
//  This controller connects to the PMOD microSD module in
//  the JD connector of the Arty A7-100T, or the SD socket of
//  KC705, AXKU5, PZKU5, Genesys2, K7BaseC, or QMCore.
//
axi_quad_spi_0 SD_Card_Controller(

    // Interface ports to the Aquila SoC.
    .s_axi_aclk(clk),
    .s_axi_aresetn(~rst),
    .s_axi_awaddr(axi_awaddr),
    .s_axi_awvalid(axi_awvalid),        // master signals write addr/ctrl valid.
    .s_axi_awready(axi_awready),        // slave ready to fetch write address.
    .s_axi_wdata(axi_wdata),            // write data to the slave.
    .s_axi_wstrb(axi_wstrb),            // byte select signal for write operation.
    .s_axi_wvalid(axi_wvalid),          // master signals write data is valid.
    .s_axi_wready(axi_wready),          // slave ready to accept the write data.
    .s_axi_araddr(axi_araddr),
    .s_axi_arready(axi_arready),        // slave ready to fetch read address.
    .s_axi_arvalid(axi_arvalid),        // master signals read addr/ctrl valid.
    .s_axi_bready(axi_bready),          // master is ready to accept the response.
    .s_axi_bresp(axi_bresp),            // reponse code from the slave.
    .s_axi_bvalid(axi_bvalid),          // slave has sent the respond signal.
    .s_axi_rdata(axi_rdata),            // read data from the slave.
    .s_axi_rready(axi_rready),          // master is ready to accept the read data.
    .s_axi_rresp(axi_rresp),            // slave sent read response.
    .s_axi_rvalid(axi_rvalid),          // slave signals read data ready.

    // Interface ports to the SD Card.
    .ext_spi_clk(clk),
    .io0_i(1'b0),
    .io0_o(spi_mosi),
    .io0_t(io0_t_dummy),                // tag signal for mosi (ignore it)
    .io1_i(spi_miso),
    .io1_o(io1_o_dummy),                // output signal for io1 (ignore it)
    .io1_t(io1_t_dummy),                // tag signal for miso (ignore it)
    .sck_i(1'b0),
    .sck_o(spi_sck),
    .sck_t(sck_t_dummy),                // tag signal for sck (ignore it)
    .ss_i(1'b0),
    .ss_o(spi_ss),
    .ss_t(ss_t_dummy),                  // tag signal for ss (ignore it)
    .ip2intc_irpt(irpt_dummy)           // interrupt from SPI controller (ignore it)
);

// ----------------------------------
//  coherence unit
// ----------------------------------
 coherence_unit #(.XLEN(32), .CLSIZE(CLSIZE), .CORE_NUMS(`CORE_NUMS), .CORE_NUMS_BITS(CORE_NUMS_BITS)) 
    coherence_unit
    (
        //===================== System signals =====================//
        .clk_i(clk),
        .rst_i(rst),
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
        .rst_i(rst),
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

`ifdef ENABLE_DDRx_MEMORY
// ----------------------------------------------------------------------------
//  cdc_sync.
//
cdc_sync synchronizer
(
    .clk_core(clk),
    .clk_memc(ui_clk),
    .rst_core_i(rst),
    .rst_memc_i(rst_mem),

    .P_strobe_i(L2_MEM_strobe),
    .P_addr_i(L2_MEM_addr),
    .P_rw_i(L2_MEM_rw),
    .P_wt_data_i(L2_MEM_data),
    .P_done_o(MEM_L2_ready),
    .P_rd_data_o(MEM_L2_data),

    .P_strobe_o(MEM_strobe_ui_clk),
    .P_addr_o(MEM_addr_ui_clk),
    .P_rw_o(MEM_rw_ui_clk),
    .P_wt_data_o(MEM_wt_data_ui_clk),
    .P_done_i(MEM_done_ui_clk),
    .P_rd_data_i(MEM_rd_data_ui_clk)
);

// ----------------------------------------------------------------------------
//  mem_arbiter.
//
mem_arbiter Memory_Arbiter
(
    // System signals
    .clk_i(ui_clk),
    .rst_i(rst_mem),

    // Aquila M_P0_CACHE master port interface signals
    .P0_MEM_strobe_i(MEM_strobe_ui_clk),
    .P0_MEM_addr_i(MEM_addr_ui_clk),
    .P0_MEM_rw_i(MEM_rw_ui_clk),
    .P0_MEM_data_i(MEM_wt_data_ui_clk),
    .P0_MEM_done_o(MEM_done_ui_clk),
    .P0_MEM_data_o(MEM_rd_data_ui_clk),

    // Aquila M_P1_DCACHE master port interface signals
    .P1_MEM_strobe_i(1'b0),
    .P1_MEM_addr_i(32'b0),
    .P1_MEM_rw_i(1'b0),
    .P1_MEM_data_i(128'b0),
    .P1_MEM_done_o(dummy_MEM_ready_2),
    .P1_MEM_data_o(dummy_MEM_rd_data_2),
    
    // Memory user interface signals
    .M_MEM_addr_o(MEM_addr),
    .M_MEM_cmd_o(MEM_cmd),
    .M_MEM_en_o(MEM_en),
    .M_MEM_wdf_data_o(MEM_wdf_data),
    .M_MEM_wdf_end_o(MEM_wdf_end),
    .M_MEM_wdf_mask_o(MEM_wdf_mask),
    .M_MEM_wdf_wren_o(MEM_wdf_wren),
    .M_MEM_rd_data_i(MEM_rd_data),
    .M_MEM_rd_data_valid_i(MEM_rd_data_valid),
    .M_MEM_rdy_i(MEM_rdy),
    .M_MEM_wdf_rdy_i(MEM_wdf_rdy)

`ifdef DDR3
    // Only for DDR3, not really used.
    ,
    .M_MEM_sr_req_o(MEM_sr_req),
    .M_MEM_ref_req_o(MEM_ref_req),
    .M_MEM_zq_req_o(MEM_zq_req)
`endif
);

// ----------------------------------------------------------------------------
//  DDR Memory Controller.
//
`ifdef DDR3

mig_7series_0 MIG(
    // Memory interface ports
    .ddr3_addr(ddr3_addr),                  // output [13:0]  ddr3_addr
    .ddr3_ba(ddr3_ba),                      // output [2:0]   ddr3_ba
    .ddr3_cas_n(ddr3_cas_n),                // output         ddr3_cas_n
    .ddr3_ck_n(ddr3_ck_n),                  // output [0:0]   ddr3_ck_n
    .ddr3_ck_p(ddr3_ck_p),                  // output [0:0]   ddr3_ck_p
    .ddr3_cke(ddr3_cke),                    // output [0:0]   ddr3_cke
    .ddr3_ras_n(ddr3_ras_n),                // output         ddr3_ras_n
    .ddr3_reset_n(ddr3_reset_n),            // output         ddr3_reset_n
    .ddr3_we_n(ddr3_we_n),                  // output         ddr3_we_n
    .ddr3_dq(ddr3_dq),                      // inout [15:0]   ddr3_dq
    .ddr3_dqs_n(ddr3_dqs_n),                // inout [1:0]    ddr3_dqs_n
    .ddr3_dqs_p(ddr3_dqs_p),                // inout [1:0]    ddr3_dqs_p

`ifndef QMCore
    .ddr3_cs_n(ddr3_cs_n),                  // output [0:0]   ddr3_cs_n
`endif
    .ddr3_dm(ddr3_dm),                      // output [1:0]   ddr3_dm
    .ddr3_odt(ddr3_odt),                    // output [0:0]   ddr3_odt
    .init_calib_complete(MEM_calib),        // output         init_calib_complete

    // Application interface ports
    .app_addr(MEM_addr),                    // input [27:0]   app_addr
    .app_cmd(MEM_cmd),                      // input [2:0]    app_cmd
    .app_en(MEM_en),                        // input          app_en
    .app_wdf_data(MEM_wdf_data),            // input [127:0]  app_wdf_data
    .app_wdf_end(MEM_wdf_end),              // input          app_wdf_end
    .app_wdf_mask(MEM_wdf_mask),            // input [15:0]   app_wdf_mask
    .app_wdf_wren(MEM_wdf_wren),            // input          app_wdf_wren
    .app_rd_data(MEM_rd_data),              // output [127:0] app_rd_data
    .app_rd_data_end(MEM_rd_data_end),      // output         app_rd_data_end
    .app_rd_data_valid(MEM_rd_data_valid),  // output         app_rd_data_valid
    .app_rdy(MEM_rdy),                      // output         app_rdy
    .app_wdf_rdy(MEM_wdf_rdy),              // output         app_wdf_rdy

    .app_sr_req(MEM_sr_req),                // input          app_sr_req
    .app_ref_req(MEM_ref_req),              // input          app_ref_req
    .app_zq_req(MEM_zq_req),                // input          app_zq_req
    .app_sr_active(MEM_sr_active),          // output         app_sr_active
    .app_ref_ack(MEM_ref_ack),              // output         app_ref_ack
    .app_zq_ack(MEM_zq_ack),                // output         app_zq_ack

    // System Clock & Reset Ports
`ifdef ARTY
    .sys_clk_i(clk_166M),                   // input          memory controller ref. clock
    .sys_rst(resetn_i),                     // input          sys_rst

    // 200MHz Reference Clock Ports (only needed when the ui_clk is not 200MHz)
    .clk_ref_i(clk_200M),
`elsif QMCore
    .sys_clk_i(clk_200M),                   // input          memory controller ref. clock
    .sys_rst(resetn_i),                     // input          sys_rst

    // 200MHz Reference Clock Ports (only needed when the ui_clk is not 200MHz)
    //.clk_ref_i(clk_200M),
`elsif Genesys2
    .sys_clk_n(sys_clk_n),                  // input          memory controller ref. clock
    .sys_clk_p(sys_clk_p),
    .sys_rst(resetn_i),                     // input          sys_rst

`else // KC705
    .sys_clk_n(sys_clk_n),                  // input          memory controller ref. clock
    .sys_clk_p(sys_clk_p),
    .sys_rst(reset_i),                      // input          sys_rst

    // 200MHz Reference Clock Ports (only needed when the ui_clk is not 200MHz)
    // .clk_ref_i(clk_200M),
`endif

    // User-logic Interface (UI) clock & reset signals.
    .ui_clk(ui_clk),                        // output         ui_clk
    .ui_clk_sync_rst(ui_rst)                // output         ui_clk_sync_rst
);

`else // DDR4
wire           dbg_clk;
wire [511 : 0] dbg_bus;

ddr4_0 MIG (
    // System Clock & Reset Ports
    .c0_sys_clk_p(sys_clk_p),                       // input wire sys_clk_p
    .c0_sys_clk_n(sys_clk_n),                       // input wire sys_clk_n
    .sys_rst(usr_reset),                            // input wire sys_rst

    .c0_init_calib_complete(MEM_calib),             // output wire init_calib_complete

    // DRAM chips interface ports
    .c0_ddr4_act_n(ddr4_act_n),                     // output wire c0_ddr4_act_n
    .c0_ddr4_adr(ddr4_addr),                        // output wire [16 : 0] ddr4_adr
    .c0_ddr4_ba(ddr4_ba),                           // output wire [1 : 0] ddr4_ba
    .c0_ddr4_bg(ddr4_bg),                           // output wire [0 : 0] c0_ddr4_bg
    .c0_ddr4_cke(ddr4_cke),                         // output wire [0 : 0] ddr4_cke
    .c0_ddr4_odt(ddr4_odt),                         // output wire [0 : 0] ddr4_odt
    .c0_ddr4_cs_n(ddr4_cs_n),                       // output wire [0 : 0] ddr4_cs_n
    .c0_ddr4_ck_t(ddr4_ck_t),                       // output wire [0 : 0] ddr4_ck_t
    .c0_ddr4_ck_c(ddr4_ck_c),                       // output wire [0 : 0] ddr4_ck_c
    .c0_ddr4_reset_n(ddr4_reset_n),                 // output wire ddr4_reset_n
    .c0_ddr4_dm_dbi_n(ddr4_dm_dbi_n),               // inout wire [3 : 0] ddr4_dm_dbi_n
    .c0_ddr4_dq(ddr4_dq),                           // inout wire [31 : 0] ddr4_dq
    .c0_ddr4_dqs_t(ddr4_dqs_t),                     // inout wire [3 : 0] ddr4_dqs_t
    .c0_ddr4_dqs_c(ddr4_dqs_c),                     // inout wire [3 : 0] ddr4_dqs_c

    // Debug ports
    .dbg_clk(dbg_clk),                              // output wire dbg_clk
    .dbg_bus(dbg_bus),                              // output wire [511 : 0] dbg_bus

    // Application interface ports
    .c0_ddr4_app_en(MEM_en),                        // input wire c0_ddr4_app_en
    .c0_ddr4_app_addr(MEM_addr),                    // input wire [28 : 0] ddr4_app_addr
    .c0_ddr4_app_cmd(MEM_cmd),                      // input wire [2 : 0] ddr4_app_cmd

    .c0_ddr4_app_wdf_data(MEM_wdf_data),            // input wire [255 : 0] ddr4_app_wdf_data
    .c0_ddr4_app_wdf_end(MEM_wdf_end),              // input wire ddr4_app_wdf_end
    .c0_ddr4_app_wdf_mask(MEM_wdf_mask),            // input wire [31 : 0] ddr4_app_wdf_mask
    .c0_ddr4_app_wdf_wren(MEM_wdf_wren),            // input wire ddr4_app_wdf_wren
    .c0_ddr4_app_wdf_rdy(MEM_wdf_rdy),              // output wire ddr4_app_wdf_rdy

    .c0_ddr4_app_rd_data(MEM_rd_data),              // output wire [255 : 0] ddr4_app_rd_data
    .c0_ddr4_app_rd_data_end(MEM_rd_data_end),      // output wire ddr4_app_rd_data_end
    .c0_ddr4_app_rd_data_valid(MEM_rd_data_valid),  // output wire ddr4_app_rd_data_valid
    .c0_ddr4_app_rdy(MEM_rdy),                      // output wire ddr4_app_rdy

    .c0_ddr4_app_hi_pri(1'b0),                      // input wire ddr4_app_hi_pri

    // User-logic Interface (UI) clock & reset signals.
    .c0_ddr4_ui_clk(ui_clk),                        // output wire ddr4_ui_clk
    .c0_ddr4_ui_clk_sync_rst(ui_rst)                // output wire ddr4_ui_clk_sync_rst
);
`endif

`endif // ifdef ENABLE_DDRx_MEMORY
endmodule
