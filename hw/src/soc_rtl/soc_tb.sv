`timescale 1ns / 1ps
// =============================================================================
//  Program : soc_tb.v
//  Author  : Chun-Jen Tsai
//  Date    : Feb/24/2020
// -----------------------------------------------------------------------------
//  Description:
//  This is the top-level Aquila testbench.
// -----------------------------------------------------------------------------
//  Revision information:
//
//  None.
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

`define SIM_CLK_RATE 100_000_000

module soc_tb #( parameter XLEN = 32, parameter CLSIZE = `CLP )();

reg  sys_reset = 1;
reg  sys_clock = 0;

wire usr_reset;
wire ui_clk, ui_rst;
wire clk, rst;

// uart
wire                uart_rx = 1; /* When the UART rx line is idle, it carries '1'. */
wire                uart_tx;
localparam CORE_NUMS_BITS = (`CORE_NUMS==1) ? 0 : $clog2(`CORE_NUMS);
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
wire                MEM_L2_ready = MEM_done_ui_clk;
wire [CLSIZE-1:0]   MEM_L2_data = MEM_rd_data_ui_clk;
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


// --------- cdc_sync ----------------------------------------------------------
wire                MEM_strobe_ui_clk;
wire [XLEN-1 : 0]   MEM_addr_ui_clk;
wire                MEM_rw_ui_clk;
wire [CLSIZE-1 : 0] MEM_wt_data_ui_clk;
wire                MEM_done_ui_clk;
wire [CLSIZE-1 : 0] MEM_rd_data_ui_clk;
// --------- Memory Controller Interface ---------------------------------------
// Xilinx MIG memory controller user-logic interface signals
wire [27:0]         MEM_addr;
wire [2:0]          MEM_cmd;
wire                MEM_en;
wire [`WDFP-1:0]    MEM_wdf_data;
wire                MEM_wdf_end;
wire [`WDFP/8-1:0]  MEM_wdf_mask = {(`WDFP/8){1'b0}};
wire                MEM_wdf_wren;
wire [`WDFP-1:0]    MEM_rd_data;
wire                MEM_rd_data_end;
wire                MEM_rd_data_valid;
wire                MEM_rdy;
wire                MEM_wdf_rdy;
wire                MEM_sr_req;
wire                MEM_ref_req;
wire                MEM_zq_req;

// sdcard
wire                spi_sel = 0;
wire [XLEN-1 : 0]   spi_dout = 0;
wire                spi_ready = 0;

// uart
wire                uart_sel;
wire [XLEN-1 : 0]   uart_dout;
wire                uart_ready;
wire  [1:0]         uart_core_sel;
// External reset signal
assign usr_reset = sys_reset;

// --------- System Clock Generator --------------------------------------------
assign clk = sys_clock;

always
  #((1_000_000_000/`SIM_CLK_RATE)/2) sys_clock <= ~sys_clock; // 100 MHz

// -----------------------------------------------------------------------------
// For the Aquila Core, the reset (rst) will lasts for 5 cycles to clear
//   all the pipeline registers.
//
localparam RST_CYCLES=5;
reg [RST_CYCLES-1 : 0] rst_count = {RST_CYCLES{1'b1}};
assign rst = rst_count[RST_CYCLES-1];

always @(posedge clk)
begin
    if (usr_reset)
        rst_count <= {RST_CYCLES{1'b1}};
    else
        rst_count <= {rst_count[RST_CYCLES-2 : 0], 1'b0};
end

// Simulate a clock-domain for DRAM
assign ui_clk = sys_clock;
assign ui_rst = rst_count[RST_CYCLES-1];

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
                .M_DEVICE_data_i(dev_dout[i])
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
//  Device Arbiter.
device_arbiter #(.XLEN(XLEN), .CORE_NUMS(`CORE_NUMS), .CORE_NUMS_BITS(CORE_NUMS_BITS))
device_arbiter(
    .clk_i(clk),
    .rst_i(rst), 

    .P_DEVICE_strobe_i(dev_strobe),
    .P_DEVICE_addr_i(dev_addr),
    .P_DEVICE_rw_i(dev_we),
    .P_DEVICE_byte_enable_i(dev_be),
    .P_DEVICE_data_i(dev_din),
    .P_DEVICE_data_ready_o(dev_ready),
    .P_DEVICE_data_o(dev_dout),

    .DEVICE_strobe_o(M_dev_strobe),
    .DEVICE_addr_o(M_dev_addr),
    .DEVICE_rw_o(M_dev_we),
    .DEVICE_byte_enable_o(M_dev_be),
    .DEVICE_data_o(M_dev_din),
    .DEVICE_data_ready_i(M_dev_ready),
    .DEVICE_data_i(M_dev_dout),

    .uart_core_sel_o(uart_core_sel)
);

// -----------------------------------------------------------------------------
//  Device address decoder.
//
//       [0] 0xC000_0000 - 0xC0FF_FFFF : UART device
//       [1] 0xC200_0000 - 0xC2FF_FFFF : SPI device
//       [2] 0xC400_0000 - 0xC4FF_FFFF : DSA device
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

wire simulation_finished;

uart #(.BAUD(`SIM_CLK_RATE/`BAUD_RATE), .CORE_NUMS(`CORE_NUMS), .CORE_NUMS_BITS(CORE_NUMS_BITS)) 
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
    .TXD(uart_tx),

    .simulation_done(simulation_finished),
    .uart_core_sel(uart_core_sel)
);

// ----------------------------------------------------------------------------
//  Print simulation termination message.
//
always @(posedge clk)
begin
    if (simulation_finished) begin
        $display();
        $display("Simulation finished.");
        $finish();
    end
end
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
//  In the real system, the memory controller 'MIG' operates across two clock
//  domains, namely, 'clk' and 'ui_clk'.  For simulation, we do not have two
//  clock domains. However, we still use two clock names (of the same system
//  clcok) such that we may actually simulate two clock domains in the future.
//

// Processor1 Memory Control Signals.
assign MEM_strobe_ui_clk  = L2_MEM_strobe;
assign MEM_addr_ui_clk    = L2_MEM_addr;
assign MEM_rw_ui_clk      = L2_MEM_rw;
assign MEM_wt_data_ui_clk = L2_MEM_data;

// ----------------------------------------------------------------------------
//  mem_arbiter.
//
mem_arbiter Memory_Arbiter
(
    // System signals
    .clk_i(ui_clk),
    .rst_i(rst),

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
    .P1_MEM_done_o(),
    .P1_MEM_data_o(),
    
    // memory user interface signals
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
    .M_MEM_wdf_rdy_i(MEM_wdf_rdy),
    .M_MEM_sr_req_o(MEM_sr_req),
    .M_MEM_ref_req_o(MEM_ref_req),
    .M_MEM_zq_req_o(MEM_zq_req)
);

// ----------------------------------------------------------------------------
//  A simple MIG simulation model.
//
//  Simple DRAM memory controller simulation.
//  0x8000_0000 ~ 0x8010_0000
localparam DRAM_NLINES = 32*(1024*1024*8*4)/`WDFP; // 1 MB DRAM
localparam DRAM_ADDR_WIDTH = $clog2(DRAM_NLINES);

mig_7series_sim #(.DATA_WIDTH(`WDFP), .N_ENTRIES(DRAM_NLINES))
MIG(
    .clk_i(clk),
    .rst_i(rst),

    // Application interface ports
    .app_addr(MEM_addr),                    // input [27:0]        app_addr
    .app_cmd(MEM_cmd),                      // input [2:0]         app_cmd
    .app_en(MEM_en),                        // input               app_en
    .app_wdf_data(MEM_wdf_data),            // input [`WDFP-1:0]   app_wdf_data
    .app_wdf_end(MEM_wdf_end),              // input               app_wdf_end
    .app_wdf_mask(MEM_wdf_mask),            // input [`WDFP/8-1:0] app_wdf_mask
    .app_wdf_wren(MEM_wdf_wren),            // input               app_wdf_wren
    .app_rd_data(MEM_rd_data),              // output [`WDFP-1:0]  app_rd_data
    .app_rd_data_end(MEM_rd_data_end),      // output              app_rd_data_end
    .app_rd_data_valid(MEM_rd_data_valid),  // output              app_rd_data_valid
    .app_rdy(MEM_rdy),                      // output              app_rdy
    .app_wdf_rdy(MEM_wdf_rdy),              // output              app_wdf_rdy
    .app_sr_req(MEM_sr_req),                // input               app_sr_req
    .app_ref_req(MEM_ref_req),              // input               app_ref_req
    .app_zq_req(MEM_zq_req),                // input               app_zq_req
    .app_sr_active(MEM_sr_active),          // output              app_sr_active
    .app_ref_ack(MEM_ref_ack),              // output              app_ref_ack
    .app_zq_ack(MEM_zq_ack)                 // output              app_zq_ack
);

`endif  // ENABLE_DDRx_MEMORY

// ----------------------------------------------------------------------------
//  Reset logic simulation.
//
reg reset_trigger;

initial begin
  forever begin
    @ (posedge reset_trigger);
    sys_reset = 1;
    @ (posedge clk);
    @ (posedge clk);
    sys_reset = 0;
  end
end

initial
begin: TEST_CASE
  #10 reset_trigger = 1;
  @(negedge sys_reset)
  reset_trigger = 0;
end

endmodule

