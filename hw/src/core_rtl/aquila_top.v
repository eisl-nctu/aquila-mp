`timescale 1 ns / 1 ps
// =============================================================================
//  Program : aquila_top.v
//  Author  : Chun-Jen Tsai
//  Date    : Oct/08/2019
// -----------------------------------------------------------------------------
//  Description:
//  This is the top-level Aquila IP wrapper for an AXI-based processor SoC.
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
//  Mar/05/2020, by Chih-Yu Hsiang:
//    Support for A standard extension.
//
//  Feb/12/2025, by Tzu-Chen Yang:
//    Implemented arbitration of I/D memory requests within aquila_top, 
//    ensuring that only one request is issued at a time.
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

module aquila_top #
(
    parameter integer HART_ID  = 0,
    parameter integer XLEN     = 32,  // Width of RISCV registers.
    parameter integer CLSIZE   = `CLP // Size of a cache block in bits.
)
(
    input                 clk_i,
    input                 rst_i,   // level-sensitive reset signal.

    // Initial program counter address for the Aquila core
    input  [XLEN-1 : 0]   base_addr_i,

    // Aquila external I/D memory interface signals
    // coherence unit signals
    input                 CU_L1_probe_strobe_i,
    input [XLEN-1 : 0]    CU_L1_probe_addr_i,
    input                 CU_L1_invalidate_i,
    input [CLSIZE-1 : 0]  CU_L1_data_i,
    input                 CU_L1_make_exclusive_i,
    input                 CU_L1_response_ready_i,
    input                 CU_L1_is_amo_i, 

    output [CLSIZE-1 : 0] L1_CU_response_data_o,
    output [CLSIZE-1 : 0] L1_CU_wb_data_o,
    output reg [XLEN-1 : 0]   L1_CU_addr_o,
    output reg            L1_CU_strobe_o,
    output reg            L1_CU_rw_o,
    output reg            L1_CU_share_modify_o,
    output                L1_CU_response_ready_o,
    output reg            L1_CU_replacement_o,
    output reg            L1_CU_is_instr_fetch_o,
    
    // atomic extenstion
    output                C2AMO_strobe_o,
    output                C2AMO_rw_o,
    output [XLEN-1 : 0]   C2AMO_addr_o,
    output [XLEN-1 : 0]   C2AMO_data_o,
    input                 C2AMO_data_ready_i,
    input  [XLEN-1 : 0]   C2AMO_data_i,
    output                C2AMO_is_amo_o,
    output [4 : 0]        C2AMO_amo_type_o,

    input                 AMO2C_strobe_i,
    input                 AMO2C_rw_i,
    input [XLEN-1 : 0]    AMO2C_addr_i,
    input [XLEN-1 : 0]    AMO2C_data_i,
    output                AMO2C_data_ready_o,
    output [XLEN-1 : 0]   AMO2C_data_o,

    // Aquila M_DEVICE master port interface signals
    output                M_DEVICE_strobe_o,
    output [XLEN-1 : 0]   M_DEVICE_addr_o,
    output                M_DEVICE_rw_o,
    output [XLEN/8-1 : 0] M_DEVICE_byte_enable_o,
    output [XLEN-1 : 0]   M_DEVICE_data_o,
    input                 M_DEVICE_data_ready_i,
    input  [XLEN-1 : 0]   M_DEVICE_data_i,

    // Aquila M_CLINT master port interface signals
    output                M_CLINT_strobe_o,
    output [XLEN-1 : 0]   M_CLINT_addr_o,
    output                M_CLINT_rw_o,
    output [XLEN-1 : 0]   M_CLINT_data_o,
    input                 M_CLINT_data_ready_i,
    input  [XLEN-1 : 0]   M_CLINT_data_i,

    // External interrupt requests from I/O devices
    input                 ext_irq_i,
    input                 tmr_irq_i,
    input                 sft_irq_i
);

// ------------- Signals for cpu, cache and master ip -------------------------
// CPU core
wire                      code_sel;
wire [1 : 0]              data_sel;

// Cache flush signals
wire                      p_cache_flush;
wire                      dcache_flushing;

// Processor to instruction memory signals.
wire                      p_i_strobe;
wire                      p_i_ready;
wire [XLEN-1 : 0]         p_i_addr;
wire [XLEN-1 : 0]         p_i_code;

wire [XLEN-1 : 0]         code_from_tcm; 
wire [XLEN-1 : 0]         code_from_cache;
wire                      tcm_i_ready;
wire                      cache_i_ready;

// Processor to data memory signals.
wire                      p_d_strobe;
wire                      p_d_ready;
wire [XLEN-1 : 0]         p_d_addr;
wire                      p_d_rw;
wire [XLEN/8-1 : 0]       p_d_byte_enable;
wire [XLEN-1 : 0]         p_d_mem2core;
wire [XLEN-1 : 0]         p_d_core2mem;
wire                      p_d_is_amo;   // Is it an atomic data access?
wire [4 : 0]              p_d_amo_type; // Type of the atomic data access.

// Choose Amo / normal signals to Dache
wire                      P_D_strobe;
wire                      P_D_ready;
wire [XLEN-1 : 0]         P_D_addr;
wire                      P_D_rw;
wire [XLEN/8-1 : 0]       P_D_byte_enable;
wire [XLEN-1 : 0]         P_D_mem2core;
wire [XLEN-1 : 0]         P_D_core2mem;
//
wire [XLEN-1 : 0]         data_from_tcm;
wire [XLEN-1 : 0]         data_from_cache;
wire                      tcm_d_ready;
wire                      cache_d_ready;

// The processor pipeline cannot be interrupted when
// external memory or device accesses is in progress.
wire is_ext_addr = (data_sel != 2'b0);
`ifdef PROFILE_DCACHE
/*
F0010000: I2M counter
F0010004: I2E counter
F0010008: I2S counter
F001000C: S2M counter
F0010010: M2Replaced counter
F0010014: E2Replaced counter

F0010018: I2M avg latency
F001001C: I2E avg latency
F0010020: I2S avg latency
F0010024: S2M avg latency
F0010028: M2Replaced avg latency
F001002C: E2Replaced avg latency
*/
wire            dcache_profile_sel = data_sel == 3 && p_d_addr[31:16] == 16'hF001;
reg             dcache_profile_sel_r;
wire             dcache_profile_ready;
wire [XLEN-1:0]  dcache_profile_data;
always @(posedge clk_i) begin
    if(rst_i)dcache_profile_sel_r <= 0;
    else if(p_d_strobe)  dcache_profile_sel_r <= dcache_profile_sel;
    else dcache_profile_sel_r <= dcache_profile_sel_r;
end
`endif 
// ----------------------------------------------------------------------------
//  Shared output signals for system devices.
wire [3 : 0]      sysdev_sel;
assign sysdev_sel = p_d_addr[19 : 16];
// System device data bus.
wire [XLEN-1 : 0]         data_from_sysdev;
wire                      sysdev_d_ready;
assign data_from_sysdev = (|sysdev_sel)? {XLEN{1'b0}} : M_CLINT_data_i;
// ----------------------------------------------------------------------------
//  We must fake the initial stack responses (at 0xFFFF_0000 ~ 0xFFFF_FFF0).
//  Any stack operations are ignored until sp is properly initialized.
//
assign sysdev_d_ready = (sysdev_sel == 4'h0)? M_CLINT_data_ready_i // 0xF000_0000 ~
                        : 1;                                // 0xF001_0000 ~
// ------ System Memory Map: DDRx DRAM, I/O Devices, or System Devices ---------
//       [0] 0x0000_0000 - 0x0FFF_FFFF : Tightly-Coupled Memory (TCM)
//       [1] 0x8000_0000 - 0xBFFF_FFFF : DDRx DRAM memory (cached)
//       [2] 0xC000_0000 - 0xCFFF_FFFF : device memory (uncached)
//       [3] 0xF000_0000 - 0xFFFF_FFFF : System devices (uncached)
//
wire [3 : 0] code_segment, data_segment;

assign code_segment = p_i_addr[XLEN-1:XLEN-4];
assign data_segment = p_d_addr[XLEN-1:XLEN-4];

assign code_sel = (code_segment == 4'h0)? 0 : 1;
assign data_sel = (data_segment == 4'h0)? 0 :
                  (data_segment == 4'hC)? 2 :
                  (data_segment == 4'hF)? 3 : 1;

assign p_i_code = (code_sel == 0)? code_from_tcm : code_from_cache;
assign p_i_ready = (code_sel == 0)? tcm_i_ready : cache_i_ready;

reg  [1:0] data_sel_r;
always @(posedge clk_i) begin
	data_sel_r <= data_sel;
end

// Delay the memory response by one clock cycle so that
//   the processor core will not miss the ready strobe.
assign p_d_mem2core = (data_sel_r == 0)? data_from_tcm :
                      (data_sel_r == 1)? data_from_cache :
                      (data_sel_r == 2)? M_DEVICE_data_i : 
`ifdef PROFILE_DCACHE
                      (dcache_profile_sel_r)? dcache_profile_data :
`endif
                      data_from_sysdev;
assign p_d_ready = (data_sel_r == 0)? tcm_d_ready :
                   (data_sel_r == 1)? cache_d_ready :
                   (data_sel_r == 2)? M_DEVICE_data_ready_i : 
`ifdef PROFILE_DCACHE
                   (dcache_profile_sel_r)? dcache_profile_ready :
`endif 
                   sysdev_d_ready;

assign M_DEVICE_strobe_o      = p_d_strobe && (data_sel == 2);
assign M_DEVICE_addr_o        = (data_sel == 2)? p_d_addr : 32'h0;
assign M_DEVICE_rw_o          = p_d_rw && (data_sel == 2);
assign M_DEVICE_byte_enable_o = p_d_byte_enable;
assign M_DEVICE_data_o        = (data_sel == 2)? p_d_core2mem : 32'h0;

assign M_CLINT_strobe_o       = p_d_strobe && (data_sel == 3 && sysdev_sel == 4'h0);
assign M_CLINT_addr_o         = p_d_addr;
assign M_CLINT_rw_o           = p_d_rw && (data_sel == 3 && sysdev_sel == 4'h0);
assign M_CLINT_data_o         = p_d_core2mem;
// ----------------------------------------------------------------------------
//  Aquila processor core
//
core_top #(.HART_ID(HART_ID), .XLEN(XLEN))
RISCV_CORE0(
    // System signals
    .clk_i(clk_i),
    .rst_i(rst_i),          // from slave register
    .stall_i(1'b0),         // disable user stall signal

    // Program counter address at reset for the Aquila core
    .init_pc_addr_i(base_addr_i),

    // Instruction port
    .code_i(p_i_code),
    .code_ready_i(p_i_ready),
    .code_addr_o(p_i_addr),
    .code_req_o(p_i_strobe),

    // Data port
    .data_i(P_D_mem2core),
    .data_ready_i(P_D_ready),
    .data_o(p_d_core2mem),
    .data_addr_o(p_d_addr),
    .data_rw_o(p_d_rw),
    .data_byte_enable_o(p_d_byte_enable),
    .data_req_o(p_d_strobe),
    .data_is_amo_o(p_d_is_amo),
    .data_amo_type_o(p_d_amo_type),
    .data_addr_ext_i(is_ext_addr),

    // Cache flush signal
    .cache_flush_o(p_cache_flush),

    // Interrupt signals
    .ext_irq_i(ext_irq_i),     // no external interrupt (yet)
    .tmr_irq_i(tmr_irq_i),
    .sft_irq_i(sft_irq_i)
);

// Core to Memory
// `ifdef ENABLE_ATOMIC_UNIT
    assign C2AMO_strobe_o   = p_d_strobe && (data_sel == 1);
    assign C2AMO_rw_o       = p_d_rw  && (data_sel == 1);
    assign C2AMO_addr_o     = p_d_addr;
    assign C2AMO_data_o     = p_d_core2mem;
    assign C2AMO_is_amo_o   = p_d_is_amo;
    assign C2AMO_amo_type_o = p_d_amo_type;

    // Core to Memory
    assign P_D_strobe      = (p_d_is_amo) ? AMO2C_strobe_i : p_d_strobe;
    assign P_D_rw          = (p_d_is_amo) ? AMO2C_rw_i : p_d_rw;
    assign P_D_addr        = (p_d_is_amo) ? AMO2C_addr_i: p_d_addr;
    assign P_D_byte_enable = (p_d_is_amo) ?'b0 : p_d_byte_enable;
    assign P_D_core2mem    = (p_d_is_amo) ? AMO2C_data_i : p_d_core2mem;
    // Memory to Core
    assign AMO2C_data_o       = (p_d_is_amo) ? p_d_mem2core : 'b0;
    assign AMO2C_data_ready_o = (p_d_is_amo) ? p_d_ready : 'b0;
    assign P_D_mem2core       = (p_d_is_amo) ? C2AMO_data_i : p_d_mem2core;    
    assign P_D_ready          = (p_d_is_amo) ? C2AMO_data_ready_i : p_d_ready;
// `else
//     assign P_D_strobe      = p_d_strobe;
//     assign P_D_rw          = p_d_rw;
//     assign P_D_addr        = p_d_addr;
//     assign P_D_byte_enable = p_d_byte_enable;
//     assign P_D_core2mem    = p_d_core2mem;  

//     assign P_D_mem2core    =  p_d_mem2core;    
//     assign P_D_ready       =  p_d_ready;
// `endif 


//L1 I cache signals
wire [XLEN-1:0] I_CU_addr;
wire I_CU_strobe;
wire CU_I_ready;

// L1 D cache signals
wire [XLEN-1:0]  D_CU_addr;
wire D_CU_strobe;
wire D_CU_rw;
wire CU_D_ready;
wire D_CU_share_modify;
wire D_CU_replacement;

// ----------------------------------------------------------------------------
//  Instiantiation of the dual-port tightly-coupled scratchpad memory module.
//  0x00000000 ~ 0x0FFFFFFF
localparam TCM_NUM_WORDS = `TCM_SIZE_IN_WORDS;
localparam TCM_ADDR_WIDTH = $clog2(TCM_NUM_WORDS);

sram_dp #(.DATA_WIDTH(XLEN), .N_ENTRIES(TCM_NUM_WORDS), .HART_ID(HART_ID))
TCM(
    // Instruction memory ports
    .clk1_i(clk_i),
    .en1_i(p_i_strobe && (code_sel == 0)),
    .we1_i(1'b0),
    .be1_i(4'b1111),
    .addr1_i(p_i_addr[TCM_ADDR_WIDTH+1 : 2]),
    .data1_i({XLEN{1'b0}}),
    .data1_o(code_from_tcm),
    .ready1_o(tcm_i_ready),

    // Data memory ports
    .clk2_i(clk_i),
    .en2_i(p_d_strobe && (data_sel == 0)),
    .we2_i(p_d_rw && (data_sel == 0)),
    .be2_i(p_d_byte_enable),
    .addr2_i(p_d_addr[TCM_ADDR_WIDTH+1 : 2]),
    .data2_i(p_d_core2mem),  // data from processor write bus
    .data2_o(data_from_tcm),
    .ready2_o(tcm_d_ready)
);
`ifdef ENABLE_DDRx_MEMORY
// ----------------------------------------------------------------------------
//  Instiantiation of the I/D-cache modules.
//

// Instruction read from I-cache port.
icache #(.XLEN(XLEN), .CACHE_SIZE(`ICACHE_SIZE), .CLSIZE(CLSIZE))
I_Cache(
    .clk_i(clk_i),
    .rst_i(rst_i),

    .p_addr_i(p_i_addr),
    .p_strobe_i(p_i_strobe && (code_sel == 1)),
    .p_flush_i(p_cache_flush),
    .p_data_o(code_from_cache),
    .p_ready_o(cache_i_ready),

    .m_addr_o(I_CU_addr),
    .m_data_i(CU_L1_data_i),
    .m_strobe_o(I_CU_strobe),
    .m_ready_i(CU_I_ready),

    .d_flushing_i(dcache_flushing)
);

// Data read/write through D-cache port.
    dcache #(.XLEN(32), .CACHE_SIZE(`DCACHE_SIZE), .CLSIZE(`CLP))
    D_Cache(
        //system signals
        .clk_i(clk_i),
        .rst_i(rst_i),
        // Processor signals
        .p_strobe_i(P_D_strobe && (data_sel == 1)),
        .p_rw_i(P_D_rw && (data_sel == 1)),
        .p_byte_enable_i(P_D_byte_enable),
        .p_addr_i(P_D_addr),
        .p_data_i(P_D_core2mem),
        .p_data_o(data_from_cache),
        .p_ready_o(cache_d_ready),
        .p_flush_i(p_cache_flush),
        .busy_flushing_o(dcache_flushing),
        .p_is_amo_i(p_d_is_amo),

        //Cache coherence signals 
        //get probe
        .probe_strobe_i(CU_L1_probe_strobe_i),
        .invalidate_i(CU_L1_invalidate_i),
        .probe_addr_i(CU_L1_probe_addr_i),
        .coherence_data_i(CU_L1_data_i),
        .coherence_done_i(CU_D_ready),
        .make_exclusive_i(CU_L1_make_exclusive_i),
        .probe_is_amo_i(CU_L1_is_amo_i),
        //response data
        .response_data_o(L1_CU_response_data_o),
        .write_back_data_o(L1_CU_wb_data_o),
        .response_ready_o(L1_CU_response_ready_o),
        
        //memory request
        .coherence_strobe_o(D_CU_strobe),
        .coherence_addr_o(D_CU_addr),
        .coherence_replacement_o(D_CU_replacement),
        .coherence_rw_o(D_CU_rw),
        .share_modify_o(D_CU_share_modify)
`ifdef PROFILE_DCACHE
        , 
        .profile_req(dcache_profile_sel && p_d_strobe),
        .profile_addr(p_d_addr),
        .profile_ready(dcache_profile_ready),
        .profile_data(dcache_profile_data)
`endif 
    );
`endif
    reg [3:0] S, S_next;
    localparam S_IDLE = 0, S_I = 1, S_D = 2;
    always @(posedge clk_i) begin
        if(rst_i)  S <= S_IDLE;
        else S <= S_next;
    end
    always @(*) begin
        case (S)
            S_IDLE: 
                if(I_CU_strobe) S_next = S_I;
                else if(D_CU_strobe)  S_next = S_D;
                else  S_next = S_IDLE;
            
            S_I: 
                if(CU_L1_response_ready_i)  S_next = S_IDLE;
                else S_next = S_I;
            
            S_D: 
                if(CU_L1_response_ready_i || !D_CU_strobe) S_next = S_IDLE;
                else S_next = S_D;
            default: S_next = S_IDLE;
        endcase
    end
    // cache arbiter
    always @(posedge clk_i)begin
        if (rst_i) begin
            L1_CU_strobe_o <= 0;
            L1_CU_addr_o <= 0;
            L1_CU_rw_o <= 0;
            L1_CU_is_instr_fetch_o <= 0;
            L1_CU_share_modify_o <= 0;
            L1_CU_replacement_o <= 0;
        end
        else if(S_next == S_I) begin
            L1_CU_strobe_o <= 1;
            L1_CU_rw_o <= 0;
            L1_CU_addr_o <= I_CU_addr;
            L1_CU_is_instr_fetch_o <= 1;
            L1_CU_share_modify_o <= 0;
            L1_CU_replacement_o <= 0;
        end
        else if(S_next == S_D) begin
            L1_CU_strobe_o <= 1;
            L1_CU_rw_o <= D_CU_rw;
            L1_CU_addr_o <= D_CU_addr;
            L1_CU_is_instr_fetch_o <= 0;
            L1_CU_share_modify_o <= D_CU_share_modify;
            L1_CU_replacement_o <= D_CU_replacement;
        end
        else begin
            L1_CU_strobe_o <= 0;
            L1_CU_addr_o <= 0;
            L1_CU_rw_o <= 0;
            L1_CU_is_instr_fetch_o <= 0;
            L1_CU_share_modify_o <= 0;
            L1_CU_replacement_o <= 0;
        end
    end

assign CU_I_ready = S == S_I ? CU_L1_response_ready_i : 0;
assign CU_D_ready =  (S == S_I) ? 0 : (S == S_D) ?  CU_L1_response_ready_i : 0;

endmodule
