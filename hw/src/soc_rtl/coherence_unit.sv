`timescale 1ns / 1ps
// =============================================================================
//  Program : coherence_unit.v
//  Author  : Lin-en Yen
//  Date    : Feb/25/2024
// -----------------------------------------------------------------------------
//  Description:
//      The coherence controller for a multi-core system.
// -----------------------------------------------------------------------------
//  Revision information:
//
//  Feb/12/2025, by Ye Chen:
//    Modified to implement the standard MESI protocol state transition method.
//    Extended support to four cores.
//
//  Apr/10/2025, by Tzu-Chen Yang:
//    Extended support to 2, 4, 8, and 16 cores.
//
//  Sept/14/2025, by Tzu-Chen Yang:
//    Extended support to N cores (no limit).
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

module coherence_unit #(parameter XLEN = 32, parameter CLSIZE = 128, parameter CORE_NUMS =`CORE_NUMS, parameter CORE_NUMS_BITS = 2)(
    //===================== System signals =====================//
    input                     clk_i,
    input                     rst_i,
    //===================== CORE 0-3 =====================//
    // request Data
    input                     broadcast_strobe[0 : CORE_NUMS-1],
    input                     read_write[0 : CORE_NUMS-1],
    input  [XLEN-1 : 0]       data_address[0 : CORE_NUMS-1],
    input                     share_modify[0 : CORE_NUMS-1],
    output reg [CLSIZE-1 :0]  Data[0 : CORE_NUMS-1],
    output reg                request_data_ready[0 : CORE_NUMS-1],
    input                     replacement[0 : CORE_NUMS-1],
    input                     L1_is_instr_fetch[0 : CORE_NUMS-1],
    // response Data
    output reg                probe_strobe[0 : CORE_NUMS-1],
    output reg [XLEN-1 : 0]   probe_data_address[0 : CORE_NUMS-1],
    output reg                invalidate[0 : CORE_NUMS-1],
    input  [CLSIZE-1 : 0]     response_data[0 : CORE_NUMS-1],
    input  [CLSIZE-1 : 0]     write_back_data[0 : CORE_NUMS-1],
    input                     response_data_ready[0 : CORE_NUMS-1],
    output reg                make_exclusive_L1[0 : CORE_NUMS-1],
    //===================== L2 cache =====================//
    //write back to L2
    output reg                write_to_L2,
    output reg [CLSIZE-1 : 0] data_to_L2,
    output reg                replacement_L2,
    //request L2 Data
    output reg                probe_strobe_L2,
    output reg [XLEN-1 : 0]   address_to_L2,
    output reg                L1_iswrite_L2,
    input  [CLSIZE-1 : 0]     response_data_L2,
    input                     response_ready_L2,
    input                     make_L1_exclusive_L2,
    output reg                L1_is_instr_fetch_L2,
    // Invalidate L2 Data
    output reg                invalidate_L2,

    //L2 invalidate L1 signal
    output  reg               CU_L2_ready_o,             // CU is ready  
    input      [XLEN-1 : 0]   L2_invalidate_L1_addr_i,   // invalidate L1 address
    input                     L2_invalidate_L1_i,        // invalidate L1 signal
    // Atomic 
    input                     P_amo_strobe,
    input  [CORE_NUMS_BITS-1:0]              AMO_id,
    input                     other_amo_done[0 : CORE_NUMS-1],
    output                    CU_L1_other_amo_o[0 : CORE_NUMS-1]
    
);

reg [CORE_NUMS_BITS-1:0]    current_wb_core_id;
reg [CORE_NUMS_BITS-1:0]    current_invalidate_core_id;
reg [CORE_NUMS_BITS-1:0]    current_rw_core_id;

integer                     i;
genvar                      idx;
wire                        have_wb, have_invalidate, have_rw;
wire                        any_wb          [0 : CORE_NUMS-1];
wire                        any_invalidate  [0 : CORE_NUMS-1];
wire                        any_rw          [0 : CORE_NUMS-1];
wire                        any_response    [0 : CORE_NUMS-1];
wire                        L1_has_response;
wire [CORE_NUMS_BITS-1:0]   L1_response_core_id;
reg [XLEN-1 : 0]            wb_addr_r;
reg [CLSIZE - 1 :0]         wb_data_r;

 reg [3:0]                   S, S_next;

localparam S_Idle              = 4'd0;
localparam S_Miss              = 4'd1; // rw miss
localparam S_Invalidate        = 4'd2; // write hit on Shared state
localparam S_Replacement       = 4'd3; // write back to L2
localparam S_WB_L2             = 4'd4; // write back to L2
localparam S_L2_Invalidate     = 4'd5; // invalidate L2
localparam S_Wait              = 4'd6; // wait for L1 to be ready
localparam S_Wait2             = 4'd7; // wait for L1 to be ready
localparam S_Replacement_Delay = 4'd8; // write back to L2
localparam S_Delay             = 4'd9;

reg                         amo_S, amo_S_next;
localparam                  amo_S_idle = 0, amo_S_busy = 1;
reg [CORE_NUMS_BITS-1:0]    amo_core_id;

wire [CLSIZE-1 : 0]         response_data_mux;
reg [XLEN-1 : 0]            invalidate_address_L2;
reg [XLEN-1 : 0]            write_data_address_L2;
reg [XLEN-1 : 0 ]           probe_data_address_L2;

always @(posedge clk_i ) begin
    if (rst_i) amo_core_id <= 0;
    else if (P_amo_strobe && amo_S == amo_S_idle) begin
        amo_core_id <= AMO_id;
    end
    else if (amo_S == amo_S_idle) begin
        amo_core_id <= 0;
    end
end

always @(posedge clk_i)
begin
    if (rst_i) amo_S <= amo_S_idle;
    else  amo_S <= amo_S_next;
end

always @(*) begin
    case (amo_S)
        amo_S_idle: amo_S_next = (P_amo_strobe) ? amo_S_busy : amo_S_idle;
        amo_S_busy: amo_S_next = (other_amo_done[amo_core_id]) ? amo_S_idle : amo_S_busy;
        default:  amo_S_next = amo_S_idle;
    endcase
end

generate
    for(idx = 0; idx < CORE_NUMS; idx = idx + 1) begin
        assign CU_L1_other_amo_o[idx] = (amo_S != amo_S_idle && idx != amo_core_id);
        assign any_wb[idx]         = (replacement[idx] && !CU_L1_other_amo_o[idx]);
        assign any_invalidate[idx] = (share_modify[idx] && !CU_L1_other_amo_o[idx]);
        assign any_rw[idx]         = (broadcast_strobe[idx] && !CU_L1_other_amo_o[idx]);
        assign any_response[idx]   = response_data_ready[idx];
    end
endgenerate

function integer find_next_index(input strobe[0 : CORE_NUMS-1], input integer core_id);
integer idx;
begin
    for (idx = 0; idx < CORE_NUMS; idx = idx+1) begin
        if (strobe[(core_id+idx)%CORE_NUMS]) begin
            return (core_id+idx)%CORE_NUMS;
        end
    end
end
endfunction

always @(posedge clk_i) begin
    if (rst_i) 
        current_wb_core_id <= 0;
    else if (S == S_Idle) 
        current_wb_core_id <= find_next_index(any_wb, 0);
    else 
        current_wb_core_id <= current_wb_core_id;
end

always @(posedge clk_i) begin
    if (rst_i) 
        current_invalidate_core_id <= 0;
    else if (S == S_Idle) 
        current_invalidate_core_id <= find_next_index(any_invalidate, 0);
    else 
        current_invalidate_core_id <= current_invalidate_core_id;
end

always @(posedge clk_i) begin
    if (rst_i) 
        current_rw_core_id <= 0;
    else if (S == S_Idle) 
        current_rw_core_id <= find_next_index(any_rw, 0);
    else 
        current_rw_core_id <= current_rw_core_id;
end
`ifdef VERILATOR
    wire [0 : CORE_NUMS-1] packed_wb_strobe;
    wire [0 : CORE_NUMS-1] packed_invalidate_strobe;
    wire [0 : CORE_NUMS-1] packed_rw_strobe;
    wire [0 : CORE_NUMS-1] packed_any_response;
    generate
        for(idx = 0; idx < CORE_NUMS; idx = idx + 1) begin
            assign packed_wb_strobe[idx]         = any_wb[idx];
            assign packed_invalidate_strobe[idx] = any_invalidate[idx];
            assign packed_rw_strobe[idx]         = any_rw[idx];
            assign packed_any_response[idx]      = any_response[idx];
        end
    endgenerate
`else
    wire [0 : CORE_NUMS-1] packed_wb_strobe         = {<<{any_wb[0 : CORE_NUMS-1]}};
    wire [0 : CORE_NUMS-1] packed_invalidate_strobe = {<<{any_invalidate[0 : CORE_NUMS-1]}};
    wire [0 : CORE_NUMS-1] packed_rw_strobe         = {<<{any_rw[0 : CORE_NUMS-1]}};
    wire [0 : CORE_NUMS-1] packed_any_response      = {<<{any_response[0 : CORE_NUMS-1]}};
`endif

assign have_wb         = | packed_wb_strobe;
assign have_invalidate = | packed_invalidate_strobe;
assign have_rw         = | packed_rw_strobe;
assign L1_has_response = | packed_any_response;
assign L1_response_core_id = find_next_index(response_data_ready, 0);
assign response_data_mux = response_data[L1_response_core_id];

always @(posedge clk_i)
begin
    if (rst_i) S <= S_Idle;
    else  S <= S_next;
end

always @(*)
begin
    case(S)
        S_Idle: begin
            if (L2_invalidate_L1_i) begin
                S_next = S_L2_Invalidate;
            end
            // write back event
            else  if (have_wb) begin
                S_next = S_Replacement_Delay;
            end
            // invalidate event
            else if (have_invalidate) begin
                S_next = S_Invalidate;
            end
            // memory request event
            else if (have_rw) begin
                S_next = S_Miss;
            end
            else begin
                S_next = S_Idle;
            end
        end
        S_Miss: S_next = S_Wait;
        S_Wait: S_next = S_Wait2;
        S_Wait2:  begin
            if (L1_has_response) begin
                if (read_write[current_rw_core_id]) begin
                    S_next = S_Delay;
                end
                else begin
                    S_next = S_WB_L2;
                end
            end
            else if (response_ready_L2) S_next = S_Delay;
            else S_next = S_Wait2;
        end
        S_Invalidate: S_next = S_Delay;
        S_Replacement_Delay: S_next = S_Replacement;
        S_Replacement: S_next = (response_ready_L2) ? S_Delay : S_Replacement;
        S_WB_L2: S_next = (response_ready_L2) ? S_Idle : S_WB_L2;
        S_L2_Invalidate: S_next = S_Idle;
        S_Delay: S_next = S_Idle;
        default: S_next = S_Idle;
    endcase
end

always @(posedge clk_i) begin
    if (rst_i) begin
        for(i = 0; i < CORE_NUMS; i = i + 1) begin
            Data[i] <= 0;
            request_data_ready[i] <= 0;
            make_exclusive_L1[i] <= 0;
        end
    end
    else if (S == S_Wait2) begin
        for(i = 0; i < CORE_NUMS; i = i + 1) begin
            if (i == current_rw_core_id) begin
                if (L1_has_response) begin
                    Data[i] <= response_data_mux;
                    request_data_ready[i] <= 1;
                    make_exclusive_L1[i] <= 0;
                end
                else if (response_ready_L2)begin
                    Data[i] <= response_data_L2;
                    request_data_ready[i] <= 1;
                    make_exclusive_L1[i] <= make_L1_exclusive_L2;
                end
                else begin
                    Data[i] <= 0;
                    request_data_ready[i] <= 0;
                    make_exclusive_L1[i] <= 0;
                end
            end
            else begin
                Data[i] <= 0;
                request_data_ready[i] <= 0;
                make_exclusive_L1[i] <= 0;
            end
        end
    end
    else if (S == S_Replacement && response_ready_L2) begin
        for(i = 0;i < CORE_NUMS; i = i + 1) begin
            if (i == current_wb_core_id) begin
                Data[i] <= 0;
                request_data_ready[i] <= 1;
                make_exclusive_L1[i] <= 0;
            end
            else begin
                Data[i] <= 0;
                request_data_ready[i] <= 0;
                make_exclusive_L1[i] <= 0;
            end
        end
    end
    else if (S == S_Invalidate) begin
        for(i = 0;i < CORE_NUMS; i = i + 1) begin
            if (i == current_invalidate_core_id) begin
                Data[i] <= 0;
                request_data_ready[i] <= 1;
                make_exclusive_L1[i] <= 0;
            end
            else begin
                Data[i] <= 0;
                request_data_ready[i] <= 0;
                make_exclusive_L1[i] <= 0;
            end
        end
    end
    else begin
        for(i = 0; i < CORE_NUMS; i = i + 1) begin
            Data[i] <= 0;
            request_data_ready[i] <= 0;
            make_exclusive_L1[i] <= 0;
        end
    end
end

always @(posedge clk_i) begin
    if (rst_i) begin
        for(i = 0; i < CORE_NUMS; i = i + 1) begin
            probe_strobe[i] <= 0;
            probe_data_address[i] <= 0;
            invalidate[i] <= 0;
        end
    end
    else if (S == S_Invalidate) begin
        for(i = 0; i < CORE_NUMS; i = i + 1) begin
            probe_strobe[i] <= i != current_invalidate_core_id;
            probe_data_address[i] <= data_address[current_invalidate_core_id];
            invalidate[i] <= i != current_invalidate_core_id;
        end
    end
    else if (S == S_L2_Invalidate) begin
        for(i = 0; i < CORE_NUMS; i = i + 1) begin
            probe_strobe[i] <= 1;
            probe_data_address[i] <= L2_invalidate_L1_addr_i;
            invalidate[i] <= 1;
        end
    end
    else if (S == S_Miss) begin
        for(i = 0; i < CORE_NUMS; i = i + 1) begin
            probe_strobe[i] <= i != current_rw_core_id;
            probe_data_address[i] <= data_address[current_rw_core_id];
            invalidate[i] <= read_write[current_rw_core_id];
        end
    end
    else begin
        for(i = 0; i < CORE_NUMS; i = i + 1) begin
            probe_strobe[i] <= 0;
            probe_data_address[i] <= 0;
            invalidate[i] <= 0;
        end
    end
end

always@(posedge clk_i) begin
    if (rst_i) begin
        wb_addr_r <= 0;
        wb_data_r <= 0;
    end
    else if (response_ready_L2) begin
        wb_addr_r <= 0;
        wb_data_r <= 0;
    end
    else if (S == S_Wait2 && S_next == S_WB_L2) begin
        wb_addr_r <= data_address[current_rw_core_id];
        wb_data_r <= response_data_mux;
    end
    else if (S == S_Replacement_Delay) begin
        wb_addr_r <= data_address[current_wb_core_id];
        wb_data_r <= write_back_data[current_wb_core_id];
    end
    else begin
        wb_addr_r <= wb_addr_r;
        wb_data_r <= wb_data_r;
    end
end
// write_to_L2, data_to_L2, write_data_address_L2
always@(posedge clk_i)begin
    if (rst_i)begin
        write_to_L2 <= 0;
        data_to_L2 <= 0;
        replacement_L2 <= 0;
        write_data_address_L2 <= 0;
    end
    else if (S == S_WB_L2 && !response_ready_L2)begin
        write_to_L2 <= 1;
        write_data_address_L2 <= wb_addr_r;
        data_to_L2 <= wb_data_r;
        replacement_L2 <= 0;
    end
    else if (S == S_Replacement && !response_ready_L2)begin
        write_to_L2 <= 1;
        write_data_address_L2 <= wb_addr_r;
        data_to_L2 <= wb_data_r;
        replacement_L2 <= 1;
    end
    else begin
        write_to_L2 <= 0;
        write_data_address_L2 <= 0;
        data_to_L2 <= 0;
        replacement_L2 <= 0;
    end
end
// probe_strobe_L2, probe_data_address_L2, L1_iswrite_L2
always@(posedge clk_i)begin
    if (rst_i)begin
        probe_strobe_L2 <= 0;
        probe_data_address_L2 <= 0;
        L1_iswrite_L2 <= 0; 
        L1_is_instr_fetch_L2 <= 0;
    end
    else if (S == S_Wait2 && !L1_has_response && !response_ready_L2) begin
        probe_strobe_L2 <= 1;
        probe_data_address_L2 <= data_address[current_rw_core_id];
        L1_iswrite_L2 <= read_write[current_rw_core_id];
        L1_is_instr_fetch_L2 <= L1_is_instr_fetch[current_rw_core_id];
    end
    else begin
        probe_strobe_L2 <= 0;
        probe_data_address_L2 <= 0;
        L1_iswrite_L2 <= 0;
        L1_is_instr_fetch_L2 <= 0;
    end
end

//invalidate_address_L2, invalidate_L2

always@(posedge clk_i)begin
    if (rst_i)begin
        invalidate_address_L2 <= 0;
        invalidate_L2 <= 0;
    end
    else if (S == S_Invalidate)begin
        invalidate_address_L2 <= data_address[current_invalidate_core_id];
        invalidate_L2 <= 1;
    end
    else begin
        invalidate_address_L2 <= 0;
        invalidate_L2 <= 0;
    end
end

// address to L2 multiplexer
always@(*)begin
    if (invalidate_L2)address_to_L2 = invalidate_address_L2;
    else if (probe_strobe_L2) address_to_L2 = probe_data_address_L2;
    else if (write_to_L2) address_to_L2 = write_data_address_L2;
    else  address_to_L2 = 0;
end

assign CU_L2_ready_o = S == S_L2_Invalidate;

endmodule
