`timescale 1ns / 1ps
// =============================================================================
//  Program : amo_arbiter.v
//  Author  : Lin-en Yen
//  Date    : Feb/25/2024
// -----------------------------------------------------------------------------
//  Description:
//      The amo arbiter for a multi-core system.
// -----------------------------------------------------------------------------
//  Revision information:
//
//  Feb/10/2025, by Ye Chen:
//    Extended support to four cores.
//
//  Apr/10/2025, by Tzu-Chen Yang:
//    Extended support to 2, 4, 8, and 16 cores.
//
//  Aug/7/2025, by Tzu-Chen Yang:
//    Changed the arbitration method to Round Robin.
//
//  Sept/14/2025, by Chun-Jen Tsai:
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

module amo_arbiter #(
    parameter XLEN   = 32,  // Width of address bus
    parameter CLSIZE = `CLP, // Size of a cache line in bits.
    parameter CORE_NUMS = `CORE_NUMS,
    parameter CORE_NUMS_BITS = (`CORE_NUMS==1) ? 1 : $clog2(`CORE_NUMS)
)
(
    // System signals
    input                       clk_i, rst_i,

    // Aquila core 0 interface
    input                       P_strobe_i[0 : CORE_NUMS-1],
    input [XLEN-1 : 0]          P_addr_i[0 : CORE_NUMS-1],
    input                       P_rw_i[0 : CORE_NUMS-1],
    input [XLEN-1 : 0]          P_data_i[0 : CORE_NUMS-1],
    output reg                  P_data_ready_o[0 : CORE_NUMS-1],
    output reg [XLEN-1 : 0]     P_data_o[0 : CORE_NUMS-1],
    input                       P_is_amo_i[0 : CORE_NUMS-1],
    input [ 4 : 0]              P_amo_type_i[0 : CORE_NUMS-1],

    // Chosen atomic intruction
    output [CORE_NUMS_BITS-1 : 0]             AMO_id_o,
    output                      AMO_strobe_o,
    output [XLEN-1 : 0]         AMO_addr_o,
    output                      AMO_rw_o,
    output [XLEN-1 : 0]         AMO_data_o,
    input                       AMO_data_ready_i,
    input [XLEN-1 : 0]          AMO_data_i,
    output                      AMO_is_amo_o,
    output [ 4 : 0]             AMO_amo_type_o
);

    localparam M_IDLE   = 0, // wait for strobe
               M_CHOOSE = 1, // choose 
               M_WAIT   = 2; // wait for device ready for request

    // input selection signals
    wire                     concurrent_strobe;
    reg  [CORE_NUMS_BITS-1 : 0]             sel_current;
    reg  [CORE_NUMS_BITS-1 : 0]             sel_previous;
 
    reg                      AMO_strobe_r;
    reg  [XLEN-1 : 0]        AMO_addr_r;
    reg                      AMO_rw_r;
    reg  [XLEN-1 : 0]        AMO_data_r;
    reg                      AMO_is_amo_r;
    reg  [ 4 : 0]            AMO_amo_type_r;

    // Keep the strobe
    reg                      P_strobe_r[0 : CORE_NUMS-1];
    reg  [XLEN-1 : 0]        P_addr_r[0 : CORE_NUMS-1];
    reg                      P_rw_r[0 : CORE_NUMS-1];
    reg  [XLEN-1 : 0]        P_data_r[0 : CORE_NUMS-1];
    reg                      P_is_amo_r[0 : CORE_NUMS-1];
    reg  [ 4 : 0]            P_amo_type_r[0 : CORE_NUMS-1];
    
    // FSM signals
     reg  [1 : 0]             c_state;
    reg  [1 : 0]             n_state;
    wire                     have_strobe;
    integer i;
    //=======================================================
    //  Keep the strobe (in case we miss strobe)
    //=======================================================
    always @(posedge clk_i) begin
        if (rst_i) begin
            for(i = 0;i < `CORE_NUMS;i = i + 1) begin
                P_strobe_r[i] <= 0;
            end
        end
        else begin
            for(i = 0;i < `CORE_NUMS;i = i + 1) begin
                if (P_strobe_i[i] && P_is_amo_i[i])
                    P_strobe_r[i] <= 1;
                else if (P_data_ready_o[i])
                    P_strobe_r[i] <= 0; // Clear the strobe
            end
        end
    end

    always @(posedge clk_i) begin
        if (rst_i) begin
            for(i = 0;i < `CORE_NUMS;i = i + 1) begin
                P_is_amo_r[i] <= 0;
            end
        end
        else begin
            for(i = 0;i < `CORE_NUMS;i = i + 1) begin
                if (P_strobe_i[i])
                    P_is_amo_r[i] <= P_is_amo_i[i];
                else if (P_data_ready_o[i])
                    P_is_amo_r[i] <= 0; // Clear the strobe
            end
        end
    end

   always @(posedge clk_i) begin
        if (rst_i) begin
            for(i = 0;i < `CORE_NUMS;i = i + 1) begin
                P_addr_r[i]      <= 0;
                P_rw_r[i]        <= 0;
                P_data_r[i]      <= 0;
                P_amo_type_r[i]  <= 0;
            end
        end
        else begin
            for(i = 0;i < `CORE_NUMS;i = i + 1) begin
                if (P_strobe_i[i]) begin
                    P_addr_r[i]      <= P_addr_i[i];
                    P_rw_r[i]        <= P_rw_i[i];
                    P_data_r[i]      <= P_data_i[i];
                    P_amo_type_r[i]  <= P_amo_type_i[i];
                end
            end
        end
    end

    //=======================================================
    //  Strobe signals selection (Round Robin) 
    //=======================================================

    always @(posedge clk_i) begin
        if (rst_i) 
            sel_previous <= CORE_NUMS-1;
        else if (c_state == M_CHOOSE)
            sel_previous <= sel_current;
    end

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
            sel_current <= CORE_NUMS-1;
        else if (c_state == M_IDLE)
            sel_current <= find_next_index(P_strobe_r, (sel_previous+1)%CORE_NUMS);
    end

    /* Record selected singnals*/
    always @(posedge clk_i) begin
        if (rst_i) 
            AMO_strobe_r <= 0;
        else if (c_state == M_CHOOSE) 
            AMO_strobe_r <= 1;
        else    
            AMO_strobe_r <= 0;
    end
    
    always @(posedge clk_i) begin
        if (rst_i) begin
            AMO_addr_r     <= 0;
            AMO_rw_r       <= 0;
            AMO_data_r     <= 0;
            AMO_is_amo_r   <= 0;
            AMO_amo_type_r <= 0;
        end else begin
            AMO_addr_r <= P_addr_r[sel_current];
            AMO_rw_r   <= P_rw_r[sel_current];
            AMO_data_r <= P_data_r[sel_current];
            AMO_is_amo_r <= P_is_amo_r[sel_current];
            AMO_amo_type_r <= P_amo_type_r[sel_current];
        end
    end

    //=======================================================
    //  Output logic
    //=======================================================
    always @(*) begin
        for(i = 0;i < `CORE_NUMS;i = i + 1) begin
            P_data_ready_o[i] = (sel_current == i && c_state == M_WAIT) ? AMO_data_ready_i : 'b0;
            P_data_o[i]       = AMO_data_i;
        end
    end

    assign AMO_id_o            = sel_current;

    assign AMO_strobe_o        = AMO_strobe_r;
    assign AMO_addr_o          = AMO_addr_r;
    assign AMO_rw_o            = AMO_rw_r;
    assign AMO_data_o          = AMO_data_r;
    assign AMO_is_amo_o        = AMO_is_amo_r;
    assign AMO_amo_type_o      = AMO_amo_type_r;

    //=======================================================
    //  Main FSM
    //=======================================================
    always @(posedge clk_i) begin
        if (rst_i)
            c_state <= M_IDLE;
        else
            c_state <= n_state;
    end

    always @(*) begin
        case (c_state)
            M_IDLE: 
                if (have_strobe) 
                    n_state = M_CHOOSE;
                else 
                    n_state = M_IDLE;
            M_CHOOSE:
                n_state = M_WAIT;
            M_WAIT:
                if (AMO_data_ready_i)
                    n_state = M_IDLE;
                else
                    n_state = M_WAIT;
        endcase
    end

    // Convert the unpacked P_strobe_r[] to a packed bit arrary packed_strobe.
    `ifdef VERILATOR
        wire [0 : CORE_NUMS-1] packed_strobe;
        genvar idx;
        generate
            for(idx = 0;idx < `CORE_NUMS;idx = idx + 1) begin
                assign packed_strobe[i] = P_strobe_r[i];
            end
        endgenerate

    `else
    wire [0 : CORE_NUMS-1] packed_strobe = {<<{P_strobe_r[0 : CORE_NUMS-1]}};
    `endif
    assign have_strobe = | packed_strobe;

endmodule
