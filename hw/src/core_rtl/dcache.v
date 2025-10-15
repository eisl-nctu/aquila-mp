`timescale 1ns / 1ps
// =============================================================================
//  Program : dcache.v
//  Author  : Jin-you Wu
//  Date    : Nov/01/2018
// -----------------------------------------------------------------------------
//  Description:
//  This module implements the L1 Data Cache with the following
//  properties:
//      4-way set associative
//      FIFO replacement policy
//      Write-back
//      Write allocate
// -----------------------------------------------------------------------------
//  Revision information:
//
//  Mar/03/2020, by Chih-Yu Hsiang:
//    Added AMO support.
//
//  Sep/24/2023, by Chun-Jen Tsai:
//    Modify the code to use distributed RAM to store VALID and DIRTY bits.
//    This modification significantly reduces the resource usage.
//
//  Feb/12/2025, by Tzu-Chen Yang:
//    Modified to implement the standard MESI protocol state transition method.
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

module dcache
#(
  parameter XLEN = 32,
  parameter CACHE_SIZE = 64,
  parameter CLSIZE = `CLP    // Cache line size.
)
(
    /////////// System signals   ///////////////////////////////////////////////
    input                     clk_i, rst_i,

    /////////// Processor signals //////////////////////////////////////////////
    input                     p_strobe_i,      // Processor request signal.
    input                     p_rw_i,          // 0 for read, 1 for write.
    input  [XLEN/8-1 : 0]     p_byte_enable_i, // Byte-enable signal.
    input  [XLEN-1 : 0]       p_addr_i,        // Memory addr of the request.
    input  [XLEN-1 : 0]       p_data_i,        // Data to main memory.
    output reg [XLEN-1 : 0]   p_data_o,        // Data from main memory.
    output                    p_ready_o,       // The cache data is ready.
    input                     p_flush_i,       // Cache flush request.,
    output     reg            busy_flushing_o, // Cache is flushing.
    input                     p_is_amo_i,      // AMO request from core.

    /////////// Cache coherence signals  ///////////////////////////////////////
    input                     probe_strobe_i,
    input                     invalidate_i,          
    input  [XLEN-1 : 0]       probe_addr_i,  
    input  [CLSIZE-1 : 0]     coherence_data_i,
    input                     coherence_done_i, 
    input                     make_exclusive_i,
    input                     probe_is_amo_i,   

    output [CLSIZE-1 : 0]     response_data_o,
    output [CLSIZE-1 : 0]     write_back_data_o,
    output [XLEN-1 : 0]       coherence_addr_o,
    output                    coherence_strobe_o,
    output                    coherence_replacement_o,
    output                    coherence_rw_o,
    output                    share_modify_o,
    output                    response_ready_o

`ifdef PROFILE_DCACHE
    ,
    input                     profile_req,
    input [XLEN-1:0]          profile_addr,
    output reg [XLEN-1:0]         profile_data,
    output reg                   profile_ready    
`endif
);
//=======================================================
// Cache Finite State Machine
//=======================================================
localparam Init             = 0,
           Idle             = 1,
           Analysis         = 2,
           WriteHitShare    = 3,
           WbtoMem          = 4,
           RdfromMem        = 5,
           WbtoMemAll       = 7,
           WbtoMemAllFinish = 8,
           WaitForAmo       = 9,
           Delay           = 10;

// Cache controller state registers
reg [ 3 : 0] S, S_nxt;

reg  [ 1 : 0]          PROBE_S, PROBE_S_nxt;

localparam PROBE_Idle       = 0,
           PROBE_Analysis   = 1;
//=======================================================
// Cache parameters
//=======================================================
localparam N_WAYS      = 2;
`define WAYS_2
// `define WAYS_4 
// `define WAYS_8
localparam N_LINES     = (CACHE_SIZE*1024*8) / (N_WAYS*CLSIZE);

localparam WAY_BITS    = $clog2(N_WAYS);
localparam BYTE_BITS   = $clog2(XLEN/8);
localparam WORD_BITS   = $clog2(CLSIZE/XLEN);
localparam LINE_BITS   = $clog2(N_LINES);
localparam NONTAG_BITS = LINE_BITS + WORD_BITS + BYTE_BITS;
localparam TAG_BITS    = XLEN - NONTAG_BITS;

//=======================================================
// N-way associative cache signals
//=======================================================
wire                   way_hit[0 : N_WAYS-1];     // Cache-way hit flag.
reg  [WAY_BITS-1 : 0]  hit_index;                 // Decoded way_hit[] signal.
wire                   cache_hit;                 // Got a cache hit?
reg  [CLSIZE-1 : 0]    c_data_i;                  // Data to write into cache.
reg  [CLSIZE-1 : 0]    c_data_update;             // Updated cache data.
reg  [CLSIZE-1 : 0]    m_data_update;             // Updated memory data.
wire [CLSIZE-1 : 0]    c_block[0 : N_WAYS-1];     // Cache blocks from N cache way.
wire [CLSIZE-1 : 0]    c_data_hit;                // Data from the hit cache block.
reg                    cache_write[0 : N_WAYS-1]; // WE signal for a $ tag & block.
reg                    valid_write[0 : N_WAYS-1]; // WE signal for a $ valid bit.
reg                    dirty_write[0 : N_WAYS-1]; // WE signal for a $ dirty bit.
reg                    share_write[0 : N_WAYS-1]; // WE signal for a $ share bit.

wire [TAG_BITS-1 : 0]  c_tag_o[0 : N_WAYS-1];     // Tag bits of current $ blocks.
wire                   c_valid_o[0 : N_WAYS-1];   // Validity of current $ blocks.
wire                   c_dirty_o[0 : N_WAYS-1];   // Dirtiness of current $ blocks.
wire                   c_share_o[0 : N_WAYS-1];   // share of current $ blocks.

reg  [LINE_BITS-1 : 0] init_count;                // Counter to initialize valid bits.

integer idx;
//=======================================================
// FIFO replace policy signals
//=======================================================
reg  [WAY_BITS-1 : 0] FIFO_cnt[0 : N_LINES-1];   // Replace policy counter.
reg  [WAY_BITS-1 : 0] victim_sel;                // The victim cache select.

//=======================================================
// Cache line and tag calculations
//=======================================================
wire [WORD_BITS-1 : 0] line_offset;
wire [LINE_BITS-1 : 0] line_index;
wire [TAG_BITS-1  : 0] tag;
wire [LINE_BITS-1 : 0] addr_sram;
// Control signals for flushing all cache blocks
//=======================================================
reg [LINE_BITS-1 : 0] N_LINES_cnt;
reg [WAY_BITS-1 : 0]  N_WAYS_cnt;
wire NeedtoWb = c_valid_o[N_WAYS_cnt] && !c_share_o[N_WAYS_cnt];
wire WbAllFinish = (N_LINES_cnt == N_LINES - 1 && N_WAYS_cnt == N_WAYS - 1);
reg WbAllFinish_r;
wire probe_on_wb_line;
wire probe_on_share_modify_line;
wire probe_on_curr_line;
wire probe_case1, probe_case2;
wire probe_addr_match_curr_line;
//=======================================================
// Cache Coherence through Broadcast
//=======================================================
wire [CLSIZE-1 : 0]    probe_block[0 : N_WAYS-1];     // Cache blocks from N cache way.
wire [TAG_BITS-1 : 0]  probe_tag_o[0 : N_WAYS-1];     // Tag bits of current $ blocks.
wire                   probe_valid_o[0 : N_WAYS-1];   // Validity of current $ blocks.
wire                   probe_dirty_o[0 : N_WAYS-1];   // Dirtiness of current $ blocks.
wire                   probe_share_o[0 : N_WAYS-1];   // Share of current $ blocks.

reg                    probe_valid_write[0 : N_WAYS-1]; // WE signal for a $ valid bit.
reg                    probe_dirty_write[0 : N_WAYS-1]; // WE signal for a $ dirty bit.
reg                    probe_share_write[0 : N_WAYS-1]; // WE signal for a $ share bit.

wire [TAG_BITS-1  : 0] probe_tag;

wire                   probe_way_hit[0 : N_WAYS-1];     // Cache-way hit flag.
reg  [WAY_BITS-1 : 0]  probe_hit_index;                 // Decoded way_hit[] signal.
wire                   probe_cache_hit;                 // Got a cache hit (valid)      
wire                   probe_cache_dirty;               // Probe cache line is dirty.
wire                   probe_shared;                    // Probe cache line is shared.
wire [LINE_BITS-1 : 0] probe_line_index;

// register invalidate signal
reg [XLEN-1:0] probe_addr_r;
reg invalidate_r;
reg probe_strobe_r;
//register memory request
reg  p_strobe_r;
reg  p_rw_r;
reg  [XLEN/8-1 : 0] p_byte_enable_r;
reg  [XLEN-1 : 0] p_addr_r;
reg  [XLEN-1 : 0] p_data_r;
//-----------------------------------------------
// Read a 32-bit word from the target cache line
//-----------------------------------------------
reg [XLEN-1 : 0] fromCache; // Get the specific word in cache line
reg [XLEN-1 : 0] fromMem;   // Get the specific word in memory line
//------------------------------------------------------------------------
// Write the correct bytes according to the signal p_byte_enable_r
//------------------------------------------------------------------------
reg [XLEN-1 : 0] update_data;
//=======================================================
//  Valid bits storage in distributed RAM
//=======================================================
wire valid_data;
wire [LINE_BITS-1 : 0] valid_write_addr;
//=======================================================
//  Dirty bits storage in distributed RAM 
//=======================================================
wire dirty_data;
wire [LINE_BITS-1 : 0] dirty_waddr;
//=======================================================
//  Share bits storage in distributed RAM
//=======================================================
wire share_data;
wire [LINE_BITS-1 : 0] share_waddr;

assign c_data_hit = c_block[hit_index];

assign line_offset = p_strobe_i ?  p_addr_i[WORD_BITS + BYTE_BITS - 1 : BYTE_BITS]
                                 :  p_addr_r[WORD_BITS + BYTE_BITS - 1 : BYTE_BITS];
assign line_index  = p_strobe_i ? p_addr_i[NONTAG_BITS - 1 : WORD_BITS + BYTE_BITS] 
                                : p_addr_r[NONTAG_BITS - 1 : WORD_BITS + BYTE_BITS];
assign tag         = p_strobe_i ? p_addr_i[XLEN - 1 : NONTAG_BITS]
                                : p_addr_r[XLEN - 1 : NONTAG_BITS];


//====================================================
// Cache Controller FSM
//====================================================
always @(posedge clk_i)
begin
    if (rst_i)
        S <= Init;
    else
        S <= S_nxt;
end

always @(*)
begin
    case (S)
        Init: // Multi-cycle initialization of the VALID bits memory.
            if (init_count < N_LINES - 1)
                S_nxt = Init;
            else
                S_nxt = Idle;
        Idle:
            if (!p_is_amo_i && probe_is_amo_i)
                S_nxt = WaitForAmo;// Wait for other amo
            else if ((p_strobe_i || p_strobe_r || p_flush_i || busy_flushing_o) && PROBE_S != PROBE_Analysis)
                S_nxt = Analysis;
            else
                S_nxt = Idle;
        Analysis:
        if (probe_on_curr_line) begin
            S_nxt = Delay;
        end else begin
            if (busy_flushing_o)
                S_nxt = WbtoMemAll;
            else if (!cache_hit)
                // victim cache line is M/E state -> Replacement
                S_nxt = (c_valid_o[victim_sel] && !c_share_o[victim_sel])? WbtoMem : RdfromMem;
            // write hit on share state -> Invalidate remote cache
            else if (cache_hit && p_rw_r && c_share_o[hit_index])
                S_nxt = WriteHitShare;
            // hit on other state (silent)
            else
                S_nxt = Idle;
        end
        WriteHitShare:
            if (probe_on_share_modify_line) S_nxt = Delay;
            else if (coherence_done_i) S_nxt = Idle;
            else S_nxt = WriteHitShare;
        Delay:
            S_nxt = Idle;
        WbtoMem:
        // wb to L2 cache finish -> Broadcasts
            if (probe_on_wb_line) S_nxt = RdfromMem;
            else if (coherence_done_i) begin
                if (busy_flushing_o)  S_nxt = WbtoMemAllFinish;
                else                 S_nxt = RdfromMem;
            end
            else S_nxt = WbtoMem;
        RdfromMem:
        // get response from remote cache -> Idle
            if (coherence_done_i)
                S_nxt = Idle;
            else
                S_nxt = RdfromMem;
        WbtoMemAll:
            if (NeedtoWb)
                S_nxt = WbtoMem;
            else
                S_nxt = WbtoMemAllFinish;
        WbtoMemAllFinish:
            S_nxt = (WbAllFinish_r)? Idle : WbtoMemAll;
        WaitForAmo:
            if (!probe_is_amo_i)
                S_nxt = Idle;
            else
                S_nxt = WaitForAmo;
        default:
            S_nxt = Idle;
    endcase
end
//====================================================
// Cache Coherence FSM
//====================================================

always @(posedge clk_i)
begin
    if (rst_i)
        PROBE_S <= PROBE_Idle;
    else
        PROBE_S <= PROBE_S_nxt;
end

always @(*)
begin
    case (PROBE_S)
        PROBE_Idle: 
        //remote cache get a strobe (probe from cache coherence unit)
            if (probe_strobe_i)
                PROBE_S_nxt = PROBE_Analysis;
            else
                PROBE_S_nxt = PROBE_Idle;
        PROBE_Analysis:
            PROBE_S_nxt = PROBE_Idle;
        default:
            PROBE_S_nxt = PROBE_Idle;
    endcase
end

//=======================================================
// Cache Coherence through Broadcast
//=======================================================

// probe_cache_hit + probe_cache_dirty -> M
// probe_cache_hit + !probe_cache_dirty -> E
// probe_cache_hit + probe_shared -> S
// !probe_cache_hit -> I

assign probe_line_index  = (probe_strobe_r) ?  probe_addr_r[NONTAG_BITS - 1 : WORD_BITS + BYTE_BITS] : 
                                                             probe_addr_i[NONTAG_BITS - 1 : WORD_BITS + BYTE_BITS];
                                                             
assign probe_tag     = (probe_strobe_r) ?  probe_addr_r[XLEN - 1 : NONTAG_BITS] : probe_addr_i[XLEN - 1 : NONTAG_BITS];

// Check and see if any cache way has the matched memory block.
`ifdef WAYS_2
assign probe_way_hit[0] = (probe_valid_o[0] && (probe_tag_o[0] == probe_tag))? 1 : 0;
assign probe_way_hit[1] = (probe_valid_o[1] && (probe_tag_o[1] == probe_tag))? 1 : 0;
always @(*) begin
    case ( { probe_way_hit[0], probe_way_hit[1]} )
        2'b10: probe_hit_index = 0;
        2'b01: probe_hit_index = 1;
        default: probe_hit_index = 0; // error: multiple-way hit!
    endcase
end
`elsif WAYS_4
assign probe_way_hit[0] = (probe_valid_o[0] && (probe_tag_o[0] == probe_tag))? 1 : 0;
assign probe_way_hit[1] = (probe_valid_o[1] && (probe_tag_o[1] == probe_tag))? 1 : 0;
assign probe_way_hit[2] = (probe_valid_o[2] && (probe_tag_o[2] == probe_tag))? 1 : 0;
assign probe_way_hit[3] = (probe_valid_o[3] && (probe_tag_o[3] == probe_tag))? 1 : 0;
always @(*) begin
    case ( { probe_way_hit[0], probe_way_hit[1], probe_way_hit[2], probe_way_hit[3] } )
        4'b1000: probe_hit_index = 0;
        4'b0100: probe_hit_index = 1;
        4'b0010: probe_hit_index = 2;
        4'b0001: probe_hit_index = 3;
        default: probe_hit_index = 0; // error: multiple-way hit!
    endcase
end
`else //8 ways
assign probe_way_hit[0] = (probe_valid_o[0] && (probe_tag_o[0] == probe_tag)) ? 1 : 0;
assign probe_way_hit[1] = (probe_valid_o[1] && (probe_tag_o[1] == probe_tag)) ? 1 : 0;
assign probe_way_hit[2] = (probe_valid_o[2] && (probe_tag_o[2] == probe_tag)) ? 1 : 0;
assign probe_way_hit[3] = (probe_valid_o[3] && (probe_tag_o[3] == probe_tag)) ? 1 : 0;
assign probe_way_hit[4] = (probe_valid_o[4] && (probe_tag_o[4] == probe_tag)) ? 1 : 0;
assign probe_way_hit[5] = (probe_valid_o[5] && (probe_tag_o[5] == probe_tag)) ? 1 : 0;
assign probe_way_hit[6] = (probe_valid_o[6] && (probe_tag_o[6] == probe_tag)) ? 1 : 0;
assign probe_way_hit[7] = (probe_valid_o[7] && (probe_tag_o[7] == probe_tag)) ? 1 : 0;
always @(*) begin
    case ( { probe_way_hit[0], probe_way_hit[1], probe_way_hit[2], probe_way_hit[3], 
              probe_way_hit[4], probe_way_hit[5], probe_way_hit[6], probe_way_hit[7] } )
        8'b10000000: probe_hit_index = 0;
        8'b01000000: probe_hit_index = 1;
        8'b00100000: probe_hit_index = 2;
        8'b00010000: probe_hit_index = 3;
        8'b00001000: probe_hit_index = 4;
        8'b00000100: probe_hit_index = 5;
        8'b00000010: probe_hit_index = 6;
        8'b00000001: probe_hit_index = 7;
        default: probe_hit_index = 0; // error: multiple-way hit!
    endcase
end
`endif

`ifdef WAYS_2
assign probe_cache_hit  = (probe_way_hit[0] || probe_way_hit[1]);
`elsif WAYS_4
assign probe_cache_hit  = (probe_way_hit[0] || probe_way_hit[1] || probe_way_hit[2] || probe_way_hit[3]);
`else //8 ways
assign probe_cache_hit  = (probe_way_hit[0] || probe_way_hit[1] || probe_way_hit[2] || probe_way_hit[3] ||
                      probe_way_hit[4] || probe_way_hit[5] || probe_way_hit[6] || probe_way_hit[7]);
`endif

// assign probe_cache_hit  = (probe_way_hit[0] || probe_way_hit[1] || probe_way_hit[2] || probe_way_hit[3]);

assign probe_cache_dirty = probe_dirty_o[probe_hit_index];
assign probe_shared       = probe_share_o[probe_hit_index];

assign response_ready_o = (PROBE_S == PROBE_Analysis && probe_cache_hit && !probe_shared);

assign probe_on_wb_line = (response_ready_o && S == WbtoMem 
                            && {c_tag_o[victim_sel], line_index, {WORD_BITS{1'b0}}, 2'b0} == {probe_tag, probe_line_index, {WORD_BITS{1'b0}}, 2'b0});

assign probe_case1 = PROBE_S == PROBE_Analysis && probe_cache_hit;
assign probe_case2 = PROBE_S == PROBE_Idle && probe_strobe_i;
assign probe_addr_match_curr_line = {p_addr_r[XLEN-1 : WORD_BITS+2], {WORD_BITS{1'b0}}, 2'b0} == {probe_tag, probe_line_index, {WORD_BITS{1'b0}}, 2'b0};
assign probe_on_curr_line = (probe_case1 || probe_case2) && probe_addr_match_curr_line;
assign probe_on_share_modify_line = ((probe_case1 && invalidate_r) || (probe_case2 && invalidate_i)) && probe_addr_match_curr_line && S == WriteHitShare;

// assign probe_on_share_modify_line = (PROBE_S == PROBE_Analysis && probe_cache_hit && probe_shared && invalidate_r) && S == WriteHitShare
//                             && {p_addr_r[XLEN-1 : WORD_BITS+2], {WORD_BITS{1'b0}}, 2'b0} == {probe_tag, probe_line_index, {WORD_BITS{1'b0}}, 2'b0};

// register invalidate signal
always @(posedge clk_i)
begin
    if (rst_i) begin
        invalidate_r <= 0;
        probe_strobe_r <= 0;
        probe_addr_r <= 0;
    end
    else if (probe_strobe_i) begin
        probe_strobe_r <= 1;
        invalidate_r <= (invalidate_i) ? 1 : 0;
        probe_addr_r <= probe_addr_i;
    end
    else begin
        invalidate_r <= 0;
        probe_strobe_r <= 0;
        probe_addr_r <= 0;
    end
end

//register memory request
always @(posedge clk_i ) begin
    if (rst_i) begin
        p_strobe_r <= 0;
        p_rw_r <= 0;
        p_byte_enable_r <= 0;
        p_addr_r <= 0;
        p_data_r <= 0;
    end
    else if (p_strobe_i) begin
        p_strobe_r <= 1;
        p_rw_r <= p_rw_i;
        p_byte_enable_r <= p_byte_enable_i;
        p_addr_r <= p_addr_i;
        p_data_r <= p_data_i;
    end
    else if (p_ready_o) begin
        p_strobe_r <= 0;
        p_rw_r <= 0;
        p_byte_enable_r <= 0;
        p_addr_r <= 0;
        p_data_r <= 0;
    end
    
end

//*** Control writing bits for cache memory using probe data. ***//

//valid bit
always @(*) begin
    if (PROBE_S == PROBE_Analysis && invalidate_r && probe_cache_hit) // MESI: M/E/S -> I
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            probe_valid_write[idx] = probe_way_hit[idx];
    else
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            probe_valid_write[idx] = 1'b0;
end

always @(*) begin
    if (PROBE_S == PROBE_Analysis && probe_cache_hit) 
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            probe_dirty_write[idx] = probe_way_hit[idx]; // MESI: M/E -> M  or M -> S
    else
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            probe_dirty_write[idx] = 1'b0;
end

always @(*) begin
     // MESI: M/E -> S
    if (PROBE_S == PROBE_Analysis && (probe_cache_hit && !probe_shared))
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            probe_share_write[idx] = probe_way_hit[idx];
    else
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            probe_share_write[idx] = 1'b0;
end

// Initialization of the valid bits to zeros upon reset.
always @ (posedge clk_i)
begin
    if (S == Init)
        init_count <= init_count + 1;
    else
        init_count <= {LINE_BITS{1'b0}};
end

// Check and see if any cache way has the matched memory block.
`ifdef WAYS_2
assign way_hit[0] = (c_valid_o[0] && (c_tag_o[0] == tag))? 1 : 0;
assign way_hit[1] = (c_valid_o[1] && (c_tag_o[1] == tag))? 1 : 0;

assign cache_hit  = (way_hit[0] || way_hit[1]);

always @(*)
begin
    case ( { way_hit[0], way_hit[1]} )
        2'b10: hit_index = 0;
        2'b01: hit_index = 1;
        default: hit_index = 0; // error: multiple-way hit!
    endcase
end
`elsif WAYS_4
assign way_hit[0] = (c_valid_o[0] && (c_tag_o[0] == tag))? 1 : 0;
assign way_hit[1] = (c_valid_o[1] && (c_tag_o[1] == tag))? 1 : 0;
assign way_hit[2] = (c_valid_o[2] && (c_tag_o[2] == tag))? 1 : 0;
assign way_hit[3] = (c_valid_o[3] && (c_tag_o[3] == tag))? 1 : 0;

assign cache_hit  = (way_hit[0] || way_hit[1] || way_hit[2] || way_hit[3]);

always @(*)
begin
    case ( { way_hit[0], way_hit[1], way_hit[2], way_hit[3] } )
        4'b1000: hit_index = 0;
        4'b0100: hit_index = 1;
        4'b0010: hit_index = 2;
        4'b0001: hit_index = 3;
        default: hit_index = 0; // error: multiple-way hit!
    endcase
end
`else // WAYS_8
assign way_hit[0] = (c_valid_o[0] && (c_tag_o[0] == tag))? 1 : 0;
assign way_hit[1] = (c_valid_o[1] && (c_tag_o[1] == tag))? 1 : 0;
assign way_hit[2] = (c_valid_o[2] && (c_tag_o[2] == tag))? 1 : 0;
assign way_hit[3] = (c_valid_o[3] && (c_tag_o[3] == tag))? 1 : 0;
assign way_hit[4] = (c_valid_o[4] && (c_tag_o[4] == tag))? 1 : 0;
assign way_hit[5] = (c_valid_o[5] && (c_tag_o[5] == tag))? 1 : 0;
assign way_hit[6] = (c_valid_o[6] && (c_tag_o[6] == tag))? 1 : 0;
assign way_hit[7] = (c_valid_o[7] && (c_tag_o[7] == tag))? 1 : 0;

assign cache_hit  = (way_hit[0] || way_hit[1] || way_hit[2] || way_hit[3] ||
                     way_hit[4] || way_hit[5] || way_hit[6] || way_hit[7]);

always @(*)
begin
    case ( { way_hit[0], way_hit[1], way_hit[2], way_hit[3], 
             way_hit[4], way_hit[5], way_hit[6], way_hit[7] } )
        8'b10000000: hit_index = 0;
        8'b01000000: hit_index = 1;
        8'b00100000: hit_index = 2;
        8'b00010000: hit_index = 3;
        8'b00001000: hit_index = 4;
        8'b00000100: hit_index = 5;
        8'b00000010: hit_index = 6;
        8'b00000001: hit_index = 7;
        default: hit_index = 0; // error: multiple-way hit!
    endcase
end
`endif

always @(posedge clk_i)
begin
    victim_sel <= FIFO_cnt[line_index];
end

always @(posedge clk_i)
begin
    if (rst_i)
        for (idx = 0; idx < N_LINES; idx = idx + 1) FIFO_cnt[idx] <= 0;
    else if (S == RdfromMem && coherence_done_i)
        FIFO_cnt[line_index] <= FIFO_cnt[line_index] + 1;
end

//====================================================
// Register some input signals from the processor.
//====================================================

//=======================================================
// Wback all cache blocks to the main memory
//=======================================================
assign addr_sram = (busy_flushing_o)? N_LINES_cnt : line_index;

always @(posedge clk_i)
begin
    if (rst_i)
        N_LINES_cnt <= 0;
    else if (S_nxt == WbtoMemAllFinish)
        N_LINES_cnt <= N_LINES_cnt + 1;
end

always @(posedge clk_i)
begin
    if (rst_i)
        N_WAYS_cnt <= 0;
    else if (N_LINES_cnt == N_LINES - 1 && S_nxt == WbtoMemAllFinish)
        N_WAYS_cnt <= N_WAYS_cnt + 1;
end

always @(posedge clk_i) begin
    WbAllFinish_r <= WbAllFinish;
end

//-----------------------------------------------
// Read a 32-bit word from the target cache line
//-----------------------------------------------

always @(*)
begin // for hit
    case (line_offset)
`ifdef ARTY
        2'b11: fromCache = c_data_hit[ 31: 0];     // [127: 96]
        2'b10: fromCache = c_data_hit[ 63: 32];    // [ 95: 64]
        2'b01: fromCache = c_data_hit[ 95: 64];    // [ 63: 32]
        2'b00: fromCache = c_data_hit[127: 96];    // [ 31:  0]
`elsif QMCore
        2'b11: fromCache = c_data_hit[ 31: 0];     // [127: 96]
        2'b10: fromCache = c_data_hit[ 63: 32];    // [ 95: 64]
        2'b01: fromCache = c_data_hit[ 95: 64];    // [ 63: 32]
        2'b00: fromCache = c_data_hit[127: 96];    // [ 31:  0]
`else // KC705, Genesys2, K7BaseC, or AXKU5
        3'b111: fromCache = c_data_hit[ 31: 0];    // [255:224]
        3'b110: fromCache = c_data_hit[ 63: 32];   // [223:192]
        3'b101: fromCache = c_data_hit[ 95: 64];   // [191:160]
        3'b100: fromCache = c_data_hit[127: 96];   // [159:128]
        3'b011: fromCache = c_data_hit[159: 128];  // [127: 96]
        3'b010: fromCache = c_data_hit[191: 160];  // [ 95: 64]
        3'b001: fromCache = c_data_hit[223: 192];  // [ 63: 32]
        3'b000: fromCache = c_data_hit[255: 224];  // [ 31:  0]
`endif
    endcase
end

always @(*)
begin // for miss
    case (line_offset)
`ifdef ARTY
        2'b11: fromMem = coherence_data_i[ 31: 0];        // [127: 96]
        2'b10: fromMem = coherence_data_i[ 63: 32];       // [ 95: 64]
        2'b01: fromMem = coherence_data_i[ 95: 64];       // [ 63: 32]
        2'b00: fromMem = coherence_data_i[127: 96];       // [ 31:  0]
`elsif QMCore
        2'b11: fromMem = coherence_data_i[ 31: 0];        // [127: 96]
        2'b10: fromMem = coherence_data_i[ 63: 32];       // [ 95: 64]
        2'b01: fromMem = coherence_data_i[ 95: 64];       // [ 63: 32]
        2'b00: fromMem = coherence_data_i[127: 96];       // [ 31:  0]
`else // KC705, Genesys2, K7BaseC, or AXKU5
        3'b111: fromMem = coherence_data_i[ 31: 0];       // [255:224]
        3'b110: fromMem = coherence_data_i[ 63: 32];      // [223:192]
        3'b101: fromMem = coherence_data_i[ 95: 64];      // [191:160]
        3'b100: fromMem = coherence_data_i[127: 96];      // [159:128]
        3'b011: fromMem = coherence_data_i[159: 128];     // [127: 96]
        3'b010: fromMem = coherence_data_i[191: 160];     // [ 95: 64]
        3'b001: fromMem = coherence_data_i[223: 192];     // [ 63: 32]
        3'b000: fromMem = coherence_data_i[255: 224];     // [ 31:  0]
`endif
    endcase
end

// Output signals   ////////////////////////////////////////////////////////////
always @(*)
begin // Note: p_data_o is significant when processor read data
    if ( (S == Analysis && !probe_on_curr_line) && cache_hit && !p_rw_r)
        p_data_o = fromCache;
    else if ((S == RdfromMem && coherence_done_i) && !p_rw_r)
        p_data_o = fromMem;
    else
        p_data_o = {XLEN{1'b0}};
end

assign p_ready_o = ((S == Analysis && !probe_on_curr_line) && cache_hit && !busy_flushing_o && (!p_rw_r || !c_share_o[hit_index])) ||
                    (S == RdfromMem && coherence_done_i) || (S == WriteHitShare && coherence_done_i);

//======================================================================
// Create a single-cycle memory request pluse for the memory controller
//======================================================================
// The old code uses the reqest/act protocol, which is corrected by the
// CDC synchronizer to match the strobe protocol of MIG. Modified to
// strobe protocol by Chun-Jen Tsai, 09/29/2023.
assign coherence_strobe_o = ((S == RdfromMem ) && !coherence_done_i) || share_modify_o || coherence_replacement_o;
assign coherence_replacement_o = (S == WbtoMem) && !coherence_done_i;

//======================================================================

assign coherence_addr_o = (S == WbtoMem) ? (busy_flushing_o) ? 
                    {c_tag_o[N_WAYS_cnt], N_LINES_cnt, {WORD_BITS{1'b0}}, 2'b0} : {c_tag_o[victim_sel], line_index, {WORD_BITS{1'b0}}, 2'b0} 
                    :
                  (S == RdfromMem || S == WriteHitShare || S_nxt == WriteHitShare) ? {p_addr_r[XLEN-1 : WORD_BITS+2], {WORD_BITS{1'b0}}, 2'b0} : {XLEN{1'b0}};

assign write_back_data_o = (S == WbtoMem && busy_flushing_o) ? c_block[N_WAYS_cnt] : c_block[victim_sel];

assign response_data_o = probe_block[probe_hit_index];
//------------------------------------------------------------------------
// Write the correct bytes according to the signal p_byte_enable_r
//------------------------------------------------------------------------

always @(*)
begin           // write miss : write hit;
    case (p_byte_enable_r)
        // DataMem_Addr[1:0] == 2'b00
        4'b0001: update_data = (S == RdfromMem && coherence_done_i) ?
                      { fromMem[31:8], p_data_r[7:0] } :
                      { fromCache[31:8], p_data_r[7:0] };
        4'b0011: update_data = (S == RdfromMem && coherence_done_i) ?
                      { fromMem[31:16], p_data_r[15:0] } :
                      { fromCache[31:16], p_data_r[15:0]};
        4'b1111: update_data = p_data_r;

        // DataMem_Addr[1:0] == 2'b01
        4'b0010: update_data = (S == RdfromMem && coherence_done_i) ?
                      { fromMem[31:16], p_data_r[15:8], fromMem[7:0] } :
                      { fromCache[31 : 16], p_data_r[15:8], fromCache[7:0] };

        // DataMem_Addr[1:0] == 2'b10
        4'b0100: update_data = (S == RdfromMem && coherence_done_i) ?
                      { fromMem[31:24], p_data_r[23:16], fromMem[15:0] } :
                      { fromCache[31:24], p_data_r[23:16], fromCache[15:0] };
        4'b1100: update_data = (S == RdfromMem && coherence_done_i) ?
                      { p_data_r[31:16], fromMem[15:0] } :
                      { p_data_r[31:16], fromCache[15:0] };

        // DataMem_Addr[1:0] == 2'b11
        4'b1000: update_data = (S == RdfromMem && coherence_done_i) ?
                      { p_data_r[31:24], fromMem[23:0] } :
                      { p_data_r[31:24], fromCache[23:0] };
        default: update_data = 32'b0;
    endcase
end

//------------------------------------------------------------------------
// Write a 32-bit word into the target cache line.
//------------------------------------------------------------------------
/* Writing into cache from the processor or the main memory */
always @(*) begin
    if (p_is_amo_i)begin
        case (line_offset)
`ifdef ARTY
        2'b11: c_data_update = {c_data_hit[127:32], p_data_i};
        2'b10: c_data_update = {c_data_hit[127:64], p_data_i, c_data_hit[31:0]};
        2'b01: c_data_update = {c_data_hit[127:96], p_data_i, c_data_hit[63:0]};
        2'b00: c_data_update = {p_data_i, c_data_hit[95:0]};
`elsif QMCore
        2'b11: c_data_update = {c_data_hit[127:32], p_data_i};
        2'b10: c_data_update = {c_data_hit[127:64], p_data_i, c_data_hit[31:0]};
        2'b01: c_data_update = {c_data_hit[127:96], p_data_i, c_data_hit[63:0]};
        2'b00: c_data_update = {p_data_i, c_data_hit[95:0]};
`else // KC705, Genesys2, K7BaseC, or AXKU5
            3'b111: c_data_update = {c_data_hit[255: 32], p_data_i};
            3'b110: c_data_update = {c_data_hit[255: 64], p_data_i, c_data_hit[ 31:0]};
            3'b101: c_data_update = {c_data_hit[255: 96], p_data_i, c_data_hit[ 63:0]};
            3'b100: c_data_update = {c_data_hit[255:128], p_data_i, c_data_hit[ 95:0]};
            3'b011: c_data_update = {c_data_hit[255:160], p_data_i, c_data_hit[127:0]};
            3'b010: c_data_update = {c_data_hit[255:192], p_data_i, c_data_hit[159:0]};
            3'b001: c_data_update = {c_data_hit[255:224], p_data_i, c_data_hit[191:0]};
            3'b000: c_data_update = {p_data_i, c_data_hit[223:0]};
`endif
        endcase
    end

    else begin
        case (line_offset)
`ifdef ARTY
        2'b11: c_data_update = {c_data_hit[127:32], update_data};
        2'b10: c_data_update = {c_data_hit[127:64], update_data, c_data_hit[31:0]};
        2'b01: c_data_update = {c_data_hit[127:96], update_data, c_data_hit[63:0]};
        2'b00: c_data_update = {update_data, c_data_hit[95:0]};
`elsif QMCore
        2'b11: c_data_update = {c_data_hit[127:32], update_data};
        2'b10: c_data_update = {c_data_hit[127:64], update_data, c_data_hit[31:0]};
        2'b01: c_data_update = {c_data_hit[127:96], update_data, c_data_hit[63:0]};
        2'b00: c_data_update = {update_data, c_data_hit[95:0]};
`else // KC705, Genesys2, K7BaseC, or AXKU5
    
            3'b111: c_data_update = {c_data_hit[255: 32], update_data};
            3'b110: c_data_update = {c_data_hit[255: 64], update_data, c_data_hit[ 31:0]};
            3'b101: c_data_update = {c_data_hit[255: 96], update_data, c_data_hit[ 63:0]};
            3'b100: c_data_update = {c_data_hit[255:128], update_data, c_data_hit[ 95:0]};
            3'b011: c_data_update = {c_data_hit[255:160], update_data, c_data_hit[127:0]};
            3'b010: c_data_update = {c_data_hit[255:192], update_data, c_data_hit[159:0]};
            3'b001: c_data_update = {c_data_hit[255:224], update_data, c_data_hit[191:0]};
        3'b000: c_data_update = {update_data, c_data_hit[223:0]};
`endif
        endcase
    end
end

always @(*) begin
    case (line_offset)
`ifdef ARTY
        2'b11: m_data_update = {coherence_data_i[127:32], update_data};
        2'b10: m_data_update = {coherence_data_i[127:64], update_data, coherence_data_i[31:0]};
        2'b01: m_data_update = {coherence_data_i[127:96], update_data, coherence_data_i[63:0]};
        2'b00: m_data_update = {update_data, coherence_data_i[95:0]};
`elsif QMCore
        2'b11: m_data_update = {coherence_data_i[127:32], update_data};
        2'b10: m_data_update = {coherence_data_i[127:64], update_data, coherence_data_i[31:0]};
        2'b01: m_data_update = {coherence_data_i[127:96], update_data, coherence_data_i[63:0]};
        2'b00: m_data_update = {update_data, coherence_data_i[95:0]};
`else // KC705, Genesys2, K7BaseC, or AXKU5
        3'b111: m_data_update = {coherence_data_i[255: 32], update_data};
        3'b110: m_data_update = {coherence_data_i[255: 64], update_data, coherence_data_i[ 31:0]};
        3'b101: m_data_update = {coherence_data_i[255: 96], update_data, coherence_data_i[ 63:0]};
        3'b100: m_data_update = {coherence_data_i[255:128], update_data, coherence_data_i[ 95:0]};
        3'b011: m_data_update = {coherence_data_i[255:160], update_data, coherence_data_i[127:0]};
        3'b010: m_data_update = {coherence_data_i[255:192], update_data, coherence_data_i[159:0]};
        3'b001: m_data_update = {coherence_data_i[255:224], update_data, coherence_data_i[191:0]};
        3'b000: m_data_update = {update_data, coherence_data_i[223:0]};
`endif
    endcase
end

always @(*)
begin
    if (!p_rw_r) // Processor read miss and update cache data
        c_data_i = (S == RdfromMem && coherence_done_i) ? coherence_data_i : {CLSIZE{1'b0}}; // read/write miss
    else begin   // Processor write cache
        if ( (S == Analysis && !probe_on_curr_line) && cache_hit) // write hit
            c_data_i = c_data_update;
        else if (S == RdfromMem && coherence_done_i)     // write miss
            c_data_i = m_data_update;
        else
            c_data_i = {CLSIZE{1'b0}};
    end
end

assign coherence_rw_o = p_rw_r;
assign share_modify_o = ((S == WriteHitShare && !coherence_done_i) || S_nxt == WriteHitShare) && !probe_on_share_modify_line; // write hit on share state
// Set a signal for flushing-in-progress notification
always @(posedge clk_i) begin
    if (rst_i)
        busy_flushing_o <= 0;
    else if (S == Idle && !busy_flushing_o)
        busy_flushing_o <= p_flush_i;
    else if (WbAllFinish_r && S == WbtoMemAllFinish)
        busy_flushing_o <= 0;
end

//=======================================================================
//  Compute the write flags for cache block & tag, valid, and dirty bits
//=======================================================================
//**** cache write ****//
always @(*) begin
    if ((S == Analysis && !probe_on_curr_line) && cache_hit && p_rw_r)
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            cache_write[idx] = way_hit[idx];
    else if (S == RdfromMem && coherence_done_i)
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            cache_write[idx] = (idx == victim_sel);
    else
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            cache_write[idx] = 1'b0;
end

//**** valid write ****//
always @(*) begin
    if (S == Init)
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            valid_write[idx] = 1'b1;
    else if (S_nxt == WbtoMemAllFinish)
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            valid_write[idx] = (idx == N_WAYS_cnt);
    else if (S == RdfromMem && coherence_done_i)
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            valid_write[idx] = (idx == victim_sel);
    else
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            valid_write[idx] = 1'b0;
end

//**** dirty write ****//
always @(*) begin
    if (S == Init)
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            dirty_write[idx] = 1'b1;
    else if (S_nxt == WbtoMemAllFinish)
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            dirty_write[idx] = (idx == N_WAYS_cnt);
    else if ((S == RdfromMem) && coherence_done_i)  //own get M
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            dirty_write[idx] = (idx == victim_sel);
    else if (S == Analysis && !probe_on_curr_line && cache_hit && p_rw_r) // write hit
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            dirty_write[idx] = (idx == hit_index);
    else
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            dirty_write[idx] = 1'b0;
end

//**** share write ****//
always @(*) begin
    if (S == Init)
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            share_write[idx] = 1'b1;
    else if (S_nxt == WbtoMemAllFinish)
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            share_write[idx] = (idx == N_WAYS_cnt);
    else if (S == RdfromMem && coherence_done_i) //own get S -> S
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            share_write[idx] = (idx == victim_sel);
    else if (S == WriteHitShare && coherence_done_i) // write hit on share state
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            share_write[idx] = (idx == hit_index);
    else
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            share_write[idx] = 1'b0;
end

//=======================================================
//  Cache data storage in Block RAM
//=======================================================
//probe write is always zero
genvar i;
generate
    for (i = 0; i < N_WAYS; i = i + 1)
    begin
        bram_dp #(.DATA_WIDTH(CLSIZE), .N_ENTRIES(N_LINES))
             DATA_BRAM(
                 .clk_i(clk_i),
                 .en_i(1'b1),
                 .a_we_i(cache_write[i]),
                 .a_addr_i(addr_sram),
                 .a_data_i(c_data_i),  // data from processor or memory.
                 .a_data_o(c_block[i]),

                 .b_we_i(1'b0), 
                 .b_addr_i(probe_line_index),
                 .b_data_i(), 
                 .b_data_o(probe_block[i])
             );
    end
endgenerate

//=======================================================
//  Tags storage in Block RAM
//=======================================================
genvar j;
generate
    for (j = 0; j < N_WAYS; j = j + 1)
    begin
        bram_dp #(.DATA_WIDTH(TAG_BITS), .N_ENTRIES(N_LINES))
             TAG_BRAM(
                 .clk_i(clk_i),
                 .en_i(1'b1),
                 .a_we_i(cache_write[j]),
                 .a_addr_i(addr_sram),
                 .a_data_i(tag),
                 .a_data_o(c_tag_o[j]),

                 .b_we_i(1'b0),
                 .b_addr_i(probe_line_index),
                 .b_data_i(),
                 .b_data_o(probe_tag_o[j])
             );
    end
endgenerate

//=======================================================
//  Valid bits storage in distributed RAM
//=======================================================
// MES: 1
// I: 0
// own get S   ->                 valid <= 1    (I -> E/S)
// own get M   ->                 valid <= 1    (I -> M)
// other get M ->                 valid <= 0    (M/E/S -> I)
assign valid_data = S == RdfromMem && coherence_done_i ? 1'b1 : 1'b0;  // own get S/M

assign valid_write_addr = 
            (S_nxt == WbtoMemAllFinish)? N_LINES_cnt :
            (S == Init)? init_count :
            (PROBE_S == PROBE_Analysis && invalidate_r && probe_cache_hit) ? probe_line_index :  //other get M
            (S == RdfromMem && coherence_done_i) ? line_index : 0; // own get S/M


genvar k;
generate
    for (k = 0; k < N_WAYS; k = k + 1)
    begin
        distri_ram_dp #(.ENTRY_NUM(N_LINES), .XLEN(1))
             VALID_RAM(
                 .clk_i(clk_i),
                 .we_i(valid_write[k] | probe_valid_write[k]),
                  //PROBE_S == PROBE_Update && probe_cache_hit && !probe_same_wt_all => remote core write done
                 .data_i(valid_data),
                 .write_addr_i(valid_write_addr),
                 // port a
                 .a_read_addr_i(addr_sram),
                 .a_data_o(c_valid_o[k]),
                 // port b
                 .b_read_addr_i(probe_line_index),
                 .b_data_o(probe_valid_o[k])
             );
    end
endgenerate

//=======================================================
//  Dirty bits storage in distributed RAM 
//=======================================================
// ESI: 0
// M: 1
// own write hit ->              dirty <= 1    (M/E/S -> M)
// own get E->                    dirty <= 0    (I -> E/S)
// own get M->                    dirty <= 1    (I -> M)
assign dirty_data = (S == RdfromMem && coherence_done_i && p_rw_r) ||  // own get M
                 (S == Analysis && !probe_on_curr_line && cache_hit && p_rw_r) ? 1'b1 : 1'b0;            // own write hit

assign dirty_waddr =
                     (S_nxt == WbtoMemAllFinish)? N_LINES_cnt :
                         (S == Init)? init_count :  
                ((PROBE_S == PROBE_Analysis && probe_cache_hit) ?  probe_line_index : line_index); 
genvar m;
generate
    for (m = 0; m < N_WAYS; m = m + 1)
    begin
        distri_ram_dp #(.ENTRY_NUM(N_LINES), .XLEN(1))
             DIRTY_RAM(
                 .clk_i(clk_i),
                 .we_i(dirty_write[m] | probe_dirty_write[m]),
                 //I
                 .data_i(dirty_data),
                 .write_addr_i(dirty_waddr),                 
                 // port a
                 .a_read_addr_i(addr_sram),
                 .a_data_o(c_dirty_o[m]),
                 // port b
                 .b_read_addr_i(probe_line_index),
                 .b_data_o(probe_dirty_o[m])
             );
    end
endgenerate

//=======================================================
//  Share bits storage in distributed RAM
//=======================================================
// MEI: 0
// S: 1
// own get S-> ~make_exclusive -> share <= 1    (I -> S)
// own get S-> make_exclusive ->  share <= 0    (I -> E)
// own get M->                    share <= 0    (I -> M)
// other get S ->                 share <= 1    (M/E/S -> S)
// other get M ->                 share <= 0    (M/E/S -> I) (clean share bit)
assign share_data = (PROBE_S == PROBE_Analysis && probe_cache_hit && !invalidate_r) ||          //other getS
                  (S == RdfromMem && coherence_done_i && !make_exclusive_i && !p_rw_r)? 1'b1 : 1'b0;       // own getS

assign share_waddr =
                    (S_nxt == WbtoMemAllFinish)? N_LINES_cnt :
                 ((S == RdfromMem && coherence_done_i)|| (S == WriteHitShare && coherence_done_i) ) ?                           // own getS, getM
                  line_index  :
                ((PROBE_S == PROBE_Analysis && probe_cache_hit) ?  probe_line_index : 0 );    // other getS, get M
genvar n;
generate
    for (n = 0; n < N_WAYS; n = n + 1)
    begin
        distri_ram_dp #(.ENTRY_NUM(N_LINES), .XLEN(1))
             SHARE_RAM(
                 .clk_i(clk_i),
                 .we_i(share_write[n] | probe_share_write[n]),
                 .data_i(share_data),
                 .write_addr_i(share_waddr),       
                 // port a
                 .a_read_addr_i(addr_sram),
                 .a_data_o(c_share_o[n]),
                 // port b
                 .b_read_addr_i(probe_line_index),
                 .b_data_o(probe_share_o[n])
             );
    end
endgenerate
`ifdef PROFILE_DCACHE
//profiler part
// I2M : Number of Event of Invalid -> Modify (write miss)
// I2E : Number of Event of Invalid -> Exclusive (read miss) 
// I2S : Number of Event of Invalid -> Shared (read miss) 
// S2M : Number of Event of Shared -> Modify  (write hit on shared state block)
// M_replaced : Number of Event for evicting a Modified block
// E_replaced : Number of Event for evicting an Exclusive block
reg [31:0] I2M, I2E, I2S;
reg [31:0] S2M;
reg [31:0] M_replaced , E_replaced;
reg [31:0] MorE2M; // write hita
// // I2M_latency : Number of Event of Invalid -> Modify (write miss)
// // I2E_latency : Number of Event of Invalid -> Exclusive (read miss) 
// // I2S_latency : Number of Event of Invalid -> Shared (read miss) 
// // S2M_latency : Number of Event of Shared -> Modify  (write hit on shared state block)
// // M2replaced_latency : Number of Event for evicting a Modified block
// // E2replaced_latency : Number of Event for evicting an Exclusive block

reg [15:0] curr_latency, write_share_latency, relpace_latency;
reg miss_flag, write_share_flag;
reg replace_flag;

reg [63:0]  I2M_latency, I2E_latency, I2S_latency;
reg [63:0]  S2M_latency;
reg [63:0]  M2replaced_latency, E2replaced_latency;

/*
F0010000: I2M counter
F0010004: I2E counter
F0010008: I2S counter
F001000C: S2M counter
F0010010: M2Replaced counter
F0010014: E2Replaced counter

// F0010018: I2M latency high 32 bit
// F001001C: I2M latency low 32 bit
// F0010020: I2E latency high 32 bit
// F0010024: I2E latency low 32 bit
// F0010028: I2S latency high 32 bit
// F001002C: I2S latency low 32 bit
// F0010030: S2M latency high 32 bit
// F0010034: S2M latency low 32 bit
// F0010038: M2Replaced latency high 32 bit
// F001003C: M2Replaced latency low 32 bit
// F0010040: E2Replaced latency high 32 bit
// F0010044: E2Replaced latency low 32 bit

// F0010048: MorE2M counter
*/

always @(posedge clk_i) begin
    if (rst_i) begin
        profile_data <= 0;
        profile_ready <= 0;
    end
    else if (profile_req) begin
        profile_ready <= 1;
        case(profile_addr[15:0])
            16'h0000: profile_data <= I2M;
            16'h0004: profile_data <= I2E;
            16'h0008: profile_data <= I2S;
            16'h000C: profile_data <= S2M;
            16'h0010: profile_data <= M_replaced;
            16'h0014: profile_data <= E_replaced;
            16'h0018: profile_data <= I2M_latency[63:32];
            16'h001C: profile_data <= I2M_latency[31:0];
            16'h0020: profile_data <= I2E_latency[63:32];
            16'h0024: profile_data <= I2E_latency[31:0];
            16'h0028: profile_data <= I2S_latency[63:32];
            16'h002C: profile_data <= I2S_latency[31:0];
            16'h0030: profile_data <= S2M_latency[63:32];
            16'h0034: profile_data <= S2M_latency[31:0];
            16'h0038: profile_data <= M2replaced_latency[63:32];
            16'h003C: profile_data <= M2replaced_latency[31:0];
            16'h0040: profile_data <= E2replaced_latency[63:32];
            16'h0044: profile_data <= E2replaced_latency[31:0];
            16'h0048: profile_data <= MorE2M;
            default:  profile_data <= 0;
        endcase
    end
    else begin
        profile_ready <= 0;
        profile_data <= 0;
    end
end

always @(posedge clk_i) begin
    if (rst_i) begin
        I2M <= 0;
        I2E <= 0;
        I2S <= 0;
        S2M <= 0;
        M_replaced <= 0;
        E_replaced <= 0;
        MorE2M <= 0;
    end
    else begin
        if (S == Analysis && !probe_on_curr_line) begin
            if (cache_hit) begin
                if (p_rw_r) begin
                   if (c_share_o[hit_index]) S2M <= S2M + 1;
                   else MorE2M <= MorE2M + 1;
                end
            end
            if (!cache_hit && c_valid_o[victim_sel]) begin
                if (!c_share_o[victim_sel])begin
                    if (c_dirty_o[victim_sel]) M_replaced <= M_replaced + 1;
                    else E_replaced <= E_replaced + 1;
                end
            end
        end
        else if (S == WriteHitShare && probe_on_share_modify_line) begin
            S2M <= S2M - 1;
        end
        else if (S == RdfromMem && coherence_done_i) begin
            if (p_rw_r) begin
                I2M <= I2M + 1;
            end
            else begin
                if (make_exclusive_i) I2E <= I2E + 1;
                else I2S <= I2S + 1;
            end
        end
    end 
end

always @(posedge clk_i ) begin
   if (rst_i) begin
       I2M_latency <= 0;
       I2E_latency <= 0;
       I2S_latency <= 0;
       miss_flag <= 0;
   end
   else begin
       if (S_nxt == RdfromMem)  miss_flag <= 1;
       else if (S == RdfromMem & coherence_done_i) begin
           miss_flag <= 0;
           if (p_rw_r) I2M_latency <= I2M_latency + curr_latency;
           else if (make_exclusive_i) I2E_latency <= I2E_latency + curr_latency;
           else I2S_latency <= I2S_latency + curr_latency;
       end
   end
end

always @(posedge clk_i ) begin
   if (rst_i) curr_latency <= 0;
   else if (miss_flag ) curr_latency <= curr_latency + 1;
   else curr_latency <= 0;
end

always @(posedge clk_i ) begin
   if (rst_i) begin
       M2replaced_latency <= 0;
       E2replaced_latency <= 0;
       replace_flag <= 0;
   end
   else begin
       if (S_nxt == WbtoMem) replace_flag <= 1;
       else if (S == WbtoMem & coherence_done_i) begin
           replace_flag <= 0;
           if (busy_flushing_o) begin
               if (c_dirty_o[N_WAYS_cnt]) M2replaced_latency <= M2replaced_latency + relpace_latency;
               else E2replaced_latency <= E2replaced_latency + relpace_latency;
           end
           else begin
               if (c_dirty_o[victim_sel]) M2replaced_latency <= M2replaced_latency + relpace_latency;
               else E2replaced_latency <= E2replaced_latency + relpace_latency;
           end
       end
   end
end

always @(posedge clk_i ) begin
   if (rst_i) relpace_latency <= 0;
   else if (replace_flag ) relpace_latency <= relpace_latency + 1;
   else relpace_latency <= 0;
end

always @(posedge clk_i ) begin
   if (rst_i) begin
       S2M_latency <= 0;
       write_share_flag <= 0;
   end
   else begin
       if (S_nxt == WriteHitShare)  write_share_flag <= 1;
       else if (probe_on_share_modify_line) begin
            write_share_flag <= 0;
       end
       else if (S == WriteHitShare & coherence_done_i) begin
           write_share_flag <= 0;
           S2M_latency <= S2M_latency + write_share_latency;
       end
   end
end

always @(posedge clk_i ) begin
   if (rst_i) write_share_latency <= 0;
   else if (write_share_flag ) write_share_latency <= write_share_latency + 1;
   else write_share_latency <= 0;
end
`endif 
endmodule
