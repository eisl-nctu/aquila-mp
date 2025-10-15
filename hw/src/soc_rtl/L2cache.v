`timescale 1ns / 1ps
// =============================================================================
//  Program : L2cache.v
//  Author  : Lin-en Yen
//  Date    : Feb/25/2024
// -----------------------------------------------------------------------------
//  Description:
//  This module implements the L2 Cache with the following
//  properties:
//      4-way set associative
//      FIFO replacement policy
//      Write-back
//      Write allocate
// -----------------------------------------------------------------------------
//  Revision information:
//
//  Feb/12/2025, by Ye Chen:
//    Modified to implement the standard MESI protocol state transition method.
//
//  Apr/10/2025, by Tzu-Chen Yang:
//    Corrected implementation details related to the write-back event.
// 
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

module L2cache
#(
  parameter XLEN = 32,
  parameter CACHE_SIZE = 64,
  parameter CLSIZE = 128    // Cache line size.
)
(
    /////////// System signals   ///////////////////////////////////////////////
    input                     clk_i, rst_i,

    /////////// cache coherence unit signal //////////////////////////////////////////////
    //write back to L2
    input                     wb_i,                 // write back signal
    input                     wb_replacement_i,     // write back address
    input  [CLSIZE-1 : 0]     wb_data_i,            // write back data
    // request L2 cache
    input                     probe_strobe_i,       // coherence unit request signal.
    input                     probe_rw_i,           // 0 for read, 1 for write.
    input  [XLEN-1 : 0]       CU_L2_addr_i,         // Memory addr of the request.
    
    output reg [CLSIZE-1 : 0] response_data_o,      // Data from main memory.
    output                    response_ready_o,     // The cache data is ready.
    output                    make_exclusive_o,     // exclusive signal
    // invalidate signal
    input                     invalidate_i,         // invalidate signal
    input                     L1_is_instr_fetch_i,  // L1 is instruction fetch

    //invalidate L1 signal
    input                     CU_ready_i,           // CU is ready  
    output reg [XLEN-1 : 0]   invalidate_L1_addr_o, // invalidate L1 address
    output                    invalidate_L1_o,      // invalidate L1 signal
    /////////// main memory signal ///////////////////////////////////////
    input                     m_ready_i,            // Memory is ready.
    input  [CLSIZE-1 : 0]     m_data_i,             // Data from main memory.
    output                    m_strobe_o,           // Memory request signal.
    output                    m_rw_o,               // 0 for read, 1 for write.
    output   [XLEN-1 : 0]     m_addr_o,             // Memory addr of the request.
    output  [CLSIZE-1 : 0]     m_data_o             // Data to main memory.
);

//=======================================================
// Cache parameters
//=======================================================
localparam N_WAYS      = 4;
localparam N_LINES     = (CACHE_SIZE*1024*8) / (N_WAYS*CLSIZE);

localparam WAY_BITS    = $clog2(N_WAYS);
localparam BYTE_BITS   = 2;
localparam WORD_BITS   = $clog2(CLSIZE/XLEN);
localparam LINE_BITS   = $clog2(N_LINES);
localparam NONTAG_BITS = LINE_BITS + WORD_BITS + BYTE_BITS;
localparam TAG_BITS    = XLEN - NONTAG_BITS;

//=======================================================
// N-way associative cache signals
//=======================================================
wire                   way_hit[0 : N_WAYS-1];     // Cache-way hit flag.
wire                   way_wb_hit[0 : N_WAYS-1];
wire                   EorM_hit[0 : N_WAYS-1];
reg  [WAY_BITS-1 : 0]  hit_index;                 // Decoded way_hit[] signal.
wire                   cache_hit;                 // Got a cache hit?
reg  [CLSIZE-1 : 0]    c_data_i;                  // Data to write into cache.
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


integer idx;

assign c_data_hit = c_block[hit_index];

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

reg [XLEN-1 : 0] addr ;
always @(*)
begin
    if (invalidate_r)
        addr = invalidate_addr_r;
    else 
        addr = CU_L2_addr_i;
end
assign line_offset =  addr[WORD_BITS + BYTE_BITS - 1 : BYTE_BITS];
assign line_index  = addr[NONTAG_BITS - 1 : WORD_BITS + BYTE_BITS] ;
assign tag         = addr[XLEN - 1 : NONTAG_BITS];
//=======================================================
// Cache Finite State Machine
//=======================================================
localparam Idle             = 0,
           Analysis         = 1,
           WbtoMem          = 2,
           RdfromMem        = 3,
           Invalidate_L1    = 4;

// Cache controller state registers
reg [ 2 : 0] S, S_nxt;

//====================================================
// Cache Controller FSM
//====================================================
always @(posedge clk_i)
begin
    if (rst_i)
        S <= Idle;
    else
        S <= S_nxt;
end

always @(*)
begin
    case (S)
        Idle:
            if (need_invalidate) S_nxt = Invalidate_L1;
            else if (probe_strobe_i || wb_i || invalidate_i)
                S_nxt = Analysis;
            else
                S_nxt = Idle;
        Analysis:
            if (!cache_hit)begin
                if (invalidate_r)
                    S_nxt = Idle;
                else if (EorM_hit_all || Wb_Miss_Clean)
                    S_nxt = Idle;
                else if (c_valid_o[victim_sel] && c_dirty_o[victim_sel]) // dirty and valid block
                // else if (c_valid_o[victim_sel] && (c_dirty_o[victim_sel] || c_share_o[victim_sel]))
                    S_nxt = WbtoMem;
                else
                    S_nxt = RdfromMem;
            end
            else 
                // cache hit
                S_nxt = Idle;
        WbtoMem:
        // write back done -> RdfromMem (memory req) or Idle(wb to L2)
        // 
            if (m_ready_i) begin
                if (wb_i) begin
                    if (need_invalidate)
                        S_nxt = Invalidate_L1;
                    else
                        S_nxt = Idle;
                end
                else    
                    S_nxt = RdfromMem;
            end
            else
                S_nxt = WbtoMem;
        RdfromMem:
        // get response from main memory -> Idle
            if (m_ready_i) begin
                if (need_invalidate)
                    S_nxt = Invalidate_L1;
                else
                    S_nxt = Idle;
            end
            else
                S_nxt = RdfromMem;
        Invalidate_L1:
            if (CU_ready_i)
                S_nxt = Idle;
            else
                S_nxt = Invalidate_L1;
        default:
            S_nxt = Idle;
    endcase
end

// register invalidate signal
reg [XLEN-1:0] invalidate_addr_r;
reg invalidate_r;
always @(posedge clk_i)
begin
    if (rst_i) begin
        invalidate_r <= 0;
        invalidate_addr_r <= 0;
    end
    else if (invalidate_i) begin
        invalidate_r <= invalidate_i;
        invalidate_addr_r <= CU_L2_addr_i;
    end
    else begin
        invalidate_r <= 0;
        invalidate_addr_r <= 0;
    end
end

//invalidate L1 signal
reg need_invalidate;
always @(posedge clk_i) begin
    if (rst_i) begin
        need_invalidate <= 0;
        invalidate_L1_addr_o <= 0;
    end
    else if (S == Analysis && !cache_hit && c_valid_o[victim_sel] && c_share_o[victim_sel]) begin
        need_invalidate <= 1;
        invalidate_L1_addr_o <= {c_tag_o[victim_sel], line_index, {WORD_BITS{1'b0}}, 2'b0};
    end
    else if (S == Invalidate_L1 && CU_ready_i) begin
        need_invalidate <= 0;
        invalidate_L1_addr_o <= 0;
      end
    else begin
        need_invalidate <= need_invalidate;
        invalidate_L1_addr_o <= invalidate_L1_addr_o;
    end
end

assign invalidate_L1_o = S_nxt == Invalidate_L1;

// a signal to indicate EorM miss and victim is clean when L1 write back to L2
// wire Wb_Miss_Clean = S == Analysis && !cache_hit && !c_valid_o[victim_sel]   && wb_i;
  wire Wb_Miss_Clean = S == Analysis && !cache_hit && !(c_valid_o[victim_sel] && (c_dirty_o[victim_sel]))  && wb_i;
// Check and see if any cache way has the matched memory block.

assign EorM_hit[0] = ((c_tag_o[0] == tag) && (c_share_o[0] == 1) && (c_valid_o[0] == 0));
assign EorM_hit[1] = ((c_tag_o[1] == tag) && (c_share_o[1] == 1) && (c_valid_o[1] == 0));
assign EorM_hit[2] = ((c_tag_o[2] == tag) && (c_share_o[2] == 1) && (c_valid_o[2] == 0));
assign EorM_hit[3] = ((c_tag_o[3] == tag) && (c_share_o[3] == 1) && (c_valid_o[3] == 0));

 wire EorM_hit_all = (EorM_hit[0] || EorM_hit[1] || EorM_hit[2] || EorM_hit[3]);

assign way_wb_hit[0] = ((c_tag_o[0] == tag) && wb_i && (c_share_o[0] == 1) && (c_valid_o[0] == 0));
assign way_wb_hit[1] = ((c_tag_o[1] == tag) && wb_i && (c_share_o[1] == 1) && (c_valid_o[1] == 0));
assign way_wb_hit[2] = ((c_tag_o[2] == tag) && wb_i && (c_share_o[2] == 1) && (c_valid_o[2] == 0));
assign way_wb_hit[3] = ((c_tag_o[3] == tag) && wb_i && (c_share_o[3] == 1) && (c_valid_o[3] == 0));

assign way_hit[0] = (c_valid_o[0] && (c_tag_o[0] == tag)) || way_wb_hit[0] ? 1 : 0;
assign way_hit[1] = (c_valid_o[1] && (c_tag_o[1] == tag)) || way_wb_hit[1] ? 1 : 0;
assign way_hit[2] = (c_valid_o[2] && (c_tag_o[2] == tag)) || way_wb_hit[2] ? 1 : 0;
assign way_hit[3] = (c_valid_o[3] && (c_tag_o[3] == tag)) || way_wb_hit[3] ? 1 : 0;
assign cache_hit  = (way_hit[0] || way_hit[1] || way_hit[2] || way_hit[3]);

always @(*)
begin
    case ( { way_hit[0], way_hit[1], way_hit[2], way_hit[3] } )
        4'b1000: hit_index = 0;
        4'b0100: hit_index = 1;
        4'b0010: hit_index = 2;
        4'b0001: hit_index = 3;
        default: begin
            hit_index = 0; // error: multiple-way hit!
        end
    endcase
end

always @(posedge clk_i)
begin
    victim_sel <= FIFO_cnt[line_index];
end

always @(posedge clk_i)
begin
    if (rst_i)
        for (idx = 0; idx < N_LINES; idx = idx + 1) FIFO_cnt[idx] <= 0;
    else if (((S == RdfromMem || (S == WbtoMem && wb_i) )&& m_ready_i) || Wb_Miss_Clean) 
        FIFO_cnt[line_index] <= FIFO_cnt[line_index] + 1;
end

// Output signals   ////////////////////////////////////////////////////////////
always @(*)
begin // Note: response_data_o is significant when processor read data
    if ((S == Analysis) && cache_hit && !wb_i)
        response_data_o = c_data_hit;
    else if (S == RdfromMem && m_ready_i)
        response_data_o = m_data_i;
    else
        response_data_o = {CLSIZE{1'b0}};
end

// cache hit on S/I state
// cache miss and git data from memory done
// write back done
assign response_ready_o = (S == Analysis && cache_hit && !invalidate_r) || (m_ready_i && ((S == RdfromMem) || (S == WbtoMem && wb_i)) )
                         || Wb_Miss_Clean ;
// assign response_ready_o = (S == Analysis && cache_hit && !invalidate_r) || (m_ready_i && ((S == RdfromMem) || (S == WbtoMem && wb_i)) )
//                          || Wb_Miss_Clean||
//                           (S == Analysis && EorM_hit_all);

//======================================================================
// Create a single-cycle memory request pluse for the memory controller
//======================================================================
assign m_strobe_o = (S == WbtoMem || S == RdfromMem) && !m_ready_i;

//======================================================================

assign m_addr_o =  (S == WbtoMem) ? {c_tag_o[victim_sel], line_index, {WORD_BITS{1'b0}}, 2'b0} :
                  (S == RdfromMem) ? {CU_L2_addr_i[XLEN-1 : WORD_BITS+2], {WORD_BITS{1'b0}}, 2'b0} : {XLEN{1'b0}};

assign m_data_o = c_block[victim_sel];

assign m_rw_o = (S == WbtoMem) ? 1 : 0;

assign make_exclusive_o = (S == Analysis && cache_hit && !probe_rw_i && !c_share_o[hit_index]) || (S == RdfromMem && m_ready_i && !probe_rw_i );

//data to cache
always @(*)
begin
    if (wb_i)   // wb
        c_data_i = wb_data_i;
    else if (S == RdfromMem && m_ready_i)     // read from memory
        c_data_i = m_data_i;
    else
        c_data_i = {CLSIZE{1'b0}};
end

//=======================================================================
//  Compute the write flags for cache block & tag, valid, and dirty bits
//=======================================================================

//**** cache write ****//
always @(*) begin
    if (S == RdfromMem && m_ready_i)begin // read from memory finish and then store to L2 cache 
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            cache_write[idx] = (idx == victim_sel);
    end
    else if (S == WbtoMem && m_ready_i && wb_i)begin // L1 write back to L2 miss (victim line is dirty)
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            cache_write[idx] = (idx == victim_sel); // write back to L2
    end
    else if (S == Analysis && cache_hit && wb_i)begin // L1 write back to L2 hit and L2 is in EorM state
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            cache_write[idx] = (idx == hit_index);
    end
    else if (Wb_Miss_Clean)begin // L1 write back to L2 miss (directly write to victim line)
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            cache_write[idx] = (idx == victim_sel);
    end
    else begin
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            cache_write[idx] = 1'b0;
    end
end

//**** valid write ****//
always @(*) begin
    // only get S or wb L2 should turn to valid
    if (S == RdfromMem && m_ready_i)begin // cache miss --> read from memory 
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            valid_write[idx] = (idx == victim_sel); // valid should be 0
    end
    else if (S == WbtoMem && m_ready_i && wb_i) begin // L1 write back to L2 miss (EorM --> S || EorM --> I )
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            valid_write[idx] = (idx == victim_sel); // valid should be 1
    end
    else if (S == Analysis && cache_hit && wb_i)begin // L1 write back to L2 hit and L2 is in EorM state
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            valid_write[idx] = (idx == hit_index); // valid from 0 to 1
    end
    else if (Wb_Miss_Clean) begin // L1 write back to L2 miss and victim is clean
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            valid_write[idx] = (idx == victim_sel); // valid from 0 to 1
    end
    else if (S == Analysis && cache_hit && !(c_share_o[hit_index] && !probe_rw_i))begin // I --> EorM || S --> EorM
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            valid_write[idx] = (idx == hit_index); // valid from 1 to 0 
    end
    else if (S == Analysis && invalidate_r && cache_hit)begin // invalidate 
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            valid_write[idx] = (idx == hit_index); // EorM (valid --> 0 share --> 1)
    end
    else begin
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            valid_write[idx] = 1'b0;
    end
end


//**** share write ****//
always @(*) begin
    if (S == RdfromMem && m_ready_i)begin // cache miss --> read from memory 
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            share_write[idx] = (idx == victim_sel); // share should be 1
    end
    else if (S == WbtoMem && m_ready_i && wb_i) begin // L1 write back to L2 miss (EorM --> I / S )
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            share_write[idx] = (idx == victim_sel); // I (share --> 0) , S (share --> 1)
    end
    else if (S == Analysis && cache_hit && !c_share_o[hit_index])begin // I --> EorM
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            share_write[idx] = (idx == hit_index); // share from 1 to 0
    end
    else if (S == Analysis && wb_i && cache_hit)begin // L1 write back to L2 hit and L2 is in EorM state (EorM --> I / S )
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            share_write[idx] = (idx == hit_index); // I (share --> 0) , S (share --> 1)
    end
    else if (Wb_Miss_Clean) begin // L1 write back to L2 miss and victim is clean (EorM --> I / S )
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            share_write[idx] = (idx == victim_sel); // I (share --> 0) , S (share --> 1)
    end
    else if (S == Analysis && invalidate_r && cache_hit)begin // invalidate 
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            share_write[idx] = (idx == hit_index); // EorM (valid --> 0 share --> 1)
    end
    else
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            share_write[idx] = 1'b0;
end

//**** dirty write ****//
always @(*) begin
     if (S == RdfromMem && m_ready_i)begin // cache miss --> read from memory 
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            dirty_write[idx] = (idx == victim_sel); // dirty should be 0
    end
    else if (S == WbtoMem && m_ready_i && wb_i)begin // L1 write back to L2 miss (EorM --> S || EorM --> I )
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            dirty_write[idx] = (idx == victim_sel); // dirty should be 1
    end
    else if (S == Analysis && wb_i && cache_hit)begin // L1 write back to L2 hit 
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            dirty_write[idx] = (idx == hit_index); // dirty should be 1
    end
    else if (Wb_Miss_Clean)begin // L1 write back to L2 miss and victim is clean
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            dirty_write[idx] = (idx == victim_sel); // dirty should be 1
    end
    else
        for (idx = 0; idx < N_WAYS; idx = idx + 1)
            dirty_write[idx] = 1'b0;
end

//=======================================================
//  Cache data storage in Block RAM
//=======================================================
//probe write is always zero
genvar i;
generate
    for (i = 0; i < N_WAYS; i = i + 1)
    begin
        sram #(.DATA_WIDTH(CLSIZE), .N_ENTRIES(N_LINES))
             DATA_BRAM(
                 .clk_i(clk_i),
                 .en_i(1'b1),
                 .we_i(cache_write[i]),
                 .addr_i(line_index),
                 .data_i(c_data_i),  // data from processor or memory.
                 .data_o(c_block[i])
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
        sram #(.DATA_WIDTH(TAG_BITS), .N_ENTRIES(N_LINES))
             TAG_BRAM(
                 .clk_i(clk_i),
                 .en_i(1'b1),
                 .we_i(cache_write[j]),
                 .addr_i(line_index),
                 .data_i(tag),
                 .data_o(c_tag_o[j])
             );
    end
endgenerate

//=======================================================
//  Valid bits storage in distributed RAM
//=======================================================
wire valid_data = (S == RdfromMem && m_ready_i) ? L1_is_instr_fetch_i :  // if L1 is instruction fetch, then valid bit is still 1
                  (S == WbtoMem && m_ready_i && wb_i) ? 1 :
                  (S == Analysis && wb_i && cache_hit) ? 1 :
                  (Wb_Miss_Clean) ? 1 :
                  (S == Analysis && cache_hit && !(c_share_o[hit_index] && !probe_rw_i)) ? L1_is_instr_fetch_i :
                  (S == Analysis && invalidate_r && cache_hit) ? 0 : 0;

genvar k;
generate
    for (k = 0; k < N_WAYS; k = k + 1)
    begin
        distri_ram #(.ENTRY_NUM(N_LINES), .XLEN(1))
             VALID_RAM(
                 .clk_i(clk_i),
                 .we_i(valid_write[k]),
                 .data_i(valid_data),
                 .write_addr_i(line_index),
                 .read_addr_i(line_index),
                 .data_o(c_valid_o[k])
             );
    end
endgenerate

//=======================================================
//  Share bits storage in distributed RAM
//=======================================================
wire share_data = (S == RdfromMem && m_ready_i) ? 1 :
                  (S == WbtoMem && m_ready_i && wb_i) ? ~wb_replacement_i :
                  (S == Analysis && cache_hit && wb_i) ? ~wb_replacement_i :
                  (Wb_Miss_Clean) ? ~wb_replacement_i :
                  (S == Analysis && cache_hit && !c_share_o[hit_index] && !wb_i) ? 1 : // get S on I -> S (s = 1)
                                                                                       //get M on I -> EorS (s = 1) 
                  (S == Analysis && invalidate_r && cache_hit) ? 1 : 0;



genvar n;
generate
    for (n = 0; n < N_WAYS; n = n + 1)
    begin
        distri_ram #(.ENTRY_NUM(N_LINES), .XLEN(1))
             SHARE_RAM(
                 .clk_i(clk_i),
                 .we_i(share_write[n]),
                 .data_i(share_data),
                 .write_addr_i(line_index),      
                 .read_addr_i(line_index),
                 .data_o(c_share_o[n])
             );
    end
endgenerate

//=======================================================
//  Dirty bits storage in distributed RAM 
//=======================================================
wire dirty_data = (S == RdfromMem && m_ready_i) ? 0 :
                  (S == WbtoMem && m_ready_i && wb_i) ? 1 :
                  (S == Analysis && cache_hit && wb_i) ? 1 :
                  (Wb_Miss_Clean) ? 1 : 0;     // own write hit

genvar m;
generate
    for (m = 0; m < N_WAYS; m = m + 1)
    begin
        distri_ram #(.ENTRY_NUM(N_LINES), .XLEN(1))
             DIRTY_RAM(
                 .clk_i(clk_i),
                 .we_i(dirty_write[m]),
                 .data_i(dirty_data),
                 .write_addr_i(line_index),      
                 .read_addr_i(line_index),
                 .data_o(c_dirty_o[m])
             );
    end
endgenerate

endmodule
