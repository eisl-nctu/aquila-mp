// =============================================================================
//  Program : cdc_sync.v
//  Author  : Po-wei Ho
//  Date    : Sep/14/2020
// -----------------------------------------------------------------------------
//  Description:
//  This is the Clock domain crossing synchronizer of the Aquila core (A RISC-V core).
//
//  Every signal has two FSM to control write and read operations of asynchronous FIFO.
//
//  After all signals are ready, this unit will output all synchronized signals for 
//  only one clock cycle.
// -----------------------------------------------------------------------------
//  Revision information:
//
//  Aug/02/2024, by Chun-Jen Tsai:
//    Fix the Clock-Domain Crossing issue. The original code did not use two reset
//    signals, one for each domain, to properly reset signals in each domain.
//    Another bug is that it uses DMEM_rw_r, instead of DMEM_rw_i in the clk_core
//    clock domain.
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

module cdc_sync #( parameter XLEN = 32, parameter CLSIZE = `CLP )
(
    // System signals
    input                 clk_core,
    input                 clk_memc,
    input                 rst_core_i,
    input                 rst_memc_i,

    // Aquila L2 CACHE port interface signals
    input                 P_strobe_i,
    input  [XLEN-1 : 0]   P_addr_i,
    input                 P_rw_i,
    input  [CLSIZE-1 : 0] P_wt_data_i,
    output                P_done_o,
    output [CLSIZE-1 : 0] P_rd_data_o,

    // MIG DCACHE port interface signals
    output                P_strobe_o,
    output [XLEN-1 : 0]   P_addr_o,
    output                P_rw_o,
    output [CLSIZE-1 : 0] P_wt_data_o,
    input                 P_done_i,
    input  [CLSIZE-1 : 0] P_rd_data_i
);

// FSM States
localparam
    P_memc_in_IDLE = 0,
    P_memc_in_WAIT = 1;
reg B, B_nxt;

localparam
    P_core_out_IDLE = 0,
    P_core_out_WAIT = 1;
reg D, D_nxt;

localparam
    P_done_core_out_IDLE = 0,
    P_done_core_out_WAIT = 1;
reg F, F_nxt;

localparam
    P_rd_data_core_out_IDLE = 0,
    P_rd_data_core_out_WAIT = 1;
reg G, G_nxt;

localparam
    P_memc_out_IDLE = 0,
    P_memc_out_WAIT = 1;
reg T, T_nxt;

localparam
    P_strobe_memc_out_IDLE = 0,
    P_strobe_memc_out_WAIT = 1;
reg U, U_nxt;

localparam
    P_addr_memc_out_IDLE = 0,
    P_addr_memc_out_WAIT = 1;
reg V, V_nxt;

localparam
    P_rw_memc_out_IDLE = 0,
    P_rw_memc_out_WAIT = 1;
reg W, W_nxt;

localparam
    P_wt_data_memc_out_IDLE = 0,
    P_wt_data_memc_out_WAIT = 1;
reg X, X_nxt;

// registers for fifo out
reg                 P_strobe_r;
reg [XLEN-1 : 0]    P_addr_r;
reg                 P_rw_r;
reg [CLSIZE-1 : 0]  P_wt_data_r;

reg                 P_done_r;
reg [CLSIZE-1 : 0]  P_rd_data_r;

wire P_memc_out_ready;
assign P_memc_out_ready = T == P_memc_out_IDLE && T_nxt == P_memc_out_WAIT;
wire P_core_out_ready;
assign P_core_out_ready = D == P_core_out_IDLE && D_nxt == P_core_out_WAIT;

// output to mig
assign P_strobe_o  = P_memc_out_ready? P_strobe_r : 0;
assign P_addr_o    = P_memc_out_ready? P_addr_r : 0;
assign P_rw_o      = P_memc_out_ready? P_rw_r : 0;
assign P_wt_data_o = P_memc_out_ready? P_wt_data_r : 0;

// output to core
assign P_done_o    = P_core_out_ready? P_done_r : 0;
assign P_rd_data_o = P_core_out_ready? P_rd_data_r : 0;

//=======================================================
//  Async_fifo signals
//=======================================================
// from core to mig
wire P_strobe_full, P_addr_full, P_rw_full, P_wt_data_full;
wire P_strobe_empty, P_addr_empty, P_rw_empty, P_wt_data_empty;

wire P_strobe_out;
wire [XLEN-1 : 0] P_addr_out;
wire P_rw_out;
wire [CLSIZE-1 : 0] P_wt_data_out;

// form mig to core
wire P_done_full, P_rd_data_full;
wire P_done_empty, P_rd_data_empty;

wire P_done_out;
wire [CLSIZE-1 : 0] P_rd_data_out;

//=======================================================
//  Mig input registers
//=======================================================
reg P_done_i_r;
reg [CLSIZE-1 : 0] P_rd_data_i_r;


always @(posedge clk_memc)
begin
    if (rst_memc_i) begin
        P_done_i_r    <= 0;
        P_rd_data_i_r <= 0;
    end        
    else if (P_done_i) begin
        P_done_i_r    <= P_done_i;
        P_rd_data_i_r <= P_rd_data_i;
    end
    else if (P_strobe_o) begin
        P_done_i_r    <= 0;
        P_rd_data_i_r <= 0;
    end
end


/* *******************************************************************************
 * Finite State Machines from cpu to fifo                                         *
 * *******************************************************************************/
//=======================================================
//  FSM IMEM of Core clock domain input
//=======================================================
localparam
    P_core_clk_in_IDLE = 0,
    P_core_clk_in_WAIT = 1;

reg P, P_nxt;

wire P_in_full_all, P_in_wr_en;
assign P_in_full_all = P_strobe_full & P_addr_full & P_rw_full & P_wt_data_full;
assign P_in_wr_en = (P == P_core_clk_in_IDLE && P_nxt == P_core_clk_in_WAIT);

always @(posedge clk_core)
begin
    if (rst_core_i)
        P <= P_core_clk_in_IDLE;
    else
        P <= P_nxt;
end

always @(*)
begin
    case (P)
        P_core_clk_in_IDLE: P_nxt = (P_strobe_i & !P_in_full_all)? P_core_clk_in_WAIT : P_core_clk_in_IDLE;
        P_core_clk_in_WAIT: P_nxt = (P_strobe_i)? P_core_clk_in_WAIT : P_core_clk_in_IDLE;
    endcase
end

/* *******************************************************************************
 * FSM from fifo to mig                                                          *
 * *******************************************************************************/
//=======================================================
//  FSM IMEM of MEMC clock domain output
//=======================================================
always @(posedge clk_memc)
begin
    if (rst_memc_i)
        T <= P_memc_out_IDLE;
    else
        T <= T_nxt;
end

always @(*)
begin
    case (T)
        P_memc_out_IDLE: T_nxt = (U == P_strobe_memc_out_WAIT && V == P_addr_memc_out_WAIT && W == P_rw_memc_out_WAIT && X == P_wt_data_memc_out_WAIT)? P_memc_out_WAIT : P_memc_out_IDLE;
        P_memc_out_WAIT: T_nxt = P_memc_out_IDLE;
    endcase
end

//=======================================================
//  FSM IMEM_strobe of MEMC clock domain output
//=======================================================
always @(posedge clk_memc)
begin
    if (rst_memc_i)
        U <= P_strobe_memc_out_IDLE;
    else
        U <= U_nxt;
end

always @(*)
begin
    case (U)
        P_strobe_memc_out_IDLE: U_nxt = (!P_strobe_empty)? P_strobe_memc_out_WAIT : P_strobe_memc_out_IDLE;
        P_strobe_memc_out_WAIT: U_nxt = (P_memc_out_ready)? P_strobe_memc_out_IDLE : P_strobe_memc_out_WAIT;
    endcase
end

always @(posedge clk_memc)
begin
    if (rst_memc_i)
        P_strobe_r <= 0;
    else if (!P_strobe_empty)
        P_strobe_r <= P_strobe_out;
end

//=======================================================
//  FSM P_addr of MEMC clock domain output
//=======================================================
always @(posedge clk_memc)
begin
    if (rst_memc_i)
        V <= P_addr_memc_out_IDLE;
    else
        V <= V_nxt;
end

always @(*)
begin
    case (V)
        P_addr_memc_out_IDLE: V_nxt = (!P_addr_empty)? P_addr_memc_out_WAIT : P_addr_memc_out_IDLE;
        P_addr_memc_out_WAIT: V_nxt = (P_memc_out_ready)? P_addr_memc_out_IDLE : P_addr_memc_out_WAIT;
    endcase
end

always @(posedge clk_memc)
begin
    if (rst_memc_i)
        P_addr_r <= 0;
    else if (!P_addr_empty)
        P_addr_r <= P_addr_out;
end

//=======================================================
//  FSM P_rw of MEMC clock domain output
//=======================================================
always @(posedge clk_memc)
begin
    if (rst_memc_i)
        W <= P_rw_memc_out_IDLE;
    else
        W <= W_nxt;
end

always @(*)
begin
    case (W)
        P_rw_memc_out_IDLE: W_nxt = (!P_rw_empty)? P_rw_memc_out_WAIT : P_rw_memc_out_IDLE;
        P_rw_memc_out_WAIT: W_nxt = (P_memc_out_ready)? P_rw_memc_out_IDLE : P_rw_memc_out_WAIT;
    endcase
end

always @(posedge clk_memc)
begin
    if (rst_memc_i)
        P_rw_r <= 0;
    else if (!P_rw_empty)
        P_rw_r <= P_rw_out;
end

//=======================================================
//  FSM P_wt_data of MEMC clock domain output
//=======================================================
always @(posedge clk_memc)
begin
    if (rst_memc_i)
        X <= P_wt_data_memc_out_IDLE;
    else
        X <= X_nxt;
end

always @(*)
begin
    case (X)
        P_wt_data_memc_out_IDLE: X_nxt = (!P_wt_data_empty)? P_wt_data_memc_out_WAIT : P_wt_data_memc_out_IDLE;
        P_wt_data_memc_out_WAIT: X_nxt = (P_memc_out_ready)? P_wt_data_memc_out_IDLE : P_wt_data_memc_out_WAIT;
    endcase
end

always @(posedge clk_memc)
begin
    if (rst_memc_i)
        P_wt_data_r <= 0;
    else if (!P_wt_data_empty)
        P_wt_data_r <= P_wt_data_out;
end

/* *******************************************************************************
 * FSM from mig to fifo                                                          *
 * *******************************************************************************/
//=======================================================
//  FSM DMEM of MEMC clock domain input
//=======================================================
wire P_done_in_wr_en;
assign P_done_in_wr_en = (B == P_memc_in_IDLE && B_nxt == P_memc_in_WAIT);

always @(posedge clk_memc)
begin
    if (rst_memc_i)
        B <= P_memc_in_IDLE;
    else
        B <= B_nxt;
end


always @(*)
begin
    case (B)
        P_memc_in_IDLE: 
            if (P_done_i_r & !P_done_full) begin
                if (!P_rw_r) begin //read means need to wait rdata
                    if (!P_rd_data_full) begin
                        B_nxt = P_memc_in_WAIT;
                    end 
                    else begin
                        B_nxt = P_memc_in_IDLE;
                    end
                end 
                else begin
                        B_nxt = P_memc_in_WAIT;
                end
            end 
            else begin
                B_nxt = P_memc_in_IDLE;
            end
            
        P_memc_in_WAIT: B_nxt = (P_done_i_r)? P_memc_in_WAIT : P_memc_in_IDLE;
    endcase
end

/* *******************************************************************************
 * Finite State Machines from fifo to cpu                                        *
 * *******************************************************************************/
//=======================================================
//  FSM DMEM of Core clock domain output
//=======================================================
always @(posedge clk_core)
begin
    if (rst_core_i)
        D <= P_core_out_IDLE;
    else
        D <= D_nxt;
end


always @(*)
begin
    case (D)
        P_core_out_IDLE: 
            if (F == P_done_core_out_WAIT) begin
                if (!P_rw_i) begin //read means need to wait rdata
                    if (G == P_rd_data_core_out_WAIT) begin
                        D_nxt = P_core_out_WAIT;
                    end 
                    else begin
                        D_nxt = P_core_out_IDLE;
                    end
                end 
                else begin
                    D_nxt = P_core_out_WAIT;
                end
            end 
            else begin
                D_nxt = P_core_out_IDLE;
            end

        P_core_out_WAIT: D_nxt = P_core_out_IDLE;
    endcase
end

//=======================================================
//  FSM P_done of Core clock domain output
//=======================================================
always @(posedge clk_core)
begin
    if (rst_core_i)
        F <= P_done_core_out_IDLE;
    else
        F <= F_nxt;
end

always @(*)
begin
    case (F)
        P_done_core_out_IDLE: F_nxt = (!P_done_empty)? P_done_core_out_WAIT : P_done_core_out_IDLE;
        P_done_core_out_WAIT: F_nxt = (P_core_out_ready)? P_done_core_out_IDLE : P_done_core_out_WAIT;
    endcase
end

always @(posedge clk_core)
begin
    if (rst_core_i)
        P_done_r <= 0;
    else if (!P_done_empty)
        P_done_r <= P_done_out;
end

//=======================================================
//  FSM P_rd_data of Core clock domain output
//=======================================================
always @(posedge clk_core)
begin
    if (rst_core_i)
        G <= P_rd_data_core_out_IDLE;
    else
        G <= G_nxt;
end

always @(*)
begin
    case (G)
        P_rd_data_core_out_IDLE: G_nxt = (!P_rd_data_empty)? P_rd_data_core_out_WAIT : P_rd_data_core_out_IDLE;
        P_rd_data_core_out_WAIT: G_nxt = (P_core_out_ready)? P_rd_data_core_out_IDLE : P_rd_data_core_out_WAIT;
    endcase
end

always @(posedge clk_core)
begin
    if (rst_core_i)
        P_rd_data_r <= 0;
    else if (!P_rd_data_empty)
        P_rd_data_r <= P_rd_data_out;
end


/* *******************************************************************************
 * Async FIFO mofules                                                            *
 * *******************************************************************************/
async_fifo_signal P_strobe
(
    .full(P_strobe_full),
    .din(P_strobe_i),
    .wr_en(P_in_wr_en),
    .empty(P_strobe_empty),
    .dout(P_strobe_out),
    .rd_en(!P_strobe_empty),
    .wr_clk(clk_core),
    .rd_clk(clk_memc)
);

async_fifo_signal P_rw
(
    .full(P_rw_full),
    .din(P_rw_i),
    .wr_en(P_in_wr_en),
    .empty(P_rw_empty),
    .dout(P_rw_out),
    .rd_en(!P_rw_empty),
    .wr_clk(clk_core),
    .rd_clk(clk_memc)
);

async_fifo_addr P_addr
(
    .full(P_addr_full),
    .din(P_addr_i),
    .wr_en(P_in_wr_en),
    .empty(P_addr_empty),
    .dout(P_addr_out),
    .rd_en(!P_addr_empty),
    .wr_clk(clk_core),
    .rd_clk(clk_memc)
);

async_fifo_data P_wt_data
(
    .full(P_wt_data_full),
    .din(P_wt_data_i),
    .wr_en(P_in_wr_en),
    .empty(P_wt_data_empty),
    .dout(P_wt_data_out),
    .rd_en(!P_wt_data_empty),
    .wr_clk(clk_core),
    .rd_clk(clk_memc)
);

async_fifo_signal P_done
(
    .full(P_done_full),
    .din(P_done_i_r),
    .wr_en(P_done_in_wr_en),
    .empty(P_done_empty),
    .dout(P_done_out),
    .rd_en(!P_done_empty),
    .wr_clk(clk_memc),
    .rd_clk(clk_core)
);

async_fifo_data P_rd_data
(
    .full(P_rd_data_full),
    .din(P_rd_data_i_r),
    .wr_en(!P_rw_r & P_done_in_wr_en),
    .empty(P_rd_data_empty),
    .dout(P_rd_data_out),
    .rd_en(!P_rd_data_empty),
    .wr_clk(clk_memc),
    .rd_clk(clk_core)
);
endmodule
