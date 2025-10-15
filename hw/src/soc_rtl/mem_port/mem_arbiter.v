// =============================================================================
//  Program : mem_arbiter.v
//  Author  : Yi-De Lee
//  Date    : Apr/9/2022
// -----------------------------------------------------------------------------
//  Description:
//      The arbiter for mig native interface.
// -----------------------------------------------------------------------------
//  Notes for UG586 7 Series FPGAs Memory Interface Solutions User Guide
//      - app_rdy      : indicates mig user interface is ready to accept command (if app_rdy change from 1 to 0, it means command is accepted)
//      - app_addr     : every address represent 8 bytes, there is only 1GB DDR in KC705 so the address space is 0x07FFFFFF ~ 0x00000000
//      - app_cmd      : 3'b000 for write command, 3'b001 for read command
//      - app_en       : request strobe to mig, app_addr and app_cmd need to be ready
//      - app_wdf_rdy  : indicates that the write data FIFO is ready to receive data
//      - app_wdf_mask : mask for wdf_data, 0 indicates overwrite byte, 1 indicates keep byte
//      - app_wdf_data : data need to be written into ddr
//      - app_wdf_wren : indicates that data on app_wdf_data is valid
//      - app_wdf_end  : indicates that current clock cycle is the last cycle of input data on wdf_data
// -----------------------------------------------------------------------------
//  Revision information:
//
//  Sep/6/2022, by Chun-Jen Tsai:
//    Modify the code to turn constants into parameters so that we can support
//    both the KC705 with CLSIZE = 256 and the Arty boards with CL_SIZE = 128.
//
//    The DRAM parameters on the Arty A7-35T and A7-100T boards:
//    128-bit memory bus, 16-bit word length, 8-beat burst, and 333 MHz memory clock.
//    The DRAM parameters on the Xilinx KC-705 boards:
//    512-bit memory bus, 64-bit word length, 8-beat burst, and 800 MHz memory clock.
//
//  Summer/2024, by Chun-Jen Tsai:
//    Added 256-bit memory bus, 32-bit word length, and DDR4 support.
//
//  Summer/2025, by Tzu-Chen Yang and Ye Chen
//    Adapted the memory arbiter for the L2 cache.
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

module mem_arbiter #(
    parameter XLEN       = 32,
    parameter CLSIZE     = `CLP,
    parameter ADDR_WIDTH = `UIADR,
    parameter DRAM_WIDTH = `DRAMD,
    parameter WDF_DBITS  = `WDFP )
(
    // System signals
    input                      clk_i, 
    input                      rst_i,

    // Aquila P1-MEM slave interface
    input                      P0_MEM_strobe_i,
    input  [XLEN-1 : 0]        P0_MEM_addr_i,
    input                      P0_MEM_rw_i,
    input  [CLSIZE-1 : 0]      P0_MEM_data_i,
    output                     P0_MEM_done_o,
    output [CLSIZE-1 : 0]      P0_MEM_data_o,

    // Aquila P2-MEM slave interface
    input                      P1_MEM_strobe_i,
    input  [XLEN-1 : 0]        P1_MEM_addr_i,
    input                      P1_MEM_rw_i,
    input  [CLSIZE-1 : 0]      P1_MEM_data_i,
    output                     P1_MEM_done_o,
    output [CLSIZE-1 : 0]      P1_MEM_data_o,

    // Xilinx MIG memory controller native user interface
    output [ADDR_WIDTH-1 : 0]  M_MEM_addr_o,
    output [  2 : 0]           M_MEM_cmd_o,
    output                     M_MEM_en_o,
    output [WDF_DBITS-1 : 0]   M_MEM_wdf_data_o,
    output                     M_MEM_wdf_end_o,
    output [WDF_DBITS/8-1 : 0] M_MEM_wdf_mask_o,
    output                     M_MEM_wdf_wren_o,
    input  [WDF_DBITS-1 : 0]   M_MEM_rd_data_i,
    input                      M_MEM_rd_data_valid_i,
    input                      M_MEM_rdy_i,
    input                      M_MEM_wdf_rdy_i,
    output                     M_MEM_sr_req_o,
    output                     M_MEM_ref_req_o,
    output                     M_MEM_zq_req_o
);

    // Strobe source, have two strobe sources (P0-MEM, P1-MEM) 
    localparam P0_STROBE = 0,
               P1_STROBE = 1;

    localparam M_IDLE   = 0, // wait for strobe
               M_CHOOSE = 1, // For Round-robin
               M_SEND   = 2, // wait for mig ready for request
               M_WAIT   = 3, // wait for rd data readed from ddr
               M_DONE   = 4; // delay one cycle, 'cause we need to reorder
                             // the readed data (for timing improvement)

    localparam DRAM_ADDR_LSBIT = $clog2(DRAM_WIDTH/8);

    // Keep the strobe
    reg                      P0_MEM_strobe_r;
    reg  [XLEN-1 : 0]        P0_MEM_addr_r;
    reg                      P0_MEM_rw_r;
    reg  [CLSIZE-1 : 0]      P0_MEM_data_r;


    reg                      P1_MEM_strobe_r;
    reg  [XLEN-1 : 0]        P1_MEM_addr_r;
    reg                      P1_MEM_rw_r;
    reg  [CLSIZE-1 : 0]      P1_MEM_data_r;

    // input selection signals
    wire                     concurrent_strobe;
    reg                      sel_current;
    reg                      sel_previous;

    reg                      sel;
    reg  [ADDR_WIDTH-1 : 0]  addr;
    reg                      rw;
    reg  [WDF_DBITS-1 : 0]   wdata;
    reg  [WDF_DBITS/8-1 : 0] wmask;

    // registered input signals
    reg                      sel_r;
    reg  [ADDR_WIDTH-1 : 0]  addr_r;
    reg                      rw_r;
    reg  [WDF_DBITS-1 : 0]   wdata_r;
    reg  [WDF_DBITS/8-1 : 0] wmask_r;

    // FSM signals
    reg  [  2 : 0]           c_state;
    reg  [  2 : 0]           n_state;
    wire                     have_strobe;
    wire                     mig_ready;
    wire [DRAM_WIDTH-1 : 0]  mig_data [0 : 7];

    // data readed from mig need reorder
    reg  [WDF_DBITS-1 : 0]   rdata_reorder;

    //=======================================================
    //  Keep the strobe (in case we miss strobe)
    //=======================================================
    // P0-MEM slave interface
    always @(posedge clk_i) begin
        if (rst_i)
            P0_MEM_strobe_r <= 0;
        else if (P0_MEM_strobe_i)
            P0_MEM_strobe_r <= 1;
        else if (P0_MEM_done_o)
            P0_MEM_strobe_r <= 0; // Clear the strobe
    end

    always @(posedge clk_i) begin
        if (rst_i)
            P0_MEM_addr_r <= 0;
        else if (P0_MEM_strobe_i)
            P0_MEM_addr_r <= P0_MEM_addr_i;
    end

    always @(posedge clk_i) begin
        if (rst_i)
            P0_MEM_rw_r <= 0;
        else if (P0_MEM_strobe_i)
            P0_MEM_rw_r <= P0_MEM_rw_i;
    end

    always @(posedge clk_i) begin
        if (rst_i)
            P0_MEM_data_r <= 0;
        else if (P0_MEM_strobe_i)
            P0_MEM_data_r <= P0_MEM_data_i;
    end
    
    // P1-MEM slave interface
    always @(posedge clk_i) begin
        if (rst_i)
            P1_MEM_strobe_r <= 0;
        else if (P1_MEM_strobe_i)
            P1_MEM_strobe_r <= 1;
        else if (P1_MEM_done_o)
            P1_MEM_strobe_r <= 0;
    end

    always @(posedge clk_i) begin
        if (rst_i)
            P1_MEM_addr_r <= 0;
        else if (P1_MEM_strobe_i)
            P1_MEM_addr_r <= P1_MEM_addr_i;
    end

    always @(posedge clk_i) begin
        if (rst_i)
            P1_MEM_rw_r <= 0;
        else if (P1_MEM_strobe_i)
            P1_MEM_rw_r <= P1_MEM_rw_i;
    end

    always @(posedge clk_i) begin
        if (rst_i)
            P1_MEM_data_r <= 0;
        else if (P1_MEM_strobe_i)
            P1_MEM_data_r <= P1_MEM_data_i;
    end

    //=======================================================
    //  Strobe signals selection (Round Robin)
    //=======================================================
    /*always @(*) begin
        if (P0_MEM_strobe_r) 
            sel = P0_STROBE;
        else if (P1_MEM_strobe_r) 
            sel = P1_STROBE;
        else
            sel = 0;
    end*/
    assign concurrent_strobe = P0_MEM_strobe_r & P1_MEM_strobe_r;
    always @(posedge clk_i) begin
        if (rst_i) 
            sel_previous <= 0;
        else if (c_state == M_CHOOSE)
            sel_previous <= sel_current;
    end

    always @(posedge clk_i) begin
        if (rst_i) 
            sel_current <= 0;
        else if(c_state == M_IDLE)begin
            if (concurrent_strobe) begin
                if(sel_previous == P0_STROBE)
                    sel_current <= P1_STROBE;
                else if(sel_previous == P1_STROBE)
                    sel_current <= P0_STROBE;
            end
            else begin
                if(P0_MEM_strobe_r)
                    sel_current <= P0_STROBE;
                else if(P1_MEM_strobe_r)
                    sel_current <= P1_STROBE;
            end
        end
    end

    always @(*) begin
        case (sel_current)
            P0_STROBE: addr = P0_MEM_addr_r[DRAM_ADDR_LSBIT +: ADDR_WIDTH];
            P1_STROBE: addr = P1_MEM_addr_r[DRAM_ADDR_LSBIT +: ADDR_WIDTH];
            default : addr = 0;
        endcase
    end

    always @(*) begin
        case (sel_current)
            P0_STROBE: rw = P0_MEM_rw_r;
            P1_STROBE: rw = P1_MEM_rw_r;
            default : rw = 0;
        endcase
    end

    always @(*) begin
        case (sel_current) // (for scalability, you may assign wdata only with D-MEM's case)
`ifdef DRAM_BLK_512
            P0_STROBE: wdata = (addr[2])? {P0_MEM_data_r, {CLSIZE{1'b0}}} : {{CLSIZE{1'b0}}, P0_MEM_data_r};
            P1_STROBE: wdata = (addr[2])? {P1_MEM_data_r, {CLSIZE{1'b0}}} : {{CLSIZE{1'b0}}, P1_MEM_data_r};
`else // DRAM block size is the same as the cache block size
            P0_STROBE: wdata = P0_MEM_data_r;
            P1_STROBE: wdata = P1_MEM_data_r;
`endif
            default : wdata = {WDF_DBITS{1'b0}};
        endcase
    end
    
    always @(*) begin
        case (sel_current)
`ifdef DRAM_BLK_128 // Each DRAM block is of 128-bit
            P0_STROBE: wmask = 16'h0000;
            P1_STROBE: wmask = 16'h0000;
            default  : wmask = 16'h0000;
`elsif DRAM_BLK_512 // Each DRAM block is of 512-bit
            P0_STROBE: wmask = (addr[2])? 64'h0000_0000_FFFF_FFFF : 64'hFFFF_FFFF_0000_0000;
            P1_STROBE: wmask = (addr[2])? 64'h0000_0000_FFFF_FFFF : 64'hFFFF_FFFF_0000_0000;
            default  : wmask = 64'hFFFF_FFFF_FFFF_FFFF;
`else // Each DRAM block is of 256-bit (AXKU5, K7BaseC)
            P0_STROBE: wmask = 32'h0000_0000;
            P1_STROBE: wmask = 32'h0000_0000;
            default  : wmask = 32'hFFFF_FFFF;
`endif
        endcase
    end

    //============================================================================
    //  Register selected strobe's signals in IDLE state (for timing improvement)
    //============================================================================
    always @(posedge clk_i) begin
        if (rst_i)
            sel_r <= 0;
        else if (c_state == M_CHOOSE) 
            sel_r <= sel_current;
    end

    always @(posedge clk_i) begin
        if (rst_i)
            addr_r <= 0;
        else if (c_state == M_CHOOSE)
            addr_r <= addr; 
    end

    always @(posedge clk_i) begin
        if (rst_i)
            rw_r <= 0;
        else if (c_state == M_CHOOSE)
            rw_r <= rw;
    end

    always @(posedge clk_i) begin
        if (rst_i)
            wdata_r <= 0;
        else if (c_state == M_CHOOSE)
            wdata_r <= wdata;
    end

    always @(posedge clk_i) begin
        if (rst_i)
            wmask_r <= 0;
        else if (c_state == M_CHOOSE)
            wmask_r <= wmask;
    end

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
                n_state = M_SEND;
            M_SEND: 
                if (mig_ready) 
                    n_state = (rw_r)? M_IDLE : M_WAIT;
                else 
                    n_state = M_SEND;
            M_WAIT:
                if (M_MEM_rd_data_valid_i)
                    n_state = M_DONE; // Delay the output one cycle for reorder read data
                else 
                    n_state = M_WAIT;
            M_DONE:
                n_state = M_IDLE;
        endcase
    end

    assign have_strobe = P0_MEM_strobe_r | P1_MEM_strobe_r;
    assign mig_ready   = (rw_r)? M_MEM_rdy_i && M_MEM_wdf_rdy_i : M_MEM_rdy_i;

    //=======================================================
    //  Logic for MIG native interface
    //=======================================================
    // Each MIG transaction bursts 8 words of data.
    assign mig_data[0] = M_MEM_rd_data_i[(DRAM_WIDTH-1 + DRAM_WIDTH * 0) -: DRAM_WIDTH];
    assign mig_data[1] = M_MEM_rd_data_i[(DRAM_WIDTH-1 + DRAM_WIDTH * 1) -: DRAM_WIDTH];
    assign mig_data[2] = M_MEM_rd_data_i[(DRAM_WIDTH-1 + DRAM_WIDTH * 2) -: DRAM_WIDTH];
    assign mig_data[3] = M_MEM_rd_data_i[(DRAM_WIDTH-1 + DRAM_WIDTH * 3) -: DRAM_WIDTH];
    assign mig_data[4] = M_MEM_rd_data_i[(DRAM_WIDTH-1 + DRAM_WIDTH * 4) -: DRAM_WIDTH];
    assign mig_data[5] = M_MEM_rd_data_i[(DRAM_WIDTH-1 + DRAM_WIDTH * 5) -: DRAM_WIDTH];
    assign mig_data[6] = M_MEM_rd_data_i[(DRAM_WIDTH-1 + DRAM_WIDTH * 6) -: DRAM_WIDTH];
    assign mig_data[7] = M_MEM_rd_data_i[(DRAM_WIDTH-1 + DRAM_WIDTH * 7) -: DRAM_WIDTH];

    // DDR3 read data reorder
    always @(posedge clk_i) begin
        if (rst_i)
            rdata_reorder <= {WDF_DBITS{1'b0}};
        else if (M_MEM_rd_data_valid_i) begin
`ifdef DDR3
            case(addr_r[2:0]) // DDR3 read data reordering
                0: rdata_reorder <= {mig_data[7], mig_data[6], mig_data[5], mig_data[4], mig_data[3], mig_data[2], mig_data[1], mig_data[0]};
                1: rdata_reorder <= {mig_data[6], mig_data[5], mig_data[4], mig_data[7], mig_data[2], mig_data[1], mig_data[0], mig_data[3]};
                2: rdata_reorder <= {mig_data[5], mig_data[4], mig_data[7], mig_data[6], mig_data[1], mig_data[0], mig_data[3], mig_data[2]};
                3: rdata_reorder <= {mig_data[4], mig_data[7], mig_data[6], mig_data[5], mig_data[0], mig_data[3], mig_data[2], mig_data[1]};
                4: rdata_reorder <= {mig_data[3], mig_data[2], mig_data[1], mig_data[0], mig_data[7], mig_data[6], mig_data[5], mig_data[4]};
                5: rdata_reorder <= {mig_data[2], mig_data[1], mig_data[0], mig_data[3], mig_data[6], mig_data[5], mig_data[4], mig_data[7]};       
                6: rdata_reorder <= {mig_data[1], mig_data[0], mig_data[3], mig_data[2], mig_data[5], mig_data[4], mig_data[7], mig_data[6]};
                7: rdata_reorder <= {mig_data[0], mig_data[3], mig_data[2], mig_data[1], mig_data[4], mig_data[7], mig_data[6], mig_data[5]};
            endcase
`else // No reordering for DDR4 memory
            rdata_reorder <= M_MEM_rd_data_i;
`endif
        end
    end

    //=======================================================
    //  Output logic
    //=======================================================
`ifdef DRAM_BLK_512 // DRAM UI data width is twice as wide as the cache-line width.
    assign P0_MEM_data_o = (addr_r[2])? rdata_reorder[511:256] : rdata_reorder[255:0];
    assign P1_MEM_data_o = (addr_r[2])? rdata_reorder[511:256] : rdata_reorder[255:0];
`else // DRAM UI data width equals cache-line width.
    assign P0_MEM_data_o = rdata_reorder;
    assign P1_MEM_data_o = rdata_reorder;
`endif
    assign S_NMEM_data_o = rdata_reorder;
    assign P0_MEM_done_o = (c_state != M_IDLE) && (n_state == M_IDLE) && (sel_r == P0_STROBE);
    assign P1_MEM_done_o = (c_state != M_IDLE) && (n_state == M_IDLE) && (sel_r == P1_STROBE);

    assign M_MEM_addr_o     = addr_r;
    assign M_MEM_cmd_o      = (rw_r)? 3'b000 : 3'b001;
    assign M_MEM_en_o       = (c_state == M_SEND) && mig_ready;
    assign M_MEM_wdf_data_o = wdata_r;
    assign M_MEM_wdf_wren_o = (c_state == M_SEND) && mig_ready && rw_r;
    assign M_MEM_wdf_end_o  = (c_state == M_SEND) && mig_ready && rw_r;
    assign M_MEM_wdf_mask_o = wmask_r;
    assign M_MEM_sr_req_o   = 0;
    assign M_MEM_ref_req_o  = 0;
    assign M_MEM_zq_req_o   = 0;

endmodule
