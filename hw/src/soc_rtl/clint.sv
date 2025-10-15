`timescale 1ns / 1ps
// =============================================================================
//  Program : clint.sv
//  Author  : Jin-you Wu
//  Date    : Feb/28/2019
// -----------------------------------------------------------------------------
//  Description:
//  This module implements the RISC-V Core Local Interrupt (CLINT) Controller.
//  The ticking signal is currently fixed to the CPU clock instead of an external
//  RTC clock. The OS (e.g., FreeRTOS) must set the frequency of clk_i to the
//  OS timer parameter properly (e.g., configCPU_CLOCK_HZ).
// -----------------------------------------------------------------------------
//  Revision information:
//
//  Aug/24/2022, by Chun-Jen Tsai:
//    Remove the TIMER parameter and use the CPU clock to drive CLINT.
//    Previous code assumes that the interrupt generator is driven by a
//    1000 Hz clock (i.e. 1 msec ticks) and the TIMER parameter is
//    used to set the number of CPU ticks within 1 msec.  Unfortunately,
//    this design does not match the popular SiFive CLINT behavior.
//
//
//  Aug/21/2025, by Tzu-Chen Yang:
//    Modified to support multi-core operation. CLINT is now global, 
//    allowing each core to access it (via an arbiter in the SoC top).
//    Each core maintains its own mtimecmp and msip registers,
//    while sharing a common mtime register. Register addresses are updated to 
//    follow SiFive recommendations. 
//    (https://sifive.cdn.prismic.io/sifive%2Fc89f6e5a-cf9e-44c3-a3db-04420702dcc1_sifive+e31+manual+v19.08.pdf)
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

module clint
#( parameter XLEN = 32 ,  // Width of address bus
    parameter CORE_NUMS = `CORE_NUMS
)
(
    input                   clk_i,
    input                   rst_i,

    input                   en_i,
    input                   we_i,
    input [XLEN-1 : 0]      addr_i,
    input [XLEN-1 : 0]      data_i,
    output reg [XLEN-1 : 0] data_o,
    output reg              data_ready_o,

    output                  tmr_irq_o[0 : CORE_NUMS-1],
    output                  sft_irq_o[0 : CORE_NUMS-1]
);
// 0xF0000000 ~ 0xF000003C: msip register of hart 0 ~ hart 15
// 0xF0004000 ~ 0xF000407C: mtimecmp register of hart 0 ~ hart 15
// 0xF000BFF8, 0xF000BFFC: mtime register

reg  [XLEN-1 : 0] msip_mem[0 : CORE_NUMS-1];
reg  [XLEN-1 : 0] mtimecmp_mem[0 : CORE_NUMS * 2 - 1];
reg  [XLEN-1 : 0] mtime_mem[0:1];
wire [63:0] mtime = { mtime_mem[1], mtime_mem[0] };
wire [63:0] mtimecmp [0 : CORE_NUMS-1];

wire [XLEN-1 : 0] msip [0 : CORE_NUMS-1];

wire carry = (mtime_mem[0] == 32'hFFFF_FFFF);

genvar idx;
generate
    for(idx = 0; idx < CORE_NUMS; idx = idx + 1)
    begin
        assign mtimecmp[idx] = { mtimecmp_mem[2*idx+1], mtimecmp_mem[2*idx] };
        assign msip[idx] = msip_mem[idx];
        assign tmr_irq_o[idx] = (mtime >= mtimecmp[idx]) & (| mtimecmp[idx]);
        assign sft_irq_o[idx] = | msip[idx];
    end
endgenerate

integer i;
always @(posedge clk_i)
begin
    if (rst_i)
    begin
        for(i = 0;i < CORE_NUMS;i = i + 1)begin
            mtimecmp_mem[2*i] <= 32'b0;
            mtimecmp_mem[2*i+1] <= 32'b0;
            msip_mem[i] <= 32'b0;
        end
        mtime_mem[0] <= 32'b0;
        mtime_mem[1] <= 32'b0;
    end
    else if (en_i && we_i)
    begin
        if(addr_i[15:12] == 4'h0) begin
            msip_mem[addr_i[5:2]] <= data_i;
        end
        else if(addr_i[15:12] == 4'h4) begin
            mtimecmp_mem[addr_i[7:2]] <= data_i;
        end
        else if(addr_i[15:4] == 12'hBFF) begin
            mtime_mem[addr_i[2]] <= data_i;
        end
    end
    else
    begin
        mtime_mem[0] <= mtime_mem[0] + 1;
        mtime_mem[1] <= mtime_mem[1] + carry;
    end
end

always @(posedge clk_i)
begin
    if (en_i)
    begin
        if(addr_i[15:12] == 4'h0) begin
            data_o <= msip_mem[addr_i[5:2]];
        end
        else if(addr_i[15:12] == 4'h4) begin
            data_o <= mtimecmp_mem[addr_i[7:2]];
        end
        else if(addr_i[15:4] == 12'hBFF) begin
            data_o <= mtime_mem[addr_i[2]];
        end
        data_ready_o <= 1;   // CJ Tsai 0306_2020: Add ready signal for bus masters.
    end
    else
        data_ready_o <= 0;
end

endmodule // clint
