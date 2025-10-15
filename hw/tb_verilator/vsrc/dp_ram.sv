module dp_ram #(
    parameter int ADDR_WIDTH = 32,
    parameter int DATA_WIDTH = 128,
    parameter int MEM_SIZE  /*verilator public*/ = 32'h40000000,  // 1 GB
    parameter int ACCESS_LANTENCY = 32'h04,  // done signal control
    parameter int MEM_OFFSET  /*verilator public*/ = 32'h80000000
) (
    // Clock and Reset
    input logic clk,
    input logic rst_n,

    input  logic                  strobe_imem,
    input  logic [ADDR_WIDTH-1:0] addr_imem_i,
    output logic [DATA_WIDTH-1:0] rdata_imem_o,
    output logic                  done_imem_o,

    input  logic                  strobe_dmem,
    input  logic [ADDR_WIDTH-1:0] addr_dmem_i,
    input  logic [DATA_WIDTH-1:0] wdata_dmem_i,
    output logic [DATA_WIDTH-1:0] rdata_dmem_o,
    input  logic                  rw_dmem_i,
    output logic                  done_dmem_o
);

    typedef enum logic [1:0] {
        IDLE  = 2'b0,
        READ,
        WRITE,
        DONE
    } state_t;

    function integer clogb2(input integer bit_depth);
        begin
            for (clogb2 = 0; bit_depth > 0; clogb2 = clogb2 + 1) bit_depth = bit_depth >> 1;
        end
    endfunction

    localparam WORD_ADDR_WIDTH = clogb2(MEM_SIZE / 4);

    state_t imem_cur_state, imem_next_state;
    state_t dmem_cur_state, dmem_next_state;

    logic [WORD_ADDR_WIDTH-1:0] imem_addr;
    logic [WORD_ADDR_WIDTH-1:0] dmem_addr;
    logic [ADDR_WIDTH-1:0] tmp_addr_imem;
    logic [ADDR_WIDTH-1:0] tmp_addr_dmem;
    logic [31:0] mem[MEM_SIZE/4-1:0]  /*verilator public_flat*/;
    logic [7:0] imem_delay_counter;
    logic [7:0] dmem_delay_counter;
    logic imem_reads_done;
    logic dmem_reads_done;
    logic dmem_writes_done;

    integer i;
    /* initial begin */
    /*     for (i = 0; i < MEM_SIZE / 4; i = i + 1) mem[i] = 32'hdeadbeef; */
    /* end */

    always_comb tmp_addr_imem = addr_imem_i - MEM_OFFSET;
    always_comb tmp_addr_dmem = addr_dmem_i - MEM_OFFSET;

    always_comb imem_addr = tmp_addr_imem[WORD_ADDR_WIDTH+2-1:2];
    always_comb dmem_addr = tmp_addr_dmem[WORD_ADDR_WIDTH+2-1:2];

    always_ff @(posedge clk) begin
        if (imem_cur_state == IDLE || imem_cur_state == DONE) imem_delay_counter <= 0;
        else imem_delay_counter <= imem_delay_counter + 1;
    end

    always_ff @(posedge clk) begin
        if (dmem_cur_state == IDLE || dmem_cur_state == DONE) dmem_delay_counter <= 0;
        else dmem_delay_counter <= dmem_delay_counter + 1;
    end

    always_comb imem_reads_done = imem_delay_counter >= ACCESS_LANTENCY;

    always_comb dmem_reads_done = dmem_delay_counter >= ACCESS_LANTENCY;
    always_comb dmem_writes_done = dmem_delay_counter >= ACCESS_LANTENCY;

    always_ff @(posedge clk) begin
        if (~rst_n) imem_cur_state <= IDLE;
        else imem_cur_state <= imem_next_state;
    end

    always_ff @(posedge clk) begin
        if (~rst_n) dmem_cur_state <= IDLE;
        else dmem_cur_state <= dmem_next_state;
    end

    always_comb begin
        case (imem_cur_state)
            IDLE:
            if (strobe_imem) imem_next_state = READ;
            else imem_next_state = IDLE;
            READ:
            if (imem_reads_done) imem_next_state = DONE;
            else imem_next_state = READ;
            DONE: imem_next_state = IDLE;
            default: imem_next_state = IDLE;
        endcase
    end

    always_comb begin
        case (dmem_cur_state)
            IDLE:
            if (strobe_dmem) dmem_next_state = (rw_dmem_i ? WRITE : READ);
            else dmem_next_state = IDLE;
            READ:
            if (dmem_reads_done) dmem_next_state = DONE;
            else dmem_next_state = READ;
            WRITE:
            if (dmem_writes_done) dmem_next_state = DONE;
            else dmem_next_state = WRITE;
            DONE: dmem_next_state = IDLE;
            default: dmem_next_state = IDLE;
        endcase
    end

    always_ff @(posedge clk) begin
        if (dmem_cur_state == WRITE) begin
            // mem[dmem_addr+0] <= wdata_dmem_i[255:224];
            // mem[dmem_addr+1] <= wdata_dmem_i[223:192];
            // mem[dmem_addr+2] <= wdata_dmem_i[191:160];
            // mem[dmem_addr+3] <= wdata_dmem_i[159:128];
            // mem[dmem_addr+4] <= wdata_dmem_i[127: 96];
            // mem[dmem_addr+5] <= wdata_dmem_i[ 95: 64];
            // mem[dmem_addr+6] <= wdata_dmem_i[ 63: 32];
            // mem[dmem_addr+7] <= wdata_dmem_i[ 31:  0];
            mem[dmem_addr+0] <= wdata_dmem_i[127: 96];
            mem[dmem_addr+1] <= wdata_dmem_i[ 95: 64];
            mem[dmem_addr+2] <= wdata_dmem_i[ 63: 32];
            mem[dmem_addr+3] <= wdata_dmem_i[ 31:  0];
        end
    end

    always_ff @(posedge clk) begin
        if (dmem_cur_state == READ) begin
            rdata_dmem_o <= {
                mem[dmem_addr+0],
                mem[dmem_addr+1],
                mem[dmem_addr+2],
                mem[dmem_addr+3]
                // mem[dmem_addr+0],
                // mem[dmem_addr+1],
                // mem[dmem_addr+2],
                // mem[dmem_addr+3],
                // mem[dmem_addr+4],
                // mem[dmem_addr+5],
                // mem[dmem_addr+6],
                // mem[dmem_addr+7]
            };
        end else begin
            rdata_dmem_o <= rdata_dmem_o;
        end
    end

    always_ff @(posedge clk) begin
        if (imem_cur_state == READ) begin
            rdata_imem_o <= {
                mem[imem_addr+0],
                mem[imem_addr+1],
                mem[imem_addr+2],
                mem[imem_addr+3]
                // mem[imem_addr+0],
                // mem[imem_addr+1],
                // mem[imem_addr+2],
                // mem[imem_addr+3],
                // mem[imem_addr+4],
                // mem[imem_addr+5],
                // mem[imem_addr+6],
                // mem[imem_addr+7]
            };
        end else begin
            rdata_imem_o <= rdata_imem_o;
        end
    end

    //to deal with strobe delay one cycle fall issue
    always_comb done_imem_o = (imem_cur_state == READ && imem_reads_done);

    //to deal with strobe delay one cycle fall issue
    always_comb
        done_dmem_o = (dmem_cur_state == READ && dmem_reads_done) || (dmem_cur_state == WRITE && dmem_writes_done);

    function [31:0] readWord;
        input integer byte_addr;
        begin
            reg [31:0] tmp_addr;
            tmp_addr = byte_addr + MEM_OFFSET;
            if (byte_addr >= MEM_OFFSET && byte_addr < MEM_OFFSET + MEM_SIZE)
                readWord = {
                    mem[tmp_addr[WORD_ADDR_WIDTH-1:0]],
                    mem[tmp_addr[WORD_ADDR_WIDTH-1:0]+1],
                    mem[tmp_addr[WORD_ADDR_WIDTH-1:0]+2],
                    mem[tmp_addr[WORD_ADDR_WIDTH-1:0]+3]
                };
            else readWord = 32'hdeadbeef;
        end
    endfunction

    task writeWord;
        /*verilator public*/
        input integer byte_addr;
        input [31:0] val;
        begin
            reg [31:0] tmp_addr;
            reg [WORD_ADDR_WIDTH-1:0] word_addr;
            tmp_addr  = byte_addr + MEM_OFFSET;
            word_addr = tmp_addr[WORD_ADDR_WIDTH+2-1:2];
            if (byte_addr >= MEM_OFFSET && byte_addr < MEM_OFFSET + MEM_SIZE) begin
                mem[word_addr] = val;
            end
        end
    endtask

    task writeByte;
        /*verilator public*/
        input integer byte_addr;
        input [7:0] val;
        begin
            reg [31:0] tmp_addr;
            tmp_addr = byte_addr - MEM_OFFSET;
            if (byte_addr >= MEM_OFFSET && byte_addr < MEM_OFFSET + MEM_SIZE) begin
                mem[tmp_addr[WORD_ADDR_WIDTH-1:0]] = val;
            end
        end
    endtask

    function [7:0] readByte;
        /*verilator public*/
        input integer byte_addr;
        begin
            reg [31:0] tmp_addr;
            tmp_addr = byte_addr - MEM_OFFSET;
            if (byte_addr >= MEM_OFFSET && byte_addr < MEM_OFFSET + MEM_SIZE)
                readByte = mem[tmp_addr[WORD_ADDR_WIDTH-1:0]];
            else readByte = 8'hef;  //dummy
        end
    endfunction
endmodule
