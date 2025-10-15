module intc #(
    parameter integer C_M_AXI_ADDR_WIDTH = 32,  // Width of the AXI addr bus.
    parameter integer C_M_AXI_DATA_WIDTH = 32,  // Width of the AXI data bus.
    parameter integer NUM_OF_IRQS        = 1,   // number of interrupts.
    parameter integer AXI_LANTENCY = 10
) (
    input clk,   // AXI bus clock signal.
    input rst_n, // AXI bus reset singal (Active Low).

    // Interface signals between Aquila and the AXI-Lite device master port.
    input                                M_DEVICE_strobe /* verilator public */,
    input   [C_M_AXI_DATA_WIDTH - 1 : 0] M_DEVICE_addr /* verilator public */,
    input                                M_DEVICE_rw /* verilator public */,
    input   [C_M_AXI_DATA_WIDTH/8-1 : 0] M_DEVICE_byte_enable,    //don't care
    input   [C_M_AXI_DATA_WIDTH - 1 : 0] M_DEVICE_core2dev_data /* verilator public */ ,
    output  reg                              M_DEVICE_data_ready /* verilator public */,
    output  reg [C_M_AXI_DATA_WIDTH - 1 : 0] M_DEVICE_dev2core_data /* verilator public */,

    input   [NUM_OF_IRQS-1 : 0]          intr,
    output  reg                          irq
);


    localparam [1:0] 
        S_IDLE  = 2'b00,
        S_READ  = 2'b01,
        S_WRITE = 2'b10,
        S_DONE  = 2'b11;

    reg [1:0] cur_state, next_state;
    reg [7:0] read_counter, write_counter;
    wire reads_done, writes_done;

    assign reads_done = (read_counter >= AXI_LANTENCY);
    assign writes_done = (write_counter >= AXI_LANTENCY);
    
    always @(posedge clk) begin
        if (~rst_n) begin
            cur_state <= S_IDLE;
        end else begin
            cur_state <= next_state;
        end
    end

    always @(*) begin
        case (cur_state)
            S_IDLE:
                if (M_DEVICE_strobe) begin
                    next_state = (M_DEVICE_rw) ? S_WRITE : S_READ;
                end else begin
                    next_state = S_IDLE;
                end
            S_READ:
                if (reads_done) begin
                    next_state = S_DONE;
                end else begin
                    next_state = S_READ;
                end
            S_WRITE:
                if (writes_done) begin
                    next_state = S_DONE;
                end else begin
                    next_state = S_WRITE;
                end
            S_DONE:
                next_state = S_IDLE;
            default:
                next_state = S_IDLE;
        endcase
    end

    always @(posedge clk) begin
        if (~rst_n) begin
            read_counter <= 0;
        end else if (cur_state == S_IDLE || cur_state == S_DONE || cur_state == S_WRITE) begin
            read_counter <= 0;
        end else begin
            read_counter <= read_counter + 1;
        end
    end

    always @(posedge clk) begin
        if (~rst_n) begin
            write_counter <= 0;
        end else if (cur_state == S_IDLE || cur_state == S_DONE || cur_state == S_READ) begin
            write_counter <= 0;
        end else begin
            write_counter <= write_counter + 1;
        end
    end

    localparam [7:0] ISR_OFFSET = 8'h00;
    localparam [7:0] IPR_OFFSET = 8'h04;
    localparam [7:0] IER_OFFSET = 8'h08;
    localparam [7:0] IAR_OFFSET = 8'h0C;
    localparam [7:0] SIE_OFFSET = 8'h10;
    localparam [7:0] CIE_OFFSET = 8'h14;
    localparam [7:0] IVR_OFFSET = 8'h18;
    localparam [7:0] MER_OFFSET = 8'h1C;
    localparam [7:0] IMR_OFFSET = 8'h20;
    localparam [7:0] ILR_OFFSET = 8'h24;


    wire [7:0] access_addr;
    wire isr_write;
    wire ipr_write;
    wire ier_write;
    wire iar_write;
    wire sie_write;
    wire cie_write;
    wire ivr_write;
    wire mer_write;
    wire imr_write;
    wire ilr_write;

    assign access_addr = M_DEVICE_addr[7:0];
    assign isr_write = ((cur_state == S_WRITE) && (access_addr == ISR_OFFSET));
    assign ipr_write = ((cur_state == S_WRITE) && (access_addr == IPR_OFFSET));
    assign ier_write = ((cur_state == S_WRITE) && (access_addr == IER_OFFSET));
    assign iar_write = ((cur_state == S_WRITE) && (access_addr == IAR_OFFSET));
    assign sie_write = ((cur_state == S_WRITE) && (access_addr == SIE_OFFSET));
    assign cie_write = ((cur_state == S_WRITE) && (access_addr == CIE_OFFSET));
    assign ivr_write = ((cur_state == S_WRITE) && (access_addr == IVR_OFFSET));
    assign mer_write = ((cur_state == S_WRITE) && (access_addr == MER_OFFSET));
    assign imr_write = ((cur_state == S_WRITE) && (access_addr == IMR_OFFSET));
    assign ilr_write = ((cur_state == S_WRITE) && (access_addr == ILR_OFFSET));

    reg [31:0] isr;
    reg [31:0] ipr;
    reg [31:0] ier;
    reg [31:0] iar;
    reg [31:0] sie;
    reg [31:0] cie;
    reg [31:0] ivr;
    reg [31:0] mer;
    reg [31:0] imr;
    reg [31:0] ilr;

    wire me;
    wire hie;
    wire has_enable_interrupt;
    integer i;

    assign has_enable_interrupt = |(ier && intr);

    always @(posedge clk) begin
        if (~rst_n) begin
            isr <= 0;
        end else if (isr_write && !hie) begin
            isr <= M_DEVICE_core2dev_data;
        end else if (me && hie && has_enable_interrupt) begin
            isr <= (isr || intr);
        end else if (iar_write) begin
            for (i = 0; i < NUM_OF_IRQS; i++) begin
                isr[i] <= (M_DEVICE_core2dev_data[i]) ? 0 : isr[i];
            end
        end
    end

    always @(posedge clk) begin
        if (~rst_n) begin
            ipr <= 0;
        end
    end
    
    always @(posedge clk) begin
        if (~rst_n) begin
            ier <= 0;
        end else if (ier_write) begin
            ier <= M_DEVICE_core2dev_data;
        end else if (sie_write) begin
            ier <= M_DEVICE_core2dev_data;
        end else if (cie_write) begin
            ier <= M_DEVICE_core2dev_data;
        end
    end

    always @(posedge clk) begin
        if (~rst_n) begin
            iar <= 0;
        end else if (iar_write) begin
            iar <= M_DEVICE_core2dev_data;
        end else if (cur_state == S_DONE) begin
            iar <= 0;
        end
    end

    always @(posedge clk) begin
        if (~rst_n) begin
            sie <= 0;
        end else if (sie_write) begin
            sie <= M_DEVICE_core2dev_data;
        end
    end
    always @(posedge clk) begin
        if (~rst_n) begin
            cie <= 0;
        end else if (cie_write) begin
            cie <= M_DEVICE_core2dev_data;
        end
    end
    always @(posedge clk) begin
        if (~rst_n) begin
            ivr <= 32'hffffffff;
        end else if (|intr) begin
            ivr <= 0;
        end else if (iar_write) begin
            ivr <= 32'hffffffff;
        end
    end
    always @(posedge clk) begin
        if (~rst_n) begin
            mer <= 0;
        end else if (mer_write) begin
            mer[0] <= M_DEVICE_core2dev_data[0];
            mer[1] <= (hie)? 1 : M_DEVICE_core2dev_data[1];
        end
    end
    always @(posedge clk) begin
        if (~rst_n) begin
            imr <= 0;
        end else if (imr_write) begin
            imr <= M_DEVICE_core2dev_data;
        end
    end
    always @(posedge clk) begin
        if (~rst_n) begin
            ilr <= 0;
        end else if (ilr_write) begin
            ilr <= M_DEVICE_core2dev_data;
        end
    end

    assign me = mer[0];
    assign hie = mer[1];

    reg [31:0] read_data;
    /* ==== READ ===== */
    always @(*) begin
        case (access_addr)
            ISR_OFFSET: read_data = isr;
            IPR_OFFSET: read_data = ipr;
            IER_OFFSET: read_data = ier;
            IAR_OFFSET: read_data = iar;
            SIE_OFFSET: read_data = sie;
            CIE_OFFSET: read_data = cie;
            IVR_OFFSET: read_data = ivr;
            MER_OFFSET: read_data = mer;
            IMR_OFFSET: read_data = imr;
            ILR_OFFSET: read_data = ilr;
            default: read_data = 32'hdeadbeaf;
        endcase
    end

    always @(posedge clk) begin
        if (~rst_n) begin
            M_DEVICE_dev2core_data <= 32'h0;
        end else if (cur_state == S_READ) begin
            M_DEVICE_dev2core_data <= read_data;
        end
    end

    always @(posedge clk) begin
        if (~rst_n) begin
            M_DEVICE_data_ready <= 1'b0;
        end else if (cur_state == S_IDLE) begin
            M_DEVICE_data_ready <= 1'b0;
        end else if (cur_state == S_DONE) begin
            M_DEVICE_data_ready <= 1'b1;
        end
    end


    /* ===== IRQ ===== */
    always @(posedge clk) begin
        if (~rst_n) begin
            irq <= 0;
        end else if (|isr) begin
            irq <= 1;
        end else begin
            irq <= 0;
        end
    end

        
endmodule
