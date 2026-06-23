//==========================================================================
//  sync_fifo.v  -- Single-clock (synchronous) FIFO
//
//  - Circular buffer in a register array (RAM)
//  - Pointers are (ADDR_WIDTH+1) bits: the extra MSB lets us tell
//    "full" apart from "empty" when the low bits are equal.
//  - Read is REGISTERED (1-cycle latency): dout is valid the cycle
//    AFTER rd_en is asserted.
//  - DEPTH must be a power of two.
//==========================================================================
module sync_fifo #(
    parameter integer DATA_WIDTH = 8,
    parameter integer DEPTH      = 16,            // power of 2
    parameter integer ADDR_WIDTH = $clog2(DEPTH)
)(
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  wr_en,
    input  wire                  rd_en,
    input  wire [DATA_WIDTH-1:0] din,
    output reg  [DATA_WIDTH-1:0] dout,
    output wire                  full,
    output wire                  empty
);

    // storage
    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];

    // pointers carry one extra MSB ("wrap" bit)
    reg [ADDR_WIDTH:0] wr_ptr;
    reg [ADDR_WIDTH:0] rd_ptr;

    wire do_write = wr_en & ~full;
    wire do_read  = rd_en & ~empty;

    // ---- write port ----
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            wr_ptr <= {(ADDR_WIDTH+1){1'b0}};
        else if (do_write) begin
            mem[wr_ptr[ADDR_WIDTH-1:0]] <= din;
            wr_ptr <= wr_ptr + 1'b1;
        end
    end

    // ---- read port (registered output) ----
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_ptr <= {(ADDR_WIDTH+1){1'b0}};
            dout   <= {DATA_WIDTH{1'b0}};
        end else if (do_read) begin
            dout   <= mem[rd_ptr[ADDR_WIDTH-1:0]];
            rd_ptr <= rd_ptr + 1'b1;
        end
    end

    // ---- flags ----
    // empty: pointers identical (same wrap bit + same address)
    assign empty = (wr_ptr == rd_ptr);
    // full:  same address but wrap bits differ (writer is one lap ahead)
    assign full  = (wr_ptr[ADDR_WIDTH]     != rd_ptr[ADDR_WIDTH]) &&
                   (wr_ptr[ADDR_WIDTH-1:0] == rd_ptr[ADDR_WIDTH-1:0]);

endmodule
