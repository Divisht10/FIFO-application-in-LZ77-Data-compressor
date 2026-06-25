//==========================================================================
//  sync_fifo.v  -- Single-clock (synchronous) FIFO
//
//  Fixed for arbitrary DEPTH (does NOT need to be a power of two).
//
//  Changes from original:
//  ─────────────────────────────────────────────────────────────────────
//  1. ADDR_WIDTH removed as a parameter.  It is derived internally as
//     $clog2(DEPTH) and used ONLY to size the memory array and the
//     address slices of the pointers.  Users set only DATA_WIDTH and
//     DEPTH.
//
//  2. Pointers are now (ADDR_WIDTH+1) bits, but the wrap point is
//     DEPTH, not 2^ADDR_WIDTH.  An explicit modulo-DEPTH rollover
//     replaces the natural binary overflow that the original relied on.
//
//  3. full / empty are derived from a dedicated COUNT register rather
//     than the MSB-toggle trick.  The MSB trick requires
//     DEPTH == 2^ADDR_WIDTH; the count approach works for any depth.
//
//  4. Memory is sized to exactly DEPTH entries.
//
//  Original behaviour preserved:
//  ─────────────────────────────────────────────────────────────────────
//  - Registered (1-cycle latency) read output.
//  - Active-low async reset.
//  - wr_en / rd_en are gated by full / empty respectively.
//  - Same port list as the original.
//==========================================================================

module sync_fifo #(
    parameter integer DATA_WIDTH = 8,
    parameter integer DEPTH      = 12          // any positive integer
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

    // ── internal address width: enough bits to index 0 .. DEPTH-1 ────
    localparam integer ADDR_WIDTH = $clog2(DEPTH);

    // ── storage ──────────────────────────────────────────────────────
    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];

    // ── pointers (binary, range 0 .. DEPTH-1) ────────────────────────
    reg [ADDR_WIDTH-1:0] wr_ptr;
    reg [ADDR_WIDTH-1:0] rd_ptr;

    // ── occupancy counter ─────────────────────────────────────────────
    // Needs one extra bit to represent the value DEPTH without overflow.
    reg [ADDR_WIDTH:0] count;

    wire do_write = wr_en & ~full;
    wire do_read  = rd_en & ~empty;

    // ── write port ───────────────────────────────────────────────────
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            wr_ptr <= {ADDR_WIDTH{1'b0}};
        else if (do_write) begin
            mem[wr_ptr] <= din;
            // Explicit modulo rollover — works for any DEPTH
            wr_ptr <= (wr_ptr == DEPTH - 1) ? {ADDR_WIDTH{1'b0}} : wr_ptr + 1'b1;
        end
    end

    // ── read port (registered output) ───────────────────────────────
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_ptr <= {ADDR_WIDTH{1'b0}};
            dout   <= {DATA_WIDTH{1'b0}};
        end else if (do_read) begin
            dout   <= mem[rd_ptr];
            rd_ptr <= (rd_ptr == DEPTH - 1) ? {ADDR_WIDTH{1'b0}} : rd_ptr + 1'b1;
        end
    end

    // ── count register ───────────────────────────────────────────────
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            count <= {(ADDR_WIDTH+1){1'b0}};
        else
            case ({do_write, do_read})
                2'b10: count <= count + 1'b1;  // write only
                2'b01: count <= count - 1'b1;  // read only
                default: ;                      // both or neither: count unchanged
            endcase
    end

    // ── flags ────────────────────────────────────────────────────────
    assign full  = (count == DEPTH[ADDR_WIDTH:0]);
    assign empty = (count == {(ADDR_WIDTH+1){1'b0}});

endmodule
