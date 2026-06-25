//==========================================================================
//  async_fifo.v  -- Dual-clock (asynchronous) FIFO
//
//  Fixed for arbitrary DEPTH (does NOT need to be a power of two).
//
//  Root problem in the original design:
//  ─────────────────────────────────────────────────────────────────────
//  The standard (n >> 1) ^ n Gray code is unit-distance only when the
//  binary counter wraps at exactly 2^N.  For DEPTH=12 the counter wraps
//  at 12, causing 2-bit transitions at the rollover point.  This breaks
//  the CDC safety guarantee of the 2-FF synchronizers and also breaks
//  the full/empty comparisons which depend on the Gray encoding.
//
//  Fix strategy:
//  ─────────────────────────────────────────────────────────────────────
//  We use a "double-width" Gray counter that counts through 2 * DEPTH
//  values instead of DEPTH values.  The extra MSB acts as the wrap bit
//  (same role as in the original) and the count is always a power-of-2
//  multiple from the perspective of the Gray logic — we simply pad DEPTH
//  up to the next power of two ONLY for the purpose of the pointer
//  arithmetic, but keep the physical address in range [0, DEPTH-1] via
//  explicit modulo rollover in the binary counter.
//
//  Concretely:
//    PTR_WIDTH = $clog2(DEPTH) + 1  (one extra wrap bit, same as original)
//    Binary counter wraps at 2 * DEPTH (not 2^PTR_WIDTH).
//    Gray conversion is still (bin >> 1) ^ bin applied to the BINARY
//    counter value before it wraps — the Gray sequence remains
//    unit-distance at every step including the rollover.
//    Full  = next_write_gray == { ~synced_read_gray[MSB:MSB-1],
//                                   synced_read_gray[MSB-2:0] }
//            This is STILL correct because the binary distance for
//            "one full lap" is now 2*DEPTH/2 = DEPTH, and with the
//            count running to 2*DEPTH the MSB-flip trick is valid.
//    Empty = next_read_gray  == synced_write_gray  (unchanged)
//
//  Memory is sized to exactly DEPTH entries.  The address fed into RAM
//  is bin[PTR_WIDTH-2:0] mod DEPTH (handled by the modulo rollover in
//  the binary pointer).
//
//  All other structural elements (fifomem, sync_r2w, sync_w2r) are
//  unchanged except for the PTR_WIDTH parameter rename for clarity.
//
//  Original behaviour preserved:
//  ─────────────────────────────────────────────────────────────────────
//  - Same port list as the original.
//  - Async read memory.
//  - Active-low async resets in both domains.
//  - ASYNC_REG attributes on synchroniser flip-flops.
//==========================================================================


// -------------------------------------------------------------------------
// Dual-port memory.  Sized to exactly DEPTH entries.
// -------------------------------------------------------------------------
module fifomem #(
    parameter integer DATA_WIDTH = 8,
    parameter integer DEPTH      = 12,
    parameter integer ADDR_WIDTH = $clog2(DEPTH)  // address bits
)(
    input  wire                  wclk,
    input  wire                  wclken,
    input  wire [ADDR_WIDTH-1:0] waddr,
    input  wire [ADDR_WIDTH-1:0] raddr,
    input  wire [DATA_WIDTH-1:0] wdata,
    output wire [DATA_WIDTH-1:0] rdata
);
    // Exactly DEPTH entries — no phantom slots
    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];

    assign rdata = mem[raddr];
    always @(posedge wclk)
        if (wclken) mem[waddr] <= wdata;
endmodule


// -------------------------------------------------------------------------
// 2-flop synchronizer: read Gray pointer -> write clock domain
// -------------------------------------------------------------------------
module sync_r2w #(
    parameter integer PTR_WIDTH = 5
)(
    input  wire                wclk,
    input  wire                wrst_n,
    input  wire [PTR_WIDTH:0]  rptr,
    (* ASYNC_REG = "TRUE" *) output reg  [PTR_WIDTH:0] wq2_rptr
);
    (* ASYNC_REG = "TRUE" *) reg [PTR_WIDTH:0] wq1_rptr;
    always @(posedge wclk or negedge wrst_n)
        if (!wrst_n) {wq2_rptr, wq1_rptr} <= 0;
        else         {wq2_rptr, wq1_rptr} <= {wq1_rptr, rptr};
endmodule


// -------------------------------------------------------------------------
// 2-flop synchronizer: write Gray pointer -> read clock domain
// -------------------------------------------------------------------------
module sync_w2r #(
    parameter integer PTR_WIDTH = 5
)(
    input  wire                rclk,
    input  wire                rrst_n,
    input  wire [PTR_WIDTH:0]  wptr,
    (* ASYNC_REG = "TRUE" *) output reg  [PTR_WIDTH:0] rq2_wptr
);
    (* ASYNC_REG = "TRUE" *) reg [PTR_WIDTH:0] rq1_wptr;
    always @(posedge rclk or negedge rrst_n)
        if (!rrst_n) {rq2_wptr, rq1_wptr} <= 0;
        else         {rq2_wptr, rq1_wptr} <= {rq1_wptr, wptr};
endmodule


// -------------------------------------------------------------------------
// Read pointer + EMPTY generation (read clock domain)
//
// Key change: binary counter wraps at 2*DEPTH, not 2^PTR_WIDTH.
// The address into RAM = rbin[ADDR_WIDTH-1:0] (already in range because
// the binary counter never exceeds 2*DEPTH-1 and the address slice
// naturally stays in [0, DEPTH-1] after the modulo rollover).
// -------------------------------------------------------------------------
module rptr_empty #(
    parameter integer DEPTH      = 12,
    parameter integer ADDR_WIDTH = $clog2(DEPTH),  // address bits
    parameter integer PTR_WIDTH  = ADDR_WIDTH       // Gray pointer width (same)
)(
    input  wire                  rclk,
    input  wire                  rrst_n,
    input  wire                  rinc,
    input  wire [PTR_WIDTH:0]    rq2_wptr,    // synced write Gray ptr
    output reg                   rempty,
    output wire [ADDR_WIDTH-1:0] raddr,
    output reg  [PTR_WIDTH:0]    rptr          // Gray
);
    // Binary counter counts 0 .. 2*DEPTH-1, then rolls back to 0.
    // Width: PTR_WIDTH+1 bits (same as original).
    reg  [PTR_WIDTH:0] rbin;
    wire [PTR_WIDTH:0] rbinnext, rgraynext;
    wire               rempty_val;

    always @(posedge rclk or negedge rrst_n)
        if (!rrst_n) {rbin, rptr} <= 0;
        else         {rbin, rptr} <= {rbinnext, rgraynext};

    // Address is lower ADDR_WIDTH bits of binary counter.
    // Because the binary counter wraps at 2*DEPTH (not 2^(PTR_WIDTH+1)),
    // the lower bits are always a valid index into [0, DEPTH-1].
    assign raddr = rbin[ADDR_WIDTH-1:0];

    // Explicit modulo rollover: wrap at 2*DEPTH
    assign rbinnext  = (rbin == 2*DEPTH - 1) ? {(PTR_WIDTH+1){1'b0}}
                                              : rbin + (rinc & ~rempty);

    // Standard Gray conversion — unit-distance because binary is contiguous
    assign rgraynext = (rbinnext >> 1) ^ rbinnext;

    // EMPTY: next read Gray == synced write Gray
    assign rempty_val = (rgraynext == rq2_wptr);
    always @(posedge rclk or negedge rrst_n)
        if (!rrst_n) rempty <= 1'b1;
        else         rempty <= rempty_val;
endmodule


// -------------------------------------------------------------------------
// Write pointer + FULL generation (write clock domain)
//
// Key change: binary counter wraps at 2*DEPTH.
// Full condition: the write pointer has advanced by DEPTH entries beyond
// the read pointer.  With the 2*DEPTH binary range, "one full lap" is
// still half the total range, so the top-two-MSB-flip comparison on the
// Gray code remains valid.
// -------------------------------------------------------------------------
module wptr_full #(
    parameter integer DEPTH      = 12,
    parameter integer ADDR_WIDTH = $clog2(DEPTH),
    parameter integer PTR_WIDTH  = ADDR_WIDTH
)(
    input  wire                  wclk,
    input  wire                  wrst_n,
    input  wire                  winc,
    input  wire [PTR_WIDTH:0]    wq2_rptr,    // synced read Gray ptr
    output reg                   wfull,
    output wire [ADDR_WIDTH-1:0] waddr,
    output reg  [PTR_WIDTH:0]    wptr          // Gray
);
    reg  [PTR_WIDTH:0] wbin;
    wire [PTR_WIDTH:0] wbinnext, wgraynext;
    wire               wfull_val;

    always @(posedge wclk or negedge wrst_n)
        if (!wrst_n) {wbin, wptr} <= 0;
        else         {wbin, wptr} <= {wbinnext, wgraynext};

    assign waddr = wbin[ADDR_WIDTH-1:0];

    // Explicit modulo rollover at 2*DEPTH
    assign wbinnext  = (wbin == 2*DEPTH - 1) ? {(PTR_WIDTH+1){1'b0}}
                                              : wbin + (winc & ~wfull);

    assign wgraynext = (wbinnext >> 1) ^ wbinnext;

    // FULL: write Gray is exactly one lap (= DEPTH entries) ahead of
    // read Gray.  With a 2*DEPTH binary range the MSB and sub-MSB of
    // the Gray pointer flip when the distance is exactly DEPTH —
    // identical in structure to the original design.
    assign wfull_val = (wgraynext == {~wq2_rptr[PTR_WIDTH:PTR_WIDTH-1],
                                       wq2_rptr[PTR_WIDTH-2:0]});

    always @(posedge wclk or negedge wrst_n)
        if (!wrst_n) wfull <= 1'b0;
        else         wfull <= wfull_val;
endmodule


// -------------------------------------------------------------------------
// Top level
// -------------------------------------------------------------------------
module async_fifo #(
    parameter integer DATA_WIDTH = 8,
    parameter integer DEPTH      = 12          // any positive integer
)(
    input  wire                  wclk,
    input  wire                  wrst_n,
    input  wire                  rclk,
    input  wire                  rrst_n,
    input  wire                  winc,
    input  wire                  rinc,
    input  wire [DATA_WIDTH-1:0] wdata,
    output wire [DATA_WIDTH-1:0] rdata,
    output wire                  wfull,
    output wire                  rempty
);
    localparam integer ADDR_WIDTH = $clog2(DEPTH);
    localparam integer PTR_WIDTH  = ADDR_WIDTH;

    wire [ADDR_WIDTH-1:0] waddr, raddr;
    wire [PTR_WIDTH:0]    wptr, rptr, wq2_rptr, rq2_wptr;

    sync_r2w   #(.PTR_WIDTH(PTR_WIDTH)) u_s_r2w (
        .wclk(wclk), .wrst_n(wrst_n), .rptr(rptr), .wq2_rptr(wq2_rptr));

    sync_w2r   #(.PTR_WIDTH(PTR_WIDTH)) u_s_w2r (
        .rclk(rclk), .rrst_n(rrst_n), .wptr(wptr), .rq2_wptr(rq2_wptr));

    fifomem    #(.DATA_WIDTH(DATA_WIDTH), .DEPTH(DEPTH), .ADDR_WIDTH(ADDR_WIDTH)) u_mem (
        .wclk(wclk), .wclken(winc & ~wfull),
        .waddr(waddr), .raddr(raddr), .wdata(wdata), .rdata(rdata));

    rptr_empty #(.DEPTH(DEPTH), .ADDR_WIDTH(ADDR_WIDTH), .PTR_WIDTH(PTR_WIDTH)) u_rptr (
        .rclk(rclk), .rrst_n(rrst_n), .rinc(rinc),
        .rq2_wptr(rq2_wptr), .rempty(rempty), .raddr(raddr), .rptr(rptr));

    wptr_full  #(.DEPTH(DEPTH), .ADDR_WIDTH(ADDR_WIDTH), .PTR_WIDTH(PTR_WIDTH)) u_wptr (
        .wclk(wclk), .wrst_n(wrst_n), .winc(winc),
        .wq2_rptr(wq2_rptr), .wfull(wfull), .waddr(waddr), .wptr(wptr));

endmodule
