//==========================================================================
//  async_fifo.v  -- Dual-clock (asynchronous) FIFO
//
//  Structure follows the classic Cliff Cummings reference design:
//    - fifomem    : dual-port memory (write on wclk, async read)
//    - wptr_full  : write pointer + FULL flag  (write clock domain)
//    - rptr_empty : read  pointer + EMPTY flag (read  clock domain)
//    - sync_r2w   : 2-FF synchronizer carrying the GRAY read ptr  -> wclk
//    - sync_w2r   : 2-FF synchronizer carrying the GRAY write ptr -> rclk
//
//  Why gray code: only ONE bit changes per increment, so when the
//  multi-bit pointer is sampled by the other clock, the worst case is
//  reading the value one step stale -- never a garbage in-between code.
//
//  Pointers are (ADDR_WIDTH+1) bits. Address into RAM = low ADDR_WIDTH bits.
//==========================================================================

// -------------------------------------------------------------------------
// Dual-port memory.  Combinational (async) read keeps the example simple.
// For real FPGA BRAM you'd register the read; see notes in the guide.
// -------------------------------------------------------------------------
module fifomem #(
    parameter integer DATA_WIDTH = 8,
    parameter integer ADDR_WIDTH = 4
)(
    input  wire                  wclk,
    input  wire                  wclken,        // write strobe (already gated by !wfull)
    input  wire [ADDR_WIDTH-1:0] waddr,
    input  wire [ADDR_WIDTH-1:0] raddr,
    input  wire [DATA_WIDTH-1:0] wdata,
    output wire [DATA_WIDTH-1:0] rdata
);
    localparam integer DEPTH = (1 << ADDR_WIDTH);
    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];

    assign rdata = mem[raddr];                   // async read
    always @(posedge wclk)
        if (wclken) mem[waddr] <= wdata;
endmodule


// -------------------------------------------------------------------------
// 2-flop synchronizer: read gray pointer -> write clock domain
// -------------------------------------------------------------------------
module sync_r2w #(
    parameter integer ADDR_WIDTH = 4
)(
    input  wire                wclk,
    input  wire                wrst_n,
    input  wire [ADDR_WIDTH:0] rptr,
    (* ASYNC_REG = "TRUE" *) output reg  [ADDR_WIDTH:0] wq2_rptr
);
    (* ASYNC_REG = "TRUE" *) reg [ADDR_WIDTH:0] wq1_rptr;
    always @(posedge wclk or negedge wrst_n)
        if (!wrst_n) {wq2_rptr, wq1_rptr} <= 0;
        else         {wq2_rptr, wq1_rptr} <= {wq1_rptr, rptr};
endmodule


// -------------------------------------------------------------------------
// 2-flop synchronizer: write gray pointer -> read clock domain
// -------------------------------------------------------------------------
module sync_w2r #(
    parameter integer ADDR_WIDTH = 4
)(
    input  wire                rclk,
    input  wire                rrst_n,
    input  wire [ADDR_WIDTH:0] wptr,
    (* ASYNC_REG = "TRUE" *) output reg  [ADDR_WIDTH:0] rq2_wptr
);
    (* ASYNC_REG = "TRUE" *) reg [ADDR_WIDTH:0] rq1_wptr;
    always @(posedge rclk or negedge rrst_n)
        if (!rrst_n) {rq2_wptr, rq1_wptr} <= 0;
        else         {rq2_wptr, rq1_wptr} <= {rq1_wptr, wptr};
endmodule


// -------------------------------------------------------------------------
// Read pointer + EMPTY generation (read clock domain)
// -------------------------------------------------------------------------
module rptr_empty #(
    parameter integer ADDR_WIDTH = 4
)(
    input  wire                  rclk,
    input  wire                  rrst_n,
    input  wire                  rinc,
    input  wire [ADDR_WIDTH:0]   rq2_wptr,       // synced write gray ptr
    output reg                   rempty,
    output wire [ADDR_WIDTH-1:0] raddr,
    output reg  [ADDR_WIDTH:0]   rptr             // gray
);
    reg  [ADDR_WIDTH:0] rbin;
    wire [ADDR_WIDTH:0] rbinnext, rgraynext;
    wire                rempty_val;

    // binary + gray pointer registers
    always @(posedge rclk or negedge rrst_n)
        if (!rrst_n) {rbin, rptr} <= 0;
        else         {rbin, rptr} <= {rbinnext, rgraynext};

    assign raddr     = rbin[ADDR_WIDTH-1:0];
    assign rbinnext  = rbin + (rinc & ~rempty);
    assign rgraynext = (rbinnext >> 1) ^ rbinnext;   // binary -> gray

    // EMPTY when the next read gray ptr catches the synced write gray ptr
    assign rempty_val = (rgraynext == rq2_wptr);
    always @(posedge rclk or negedge rrst_n)
        if (!rrst_n) rempty <= 1'b1;
        else         rempty <= rempty_val;
endmodule


// -------------------------------------------------------------------------
// Write pointer + FULL generation (write clock domain)
// -------------------------------------------------------------------------
module wptr_full #(
    parameter integer ADDR_WIDTH = 4
)(
    input  wire                  wclk,
    input  wire                  wrst_n,
    input  wire                  winc,
    input  wire [ADDR_WIDTH:0]   wq2_rptr,       // synced read gray ptr
    output reg                   wfull,
    output wire [ADDR_WIDTH-1:0] waddr,
    output reg  [ADDR_WIDTH:0]   wptr             // gray
);
    reg  [ADDR_WIDTH:0] wbin;
    wire [ADDR_WIDTH:0] wbinnext, wgraynext;
    wire                wfull_val;

    always @(posedge wclk or negedge wrst_n)
        if (!wrst_n) {wbin, wptr} <= 0;
        else         {wbin, wptr} <= {wbinnext, wgraynext};

    assign waddr     = wbin[ADDR_WIDTH-1:0];
    assign wbinnext  = wbin + (winc & ~wfull);
    assign wgraynext = (wbinnext >> 1) ^ wbinnext;   // binary -> gray

    // FULL when next write gray ptr == read gray ptr with the TWO MSBs flipped
    assign wfull_val = (wgraynext == {~wq2_rptr[ADDR_WIDTH:ADDR_WIDTH-1],
                                       wq2_rptr[ADDR_WIDTH-2:0]});
    always @(posedge wclk or negedge wrst_n)
        if (!wrst_n) wfull <= 1'b0;
        else         wfull <= wfull_val;
endmodule


// -------------------------------------------------------------------------
// Top level
// -------------------------------------------------------------------------
module async_fifo #(
    parameter integer DATA_WIDTH = 8,
    parameter integer ADDR_WIDTH = 4              // DEPTH = 2**ADDR_WIDTH
)(
    input  wire                  wclk,
    input  wire                  wrst_n,
    input  wire                  rclk,
    input  wire                  rrst_n,
    input  wire                  winc,            // write request
    input  wire                  rinc,            // read  request
    input  wire [DATA_WIDTH-1:0] wdata,
    output wire [DATA_WIDTH-1:0] rdata,
    output wire                  wfull,
    output wire                  rempty
);
    wire [ADDR_WIDTH-1:0] waddr, raddr;
    wire [ADDR_WIDTH:0]   wptr, rptr, wq2_rptr, rq2_wptr;

    sync_r2w   #(ADDR_WIDTH) u_s_r2w (
        .wclk(wclk), .wrst_n(wrst_n), .rptr(rptr), .wq2_rptr(wq2_rptr));

    sync_w2r   #(ADDR_WIDTH) u_s_w2r (
        .rclk(rclk), .rrst_n(rrst_n), .wptr(wptr), .rq2_wptr(rq2_wptr));

    fifomem    #(DATA_WIDTH, ADDR_WIDTH) u_mem (
        .wclk(wclk), .wclken(winc & ~wfull),
        .waddr(waddr), .raddr(raddr), .wdata(wdata), .rdata(rdata));

    rptr_empty #(ADDR_WIDTH) u_rptr (
        .rclk(rclk), .rrst_n(rrst_n), .rinc(rinc),
        .rq2_wptr(rq2_wptr), .rempty(rempty), .raddr(raddr), .rptr(rptr));

    wptr_full  #(ADDR_WIDTH) u_wptr (
        .wclk(wclk), .wrst_n(wrst_n), .winc(winc),
        .wq2_rptr(wq2_rptr), .wfull(wfull), .waddr(waddr), .wptr(wptr));

endmodule
