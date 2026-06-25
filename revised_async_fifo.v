//==========================================================================
// revised_async_fifo.v  -- Dual-clock (asynchronous) FIFO
//
//  
//  - CDC safety is guaranteed by free-running power-of-2 counters.
//  - Pointer math is handled natively in binary after reconversion.
//  - Memory addressing is handled by dedicated modulo [0, DEPTH-1] counters.
//==========================================================================

// -------------------------------------------------------------------------
// Dual-port memory. Sized to exactly DEPTH entries.
// -------------------------------------------------------------------------
module fifomem #(
    parameter integer DATA_WIDTH = 8,
    parameter integer DEPTH      = 12,
    parameter integer ADDR_WIDTH = $clog2(DEPTH)
)(
    input  wire                  wclk,
    input  wire                  wclken,
    input  wire [ADDR_WIDTH-1:0] waddr,
    input  wire [ADDR_WIDTH-1:0] raddr,
    input  wire [DATA_WIDTH-1:0] wdata,
    output wire [DATA_WIDTH-1:0] rdata
);
    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];
    
    assign rdata = mem[raddr];
    
    always @(posedge wclk)
        if (wclken) mem[waddr] <= wdata;
endmodule

// -------------------------------------------------------------------------
// Standard 2-flop synchronizers
// -------------------------------------------------------------------------
module sync_ff #(
    parameter integer WIDTH = 5
)(
    input  wire             clk,
    input  wire             rst_n,
    input  wire [WIDTH-1:0] d,
    (* ASYNC_REG = "TRUE" *) output reg [WIDTH-1:0] q
);
    (* ASYNC_REG = "TRUE" *) reg [WIDTH-1:0] q1;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) {q, q1} <= 0;
        else        {q, q1} <= {q1, d};
endmodule

// -------------------------------------------------------------------------
// Read pointer + EMPTY generation
// -------------------------------------------------------------------------
module rptr_empty #(
    parameter integer DEPTH      = 12,
    parameter integer ADDR_WIDTH = $clog2(DEPTH),
    parameter integer CDC_WIDTH  = $clog2(DEPTH) + 1
)(
    input  wire                 rclk,
    input  wire                 rrst_n,
    input  wire                 rinc,
    input  wire [CDC_WIDTH-1:0] rq2_wptr, // Synced Gray write pointer
    output reg                  rempty,
    output reg [ADDR_WIDTH-1:0] raddr,    // Modulo memory address
    output reg [CDC_WIDTH-1:0]  rptr      // Gray pointer out to write domain
);
    reg  [CDC_WIDTH-1:0] rbin_cdc;
    wire [CDC_WIDTH-1:0] rbin_cdc_next;
    wire [CDC_WIDTH-1:0] rgray_next;
    wire [ADDR_WIDTH-1:0] raddr_next;
    wire [CDC_WIDTH-1:0] wbin_cdc_synced;
    wire                 rempty_val;

    // Standard Gray to Binary Conversion (Synthesizes to XOR cascade)
    function [CDC_WIDTH-1:0] gray2bin;
        input [CDC_WIDTH-1:0] gray;
        integer i;
        begin
            gray2bin[CDC_WIDTH-1] = gray[CDC_WIDTH-1];
            for (i = CDC_WIDTH-2; i >= 0; i = i - 1) begin
                gray2bin[i] = gray2bin[i+1] ^ gray[i];
            end
        end
    endfunction

    // 1. Reconvert the synced Write Gray pointer back to plain Binary
    assign wbin_cdc_synced = gray2bin(rq2_wptr);

    // 2. Advance the free-running power-of-2 CDC binary counter
    assign rbin_cdc_next = rbin_cdc + (rinc & ~rempty);
    
    // 3. Convert next binary state to perfectly unit-distance Gray code
    assign rgray_next    = (rbin_cdc_next >> 1) ^ rbin_cdc_next;

    // 4. Advance the dedicated [0, DEPTH-1] RAM address
    assign raddr_next = (rinc & ~rempty) ? 
                        ((raddr == DEPTH - 1) ? 0 : raddr + 1) : 
                        raddr;

    // 5. Plain binary comparison for Empty (Reads caught up to Writes)
    assign rempty_val = (rbin_cdc_next == wbin_cdc_synced);

    always @(posedge rclk or negedge rrst_n) begin
        if (!rrst_n) begin
            rbin_cdc <= 0;
            rptr     <= 0;
            raddr    <= 0;
            rempty   <= 1'b1;
        end else begin
            rbin_cdc <= rbin_cdc_next;
            rptr     <= rgray_next;
            raddr    <= raddr_next;
            rempty   <= rempty_val;
        end
    end
endmodule

// -------------------------------------------------------------------------
// Write pointer + FULL generation
// -------------------------------------------------------------------------
module wptr_full #(
    parameter integer DEPTH      = 12,
    parameter integer ADDR_WIDTH = $clog2(DEPTH),
    parameter integer CDC_WIDTH  = $clog2(DEPTH) + 1
)(
    input  wire                 wclk,
    input  wire                 wrst_n,
    input  wire                 winc,
    input  wire [CDC_WIDTH-1:0] wq2_rptr, // Synced Gray read pointer
    output reg                  wfull,
    output reg [ADDR_WIDTH-1:0] waddr,    // Modulo memory address
    output reg [CDC_WIDTH-1:0]  wptr      // Gray pointer out to read domain
);
    reg  [CDC_WIDTH-1:0] wbin_cdc;
    wire [CDC_WIDTH-1:0] wbin_cdc_next;
    wire [CDC_WIDTH-1:0] wgray_next;
    wire [ADDR_WIDTH-1:0] waddr_next;
    wire [CDC_WIDTH-1:0] rbin_cdc_synced;
    wire [CDC_WIDTH-1:0] count_diff;
    wire                 wfull_val;

    // Standard Gray to Binary Conversion
    function [CDC_WIDTH-1:0] gray2bin;
        input [CDC_WIDTH-1:0] gray;
        integer i;
        begin
            gray2bin[CDC_WIDTH-1] = gray[CDC_WIDTH-1];
            for (i = CDC_WIDTH-2; i >= 0; i = i - 1) begin
                gray2bin[i] = gray2bin[i+1] ^ gray[i];
            end
        end
    endfunction

    // 1. Reconvert the synced Read Gray pointer back to plain Binary
    assign rbin_cdc_synced = gray2bin(wq2_rptr);

    // 2. Advance the free-running power-of-2 CDC binary counter
    assign wbin_cdc_next = wbin_cdc + (winc & ~wfull);
    assign wgray_next    = (wbin_cdc_next >> 1) ^ wbin_cdc_next;

    // 3. Advance the dedicated [0, DEPTH-1] RAM address
    assign waddr_next = (winc & ~wfull) ? 
                        ((waddr == DEPTH - 1) ? 0 : waddr + 1) : 
                        waddr;

    // 4. Plain binary arithmetic to determine in-flight items.
    // Two's complement modulo handles the power-of-2 wrap automatically.
    assign count_diff = wbin_cdc_next - rbin_cdc_synced;
    assign wfull_val  = (count_diff >= DEPTH);

    always @(posedge wclk or negedge wrst_n) begin
        if (!wrst_n) begin
            wbin_cdc <= 0;
            wptr     <= 0;
            waddr    <= 0;
            wfull    <= 1'b0;
        end else begin
            wbin_cdc <= wbin_cdc_next;
            wptr     <= wgray_next;
            waddr    <= waddr_next;
            wfull    <= wfull_val;
        end
    end
endmodule

// -------------------------------------------------------------------------
// Top level
// -------------------------------------------------------------------------
module async_fifo #(
    parameter integer DATA_WIDTH = 8,
    parameter integer DEPTH      = 12
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
    localparam integer CDC_WIDTH  = $clog2(DEPTH) + 1; 

    wire [ADDR_WIDTH-1:0] waddr, raddr;
    wire [CDC_WIDTH-1:0]  wptr, rptr, wq2_rptr, rq2_wptr;

    sync_ff #(.WIDTH(CDC_WIDTH)) u_s_r2w (
        .clk(wclk), .rst_n(wrst_n), .d(rptr), .q(wq2_rptr));
        
    sync_ff #(.WIDTH(CDC_WIDTH)) u_s_w2r (
        .clk(rclk), .rst_n(rrst_n), .d(wptr), .q(rq2_wptr));

    fifomem #(
        .DATA_WIDTH(DATA_WIDTH), 
        .DEPTH(DEPTH), 
        .ADDR_WIDTH(ADDR_WIDTH)
    ) u_mem (
        .wclk(wclk), .wclken(winc & ~wfull),
        .waddr(waddr), .raddr(raddr), .wdata(wdata), .rdata(rdata)
    );

    rptr_empty #(
        .DEPTH(DEPTH), 
        .ADDR_WIDTH(ADDR_WIDTH), 
        .CDC_WIDTH(CDC_WIDTH)
    ) u_rptr (
        .rclk(rclk), .rrst_n(rrst_n), .rinc(rinc),
        .rq2_wptr(rq2_wptr), .rempty(rempty), .raddr(raddr), .rptr(rptr)
    );

    wptr_full #(
        .DEPTH(DEPTH), 
        .ADDR_WIDTH(ADDR_WIDTH), 
        .CDC_WIDTH(CDC_WIDTH)
    ) u_wptr (
        .wclk(wclk), .wrst_n(wrst_n), .winc(winc),
        .wq2_rptr(wq2_rptr), .wfull(wfull), .waddr(waddr), .wptr(wptr)
    );

endmodule